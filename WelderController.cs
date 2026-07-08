using System;
using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.Game.ModAPI;
using VRage.Utils;
using VRageMath;
using MyShipConnectorStatus = Sandbox.ModAPI.Ingame.MyShipConnectorStatus;
using FlightMode = Sandbox.ModAPI.Ingame.FlightMode;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;
using IMySlimBlock = VRage.Game.ModAPI.IMySlimBlock;

namespace ColonyFramework
{
    // Per-mission brain for a WELDER drone building a projected blueprint. Mirrors the miner's
    // lifecycle and reuses its proven primitives (NavState awareness, AvoidanceProbe transit sensing,
    // BoreController Face/ThrustAlong, DroneUtil block/cargo helpers, the same watchdog->retry->fail
    // discipline). Flight around the construction is RADIAL: hold a standoff outside the target's
    // bounding volume, approach a block straight in from outside, weld nose-on, back straight out —
    // no interior flight, no stored paths (the staged build order added in Drop 2 keeps interiors
    // externally reachable while the hull is open).
    public class WelderController
    {
        // Mission.Phase values (welder mapping — observability + resume routing)
        private const int PhaseCommission = 0;
        private const int PhaseDockLoad   = 1; // docked/docking at base: load components + recharge
        private const int PhaseTransit    = 2; // cruise to the build-site standoff
        private const int PhaseWork       = 3; // select block / approach / weld / back out
        private const int PhaseReturn     = 4; // cruise home (resupply or done)

        // Work sub-states
        private const int WorkSelect  = 0;
        private const int WorkApproach = 1;
        private const int WorkWeldIn  = 2;
        private const int WorkBackOut = 3;

        // Flight constants (values proven by the miner)
        private const float  CruiseSpeedLimit  = 70f;
        private const double CruiseAltitudeAgl = 100.0;
        private const double ArriveDistance    = 12.0;
        private const double SiteStandoffUp    = 40.0;  // m above the projected volume's top for the site hold
        private const double ApproachStandoff  = 15.0;  // m outside the block along the radial for the approach point
        private const double ApproachTol       = 3.0;
        private const double ApproachSpeed     = 6.0;
        private const double OrbitLegSpeed     = 12.0;  // arc legs around the construction (fluid sweep)
        private const double WelderMinAgl      = 12.0;  // orbit/standoff points never dip below surface + this
        private const double ApproachMinAgl    = 4.0;   // approach points may be lower (ground-level blocks) but never in the ground
        private const double SiteRerouteSecs   = 3.0;   // orbit step / avoidance re-issue cadence at the site
        private const double ApproachTimeoutSecs = 180.0; // whole approach (incl. orbit) budget before deferring the block
        private const double WeldInSpeed       = 0.8;   // m/s nose-in creep (dampers on)
        private const double WeldReach         = 2.0;   // m — the welder TOOL's work-zone reach (stop when tool is this close)
        private const double SettleSpeed       = 0.5;
        private const double ContactSpeedEps   = 0.1;   // inward speed below this while pressing = contact
        private const double WeldBlockTimeoutSecs = 45.0; // per-block ceiling before deferring it
        private const double WeldStallSecs     = 180.0; // no RemainingBlocks progress at the site → give up gracefully
        private const double SelectThrottleSecs = 1.0;
        private const double LegTimeoutSecs    = 150.0;
        private const double LegStuckSecs      = 20.0;
        private const double LegProgressEps    = 1.0;
        private const int    MaxRetries        = 3;
        private const double LowPowerThreshold = 0.20;
        private const double ChargeTargetPct   = 0.50;
        private const double ChargeStallSecs   = 60.0;
        private const double CargoLoadTarget   = 0.60;  // stop loading components at this cargo fill

        private const double DockMoveSpeed = 6.0; // slow hop used by the transit-recover leg

        private readonly FlightController _fc = new FlightController(); // F4.1: THE actuator owner for all flight
        private readonly NavState _nav = new NavState();
        private readonly OrbitNav _orbit = new OrbitNav(); // around-the-hull geometry (feeds goals to _fc)
        private readonly DockMachine _dock = new DockMachine(); // shared proven connector-dock sequence
        private bool _issuedDirect; // approach: a direct leg to the approach point is in flight
        private bool _dockIssued;   // dock-load: the Dock verb is running

        private bool _initialized;
        private int _retries;
        private int _workSub;
        private bool _loading; // DockLoad phase: false = flying the dock, true = connected and loading
        private bool _returningToResupply; // Return phase ends in DockLoad (resupply) vs mission complete
        private DateTime _legStart, _legProgressTime;
        private double _legMinDist = double.MaxValue;
        private DateTime _lastLog, _lastSelect, _weldStart, _chargeProgressTime, _lastSiteProgress;
        private double _chargeRefPct;
        private int _lastRemaining = -1;
        private Vector3I _targetCell; private Vector3D _targetWorld, _approachPoint;
        private bool _hasTarget;
        private DateTime _lastSiteReroute; // orbit/avoidance re-issue throttle at the site
        private DateTime _approachStart;   // whole-approach budget (orbit legs reset autopilot, so LegOk can't time this)
        private readonly HashSet<Vector3I> _deferred = new HashSet<Vector3I>();
        private bool _deferredRetried; // one full retry pass over deferred blocks before declaring stuck
        private readonly BlueprintStager _stager = new BlueprintStager(); // shipyard build order (frame -> internals -> closure)
        private int _announcedStage = -1;

        private ConnectorReservations _cons; // fleet connector traffic control (set each tick)
        private WeldCoordinator _coord;      // multi-welder block claims + spacing bubbles (set each tick)
        private DateTime _dockWaitStart, _lastDockWaitRetry; // hold pattern when all connectors are busy
        private bool _dockWaiting;

        // ── Entry point (called at ~6 Hz by DroneExecutor) ──────────────────────────────────────────
        private long _missionId;

        public void Advance(Colony colony, Mission m, IMyCubeGrid grid, ConnectorReservations cons, WeldCoordinator coord)
        {
            _cons = cons;
            _coord = coord;
            _missionId = m.Id;
            if (coord != null) coord.UpdatePresence(m.Id, grid.GetPosition());
            var rc = DroneUtil.FindRc(grid);
            if (rc == null) { Fail(colony, m, grid, "no remote control"); return; }
            if (rc.IsUnderControl) return; // player has the wheel — never fight the pilot
            _nav.Refresh(grid, rc, DroneUtil.FindConnector(grid));

            if (!_initialized)
            {
                _initialized = true;
                if (m.Phase != PhaseCommission)
                {
                    // Fresh controller after a reload mid-mission: safest re-acquire is to go home,
                    // resupply, and resume from the dock (mirrors the miner's OnResume philosophy).
                    Log(m, "resumed after reload — returning to base to re-acquire");
                    BeginReturn(colony, m, grid, true);
                    return;
                }
            }

            _projectorId = m.TargetEntityId;
            var projector = MyAPIGateway.Entities.GetEntityById(m.TargetEntityId) as IMyProjector;
            if (projector == null || projector.Closed)
            { Complete(colony, m, grid, "projector gone"); return; }

            switch (m.Phase)
            {
                case PhaseCommission: TickCommission(colony, m, grid); break;
                case PhaseDockLoad:   TickDockLoad(colony, m, grid, projector); break;
                case PhaseTransit:    TickTransit(colony, m, grid, projector); break;
                case PhaseWork:       TickWork(colony, m, grid, projector); break;
                case PhaseReturn:     TickReturn(colony, m, grid); break;
            }

            // ONE actuator owner (FLIGHT.md §5.1): pump the flight core every tick. During DockLoad
            // the DockMachine owns actuators and _fc is Released (Idle) — its Tick is a no-op.
            _fc.Tick(grid);
        }

        // ── Commission: same shape as the miner (welders don't need the full draw math — verify RC,
        // welders and a connector exist, then head to load) ──────────────────────────────────────────
        private void TickCommission(Colony colony, Mission m, IMyCubeGrid grid)
        {
            if (DroneUtil.FindWelders(grid).Count == 0) { Fail(colony, m, grid, "no welders on drone"); return; }
            if (DroneUtil.FindConnector(grid) == null) { Fail(colony, m, grid, "no connector on drone"); return; }
            // FLIGHT.md §4 static capability gates (named deficiency). The measured hop runs at the
            // welder's F4 convert — commissioning here may start DOCKED, where a hop is impossible.
            var model = ShipSelfModel.Build(grid);
            string deficiency = model == null ? "no remote control / physics" : model.Deficiency();
            if (deficiency != null)
            {
                if (!MyAPIGateway.Utilities.IsDedicated)
                    MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                        "'{0}' self-test FAILED: {1}", grid.DisplayName, deficiency));
                Fail(colony, m, grid, "self-test FAIL: " + deficiency);
                return;
            }
            var asset = colony.Assets.GetByEntityId(m.AssignedAssetId);
            if (asset != null) asset.CapabilitySummary = model.Summary();
            Log(m, string.Format("commissioned (welder) — {0}; heading to load components", model.Summary()));
            BeginDockLoad(colony, m, grid);
        }

        // ── DockLoad: get connected to the base, pull components, recharge, then fly to the site ────
        private void BeginDockLoad(Colony colony, Mission m, IMyCubeGrid grid)
        {
            _fc.Release(grid);        // hand the actuators to the DockMachine — one owner at a time
            _fc.ClearWorkVolumes();
            m.Phase = PhaseDockLoad;
            var droneCon = DroneUtil.FindConnector(grid);
            _loading = droneCon != null && droneCon.Status == MyShipConnectorStatus.Connected;
            if (!_loading)
            {
                DroneUtil.PrepareForFlight(grid); // batteries auto + thrusters on + unlock — autopilot won't do any of it
                _dock.Reset();                    // shared dock machine flies the approach itself
                _dockIssued = false;              // TickDockLoad issues the Dock verb fresh
            }
            else
            {
                BeginCharging(grid); // job-scaled; skips Recharge entirely if already above the requirement
            }
            ResetLeg();
            _legStart = DateTime.UtcNow;
        }

        private void TickDockLoad(Colony colony, Mission m, IMyCubeGrid grid, IMyProjector projector)
        {
            var droneCon = DroneUtil.FindConnector(grid);
            var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as IMyCubeBlock;
            if (droneCon == null || core == null) { Fail(colony, m, grid, "connector/core lost"); return; }

            if (!_loading)
            {
                // F5: docking runs as a flight-core VERB (the proven DockMachine still drives).
                if (_dockIssued && _fc.Status == FlightController.VerbStatus.Done)
                { _dockIssued = false; _dockWaiting = false; EnterLoading(grid); return; }
                if (_dockIssued && _fc.Status == FlightController.VerbStatus.Failed)
                {
                    _dockIssued = false;
                    if (_fc.FailReason != "no free base connector")
                    { RetryOrFail(colony, m, grid, _fc.FailReason); return; }
                    // Legitimate queueing, not a failure: hold on dampers and re-ask periodically.
                    if (!_dockWaiting) { _dockWaiting = true; _dockWaitStart = DateTime.UtcNow; Log(m, "all connectors busy — holding for a free one"); }
                }
                if (_dockWaiting)
                {
                    var rcw = DroneUtil.FindRc(grid);
                    if (rcw != null) { rcw.SetAutoPilotEnabled(false); if (!rcw.DampenersOverride) rcw.DampenersOverride = true; }
                    if ((DateTime.UtcNow - _dockWaitStart).TotalSeconds > 600)
                    { _dockWaiting = false; RetryOrFail(colony, m, grid, "no connector freed up in 10 min"); return; }
                    if ((DateTime.UtcNow - _lastDockWaitRetry).TotalSeconds < 10) return; // still cooling down
                    _lastDockWaitRetry = DateTime.UtcNow;
                }
                if (!_dockIssued)
                {
                    _dock.Reset();
                    _fc.Dock(grid, _dock, core, _cons);
                    _dockIssued = true;
                }
                return;
            }

            // Connected: pull components for the projector's remaining blueprint, recharge, undock.
            var want = new ProjectorReader().RequiredComponents(core.CubeGrid, out _tmpProj, out _tmpBlocks);
            DroneUtil.LoadComponents(grid, droneCon, want);

            double charge = DroneUtil.MinBatteryCharge(grid);
            if (charge > _chargeRefPct + 0.01) { _chargeRefPct = charge; _chargeProgressTime = DateTime.UtcNow; }
            bool charged = charge >= _requiredCharge || DroneUtil.HasInfinitePower(grid);
            bool stalled = (DateTime.UtcNow - _chargeProgressTime).TotalSeconds > ChargeStallSecs;
            bool haveSomething = DroneUtil.CargoFill(grid) > 0.02;
            if ((charged || stalled) && haveSomething)
            {
                DroneUtil.PrepareForFlight(grid);
                Log(m, string.Format("loaded + charged ({0:N0}%), flying to build site", charge * 100));
                BeginTransit(m, grid, projector);
            }
            else if ((DateTime.UtcNow - _legStart).TotalSeconds > 120 && !haveSomething)
            {
                // Base has no components at all for this blueprint — stand down and let production catch up.
                DroneUtil.SetBatteriesRecharge(grid, false);
                Complete(colony, m, grid, "no components available yet — waiting on production");
            }
        }

        private int _tmpProj, _tmpBlocks;
        private double _requiredCharge = ChargeTargetPct;
        private long _projectorId; // mission target, stashed each tick for helpers that lack the Mission

        private void EnterLoading(IMyCubeGrid grid)
        {
            _loading = true;
            _fc.Release(grid);
            BeginCharging(grid);
            _legStart = DateTime.UtcNow;
        }

        // Charge scaled to the JOB, not a flat target: a 5-block test print needs 30%, a big print
        // still caps at ChargeTargetPct (50%) — never full; the parker's recharge-to-FULL applies
        // only to IDLE-docked drones. Already above the requirement → skip Recharge mode entirely
        // (batteries stay Auto): grab components and go.
        private void BeginCharging(IMyCubeGrid grid)
        {
            int remaining = 100;
            var proj = MyAPIGateway.Entities.GetEntityById(_projectorId) as IMyProjector;
            if (proj != null && proj.IsProjecting) remaining = Math.Max(proj.RemainingBlocks, 1);
            _requiredCharge = Math.Max(0.30, Math.Min(ChargeTargetPct, 0.25 + 0.005 * remaining));
            double cur = DroneUtil.MinBatteryCharge(grid);
            if (cur < _requiredCharge) DroneUtil.SetBatteriesRecharge(grid, true);
            _chargeRefPct = cur;
            _chargeProgressTime = DateTime.UtcNow;
        }

        // ── Transit to the site standoff (autopilot + avoidance) ─────────────────────────────────────
        private void BeginTransit(Mission m, IMyCubeGrid grid, IMyProjector projector)
        {
            m.Phase = PhaseTransit;
            // F4.1: planned corridor on the flight core. The construction is declared a WORK VOLUME
            // so steering never treats it (or its separate under-construction grid) as an obstacle.
            _fc.ClearWorkVolumes();
            _fc.DeclareWorkVolume(KeepOut(projector, grid));
            _fc.Transit(grid, FlightCorridor.Plan(grid.GetPosition(), SiteStandoff(projector), _fc.Profile.CruiseAgl),
                        CruiseSpeedLimit);
        }

        private void TickTransit(Colony colony, Mission m, IMyCubeGrid grid, IMyProjector projector)
        {
            if (_fc.Status == FlightController.VerbStatus.Failed)
            { RetryOrFail(colony, m, grid, "site transit: " + _fc.FailReason); return; }
            if (_fc.Status != FlightController.VerbStatus.Done) return; // corridor in progress
            _retries = 0;
            m.Phase = PhaseWork;
            _workSub = WorkSelect;
            _deferred.Clear();
            _deferredRetried = false;
            _lastRemaining = projector.RemainingBlocks;
            _lastSiteProgress = DateTime.UtcNow;
            Log(m, string.Format("at build site, {0} blocks remaining", projector.RemainingBlocks));
        }

        // ── Work: select -> approach -> weld-in -> back-out, repeat ──────────────────────────────────
        private void TickWork(Colony colony, Mission m, IMyCubeGrid grid, IMyProjector projector)
        {
            // MISSION.md D4: distance-aware energy ledger (live draw — welders + thrusters running now).
            Vector3D coreStandoffForLedger;
            if (TryCoreStandoff(colony, out coreStandoffForLedger))
            {
                string ledger = MissionLedger.ShouldReturn(grid, coreStandoffForLedger, 0);
                if (ledger != null)
                {
                    if (!MyAPIGateway.Utilities.IsDedicated)
                        MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                            "'{0}' returning to recharge — {1}", grid.DisplayName, ledger));
                    BeginReturn(colony, m, grid, true);
                    return;
                }
            }

            // Site-level progress watchdog: blocks are getting placed, right?
            if (projector.RemainingBlocks < _lastRemaining)
            { _lastRemaining = projector.RemainingBlocks; _lastSiteProgress = DateTime.UtcNow; }
            if ((DateTime.UtcNow - _lastSiteProgress).TotalSeconds > WeldStallSecs)
            {
                if (!MyAPIGateway.Utilities.IsDedicated)
                    MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                        "Construction stalled ({0} blocks remain, {1} deferred) — welder standing down", projector.RemainingBlocks, _deferred.Count));
                Complete(colony, m, grid, "construction stalled");
                return;
            }

            if (projector.RemainingBlocks <= 0)
            {
                if (!MyAPIGateway.Utilities.IsDedicated)
                    MyAPIGateway.Utilities.ShowMessage("Colony", "Ship construction complete");
                Complete(colony, m, grid, "blueprint fully welded");
                return;
            }

            switch (_workSub)
            {
                case WorkSelect:   TickSelect(colony, m, grid, projector); break;
                case WorkApproach: TickApproach(colony, m, grid, projector); break;
                case WorkWeldIn:   TickWeldIn(colony, m, grid, projector); break;
                case WorkBackOut:  TickBackOut(colony, m, grid); break;
            }
        }

        private void TickSelect(Colony colony, Mission m, IMyCubeGrid grid, IMyProjector projector)
        {
            // The flight core is already holding (Done verbs hold where they finished).
            if ((DateTime.UtcNow - _lastSelect).TotalSeconds < SelectThrottleSecs) return;
            _lastSelect = DateTime.UtcNow;

            var projected = projector.ProjectedGrid;
            if (projected == null) { Complete(colony, m, grid, "projection stopped"); return; }

            var blocks = new List<IMySlimBlock>();
            projected.GetBlocks(blocks);
            _stager.EnsureBuilt(blocks); // one-time classification: frame / internals / closure

            // Shipyard order: lowest STAGE first (frame -> internals -> closure), bottom-up within a
            // stage (build from the keel), nearest as the tiebreak. CanBuild still gates placement, so
            // stage members whose neighbours don't exist yet are skipped and picked up next pass.
            IMySlimBlock best = null;
            int bestStage = int.MaxValue;
            double bestAlt = double.MaxValue, bestSq = double.MaxValue;
            Vector3D pos = grid.GetPosition();
            Vector3D up = _nav.Valid ? _nav.GravityUp : Vector3D.Up;
            for (int i = 0; i < blocks.Count; i++)
            {
                var b = blocks[i];
                if (_deferred.Contains(b.Position)) continue;
                if (projector.CanBuild(b, true) != BuildCheckResult.OK) continue;
                Vector3D w = projected.GridIntegerToWorld(b.Position);
                // Multi-welder bubble: stay off other welders' claimed blocks and away from the
                // drones themselves — the fleet spreads across different patches of the hull.
                if (_coord != null && _coord.NearOthers(m.Id, w, WeldCoordinator.WelderBubble)) continue;
                int stage = _stager.StageOf(b);
                double alt = Vector3D.Dot(w, up);
                double sq = Vector3D.DistanceSquared(w, pos);
                if (stage < bestStage
                    || (stage == bestStage && alt < bestAlt - 1.0)
                    || (stage == bestStage && System.Math.Abs(alt - bestAlt) <= 1.0 && sq < bestSq))
                { bestStage = stage; bestAlt = alt; bestSq = sq; best = b; }
            }

            if (best != null && bestStage != _announcedStage)
            {
                _announcedStage = bestStage;
                if (!MyAPIGateway.Utilities.IsDedicated)
                    MyAPIGateway.Utilities.ShowMessage("Colony", "Construction stage: " + BlueprintStager.StageName(bestStage));
                Log(m, "construction stage: " + BlueprintStager.StageName(bestStage));
            }

            if (best == null)
            {
                if (_deferred.Count > 0 && !_deferredRetried)
                {
                    _deferredRetried = true; // structure grew since they were deferred — one retry pass
                    _deferred.Clear();
                    return;
                }
                // Nothing buildable: either components are missing (go resupply) or geometry blocks us.
                if (DroneUtil.CargoFill(grid) < 0.02)
                { Log(m, "out of components — returning to resupply"); BeginReturn(colony, m, grid, true); }
                else
                {
                    if (!MyAPIGateway.Utilities.IsDedicated)
                        MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                            "Welder: nothing buildable now ({0} blocks remain — may need the first block placed or more parts)", projector.RemainingBlocks));
                    Complete(colony, m, grid, "nothing buildable");
                }
                return;
            }

            _targetCell = best.Position;
            _targetWorld = projected.GridIntegerToWorld(best.Position);
            if (_coord != null && !_coord.TryClaim(m.Id, projector.EntityId, best.Position, _targetWorld))
                return; // another welder claimed it between checks — re-select next pass
            // Radial approach: from the construction's volume center THROUGH the block, out to standoff.
            // TRUE radial, any angle — 45° down or up is fine (user-directed): the nose-in creep uses
            // direct gyro control (Face), which CAN pitch, unlike autopilot. The terrain clamp below is
            // the only floor — an approach point never ends up underground.
            Vector3D up0 = _nav.Valid ? _nav.GravityUp : Vector3D.Up;
            Vector3D center = projected.WorldAABB.Center;
            Vector3D radial = _targetWorld - center;
            if (radial.LengthSquared() < 1.0) radial = up0;  // dead-centre: come from above
            radial = Vector3D.Normalize(radial);
            double selfRadius = grid.WorldAABB.HalfExtents.Length();
            _approachPoint = _orbit.ClampAboveTerrain(_targetWorld + radial * (selfRadius + ApproachStandoff), up0, ApproachMinAgl);
            _hasTarget = true;
            _workSub = WorkApproach;
            _approachStart = DateTime.UtcNow;
            _lastSiteReroute = default(DateTime); // first TickApproach issues the first orbit/direct leg immediately
            _issuedDirect = false;
            _fc.ClearWorkVolumes();
            _fc.DeclareWorkVolume(KeepOut(projector, grid)); // the construction is the work, not an obstacle
            Log(m, string.Format("weld: target block at ({0:F0}, {1:F0}, {2:F0}), {3} remaining",
                _targetWorld.X, _targetWorld.Y, _targetWorld.Z, projector.RemainingBlocks));
        }

        private void TickApproach(Colony colony, Mission m, IMyCubeGrid grid, IMyProjector projector)
        {
            if (!_hasTarget) { _workSub = WorkSelect; return; }
            Vector3D pos = grid.GetPosition();
            Vector3D up = _nav.Valid ? _nav.GravityUp : Vector3D.Up;

            // Whole-approach budget: too long overall → this block is awkward right now, defer it.
            if ((DateTime.UtcNow - _approachStart).TotalSeconds > ApproachTimeoutSecs)
            { DeferTarget("approach timeout"); return; }
            if (_fc.Status == FlightController.VerbStatus.Failed)
            { DeferTarget("approach: " + _fc.FailReason); return; }

            if (_issuedDirect)
            {
                if (_fc.Status != FlightController.VerbStatus.Done) return; // direct leg still flying

                // At the approach point. Reach test: one ray from here to the block — if the
                // construction itself is in the way, the block isn't externally reachable; defer.
                IHitInfo hit;
                if (MyAPIGateway.Physics.CastRay(_approachPoint, _targetWorld, out hit) && hit != null)
                {
                    var hitGrid = hit.HitEntity as IMyCubeGrid;
                    if (hitGrid != null && hitGrid.EntityId != grid.EntityId
                        && Vector3D.DistanceSquared(hit.Position, _targetWorld) > 9.0)
                    { DeferTarget("blocked by structure"); return; }
                }
                var welders = DroneUtil.FindWelders(grid);
                if (welders.Count == 0) { Fail(colony, m, grid, "welders lost"); return; }
                var projected = projector.ProjectedGrid;
                double blockHalf = projected != null && projected.GridSizeEnum == VRage.Game.MyCubeSize.Large ? 1.25 : 0.25;
                DroneUtil.SetWelders(grid, true);
                Log(m, "weld: tool on, creeping in"); // enable is otherwise invisible in-game
                _fc.OrientAndCreep(grid, _targetWorld, welders[0] as VRage.Game.ModAPI.IMyCubeBlock,
                                   WeldReach + blockHalf, WeldInSpeed);
                _weldStart = DateTime.UtcNow;
                _workSub = WorkWeldIn;
                return;
            }

            // Orbit-or-direct as goals into the CONTINUOUS velocity loop (re-issuing a goal blends
            // smoothly — no autopilot churn): slide along the keep-out sphere until line-of-sight
            // opens, then commit one direct leg and let its arrival latch fire.
            if ((DateTime.UtcNow - _lastSiteReroute).TotalSeconds >= SiteRerouteSecs)
            {
                _lastSiteReroute = DateTime.UtcNow;
                Vector3D wp;
                bool direct = _orbit.NextStep(pos, _approachPoint, KeepOut(projector, grid), up, WelderMinAgl, out wp);
                if (direct) { _fc.MoveTo(grid, _approachPoint, ApproachSpeed * 2); _issuedDirect = true; }
                else _fc.MoveTo(grid, wp, OrbitLegSpeed);
            }
        }

        private void TickWeldIn(Colony colony, Mission m, IMyCubeGrid grid, IMyProjector projector)
        {
            // The OrientAndCreep verb owns flight: tool faced down the line, creeping onto the stop
            // ring, then holding pose. This state only watches the WORK.
            var projected = projector.ProjectedGrid;
            if (_fc.Status == FlightController.VerbStatus.Failed)
            { DeferTarget("weld-in: " + _fc.FailReason); BackOut(grid); return; }

            if (_coord != null) _coord.TryClaim(m.Id, projector.EntityId, _targetCell, _targetWorld); // keep the claim fresh
            bool placed = projected == null || projected.GetCubeBlock(_targetCell) == null;
            if (placed)
            {
                Log(m, string.Format("weld: block complete ({0} remaining)", projector.RemainingBlocks));
                BackOut(grid);
                return;
            }
            if ((DateTime.UtcNow - _weldStart).TotalSeconds > WeldBlockTimeoutSecs)
            { DeferTarget("weld timeout (missing components?)"); BackOut(grid); return; }
        }

        private void BackOut(IMyCubeGrid grid)
        {
            DroneUtil.SetWelders(grid, false);
            _fc.SlideTo(grid, _approachPoint, ApproachSpeed); // attitude LOCKED — never swing the nose beside the hull
            _workSub = WorkBackOut;
        }

        private void TickBackOut(Colony colony, Mission m, IMyCubeGrid grid)
        {
            if (_fc.Status == FlightController.VerbStatus.Failed)
            { RetryOrFail(colony, m, grid, "back-out: " + _fc.FailReason); return; }
            if (_fc.Status != FlightController.VerbStatus.Done) return;
            _hasTarget = false;
            _workSub = WorkSelect;
        }

        private void DeferTarget(string why)
        {
            if (_hasTarget) _deferred.Add(_targetCell);
            if (_coord != null) _coord.ReleaseClaim(_missionId); // free the cell — another welder's angle may work
            _hasTarget = false;
            _workSub = WorkSelect;
        }

        // ── Return home (resupply or done) ───────────────────────────────────────────────────────────
        private void BeginReturn(Colony colony, Mission m, IMyCubeGrid grid, bool resupply)
        {
            DroneUtil.SetWelders(grid, false);
            _returningToResupply = resupply;
            m.Phase = PhaseReturn;
            Vector3D standoff;
            if (!TryCoreStandoff(colony, out standoff)) { Complete(colony, m, grid, "no core"); return; }
            _fc.ClearWorkVolumes();
            _fc.Transit(grid, FlightCorridor.Plan(grid.GetPosition(), standoff, _fc.Profile.CruiseAgl), CruiseSpeedLimit);
        }

        private void TickReturn(Colony colony, Mission m, IMyCubeGrid grid)
        {
            if (_fc.Status == FlightController.VerbStatus.Failed)
            { RetryOrFail(colony, m, grid, "return: " + _fc.FailReason); return; }
            if (_fc.Status != FlightController.VerbStatus.Done) return;
            _retries = 0;
            if (_returningToResupply) BeginDockLoad(colony, m, grid); // dock -> load -> charge -> back to the site
            else { _fc.Release(grid); Complete(colony, m, grid, "recalled to base"); } // graceful recall: stop here
        }

        // Graceful back-out (/colony recall): stop welding and fly home; mission completes at the standoff.
        public void Recall(Colony colony, Mission m, IMyCubeGrid grid)
        {
            BeginReturn(colony, m, grid, false);
        }

        // ── Shared flight helpers ────────────────────────────────────────────────────────────────────
        private Vector3D SiteStandoff(IMyProjector projector)
        {
            var projected = projector.ProjectedGrid;
            BoundingBoxD box = projected != null ? projected.WorldAABB : projector.CubeGrid.WorldAABB;
            Vector3D up = _nav.Valid ? _nav.GravityUp : Vector3D.Up;
            return _orbit.ClampAboveTerrain(box.Center + up * (box.HalfExtents.Length() + SiteStandoffUp), up, WelderMinAgl);
        }

        // The construction's keep-out sphere: projected + real extents, inflated by the drone's own
        // size — the orbit rule keeps every flight leg outside this; only the final nose-in enters.
        private BoundingSphereD KeepOut(IMyProjector projector, IMyCubeGrid self)
        {
            var projected = projector.ProjectedGrid;
            BoundingBoxD box = projected != null ? projected.WorldAABB : projector.CubeGrid.WorldAABB;
            double selfRadius = self.WorldAABB.HalfExtents.Length();
            return new BoundingSphereD(box.Center, box.HalfExtents.Length() + selfRadius + 4.0);
        }


        private bool TryCoreStandoff(Colony colony, out Vector3D standoff)
        {
            standoff = default(Vector3D);
            var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as IMyCubeBlock;
            if (core == null) return false;
            Vector3D pos = core.GetPosition();
            standoff = pos + (_nav.Valid ? _nav.GravityUp : Vector3D.Up) * 100.0;
            return true;
        }

        // ── Watchdog / recovery / terminal states (same discipline as the miner) ─────────────────────
        private void ResetLeg()
        {
            _legStart = DateTime.UtcNow;
            _legMinDist = double.MaxValue;
            _legProgressTime = DateTime.UtcNow;
        }

        private string LegOk(double dist, double timeoutSecs, string leg)
        {
            if (dist < _legMinDist - LegProgressEps) { _legMinDist = dist; _legProgressTime = DateTime.UtcNow; }
            else if (dist < _legMinDist) _legMinDist = dist;
            if ((DateTime.UtcNow - _legStart).TotalSeconds > timeoutSecs) return leg + " timeout";
            if ((DateTime.UtcNow - _legProgressTime).TotalSeconds > LegStuckSecs) return leg + " no progress (stuck)";
            return null;
        }

        private void RetryOrFail(Colony colony, Mission m, IMyCubeGrid grid, string reason)
        {
            _retries++;
            if (_retries > MaxRetries) { Fail(colony, m, grid, reason); return; }
            Log(m, string.Format("retry {0}/{1} — {2}", _retries, MaxRetries, reason));
            // Soft reset: stabilize (flight core released → dampeners), re-enter the phase fresh.
            _fc.Release(grid);
            DroneUtil.SetWelders(grid, false);
            _dock.Reset(); // fresh dock attempt (new connector pick) on the retry
            switch (m.Phase)
            {
                case PhaseDockLoad: BeginDockLoad(colony, m, grid); break;
                case PhaseTransit:
                {
                    var proj = MyAPIGateway.Entities.GetEntityById(_projectorId) as IMyProjector;
                    if (proj != null) BeginTransit(m, grid, proj); else Fail(colony, m, grid, "projector gone");
                    break;
                }
                case PhaseWork:     _hasTarget = false; _workSub = WorkSelect; _fc.Hover(grid); break;
                case PhaseReturn:   BeginReturn(colony, m, grid, _returningToResupply); break;
            }
        }

        private void Complete(Colony colony, Mission m, IMyCubeGrid grid, string reason)
        {
            Cleanup(grid);
            colony.Missions.Complete(m.Id);
            var asset = colony.Assets.GetByEntityId(m.AssignedAssetId);
            if (asset != null) { asset.Status = AssetStatus.Idle; asset.AssignedMissionId = 0; }
            Log(m, "complete: " + reason);
        }

        private void Fail(Colony colony, Mission m, IMyCubeGrid grid, string reason)
        {
            Cleanup(grid);
            colony.Missions.Fail(m.Id);
            var asset = colony.Assets.GetByEntityId(m.AssignedAssetId);
            if (asset != null) { asset.Status = AssetStatus.Idle; asset.AssignedMissionId = 0; }
            if (!MyAPIGateway.Utilities.IsDedicated)
                MyAPIGateway.Utilities.ShowMessage("Colony", string.Format("Welder mission {0} failed: {1}", m.Id, reason));
            Log(m, "FAILED: " + reason);
        }

        public void Abort(Colony colony, Mission m, IMyCubeGrid grid)
        {
            if (grid != null) Cleanup(grid);
            Fail(colony, m, grid, "aborted by command");
        }

        private void Cleanup(IMyCubeGrid grid)
        {
            if (grid == null) return;
            if (_cons != null) _cons.Release(grid.EntityId);
            if (_coord != null) _coord.ReleaseClaim(_missionId);
            DroneUtil.SetWelders(grid, false);
            DroneUtil.SetBatteriesRecharge(grid, false); // never leave Recharge leaked into idle
            _fc.Release(grid); // flight core release restores dampeners — the terminal safety net
            _fc.ClearWorkVolumes();
        }

        private void Log(Mission m, string msg)
        {
            if ((DateTime.UtcNow - _lastLog).TotalSeconds < 1.0) return; // light throttle for repeated states
            _lastLog = DateTime.UtcNow;
            MyLog.Default.WriteLineAndConsole(string.Format("[ColonyFramework] Weld mission {0}: {1}", m.Id, msg));
        }
    }
}
