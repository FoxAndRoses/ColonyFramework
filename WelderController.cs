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
        private const double WeldReach         = 2.0;   // m past the drone's own radius that counts as "in welder range"
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

        private readonly BoreController _fly = new BoreController();
        private readonly NavState _nav = new NavState();
        private readonly AvoidanceProbe _avoid = new AvoidanceProbe();
        private readonly OrbitNav _orbit = new OrbitNav(); // fluid around-the-hull navigation at the site
        private readonly DockMachine _dock = new DockMachine(); // shared proven connector-dock sequence

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

        // ── Entry point (called at ~6 Hz by DroneExecutor) ──────────────────────────────────────────
        public void Advance(Colony colony, Mission m, IMyCubeGrid grid)
        {
            var rc = DroneUtil.FindRc(grid);
            if (rc == null) { Fail(colony, m, grid, "no remote control"); return; }
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
        }

        // ── Commission: same shape as the miner (welders don't need the full draw math — verify RC,
        // welders and a connector exist, then head to load) ──────────────────────────────────────────
        private void TickCommission(Colony colony, Mission m, IMyCubeGrid grid)
        {
            if (DroneUtil.FindWelders(grid).Count == 0) { Fail(colony, m, grid, "no welders on drone"); return; }
            if (DroneUtil.FindConnector(grid) == null) { Fail(colony, m, grid, "no connector on drone"); return; }
            Log(m, "commissioned (welder), heading to load components");
            BeginDockLoad(colony, m, grid);
        }

        // ── DockLoad: get connected to the base, pull components, recharge, then fly to the site ────
        private void BeginDockLoad(Colony colony, Mission m, IMyCubeGrid grid)
        {
            m.Phase = PhaseDockLoad;
            var droneCon = DroneUtil.FindConnector(grid);
            _loading = droneCon != null && droneCon.Status == MyShipConnectorStatus.Connected;
            if (!_loading)
            {
                DroneUtil.PrepareForFlight(grid); // batteries auto + thrusters on + unlock — autopilot won't do any of it
                _dock.Reset();                    // shared dock machine flies the approach itself
            }
            else
            {
                DroneUtil.SetBatteriesRecharge(grid, true);
                _chargeRefPct = DroneUtil.MinBatteryCharge(grid);
                _chargeProgressTime = DateTime.UtcNow;
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
                string r = _dock.Tick(grid, _nav, droneCon, core);
                if (r == DockMachine.Connected) { EnterLoading(grid); return; }
                if (r != null && r.StartsWith("fail:")) { RetryOrFail(colony, m, grid, r.Substring(5)); return; }
                return;
            }

            // Connected: pull components for the projector's remaining blueprint, recharge, undock.
            var want = new ProjectorReader().RequiredComponents(core.CubeGrid, out _tmpProj, out _tmpBlocks);
            DroneUtil.LoadComponents(grid, droneCon, want);

            double charge = DroneUtil.MinBatteryCharge(grid);
            if (charge > _chargeRefPct + 0.01) { _chargeRefPct = charge; _chargeProgressTime = DateTime.UtcNow; }
            bool charged = charge >= ChargeTargetPct || DroneUtil.HasInfinitePower(grid);
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

        private void EnterLoading(IMyCubeGrid grid)
        {
            _loading = true;
            _fly.Release(grid);
            DroneUtil.SetBatteriesRecharge(grid, true);
            _chargeRefPct = DroneUtil.MinBatteryCharge(grid);
            _chargeProgressTime = DateTime.UtcNow;
            _legStart = DateTime.UtcNow;
        }

        // ── Transit to the site standoff (autopilot + avoidance) ─────────────────────────────────────
        private void BeginTransit(Mission m, IMyCubeGrid grid, IMyProjector projector)
        {
            m.Phase = PhaseTransit;
            CruiseTo(grid, SiteStandoff(projector), CruiseSpeedLimit, "build site", null);
        }

        private void TickTransit(Colony colony, Mission m, IMyCubeGrid grid, IMyProjector projector)
        {
            Vector3D standoff = SiteStandoff(projector);
            double dist = Vector3D.Distance(grid.GetPosition(), standoff);
            if (dist > ArriveDistance)
            {
                var rc = DroneUtil.FindRc(grid);
                if (rc != null && !rc.DampenersOverride) rc.DampenersOverride = true;
                Vector3D via; string obstacle;
                if (_avoid.TryGetDetour(_nav, grid, standoff, out via, out obstacle,
                    40.0, projector.CubeGrid.EntityId)) // destination structure isn't an obstacle
                {
                    CruiseTo(grid, standoff, CruiseSpeedLimit, "build site", via);
                    Log(m, string.Format("deflected around obstacle {0}, resumed heading", obstacle));
                    return;
                }
                string fail = LegOk(dist, LegTimeoutSecs, "site transit");
                if (fail != null) RetryOrFail(colony, m, grid, fail);
                return;
            }
            var rc2 = DroneUtil.FindRc(grid);
            if (rc2 != null) rc2.SetAutoPilotEnabled(false);
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
            // Low power protection (welders + thrusters drain fast).
            if (!DroneUtil.HasInfinitePower(grid))
            {
                double charge = DroneUtil.MinBatteryCharge(grid);
                if (charge < LowPowerThreshold)
                {
                    if (!MyAPIGateway.Utilities.IsDedicated)
                        MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                            "Welder low power ({0:N0}%), returning to recharge", charge * 100));
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
            _fly.Drive(grid, grid.WorldMatrix.Forward, grid.WorldMatrix.Forward, 0); // hold (dampers)
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
            // Radial approach: from the construction's volume center THROUGH the block, out to standoff.
            // The radial's elevation is floored at horizontal — nothing is ever approached from below —
            // and the point is clamped above terrain, so a ground-level block gets a level approach.
            Vector3D up0 = _nav.Valid ? _nav.GravityUp : Vector3D.Up;
            Vector3D center = projected.WorldAABB.Center;
            Vector3D radial = _targetWorld - center;
            double downComp = Vector3D.Dot(radial, up0);
            if (downComp < 0) radial -= up0 * downComp;      // strip the below-horizon component
            if (radial.LengthSquared() < 1.0) radial = up0;  // dead-centre / straight-below: come from above
            radial = Vector3D.Normalize(radial);
            double selfRadius = grid.WorldAABB.HalfExtents.Length();
            _approachPoint = _orbit.ClampAboveTerrain(_targetWorld + radial * (selfRadius + ApproachStandoff), up0, ApproachMinAgl);
            _hasTarget = true;
            _workSub = WorkApproach;
            _approachStart = DateTime.UtcNow;
            _lastSiteReroute = default(DateTime); // first TickApproach issues the first orbit/direct leg immediately
            Log(m, string.Format("weld: target block at ({0:F0}, {1:F0}, {2:F0}), {3} remaining",
                _targetWorld.X, _targetWorld.Y, _targetWorld.Z, projector.RemainingBlocks));
        }

        private void TickApproach(Colony colony, Mission m, IMyCubeGrid grid, IMyProjector projector)
        {
            if (!_hasTarget) { _workSub = WorkSelect; return; }
            Vector3D pos = grid.GetPosition();
            Vector3D up = _nav.Valid ? _nav.GravityUp : Vector3D.Up;
            double dist = Vector3D.Distance(pos, _approachPoint);

            if (dist > ApproachTol || _nav.Speed > SettleSpeed)
            {
                var rcm = DroneUtil.FindRc(grid);
                if (rcm != null && !rcm.DampenersOverride) rcm.DampenersOverride = true;

                // Whole-approach budget (orbit legs re-issue autopilot, so a per-leg watchdog can't
                // time this): too long overall → this block is awkward to reach right now, defer it.
                if ((DateTime.UtcNow - _approachStart).TotalSeconds > ApproachTimeoutSecs)
                { DeferTarget("approach timeout"); return; }

                if ((DateTime.UtcNow - _lastSiteReroute).TotalSeconds >= SiteRerouteSecs)
                {
                    _lastSiteReroute = DateTime.UtcNow;

                    // Third-party obstacles first (other grids / terrain rises); the construction
                    // itself is excluded — the orbit rule owns it.
                    Vector3D via; string obstacle;
                    if (_avoid.TryGetDetour(_nav, grid, _approachPoint, out via, out obstacle,
                        8.0, projector.CubeGrid.EntityId))
                    {
                        CruiseToDirect(grid, via, (float)OrbitLegSpeed, "avoid detour");
                        Log(m, string.Format("deflected around obstacle {0}", obstacle));
                        return;
                    }

                    // Orbit-or-direct: slide along the keep-out sphere until line-of-sight to the
                    // approach point opens, then fly straight at it. Stateless — one step per reroute.
                    Vector3D wp;
                    bool direct = _orbit.NextStep(pos, _approachPoint, KeepOut(projector, grid), up, WelderMinAgl, out wp);
                    CruiseToDirect(grid, wp, (float)(direct ? ApproachSpeed : OrbitLegSpeed),
                        direct ? "weld approach" : "orbit");
                }
                return;
            }
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) rc.SetAutoPilotEnabled(false);

            // Reach test: one ray from the approach point to the block — if the construction itself is
            // in the way, this block isn't externally reachable right now; defer it.
            IHitInfo hit;
            if (MyAPIGateway.Physics.CastRay(_approachPoint, _targetWorld, out hit) && hit != null)
            {
                var hitGrid = hit.HitEntity as IMyCubeGrid;
                if (hitGrid != null && hitGrid.EntityId != grid.EntityId
                    && Vector3D.DistanceSquared(hit.Position, _targetWorld) > 9.0)
                { DeferTarget("blocked by structure"); return; }
            }
            DroneUtil.SetWelders(grid, true);
            _weldStart = DateTime.UtcNow;
            _workSub = WorkWeldIn;
            ResetLeg();
        }

        private void TickWeldIn(Colony colony, Mission m, IMyCubeGrid grid, IMyProjector projector)
        {
            var projected = projector.ProjectedGrid;
            Vector3D pos = grid.GetPosition();
            Vector3D toBlock = _targetWorld - pos;
            double dist = toBlock.Length();
            if (dist < 1e-2) return;
            Vector3D dir = toBlock / dist;

            _fly.Face(grid, DroneUtil.FindRc(grid).WorldMatrix.Forward, dir); // welders face RC forward

            double selfRadius = grid.WorldAABB.HalfExtents.Length();
            double stopDist = selfRadius + WeldReach;

            // Done? The projection no longer contains the cell once the block is placed; the welder
            // (still running, still in range) then welds it up. Give it a beat past placement.
            bool placed = projected == null || projected.GetCubeBlock(_targetCell) == null;
            if (placed)
            {
                Log(m, string.Format("weld: block complete ({0} remaining)", projector.RemainingBlocks));
                BackOut(grid);
                return;
            }
            if ((DateTime.UtcNow - _weldStart).TotalSeconds > WeldBlockTimeoutSecs)
            { DeferTarget("weld timeout (missing components?)"); BackOut(grid); return; }

            if (dist > stopDist)
                _fly.ThrustAlong(grid, dir, WeldInSpeed, 1.0f, 0.5f); // ease nose-in; dampers brake the rest
            // inside stopDist: hold and let the welder work (dampers hold position)
        }

        private void BackOut(IMyCubeGrid grid)
        {
            DroneUtil.SetWelders(grid, false);
            _workSub = WorkBackOut;
            ResetLeg();
        }

        private void TickBackOut(Colony colony, Mission m, IMyCubeGrid grid)
        {
            Vector3D pos = grid.GetPosition();
            Vector3D toOut = _approachPoint - pos;
            double dist = toOut.Length();
            if (dist > ApproachTol)
            {
                _fly.ThrustAlong(grid, toOut / dist, ApproachSpeed * 0.5, 1.0f, 0.5f);
                string fail = LegOk(dist, 60.0, "weld back-out");
                if (fail != null) { RetryOrFail(colony, m, grid, fail); }
                return;
            }
            _fly.Release(grid);
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) rc.DampenersOverride = true;
            _hasTarget = false;
            _workSub = WorkSelect;
        }

        private void DeferTarget(string why)
        {
            if (_hasTarget) _deferred.Add(_targetCell);
            _hasTarget = false;
            _workSub = WorkSelect;
        }

        // ── Return home (resupply or done) ───────────────────────────────────────────────────────────
        private void BeginReturn(Colony colony, Mission m, IMyCubeGrid grid, bool resupply)
        {
            DroneUtil.SetWelders(grid, false);
            _fly.Release(grid);
            _returningToResupply = resupply;
            m.Phase = PhaseReturn;
            Vector3D standoff;
            if (!TryCoreStandoff(colony, out standoff)) { Complete(colony, m, grid, "no core"); return; }
            CruiseTo(grid, standoff, CruiseSpeedLimit, "base standoff", null);
        }

        private void TickReturn(Colony colony, Mission m, IMyCubeGrid grid)
        {
            Vector3D standoff;
            if (!TryCoreStandoff(colony, out standoff)) { Complete(colony, m, grid, "no core"); return; }
            double dist = Vector3D.Distance(grid.GetPosition(), standoff);
            if (dist > ArriveDistance)
            {
                var rc = DroneUtil.FindRc(grid);
                if (rc != null && !rc.DampenersOverride) rc.DampenersOverride = true;
                Vector3D via; string obstacle;
                if (_avoid.TryGetDetour(_nav, grid, standoff, out via, out obstacle))
                {
                    CruiseTo(grid, standoff, CruiseSpeedLimit, "base standoff", via);
                    Log(m, string.Format("deflected around obstacle {0}, resumed heading", obstacle));
                    return;
                }
                string fail = LegOk(dist, LegTimeoutSecs, "return");
                if (fail != null) RetryOrFail(colony, m, grid, fail);
                return;
            }
            var rc2 = DroneUtil.FindRc(grid);
            if (rc2 != null) rc2.SetAutoPilotEnabled(false);
            if (_nav.Speed > SettleSpeed) return; // let dampers stop it
            _retries = 0;
            if (_returningToResupply) BeginDockLoad(colony, m, grid); // dock -> load -> charge -> back to the site
            else Complete(colony, m, grid, "recalled to base");       // graceful recall: stop here, mission over
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

        // Climb-then-cruise (mirrors the miner's EngageCruise, incl. the optional avoidance detour).
        private void CruiseTo(IMyCubeGrid grid, Vector3D target, float speed, string label, Vector3D? via)
        {
            var rc = DroneUtil.FindRc(grid);
            if (rc == null) return;
            rc.DampenersOverride = true;
            rc.ClearWaypoints();
            Vector3D pos = grid.GetPosition();
            double agl;
            if (DroneUtil.TryGetAltitude(grid, out agl) && agl < CruiseAltitudeAgl)
                rc.AddWaypoint(pos + (_nav.Valid ? _nav.GravityUp : Vector3D.Up) * (CruiseAltitudeAgl - agl), "climb to cruise");
            if (via.HasValue) rc.AddWaypoint(via.Value, "avoid detour");
            rc.AddWaypoint(target, label);
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = speed;
            rc.SetAutoPilotEnabled(true);
            ResetLeg();
        }

        // Direct single-waypoint hop (short legs near the site/base — no cruise climb).
        private void CruiseToDirect(IMyCubeGrid grid, Vector3D target, float speed, string label)
        {
            var rc = DroneUtil.FindRc(grid);
            if (rc == null) return;
            rc.DampenersOverride = true;
            rc.ClearWaypoints();
            rc.AddWaypoint(target, label);
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = speed;
            rc.SetAutoPilotEnabled(true);
            ResetLeg();
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
            // Soft reset: stabilize, then re-enter the current phase from its beginning.
            _fly.Release(grid);
            DroneUtil.SetWelders(grid, false);
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) { rc.SetAutoPilotEnabled(false); rc.DampenersOverride = true; }
            _dock.Reset(); // fresh dock attempt (new connector pick) on the retry
            switch (m.Phase)
            {
                case PhaseDockLoad: BeginDockLoad(colony, m, grid); break;
                case PhaseTransit:  CruiseTo(grid, grid.GetPosition() + _nav.GravityUp * 30, (float)DockMoveSpeed, "recover climb", null); m.Phase = PhaseTransit; break;
                case PhaseWork:     _hasTarget = false; _workSub = WorkSelect; break;
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
            DroneUtil.SetWelders(grid, false);
            DroneUtil.SetBatteriesRecharge(grid, false); // never leave Recharge leaked into idle
            _fly.Release(grid);
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) { rc.SetAutoPilotEnabled(false); rc.DampenersOverride = true; }
        }

        private void Log(Mission m, string msg)
        {
            if ((DateTime.UtcNow - _lastLog).TotalSeconds < 1.0) return; // light throttle for repeated states
            _lastLog = DateTime.UtcNow;
            MyLog.Default.WriteLineAndConsole(string.Format("[ColonyFramework] Weld mission {0}: {1}", m.Id, msg));
        }
    }
}
