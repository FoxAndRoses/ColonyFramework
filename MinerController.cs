using System;
using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.Utils;
using VRageMath;
using FlightMode = Sandbox.ModAPI.Ingame.FlightMode;
using MyShipConnectorStatus = Sandbox.ModAPI.Ingame.MyShipConnectorStatus;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;
using IMyCubeBlock = VRage.Game.ModAPI.IMyCubeBlock;

namespace ColonyFramework
{
    // Executes ONE drone's mining mission. Instance per mission (owned by DroneExecutor).
    // Phases (Mission.Phase): Commission(0) -> Transit(1) -> StartBore(2) -> Mining(3) -> Retreat(4).
    // Mining is a "+" bore pattern: Center, North, South, East, West, each straight down to
    // ore-depth + 1 m, exiting on pattern-complete or cargo >= 80%.
    public class MinerController
    {
        private const int PhaseCommission = 0;
        private const int PhaseTransit    = 1;
        private const int PhaseStartBore  = 2;
        private const int PhaseMining     = 3;
        private const int PhaseRetreat    = 4;
        private const int PhaseDock       = 5;

        private const int DockOver    = 0; // autopilot to the staging point (out in front of + above the connector)
        private const int DockAlign   = 1; // DAMPENERS ON, UP HIGH: rotate to face the connector, then cut dampeners
        private const int DockDescend = 2; // DAMPENERS OFF: lower straight down to connector altitude, holding heading
        private const int DockLineup  = 3; // DAMPENERS OFF: fine lateral onto the connector axis
        private const int DockReverse = 4; // DAMPENERS OFF: reverse straight in along the axis and lock
        private const int DockUnload  = 5; // locked: transfer cargo into the base, then complete
        private const int DockRecover = 6; // a stage failed: fly back to the core standoff, then retry the dock

        private const int BoreReposition = 0;
        private const int BoreDescend    = 1;
        private const int BoreDrilling   = 2;
        private const int BoreAscend     = 3;

        private const int RetreatAscend = 0; // reverse straight up out of the shaft
        private const int RetreatLevel  = 1; // pitch to horizontal/level before turning to head home
        private const int RetreatReturn = 2; // autopilot to the colony-core standoff

        private const float  TransitSpeedLimit  = 25f;
        private const double ArriveDistance     = 15.0;
        private const double TransitTimeoutSecs = 150.0; // autopilot leg must arrive within this or it's aborted
        private const double ReturnTimeoutSecs  = 150.0; // same for the return-home autopilot leg
        private const double RunawayMargin      = 100.0; // if a leg gets this far past its closest approach, abort (flew past)
        private const int    MaxRetries         = 3;     // re-acquire + retry a failed leg this many times before erroring
        private const double LegStuckSecs       = 20.0;  // no closer-approach for this long = stuck / fighting itself
        private const double LegProgressEps     = 1.0;   // metres of improvement that counts as "progress"
        private const double CommissionSpikeSecs = 1.0;
        private const double MinRuntimeMinutes   = 10.0;

        private const double BoreApproachSpeed  = 6.0;   // fast descent, high altitude
        private const double BoreMediumSpeed    = 1.0;   // controlled, near surface
        private const double BoreDrillSpeed     = 0.15;  // in-rock crawl — drills stay ahead of the hull
        private const double AltFast            = 25.0;
        private const double AltMedium          = 5.0;

        private const double OrientReadyDot        = 0.985; // nose-down settled (~10°)
        private const double UprightMinDot         = 0.5;   // flip guard (>60° off down)
        private const double ContactSpeedEps       = 0.1;   // m/s descending slower than this = stalled on surface
        private const double MinDescendSecs        = 1.5;   // debounce before trusting a stall
        private const double SurfacePenetrationCap = -1.0;  // alt(m) below surface that forces contact (backstop)

        private const double TargetDepthMargin = 1.0;  // ore-waypoint depth + 1 m
        private const double BoreSpacing       = 1.0;  // "translate exactly 1 m"
        private const double ClearanceAlt      = 12.0; // ascend to this (m above surface) between bores
        private const double RepoSpeed         = 2.0;
        private const double RepoTolerance     = 0.4;
        private const double RetreatAscendSpeed = 3.0;
        private const double LevelDot           = 0.95; // drone "up" vs anti-gravity before heading home
        private const double ReturnClearanceAlt   = 30.0; // climb to this (m above surface) out of the shaft before returning
        private const double ReturnStandoffHeight = 100.0; // hold this far above the colony core (high, so the return cruise clears terrain)
        private const double CruiseAltitudeAgl    = 100.0; // climb to >= this (m above surface) before cruising — ground-avoidance floor
        private const double RecoverLevelTimeoutSecs = 10.0; // max time to gyro-level during a soft reset before resuming anyway

        // Docking is connector-relative and uses the DRONE CONNECTOR (not the RC) as the distance
        // reference. 3 steps: fly over the connector (StageFwd out + StageUp up) → descend straight
        // down to connector altitude → reverse into the connector at tiered speed. Dampeners stay ON.
        private const double StageFwd        = 20.0; // m in front of the base connector — descend out here, NOT directly above it (avoids hitting the base on the way down)
        private const double StageUp         = 40.0; // m above the base connector for the over/staging point
        private const double DockMoveSpeed   = 6.0;  // cruise cap flying over to the staging point
        private const double DockArriveTol   = 3.0;  // arrival tolerance at the staging / altitude points
        private const double DockSettleSpeed = 0.5;  // m/s "settled" threshold before advancing a leg
        private const double DockMaxSafeSpeed = 12.0; // m/s — if the dampeners-off controller runs away, panic-stop
        private const double DockDescendSpeed = 1.0; // m/s straight-down descent to connector height (gentle — limits momentum so it doesn't sink past)
        private const double DockAlignDot    = 0.98; // drone connector forward vs -base forward
        private const double CrawlFarDist    = 10.0; // > this: crawl fast tier
        private const double CrawlMidDist    = 5.0;  // > this: crawl mid tier (else near tier)
        private const double CrawlSpeedFar   = 2.0;  // m/s   (>10 m)
        private const double CrawlSpeedMid   = 1.0;  // m/s   (5–10 m)
        private const double CrawlSpeedNear  = 0.25; // m/s   (<5 m, until bump)
        private const double DockLateralTol  = 0.3;  // m off the connector axis we tolerate before reversing straight in
        private const double DockRunawayMargin = 4.0; // m past closest approach on a dampers-off dock leg = overshoot → recover
        private const double DockBelowTol     = 1.5;  // m below connector altitude tolerated before climbing back up
        private const double DockUnloadSecs  = 30.0; // max time to drain cargo once locked before completing anyway
        private const double DockTimeoutSecs = 180;  // give up → complete near base

        private const double CargoThreshold     = 0.80;
        private const double LowPowerThreshold  = 0.20;
        private const double BoreTimeoutSeconds = 600;  // ceiling = 10-min runtime floor
        private const double AlignmentMinDot    = 0.7;
        private const double StuckDistance      = 1.0;
        private const double StuckSeconds       = 20;

        // "+" pattern offsets: Center, North, South, East, West.
        private static readonly int[] DU = { 0, 0, 0, +1, -1 };
        private static readonly int[] DV = { 0, +1, -1, 0, 0 };
        private static readonly string[] BoreName = { "C", "N", "S", "E", "W" };

        private readonly BoreController _bore = new BoreController();

        private bool _started; // first Advance seen (distinguishes a fresh/resumed controller)
        private bool _commissionStarted;
        private DateTime _commissionStart;
        private DateTime _boreStart;
        private Vector3D _progressPos;
        private DateTime _progressTime;
        private bool _progressInit;

        private int _boreSub;
        private int _retreatSub;
        private int _dockSub;
        private long _dockConnectorId;
        private DateTime _dockStart;
        private DateTime _unloadStart;
        private DateTime _lastDockLog;
        private DateTime _legStart;   // start time of the current autopilot leg (transit / return)
        private double _legMinDist;   // closest approach to the leg's waypoint (for runaway detection)
        private DateTime _legProgressTime; // last time the leg got closer to its waypoint (for stuck detection)
        private int _retries;         // retries used at the current stuck point (reset on forward progress)
        private bool _recovering;     // soft reset in progress: stop + gyro-level before resuming
        private int _recoverResume;   // the phase to re-acquire once leveled
        private DateTime _recoverStart;
        private int _boreIndex;
        private bool _oriented;
        private DateTime _subStart;
        private bool _basisSet;
        private Vector3D _u, _v;
        private Vector3D _boreContact;
        private bool _depthSet;
        private double _targetDepth;

        public void Advance(Colony colony, Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            if (!_started) { _started = true; OnResume(colony, m, deposit, grid); }
            try
            {
                if (_recovering) { TickRecover(colony, m, deposit, grid); return; }
                switch (m.Phase)
                {
                    case PhaseCommission: TickCommission(colony, m, deposit, grid); break;
                    case PhaseTransit:    TickTransit(colony, m, deposit, grid); break;
                    case PhaseStartBore:  StartBore(colony, m, deposit, grid); break;
                    case PhaseMining:     TickMining(colony, m, deposit, grid); break;
                    case PhaseRetreat:    TickRetreating(colony, m, deposit, grid); break;
                    case PhaseDock:       TickDock(colony, m, deposit, grid); break;
                }
            }
            catch (Exception e)
            {
                MyLog.Default.WriteLineAndConsole("[ColonyFramework] Miner error mission " + m.Id + ": " + e.Message);
            }
        }

        // First tick of a fresh controller. A controller is created on dispatch (Phase=Commission,
        // handled normally) OR when a mission already in flight resumes after a world reload — in
        // which case every watchdog clock is still at default(DateTime) (year 0001), which would
        // make the very first LegOk read ~2000 years elapsed and instantly "time out". Give every
        // clock a fresh start and re-acquire the in-flight leg from a known-safe hover.
        private void OnResume(Colony colony, Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            var now = DateTime.UtcNow;
            _commissionStart = _boreStart = _dockStart = _subStart = _progressTime = now;
            _legStart = _legProgressTime = _lastDockLog = now;
            _legMinDist = double.MaxValue;
            if (m.Phase == PhaseCommission) return; // brand-new mission: dispatch drives commissioning

            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: resumed in phase {1}, re-acquiring", m.Id, m.Phase));
            StabilizeDrone(grid);
            switch (m.Phase)
            {
                case PhaseTransit:   EngageTransit(grid, deposit); break;
                case PhaseStartBore: break; // quick alignment check; fresh clocks are enough
                case PhaseMining:    BeginRetreat(m, grid, "resumed after reload"); break; // safest: back out and return
                case PhaseRetreat:   if (EngageReturn(colony, grid)) _retreatSub = RetreatReturn; else CompleteMission(colony, m, grid); break;
                case PhaseDock:      BeginDock(colony, m, grid, grid.GetPosition()); break;
            }
        }

        // ── Commission: spike consumers, estimate runtime, refuse if under reserve ──────────
        private void TickCommission(Colony colony, Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            if (DroneUtil.HasInfinitePower(grid))
            {
                MyLog.Default.WriteLineAndConsole(string.Format(
                    "[ColonyFramework] Mission {0}: commissioned (reactor power), dispatching", m.Id));
                EngageTransit(grid, deposit);
                m.Phase = PhaseTransit;
                return;
            }

            if (!_commissionStarted)
            {
                _commissionStarted = true;
                _commissionStart = DateTime.UtcNow;
                DroneUtil.SetSpike(grid, true);
                return;
            }

            if ((DateTime.UtcNow - _commissionStart).TotalSeconds < CommissionSpikeSecs) return;

            double stored, output;
            int batteries = DroneUtil.SumBatteryPower(grid, out stored, out output);
            DroneUtil.SetSpike(grid, false);

            double runtimeMin = batteries == 0 ? 999.0
                              : output > 0 ? (stored / output) * 60.0 : 0.0;

            if (runtimeMin >= MinRuntimeMinutes)
            {
                MyLog.Default.WriteLineAndConsole(string.Format(
                    "[ColonyFramework] Mission {0}: commissioned: ~{1:F0} min runtime, dispatching", m.Id, runtimeMin));
                EngageTransit(grid, deposit);
                m.Phase = PhaseTransit;
            }
            else
            {
                FailMission(colony, m, grid, string.Format(
                    "insufficient runtime ({0:F1} min < {1:F0}) — staying idle", runtimeMin, MinRuntimeMinutes));
            }
        }

        private void EngageTransit(IMyCubeGrid grid, DepositRecord deposit)
        {
            DroneUtil.ReleaseGrid(grid);
            Vector3D standoff = NavMath.ComputeStandoff(deposit.Position, grid.GetPosition());
            EngageCruise(grid, standoff, TransitSpeedLimit, "Deposit " + deposit.Id + " standoff");
        }

        // Climb-then-cruise RC route used for the long transit/return legs: climb STRAIGHT UP to a
        // safe cruise altitude first (if below it), then fly diagonally to the high standoff — so the
        // drone never skims terrain on a straight A→B line. RC autopilot can climb up and descend
        // diagonally fine; it just can't pitch straight down, so the route never ends straight below.
        private void EngageCruise(IMyCubeGrid grid, Vector3D target, float speed, string label)
        {
            var rc = DroneUtil.FindRc(grid);
            if (rc == null) return;
            rc.DampenersOverride = true; // autopilot needs dampeners to brake (also heals a drone left off)
            rc.ClearWaypoints();
            Vector3D pos = grid.GetPosition();
            double agl;
            if (DroneUtil.TryGetAltitude(grid, out agl) && agl < CruiseAltitudeAgl)
                rc.AddWaypoint(pos + Up(pos) * (CruiseAltitudeAgl - agl), "climb to cruise");
            rc.AddWaypoint(target, label);
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = speed;
            rc.SetAutoPilotEnabled(true);
            ResetLeg();
        }

        private void TickTransit(Colony colony, Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            Vector3D standoff = NavMath.ComputeStandoff(deposit.Position, grid.GetPosition());
            double dist = Vector3D.Distance(grid.GetPosition(), standoff);

            if (dist > ArriveDistance)
            {
                // Dampeners MUST stay on so autopilot can brake at the waypoint (off = flies past
                // forever). Set true only when it's actually off, so we heal a poisoned/resumed
                // drone without writing the property every tick (which fights autopilot).
                var rc2 = DroneUtil.FindRc(grid);
                if (rc2 != null && !rc2.DampenersOverride) rc2.DampenersOverride = true;
                if ((DateTime.UtcNow - _lastDockLog).TotalSeconds >= 3)
                {
                    _lastDockLog = DateTime.UtcNow;
                    double spd = grid.Physics != null ? grid.Physics.LinearVelocity.Length() : 0;
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] Mission {0}: transit dist={1:F0} vel={2:F1} damp={3}",
                        m.Id, dist, spd, rc2 != null && rc2.DampenersOverride));
                }
                string fail = LegOk(dist, TransitTimeoutSecs, "transit");
                if (fail != null) RetryOrFail(colony, m, deposit, grid, fail);
                return; // still flying (or retrying)
            }

            _retries = 0; // arrived — forward progress
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) rc.SetAutoPilotEnabled(false);
            m.Phase = PhaseStartBore;
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: '{1}' arrived at deposit {2}, starting bore",
                m.Id, grid.DisplayName, m.TargetDepositId));
        }

        // Resets the per-leg watchdog trackers. Call when an autopilot/move leg is (re)engaged.
        private void ResetLeg()
        {
            _legStart = DateTime.UtcNow;
            _legMinDist = double.MaxValue;
            _legProgressTime = DateTime.UtcNow;
        }

        // Watchdog for a flight leg. Returns a non-null reason if the leg is failing — timeout
        // (never arrived), runaway (flew past its closest approach), or no progress (stuck /
        // fighting itself) — else null. The caller routes the reason into RetryOrFail.
        private string LegOk(double dist, double timeoutSecs, string leg)
        {
            if (dist < _legMinDist - LegProgressEps) { _legMinDist = dist; _legProgressTime = DateTime.UtcNow; }
            else if (dist < _legMinDist) _legMinDist = dist;

            if ((DateTime.UtcNow - _legStart).TotalSeconds > timeoutSecs) return leg + " timeout";
            if (dist > _legMinDist + RunawayMargin) return leg + " runaway (flew past)";
            if ((DateTime.UtcNow - _legProgressTime).TotalSeconds > LegStuckSecs) return leg + " no progress (stuck)";
            return null;
        }

        // Puts the drone in a known-safe hover: autopilot off, dampeners ON, gyro/thrust overrides
        // cleared, drills off. Used as the clean slate before re-acquiring a failed leg.
        private void StabilizeDrone(IMyCubeGrid grid)
        {
            if (grid == null) return;
            _bore.Release(grid);
            DroneUtil.SetDrills(grid, false);
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) { rc.SetAutoPilotEnabled(false); rc.DampenersOverride = true; }
        }

        // Self-heal: re-acquire and retry the current leg up to MaxRetries; then announce the error
        // in chat and fail the mission. Reason carries the failure cause.
        private void RetryOrFail(Colony colony, Mission m, DepositRecord deposit, IMyCubeGrid grid, string reason)
        {
            _retries++;
            if (_retries > MaxRetries)
            {
                MyLog.Default.WriteLineAndConsole(string.Format(
                    "[ColonyFramework] Mission {0}: ERROR after {1} retries: {2}", m.Id, MaxRetries, reason));
                if (!MyAPIGateway.Utilities.IsDedicated)
                    MyAPIGateway.Utilities.ShowMessage("Colony", string.Format("Error occurred: {0} (mission {1})", reason, m.Id));
                FailMission(colony, m, grid, reason);
                return;
            }

            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: retry {1}/{2} — {3}", m.Id, _retries, MaxRetries, reason));
            BeginRecover(grid, m.Phase);
        }

        // Soft reset: stop on dampeners and enter the gyro-level recovery before re-acquiring the leg.
        private void BeginRecover(IMyCubeGrid grid, int resumePhase)
        {
            StabilizeDrone(grid); // autopilot off, DAMPENERS ON, gyro/thrust cleared, drills off
            _recovering = true;
            _recoverResume = resumePhase;
            _recoverStart = DateTime.UtcNow;
        }

        // Runs each tick while recovering: gyro-rotate the drone level/upright (dampeners holding it
        // still), then re-acquire the failed phase from a safe, high start (the re-acquire routes
        // through EngageCruise / DockRecover, which climb to a safe altitude before proceeding).
        private void TickRecover(Colony colony, Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            Vector3D pos = grid.GetPosition();
            var rc = DroneUtil.FindRc(grid);
            if (rc != null && !rc.DampenersOverride) rc.DampenersOverride = true;

            double levelDot = _bore.Face(grid, grid.WorldMatrix.Up, Up(pos)); // gyro to upright; dampeners hold position
            double vel = grid.Physics != null ? grid.Physics.LinearVelocity.Length() : 0;
            bool level = levelDot > LevelDot && vel < DockSettleSpeed;
            bool timedOut = (DateTime.UtcNow - _recoverStart).TotalSeconds > RecoverLevelTimeoutSecs;
            if (!level && !timedOut) return; // keep leveling

            _bore.Release(grid); // clear the gyro override before handing back to autopilot
            _recovering = false;
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: recovered (level{1}), re-acquiring phase {2}",
                m.Id, timedOut && !level ? " timeout" : "", _recoverResume));
            switch (_recoverResume)
            {
                case PhaseTransit: EngageTransit(grid, deposit); break;
                case PhaseRetreat: if (EngageReturn(colony, grid)) _retreatSub = RetreatReturn; else CompleteMission(colony, m, grid); break;
                case PhaseDock:
                    Vector3D standoff;
                    if (TryCoreStandoff(colony, out standoff)) EngageCruise(grid, standoff, (float)DockMoveSpeed, "core standoff");
                    _dockSub = DockRecover;
                    _dockStart = DateTime.UtcNow;
                    break;
            }
        }

        private void StartBore(Colony colony, Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            var rc = DroneUtil.FindRc(grid);
            var drills = DroneUtil.FindDrills(grid);
            if (rc == null || drills.Count == 0)
            {
                FailMission(colony, m, grid, rc == null ? "no remote control" : "no drills");
                return;
            }

            double dot = Vector3D.Dot(drills[0].WorldMatrix.Forward, rc.WorldMatrix.Forward);
            if (dot < AlignmentMinDot)
            {
                FailMission(colony, m, grid, string.Format("drills not aligned with RC forward (dot {0:F2})", dot));
                return;
            }

            rc.SetAutoPilotEnabled(false);
            rc.DampenersOverride = true; // dampeners ON for the whole bore/retreat (never off during mining)
            _retries = 0; // entered mining — progress
            _boreStart = DateTime.UtcNow;
            _boreIndex = 0;
            _boreSub = BoreReposition;
            _subStart = DateTime.UtcNow;
            m.Phase = PhaseMining;
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: +-pattern bore at deposit {1}, {2} drills",
                m.Id, m.TargetDepositId, drills.Count));
        }

        // ── Mining: the "+" bore pattern ────────────────────────────────────────────────────
        private void TickMining(Colony colony, Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            var drills = DroneUtil.FindDrills(grid);
            if (drills.Count == 0) { BeginRetreat(m, grid, "drills lost"); return; }

            if (!DroneUtil.HasInfinitePower(grid))
            {
                double charge = DroneUtil.MinBatteryCharge(grid);
                if (charge < LowPowerThreshold)
                {
                    BeginRetreat(m, grid, string.Format("low power ({0:N0}%)", charge * 100));
                    return;
                }
            }

            Vector3D drillFwd = drills[0].WorldMatrix.Forward;
            Vector3D pos = grid.GetPosition();

            float interference;
            Vector3D gravity = MyAPIGateway.Physics.CalculateNaturalGravityAt(pos, out interference);
            Vector3D downDir = gravity.LengthSquared() > 0.01
                ? Vector3D.Normalize(gravity)
                : Vector3D.Normalize(deposit.Position - pos);

            if (!_basisSet)
            {
                _u = Vector3D.Normalize(Vector3D.CalculatePerpendicularVector(downDir));
                _v = Vector3D.Normalize(Vector3D.Cross(downDir, _u));
                _basisSet = true;
            }

            // Flip guard — only after the drone has been upright once.
            if (_oriented && Vector3D.Dot(drillFwd, downDir) < UprightMinDot)
            {
                BeginRetreat(m, grid, "orientation guard (tilt > 60° off vertical)");
                return;
            }

            double altitude;
            bool gotAlt = DroneUtil.TryGetAltitude(grid, out altitude);

            if (_boreSub == BoreReposition)
            {
                Vector3D target = deposit.Position + _u * (DU[_boreIndex] * BoreSpacing) + _v * (DV[_boreIndex] * BoreSpacing);
                Vector3D toT = target - pos;
                Vector3D horiz = toT - downDir * Vector3D.Dot(toT, downDir);
                if (horiz.Length() < RepoTolerance)
                {
                    _boreSub = BoreDescend;
                    _subStart = DateTime.UtcNow;
                }
                else
                {
                    _bore.Drive(grid, drillFwd, downDir, 0);              // hold nose-down; dampers hold altitude
                    _bore.ThrustAlong(grid, Vector3D.Normalize(horiz), RepoSpeed);
                }
            }
            else if (_boreSub == BoreDescend)
            {
                if (!_oriented)
                {
                    _bore.Drive(grid, drillFwd, downDir, 0);             // settle nose-down, no descent
                    if (Vector3D.Dot(drillFwd, downDir) > OrientReadyDot)
                    {
                        _oriented = true;
                        _subStart = DateTime.UtcNow;
                    }
                }
                else
                {
                    double sp = gotAlt
                        ? (altitude > AltFast ? BoreApproachSpeed : altitude > AltMedium ? BoreMediumSpeed : BoreDrillSpeed)
                        : BoreMediumSpeed;
                    _bore.Drive(grid, drillFwd, downDir, sp);            // descend drills-OFF until it stalls on the surface

                    double downSpeed = grid.Physics != null ? Vector3D.Dot(grid.Physics.LinearVelocity, downDir) : 0;
                    bool longEnough = (DateTime.UtcNow - _subStart).TotalSeconds > MinDescendSecs;
                    bool stalled = downSpeed < ContactSpeedEps;
                    bool tooDeep = gotAlt && altitude < SurfacePenetrationCap;
                    if (longEnough && (stalled || tooDeep))
                    {
                        _boreContact = pos;
                        if (!_depthSet)
                        {
                            _targetDepth = Vector3D.Dot(deposit.Position - pos, downDir) + TargetDepthMargin;
                            if (_targetDepth < TargetDepthMargin) _targetDepth = TargetDepthMargin;
                            _depthSet = true;
                        }
                        DroneUtil.SetDrills(grid, true);
                        _boreSub = BoreDrilling;
                        _subStart = DateTime.UtcNow;
                        MyLog.Default.WriteLineAndConsole(string.Format(
                            "[ColonyFramework] Mission {0}: bore {1} ({2}/5) start, target depth {3:F1}m",
                            m.Id, BoreName[_boreIndex], _boreIndex + 1, _targetDepth));
                    }
                }
            }
            else if (_boreSub == BoreDrilling)
            {
                if (DroneUtil.CargoFill(grid) >= CargoThreshold)
                {
                    DroneUtil.SetDrills(grid, false);
                    BeginRetreat(m, grid, "cargo 80%");
                    return;
                }
                double pen = Vector3D.Dot(pos - _boreContact, downDir);
                if (pen >= _targetDepth)
                {
                    DroneUtil.SetDrills(grid, false);
                    _boreSub = BoreAscend;
                    _subStart = DateTime.UtcNow;
                }
                else
                {
                    _bore.Drive(grid, drillFwd, downDir, BoreDrillSpeed);
                }
            }
            else // BoreAscend
            {
                _bore.Drive(grid, drillFwd, downDir, 0);                       // hold nose-down
                _bore.ThrustAlong(grid, -downDir, RetreatAscendSpeed, 1.0f);    // full-power climb (heavy with cargo)
                if (gotAlt && altitude >= ClearanceAlt)
                {
                    _boreIndex++;
                    if (_boreIndex >= DU.Length)
                    {
                        BeginRetreat(m, grid, "+ pattern complete");
                        return;
                    }
                    _boreSub = BoreReposition;
                    _subStart = DateTime.UtcNow;
                }
            }

            // Stuck watchdog (real-hang safety): reset whenever the drone moves > StuckDistance.
            if (!_progressInit || Vector3D.Distance(pos, _progressPos) > StuckDistance)
            {
                _progressPos = pos;
                _progressTime = DateTime.UtcNow;
                _progressInit = true;
            }
            else if ((DateTime.UtcNow - _progressTime).TotalSeconds > StuckSeconds)
            {
                BeginRetreat(m, grid, "no progress (stuck)");
                return;
            }

            if ((DateTime.UtcNow - _boreStart).TotalSeconds > BoreTimeoutSeconds)
                BeginRetreat(m, grid, "timeout");
        }

        private void BeginRetreat(Mission m, IMyCubeGrid grid, string reason)
        {
            DroneUtil.SetDrills(grid, false);
            _bore.Release(grid); // clear gyro + thrust overrides so retreat thrust takes effect
            _retreatSub = RetreatAscend;
            m.Phase = PhaseRetreat;
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: retreating ({1})", m.Id, reason));
        }

        private void TickRetreating(Colony colony, Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            Vector3D pos = grid.GetPosition();
            float interference;
            Vector3D gravity = MyAPIGateway.Physics.CalculateNaturalGravityAt(pos, out interference);
            Vector3D downDir = gravity.LengthSquared() > 0.01
                ? Vector3D.Normalize(gravity)
                : Vector3D.Normalize(deposit.Position - pos);

            if (_retreatSub == RetreatAscend)
            {
                // Reverse straight up out of the shaft: hold nose-down (so up-thrusters keep
                // pointing up) and climb at FULL power (the drone is heavy with cargo).
                var drills = DroneUtil.FindDrills(grid);
                Vector3D drillFwd = drills.Count > 0 ? drills[0].WorldMatrix.Forward : downDir;
                _bore.Drive(grid, drillFwd, downDir, 0);
                _bore.ThrustAlong(grid, -downDir, RetreatAscendSpeed, 1.0f);

                double alt;
                bool clear = DroneUtil.TryGetAltitude(grid, out alt)
                    ? alt >= ReturnClearanceAlt
                    : Vector3D.Distance(pos, deposit.Position) >= ReturnClearanceAlt;
                if (!clear) return;

                MyLog.Default.WriteLineAndConsole(string.Format(
                    "[ColonyFramework] Mission {0}: clear of shaft, leveling out", m.Id));
                _retries = 0; // sub-state advanced — progress
                _retreatSub = RetreatLevel;
                return;
            }

            if (_retreatSub == RetreatLevel)
            {
                // Pitch up to horizontal (drone "up" → anti-gravity) and hover in place BEFORE
                // turning to head home — avoids the unstable list when going straight from
                // nose-down to autopilot.
                Vector3D upDir = gravity.LengthSquared() > 0.01 ? -Vector3D.Normalize(gravity) : Vector3D.Up;
                double levelDot = _bore.Face(grid, grid.WorldMatrix.Up, upDir); // gyro levels; dampeners hold position

                if (levelDot > LevelDot)
                {
                    _bore.Release(grid);
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] Mission {0}: level, returning to base", m.Id));
                    if (!MyAPIGateway.Utilities.IsDedicated)
                        MyAPIGateway.Utilities.ShowMessage("Colony", "Mining mission complete, returning to base");
                    if (!EngageReturn(colony, grid)) { CompleteMission(colony, m, grid); return; }
                    _retries = 0; // sub-state advanced — progress
                    _retreatSub = RetreatReturn;
                }
                return;
            }

            // RetreatReturn: RC autopilot flies to the colony-core standoff, then we let dampeners
            // bring it to a FULL STOP before docking — disabling dampeners while still moving fast
            // sends it coasting into the ground.
            Vector3D coreStandoff;
            if (!TryCoreStandoff(colony, out coreStandoff)) { CompleteMission(colony, m, grid); return; }
            double rdist = Vector3D.Distance(pos, coreStandoff);
            if (rdist > ArriveDistance)
            {
                var rcd = DroneUtil.FindRc(grid);
                if (rcd != null && !rcd.DampenersOverride) rcd.DampenersOverride = true; // heal only if off (autopilot needs it to brake)
                string fail = LegOk(rdist, ReturnTimeoutSecs, "return");
                if (fail != null) RetryOrFail(colony, m, deposit, grid, fail);
                return; // still flying home (or retrying)
            }
            _retries = 0; // reached the core standoff — forward progress

            var rc = DroneUtil.FindRc(grid);
            if (rc != null) rc.SetAutoPilotEnabled(false); // dampeners stay ON and brake it to a hover

            double speed = grid.Physics != null ? grid.Physics.LinearVelocity.Length() : 0;
            if (speed > DockSettleSpeed) return; // wait until stopped (dampeners holding) before dampeners-off dock

            BeginDock(colony, m, grid, pos);
        }

        // Arrived at the core standoff — pick a base connector and start the docking approach.
        private void BeginDock(Colony colony, Mission m, IMyCubeGrid grid, Vector3D pos)
        {
            var coreBlock = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as IMyCubeBlock;
            IMyCubeGrid coreGrid = coreBlock != null ? coreBlock.CubeGrid : null;
            var baseCon = coreGrid != null ? DroneUtil.FindFreeConnectorOnGroup(coreGrid, pos) : null;
            var droneCon = DroneUtil.FindConnector(grid);
            if (baseCon == null || droneCon == null)
            {
                MyLog.Default.WriteLineAndConsole(string.Format(
                    "[ColonyFramework] Mission {0}: no connector available, completing at standoff", m.Id));
                CompleteMission(colony, m, grid);
                return;
            }

            _dockConnectorId = baseCon.EntityId;
            _dockSub = DockOver;
            _dockStart = DateTime.UtcNow;
            m.Phase = PhaseDock;

            // Dampeners stay ON. Autopilot flies us over the connector (dampeners braking it);
            // they're only cut later for the short final reverse, after we're stable at the connector.
            Vector3D over = baseCon.GetPosition() + baseCon.WorldMatrix.Forward * StageFwd + Up(baseCon.GetPosition()) * StageUp;
            EngageAutopilot(grid, over);
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: navigating to docking staging point", m.Id));
        }

        private static Vector3D Up(Vector3D at)
        {
            float interference;
            Vector3D g = MyAPIGateway.Physics.CalculateNaturalGravityAt(at, out interference);
            return g.LengthSquared() > 0.01 ? -Vector3D.Normalize(g) : Vector3D.Up;
        }

        private void EngageAutopilot(IMyCubeGrid grid, Vector3D point)
        {
            var rc = DroneUtil.FindRc(grid);
            if (rc == null) return;
            rc.DampenersOverride = true; // autopilot needs dampeners to brake
            rc.ClearWaypoints();
            rc.AddWaypoint(point, "Dock waypoint");
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = (float)DockMoveSpeed;
            rc.SetAutoPilotEnabled(true);
            ResetLeg();
        }

        private bool EngageReturn(Colony colony, IMyCubeGrid grid)
        {
            Vector3D standoff;
            if (!TryCoreStandoff(colony, out standoff)) return false;
            if (DroneUtil.FindRc(grid) == null) return false;
            EngageCruise(grid, standoff, TransitSpeedLimit, "Colony core standoff");
            return true;
        }

        // A holding point ReturnStandoffHeight metres above the colony core block.
        private bool TryCoreStandoff(Colony colony, out Vector3D standoff)
        {
            standoff = Vector3D.Zero;
            long coreId = colony.State.CoreEntityId;
            if (coreId == 0) return false;
            var core = MyAPIGateway.Entities.GetEntityById(coreId);
            if (core == null) return false;
            Vector3D corePos = core.GetPosition();
            float interference;
            Vector3D g = MyAPIGateway.Physics.CalculateNaturalGravityAt(corePos, out interference);
            Vector3D up = g.LengthSquared() > 0.01 ? -Vector3D.Normalize(g) : Vector3D.Up;
            standoff = corePos + up * ReturnStandoffHeight;
            return true;
        }

        private void CompleteMission(Colony colony, Mission m, IMyCubeGrid grid)
        {
            _bore.Release(grid);
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) { rc.SetAutoPilotEnabled(false); rc.DampenersOverride = true; } // stop autopilot, restore dampeners
            double cargo = DroneUtil.CargoFill(grid);
            colony.Missions.Complete(m.Id);
            var asset = colony.Assets.GetByEntityId(m.AssignedAssetId);
            if (asset != null) { asset.Status = AssetStatus.Idle; asset.AssignedMissionId = 0; }
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0} complete: deposit {1} mined (drone cargo now {2:N0}%), asset idle",
                m.Id, m.TargetDepositId, cargo * 100));
        }

        // ── Dock: go to staging point, lower to connector height, get in-line, reverse-crawl, lock.
        // All distances use the DRONE CONNECTOR position (dPos), not the RC/grid centre.
        private void TickDock(Colony colony, Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            var baseCon = MyAPIGateway.Entities.GetEntityById(_dockConnectorId) as IMyShipConnector;
            var droneCon = DroneUtil.FindConnector(grid);
            if (baseCon == null || droneCon == null) { CompleteMission(colony, m, grid); return; }

            if ((DateTime.UtcNow - _dockStart).TotalSeconds > DockTimeoutSecs)
            {
                DockFallback(colony, m, grid, "dock timeout");
                return;
            }

            Vector3D bPos = baseCon.GetPosition();
            Vector3D bFwd = baseCon.WorldMatrix.Forward;     // outward face (flip to Backward if it docks wrong side)
            Vector3D dPos = droneCon.GetPosition();          // DRONE CONNECTOR — reference for the final reverse
            Vector3D dFwd = droneCon.WorldMatrix.Forward;
            Vector3D vel = grid.Physics != null ? (Vector3D)grid.Physics.LinearVelocity : Vector3D.Zero;
            // Autopilot drives the RC block (not the connector), so the over/down legs must gate on
            // the RC reaching its waypoint — gating on the connector parks it one RC↔connector
            // offset short forever (it never reaches DockArriveTol). The final reverse stays
            // connector-referenced and closed-loop, so it self-corrects that offset.
            Vector3D rcPos; { var r = DroneUtil.FindRc(grid); rcPos = r != null ? r.GetPosition() : dPos; }

            Vector3D up = Up(dPos);

            if (_dockSub == DockRecover)
            {
                // A stage failed: fly (DAMPENERS ON) back to the safe standoff above the core, then
                // restart the whole dock approach. This is the "reliable fallback" — always retreat
                // to a known-good position rather than fighting at the connector.
                Vector3D coreStandoff;
                if (!TryCoreStandoff(colony, out coreStandoff)) { CompleteMission(colony, m, grid); return; }
                var rcr = DroneUtil.FindRc(grid);
                if (rcr != null && !rcr.DampenersOverride) rcr.DampenersOverride = true;
                double rd = Vector3D.Distance(rcPos, coreStandoff);
                DockTelemetry(m, "recover", rd, vel.Length(), 0, 0);
                if (rd <= ArriveDistance && vel.Length() < DockSettleSpeed)
                {
                    if (rcr != null) rcr.SetAutoPilotEnabled(false);
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] Mission {0}: back at standoff, retrying dock", m.Id));
                    BeginDock(colony, m, grid, grid.GetPosition());
                }
                else { string fail = LegOk(rd, DockTimeoutSecs, "dock recover"); if (fail != null) { DockFallback(colony, m, grid, fail); return; } }
            }
            else if (_dockSub == DockOver)
            {
                // Autopilot (DAMPENERS ON) flies to the staging point: StageFwd in FRONT of the
                // connector and StageUp ABOVE it. Everything else happens from here — up high and
                // well clear of the base.
                Vector3D over = bPos + bFwd * StageFwd + up * StageUp;
                double dist = Vector3D.Distance(rcPos, over); // gate on the RC — autopilot drives it
                DockTelemetry(m, "over", dist, vel.Length(), Vector3D.Dot(dFwd, -bFwd), 0);
                if (dist <= DockArriveTol && vel.Length() < DockSettleSpeed)
                {
                    _retries = 0; // sub-state advanced — progress
                    var rc = DroneUtil.FindRc(grid);
                    if (rc != null) rc.SetAutoPilotEnabled(false); // gyro takes over for the turn; dampeners stay ON
                    _dockSub = DockAlign;
                    ResetLeg();
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] Mission {0}: at staging point ({1:F0} m in front, up high), facing connector", m.Id, StageFwd));
                }
                else { string fail = LegOk(dist, DockTimeoutSecs, "dock approach"); if (fail != null) { DockFallback(colony, m, grid, fail); return; } }
            }
            else if (_dockSub == DockAlign)
            {
                // DAMPENERS ON, UP HIGH at the staging point: rotate to face the connector. The drone
                // connector is mounted far from the grid centre, so the turn swings it through a wide
                // arc — doing it up here (not at connector altitude beside the base) keeps that arc
                // clear of the structure. Cut dampeners only once facing + stable.
                double a = _bore.Face(grid, dFwd, -bFwd); // gyro only; dampeners hold position
                DockTelemetry(m, "align", Vector3D.Distance(dPos, bPos), vel.Length(), a, 0);
                if (a > DockAlignDot && vel.Length() < DockSettleSpeed)
                {
                    _retries = 0; // sub-state advanced — progress
                    var rc = DroneUtil.FindRc(grid);
                    if (rc != null) rc.DampenersOverride = false; // stabilised above the connector; software dampers take over
                    _dockSub = DockDescend;
                    ResetLeg();
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] Mission {0}: facing connector + stable, descending on-axis", m.Id));
                }
                else { string fail = LegOk(vel.Length(), DockTimeoutSecs, "dock align"); if (fail != null) { DockFallback(colony, m, grid, fail); return; } }
            }
            else if (_dockSub == DockDescend)
            {
                // DAMPENERS OFF: gyro HOLDS the facing (no autopilot re-yaw) while Maneuver lowers the
                // connector straight down to connector altitude, still StageFwd out in front — the
                // same controlled descent used for the ore bore. Heading is fixed, so the connector
                // stays lined up on the way down (no swing into the base).
                _bore.Face(grid, dFwd, -bFwd);
                Vector3D target = bPos + bFwd * StageFwd; // connector altitude, still out in front
                Vector3D to = target - dPos;
                double dist = to.Length();
                double belowBy = Vector3D.Dot(bPos - dPos, up); // >0 = connector has sunk below the base connector
                double sat = _bore.Maneuver(grid, to, DockDescendSpeed, 0.0); // Maneuver eases to the target — climbs back if below
                DockTelemetry(m, "descend", dist, vel.Length(), Vector3D.Dot(dFwd, -bFwd), sat);
                // Only settle once at connector altitude (not still below it) and stopped.
                if (dist <= DockArriveTol && belowBy <= DockBelowTol && vel.Length() < DockSettleSpeed)
                {
                    _retries = 0; // sub-state advanced — progress
                    _dockSub = DockLineup;
                    ResetLeg();
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] Mission {0}: at connector altitude, lining up on axis", m.Id));
                }
                else if (DockWatch(dist)) { DockFallback(colony, m, grid, "dock descend overshoot/stuck"); return; }
            }
            else if (_dockSub == DockLineup || _dockSub == DockReverse)
            {
                // DAMPENERS OFF. Gyro holds the connector facing the base; Maneuver (software damper)
                // moves it. Never drive straight at the connector from off-axis — the hull jams
                // against the base. DockLineup slides laterally onto the axis; DockReverse then backs
                // straight in. Any overshoot/stuck on these legs triggers the standoff fallback.
                if (vel.Length() > DockMaxSafeSpeed) { DockFallback(colony, m, grid, string.Format("dock overspeed ({0:F0} m/s)", vel.Length())); return; }
                if (droneCon.Status == MyShipConnectorStatus.Connected) { BeginUnload(m, grid); return; }
                if (droneCon.Status == MyShipConnectorStatus.Connectable) droneCon.Connect();

                // Overshoot-below guard: if the connector has dropped below connector altitude (e.g.
                // momentum carried it down), go back to the descend stage to climb straight back up
                // before continuing — never keep sinking toward the surface.
                double belowBy = Vector3D.Dot(bPos - dPos, up);
                if (belowBy > DockBelowTol)
                {
                    _dockSub = DockDescend;
                    ResetLeg();
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] Mission {0}: dropped {1:F1} m below connector, climbing back to altitude", m.Id, belowBy));
                    return;
                }

                _bore.Face(grid, dFwd, -bFwd); // hold the drone connector aimed at the base connector

                Vector3D rel = dPos - bPos;                 // drone connector relative to base connector
                double along = Vector3D.Dot(rel, bFwd);     // distance out in front along the axis (>0 = in front)
                Vector3D lateralVec = rel - bFwd * along;   // off-axis component
                double lateral = lateralVec.Length();

                if (_dockSub == DockLineup)
                {
                    double holdAlong = System.Math.Max(along, StageFwd);   // stay/back off to standoff while sliding
                    Vector3D to = (bPos + bFwd * holdAlong) - dPos;        // ~ pure lateral slide onto the axis
                    double sat = _bore.Maneuver(grid, to, CrawlSpeedMid, 0.0);
                    DockTelemetry(m, "lineup", to.Length(), vel.Length(), Vector3D.Dot(dFwd, -bFwd), sat, lateral);
                    if (lateral <= DockLateralTol && vel.Length() < DockSettleSpeed)
                    {
                        _retries = 0;
                        _dockSub = DockReverse;
                        ResetLeg();
                        MyLog.Default.WriteLineAndConsole(string.Format(
                            "[ColonyFramework] Mission {0}: lined up on axis, reversing straight in", m.Id));
                    }
                    else if (DockWatch(lateral)) { DockFallback(colony, m, grid, "dock lineup overshoot/stuck"); return; }
                }
                else // DockReverse
                {
                    if (lateral > DockLateralTol * 3) { _dockSub = DockLineup; ResetLeg(); return; } // drifted off-axis: re-line-up
                    Vector3D to = bPos - dPos;
                    double d = to.Length();
                    double sp = d > CrawlFarDist ? CrawlSpeedFar : d > CrawlMidDist ? CrawlSpeedMid : CrawlSpeedNear;
                    double sat = _bore.Maneuver(grid, to, sp, 0.0); // magnet snaps at Connectable
                    DockTelemetry(m, "reverse", d, vel.Length(), Vector3D.Dot(dFwd, -bFwd), sat, lateral);
                    if (DockWatch(d)) { DockFallback(colony, m, grid, "dock reverse overshoot/stuck"); return; }
                }
            }
            else // DockUnload
            {
                TickUnload(colony, m, grid, droneCon);
            }
        }

        // Watchdog for a DAMPENERS-OFF dock leg (lineup/reverse): returns true if the connector has
        // overshot its closest approach by DockRunawayMargin, or made no progress for LegStuckSecs.
        private bool DockWatch(double dist)
        {
            if (dist < _legMinDist - LegProgressEps) { _legMinDist = dist; _legProgressTime = DateTime.UtcNow; }
            else if (dist < _legMinDist) _legMinDist = dist;
            bool overshoot = dist > _legMinDist + DockRunawayMargin;
            bool stuck = (DateTime.UtcNow - _legProgressTime).TotalSeconds > LegStuckSecs;
            return overshoot || stuck;
        }

        // Reliable dock fallback: stabilise (DAMPENERS ON, overrides cleared), fly back to the core
        // standoff, then restart the dock — up to MaxRetries, after which announce the error and fail.
        private void DockFallback(Colony colony, Mission m, IMyCubeGrid grid, string reason)
        {
            _retries++;
            if (_retries > MaxRetries)
            {
                MyLog.Default.WriteLineAndConsole(string.Format(
                    "[ColonyFramework] Mission {0}: ERROR after {1} dock retries: {2}", m.Id, MaxRetries, reason));
                if (!MyAPIGateway.Utilities.IsDedicated)
                    MyAPIGateway.Utilities.ShowMessage("Colony", string.Format("Error occurred: {0} (mission {1})", reason, m.Id));
                FailMission(colony, m, grid, reason);
                return;
            }
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: dock retry {1}/{2} — {3}, soft reset", m.Id, _retries, MaxRetries, reason));
            BeginRecover(grid, PhaseDock); // stop + gyro-level, then climb to the core standoff and re-dock
        }

        // Connector locked: stop drills/overrides, restore dampers, begin draining cargo into the base.
        private void BeginUnload(Mission m, IMyCubeGrid grid)
        {
            DroneUtil.SetDrills(grid, false);
            _bore.Release(grid);
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) rc.DampenersOverride = true; // locked to base; dampers on
            _dockSub = DockUnload;
            _unloadStart = DateTime.UtcNow;
            if (!MyAPIGateway.Utilities.IsDedicated)
                MyAPIGateway.Utilities.ShowMessage("Colony", "Docked at base, transferring cargo");
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: docked + locked, transferring cargo to base", m.Id));
        }

        // Push cargo through the locked connector into the base; complete when empty (or after a
        // grace timeout). The mission is only complete once cargo has been delivered.
        private void TickUnload(Colony colony, Mission m, IMyCubeGrid grid, IMyShipConnector droneCon)
        {
            bool empty = DroneUtil.UnloadCargo(grid, droneCon);
            bool timedOut = (DateTime.UtcNow - _unloadStart).TotalSeconds > DockUnloadSecs;
            if (!empty && !timedOut) return;

            double fill = DroneUtil.CargoFill(grid);
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: cargo transfer {1} (drone fill {2:N0}%)",
                m.Id, empty ? "complete" : "timed out", fill * 100));
            if (!MyAPIGateway.Utilities.IsDedicated)
                MyAPIGateway.Utilities.ShowMessage("Colony", empty ? "Mining mission complete, cargo delivered" : "Docked; cargo transfer incomplete");
            CompleteMission(colony, m, grid);
        }

        private void DockTelemetry(Mission m, string leg, double dist, double speed, double alignDot, double sat, double lateral = -1)
        {
            if ((DateTime.UtcNow - _lastDockLog).TotalSeconds < 3) return;
            _lastDockLog = DateTime.UtcNow;
            string latStr = lateral >= 0 ? string.Format(" lat={0:F1}", lateral) : "";
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: dock[{1}] dist={2:F1} vel={3:F1} align={4:F2} thrustSat={5:F2}{6}",
                m.Id, leg, dist, speed, alignDot, sat, latStr));
        }

        // Hard stop: clear controls, fail the mission, free the asset.
        public void Abort(Colony colony, Mission m, IMyCubeGrid grid)
        {
            if (grid != null)
            {
                _bore.Release(grid);
                var rc = DroneUtil.FindRc(grid);
                if (rc != null) rc.SetAutoPilotEnabled(false);
                DroneUtil.SetDrills(grid, false);
            }
            FailMission(colony, m, grid, "aborted by command");
        }

        // Graceful back-out: fly to standoff and complete.
        public void Recall(Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) rc.SetAutoPilotEnabled(false);
            BeginRetreat(m, grid, "recalled");
        }

        private void FailMission(Colony colony, Mission m, IMyCubeGrid grid, string reason)
        {
            colony.Missions.Fail(m.Id);
            var asset = colony.Assets.GetByEntityId(m.AssignedAssetId);
            if (asset != null) { asset.Status = AssetStatus.Idle; asset.AssignedMissionId = 0; }
            if (grid != null)
            {
                _bore.Release(grid);
                DroneUtil.SetDrills(grid, false);
                var rc = DroneUtil.FindRc(grid);
                if (rc != null) { rc.SetAutoPilotEnabled(false); rc.DampenersOverride = true; } // stop autopilot, restore dampeners
            }
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0} failed: {1}", m.Id, reason));
        }
    }
}
