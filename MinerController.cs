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

        // DAMPENERS STAY ON for the whole dock — gravity is always cancelled by the game. Autopilot
        // (diagonal, never straight down) puts the drone a clearance above the connector and in front;
        // only the short magnet-assisted reverse eases down to the low connector.
        private const int DockApproach = 0; // autopilot: diagonal to the shimmy-top (a safe height in front of the connector)
        private const int DockShimmy   = 1; // autopilot: zig-zag down (diagonal legs) to clearance above the connector
        private const int DockAlign    = 2; // gyro: face the connector (dampeners hold position)
        private const int DockReverse  = 3; // dampeners ON: gentle down-and-in nudge; magnet mates
        private const int DockUnload   = 4; // locked: transfer cargo into the base, then complete
        private const int DockRecover  = 5; // a stage failed: fly back to the core standoff, then retry the dock

        private const int BoreReposition = 0;
        private const int BoreDescend    = 1;
        private const int BoreDrilling   = 2;
        private const int BoreAscend     = 3;

        private const int RetreatAscend = 0; // reverse straight up out of the shaft
        private const int RetreatLevel  = 1; // pitch to horizontal/level before turning to head home
        private const int RetreatReturn = 2; // autopilot to the colony-core standoff

        private const float  TransitSpeedLimit  = 25f; // careful cap used near the base / destination
        private const float  CruiseSpeedLimit   = 70f; // fast cruise in open air between base and deposit
        private const double NearBaseSlowDist   = 150.0; // within this of the base OR the destination, drop to the careful cap
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
        private const double GroundAvoidAgl       = 40.0;  // mid-cruise: if AGL drops below this, climb back up (active ground avoidance)
        private const double ClimbReengageSecs    = 3.0;   // throttle for the mid-cruise climb re-engage
        private const double RecoverLevelTimeoutSecs = 10.0; // max time to gyro-level during a soft reset before resuming anyway

        // Docking is connector-relative and uses the DRONE CONNECTOR (not the RC) as the distance
        // reference. 3 steps: fly over the connector (StageFwd out + StageUp up) → descend straight
        // down to connector altitude → reverse into the connector at tiered speed. Dampeners stay ON.
        private const double StageFwd        = 20.0; // m in front of the base connector (staging point for the reverse)
        private const double StageUp         = 10.0; // (legacy)
        private const double DockClearance   = 1.0;  // m above connector altitude the shimmy bottoms out at (so the reverse is ~horizontal, not fighting dampers)
        // Shimmy descent: zig-zag DOWN in front of the connector via short diagonal autopilot legs
        // (RC autopilot can't go straight down, but it descends a slope fine). Dampeners stay ON.
        private const double ShimmyTop  = 25.0; // m above connector the diagonal approach delivers us to (top of the shimmy)
        private const double ShimmyDrop = 5.0;  // m of altitude dropped per shimmy leg
        private const double ShimmyStep = 8.0;  // m of horizontal zig-zag per leg (slope = atan(Drop/Step) ≈ 32°, autopilot-friendly)
        // Reverse-into-connector failsafe: a slow creep, so it fails ONLY on overshoot or bump-fail.
        private const double ReverseSpeed     = 1.0; // m/s firm backward creep toward the connector
        private const double ReverseOvershoot = 5.0; // m past the closest approach = flew past the connector → retry
        private const double BumpDist         = 3.0; // m connector-to-connector that counts as "at the connector" (magnet range)
        private const double BumpFailSecs     = 30.0;// centered & pressed this long without locking → bump failed → retry (time to fine-tune)
        private const double LockTrySecs      = 1.5; // how often to attempt the connector lock (not every tick — that's far too fast)
        // Velocity-controlled descent: modAPI dampers OFF, our up-thrust modulated by descent rate to
        // hold a slow "barely descending" rate that eases to 0 at connector altitude (no toggling).
        private const double DescendTargetRate   = 0.3;  // m/s — slow descent rate we hold ("barely descending")
        private const double DescendEaseGain     = 0.2;  // target rate = min(DescendTargetRate, |vErr|*this) — eases to 0 near target
        private const double DescendThrottleGain = 0.8;  // up-thrust (×weight) added per m/s of rate error
        private const double DescendMinThrottle  = 0.92; // gentle sink only (~0.8 m/s²) so it never builds speed it must hard-brake
        private const double DescendMaxThrottle  = 2.0;  // command up to 2× weight to brake (drone delivers what it can)
        private const double DescendBumpRate     = 0.3;  // if below target, climb at this rate (bump up)
        private const double DescendStuckSecs    = 30.0; // no altitude progress for this long → recover (lenient window)
        private const double DescendPanicSpeed   = 3.0;  // if descent exceeds this, hand to modAPI dampers (anti-slam)
        private const double DockMoveSpeed   = 6.0;  // cruise cap flying over to the staging point
        private const double DockArriveTol   = 3.0;  // arrival tolerance at the staging / altitude points
        private const double DockSettleSpeed = 0.5;  // m/s "settled" threshold before advancing a leg
        private const double DockMaxSafeSpeed = 12.0; // m/s — if the dampeners-off controller runs away, panic-stop
        private const double DockDescendSpeed = 1.0; // base m/s cap for the dampers-off dock pilot (scaled down each retry)
        // Per-axis pilot deadbands (dampers-off dock): inside these the drone just holds — no hunting.
        private const double AltDeadband     = 0.5;   // m vertical tolerance to count as "at connector altitude"
        private const double NavVelDeadband  = 0.2;   // m/s velocity tolerance the pilot ignores (anti-oscillation)
        private const double NavSettleSpeed  = 0.3;   // m/s "settled" threshold before a dampers-off sub-state advances
        private const double AlignHoldDot    = 0.9995;// once this aligned, stop gyro micro-correction (anti-oscillation)
        private const double DockAlignDot    = 0.98; // drone connector forward vs -base forward
        private const double CrawlFarDist    = 10.0; // > this: crawl fast tier
        private const double CrawlMidDist    = 5.0;  // > this: crawl mid tier (else near tier)
        private const double CrawlSpeedFar   = 2.0;  // m/s   (>10 m)
        private const double CrawlSpeedMid   = 1.0;  // m/s   (5–10 m)
        private const double CrawlSpeedNear  = 0.25; // m/s   (<5 m, until bump)
        private const double DockLateralTol  = 0.15; // m off the connector axis tolerated before backing straight in (fine — the magnet needs near-coaxial)
        private const double CenterGain      = 0.6;  // lateral shift speed = lateral * this → corrections go minute as it nears the axis
        private const double CenterMinSpeed  = 0.05; // m/s floor so minute lateral corrections still move the drone
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
        private readonly NavState _nav = new NavState(); // situational awareness, refreshed each tick

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
        private double _shimmyHeight; // current shimmy leg's target height above the connector
        private bool _shimmyOut;      // current shimmy leg's zig-zag side (out = StageFwd+ShimmyStep)
        private long _dockConnectorId;
        private DateTime _dockStart;
        private DateTime _bumpStart;  // when the reverse first pressed centered against the connector (for bump-fail detection)
        private DateTime _lastLockTry; // last time we attempted droneCon.Connect() (throttle, not every tick)
        private DateTime _unloadStart;
        private DateTime _lastDockLog;
        private DateTime _legStart;   // start time of the current autopilot leg (transit / return)
        private double _legMinDist;   // closest approach to the leg's waypoint (for runaway detection)
        private DateTime _legProgressTime; // last time the leg got closer to its waypoint (for stuck detection)
        private int _retries;         // retries used at the current stuck point (reset on forward progress)
        private bool _recovering;     // soft reset in progress: stop + gyro-level before resuming
        private int _recoverResume;   // the phase to re-acquire once leveled
        private DateTime _recoverStart;
        private DateTime _lastGroundAvoid; // throttle for mid-cruise climb re-engages
        private double _dockSpeedScale = 1.0; // soft failsafe: each dock retry crawls slower
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

            // Awareness before execution: refresh the drone's self-knowledge once per tick. No phase
            // issues movement while this is invalid (missing RC/physics) — it must know itself first.
            _nav.Refresh(grid, DroneUtil.FindRc(grid), DroneUtil.FindConnector(grid));
            if (!_nav.Valid) return;

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
            EngageCruise(grid, standoff, CruiseSpeedLimit, "Deposit " + deposit.Id + " standoff");
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

        // Active ground avoidance for autopilot legs: a straight/diagonal route between points of
        // different elevation can still dip toward terrain. If the drone is mid-cruise below the
        // floor AGL, signal the caller to re-engage the climb-then-cruise (throttled) so it pulls up.
        private bool NeedsClimb(IMyCubeGrid grid, Mission m, string leg)
        {
            double agl;
            if (!DroneUtil.TryGetAltitude(grid, out agl) || agl >= GroundAvoidAgl) return false;
            if ((DateTime.UtcNow - _lastGroundAvoid).TotalSeconds < ClimbReengageSecs) return false;
            _lastGroundAvoid = DateTime.UtcNow;
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: {1} too low ({2:F0} m AGL) — climbing", m.Id, leg, agl));
            return true;
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
                if (rc2 != null) rc2.SpeedLimit = CruiseSpeed(colony, grid.GetPosition(), standoff); // fast in open air, slow near base/deposit
                if (NeedsClimb(grid, m, "transit")) { EngageTransit(grid, deposit); return; } // active ground avoidance
                Narrate(m, "transit", standoff);
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

        // Throttled self-report: the drone narrating its awareness (pos/vel/orientation/altitude vs
        // target) for the current phase. One consistent telemetry line for every phase.
        private void Narrate(Mission m, string phase, Vector3D target, double sat = -1)
        {
            if ((DateTime.UtcNow - _lastDockLog).TotalSeconds < 3) return;
            _lastDockLog = DateTime.UtcNow;
            string line = _nav.Report(m.Id, phase, target);
            if (sat >= 0) line += string.Format(" thrustSat={0:F2}", sat);
            MyLog.Default.WriteLineAndConsole(line);
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
                    _bore.Drive(grid, drillFwd, downDir, sp);            // dampeners ON; push down, game brakes (player-like)

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
                    _bore.Drive(grid, drillFwd, downDir, BoreDrillSpeed); // dampeners ON; crawl into rock
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
                if (rcd != null) rcd.SpeedLimit = CruiseSpeed(colony, pos, coreStandoff); // fast in open air, slow near base
                if (NeedsClimb(grid, m, "return")) { EngageReturn(colony, grid); return; } // active ground avoidance
                Narrate(m, "return", coreStandoff);
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
        // The current shimmy leg's waypoint: StageFwd (+ ShimmyStep when zigged out) in front of the
        // connector, at the current shimmy height above it.
        private Vector3D ShimmyTarget(Vector3D bPos, Vector3D bFwd, Vector3D up)
        {
            return bPos + bFwd * (StageFwd + (_shimmyOut ? ShimmyStep : 0.0)) + up * _shimmyHeight;
        }

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
            _dockSub = DockApproach;
            _dockStart = DateTime.UtcNow;
            m.Phase = PhaseDock;

            // DAMPENERS STAY ON for the whole dock. Autopilot flies a diagonal to the shimmy-top
            // (in front of + a safe height above the connector); from there we zig-zag down.
            Vector3D bp = baseCon.GetPosition();
            Vector3D top = bp + baseCon.WorldMatrix.Forward * StageFwd + Up(bp) * ShimmyTop;
            EngageAutopilot(grid, top);
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
            EngageCruise(grid, standoff, CruiseSpeedLimit, "Colony core standoff");
            return true;
        }

        private bool TryCorePos(Colony colony, out Vector3D corePos)
        {
            corePos = Vector3D.Zero;
            long coreId = colony.State.CoreEntityId;
            if (coreId == 0) return false;
            var core = MyAPIGateway.Entities.GetEntityById(coreId);
            if (core == null) return false;
            corePos = core.GetPosition();
            return true;
        }

        // Fast cruise in open air; drop to the careful cap within NearBaseSlowDist of the base OR of
        // the destination, so it never barrels into the base structure or overshoots the deposit.
        private float CruiseSpeed(Colony colony, Vector3D pos, Vector3D target)
        {
            Vector3D corePos;
            bool nearBase = TryCorePos(colony, out corePos) && Vector3D.Distance(pos, corePos) < NearBaseSlowDist;
            bool nearDest = Vector3D.Distance(pos, target) < NearBaseSlowDist;
            return (nearBase || nearDest) ? TransitSpeedLimit : CruiseSpeedLimit;
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
            else if (_dockSub == DockApproach)
            {
                // Autopilot + DAMPENERS ON: diagonal to the shimmy-top — StageFwd in front of the
                // connector and ShimmyTop above it. A normal sloped descent (never straight down).
                Vector3D top = bPos + bFwd * StageFwd + up * ShimmyTop;
                double dist = Vector3D.Distance(rcPos, top);
                Narrate(m, "dock/approach", top);
                if (dist <= DockArriveTol && vel.Length() < DockSettleSpeed)
                {
                    _retries = 0;
                    _shimmyHeight = ShimmyTop - ShimmyDrop; // first step down
                    _shimmyOut = true;                      // zig out first
                    EngageAutopilot(grid, ShimmyTarget(bPos, bFwd, up));
                    _dockSub = DockShimmy;
                    ResetLeg();
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] Mission {0}: at shimmy-top, zig-zagging down to the connector", m.Id));
                }
                else { string fail = LegOk(dist, DockTimeoutSecs, "dock approach"); if (fail != null) { DockFallback(colony, m, grid, fail); return; } }
            }
            else if (_dockSub == DockShimmy)
            {
                // Autopilot + DAMPENERS ON: zig-zag DOWN in front of the connector. Each leg is a short
                // diagonal (ShimmyStep across, ShimmyDrop down) — autopilot descends slopes fine, so we
                // lose altitude in a tiny footprint with the dampeners always cancelling gravity.
                Vector3D target = ShimmyTarget(bPos, bFwd, up);
                double dist = Vector3D.Distance(rcPos, target);
                Narrate(m, "dock/shimmy", target);
                if (dist <= DockArriveTol && vel.Length() < DockSettleSpeed)
                {
                    _retries = 0;
                    if (_shimmyHeight <= DockClearance + 0.1)
                    {
                        var rc = DroneUtil.FindRc(grid);
                        if (rc != null) rc.SetAutoPilotEnabled(false); // gyro takes over; DAMPENERS STAY ON
                        _dockSub = DockAlign;
                        ResetLeg();
                        MyLog.Default.WriteLineAndConsole(string.Format(
                            "[ColonyFramework] Mission {0}: at connector altitude (+{1:F0} m), facing connector", m.Id, DockClearance));
                    }
                    else
                    {
                        _shimmyHeight = System.Math.Max(DockClearance, _shimmyHeight - ShimmyDrop);
                        _shimmyOut = !_shimmyOut; // zig back the other way as it drops
                        EngageAutopilot(grid, ShimmyTarget(bPos, bFwd, up));
                    }
                }
                else { string fail = LegOk(dist, DockTimeoutSecs, "dock shimmy"); if (fail != null) { DockFallback(colony, m, grid, fail); return; } }
            }
            else if (_dockSub == DockAlign)
            {
                // DAMPENERS ON: rotate to face the connector while the dampeners hold position.
                var rca = DroneUtil.FindRc(grid);
                if (rca != null && !rca.DampenersOverride) rca.DampenersOverride = true;
                double a = _bore.Face(grid, dFwd, -bFwd);
                DockTelemetry(m, "align", Vector3D.Distance(dPos, bPos), vel.Length(), a, 0);
                if (a > DockAlignDot && vel.Length() < DockSettleSpeed)
                {
                    _retries = 0;
                    _dockSub = DockReverse;
                    ResetLeg();                          // resets _legMinDist (closest-approach tracker)
                    _dockStart = DateTime.UtcNow;        // give the reverse its own generous time budget
                    _bumpStart = default(DateTime);
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] Mission {0}: facing connector + stable, reversing in (dampeners ON)", m.Id));
                }
                else { string fail = LegOk(vel.Length(), DockTimeoutSecs, "dock align"); if (fail != null) { DockFallback(colony, m, grid, fail); return; } }
            }
            else if (_dockSub == DockReverse)
            {
                // DAMPENERS ON the whole time — they cancel gravity and hold altitude. Gyro holds the
                // facing; a firm nudge backs the drone toward the connector (~horizontal) and the magnet
                // mates it. This is a deliberately slow creep, so it is NOT failed on "no progress" —
                // only TARGET OVERSHOOT (flew past the connector) or BUMP FAILED (reached the connector
                // but it won't lock) triggers a retry; otherwise it keeps creeping in.
                var rc = DroneUtil.FindRc(grid);
                if (rc != null && !rc.DampenersOverride) rc.DampenersOverride = true;

                // Try to LOCK occasionally (every LockTrySecs, NOT every tick): the instant the game
                // reports the connectors aligned enough, take it. On success BeginUnload stops ALL
                // maneuvering (Release clears thrust + gyro), keeps dampers ON, and drains cargo to base.
                if (droneCon.Status == MyShipConnectorStatus.Connected) { BeginUnload(m, grid); return; }
                if (droneCon.Status == MyShipConnectorStatus.Connectable
                    && (DateTime.UtcNow - _lastLockTry).TotalSeconds > LockTrySecs)
                {
                    _lastLockTry = DateTime.UtcNow;
                    droneCon.Connect();
                    if (droneCon.Status == MyShipConnectorStatus.Connected) { BeginUnload(m, grid); return; }
                }

                _bore.Face(grid, dFwd, -bFwd); // hold the drone connector aimed at the base connector

                // Decompose where the drone connector sits relative to the base connector AXIS (line
                // through bPos along bFwd): along = distance out in front, offAxis = sideways+vertical miss.
                Vector3D rel = dPos - bPos;
                double along = Vector3D.Dot(rel, bFwd);
                Vector3D offAxis = rel - bFwd * along;
                double lateral = offAxis.Length();
                double dist = rel.Length();
                double inwardSpeed = Vector3D.Dot(vel, -bFwd);   // how fast we're closing along the axis

                // CENTER-FIRST: whenever we're off the axis (at ANY distance) slide onto it BEFORE backing
                // in. This kills both stalls seen in the logs — a big offset can't persist (the 5.4 m /
                // lat 3 mid-air freeze) and the housings can't jam off-center before the faces meet (the
                // lat 0.3 bump-fail). Both motions are CARDINAL for the aligned drone, so ThrustAlong
                // always finds a thruster to fire (a diagonal toward bPos fired none → the freeze):
                //  • off-axis → push along -offAxis (sideways/up); minAlign 0.5 fires the contributing
                //              faces even for a diagonal miss; speed scales with lateral so corrections go
                //              MINUTE near the axis (no overshoot, fine enough for the magnet).
                //  • centered → push along -bFwd straight down the axis, gentle as it nears; magnet mates.
                string leg;
                if (lateral > DockLateralTol)
                {
                    double shiftSpeed = System.Math.Min(CrawlSpeedNear, System.Math.Max(CenterMinSpeed, lateral * CenterGain));
                    _bore.ThrustAlong(grid, -offAxis / lateral, shiftSpeed, 1.0f, 0.5f);
                    leg = "shift";
                    _bumpStart = default(DateTime);                                       // still centering, not a lock-fail
                }
                else
                {
                    double rs = dist > CrawlFarDist ? CrawlSpeedFar : dist > CrawlMidDist ? CrawlSpeedMid : CrawlSpeedNear;
                    _bore.ThrustAlong(grid, -bFwd, rs, 1.0f);
                    leg = "reverse";

                    // BUMP/LOCK-FAIL: centered AND pressed against the connector (inward speed dead) but it
                    // still won't lock — give it BumpFailSecs of occasional Connect() attempts, then retry.
                    if (dist <= BumpDist && inwardSpeed < ContactSpeedEps)
                    {
                        if (_bumpStart == default(DateTime)) _bumpStart = DateTime.UtcNow;
                        else if ((DateTime.UtcNow - _bumpStart).TotalSeconds > BumpFailSecs)
                        { DockFallback(colony, m, grid, "dock bump failed (connector won't lock)"); return; }
                    }
                    else _bumpStart = default(DateTime);
                }
                DockTelemetry(m, leg, dist, vel.Length(), Vector3D.Dot(dFwd, -bFwd), inwardSpeed, lateral);

                if (dist < _legMinDist) _legMinDist = dist;                               // track closest approach
                if (dist > _legMinDist + ReverseOvershoot)                                // TARGET OVERSHOOT — flew past
                { DockFallback(colony, m, grid, "dock reverse overshoot"); return; }
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
            _dockSpeedScale = System.Math.Max(0.25, _dockSpeedScale * 0.6); // soft failsafe: retry the dock slower
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: dock retry {1}/{2} — {3}, soft reset (speed x{4:F2})", m.Id, _retries, MaxRetries, reason, _dockSpeedScale));
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
            string latStr = lateral >= 0 ? string.Format(" lat={0:F2}", lateral) : "";
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
