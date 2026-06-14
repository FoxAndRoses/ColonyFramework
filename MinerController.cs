using System;
using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.Utils;
using VRageMath;
using FlightMode = Sandbox.ModAPI.Ingame.FlightMode;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

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

        private const int BoreReposition = 0;
        private const int BoreDescend    = 1;
        private const int BoreDrilling   = 2;
        private const int BoreAscend     = 3;

        private const int RetreatAscend = 0; // reverse straight up out of the shaft
        private const int RetreatReturn = 1; // autopilot to the colony-core standoff

        private const float  TransitSpeedLimit  = 25f;
        private const double ArriveDistance     = 15.0;
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
        private const double ReturnClearanceAlt   = 30.0; // climb to this (m above surface) out of the shaft before returning
        private const double ReturnStandoffHeight = 40.0; // hold this far above the colony core

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

        private bool _commissionStarted;
        private DateTime _commissionStart;
        private DateTime _boreStart;
        private Vector3D _progressPos;
        private DateTime _progressTime;
        private bool _progressInit;

        private int _boreSub;
        private int _retreatSub;
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
            try
            {
                switch (m.Phase)
                {
                    case PhaseCommission: TickCommission(colony, m, deposit, grid); break;
                    case PhaseTransit:    TickTransit(m, deposit, grid); break;
                    case PhaseStartBore:  StartBore(colony, m, deposit, grid); break;
                    case PhaseMining:     TickMining(colony, m, deposit, grid); break;
                    case PhaseRetreat:    TickRetreating(colony, m, deposit, grid); break;
                }
            }
            catch (Exception e)
            {
                MyLog.Default.WriteLineAndConsole("[ColonyFramework] Miner error mission " + m.Id + ": " + e.Message);
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
            var rc = DroneUtil.FindRc(grid);
            if (rc == null) return;
            Vector3D standoff = NavMath.ComputeStandoff(deposit.Position, grid.GetPosition());
            rc.ClearWaypoints();
            rc.AddWaypoint(standoff, "Deposit " + deposit.Id + " standoff");
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = TransitSpeedLimit;
            rc.SetAutoPilotEnabled(true);
        }

        private void TickTransit(Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            Vector3D standoff = NavMath.ComputeStandoff(deposit.Position, grid.GetPosition());
            if (Vector3D.Distance(grid.GetPosition(), standoff) > ArriveDistance) return;

            var rc = DroneUtil.FindRc(grid);
            if (rc != null) rc.SetAutoPilotEnabled(false);
            m.Phase = PhaseStartBore;
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: '{1}' arrived at deposit {2}, starting bore",
                m.Id, grid.DisplayName, m.TargetDepositId));
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

                _bore.Release(grid); // hand control to autopilot for the trip home
                MyLog.Default.WriteLineAndConsole(string.Format(
                    "[ColonyFramework] Mission {0}: mining mission complete, returning to base", m.Id));
                if (!MyAPIGateway.Utilities.IsDedicated)
                    MyAPIGateway.Utilities.ShowMessage("Colony", "Mining mission complete, returning to base");

                if (!EngageReturn(colony, grid)) { CompleteMission(colony, m, grid); return; } // no core → finish here
                _retreatSub = RetreatReturn;
                return;
            }

            // RetreatReturn: RC autopilot flies to the colony-core standoff; complete on arrival.
            Vector3D coreStandoff;
            if (!TryCoreStandoff(colony, out coreStandoff)) { CompleteMission(colony, m, grid); return; }
            if (Vector3D.Distance(pos, coreStandoff) > ArriveDistance) return;

            var rc = DroneUtil.FindRc(grid);
            if (rc != null) rc.SetAutoPilotEnabled(false);
            CompleteMission(colony, m, grid);
        }

        private bool EngageReturn(Colony colony, IMyCubeGrid grid)
        {
            Vector3D standoff;
            if (!TryCoreStandoff(colony, out standoff)) return false;
            var rc = DroneUtil.FindRc(grid);
            if (rc == null) return false;
            rc.ClearWaypoints();
            rc.AddWaypoint(standoff, "Colony core standoff");
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = TransitSpeedLimit;
            rc.SetAutoPilotEnabled(true);
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
            double cargo = DroneUtil.CargoFill(grid);
            colony.Missions.Complete(m.Id);
            var asset = colony.Assets.GetByEntityId(m.AssignedAssetId);
            if (asset != null) { asset.Status = AssetStatus.Idle; asset.AssignedMissionId = 0; }
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0} complete: deposit {1} mined (cargo {2:N0}%), asset idle, at base standoff",
                m.Id, m.TargetDepositId, cargo * 100));
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
            }
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0} failed: {1}", m.Id, reason));
        }
    }
}
