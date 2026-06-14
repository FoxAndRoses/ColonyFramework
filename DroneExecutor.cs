using System;
using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.Utils;
using VRageMath;
using MyPlanetElevation = Sandbox.ModAPI.Ingame.MyPlanetElevation;
using FlightMode = Sandbox.ModAPI.Ingame.FlightMode;
using IMyCubeGrid  = VRage.Game.ModAPI.IMyCubeGrid;
using IMyCubeBlock = VRage.Game.ModAPI.IMyCubeBlock;

namespace ColonyFramework
{
    // Capability-aware mining. Phases:
    //   Commission(0): at base, power self-test; pass → release + transit, fail → idle.
    //   Transit(1):    RC autopilot to the deposit standoff.
    //   StartBore(2):  drill/RC alignment build-check.
    //   Mining(3):     contact surface-skim across a 9×9 grid (this drone cannot deep-bore).
    //   Retreat(4):    climb to standoff, complete the mission.
    public class DroneExecutor
    {
        private const int PhaseCommission = 0;
        private const int PhaseTransit    = 1;
        private const int PhaseStartBore  = 2;
        private const int PhaseMining     = 3;
        private const int PhaseRetreat    = 4;

        private const double ArriveDistance     = 15.0;
        private const double BoreApproachSpeed  = 6.0;   // fast descent, high altitude
        private const double BoreMediumSpeed    = 1.0;   // controlled, near surface
        private const double BoreDrillSpeed     = 0.15;  // crawl, final approach to contact
        private const double AltFast            = 25.0;  // m above surface → fast
        private const double AltMedium          = 5.0;   // m above surface → medium (below → crawl)
        private const double SpaceFastDist      = 100.0; // fallback (no planet): m from deposit → fast

        private const double CommissionSpikeSecs   = 1.0;   // spike all consumers this long before reading power
        private const double MinRuntimeMinutes     = 10.0;  // refuse dispatch below this estimated runtime
        private const double OrientReadyDot        = 0.985; // nose-down considered settled (~10°)
        private const double UprightMinDot         = 0.5;   // flip guard (>60° off down) — only after Oriented
        private const double ContactSpeedEps       = 0.1;   // m/s: descending slower than this = stalled on surface
        private const double MinDescendSecs        = 1.5;   // debounce before trusting a stall
        private const double SurfacePenetrationCap = -1.0;  // alt(m) below surface that forces contact (safety)
        private const double SkimDwellSecs         = 3.0;   // seconds drilling in place per cell

        private const double SafeHeight         = 15.0;  // m above surface to climb to before sliding to next cell
        private const int    GridN              = 9;     // 9×9 = 81 cells
        private const double GridSpacing        = 3.0;   // m between cells
        private const double RepoSpeed          = 3.0;   // m/s lateral slide between cells
        private const double RepoTolerance      = 1.5;   // m horizontal arrival tolerance
        private const double RetreatAscendSpeed = 3.0;

        private const double CargoThreshold     = 0.80;
        private const double LowPowerThreshold  = 0.20;
        private const double BoreTimeoutSeconds = 7200; // testing ceiling; a full 9×9 pass is slow
        private const double AlignmentMinDot    = 0.7;
        private const double StuckDistance      = 1.0;
        private const double StuckSeconds       = 20;
        private const float  TransitSpeedLimit  = 25f;

        // Mining sub-states.
        private const int SkimDescend    = 0; // orient → descend → detect surface contact
        private const int SkimDwell      = 1; // hold, drills on, carve the surface divot
        private const int SkimRetreat    = 2; // climb to SafeHeight
        private const int SkimReposition = 3; // lateral slide to the next cell

        private readonly Dictionary<long, DateTime> _boreStart    = new Dictionary<long, DateTime>();
        private readonly Dictionary<long, Vector3D> _progressPos  = new Dictionary<long, Vector3D>();
        private readonly Dictionary<long, DateTime> _progressTime = new Dictionary<long, DateTime>();
        private readonly Dictionary<long, BoreJob>  _job          = new Dictionary<long, BoreJob>();
        private readonly BoreController _bore = new BoreController();

        // Ephemeral per-mission state (not persisted).
        private sealed class BoreJob
        {
            public int SubState;
            public bool Oriented;             // achieved stable nose-down at least once (gates the flip guard)
            public DateTime SubStart;         // set on every sub-state change (dwell / debounce timing)
            public DateTime CommissionStart;
            public bool CommissionStarted;
            public bool GridSet;
            public Vector3D GridOrigin, U, V; // grid anchor + horizontal basis
            public int CellIndex;
            public Vector3D CellContact;      // surface-stall point of the current cell
        }

        public void Tick(Colony colony)
        {
            var ms = colony.Missions.Missions;
            for (int i = 0; i < ms.Count; i++)
            {
                var m = ms[i];
                if (m.Status != MissionStatus.InProgress || m.Type != MissionType.Mine) continue;

                var deposit = colony.Deposits.GetById(m.TargetDepositId);
                var grid = MyAPIGateway.Entities.GetEntityById(m.AssignedAssetId) as IMyCubeGrid;
                if (deposit == null || grid == null) continue;

                try
                {
                    switch (m.Phase)
                    {
                        case PhaseCommission: TickCommission(colony, m, deposit, grid); break;
                        case PhaseTransit:    TickTransit(m, deposit, grid); break;
                        case PhaseStartBore:  StartBore(colony, m, deposit, grid); break;
                        case PhaseMining:     TickMining(m, deposit, grid); break;
                        case PhaseRetreat:    TickRetreating(colony, m, deposit, grid); break;
                    }
                }
                catch (Exception e)
                {
                    MyLog.Default.WriteLineAndConsole("[ColonyFramework] Executor error mission " + m.Id + ": " + e.Message);
                }
            }
        }

        // ── Commissioning: spike consumers, estimate runtime, refuse if under reserve ──────
        private void TickCommission(Colony colony, Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            BoreJob job;
            if (!_job.TryGetValue(m.Id, out job)) { job = new BoreJob(); _job[m.Id] = job; }

            // Reactor-powered drones provide continuous power; skip the battery runtime test.
            if (HasInfinitePower(grid))
            {
                MyLog.Default.WriteLineAndConsole(string.Format(
                    "[ColonyFramework] Mission {0}: commissioned (reactor power), dispatching", m.Id));
                EngageTransit(grid, deposit);
                m.Phase = PhaseTransit;
                return;
            }

            if (!job.CommissionStarted)
            {
                job.CommissionStarted = true;
                job.CommissionStart = DateTime.UtcNow;
                SetSpike(grid, true); // all thrusters full + drills on (gear still locked → no/low movement)
                return;
            }

            if ((DateTime.UtcNow - job.CommissionStart).TotalSeconds < CommissionSpikeSecs) return;

            double stored, output;
            int batteries = SumBatteryPower(grid, out stored, out output);
            SetSpike(grid, false); // clear the spike

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
                FailMission(colony, m, string.Format(
                    "insufficient runtime ({0:F1} min < {1:F0}) — staying idle", runtimeMin, MinRuntimeMinutes));
            }
        }

        private void EngageTransit(IMyCubeGrid grid, DepositRecord deposit)
        {
            ReleaseGrid(grid);
            var rc = FindRc(grid);
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

            var rc = FindRc(grid);
            if (rc != null) rc.SetAutoPilotEnabled(false);
            m.Phase = PhaseStartBore;
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: '{1}' arrived at deposit {2}, starting bore",
                m.Id, grid.DisplayName, m.TargetDepositId));
        }

        private void StartBore(Colony colony, Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            var rc = FindRc(grid);
            var drills = FindDrills(grid);
            if (rc == null || drills.Count == 0)
            {
                FailMission(colony, m, rc == null ? "no remote control" : "no drills");
                return;
            }

            double dot = Vector3D.Dot(drills[0].WorldMatrix.Forward, rc.WorldMatrix.Forward);
            if (dot < AlignmentMinDot)
            {
                FailMission(colony, m, string.Format("drills not aligned with RC forward (dot {0:F2})", dot));
                return;
            }

            rc.SetAutoPilotEnabled(false);
            _boreStart[m.Id] = DateTime.UtcNow;
            m.Phase = PhaseMining; // drills enabled per-cell during SkimDwell to conserve power
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: surface-skim grid at deposit {1}, {2} drills",
                m.Id, m.TargetDepositId, drills.Count));
        }

        // ── Mining: contact surface-skim across a serpentine grid ─────────────────────────
        private void TickMining(Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            var drills = FindDrills(grid);
            if (drills.Count == 0) { BeginRetreat(m, deposit, grid, "drills lost"); return; }

            if (!HasInfinitePower(grid))
            {
                double charge = MinBatteryCharge(grid);
                if (charge < LowPowerThreshold)
                {
                    BeginRetreat(m, deposit, grid, string.Format("low power ({0:N0}%)", charge * 100));
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

            BoreJob job;
            if (!_job.TryGetValue(m.Id, out job)) { job = new BoreJob(); _job[m.Id] = job; }

            // Flip guard — only after the drone has been upright once (so the initial
            // nose-down pitch from level transit never false-triggers it).
            if (job.Oriented && Vector3D.Dot(drillFwd, downDir) < UprightMinDot)
            {
                BeginRetreat(m, deposit, grid, "orientation guard (tilt > 60° off vertical)");
                return;
            }

            double altitude;
            bool gotAlt = TryGetAltitude(grid, out altitude);

            if (job.SubState == SkimDescend)
            {
                if (!job.Oriented)
                {
                    _bore.Drive(grid, drillFwd, downDir, 0); // pitch to nose-down + hover, no descent
                    if (Vector3D.Dot(drillFwd, downDir) > OrientReadyDot)
                    {
                        job.Oriented = true;
                        job.SubStart = DateTime.UtcNow;
                    }
                }
                else
                {
                    double sp = gotAlt
                        ? (altitude > AltFast ? BoreApproachSpeed : altitude > AltMedium ? BoreMediumSpeed : BoreDrillSpeed)
                        : (Vector3D.Distance(pos, deposit.Position) > SpaceFastDist ? BoreApproachSpeed : BoreDrillSpeed);
                    _bore.Drive(grid, drillFwd, downDir, sp);

                    double downSpeed = grid.Physics != null ? Vector3D.Dot(grid.Physics.LinearVelocity, downDir) : 0;
                    bool descendedLongEnough = (DateTime.UtcNow - job.SubStart).TotalSeconds > MinDescendSecs;
                    bool stalled = downSpeed < ContactSpeedEps;
                    bool tooDeep = gotAlt && altitude < SurfacePenetrationCap;
                    if (descendedLongEnough && (stalled || tooDeep))
                    {
                        if (!job.GridSet)
                        {
                            job.GridOrigin = pos;
                            job.U = Vector3D.Normalize(Vector3D.CalculatePerpendicularVector(downDir));
                            job.V = Vector3D.Normalize(Vector3D.Cross(downDir, job.U));
                            job.CellIndex = 0;
                            job.GridSet = true;
                        }
                        job.CellContact = pos;
                        for (int i = 0; i < drills.Count; i++) drills[i].Enabled = true;
                        job.SubState = SkimDwell;
                        job.SubStart = DateTime.UtcNow;
                        MyLog.Default.WriteLineAndConsole(string.Format(
                            "[ColonyFramework] Mission {0}: cell {1}/{2} skim", m.Id, job.CellIndex + 1, GridN * GridN));
                    }
                }
            }
            else if (job.SubState == SkimDwell)
            {
                _bore.Drive(grid, drillFwd, downDir, 0); // hold position; dampers hold altitude; drills carve
                if ((DateTime.UtcNow - job.SubStart).TotalSeconds > SkimDwellSecs)
                {
                    for (int i = 0; i < drills.Count; i++) drills[i].Enabled = false;
                    job.SubState = SkimRetreat;
                    job.SubStart = DateTime.UtcNow;
                }
            }
            else if (job.SubState == SkimRetreat)
            {
                _bore.Drive(grid, drillFwd, downDir, 0);          // hold nose-down
                ThrustAlong(grid, -downDir, RetreatAscendSpeed);   // climb straight up
                if (gotAlt && altitude >= SafeHeight)
                {
                    job.CellIndex++;
                    if (job.CellIndex >= GridN * GridN)
                    {
                        BeginRetreat(m, deposit, grid, "9x9 grid complete (testing)");
                        return;
                    }
                    job.SubState = SkimReposition;
                    job.SubStart = DateTime.UtcNow;
                }
            }
            else // SkimReposition — slide horizontally at altitude to the next cell
            {
                Vector3D target = CellTarget(job);
                Vector3D toT = target - pos;
                Vector3D horiz = toT - downDir * Vector3D.Dot(toT, downDir);
                if (horiz.Length() < RepoTolerance)
                {
                    job.SubState = SkimDescend; // re-descend into this cell (Oriented stays true)
                    job.SubStart = DateTime.UtcNow;
                }
                else
                {
                    _bore.Drive(grid, drillFwd, downDir, 0);                 // hold nose-down; dampers hold altitude
                    ThrustAlong(grid, Vector3D.Normalize(horiz), RepoSpeed); // lateral slide
                }
            }

            // Stuck watchdog (real-hang safety): reset whenever the drone moves > StuckDistance.
            Vector3D lastPos;
            if (!_progressPos.TryGetValue(m.Id, out lastPos) || Vector3D.Distance(pos, lastPos) > StuckDistance)
            {
                _progressPos[m.Id]  = pos;
                _progressTime[m.Id] = DateTime.UtcNow;
            }
            else if ((DateTime.UtcNow - _progressTime[m.Id]).TotalSeconds > StuckSeconds)
            {
                BeginRetreat(m, deposit, grid, "no progress (stuck)");
                return;
            }

            // Cargo / "reached" early-exits remain disabled while grid-testing; deterministic
            // end is the 81-cell completion in SkimRetreat. Timeout is the ultimate ceiling.
            DateTime start;
            if (_boreStart.TryGetValue(m.Id, out start) && (DateTime.UtcNow - start).TotalSeconds > BoreTimeoutSeconds)
                BeginRetreat(m, deposit, grid, "timeout");
        }

        private void BeginRetreat(Mission m, DepositRecord deposit, IMyCubeGrid grid, string reason)
        {
            var drills = FindDrills(grid);
            for (int i = 0; i < drills.Count; i++) drills[i].Enabled = false;
            _bore.Release(grid); // clear gyro + thrust overrides so retreat thrust takes effect
            _boreStart.Remove(m.Id);
            _progressPos.Remove(m.Id);
            _progressTime.Remove(m.Id);
            _job.Remove(m.Id);
            m.Phase = PhaseRetreat;
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0}: retreating ({1})", m.Id, reason));
        }

        private void TickRetreating(Colony colony, Mission m, DepositRecord deposit, IMyCubeGrid grid)
        {
            Vector3D standoff = NavMath.ComputeStandoff(deposit.Position, grid.GetPosition());
            Vector3D toStandoff = standoff - grid.GetPosition();
            if (toStandoff.Length() > ArriveDistance)
            {
                ThrustAlong(grid, Vector3D.Normalize(toStandoff), RetreatAscendSpeed);
                return;
            }

            _bore.Release(grid);
            double cargo = CargoFill(grid);
            colony.Missions.Complete(m.Id);
            var asset = colony.Assets.GetByEntityId(m.AssignedAssetId);
            if (asset != null) { asset.Status = AssetStatus.Idle; asset.AssignedMissionId = 0; }
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0} complete: deposit {1} mined (cargo {2:N0}%), asset idle",
                m.Id, m.TargetDepositId, cargo * 100));
        }

        public void AbortAll(Colony colony)
        {
            var ms = colony.Missions.Missions;
            for (int i = 0; i < ms.Count; i++)
            {
                var m = ms[i];
                if (m.Status != MissionStatus.Assigned && m.Status != MissionStatus.InProgress) continue;
                var grid = MyAPIGateway.Entities.GetEntityById(m.AssignedAssetId) as IMyCubeGrid;
                if (grid != null)
                {
                    _bore.Release(grid);
                    var rc = FindRc(grid);
                    if (rc != null) rc.SetAutoPilotEnabled(false);
                    var drills = FindDrills(grid);
                    for (int d = 0; d < drills.Count; d++) drills[d].Enabled = false;
                }
                FailMission(colony, m, "aborted by command");
            }
        }

        public void RecallAll(Colony colony)
        {
            var ms = colony.Missions.Missions;
            for (int i = 0; i < ms.Count; i++)
            {
                var m = ms[i];
                if (m.Status != MissionStatus.InProgress || m.Phase >= PhaseRetreat) continue;
                var deposit = colony.Deposits.GetById(m.TargetDepositId);
                var grid = MyAPIGateway.Entities.GetEntityById(m.AssignedAssetId) as IMyCubeGrid;
                if (deposit == null || grid == null) continue;
                var rc = FindRc(grid);
                if (rc != null) rc.SetAutoPilotEnabled(false);
                BeginRetreat(m, deposit, grid, "recalled");
            }
        }

        // Clears any stale gyro/thrust overrides on a grid (e.g. baked into a spawned/pasted
        // blueprint). Called on registration so a claimed drone won't thrust on its own.
        public void ReleaseControls(IMyCubeGrid grid)
        {
            if (grid != null) _bore.Release(grid);
        }

        private void ThrustAlong(IMyCubeGrid grid, Vector3D dir, double targetSpeed)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null || grid.Physics == null) return;
            double speedAlong = Vector3D.Dot(grid.Physics.LinearVelocity, dir);
            bool push = speedAlong < targetSpeed;
            var thrusters = new List<IMyThrust>();
            ts.GetBlocksOfType(thrusters);
            for (int i = 0; i < thrusters.Count; i++)
            {
                var t = thrusters[i];
                double d = Vector3D.Dot(t.WorldMatrix.Backward, dir);
                t.ThrustOverridePercentage = (push && d > 0.9) ? 0.5f : 0f;
            }
        }

        private void FailMission(Colony colony, Mission m, string reason)
        {
            colony.Missions.Fail(m.Id);
            var asset = colony.Assets.GetByEntityId(m.AssignedAssetId);
            if (asset != null) { asset.Status = AssetStatus.Idle; asset.AssignedMissionId = 0; }
            _boreStart.Remove(m.Id);
            _progressPos.Remove(m.Id);
            _progressTime.Remove(m.Id);
            _job.Remove(m.Id);
            var grid = MyAPIGateway.Entities.GetEntityById(m.AssignedAssetId) as IMyCubeGrid;
            if (grid != null)
            {
                _bore.Release(grid);
                var drills = FindDrills(grid);
                for (int i = 0; i < drills.Count; i++) drills[i].Enabled = false;
            }
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Mission {0} failed: {1}", m.Id, reason));
        }

        // World position of the current grid cell, serpentine raster (every move is adjacent).
        private static Vector3D CellTarget(BoreJob job)
        {
            int idx = job.CellIndex, row = idx / GridN, col = idx % GridN;
            if ((row & 1) == 1) col = GridN - 1 - col;
            return job.GridOrigin + job.U * (col * GridSpacing) + job.V * (row * GridSpacing);
        }

        private static bool TryGetAltitude(IMyCubeGrid grid, out double altitude)
        {
            altitude = 0;
            var rc = FindRc(grid);
            if (rc == null) return false;
            return rc.TryGetPlanetElevation(MyPlanetElevation.Surface, out altitude);
        }

        private static void SetSpike(IMyCubeGrid grid, bool on)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return;
            var thrusters = new List<IMyThrust>();
            ts.GetBlocksOfType(thrusters);
            for (int i = 0; i < thrusters.Count; i++) thrusters[i].ThrustOverridePercentage = on ? 1f : 0f;
            var drills = new List<IMyShipDrill>();
            ts.GetBlocksOfType(drills);
            for (int i = 0; i < drills.Count; i++) drills[i].Enabled = on;
        }

        // Sums battery stored energy (MWh) and current output (MW); returns battery count.
        private static int SumBatteryPower(IMyCubeGrid grid, out double stored, out double output)
        {
            stored = 0; output = 0;
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return 0;
            var bats = new List<IMyBatteryBlock>();
            ts.GetBlocksOfType(bats);
            for (int i = 0; i < bats.Count; i++)
            {
                stored += bats[i].CurrentStoredPower;
                output += bats[i].CurrentOutput;
            }
            return bats.Count;
        }

        private void ReleaseGrid(IMyCubeGrid grid)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return;

            var gears = new List<SpaceEngineers.Game.ModAPI.IMyLandingGear>();
            ts.GetBlocksOfType(gears);
            for (int i = 0; i < gears.Count; i++)
            {
                gears[i].AutoLock = false;
                gears[i].Unlock();
            }

            var connectors = new List<IMyShipConnector>();
            ts.GetBlocksOfType(connectors);
            for (int i = 0; i < connectors.Count; i++) connectors[i].Disconnect();
        }

        private static IMyRemoteControl FindRc(IMyCubeGrid grid)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return null;
            var rcs = new List<IMyRemoteControl>();
            ts.GetBlocksOfType(rcs);
            return rcs.Count > 0 ? rcs[0] : null;
        }

        private static List<IMyShipDrill> FindDrills(IMyCubeGrid grid)
        {
            var drills = new List<IMyShipDrill>();
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts != null) ts.GetBlocksOfType(drills);
            return drills;
        }

        private static double CargoFill(IMyCubeGrid grid)
        {
            var blocks = new List<VRage.Game.ModAPI.IMySlimBlock>();
            grid.GetBlocks(blocks);
            double cur = 0, max = 0;
            for (int b = 0; b < blocks.Count; b++)
            {
                var fat = blocks[b].FatBlock as IMyCubeBlock;
                if (fat == null || !fat.HasInventory) continue;
                for (int i = 0; i < fat.InventoryCount; i++)
                {
                    var inv = fat.GetInventory(i);
                    if (inv == null) continue;
                    cur += (double)inv.CurrentVolume;
                    max += (double)inv.MaxVolume;
                }
            }
            return max > 0 ? cur / max : 0;
        }

        private static bool HasInfinitePower(IMyCubeGrid grid)
        {
            var blocks = new List<VRage.Game.ModAPI.IMySlimBlock>();
            grid.GetBlocks(blocks);
            for (int b = 0; b < blocks.Count; b++)
            {
                var reactor = blocks[b].FatBlock as IMyReactor;
                if (reactor != null && reactor.IsWorking) return true;
            }
            return false;
        }

        private static double MinBatteryCharge(IMyCubeGrid grid)
        {
            var blocks = new List<VRage.Game.ModAPI.IMySlimBlock>();
            grid.GetBlocks(blocks);
            double min = 1.0;
            bool found = false;
            for (int b = 0; b < blocks.Count; b++)
            {
                var bat = blocks[b].FatBlock as IMyBatteryBlock;
                if (bat == null) continue;
                double charge = bat.MaxStoredPower > 0
                    ? (double)bat.CurrentStoredPower / (double)bat.MaxStoredPower : 1.0;
                if (charge < min) min = charge;
                found = true;
            }
            return found ? min : 1.0; // no batteries = external power, don't recall
        }
    }
}
