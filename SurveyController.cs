using System;
using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using VRage.Utils;
using VRageMath;
using FlightMode = Sandbox.ModAPI.Ingame.FlightMode;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

namespace ColonyFramework
{
    // Per-mission brain for an ore SURVEY: fly the next segment of an expanding ring around the
    // colony at LOW altitude (terrain-following waypoints), running one bounded voxel scan per
    // point — the drone physically finds ore the colony doesn't know about. The ring cursor
    // (radius + angle) persists in ColonyState, so successive surveys expand coverage outward
    // instead of rescanning. Dispatched only on demand (production short an unknown ore) or when
    // the colony has no deposits at all — never as a continuous patrol.
    public class SurveyController
    {
        private const int PhaseCommission = 0;
        private const int PhaseTransit    = 1; // to the first ring point
        private const int PhaseSurvey     = 2; // point -> scan -> next point
        private const int PhaseReturn     = 3;

        private const double FirstRing     = 300.0;  // radius of the first survey ring around the core
        private const double RingStep      = 250.0;  // ring growth per completed lap (~scan diameter w/ overlap)
        private const double PointSpacing  = 200.0;  // metres along the arc between scan points
        private const int    MaxWaypoints  = 12;     // scan points per mission before heading home
        private const double SurveyAgl     = 50.0;   // LOW pass: 150 m scan sphere reaches ~100 m underground
        private const double SurveyTerrainClearance = 25.0; // avoidance floor for the low legs (default 40 would fight SurveyAgl)
        private const float  SurveySpeed   = 30f;
        private const double ArriveTol     = 8.0;
        private const double ScanRadius    = 150.0;  // same bounded LOD2 scan the detectors use
        private const int    ScanLod       = 2;
        private const double LegTimeoutSecs = 120.0;
        private const double LegStuckSecs  = 20.0;
        private const double LegProgressEps = 1.0;
        private const int    MaxRetries    = 3;
        private const double LowPowerThreshold = 0.20;
        private const double ArriveDistance = 12.0;

        private readonly OreScanner _scanner = new OreScanner();
        private readonly NavState _nav = new NavState();
        private readonly AvoidanceProbe _avoid = new AvoidanceProbe();
        private readonly BoreController _fly = new BoreController(); // Release only

        private bool _initialized;
        private int _retries;
        private int _pointsDone;
        private Vector3D _ringU, _ringV; private bool _basisSet;
        private Vector3D _waypoint; private bool _hasWaypoint;
        private DateTime _lastReroute; // throttle for avoidance route re-issues (same 3 s as the miner)
        private DateTime _legStart, _legProgressTime;
        private double _legMinDist = double.MaxValue;
        private MyPlanet _planet; private DateTime _lastPlanetFind;

        public void Advance(Colony colony, Mission m, IMyCubeGrid grid)
        {
            var rc = DroneUtil.FindRc(grid);
            if (rc == null) { Fail(colony, m, grid, "no remote control"); return; }
            _nav.Refresh(grid, rc, DroneUtil.FindConnector(grid));

            if (!_initialized)
            {
                _initialized = true;
                if (m.Phase != PhaseCommission) { BeginReturn(colony, m, grid); return; } // reload mid-survey: go home, cursor is saved
            }

            var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as VRage.Game.ModAPI.IMyCubeBlock;
            if (core == null) { Complete(colony, m, grid, "no core"); return; }

            // Low power: head home; the cursor is persisted per scanned point, nothing is lost.
            if (m.Phase != PhaseReturn && !DroneUtil.HasInfinitePower(grid))
            {
                double charge = DroneUtil.MinBatteryCharge(grid);
                if (charge < LowPowerThreshold)
                {
                    if (!MyAPIGateway.Utilities.IsDedicated)
                        MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                            "Survey drone low power ({0:N0}%), returning", charge * 100));
                    BeginReturn(colony, m, grid);
                    return;
                }
            }

            switch (m.Phase)
            {
                case PhaseCommission: TickCommission(colony, m, grid, core); break;
                case PhaseTransit:
                case PhaseSurvey:     TickSurvey(colony, m, grid, core); break;
                case PhaseReturn:     TickReturn(colony, m, grid, core); break;
            }
        }

        private void TickCommission(Colony colony, Mission m, IMyCubeGrid grid, VRage.Game.ModAPI.IMyCubeBlock core)
        {
            if (DroneUtil.FindOreDetector(grid) == null) { Fail(colony, m, grid, "no working ore detector"); return; }
            DroneUtil.ReleaseGrid(grid);
            if (colony.State.SurveyedRadius <= 0) colony.State.SurveyedRadius = FirstRing;
            Log(m, string.Format("survey start{0} — ring {1:F0} m at {2:F0}°",
                m.TargetOre != null ? " (hunting " + m.TargetOre + ")" : "",
                colony.State.SurveyedRadius, colony.State.SurveyedAngleDeg));
            m.Phase = PhaseTransit;
            NextWaypoint(colony, m, grid, core);
        }

        // Transit-to-first-point and the point loop share the same movement logic.
        private void TickSurvey(Colony colony, Mission m, IMyCubeGrid grid, VRage.Game.ModAPI.IMyCubeBlock core)
        {
            if (!_hasWaypoint) { NextWaypoint(colony, m, grid, core); return; }

            double dist = Vector3D.Distance(grid.GetPosition(), _waypoint);
            if (dist > ArriveTol || _nav.Speed > 1.0)
            {
                var rc = DroneUtil.FindRc(grid);
                if (rc != null && !rc.DampenersOverride) rc.DampenersOverride = true;
                Vector3D via; string obstacle;
                if (_avoid.TryGetDetour(_nav, grid, _waypoint, out via, out obstacle, SurveyTerrainClearance)
                    && (DateTime.UtcNow - _lastReroute).TotalSeconds >= 3.0)
                {
                    _lastReroute = DateTime.UtcNow;
                    FlyTo(grid, _waypoint, SurveySpeed, "survey point", via);
                    Log(m, string.Format("deflected around obstacle {0}", obstacle));
                    return;
                }
                string fail = LegOk(dist, LegTimeoutSecs, "survey leg");
                if (fail != null) RetryOrFail(colony, m, grid, fail);
                return;
            }

            // Arrived + settled: one bounded scan at this point, advance the persistent cursor.
            var rcs = DroneUtil.FindRc(grid);
            if (rcs != null) rcs.SetAutoPilotEnabled(false);
            m.Phase = PhaseSurvey;
            _retries = 0;

            long tick = MyAPIGateway.Session.GameDateTime.Ticks;
            int cells = _scanner.Scan(colony.Deposits, grid.GetPosition(), ScanRadius, ScanLod, grid.EntityId, tick);
            _pointsDone++;

            double stepDeg = 360.0 * PointSpacing / (2.0 * Math.PI * colony.State.SurveyedRadius);
            colony.State.SurveyedAngleDeg += stepDeg;
            if (colony.State.SurveyedAngleDeg >= 360.0)
            {
                colony.State.SurveyedAngleDeg = 0.0;
                colony.State.SurveyedRadius += RingStep;
            }

            Log(m, string.Format("survey: point {0}/{1} scanned — {2} ore cells (deposits: {3})",
                _pointsDone, MaxWaypoints, cells, colony.Deposits.Deposits.Count));

            // Found the ore we were sent for? Done — demand mining takes it from here.
            if (m.TargetOre != null && colony.Deposits.FindNearestUnclaimed(core.GetPosition(), m.TargetOre) != null)
            {
                if (!MyAPIGateway.Utilities.IsDedicated)
                    MyAPIGateway.Utilities.ShowMessage("Colony", "Survey found " + m.TargetOre + " — marking for mining");
                BeginReturn(colony, m, grid);
                return;
            }

            if (_pointsDone >= MaxWaypoints) { BeginReturn(colony, m, grid); return; }
            NextWaypoint(colony, m, grid, core);
        }

        private void NextWaypoint(Colony colony, Mission m, IMyCubeGrid grid, VRage.Game.ModAPI.IMyCubeBlock core)
        {
            Vector3D corePos = core.GetPosition();
            Vector3D up = _nav.Valid ? _nav.GravityUp : Vector3D.Up;
            if (!_basisSet)
            {
                _ringU = Vector3D.Normalize(Vector3D.CalculatePerpendicularVector(up));
                _ringV = Vector3D.Normalize(Vector3D.Cross(up, _ringU));
                _basisSet = true;
            }

            double r = colony.State.SurveyedRadius <= 0 ? FirstRing : colony.State.SurveyedRadius;
            double a = colony.State.SurveyedAngleDeg * Math.PI / 180.0;
            Vector3D point = corePos + _ringU * (Math.Cos(a) * r) + _ringV * (Math.Sin(a) * r);

            // Pin the waypoint to the terrain: surface height at that spot + SurveyAgl (LOW pass).
            RefreshPlanet(corePos);
            if (_planet != null)
            {
                Vector3D surface = _planet.GetClosestSurfacePointGlobal(ref point);
                point = surface + up * SurveyAgl;
            }
            _waypoint = point;
            _hasWaypoint = true;
            FlyTo(grid, _waypoint, SurveySpeed, "survey point", null);
        }

        private void BeginReturn(Colony colony, Mission m, IMyCubeGrid grid)
        {
            _fly.Release(grid);
            m.Phase = PhaseReturn;
            var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId);
            if (core == null) { Complete(colony, m, grid, "no core"); return; }
            Vector3D standoff = core.GetPosition() + (_nav.Valid ? _nav.GravityUp : Vector3D.Up) * 100.0;
            FlyTo(grid, standoff, SurveySpeed, "return", null);
            ResetLeg();
        }

        private void TickReturn(Colony colony, Mission m, IMyCubeGrid grid, VRage.Game.ModAPI.IMyCubeBlock core)
        {
            Vector3D standoff = core.GetPosition() + (_nav.Valid ? _nav.GravityUp : Vector3D.Up) * 100.0;
            double dist = Vector3D.Distance(grid.GetPosition(), standoff);
            if (dist > ArriveDistance)
            {
                var rc = DroneUtil.FindRc(grid);
                if (rc != null && !rc.DampenersOverride) rc.DampenersOverride = true;
                Vector3D via; string obstacle;
                if (_avoid.TryGetDetour(_nav, grid, standoff, out via, out obstacle)
                    && (DateTime.UtcNow - _lastReroute).TotalSeconds >= 3.0)
                { _lastReroute = DateTime.UtcNow; FlyTo(grid, standoff, SurveySpeed, "return", via); return; }
                string fail = LegOk(dist, LegTimeoutSecs, "survey return");
                if (fail != null) RetryOrFail(colony, m, grid, fail);
                return;
            }
            var rc2 = DroneUtil.FindRc(grid);
            if (rc2 != null) rc2.SetAutoPilotEnabled(false);
            Complete(colony, m, grid, string.Format(
                "{0} points scanned; coverage now {1:F0} m @ {2:F0}°",
                _pointsDone, colony.State.SurveyedRadius, colony.State.SurveyedAngleDeg));
        }

        // ── helpers (same discipline as the other controllers) ──────────────────────────────────────
        private void FlyTo(IMyCubeGrid grid, Vector3D target, float speed, string label, Vector3D? via)
        {
            var rc = DroneUtil.FindRc(grid);
            if (rc == null) return;
            rc.DampenersOverride = true;
            rc.ClearWaypoints();
            if (via.HasValue) rc.AddWaypoint(via.Value, "avoid detour");
            rc.AddWaypoint(target, label);
            rc.FlightMode = FlightMode.OneWay;
            rc.SpeedLimit = speed;
            rc.SetAutoPilotEnabled(true);
            ResetLeg();
        }

        private void RefreshPlanet(Vector3D near)
        {
            if (_planet != null && (DateTime.UtcNow - _lastPlanetFind).TotalSeconds < 60) return;
            _lastPlanetFind = DateTime.UtcNow;
            _planet = MyGamePruningStructure.GetClosestPlanet(near);
        }

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
            _fly.Release(grid);
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) { rc.SetAutoPilotEnabled(false); rc.DampenersOverride = true; }
            if (m.Phase == PhaseReturn) BeginReturn(colony, m, grid);
            else { _hasWaypoint = false; }        // re-issue the current ring point fresh
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
            _fly.Release(grid);
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) { rc.SetAutoPilotEnabled(false); rc.DampenersOverride = true; }
        }

        private DateTime _lastLog;
        private void Log(Mission m, string msg)
        {
            if ((DateTime.UtcNow - _lastLog).TotalSeconds < 1.0) return;
            _lastLog = DateTime.UtcNow;
            MyLog.Default.WriteLineAndConsole(string.Format("[ColonyFramework] Survey mission {0}: {1}", m.Id, msg));
        }
    }
}
