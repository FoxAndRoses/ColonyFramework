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
        private readonly FlightController _fc = new FlightController(); // F4.3: all legs on the flight core

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
            if (rc.IsUnderControl) return; // player has the wheel — never fight the pilot
            _nav.Refresh(grid, rc, DroneUtil.FindConnector(grid));

            if (!_initialized)
            {
                _initialized = true;
                if (m.Phase != PhaseCommission) { BeginReturn(colony, m, grid); return; } // reload mid-survey: go home, cursor is saved
            }

            var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as VRage.Game.ModAPI.IMyCubeBlock;
            if (core == null) { Complete(colony, m, grid, "no core"); return; }

            // MISSION.md D4 + Story S-A: distance-aware energy ledger bounds the ring — the drone
            // flies points until the RETURN leg (which grows with the ring) would eat the battery.
            // The persisted cursor makes partial laps lossless.
            if (m.Phase != PhaseReturn)
            {
                string ledger = MissionLedger.ShouldReturn(grid, core.GetPosition(), 0);
                if (ledger != null)
                {
                    if (!MyAPIGateway.Utilities.IsDedicated)
                        MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                            "'{0}' returning — {1}", grid.DisplayName, ledger));
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
            _fc.Tick(grid); // one actuator owner (FLIGHT.md §5.1)
        }

        private void TickCommission(Colony colony, Mission m, IMyCubeGrid grid, VRage.Game.ModAPI.IMyCubeBlock core)
        {
            if (DroneUtil.FindOreDetector(grid) == null) { Fail(colony, m, grid, "no working ore detector"); return; }
            // FLIGHT.md §4 static capability gates (measured hop lands with the survey F4 convert).
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
            DroneUtil.PrepareForFlight(grid);
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

            // F4.3: the flight core owns the leg (steering handles terrain/grids; no detour churn).
            if (_fc.Status == FlightController.VerbStatus.Failed)
            { RetryOrFail(colony, m, grid, "survey leg: " + _fc.FailReason); return; }
            if (_fc.Status != FlightController.VerbStatus.Done) return;

            // Arrived + settled: one bounded scan at this point, advance the persistent cursor.
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
            _fc.MoveTo(grid, _waypoint, SurveySpeed); // low legs: steering's terrain probes ride along
        }

        private void BeginReturn(Colony colony, Mission m, IMyCubeGrid grid)
        {
            m.Phase = PhaseReturn;
            var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId);
            if (core == null) { Complete(colony, m, grid, "no core"); return; }
            Vector3D standoff = core.GetPosition() + (_nav.Valid ? _nav.GravityUp : Vector3D.Up) * 100.0;
            _fc.Transit(grid, FlightCorridor.Plan(grid.GetPosition(), standoff, _fc.Profile.CruiseAgl), SurveySpeed * 2);
        }

        private void TickReturn(Colony colony, Mission m, IMyCubeGrid grid, VRage.Game.ModAPI.IMyCubeBlock core)
        {
            if (_fc.Status == FlightController.VerbStatus.Failed)
            { RetryOrFail(colony, m, grid, "survey return: " + _fc.FailReason); return; }
            if (_fc.Status != FlightController.VerbStatus.Done) return;
            _fc.Release(grid);
            Complete(colony, m, grid, string.Format(
                "{0} points scanned; coverage now {1:F0} m @ {2:F0}°",
                _pointsDone, colony.State.SurveyedRadius, colony.State.SurveyedAngleDeg));
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
            _fc.Release(grid);
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

        // Graceful back-out (/colony recall): head home; the coverage cursor is already persisted.
        public void Recall(Colony colony, Mission m, IMyCubeGrid grid)
        {
            BeginReturn(colony, m, grid);
        }

        private void Cleanup(IMyCubeGrid grid)
        {
            if (grid == null) return;
            DroneUtil.SetBatteriesRecharge(grid, false); // never leave Recharge leaked into idle
            _fc.Release(grid);
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
