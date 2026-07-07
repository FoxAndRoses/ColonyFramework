using System;
using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.Utils;
using VRageMath;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

namespace ColonyFramework
{
    // FLIGHT.md F3 — corridor planning: sample the heightmap along the straight line and raise
    // waypoints where terrain intrudes on the cruise altitude, so the drone climbs BEFORE a ridge
    // instead of reacting at it (steering remains the reactive safety net for what a plan can't
    // see: other grids, overhangs, changes since planning). Pure heightmap math — no graph search;
    // a planet is a heightfield, so a clamped polyline IS the path.
    public static class FlightCorridor
    {
        private const double SampleStep = 100.0;  // m between terrain samples
        private const double KeepThreshold = 8.0; // a sample becomes a waypoint if raised > this above the line
        private const int MaxWaypoints = 10;

        public static List<Vector3D> Plan(Vector3D start, Vector3D goal, double cruiseAgl)
        {
            var route = new List<Vector3D>();
            var planet = Sandbox.Game.Entities.MyGamePruningStructure.GetClosestPlanet(start);
            double dist = Vector3D.Distance(start, goal);
            if (planet == null || dist < SampleStep * 1.5) { route.Add(goal); return route; } // space/short: direct

            Vector3D planetCenter = planet.PositionComp.GetPosition();
            int n = (int)Math.Min(60, dist / SampleStep);
            for (int i = 1; i < n; i++)
            {
                double t = (double)i / n;
                Vector3D onLine = Vector3D.Lerp(start, goal, t);
                Vector3D up = Vector3D.Normalize(onLine - planetCenter);
                Vector3D probe = onLine;
                Vector3D surface = planet.GetClosestSurfacePointGlobal(ref probe);
                double clearance = Vector3D.Dot(onLine - surface, up);
                if (clearance < cruiseAgl)
                {
                    Vector3D raised = surface + up * cruiseAgl;
                    // Keep only meaningful raises, spaced out — a handful of ridge-toppers, not a picket fence.
                    if (Vector3D.Dot(raised - onLine, up) > KeepThreshold
                        && (route.Count == 0 || Vector3D.Distance(route[route.Count - 1], raised) > SampleStep * 1.5))
                        route.Add(raised);
                }
            }
            // Cap the count (evenly thinned) and always end exactly at the goal.
            if (route.Count > MaxWaypoints)
            {
                var thinned = new List<Vector3D>(MaxWaypoints);
                double step = (double)route.Count / MaxWaypoints;
                for (int i = 0; i < MaxWaypoints; i++) thinned.Add(route[(int)(i * step)]);
                route = thinned;
            }
            route.Add(goal);
            return route;
        }
    }

    // F1/F3 acceptance harness (FLIGHT.md §7): `/colony flighttest [dist]` — pad → point → pad on
    // the flight core. Short legs use MoveTo; ≥200 m legs use a planned CORRIDOR (Transit), so one
    // command demonstrates the velocity loop, steering, and terrain-ahead planning together.
    public class FlightTest
    {
        private readonly FlightController _fc = new FlightController();
        private readonly OrbitNav _terrain = new OrbitNav();
        private int _leg; // 0=out 1=holdA 2=back 3=holdB 4=done
        private Vector3D _start, _far;
        private DateTime _holdStart, _lastNarrate;
        private double _dist;
        public bool Done { get { return _leg >= 4; } }

        public FlightTest(IMyCubeGrid grid, double dist)
        {
            _dist = Math.Max(50, Math.Min(500, dist));
            _start = grid.GetPosition();
            float interference;
            Vector3D g = MyAPIGateway.Physics.CalculateNaturalGravityAt(_start, out interference);
            Vector3D up = g.LengthSquared() > 0.01 ? -Vector3D.Normalize(g) : Vector3D.Up;
            Vector3D lateral = Vector3D.Normalize(Vector3D.CalculatePerpendicularVector(up));
            _far = _terrain.ClampAboveTerrain(_start + lateral * _dist + up * 10, up, _fc.Profile.MinAgl);
            _start = _start + up * 5; // return point slightly above the pad, not inside it
            DroneUtil.PrepareForFlight(grid);
            Go(grid, _far);
            Log(grid, string.Format("leg 1 — out {0:F0} m{1}", _dist, _dist >= 200 ? " (corridor)" : ""));
        }

        private void Go(IMyCubeGrid grid, Vector3D goal)
        {
            if (_dist >= 200)
                _fc.Transit(grid, FlightCorridor.Plan(grid.GetPosition(), goal, _fc.Profile.CruiseAgl));
            else
                _fc.MoveTo(grid, goal);
        }

        public void Tick(IMyCubeGrid grid)
        {
            if (Done) return;
            _fc.Tick(grid);

            if (_fc.Status == FlightController.VerbStatus.Failed)
            {
                Log(grid, "FAILED: " + _fc.FailReason);
                _leg = 4;
                return;
            }

            if ((DateTime.UtcNow - _lastNarrate).TotalSeconds >= 3 && grid.Physics != null && _leg % 2 == 0)
            {
                _lastNarrate = DateTime.UtcNow;
                Vector3D goal = _leg == 0 ? _far : _start;
                Log(grid, string.Format("dist {0:F0} m, v {1:F1} m/s",
                    Vector3D.Distance(grid.GetPosition(), goal), grid.Physics.LinearVelocity.Length()));
            }

            switch (_leg)
            {
                case 0:
                    if (_fc.Status == FlightController.VerbStatus.Done)
                    { _leg = 1; _holdStart = DateTime.UtcNow; Log(grid, "leg 1 arrived — holding 3 s"); }
                    break;
                case 1:
                    if ((DateTime.UtcNow - _holdStart).TotalSeconds >= 3)
                    { _leg = 2; Go(grid, _start); Log(grid, "leg 2 — returning"); }
                    break;
                case 2:
                    if (_fc.Status == FlightController.VerbStatus.Done)
                    { _leg = 3; _holdStart = DateTime.UtcNow; Log(grid, "leg 2 arrived — settling"); }
                    break;
                case 3:
                    if ((DateTime.UtcNow - _holdStart).TotalSeconds >= 2)
                    {
                        _fc.Release(grid);
                        _leg = 4;
                        Log(grid, "COMPLETE — smooth loop, releasing to dampeners");
                        if (!MyAPIGateway.Utilities.IsDedicated)
                            MyAPIGateway.Utilities.ShowMessage("Colony", "flight test complete");
                    }
                    break;
            }
        }

        public void Abort(IMyCubeGrid grid) { _fc.Release(grid); _leg = 4; }

        private static void Log(IMyCubeGrid grid, string msg)
        {
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] flighttest '{0}': {1}", grid.DisplayName, msg));
        }
    }
}
