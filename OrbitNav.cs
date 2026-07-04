using System;
using Sandbox.Game.Entities;
using VRageMath;

namespace ColonyFramework
{
    // Fluid obstacle-relative navigation without pathfinding: the obstacle (e.g. a ship under
    // construction) is a KEEP-OUT SPHERE, and the drone either flies straight to its goal (when the
    // segment misses the sphere) or SLIDES ALONG the sphere toward it — one geometric arc step per
    // reroute tick, recomputed from the current position only. No stored path, no graph, no search;
    // the emergent flight is a smooth sweep around the hull that hands off to a direct leg the
    // moment line-of-sight opens. Every point is clamped above the terrain (heightmap), so arcs
    // crest over ground the way survey waypoints do.
    public class OrbitNav
    {
        private const double ArcStepDeg = 35.0;   // how far around the sphere each re-issued waypoint advances
        private const double OrbitSlack = 30.0;   // fly the arc up to this far outside the keep-out radius

        private MyPlanet _planet;
        private DateTime _lastPlanetFind;

        // Next waypoint toward 'goal' honoring the keep-out sphere. Returns true when the step is
        // DIRECT (waypoint == clamped goal — line of sight is open); false while orbiting.
        public bool NextStep(Vector3D pos, Vector3D goal, BoundingSphereD keepOut, Vector3D up, double minAgl,
                             out Vector3D waypoint)
        {
            if (!SegmentHitsSphere(pos, goal, keepOut.Center, keepOut.Radius))
            {
                waypoint = ClampAboveTerrain(goal, up, minAgl);
                return true;
            }

            // Slide along the sphere in the (pos, centre, goal) plane, one arc step toward the goal.
            Vector3D a = pos - keepOut.Center;
            double ra = a.Length();
            if (ra < 1.0) { waypoint = ClampAboveTerrain(goal, up, minAgl); return true; } // degenerate: at centre
            Vector3D an = a / ra;
            Vector3D bn = Vector3D.Normalize(goal - keepOut.Center);

            Vector3D axis = Vector3D.Cross(an, bn);
            if (axis.LengthSquared() < 1e-6) axis = Vector3D.CalculatePerpendicularVector(an); // antipodal: any great circle
            axis = Vector3D.Normalize(axis);

            double orbitR = MathHelper.Clamp(ra, keepOut.Radius, keepOut.Radius + OrbitSlack);
            Vector3D dir = Vector3D.Transform(an, QuaternionD.CreateFromAxisAngle(axis, ArcStepDeg * Math.PI / 180.0));
            waypoint = ClampAboveTerrain(keepOut.Center + dir * orbitR, up, minAgl);
            return false;
        }

        // Never below the planet surface + minAgl at that spot (no-op with no planet, e.g. space).
        public Vector3D ClampAboveTerrain(Vector3D point, Vector3D up, double minAgl)
        {
            if (_planet == null || (DateTime.UtcNow - _lastPlanetFind).TotalSeconds > 60)
            {
                _lastPlanetFind = DateTime.UtcNow;
                _planet = MyGamePruningStructure.GetClosestPlanet(point);
            }
            if (_planet == null) return point;
            Vector3D surface = _planet.GetClosestSurfacePointGlobal(ref point);
            double clearance = Vector3D.Dot(point - surface, up);
            return clearance >= minAgl ? point : surface + up * minAgl;
        }

        private static bool SegmentHitsSphere(Vector3D p0, Vector3D p1, Vector3D c, double r)
        {
            Vector3D d = p1 - p0;
            double len = d.Length();
            if (len < 1e-3) return false;
            Vector3D dn = d / len;
            double t = MathHelper.Clamp(Vector3D.Dot(c - p0, dn), 0.0, len);
            return Vector3D.DistanceSquared(p0 + dn * t, c) < r * r;
        }
    }
}
