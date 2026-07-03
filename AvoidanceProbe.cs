using System;
using System.Collections.Generic;
using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using VRage.Game.ModAPI;
using VRage.ModAPI;
using VRage.Utils;
using VRageMath;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

namespace ColonyFramework
{
    // Reactive obstacle sensing for autopilot legs. Stateless from the flight's point of view: every
    // probe re-derives the answer from the drone's CURRENT position/velocity — no route memory, no
    // stored paths. Three layers, cheapest first, so the common case (empty corridor at cruise
    // altitude) costs a few lookups and NO raycasts:
    //   1. terrain look-ahead — planet heightmap samples ahead of the ship (rays can't hit voxels);
    //   2. grid broad-phase  — ~1 Hz pruning-structure sphere query, AABB vs flight corridor;
    //   3. ray fan           — 3 physics raycasts ONLY while a grid AABB blocks the corridor.
    // On a hit it proposes a single detour point (slide away from the obstacle + up); the caller
    // re-issues the route through it and re-probes next tick. Escalation/give-up stays with the
    // caller's existing leg watchdog (LegOk -> RetryOrFail) — this class only senses and suggests.
    public class AvoidanceProbe
    {
        private const double BroadphaseSecs   = 1.0;   // how often the sphere query / planet lookup refresh
        private const double ProbeRadius      = 300.0; // broad-phase sphere radius around the drone
        private const double CorridorSecs     = 5.0;   // corridor length = speed * this (bounded below/above)
        private const double MinCorridor      = 60.0;  // even when slow, look this far ahead
        private const double CorridorMargin   = 12.0;  // inflate obstacle AABBs by this (drone size + clearance)
        private const double RayFanDeg        = 25.0;  // lateral fan half-angle for the 3-ray narrow phase
        private const double AvoidBiasLateral = 40.0;  // sideways slide of the detour point
        private const double AvoidBiasUp      = 25.0;  // upward slide of the detour point (over beats around)
        private const double TerrainLookNear  = 100.0; // heightmap sample distances ahead along the velocity
        private const double TerrainLookFar   = 250.0;
        private const double TerrainClearance = 40.0;  // min clearance over terrain ahead (matches GroundAvoidAgl)
        private const double SenseLogSecs     = 3.0;   // throttle for the corridor-blocked sensing line

        private DateTime _lastBroad;
        private DateTime _lastSenseLog;
        private readonly List<IMyEntity> _nearby = new List<IMyEntity>(); // broad-phase cache (~1s)
        private MyPlanet _planet;
        private DateTime _lastPlanetFind;

        // True if the corridor toward 'target' is obstructed; 'via' is the suggested detour waypoint
        // and 'obstacle' names what's in the way (for the caller's log line). terrainClearance overrides
        // the default look-ahead floor (survey legs fly ~50 m AGL and pass a lower floor so routine
        // low flight isn't read as terrain danger).
        public bool TryGetDetour(NavState nav, IMyCubeGrid self, Vector3D target, out Vector3D via, out string obstacle,
                                 double terrainClearance = TerrainClearance)
        {
            via = default(Vector3D);
            obstacle = null;
            if (nav == null || !nav.Valid || self == null) return false;

            Vector3D pos = nav.Com;
            Vector3D toTarget = target - pos;
            double distToTarget = toTarget.Length();
            if (distToTarget < 1.0) return false;

            // Corridor direction: where we're GOING (velocity) when moving, else where we intend to go.
            Vector3D dir = nav.Speed > 2.0 ? nav.Velocity / nav.Speed : toTarget / distToTarget;
            // Never probe past the destination — the target base/standoff must not read as an obstacle.
            double corridorLen = Math.Min(distToTarget,
                MathHelper.Clamp(nav.Speed * CorridorSecs, MinCorridor, 500.0));

            RefreshBroadphase(pos);

            // ── Layer 1: terrain ahead (heightmap, no rays — physics rays can't hit voxels) ─────────
            if (_planet != null)
            {
                Vector3D up = nav.GravityUp;
                Vector3D dirH = dir - up * Vector3D.Dot(dir, up); // horizontal component of the heading
                if (dirH.LengthSquared() > 1e-4)
                {
                    dirH = Vector3D.Normalize(dirH);
                    double look1 = Math.Min(TerrainLookNear, corridorLen);
                    double look2 = Math.Min(TerrainLookFar, corridorLen);
                    for (int s = 0; s < 2; s++)
                    {
                        Vector3D sample = pos + dirH * (s == 0 ? look1 : look2);
                        Vector3D surface = _planet.GetClosestSurfacePointGlobal(ref sample);
                        double clearance = Vector3D.Dot(sample - surface, up);
                        if (clearance < terrainClearance)
                        {
                            // Terrain rises into the corridor — detour is the sample point lifted clear.
                            via = sample + up * (terrainClearance * 2.0 - clearance);
                            obstacle = "terrain ahead";
                            SenseLog(self, obstacle, s == 0 ? look1 : look2);
                            return true;
                        }
                    }
                }
            }

            // ── Layer 2: grid broad-phase — any nearby grid's AABB across the corridor? ─────────────
            RayD ray = new RayD(pos, dir);
            IMyCubeGrid blocking = null;
            double blockDist = double.MaxValue;
            for (int i = 0; i < _nearby.Count; i++)
            {
                var grid = _nearby[i] as IMyCubeGrid;
                if (grid == null || grid.EntityId == self.EntityId) continue;
                if (SameGroup(self, grid)) continue; // own docked/attached structure is not an obstacle

                BoundingBoxD box = grid.WorldAABB;
                box.Inflate(CorridorMargin);
                double? t = box.Intersects(ray);
                if (t.HasValue && t.Value < corridorLen && t.Value < blockDist)
                {
                    blockDist = t.Value;
                    blocking = grid;
                }
            }
            if (blocking == null) return false; // empty corridor — the overwhelmingly common case

            SenseLog(self, blocking.DisplayName, blockDist);

            // ── Layer 3: ray fan — only now do we pay for physics raycasts ──────────────────────────
            Vector3D hitPos; Vector3D hitNormal;
            if (!RayFan(self, pos, dir, corridorLen, nav.GravityUp, out hitPos, out hitNormal))
            {
                // AABB says blocked but rays passed (corner clip / conservative box): steer off the
                // AABB's near-intersect point anyway — cheap and safe.
                hitPos = pos + dir * blockDist;
                hitNormal = -dir;
            }

            // Detour: slide sideways away from the hit and UP (over beats around near the ground).
            Vector3D lateral = Vector3D.Cross(nav.GravityUp, dir);
            if (lateral.LengthSquared() < 1e-4) lateral = Vector3D.CalculatePerpendicularVector(dir);
            lateral = Vector3D.Normalize(lateral);
            double side = Vector3D.Dot(hitPos - pos, lateral) >= 0 ? -1.0 : 1.0; // away from the hit side
            via = hitPos + lateral * (side * AvoidBiasLateral) + nav.GravityUp * AvoidBiasUp;
            obstacle = "'" + (blocking.DisplayName ?? "grid") + "'";
            return true;
        }

        // 3-ray narrow phase: center + two lateral fan rays. Physics CastRay hits grids (NOT voxels —
        // terrain is layer 1's job). Ignores hits on self / own physical group.
        private bool RayFan(IMyCubeGrid self, Vector3D pos, Vector3D dir, double len, Vector3D up,
                            out Vector3D hitPos, out Vector3D hitNormal)
        {
            hitPos = default(Vector3D);
            hitNormal = default(Vector3D);
            double selfRadius = self.WorldAABB.HalfExtents.Length();
            double best = double.MaxValue;

            for (int i = -1; i <= 1; i++)
            {
                Vector3D d = i == 0 ? dir
                    : Vector3D.Normalize(Vector3D.Transform(dir,
                        QuaternionD.CreateFromAxisAngle(up, i * RayFanDeg * Math.PI / 180.0)));
                Vector3D from = pos + d * (selfRadius + 2.0); // start clear of our own hull
                Vector3D to = pos + d * len;

                IHitInfo hit;
                if (!MyAPIGateway.Physics.CastRay(from, to, out hit) || hit == null) continue;
                var hitGrid = hit.HitEntity as IMyCubeGrid;
                if (hitGrid != null && (hitGrid.EntityId == self.EntityId || SameGroup(self, hitGrid))) continue;

                double d2 = Vector3D.DistanceSquared(pos, hit.Position);
                if (d2 < best) { best = d2; hitPos = hit.Position; hitNormal = hit.Normal; }
            }
            return best < double.MaxValue;
        }

        // ~1 Hz: refresh the nearby-entity cache (pruning-structure sphere query) and the planet ref.
        private void RefreshBroadphase(Vector3D pos)
        {
            if ((DateTime.UtcNow - _lastBroad).TotalSeconds < BroadphaseSecs) return;
            _lastBroad = DateTime.UtcNow;

            var sphere = new BoundingSphereD(pos, ProbeRadius);
            _nearby.Clear();
            var found = MyAPIGateway.Entities.GetTopMostEntitiesInSphere(ref sphere);
            if (found != null) _nearby.AddRange(found);

            if ((DateTime.UtcNow - _lastPlanetFind).TotalSeconds > 30.0 || _planet == null)
            {
                _lastPlanetFind = DateTime.UtcNow;
                _planet = MyGamePruningStructure.GetClosestPlanet(pos);
            }
        }

        private static bool SameGroup(IMyCubeGrid a, IMyCubeGrid b)
        {
            var ga = a.GetGridGroup(GridLinkTypeEnum.Physical);
            return ga != null && ga == b.GetGridGroup(GridLinkTypeEnum.Physical);
        }

        // Throttled sensing line — proves layers 1/2 fired even before any deflection is visible.
        private void SenseLog(IMyCubeGrid self, string what, double dist)
        {
            if ((DateTime.UtcNow - _lastSenseLog).TotalSeconds < SenseLogSecs) return;
            _lastSenseLog = DateTime.UtcNow;
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] avoid: '{0}' corridor blocked by {1} ({2:F0} m ahead)",
                self.DisplayName, what, dist));
        }
    }
}
