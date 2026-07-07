using System;
using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.Game.ModAPI.Ingame;
using VRage.Utils;
using VRageMath;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

namespace ColonyFramework
{
    // FLIGHT.md §5.3 / F2 — context steering: spatial awareness that feeds a DIRECTION into the
    // velocity loop instead of spawning detour waypoints. Each decision tick we score a fan of
    // candidate directions (goal alignment + continuity − grid danger − camera penalty), take the
    // best whose TERRAIN probe passes, and blend in the climb the terrain demands. A blocked
    // direction just scores lower — it can never become a contradictory goal, so the old
    // deflection loops ("blocked by 'e' … deflected … blocked …") are impossible by construction.
    //
    // Obstacle identity (the welder killer): grids intersecting a declared WORK VOLUME are not
    // obstacles — the maneuver's own geometry owns that space. Terrain is always real.
    public class FlightSteering
    {
        private const double SteerIntervalSecs = 0.5;   // decision rate (cached between)
        private const double ObstacleRefreshSecs = 2.0; // broad-phase entity query rate
        private const double CameraIntervalSecs = 0.5;  // forward raycast rate (charge-friendly)
        private const double GridMargin = 8.0;          // keep-out inflation beyond AABB sphere
        private const double GridDangerWeight = 2.0;
        private const double Stickiness = 0.15;         // bonus for keeping last tick's choice (anti flip-flop)
        private const double CamPenaltySecs = 5.0;      // how long a camera hit poisons a direction
        private const double MinTerrainFrac = 0.4;      // candidate rejected below 40% of minAgl clearance
        private static readonly double[] YawDeg = { 0, 20, -20, 40, -40, 65, -65, 90, -90, 120, -120, 180 };

        private readonly List<BoundingSphereD> _obstacles = new List<BoundingSphereD>();
        private Sandbox.Game.Entities.MyPlanet _planet;
        private DateTime _lastSteer, _lastObstacles, _lastLog, _lastCam, _camHitAt, _lastPlanet;
        private Vector3D _lastDir;
        private bool _hasLast;
        private Vector3D _camHitDir;
        private Vector3D _cachedDir;
        private double _cachedVCap;
        private bool _cachedDeviated;

        // True = path needs shaping: fly 'dir' at up to 'vCap'. False = direct line is clean.
        public bool Steer(IMyCubeGrid grid, ShipSelfModel model, FlightProfile profile,
                          Vector3D pos, Vector3D vel, Vector3D goal,
                          List<BoundingSphereD> workVolumes,
                          out Vector3D dir, out double vCap)
        {
            dir = _cachedDir; vCap = _cachedVCap;
            if ((DateTime.UtcNow - _lastSteer).TotalSeconds < SteerIntervalSecs) return _cachedDeviated;
            _lastSteer = DateTime.UtcNow;

            Vector3D toGoal = goal - pos;
            double dist = toGoal.Length();
            if (dist < 10) { _cachedDeviated = false; return false; } // precision range: arrival logic owns it
            Vector3D goalDir = toGoal / dist;

            float interference;
            Vector3D gVec = MyAPIGateway.Physics.CalculateNaturalGravityAt(pos, out interference);
            Vector3D up = gVec.LengthSquared() > 0.01 ? -Vector3D.Normalize(gVec) : Vector3D.Up;

            // Look-ahead scales with the ship's own stopping distance — a fast/heavy ship looks farther.
            double lookahead = MathHelper.Clamp(
                model.BrakingDistance(vel.Length()) * 1.5 + model.PhysicalRadius, 40.0, 200.0);
            double probeLen = Math.Min(lookahead, dist);

            RefreshObstacles(grid, pos, probeLen, workVolumes);

            // Score the fan: goal alignment + last-choice stickiness − grid danger − camera penalty.
            // Then walk candidates best-first and take the first whose TERRAIN probe passes.
            Vector3D bestDir = up; double bestCap = 5.0; bool found = false; double chosenYaw = 0;
            var scored = new List<KeyValuePair<double, int>>(YawDeg.Length);
            for (int i = 0; i < YawDeg.Length; i++)
            {
                Vector3D cand = RotateAboutUp(goalDir, up, YawDeg[i] * Math.PI / 180.0);
                double interest = 1.0 - Math.Abs(YawDeg[i]) / 180.0 * 1.2;
                if (_hasLast && Vector3D.Dot(cand, _lastDir) > 0.95) interest += Stickiness;
                double danger = GridDanger(pos, cand, probeLen);
                if ((DateTime.UtcNow - _camHitAt).TotalSeconds < CamPenaltySecs
                    && Vector3D.Dot(cand, _camHitDir) > 0.9) danger += 1.0;
                scored.Add(new KeyValuePair<double, int>(interest - GridDangerWeight * danger, i));
            }
            scored.Sort((a, b) => b.Key.CompareTo(a.Key));

            for (int s = 0; s < scored.Count; s++)
            {
                int i = scored[s].Value;
                Vector3D cand = RotateAboutUp(goalDir, up, YawDeg[i] * Math.PI / 180.0);
                double climb;
                if (!TerrainOk(pos, cand, probeLen, up, profile.MinAgl, out climb)) continue;
                // Blend in the climb the terrain ahead demands (gentle slope, not a spike).
                bestDir = climb > 0.1 ? Vector3D.Normalize(cand + up * (climb / (probeLen * 0.5))) : cand;
                chosenYaw = YawDeg[i];
                // Slow down proportionally to how compromised the choice is (turning hard or climbing).
                double hazard = Math.Abs(chosenYaw) / 180.0 + (climb > 0.1 ? 0.3 : 0);
                bestCap = hazard > 0.05 ? Math.Max(8.0, profile.VMaxCruise * (1.0 - hazard)) : double.MaxValue;
                found = true;
                break;
            }
            if (!found)
            {
                // Boxed in: the universal SE escape is straight up (terrain can't be above minAgl forever).
                bestDir = up; bestCap = 8.0; chosenYaw = 999;
            }

            CameraVerify(grid, pos, bestDir, probeLen, workVolumes);

            _lastDir = bestDir; _hasLast = true;
            _cachedDir = bestDir; _cachedVCap = bestCap;
            _cachedDeviated = chosenYaw != 0 || Vector3D.Dot(bestDir, goalDir) < 0.98;

            if (_cachedDeviated && (DateTime.UtcNow - _lastLog).TotalSeconds > 3)
            {
                _lastLog = DateTime.UtcNow;
                MyLog.Default.WriteLineAndConsole(string.Format(
                    "[ColonyFramework] steer '{0}': {1} (cap {2:F0} m/s, lookahead {3:F0} m)",
                    grid.DisplayName,
                    chosenYaw == 999 ? "boxed in — climbing" :
                    Math.Abs(chosenYaw) > 0.1 ? string.Format("yaw {0:+0;-0}°", chosenYaw) : "climbing for terrain",
                    bestCap > 1000 ? 999 : bestCap, probeLen));
            }
            dir = bestDir; vCap = bestCap;
            return _cachedDeviated;
        }

        // ── Grid obstacles: keep-out spheres, minus own group and anything inside a work volume ──
        private void RefreshObstacles(IMyCubeGrid self, Vector3D pos, double lookahead, List<BoundingSphereD> workVolumes)
        {
            if ((DateTime.UtcNow - _lastObstacles).TotalSeconds < ObstacleRefreshSecs) return;
            _lastObstacles = DateTime.UtcNow;
            _obstacles.Clear();

            var sphere = new BoundingSphereD(pos, lookahead + 100);
            var near = MyAPIGateway.Entities.GetTopMostEntitiesInSphere(ref sphere);
            var ownGroup = self.GetGridGroup(VRage.Game.ModAPI.GridLinkTypeEnum.Physical);
            for (int i = 0; i < near.Count; i++)
            {
                var g = near[i] as IMyCubeGrid;
                if (g == null || g.EntityId == self.EntityId) continue;
                if (ownGroup != null && g.GetGridGroup(VRage.Game.ModAPI.GridLinkTypeEnum.Physical) == ownGroup) continue;
                var vol = g.WorldVolume;
                bool exempt = false;
                if (workVolumes != null)
                    for (int w = 0; w < workVolumes.Count && !exempt; w++)
                        if (workVolumes[w].Intersects(vol)) exempt = true; // the thing we're working on is not an obstacle
                if (exempt) continue;
                _obstacles.Add(new BoundingSphereD(vol.Center, vol.Radius + GridMargin));
            }
        }

        // Fractional danger of flying 'cand' for 'len' meters: 1 = obstacle right here, 0 = clear.
        private double GridDanger(Vector3D pos, Vector3D cand, double len)
        {
            double worst = 0;
            for (int i = 0; i < _obstacles.Count; i++)
            {
                var o = _obstacles[i];
                Vector3D rel = o.Center - pos;
                double along = Vector3D.Dot(rel, cand);
                if (along < -o.Radius || along > len + o.Radius) continue;
                double lateralSq = rel.LengthSquared() - along * along;
                if (lateralSq > o.Radius * o.Radius) continue;
                double t = MathHelper.Clamp(along / len, 0, 1);
                double danger = 1.0 - t * 0.8; // nearer = more dangerous
                if (danger > worst) worst = danger;
            }
            return worst;
        }

        // ── Terrain: heightmap clearance at two points ahead; 'climb' = extra altitude demanded ──
        private bool TerrainOk(Vector3D pos, Vector3D cand, double len, Vector3D up, double minAgl, out double climb)
        {
            climb = 0;
            var planet = Planet(pos);
            if (planet == null) return true; // space: no terrain
            double worstClear = double.MaxValue;
            for (int s = 1; s <= 2; s++)
            {
                Vector3D sample = pos + cand * (len * 0.5 * s);
                Vector3D surface = planet.GetClosestSurfacePointGlobal(ref sample);
                double clear = Vector3D.Dot(sample - surface, up);
                if (clear < worstClear) worstClear = clear;
            }
            if (worstClear < minAgl * MinTerrainFrac) return false; // would fly into a rise: reject
            if (worstClear < minAgl) climb = minAgl - worstClear;   // acceptable but climb while going
            return true;
        }

        // ── Camera: verify the CHOSEN direction with a real raycast (voxels + grids — catches
        // overhangs and cliffs the heightmap can't see). A hit poisons that direction for a while. ──
        private void CameraVerify(IMyCubeGrid grid, Vector3D pos, Vector3D dir, double len, List<BoundingSphereD> workVolumes)
        {
            if ((DateTime.UtcNow - _lastCam).TotalSeconds < CameraIntervalSecs) return;
            _lastCam = DateTime.UtcNow;

            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return;
            var cams = new List<Sandbox.ModAPI.IMyCameraBlock>();
            ts.GetBlocksOfType(cams);
            for (int i = 0; i < cams.Count; i++)
            {
                var cam = cams[i];
                if (!cam.IsWorking) continue;
                if (Vector3D.Dot(cam.WorldMatrix.Forward, dir) < 0.7) continue; // outside this camera's cone
                cam.EnableRaycast = true;
                if (!cam.CanScan(len)) return; // charging — heightmap + spheres still guard
                var info = cam.Raycast(len);
                if (info.IsEmpty() || !info.HitPosition.HasValue) return;
                // Hits inside a work volume are the work, not a hazard.
                if (workVolumes != null)
                    for (int w = 0; w < workVolumes.Count; w++)
                        if (workVolumes[w].Contains(info.HitPosition.Value) == ContainmentType.Contains) return;
                if (Vector3D.DistanceSquared(info.HitPosition.Value, pos) < len * len * 0.81)
                {
                    _camHitDir = dir;
                    _camHitAt = DateTime.UtcNow;
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] steer '{0}': camera hit {1} at {2:F0} m — poisoning direction",
                        grid.DisplayName, info.Type, Math.Sqrt(Vector3D.DistanceSquared(info.HitPosition.Value, pos))));
                }
                return; // one camera per pass is enough
            }
        }

        private Sandbox.Game.Entities.MyPlanet Planet(Vector3D near)
        {
            if (_planet == null || (DateTime.UtcNow - _lastPlanet).TotalSeconds > 60)
            {
                _lastPlanet = DateTime.UtcNow;
                _planet = Sandbox.Game.Entities.MyGamePruningStructure.GetClosestPlanet(near);
            }
            return _planet;
        }

        private static Vector3D RotateAboutUp(Vector3D v, Vector3D up, double angleRad)
        {
            if (Math.Abs(angleRad) < 1e-6) return v;
            return Vector3D.Transform(v, QuaternionD.CreateFromAxisAngle(up, angleRad));
        }
    }
}
