using System;
using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.Utils;
using VRageMath;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

namespace ColonyFramework
{
    // FLIGHT.md §5 — per-ship-type tunables. Planner, steering, and verbs all read the SAME profile
    // (one source of truth per constraint). F1 carries the minimal set; F2/F3 extend it.
    public class FlightProfile
    {
        public double VMaxCruise = 70.0;   // hard cap; braking-aware speed usually rules before this
        public double ArriveTol = 3.0;    // m — arrival latch radius
        public double SettleSpeed = 0.75;  // m/s — "stopped enough" for arrival
        public double MinAgl = 20.0;   // m — test/cruise points are clamped at least this high

        public static readonly FlightProfile Default = new FlightProfile();
    }

    // FLIGHT.md §3 + §5.1 — THE actuator owner. One per drone; nothing else may touch thrusters,
    // gyros, or dampeners while a verb runs. The control cascade: position error → desired velocity
    // (trapezoid from the ship's OWN braking accel) → velocity error → force with full gravity
    // feedforward → per-body-axis thrust overrides. Deadbands + a latched arrival kill the jitter.
    public class FlightController
    {
        public enum VerbStatus { Idle, Running, Done, Failed }

        private const double Kv = 1.5;             // velocity-error gain (1/s) — gentle, 6 Hz stable
        private const double DeadbandDv = 0.4;     // m/s — below this, hand the axis back to dampeners
        private const double NoProgressSecs = 30.0;
        private const double GameSpeedCap = 95.0;  // SE hard speed limit safety margin

        private readonly BoreController _fly = new BoreController(); // proven damped gyro Face
        private readonly FlightSteering _steer = new FlightSteering(); // F2: context steering + camera probe
        private readonly List<BoundingSphereD> _workVolumes = new List<BoundingSphereD>();
        private ShipSelfModel _model;
        private double _modelMass;

        private int _verb;                          // 0=none 1=hover 2=moveto
        private Vector3D _goal, _holdPoint;
        private double _vMax;
        private bool _arrived;                      // latched (re-arms only past 2×tol)
        private DateTime _progressTime;
        private double _progressBest = double.MaxValue;

        public VerbStatus Status { get; private set; }
        public string FailReason { get; private set; }
        public FlightProfile Profile = FlightProfile.Default;

        // ── Verbs ────────────────────────────────────────────────────────────────────────────────
        public void Hover(IMyCubeGrid grid)
        {
            _verb = 1;
            _holdPoint = grid.GetPosition();
            _vMax = 5.0; // drift corrections are gentle; the trapezoid shrinks them further near the point
            Status = VerbStatus.Running;
        }

        public void MoveTo(IMyCubeGrid grid, Vector3D goal, double vMax = 0)
        {
            _verb = 2;
            _goal = goal;
            _vMax = vMax > 0 ? vMax : Profile.VMaxCruise;
            _arrived = false;
            _progressBest = double.MaxValue;
            _progressTime = DateTime.UtcNow;
            Status = VerbStatus.Running;
        }

        // FLIGHT.md §5.3 — the active maneuver declares volumes that are NOT obstacles (the
        // construction being welded, the base near dock). Steering exempts grids inside them;
        // the maneuver's own geometry owns safety there. Terrain is never exempt.
        public void DeclareWorkVolume(BoundingSphereD sphere) { _workVolumes.Add(sphere); }
        public void ClearWorkVolumes() { _workVolumes.Clear(); }

        // Give everything back: overrides zero, dampeners ON (the safety net on every exit path).
        public void Release(IMyCubeGrid grid)
        {
            _verb = 0;
            Status = VerbStatus.Idle;
            _fly.Release(grid);
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) { rc.SetAutoPilotEnabled(false); rc.DampenersOverride = true; }
        }

        // ── The loop (tick at ~6 Hz) ─────────────────────────────────────────────────────────────
        public void Tick(IMyCubeGrid grid)
        {
            if (_verb == 0) return;
            var rc = DroneUtil.FindRc(grid);
            if (rc == null || grid.Physics == null) { Fail(grid, "rc/physics lost"); return; }

            // Self-knowledge, refreshed when mass drifts (cargo picked up/dumped changes everything).
            if (_model == null || Math.Abs(grid.Physics.Mass - _modelMass) > _modelMass * 0.10)
            {
                _model = ShipSelfModel.Build(grid);
                if (_model == null) { Fail(grid, "self-model unavailable"); return; }
                _modelMass = grid.Physics.Mass;
            }

            Vector3D pos = grid.GetPosition();
            Vector3D vel = grid.Physics.LinearVelocity;
            float interference;
            Vector3D gVec = MyAPIGateway.Physics.CalculateNaturalGravityAt(pos, out interference);
            Vector3D up = gVec.LengthSquared() > 0.01 ? -Vector3D.Normalize(gVec) : rc.WorldMatrix.Up;

            // Attitude: hold LEVEL (translation is body-frame allocated, so no yaw needed; a level
            // ship keeps its strongest thrusters — up — against gravity). Verbs with special facing
            // come in F2/F4 (OrientAndCreep).
            _fly.Face(grid, rc.WorldMatrix.Up, up);

            Vector3D target = _verb == 2 ? _goal : _holdPoint;
            Vector3D err = target - pos;
            double dist = err.Length();

            if (_verb == 2)
            {
                // Latched arrival (FLIGHT.md §5.4): flip once, never flicker.
                if (!_arrived && dist < Profile.ArriveTol && vel.Length() < Profile.SettleSpeed)
                {
                    _arrived = true;
                    Status = VerbStatus.Done;
                    _holdPoint = target;
                    _verb = 1; // hold position at the goal until told otherwise
                    IdleHold(grid);
                    return;
                }
                // Watchdog: closing distance is progress; none for 30 s = failed leg.
                if (dist < _progressBest - 1.0) { _progressBest = dist; _progressTime = DateTime.UtcNow; }
                else if ((DateTime.UtcNow - _progressTime).TotalSeconds > NoProgressSecs)
                { Fail(grid, string.Format("no progress toward goal ({0:F0} m away)", dist)); return; }
            }

            // Desired velocity: trapezoid — approach speed limited by the ship's OWN braking ability
            // (v = √(2·a·d)), the verb's cap, and the game's. "Fast vs slow" is physics, not a constant.
            double aBrake = WeakestAccel();
            double vAllowed = Math.Min(Math.Min(_vMax, GameSpeedCap), Math.Sqrt(Math.Max(0, 2 * aBrake * dist)));
            Vector3D vStar;
            Vector3D steerDir; double steerCap;
            if (_verb == 2 && dist > Profile.ArriveTol * 3
                && _steer.Steer(grid, _model, Profile, pos, vel, target, _workVolumes, out steerDir, out steerCap))
            {
                // Path needs shaping (terrain/grid/camera): fly the steered direction at its cap.
                // The trapezoid's goal-distance brake still applies — deviation never overshoots.
                vStar = steerDir * Math.Min(vAllowed, steerCap);
            }
            else
            {
                vStar = dist > 0.05 ? (err / dist) * vAllowed : Vector3D.Zero;
            }

            Vector3D dv = vStar - vel;
            if (dv.Length() < DeadbandDv && vStar.Length() < 0.5)
            {
                IdleHold(grid); // close and slow: dampeners hold better than we can — zero chatter
                return;
            }

            // Force: velocity correction + FULL gravity feedforward (dampeners are OFF while we fly).
            rc.DampenersOverride = false;
            Vector3D accel = dv * Kv;
            Vector3D force = grid.Physics.Mass * accel - grid.Physics.Mass * gVec;
            ApplyForce(grid, rc, force);
        }

        // Hand control to dampeners: overrides zeroed, dampers on. Gyro keeps leveling via Face.
        private void IdleHold(IMyCubeGrid grid)
        {
            ZeroThrust(grid);
            var rc = DroneUtil.FindRc(grid);
            if (rc != null && !rc.DampenersOverride) rc.DampenersOverride = true;
        }

        private void Fail(IMyCubeGrid grid, string reason)
        {
            FailReason = reason;
            Status = VerbStatus.Failed;
            _verb = 0;
            Release(grid); // dampeners back on — the safety net on EVERY failure path
        }

        // Weakest derated axis accel — braking may need any axis depending on travel direction;
        // planning on the worst one is safe and simple (F2 refines to direction-aware).
        private double WeakestAccel()
        {
            double a = double.MaxValue;
            for (int i = 0; i < 6; i++) if (_model.Accel(i) < a) a = _model.Accel(i);
            return Math.Max(0.2, a); // floor: never divide the trapezoid to zero
        }

        // Project the world-frame force onto the RC body axes and set overrides on each axis's
        // thrusters, distributed by their share of the axis total. Opposing axes get zero.
        private void ApplyForce(IMyCubeGrid grid, IMyRemoteControl rc, Vector3D force)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return;
            var thrusters = new List<IMyThrust>();
            ts.GetBlocksOfType(thrusters);

            var m = rc.WorldMatrix;
            Vector3D[] axes = { m.Forward, m.Backward, m.Up, m.Down, m.Left, m.Right };
            double[] demand = new double[6];
            for (int a = 0; a < 6; a++)
                demand[a] = Math.Max(0, Vector3D.Dot(force, axes[a])); // each axis carries only positive pull

            // Axis totals from the model (already classified there, same math).
            for (int i = 0; i < thrusters.Count; i++)
            {
                var t = thrusters[i];
                Vector3D push = t.WorldMatrix.Backward;
                int best = 0; double bestDot = -2;
                for (int a = 0; a < 6; a++)
                {
                    double d = Vector3D.Dot(push, axes[a]);
                    if (d > bestDot) { bestDot = d; best = a; }
                }
                if (bestDot < 0.7 || _model.ThrustN[best] <= 0) { t.ThrustOverride = 0f; continue; }
                double share = demand[best] * (t.MaxEffectiveThrust / _model.ThrustN[best]);
                t.ThrustOverride = (float)Math.Min(share, t.MaxEffectiveThrust);
            }
        }

        private static void ZeroThrust(IMyCubeGrid grid)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return;
            var thrusters = new List<IMyThrust>();
            ts.GetBlocksOfType(thrusters);
            for (int i = 0; i < thrusters.Count; i++) thrusters[i].ThrustOverride = 0f;
        }
    }

    // F1 acceptance harness (FLIGHT.md §7): pad → point → pad on the velocity loop, driven by
    // `/colony flighttest [dist]`. Proves smooth ramp/cruise/brake from the ship's own numbers
    // WITHOUT converting any frozen mission path.
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
            _fc.MoveTo(grid, _far);
            Log(grid, string.Format("leg 1 — out {0:F0} m", _dist));
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

            // Narrate speed vs distance every 3 s — the log evidence that speed comes from physics.
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
                    { _leg = 2; _fc.MoveTo(grid, _start); Log(grid, "leg 2 — returning"); }
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
