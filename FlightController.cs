using System;
using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.Utils;
using VRageMath;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;
using IMyCubeBlock = VRage.Game.ModAPI.IMyCubeBlock;

namespace ColonyFramework
{
    // FLIGHT.md §5 — per-ship-type tunables. Planner, steering, and verbs all read the SAME profile
    // (one source of truth per constraint).
    public class FlightProfile
    {
        public double VMaxCruise = 70.0;   // hard cap; braking-aware speed usually rules before this
        public double ArriveTol = 3.0;    // m — arrival latch radius (final points)
        public double PassTol = 15.0;   // m — corridor intermediate points are passed, not parked at
        public double SettleSpeed = 0.75;  // m/s — "stopped enough" for arrival
        public double MinAgl = 20.0;   // m — steering's terrain floor
        public double CruiseAgl = 60.0;   // m — corridor planning altitude over terrain

        public static readonly FlightProfile Default = new FlightProfile();
    }

    // FLIGHT.md §3 + §5 — THE actuator owner. One per drone; nothing else touches thrusters, gyros,
    // or dampeners while a verb runs. Cascade: position error → desired velocity (trapezoid from the
    // ship's OWN braking accel) → velocity error → force with gravity feedforward → per-body-axis
    // thrust overrides. Verbs: Hover, MoveTo, Transit (corridor), OrientAndCreep, SlideTo, Release.
    public class FlightController
    {
        public enum VerbStatus { Idle, Running, Done, Failed }

        private const int VNone = 0, VHover = 1, VMoveTo = 2, VTransit = 3, VCreep = 4, VSlide = 5;

        private const double Kv = 1.5;             // velocity-error gain (1/s) — gentle, 6 Hz stable
        private const double DeadbandDv = 0.4;     // m/s — below this, hand the axis back to dampeners
        private const double NoProgressSecs = 30.0;
        private const double CreepNoProgressSecs = 60.0;
        private const double GameSpeedCap = 95.0;  // SE hard speed limit safety margin

        private readonly BoreController _fly = new BoreController(); // proven damped gyro Face
        private readonly FlightSteering _steer = new FlightSteering(); // F2: context steering + camera probe
        private readonly List<BoundingSphereD> _workVolumes = new List<BoundingSphereD>();
        private ShipSelfModel _model;
        private double _modelMass;

        private int _verb;
        private Vector3D _goal, _holdPoint, _lockDir;
        private List<Vector3D> _route; private int _routeIdx; private double[] _routeRemainAfter;
        private long _faceBlockId; private double _stopDist;
        private double _vMax;
        private bool _arrived;                      // latched (per FLIGHT.md §5.4)
        private DateTime _progressTime;
        private double _progressBest = double.MaxValue;

        public VerbStatus Status { get; private set; }
        public string FailReason { get; private set; }
        public FlightProfile Profile = FlightProfile.Default;

        // ── Verbs ────────────────────────────────────────────────────────────────────────────────
        public void Hover(IMyCubeGrid grid)
        {
            _verb = VHover;
            _holdPoint = grid.GetPosition();
            _vMax = 5.0;
            Status = VerbStatus.Running;
        }

        public void MoveTo(IMyCubeGrid grid, Vector3D goal, double vMax = 0)
        {
            _verb = VMoveTo;
            _goal = goal;
            _vMax = vMax > 0 ? vMax : Profile.VMaxCruise;
            ResetLegState();
        }

        // F3: fly a corridor — intermediate points are PASSED at speed (loose tolerance, no settle),
        // only the final point gets the arrival latch. Braking distance is measured against the
        // remaining ROUTE length, so corners don't cause braking.
        public void Transit(IMyCubeGrid grid, List<Vector3D> route, double vMax = 0)
        {
            if (route == null || route.Count == 0) { Fail(grid, "empty route"); return; }
            _route = route;
            _routeIdx = 0;
            _routeRemainAfter = new double[route.Count];
            for (int i = route.Count - 2; i >= 0; i--)
                _routeRemainAfter[i] = _routeRemainAfter[i + 1] + Vector3D.Distance(route[i], route[i + 1]);
            _verb = VTransit;
            _vMax = vMax > 0 ? vMax : Profile.VMaxCruise;
            ResetLegState();
        }

        // F4: precision nose-in — face 'tool's forward along the line to 'goal', creep until the
        // TOOL is within stopDist, then hold pose (position + facing). Steering off: this runs
        // inside a declared work volume where the maneuver's geometry owns safety.
        public void OrientAndCreep(IMyCubeGrid grid, Vector3D goal, IMyCubeBlock tool, double stopDist, double vCreep)
        {
            _verb = VCreep;
            _goal = goal;
            _faceBlockId = tool != null ? tool.EntityId : 0;
            _stopDist = stopDist;
            _vMax = vCreep;
            ResetLegState();
        }

        // F4: translate with attitude LOCKED to the current facing (back-outs near structure —
        // swinging the nose while reversing beside a hull is how collisions happen).
        public void SlideTo(IMyCubeGrid grid, Vector3D goal, double vMax = 3.0)
        {
            var rc = DroneUtil.FindRc(grid);
            _lockDir = rc != null ? rc.WorldMatrix.Forward : Vector3D.Forward;
            _verb = VSlide;
            _goal = goal;
            _vMax = vMax;
            ResetLegState();
        }

        // FLIGHT.md §5.3 — the active maneuver declares volumes that are NOT obstacles. Steering
        // exempts grids inside them; the maneuver's geometry owns safety there. Terrain never exempt.
        public void DeclareWorkVolume(BoundingSphereD sphere) { _workVolumes.Add(sphere); }
        public void ClearWorkVolumes() { _workVolumes.Clear(); }

        // Give everything back: overrides zero, dampeners ON (the safety net on every exit path).
        public void Release(IMyCubeGrid grid)
        {
            _verb = VNone;
            Status = VerbStatus.Idle;
            _fly.Release(grid);
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) { rc.SetAutoPilotEnabled(false); rc.DampenersOverride = true; }
        }

        private void ResetLegState()
        {
            _arrived = false;
            _progressBest = double.MaxValue;
            _progressTime = DateTime.UtcNow;
            Status = VerbStatus.Running;
        }

        // ── The loop (tick at ~6 Hz) ─────────────────────────────────────────────────────────────
        public void Tick(IMyCubeGrid grid)
        {
            if (_verb == VNone) return;
            var rc = DroneUtil.FindRc(grid);
            if (rc == null || grid.Physics == null) { Fail(grid, "rc/physics lost"); return; }

            // Self-knowledge, refreshed when mass drifts (cargo changes everything).
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

            // Reference position: the CREEP verb measures from the tool, not the grid centre —
            // "the welder is in range" is the contract, wherever the tool is mounted.
            IMyCubeBlock faceBlock = _faceBlockId != 0
                ? MyAPIGateway.Entities.GetEntityById(_faceBlockId) as IMyCubeBlock : null;
            Vector3D refPos = _verb == VCreep && faceBlock != null ? faceBlock.GetPosition() : pos;

            // Current target point for this verb.
            Vector3D target;
            if (_verb == VTransit)
            {
                target = _route[_routeIdx];
                if (_routeIdx < _route.Count - 1 && Vector3D.Distance(pos, target) < Profile.PassTol)
                { _routeIdx++; target = _route[_routeIdx]; }
            }
            else if (_verb == VHover) target = _holdPoint;
            else if (_arrived) target = _holdPoint; // done verbs hold where they finished
            else target = _goal;

            Vector3D err = target - refPos;
            double dist = err.Length();

            // ── Attitude policy per verb ─────────────────────────────────────────────────────────
            if (_verb == VCreep && faceBlock != null && dist > 0.1)
                _fly.Face(grid, faceBlock.WorldMatrix.Forward, err / dist);
            else if (_verb == VSlide)
                _fly.Face(grid, rc.WorldMatrix.Forward, _lockDir);
            else
                _fly.Face(grid, rc.WorldMatrix.Up, up); // cruise verbs fly level

            // ── Completion + watchdog ────────────────────────────────────────────────────────────
            if (!_arrived && Status == VerbStatus.Running)
            {
                bool done =
                    _verb == VCreep ? dist <= _stopDist :
                    _verb == VTransit ? (_routeIdx == _route.Count - 1
                                          && dist < Profile.ArriveTol && vel.Length() < Profile.SettleSpeed) :
                    (_verb == VMoveTo || _verb == VSlide) && dist < Profile.ArriveTol && vel.Length() < Profile.SettleSpeed;
                if (done)
                {
                    _arrived = true;
                    Status = VerbStatus.Done;
                    _holdPoint = _verb == VCreep ? pos : target; // creep holds where the TOOL landed in range
                    IdleHold(grid);
                    return;
                }
                double watchdog = _verb == VCreep ? CreepNoProgressSecs : NoProgressSecs;
                if (dist < _progressBest - 1.0) { _progressBest = dist; _progressTime = DateTime.UtcNow; }
                else if ((DateTime.UtcNow - _progressTime).TotalSeconds > watchdog)
                { Fail(grid, string.Format("no progress toward goal ({0:F0} m away)", dist)); return; }
            }
            if (_arrived) { err = _holdPoint - pos; dist = err.Length(); } // hold uses grid position

            // ── Desired velocity: trapezoid from the ship's OWN braking accel ────────────────────
            double aBrake = WeakestAccel();
            double brakeBaseDist = _verb == VTransit && !_arrived
                ? dist + _routeRemainAfter[_routeIdx] // corners don't brake: distance-to-END rules
                : _verb == VCreep && !_arrived
                    ? Math.Max(0, dist - _stopDist)   // creep brakes onto the stop ring, not the block
                    : dist;
            double vAllowed = Math.Min(Math.Min(_arrived ? 5.0 : _vMax, GameSpeedCap),
                                       Math.Sqrt(Math.Max(0, 2 * aBrake * brakeBaseDist)));

            Vector3D vStar;
            Vector3D steerDir = Vector3D.Zero; double steerCap = 0;
            bool steered = (_verb == VMoveTo || _verb == VTransit) && !_arrived
                && dist > Profile.ArriveTol * 3
                && _steer.Steer(grid, _model, Profile, pos, vel, target, _workVolumes, out steerDir, out steerCap);
            if (steered)
                vStar = steerDir * Math.Min(vAllowed, steerCap);
            else
                vStar = dist > 0.05 ? (err / dist) * vAllowed : Vector3D.Zero;

            Vector3D dv = vStar - vel;
            if (dv.Length() < DeadbandDv && vStar.Length() < 0.5)
            {
                IdleHold(grid); // close and slow: dampeners hold better than we can — zero chatter
                return;
            }

            // Force: velocity correction + FULL gravity feedforward (dampeners OFF while we fly).
            rc.DampenersOverride = false;
            Vector3D force = grid.Physics.Mass * (dv * Kv) - grid.Physics.Mass * gVec;
            ApplyForce(grid, rc, force);
        }

        // Hand control to dampeners: overrides zeroed, dampers on. Gyro keeps its facing via Face.
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
            _verb = VNone;
            Release(grid); // dampeners back on — the safety net on EVERY failure path
        }

        // Weakest derated axis accel — braking may need any axis; planning on the worst is safe.
        private double WeakestAccel()
        {
            double a = double.MaxValue;
            for (int i = 0; i < 6; i++) if (_model.Accel(i) < a) a = _model.Accel(i);
            return Math.Max(0.2, a);
        }

        // Project the world-frame force onto the RC body axes; overrides per thruster by share.
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
                demand[a] = Math.Max(0, Vector3D.Dot(force, axes[a]));

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
}
