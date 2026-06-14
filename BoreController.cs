using System.Collections.Generic;
using Sandbox.ModAPI;
using VRageMath;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

namespace ColonyFramework
{
    // Direct flight control for the bore phase: gyro-override orientation hold +
    // velocity-limited advance via thrust override. Autopilot must be OFF while in use.
    public class BoreController
    {
        private const double GyroGain = 1.0;
        private const double DampGain = 1.5; // high damping — settles nose-down without overshooting past vertical
        private const float MaxGyroRpm = 1.0f;

        // Aligns 'aimAxis' (e.g. drill forward) to 'targetDir' and advances along it at ~targetSpeed.
        // Returns alignment dot for the caller's gating/logging.
        public double Drive(IMyCubeGrid grid, Vector3D aimAxis, Vector3D targetDir, double targetSpeed)
        {
            double dot = Align(grid, aimAxis, targetDir);

            // Only thrust once roughly aligned, so we don't bore sideways.
            if (dot > 0.95) Advance(grid, targetDir, targetSpeed);
            else Advance(grid, targetDir, 0); // hold position while turning

            return dot;
        }

        // Bang-bang velocity control along an arbitrary world direction WITHOUT changing
        // orientation: overrides thrusters facing 'dir' to push while below targetSpeed,
        // releasing them (to dampers) at/above. Caller holds orientation via Drive(.., 0).
        public void ThrustAlong(IMyCubeGrid grid, Vector3D dir, double targetSpeed, float power = 0.5f)
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
                t.ThrustOverridePercentage = (push && d > 0.9) ? power : 0f;
            }
        }

        // Gyro-only: rotate so 'aimAxis' points along 'targetDir' WITHOUT translating. Returns the
        // alignment dot. Pair with ThrustAlong or Maneuver when facing and travel directions differ.
        public double Face(IMyCubeGrid grid, Vector3D aimAxis, Vector3D targetDir)
        {
            return Align(grid, aimAxis, targetDir);
        }

        // Pilot controller for flying with inertial dampeners OFF — our own software dampers.
        // It ALWAYS fully counters gravity, then adds a strong velocity term to track a target
        // velocity toward 'toTarget' (eased to a stop). With no target it just hovers (brakes
        // velocity + holds altitude), exactly like dampeners but controllable per axis.
        // Returns the worst-direction thrust saturation: >1 means the drone physically can't make
        // the commanded force in some direction (i.e. it's underpowered, not a tuning problem).
        private const double VelGain = 0.6; // desired approach speed = distance * VelGain (capped at maxSpeed)
        private const double VelKp   = 3.0; // acceleration per unit velocity error (firmer than before)
        public double Maneuver(IMyCubeGrid grid, Vector3D toTarget, double maxSpeed, double stopTol)
        {
            if (grid.Physics == null) return 0;
            double dist = toTarget.Length();
            Vector3D dir = dist > 1e-3 ? toTarget / dist : Vector3D.Zero;
            double desiredSpeed = dist <= stopTol ? 0.0 : System.Math.Min(maxSpeed, dist * VelGain);
            Vector3D velErr = dir * desiredSpeed - (Vector3D)grid.Physics.LinearVelocity;

            float interference;
            Vector3D g = MyAPIGateway.Physics.CalculateNaturalGravityAt(grid.GetPosition(), out interference);
            Vector3D accel = velErr * VelKp - g;             // track velocity + FULL gravity compensation
            return ApplyForce(grid, accel * grid.Physics.Mass);
        }

        // Distributes a desired world force across thrusters. Per thrust direction the commanded
        // component is SHARED across all co-directional thrusters (override = comp / total-max-in-dir),
        // so the summed thrust equals the command — not N× it. Returns the max (needed/available)
        // ratio across directions: >1 = saturated (underpowered) in that direction.
        private readonly Dictionary<Vector3I, double> _dirMax = new Dictionary<Vector3I, double>();
        private double ApplyForce(IMyCubeGrid grid, Vector3D force)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return 0;
            var thrusters = new List<IMyThrust>();
            ts.GetBlocksOfType(thrusters);

            _dirMax.Clear();
            for (int i = 0; i < thrusters.Count; i++)
            {
                var d = thrusters[i].GridThrustDirection;
                double cur; _dirMax.TryGetValue(d, out cur);
                _dirMax[d] = cur + thrusters[i].MaxEffectiveThrust;
            }

            double worstSat = 0;
            for (int i = 0; i < thrusters.Count; i++)
            {
                var t = thrusters[i];
                double comp = Vector3D.Dot(force, t.WorldMatrix.Backward);
                double total; _dirMax.TryGetValue(t.GridThrustDirection, out total);
                if (comp > 0 && total > 0)
                {
                    double ratio = comp / total;
                    if (ratio > worstSat) worstSat = ratio;
                    t.ThrustOverridePercentage = MathHelper.Clamp((float)ratio, 0f, 1f);
                }
                else t.ThrustOverridePercentage = 0f;
            }
            return worstSat;
        }

        // Per-axis software dampener (dampeners OFF), consuming the awareness layer. Splits the error
        // to the target into a VERTICAL (gravity-up) axis and a HORIZONTAL axis; holds each toward
        // zero at a capped, eased speed with a position + velocity deadband; brakes sideways drift;
        // ALWAYS fully compensates gravity. Inside both deadbands it is a pure hover — no hunting, no
        // oscillation, no wasted power. Because altitude is its own axis, "at the right height but
        // off horizontally" no longer blocks, and altitude is actively held during lateral/axial
        // moves. Returns worst-axis thrust saturation (>1 = physically underpowered).
        private const double ApproachGain = 0.8; // desired axis speed = error * gain, capped at maxSpeed
        public double Navigate(IMyCubeGrid grid, NavState nav, Vector3D targetPos,
                               double maxSpeed, double posDeadband, double velDeadband)
        {
            if (grid.Physics == null || !nav.Valid) return 0;
            Vector3D up = nav.GravityUp;

            double vErr = nav.VerticalError(targetPos);          // signed, along up
            Vector3D hErrVec = nav.HorizontalTo(targetPos);
            double hErr = hErrVec.Length();
            Vector3D hDir = hErr > 1e-3 ? hErrVec / hErr : Vector3D.Zero;

            double vDes = System.Math.Abs(vErr) <= posDeadband ? 0.0 : Clamp(vErr * ApproachGain, -maxSpeed, maxSpeed);
            double hDes = hErr <= posDeadband ? 0.0 : System.Math.Min(maxSpeed, hErr * ApproachGain);

            double vAct = nav.VertSpeed;
            Vector3D hVelVec = nav.Velocity - up * vAct;
            double hAct = Vector3D.Dot(hVelVec, hDir);           // velocity toward the target (horizontal)
            Vector3D hDrift = hVelVec - hDir * hAct;             // sideways drift to cancel

            double vVelErr = vDes - vAct; if (System.Math.Abs(vVelErr) < velDeadband) vVelErr = 0;
            double hVelErr = hDes - hAct; if (System.Math.Abs(hVelErr) < velDeadband) hVelErr = 0;

            Vector3D accel = up * (vVelErr * VelKp)
                           + hDir * (hVelErr * VelKp)
                           - hDrift * VelKp;                     // kill sideways drift

            float interference;
            Vector3D g = MyAPIGateway.Physics.CalculateNaturalGravityAt(grid.GetPosition(), out interference);
            accel -= g;                                          // full gravity compensation
            return ApplyForce(grid, accel * grid.Physics.Mass);
        }

        private static double Clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

        // Like Face, but once aligned within holdDot it STOPS issuing gyro override (lets it coast)
        // instead of micro-correcting every tick — kills the "nervous" oscillation and saves power.
        // Does not touch thrusters (Navigate owns those).
        public double FaceHold(IMyCubeGrid grid, Vector3D aimAxis, Vector3D targetDir, double holdDot)
        {
            double dot = Vector3D.Dot(aimAxis, targetDir);
            if (dot >= holdDot) { ClearGyros(grid); return dot; }
            return Align(grid, aimAxis, targetDir);
        }

        private void ClearGyros(IMyCubeGrid grid)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return;
            var gyros = new List<IMyGyro>();
            ts.GetBlocksOfType(gyros);
            for (int i = 0; i < gyros.Count; i++)
            {
                gyros[i].GyroOverride = false;
                gyros[i].Pitch = 0; gyros[i].Yaw = 0; gyros[i].Roll = 0;
            }
        }

        public void Release(IMyCubeGrid grid)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return;

            var gyros = new List<IMyGyro>();
            ts.GetBlocksOfType(gyros);
            for (int i = 0; i < gyros.Count; i++)
            {
                gyros[i].GyroOverride = false;
                gyros[i].Pitch = 0; gyros[i].Yaw = 0; gyros[i].Roll = 0;
            }

            var thrusters = new List<IMyThrust>();
            ts.GetBlocksOfType(thrusters);
            for (int i = 0; i < thrusters.Count; i++)
                thrusters[i].ThrustOverridePercentage = 0f;
        }

        private double Align(IMyCubeGrid grid, Vector3D current, Vector3D target)
        {
            double dot = Vector3D.Dot(current, target);
            Vector3D axis = Vector3D.Cross(current, target);
            double angle = axis.Normalize(); // sin(theta) for small angles
            if (dot < 0) angle = 3.14159 - angle; // pointing backwards: full turn needed

            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return dot;
            var gyros = new List<IMyGyro>();
            ts.GetBlocksOfType(gyros);

            Vector3D angVel = grid.Physics != null ? (Vector3D)grid.Physics.AngularVelocity : Vector3D.Zero;
            Vector3D command = axis * angle * GyroGain - angVel * DampGain;

            for (int i = 0; i < gyros.Count; i++)
            {
                var g = gyros[i];
                Vector3D local = Vector3D.TransformNormal(command, MatrixD.Transpose(g.WorldMatrix));
                g.GyroOverride = true;
                g.Pitch = MathHelper.Clamp((float)-local.X, -MaxGyroRpm, MaxGyroRpm);
                g.Yaw   = MathHelper.Clamp((float)-local.Y, -MaxGyroRpm, MaxGyroRpm);
                g.Roll  = MathHelper.Clamp((float)-local.Z, -MaxGyroRpm, MaxGyroRpm);
            }
            return dot;
        }

        // Bang-bang velocity control along targetDir: override thrusters pushing that way
        // while below targetSpeed, release them when at/above (dampeners brake everything else).
        private void Advance(IMyCubeGrid grid, Vector3D targetDir, double targetSpeed)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null || grid.Physics == null) return;

            double speedAlong = Vector3D.Dot(grid.Physics.LinearVelocity, targetDir);
            bool push = targetSpeed > 0 && speedAlong < targetSpeed;

            var thrusters = new List<IMyThrust>();
            ts.GetBlocksOfType(thrusters);
            for (int i = 0; i < thrusters.Count; i++)
            {
                var t = thrusters[i];
                // A thruster pushes the ship along its WorldMatrix.Backward.
                double d = Vector3D.Dot(t.WorldMatrix.Backward, targetDir);
                t.ThrustOverridePercentage = (push && d > 0.9) ? 0.15f : 0f;
            }
        }
    }
}
