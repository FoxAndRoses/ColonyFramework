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
        public void ThrustAlong(IMyCubeGrid grid, Vector3D dir, double targetSpeed)
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
