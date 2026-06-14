using Sandbox.ModAPI;
using VRageMath;

namespace ColonyFramework
{
    public static class NavMath
    {
        public const double StandoffMeters = 120.0;

        // Gravity: standoff straight above the deposit. Space: offset along 'fallback' direction.
        public static Vector3D ComputeStandoff(Vector3D depositPos, Vector3D fallbackFrom)
        {
            float interference;
            Vector3D gravity = MyAPIGateway.Physics.CalculateNaturalGravityAt(depositPos, out interference);
            Vector3D dir;
            if (gravity.LengthSquared() > 0.01)
                dir = -Vector3D.Normalize(gravity);
            else
            {
                dir = fallbackFrom - depositPos;
                dir = dir.LengthSquared() < 1.0 ? Vector3D.Up : Vector3D.Normalize(dir);
            }
            return depositPos + dir * StandoffMeters;
        }
    }
}
