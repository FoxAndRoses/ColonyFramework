using Sandbox.ModAPI;
using VRageMath;
using MyPlanetElevation = Sandbox.ModAPI.Ingame.MyPlanetElevation;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

namespace ColonyFramework
{
    // Navigation Awareness Layer: the snapshot the drone computes about ITSELF each tick — where it
    // is, how fast it's moving, which way it's facing, how high it is. Every mission phase consumes
    // this single source of truth instead of re-deriving the same numbers, and validates it before
    // issuing any movement. The drone must understand itself before it can control itself.
    public class NavState
    {
        public bool Valid;            // rc + physics present and usable

        // PositionState
        public Vector3D Com;          // grid centre of mass (world)
        public Vector3D ConnectorPos; // drone connector position — the docking distance reference; = Com if none

        // VelocityState
        public Vector3D Velocity;     // world linear velocity
        public double Speed;          // |Velocity|
        public double VertSpeed;      // component along GravityUp (+ = rising)
        public double HorizSpeed;     // magnitude in the gravity plane

        // OrientationState
        public Vector3D Forward, Up, GravityUp;
        public double LevelDot;       // grid Up vs anti-gravity (1 = perfectly upright)

        // MassState — TRUE physical mass incl. cargo (Physics.Mass under-reports a loaded grid,
        // which made the loaded descent under-thrust and never brake).
        public double Mass;

        // AltitudeState
        public bool AltValid;         // planet elevation readable (false in space)
        public double Agl;            // metres above surface

        public void Refresh(IMyCubeGrid grid, IMyRemoteControl rc, IMyShipConnector droneCon)
        {
            Valid = false;
            if (grid == null || grid.Physics == null || rc == null) return;

            Com = grid.Physics.CenterOfMassWorld;
            ConnectorPos = droneCon != null ? droneCon.GetPosition() : Com;

            Forward = grid.WorldMatrix.Forward;
            Up = grid.WorldMatrix.Up;

            float interference;
            Vector3D g = MyAPIGateway.Physics.CalculateNaturalGravityAt(Com, out interference);
            GravityUp = g.LengthSquared() > 0.01 ? -Vector3D.Normalize(g) : Up; // space: no gravity frame, fall back to ship up
            LevelDot = Vector3D.Dot(Up, GravityUp);

            Mass = grid.Physics.Mass;
            var sm = rc.CalculateShipMass();           // accurate total mass incl. cargo
            if (sm.PhysicalMass > 0) Mass = sm.PhysicalMass;

            Velocity = (Vector3D)grid.Physics.LinearVelocity;
            Speed = Velocity.Length();
            VertSpeed = Vector3D.Dot(Velocity, GravityUp);
            HorizSpeed = (Velocity - GravityUp * VertSpeed).Length();

            double agl;
            AltValid = rc.TryGetPlanetElevation(MyPlanetElevation.Surface, out agl);
            Agl = AltValid ? agl : 0;

            Valid = true;
        }

        // Signed vertical distance from the connector to a world point (+ = the point is above us).
        public double VerticalError(Vector3D worldPoint)
        {
            return Vector3D.Dot(worldPoint - ConnectorPos, GravityUp);
        }

        // Horizontal (gravity-plane) offset from the connector to a world point.
        public Vector3D HorizontalTo(Vector3D worldPoint)
        {
            Vector3D d = worldPoint - ConnectorPos;
            return d - GravityUp * Vector3D.Dot(d, GravityUp);
        }

        // One-line self-report: the drone narrating what it knows about itself and its target.
        public string Report(long missionId, string phase, Vector3D target)
        {
            return string.Format(
                "[ColonyFramework] Mission {0}: {1} | spd={2:F1} (v={3:F1} h={4:F1}) level={5:F2} agl={6} | distV={7:F1} distH={8:F1}",
                missionId, phase, Speed, VertSpeed, HorizSpeed, LevelDot,
                AltValid ? Agl.ToString("F0") : "?", VerticalError(target), HorizontalTo(target).Length());
        }
    }
}
