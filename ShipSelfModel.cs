using System;
using System.Collections.Generic;
using System.Text;
using Sandbox.ModAPI;
using VRageMath;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

namespace ColonyFramework
{
    // FLIGHT.md §4 — "what a ship believes it is". Built from the grid at commissioning (cheap,
    // once per launch): mass, thrust per body axis, TWR, braking capability, tools and ports.
    // Every capability number the flight model will ever use comes from HERE, not from constants.
    // Body axes are the REMOTE CONTROL's frame: 0=Forward 1=Backward 2=Up 3=Down 4=Left 5=Right
    // (a thruster "on axis Up" pushes the ship toward RC-up, i.e. it lifts).
    public class ShipSelfModel
    {
        public static readonly string[] AxisName = { "forward", "backward", "up", "down", "left", "right" };
        private const double SafetyDerate = 0.7;   // never plan on more than 70% of theoretical accel
        private const double MinHoverTwr = 1.15;   // below this the ship cannot reliably hold altitude

        // Body
        public double Mass;             // kg, physics (includes cargo — rebuild when load changes)
        public double PhysicalRadius;   // m, AABB half-diagonal
        public VRage.Game.MyCubeSize GridSize;

        // Propulsion
        public readonly double[] ThrustN = new double[6]; // Newtons per body axis
        public int GyroCount;
        public double GravityG;         // m/s², local (0 in space)
        public double TwrUp;            // up-thrust vs weight (999 in space)

        // Power
        public double BatteryCapMWh, BatteryStoredMWh;
        public bool HasReactor;

        // Tools & ports
        public int Connectors, Welders, Drills, OreDetectors, Cameras;
        public double CargoFillFrac;

        // Flags (derived)
        public bool CanHoverNow;        // TWR_up at CURRENT mass clears the floor
        public bool CanPitchWork;       // thrust on all 6 axes — can maneuver at any attitude
        public bool HasForwardSensor;   // camera aboard (F2's terrain raycast probe)

        // Net acceleration the ship can produce along a body axis, derated for planning.
        public double Accel(int axis) { return Mass > 0 ? ThrustN[axis] / Mass * SafetyDerate : 0; }

        // Stopping distance from speed v using the thrusters that oppose FORWARD flight.
        public double BrakingDistance(double v)
        {
            double a = Accel(1); // "backward" axis thrusters brake forward motion
            return a > 0.01 ? v * v / (2 * a) : double.MaxValue;
        }

        public static ShipSelfModel Build(IMyCubeGrid grid)
        {
            var rc = DroneUtil.FindRc(grid);
            if (rc == null || grid.Physics == null) return null;

            var m = new ShipSelfModel();
            m.Mass = grid.Physics.Mass;
            m.PhysicalRadius = grid.WorldAABB.HalfExtents.Length();
            m.GridSize = grid.GridSizeEnum;

            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            if (ts == null) return null;

            // Thrust per body axis: a thruster pushes along its WorldMatrix.Backward; classify that
            // push direction against the RC's frame (the drone's "body").
            var rcM = rc.WorldMatrix;
            Vector3D[] axes = { rcM.Forward, rcM.Backward, rcM.Up, rcM.Down, rcM.Left, rcM.Right };
            var thrusters = new List<IMyThrust>();
            ts.GetBlocksOfType(thrusters);
            for (int i = 0; i < thrusters.Count; i++)
            {
                Vector3D push = thrusters[i].WorldMatrix.Backward;
                int best = 0; double bestDot = -2;
                for (int a = 0; a < 6; a++)
                {
                    double d = Vector3D.Dot(push, axes[a]);
                    if (d > bestDot) { bestDot = d; best = a; }
                }
                if (bestDot > 0.7) m.ThrustN[best] += thrusters[i].MaxEffectiveThrust;
            }

            var gyros = new List<IMyGyro>();
            ts.GetBlocksOfType(gyros);
            m.GyroCount = gyros.Count;

            float interference;
            Vector3D g = MyAPIGateway.Physics.CalculateNaturalGravityAt(grid.GetPosition(), out interference);
            m.GravityG = g.Length();
            // TWR uses the thrusters opposing gravity RIGHT NOW (world-up projected onto body axes):
            // for a level drone that's the "up" axis; a napped/odd-attitude drone still reads sanely
            // because commissioning wakes it level on the pad.
            m.TwrUp = m.GravityG > 0.05
                ? m.ThrustN[2] / (m.Mass * m.GravityG)
                : 999.0;

            double batOut, reactorOut;
            DroneUtil.MeasurePower(grid, out m.BatteryStoredMWh, out m.BatteryCapMWh, out reactorOut, out batOut);
            m.HasReactor = DroneUtil.HasInfinitePower(grid);

            m.Connectors = DroneUtil.FindConnectors(grid).Count;
            m.Welders = DroneUtil.FindWelders(grid).Count;
            m.Drills = DroneUtil.FindDrills(grid).Count;
            m.OreDetectors = DroneUtil.FindOreDetector(grid) != null ? 1 : 0;
            var cams = new List<IMyCameraBlock>();
            ts.GetBlocksOfType(cams);
            m.Cameras = cams.Count;
            m.CargoFillFrac = DroneUtil.CargoFill(grid);

            m.CanHoverNow = m.TwrUp >= MinHoverTwr;
            m.CanPitchWork = true;
            for (int a = 0; a < 6; a++) if (m.ThrustN[a] <= 0) m.CanPitchWork = false;
            m.HasForwardSensor = m.Cameras > 0;
            return m;
        }

        // First NAMED deficiency that makes flight unsafe, or null if fit to fly. Ordered so the
        // most fundamental problem is reported first (FLIGHT.md: abort with a named deficiency).
        public string Deficiency()
        {
            if (GyroCount == 0) return "no gyros — cannot orient";
            for (int a = 0; a < 6; a++)
                if (ThrustN[a] <= 0)
                    return string.Format("no {0} thrust — dampeners cannot brake that direction", AxisName[a]);
            if (GravityG > 0.05 && TwrUp < MinHoverTwr)
                return string.Format("up-TWR {0:F2} at current mass ({1:F1} t) — add up thrusters or shed cargo",
                    TwrUp, Mass / 1000.0);
            if (BatteryCapMWh <= 0 && !HasReactor) return "no power source (battery or reactor)";
            return null;
        }

        // Compact capability line for /colony assets and the AssetRecord (persisted).
        public string Summary()
        {
            var sb = new StringBuilder(128);
            sb.Append(string.Format("{0:F1}t", Mass / 1000.0));
            sb.Append(GravityG > 0.05 ? string.Format(" TWR {0:F1}", TwrUp) : " (space)");
            sb.Append(string.Format(" brake {0:F0}m@30", Math.Min(999, BrakingDistance(30))));
            int axesWithThrust = 0; for (int a = 0; a < 6; a++) if (ThrustN[a] > 0) axesWithThrust++;
            sb.Append(string.Format(" axes {0}/6", axesWithThrust));
            if (Drills > 0) sb.Append(" drills:" + Drills);
            if (Welders > 0) sb.Append(" welders:" + Welders);
            if (OreDetectors > 0) sb.Append(" oredet");
            sb.Append(" con:" + Connectors);
            sb.Append(Cameras > 0 ? " cam:" + Cameras : " NO-CAM");
            sb.Append(HasReactor ? " reactor" : string.Format(" batt {0:F1}MWh", BatteryCapMWh));
            return sb.ToString();
        }
    }

    // FLIGHT.md §4 launch self-test, flight portion: after unlock, a short measured hop — command
    // ~hover+2 m/s² of up-thrust for 1.5 s with dampeners off, measure the achieved vertical
    // acceleration, and compare against what the model PREDICTS. Catches what static math can't:
    // damaged thrusters, wrong orientation assumptions, hidden mass. Skipped in zero-g.
    public class LaunchSelfTest
    {
        private const double HopSecs = 1.5, SettleSecs = 1.5;
        private const double RequestNetAccel = 2.0;  // m/s² above hover we ask for
        private const double PassFraction = 0.5;     // measured must reach 50% of predicted

        private int _state; // 0=hop 1=settle 2=passed 3=failed
        private DateTime _phaseStart;
        private double _v0Up, _expectedNet;
        private Vector3D _up;

        public string FailReason { get; private set; }
        public double MeasuredAccel { get; private set; }
        public bool Done { get { return _state >= 2; } }
        public bool Passed { get { return _state == 2; } }

        public void Begin(IMyCubeGrid grid, ShipSelfModel model)
        {
            if (grid.Physics == null || model == null) { _state = 3; FailReason = "no physics/model"; return; }
            if (model.GravityG <= 0.05) { _state = 2; MeasuredAccel = RequestNetAccel; return; } // zero-g: nothing to hop against

            float interference;
            Vector3D g = MyAPIGateway.Physics.CalculateNaturalGravityAt(grid.GetPosition(), out interference);
            _up = -Vector3D.Normalize(g);

            // Up-thrusters (push direction ≈ world up right now — drone sits level on the pad).
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            var thrusters = new List<IMyThrust>();
            if (ts != null) ts.GetBlocksOfType(thrusters);
            double upThrustTotal = 0;
            var ups = new List<IMyThrust>();
            for (int i = 0; i < thrusters.Count; i++)
                if (Vector3D.Dot(thrusters[i].WorldMatrix.Backward, _up) > 0.7)
                { ups.Add(thrusters[i]); upThrustTotal += thrusters[i].MaxEffectiveThrust; }
            if (upThrustTotal <= 0) { _state = 3; FailReason = "no lift thrusters found at launch attitude"; return; }

            double needF = grid.Physics.Mass * (model.GravityG + RequestNetAccel);
            double frac = Math.Min(1.0, needF / upThrustTotal);
            _expectedNet = Math.Min(RequestNetAccel, upThrustTotal / grid.Physics.Mass - model.GravityG);

            var rc = DroneUtil.FindRc(grid);
            if (rc != null) rc.DampenersOverride = false; // dampeners would fight the measured pulse
            for (int i = 0; i < ups.Count; i++)
                ups[i].ThrustOverride = (float)(ups[i].MaxEffectiveThrust * frac);

            _v0Up = Vector3D.Dot(grid.Physics.LinearVelocity, _up);
            _phaseStart = DateTime.UtcNow;
            _state = 0;
        }

        public void Tick(IMyCubeGrid grid, ShipSelfModel model)
        {
            if (Done || grid == null || grid.Physics == null) return;

            if (_state == 0) // hop: pulse running
            {
                double t = (DateTime.UtcNow - _phaseStart).TotalSeconds;
                if (t < HopSecs) return;
                double dv = Vector3D.Dot(grid.Physics.LinearVelocity, _up) - _v0Up;
                MeasuredAccel = dv / t;
                EndPulse(grid);
                _phaseStart = DateTime.UtcNow;
                _state = 1;
                return;
            }

            // settle: dampeners arrest the hop, then judge
            if ((DateTime.UtcNow - _phaseStart).TotalSeconds < SettleSecs) return;
            if (MeasuredAccel >= _expectedNet * PassFraction) _state = 2;
            else
            {
                _state = 3;
                FailReason = string.Format(
                    "measured lift {0:F1} m/s² vs predicted {1:F1} — thrusters damaged, blocked, or ship heavier than modeled",
                    MeasuredAccel, _expectedNet);
            }
        }

        private static void EndPulse(IMyCubeGrid grid)
        {
            var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
            var thrusters = new List<IMyThrust>();
            if (ts != null) ts.GetBlocksOfType(thrusters);
            for (int i = 0; i < thrusters.Count; i++) thrusters[i].ThrustOverride = 0f;
            var rc = DroneUtil.FindRc(grid);
            if (rc != null) rc.DampenersOverride = true;
        }
    }
}
