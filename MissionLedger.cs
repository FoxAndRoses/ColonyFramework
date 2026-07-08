using System;
using Sandbox.ModAPI;
using VRageMath;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

namespace ColonyFramework
{
    // MISSION.md D4 — the battery is a fuel tank. Replaces the flat 20% abort: a drone 2 km from
    // home aborts EARLIER than one 200 m out, a reactor drone never aborts, and every trip is
    // NAMED with its numbers. Cost model: measured/live draw × (distance home at a conservative
    // cruise + a docking reserve) × safety margin, plus an absolute hard floor.
    public static class MissionLedger
    {
        private const double CruiseEstimate = 30.0;   // m/s conservative planning speed
        private const double DockReserveSecs = 120.0; // shimmy/align/reverse time at the pad
        private const double SafetyMargin = 1.5;
        private const double HardFloorPct = 0.10;     // below this stored fraction, return regardless

        // Null = mission may continue; otherwise the NAMED reason to return now.
        // measuredDrawMw: the commissioning spike draw if the caller has one, else <=0 to use the
        // LIVE draw (thrusters+tools running now — representative mid-work).
        public static string ShouldReturn(IMyCubeGrid grid, Vector3D homePos, double measuredDrawMw)
        {
            if (grid == null || DroneUtil.HasInfinitePower(grid)) return null;

            double stored, cap, reactorOut, batOut;
            DroneUtil.MeasurePower(grid, out stored, out cap, out reactorOut, out batOut);
            if (cap <= 0) return null; // no batteries at all and no reactor → commissioning already rejected it

            double pct = stored / cap;
            if (pct < HardFloorPct)
                return string.Format("energy floor: {0:P0} stored", pct);

            double drawMw = measuredDrawMw > 0.01 ? measuredDrawMw : Math.Max(0.05, batOut + reactorOut);
            double dist = Vector3D.Distance(grid.GetPosition(), homePos);
            double needMwh = drawMw * ((dist / CruiseEstimate + DockReserveSecs) / 3600.0) * SafetyMargin;
            if (stored < needMwh)
                return string.Format("energy ledger: {0:P0} stored ({1:F2} MWh), return over {2:F0} m needs {3:F2} MWh",
                    pct, stored, dist, needMwh);
            return null;
        }
    }
}
