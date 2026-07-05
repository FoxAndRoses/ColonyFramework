using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.Utils;
using VRageMath;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

namespace ColonyFramework
{
    // Initiates execution of an assigned mission: validates the asset, then hands off to the
    // executor's Commissioning phase (power self-test). The executor releases the grid and
    // flies it only if commissioning passes — so a weak drone stays put rather than launching.
    public class DispatchService
    {
        private const int PhaseCommission = 0;

        // Returns a human-readable result for chat/log.
        public string DispatchFirstAssigned(Colony colony)
        {
            var ms = colony.Missions.Missions;
            Mission mission = null;
            for (int i = 0; i < ms.Count; i++)
                if (ms[i].Status == MissionStatus.Assigned) { mission = ms[i]; break; }
            if (mission == null)
                for (int i = 0; i < ms.Count; i++)
                    if (ms[i].Status == MissionStatus.InProgress) { mission = ms[i]; break; } // retry stuck mission
            if (mission == null) return "no assigned or in-progress missions to dispatch";

            var deposit = colony.Deposits.GetById(mission.TargetDepositId);
            if (deposit == null) return "mission " + mission.Id + " has no deposit";

            var grid = MyAPIGateway.Entities.GetEntityById(mission.AssignedAssetId) as IMyCubeGrid;
            if (grid == null) return "asset grid not found for mission " + mission.Id;

            if (DroneUtil.FindRc(grid) == null) return "asset '" + grid.DisplayName + "' has no Remote Control";

            colony.Missions.SetInProgress(mission.Id);
            mission.Phase = PhaseCommission;

            string msg = string.Format(
                "commissioning '{0}' for deposit {1} ({2}) — power self-test before launch",
                grid.DisplayName, deposit.Id, deposit.OreType);
            MyLog.Default.WriteLineAndConsole("[ColonyFramework] " + msg);
            return msg;
        }

        private readonly System.Collections.Generic.Dictionary<long, System.DateTime> _lastLaunch
            = new System.Collections.Generic.Dictionary<long, System.DateTime>();
        private const double LaunchSpacingSecs = 12.0; // min gap between launches per colony (takeoff separation)

        // Autonomous dispatch: launch Assigned missions whose assets are present and flyable (sets
        // them InProgress → commissioning). STAGGERED: at most one launch per colony per
        // LaunchSpacingSecs — two drones lifting off the same second from adjacent pads collided in
        // testing (avoidance can't help at 0 m separation). Returns count launched (0 or 1).
        public int AutoDispatchAssigned(Colony colony)
        {
            System.DateTime last;
            if (_lastLaunch.TryGetValue(colony.OwnerKey, out last)
                && (System.DateTime.UtcNow - last).TotalSeconds < LaunchSpacingSecs) return 0;

            int launched = 0;
            var ms = colony.Missions.Missions;
            for (int i = 0; i < ms.Count && launched == 0; i++)
            {
                var mission = ms[i];
                if (mission.Status != MissionStatus.Assigned) continue;

                var asset = colony.Assets.GetByEntityId(mission.AssignedAssetId);
                if (asset == null || !asset.AutoDispatchEnabled) continue; // autonomy not enabled / drone parked

                var grid = MyAPIGateway.Entities.GetEntityById(mission.AssignedAssetId) as IMyCubeGrid;
                if (grid == null || DroneUtil.FindRc(grid) == null) continue; // asset gone/unflyable — leave assigned

                colony.Missions.SetInProgress(mission.Id);
                mission.Phase = PhaseCommission;
                launched++;
                _lastLaunch[colony.OwnerKey] = System.DateTime.UtcNow;
                MyLog.Default.WriteLineAndConsole(string.Format(
                    "[ColonyFramework] Auto-dispatching '{0}' (mission {1}, {2})",
                    grid.DisplayName, mission.Id, mission.Type));
            }
            return launched;
        }
    }
}
