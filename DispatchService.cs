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

        // Autonomous dispatch: launch every Assigned mission whose asset is present and flyable (sets it
        // InProgress → commissioning). Called each assignment cycle so a drone that just recharged and
        // freed up gets its next mission without a manual /colony dispatch. Returns count launched.
        public int AutoDispatchAssigned(Colony colony)
        {
            int launched = 0;
            var ms = colony.Missions.Missions;
            for (int i = 0; i < ms.Count; i++)
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
                MyLog.Default.WriteLineAndConsole(string.Format(
                    "[ColonyFramework] Auto-dispatching '{0}' for deposit {1} (mission {2})",
                    grid.DisplayName, mission.TargetDepositId, mission.Id));
            }
            return launched;
        }
    }
}
