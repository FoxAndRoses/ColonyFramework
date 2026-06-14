using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.Game.ModAPI;
using VRage.ModAPI;
using VRageMath;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

namespace ColonyFramework
{
    public class ColonyCommands
    {
        private readonly ColonyRegistry _registry;
        private readonly OreScanner _scanner = new OreScanner();
        private readonly ResourceTracker _tracker = new ResourceTracker();
        private readonly DispatchService _dispatch = new DispatchService();
        private readonly DroneExecutor _executor;

        public ColonyCommands(ColonyRegistry registry, DroneExecutor executor) { _registry = registry; _executor = executor; }

        // Resolve the commanding player's active colony, or null with a message.
        private Colony PlayerColony(out string error)
        {
            error = null;
            var player = MyAPIGateway.Session != null ? MyAPIGateway.Session.Player : null;
            if (player == null) { error = "no player"; return null; }
            long ownerKey = Ownership.ResolveOwnerKey(player.IdentityId);
            var colony = _registry.Get(ownerKey);
            if (colony == null || !colony.Active)
            {
                error = "no active colony - build and power a Colony Core";
                return null;
            }
            return colony;
        }

        public void Handle(string messageText, ref bool sendToOthers)
        {
            if (messageText == null) return;
            var text = messageText.Trim().ToLowerInvariant();
            if (!text.StartsWith("/colony ")) return;
            sendToOthers = false;

            string error;
            var colony = PlayerColony(out error);
            if (colony == null)
            {
                MyAPIGateway.Utilities.ShowMessage("Colony", error);
                return;
            }

            if (text == "/colony resources")
            {
                var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as IMyCubeBlock;
                if (core != null)
                    _tracker.Scan(core.CubeGrid, colony.OwnerKey, colony.Resources, MyAPIGateway.Session.GameDateTime.Ticks);

                var r = colony.Resources;
                MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                    "ores: {0} ({1:N0}kg) | ingots: {2} ({3:N0}kg) | building materials: {4} ({5:N0})",
                    r.Ore.Count, r.Total(r.Ore), r.Ingots.Count, r.Total(r.Ingots),
                    r.Components.Count, r.Total(r.Components)));
                return;
            }

            if (text == "/colony info")
            {
                long ageTicks = MyAPIGateway.Session.GameDateTime.Ticks - colony.State.CreatedGameTicks;
                if (ageTicks < 0) ageTicks = 0;
                var age = new System.TimeSpan(ageTicks);
                MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                    "{0} (id {1}) | founder {2} | age {3}d {4}h | deposits {5}",
                    string.IsNullOrEmpty(colony.State.Name) ? "Unnamed" : colony.State.Name,
                    colony.State.ColonyId, colony.State.FounderId, age.Days, age.Hours,
                    colony.Deposits.Deposits.Count));
                return;
            }

            if (text == "/colony scan")
            {
                var player = MyAPIGateway.Session.Player;
                Vector3D center = player.GetPosition();
                int cells = _scanner.Scan(colony.Deposits, center, 25.0, 2, 0, 0);
                MyAPIGateway.Utilities.ShowMessage("ColonyScan", string.Format(
                    "hit {0} ore cells; deposits in DB: {1}", cells, colony.Deposits.Deposits.Count));
                return;
            }

            if (text == "/colony missions")
            {
                int pending = 0, assigned = 0;
                var ms = colony.Missions.Missions;
                for (int i = 0; i < ms.Count; i++)
                {
                    if (ms[i].Status == MissionStatus.PendingAssignment) pending++;
                    else if (ms[i].Status == MissionStatus.Assigned) assigned++;
                }
                MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                    "deposits: {0}, missions: {1} (pending {2}, assigned {3})",
                    colony.Deposits.Deposits.Count, ms.Count, pending, assigned));
                return;
            }

            if (text == "/colony register")
            {
                var player = MyAPIGateway.Session.Player;
                Vector3D ppos = player.GetPosition();
                var ents = new HashSet<IMyEntity>();
                MyAPIGateway.Entities.GetEntities(ents, e => e is IMyCubeGrid);
                IMyCubeGrid nearest = null;
                double bestSq = 100.0 * 100.0;
                foreach (var e in ents)
                {
                    var g = e as IMyCubeGrid;
                    if (g == null) continue;
                    double sq = Vector3D.DistanceSquared(g.GetPosition(), ppos);
                    if (sq < bestSq) { bestSq = sq; nearest = g; }
                }
                if (nearest == null)
                {
                    MyAPIGateway.Utilities.ShowMessage("Colony", "no grid within 100m to register");
                    return;
                }
                colony.Assets.Register(nearest.EntityId, AssetType.Miner, nearest.GetPosition(), nearest.DisplayName);
                _executor.ReleaseControls(nearest); // clear any stale overrides so it won't thrust on its own
                MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                    "registered '{0}' as miner ({1} assets total)", nearest.DisplayName, colony.Assets.Assets.Count));
                return;
            }

            if (text == "/colony assets")
            {
                int idle = 0, assigned = 0, offline = 0;
                var a = colony.Assets.Assets;
                for (int i = 0; i < a.Count; i++)
                {
                    switch (a[i].Status)
                    {
                        case AssetStatus.Idle: idle++; break;
                        case AssetStatus.Assigned: assigned++; break;
                        case AssetStatus.Offline: offline++; break;
                    }
                }
                MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                    "assets: {0} (idle {1}, assigned {2}, offline {3})", a.Count, idle, assigned, offline));
                return;
            }

            if (text == "/colony dispatch")
            {
                MyAPIGateway.Utilities.ShowMessage("Colony", _dispatch.DispatchFirstAssigned(colony));
                return;
            }

            if (text == "/colony abort")
            {
                _executor.AbortAll(colony);
                MyAPIGateway.Utilities.ShowMessage("Colony", "aborted all active missions");
                return;
            }

            if (text == "/colony recall")
            {
                _executor.RecallAll(colony);
                MyAPIGateway.Utilities.ShowMessage("Colony", "recalling active drones");
                return;
            }
        }
    }
}
