using System.Collections.Generic;
using VRage.Game.ModAPI.Ingame;
using IMyCubeBlock = VRage.Game.ModAPI.IMyCubeBlock;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;
using IMySlimBlock = VRage.Game.ModAPI.IMySlimBlock;

namespace ColonyFramework
{
    public class ResourceTracker
    {
        private readonly List<IMyCubeGrid> _grids = new List<IMyCubeGrid>();
        private readonly List<IMySlimBlock> _blocks = new List<IMySlimBlock>();
        private readonly List<MyInventoryItem> _items = new List<MyInventoryItem>();

        public void Scan(IMyCubeGrid coreGrid, long ownerKey, ResourceSnapshot snapshot, long nowTicks)
        {
            snapshot.Clear();
            if (coreGrid == null) return;

            _grids.Clear();
            var group = coreGrid.GetGridGroup(VRage.Game.ModAPI.GridLinkTypeEnum.Physical);
            if (group != null) group.GetGrids(_grids);
            if (_grids.Count == 0) _grids.Add(coreGrid);

            _blocks.Clear();
            coreGrid.GetBlocks(_blocks);
            int coreBlockCount = _blocks.Count;

            int invBlocks = 0, itemCount = 0, gridsScanned = 0;
            string sample = "none";

            for (int g = 0; g < _grids.Count; g++)
            {
                var grid = _grids[g];
                if (grid == null) continue;
                if (grid != coreGrid && !OwnedBy(grid, ownerKey)) continue;
                gridsScanned++;

                _blocks.Clear();
                grid.GetBlocks(_blocks);
                for (int b = 0; b < _blocks.Count; b++)
                {
                    var fat = _blocks[b].FatBlock as IMyCubeBlock;
                    if (fat == null || !fat.HasInventory) continue;
                    invBlocks++;

                    for (int i = 0; i < fat.InventoryCount; i++)
                    {
                        var inv = fat.GetInventory(i);
                        if (inv == null) continue;
                        _items.Clear();
                        inv.GetItems(_items);
                        for (int it = 0; it < _items.Count; it++)
                        {
                            var item = _items[it];
                            string typeId = item.Type.TypeId;
                            string subtype = item.Type.SubtypeId;
                            double amount = (double)item.Amount;
                            itemCount++;
                            if (sample == "none") sample = typeId + "/" + subtype;

                            if (typeId == "MyObjectBuilder_Ore") Add(snapshot.Ore, subtype, amount);
                            else if (typeId == "MyObjectBuilder_Ingot") Add(snapshot.Ingots, subtype, amount);
                            else if (typeId == "MyObjectBuilder_Component") Add(snapshot.Components, subtype, amount);
                        }
                    }
                }
            }

            snapshot.LastUpdatedTicks = nowTicks;

            VRage.Utils.MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] Resource scan core grid '{0}' ({1} blocks): gridsInGroup={2}, gridsScanned={3}, invBlocks={4}, items={5}, sample={6}, ore={7} ingot={8} comp={9}",
                coreGrid.DisplayName, coreBlockCount,
                _grids.Count, gridsScanned, invBlocks, itemCount, sample,
                snapshot.Ore.Count, snapshot.Ingots.Count, snapshot.Components.Count));
        }

        private static bool OwnedBy(IMyCubeGrid grid, long ownerKey)
        {
            var owners = grid.BigOwners;
            if (owners == null || owners.Count == 0) return false;
            for (int i = 0; i < owners.Count; i++)
                if (Ownership.ResolveOwnerKey(owners[i]) == ownerKey) return true;
            return false;
        }

        private static void Add(Dictionary<string, double> dict, string key, double amount)
        {
            double cur;
            dict.TryGetValue(key, out cur);
            dict[key] = cur + amount;
        }
    }
}
