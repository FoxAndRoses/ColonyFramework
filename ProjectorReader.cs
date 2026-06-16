using System.Collections.Generic;
using Sandbox.ModAPI;
using Sandbox.Definitions;
using VRage.Game.ModAPI;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;
using IMySlimBlock = VRage.Game.ModAPI.IMySlimBlock;

namespace ColonyFramework
{
    // Reads the projector(s) on a grid group and sums the component bill-of-materials of the projected
    // blueprint(s). Services layer: touches the SE API (projectors, block definitions); returns plain
    // data (subtype -> count) for the caller to compare against colony stock. The demand foundation for
    // the production pipeline — it does not queue assemblers or move anything.
    public class ProjectorReader
    {
        private readonly List<IMyCubeGrid> _grids = new List<IMyCubeGrid>();
        private readonly List<IMyProjector> _projectors = new List<IMyProjector>();
        private readonly List<IMySlimBlock> _blocks = new List<IMySlimBlock>();

        // subtype -> total components required to build every projected block across all active projectors
        // on the core's physical group. 'projectors' = active projectors counted; 'blocks' = projected
        // blocks summed. NOTE: this is the FULL blueprint cost (every block), not just the unbuilt ones.
        public Dictionary<string, double> RequiredComponents(IMyCubeGrid coreGrid, out int projectors, out int blocks)
        {
            var required = new Dictionary<string, double>();
            projectors = 0;
            blocks = 0;
            if (coreGrid == null) return required;

            _grids.Clear();
            var group = coreGrid.GetGridGroup(GridLinkTypeEnum.Physical);
            if (group != null) group.GetGrids(_grids);
            if (_grids.Count == 0) _grids.Add(coreGrid);

            for (int g = 0; g < _grids.Count; g++)
            {
                var grid = _grids[g];
                if (grid == null) continue;
                var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
                if (ts == null) continue;

                _projectors.Clear();
                ts.GetBlocksOfType(_projectors);
                for (int p = 0; p < _projectors.Count; p++)
                {
                    var proj = _projectors[p];
                    if (proj == null || !proj.IsProjecting) continue;
                    var projected = proj.ProjectedGrid;
                    if (projected == null) continue;
                    projectors++;

                    _blocks.Clear();
                    projected.GetBlocks(_blocks);
                    for (int b = 0; b < _blocks.Count; b++)
                    {
                        var def = _blocks[b].BlockDefinition as MyCubeBlockDefinition;
                        if (def == null || def.Components == null) continue;
                        blocks++;
                        for (int c = 0; c < def.Components.Length; c++)
                        {
                            var comp = def.Components[c];
                            if (comp.Definition == null) continue;
                            Add(required, comp.Definition.Id.SubtypeName, comp.Count);
                        }
                    }
                }
            }
            return required;
        }

        private static void Add(Dictionary<string, double> dict, string key, double amount)
        {
            double cur;
            dict.TryGetValue(key, out cur);
            dict[key] = cur + amount;
        }
    }
}
