using System;
using System.Text;
using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.Game.GUI.TextPanel;
using VRage.Utils;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;
using IMyCubeBlock = VRage.Game.ModAPI.IMyCubeBlock;
using IMyTextSurfaceProvider = Sandbox.ModAPI.Ingame.IMyTextSurfaceProvider;
using IMyTerminalBlock = Sandbox.ModAPI.IMyTerminalBlock;

namespace ColonyFramework
{
    // Colony dashboards on vanilla LCDs: any base-group block whose name contains "[Colony]" and has
    // a text surface gets a live status board (missions, fleet, resources, power, survey coverage) on
    // the slow production cadence. Pure observability — the colony visibly runs itself. Also owns the
    // throttled base-low-power chat warning, since it computes the power picture anyway.
    public class LcdService
    {
        private const string Tag = "[Colony]";
        private const double LowPowerFrac = 0.20;
        private const double WarnThrottleSecs = 300.0;

        private readonly StringBuilder _sb = new StringBuilder(1024);
        private readonly Dictionary<long, DateTime> _lastWarn = new Dictionary<long, DateTime>();

        public void Tick(Colony colony)
        {
            if (colony == null || !colony.Active) return;
            var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as IMyCubeBlock;
            if (core == null) return;

            double stored, cap, batOut, reactorOut;
            DroneUtil.GroupPower(core.CubeGrid, out stored, out cap, out batOut, out reactorOut);
            WarnIfLowPower(colony, stored, cap);

            string text = BuildDashboard(colony, stored, cap, batOut, reactorOut);
            WriteToTaggedSurfaces(core.CubeGrid, text);
        }

        private string BuildDashboard(Colony colony, double stored, double cap, double batOut, double reactorOut)
        {
            _sb.Clear();
            _sb.Append("== ").Append(string.IsNullOrEmpty(colony.State.Name) ? "Colony" : colony.State.Name).Append(" ==\n");

            // Missions (active only, capped lines)
            int shown = 0, activeCount = 0;
            var ms = colony.Missions.Missions;
            for (int i = 0; i < ms.Count; i++)
                if (ms[i].Status == MissionStatus.Assigned || ms[i].Status == MissionStatus.InProgress) activeCount++;
            _sb.Append("Missions: ").Append(activeCount).Append('\n');
            for (int i = 0; i < ms.Count && shown < 6; i++)
            {
                var m = ms[i];
                if (m.Status != MissionStatus.Assigned && m.Status != MissionStatus.InProgress) continue;
                shown++;
                string what = m.Type == MissionType.Weld ? "weld"
                    : m.Type == MissionType.Survey ? (m.TargetOre != null ? "survey " + m.TargetOre : "survey")
                    : "mine";
                var grid = MyAPIGateway.Entities.GetEntityById(m.AssignedAssetId) as IMyCubeGrid;
                _sb.Append("  #").Append(m.Id).Append(' ').Append(what).Append(" — ")
                   .Append(grid != null ? grid.DisplayName : "?").Append('\n');
            }

            // Fleet
            int idle = 0, busy = 0, offline = 0;
            var assets = colony.Assets.Assets;
            for (int i = 0; i < assets.Count; i++)
            {
                if (assets[i].Status == AssetStatus.Idle) idle++;
                else if (assets[i].Status == AssetStatus.Assigned) busy++;
                else offline++;
            }
            _sb.Append("Fleet: ").Append(assets.Count).Append(" (").Append(busy).Append(" out, ")
               .Append(idle).Append(" idle").Append(offline > 0 ? ", " + offline + " offline" : "").Append(")\n");

            // Resources one-liner
            var r = colony.Resources;
            _sb.Append("Stock: ore ").Append(FormatK(r.Total(r.Ore)))
               .Append(" | ingots ").Append(FormatK(r.Total(r.Ingots)))
               .Append(" | parts ").Append(FormatK(r.Total(r.Components))).Append('\n');

            // Power
            double pct = cap > 0 ? stored / cap * 100.0 : 0;
            _sb.Append("Power: ").Append(pct.ToString("F0")).Append("% stored");
            if (reactorOut > 0.005) _sb.Append(" | reactor ").Append(reactorOut.ToString("F1")).Append(" MW");
            if (batOut > 0.005) _sb.Append(" | batt out ").Append(batOut.ToString("F1")).Append(" MW");
            _sb.Append('\n');

            // Knowledge
            _sb.Append("Deposits: ").Append(colony.Deposits.Deposits.Count);
            if (colony.State.SurveyedRadius > 0)
                _sb.Append(" | surveyed ").Append(colony.State.SurveyedRadius.ToString("F0")).Append(" m");
            _sb.Append('\n');
            return _sb.ToString();
        }

        private void WriteToTaggedSurfaces(IMyCubeGrid coreGrid, string text)
        {
            var grids = new List<IMyCubeGrid>();
            var group = coreGrid.GetGridGroup(VRage.Game.ModAPI.GridLinkTypeEnum.Physical);
            if (group != null) group.GetGrids(grids);
            if (grids.Count == 0) grids.Add(coreGrid);

            var blocks = new List<IMyTerminalBlock>();
            for (int g = 0; g < grids.Count; g++)
            {
                var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grids[g]);
                if (ts == null) continue;
                blocks.Clear();
                ts.GetBlocksOfType(blocks, b => b.CustomName != null && b.CustomName.Contains(Tag));
                for (int i = 0; i < blocks.Count; i++)
                {
                    var provider = blocks[i] as IMyTextSurfaceProvider;
                    if (provider == null || provider.SurfaceCount == 0) continue;
                    var surface = provider.GetSurface(0);
                    surface.ContentType = ContentType.TEXT_AND_IMAGE;
                    surface.WriteText(text);
                }
            }
        }

        private void WarnIfLowPower(Colony colony, double stored, double cap)
        {
            if (cap <= 0 || stored / cap >= LowPowerFrac) return;
            DateTime last;
            if (_lastWarn.TryGetValue(colony.OwnerKey, out last)
                && (DateTime.UtcNow - last).TotalSeconds < WarnThrottleSecs) return;
            _lastWarn[colony.OwnerKey] = DateTime.UtcNow;
            if (!MyAPIGateway.Utilities.IsDedicated)
                MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                    "Base power low: {0:F0}% stored", stored / cap * 100.0));
            MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] colony {0}: base power low ({1:F0}%)", colony.OwnerKey, stored / cap * 100.0));
        }

        private static string FormatK(double v)
        {
            return v >= 1000000 ? (v / 1000000).ToString("F1") + "M"
                 : v >= 1000 ? (v / 1000).ToString("F1") + "k"
                 : v.ToString("F0");
        }
    }
}
