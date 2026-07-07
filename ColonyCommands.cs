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
        private readonly ProductionService _production = new ProductionService();
        private readonly DispatchService _dispatch = new DispatchService();

        // Single source of truth for the help listing (keep in sync as commands are added).
        private static readonly string[] HelpLines = {
            "/colony help — list commands",
            "/colony info — name, age, deposit count",
            "/colony scan — scan ore near you",
            "/colony missions — mission counts",
            "/colony register — register nearest drone (auto-detects miner/welder)",
            "/colony assets — drone roster",
            "/colony resources — ore / ingot / component stock",
            "/colony build — projector blueprint vs stock (missing parts)",
            "/colony status — live mission/drone status, power, survey coverage",
            "/colony name <name> — rename the colony (LCDs/dashboard use it)",
            "/colony flighttest [50-500] — nearest idle drone flies out-and-back on the new flight core",
            "/colony dispatch — enable autonomous mining",
            "/colony abort — stop all missions",
            "/colony recall — bring drones home",
        };
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
            if (text != "/colony" && !text.StartsWith("/colony ")) return;
            sendToOthers = false;

            // Help works without a colony, so handle it (and a bare "/colony") before resolving one.
            if (text == "/colony" || text == "/colony help") { ShowHelp(); return; }

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

            if (text == "/colony build")
            {
                HandleBuild(colony);
                return;
            }

            if (text == "/colony status")
            {
                HandleStatus(colony);
                return;
            }

            if (text.StartsWith("/colony name "))
            {
                // Use the ORIGINAL message for the name — 'text' is lowercased for dispatch.
                string newName = messageText.Trim().Substring("/colony name ".Length).Trim();
                if (newName.Length == 0) { MyAPIGateway.Utilities.ShowMessage("Colony", "usage: /colony name <name>"); return; }
                if (newName.Length > 32) newName = newName.Substring(0, 32);
                colony.State.Name = newName;
                MyAPIGateway.Utilities.ShowMessage("Colony", "colony renamed to '" + newName + "'");
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

                // Never register the colony's own base structure (the core grid + everything physically
                // attached to it) as a mining asset — that block dispatch of the real drones.
                var baseGrids = new HashSet<long>();
                var coreBlock = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as IMyCubeBlock;
                if (coreBlock != null)
                {
                    var bgrids = new List<IMyCubeGrid>();
                    var group = coreBlock.CubeGrid.GetGridGroup(GridLinkTypeEnum.Physical);
                    if (group != null) group.GetGrids(bgrids);
                    if (bgrids.Count == 0) bgrids.Add(coreBlock.CubeGrid);
                    for (int i = 0; i < bgrids.Count; i++) baseGrids.Add(bgrids[i].EntityId);
                }

                var ents = new HashSet<IMyEntity>();
                MyAPIGateway.Entities.GetEntities(ents, e => e is IMyCubeGrid);
                IMyCubeGrid nearest = null;
                double bestSq = 100.0 * 100.0;
                foreach (var e in ents)
                {
                    var g = e as IMyCubeGrid;
                    if (g == null) continue;
                    if (baseGrids.Contains(g.EntityId)) continue; // skip the base/core structure
                    if (g.IsStatic) continue;                     // a station isn't a drone
                    double sq = Vector3D.DistanceSquared(g.GetPosition(), ppos);
                    if (sq < bestSq) { bestSq = sq; nearest = g; }
                }
                if (nearest == null)
                {
                    MyAPIGateway.Utilities.ShowMessage("Colony", "no eligible drone grid within 100m (the base/core is excluded)");
                    return;
                }
                // Classify by tooling: drills -> Miner (wins if both), welders -> Welder.
                AssetType type = DroneUtil.FindDrills(nearest).Count > 0 ? AssetType.Miner
                    : DroneUtil.FindWelders(nearest).Count > 0 ? AssetType.Welder
                    : AssetType.Miner;
                colony.Assets.Register(nearest.EntityId, type, nearest.GetPosition(), nearest.DisplayName);
                _executor.ReleaseControls(nearest); // clear any stale overrides so it won't thrust on its own
                MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                    "registered '{0}' as {1} ({2} assets total)", nearest.DisplayName,
                    type == AssetType.Welder ? "welder" : "miner", colony.Assets.Assets.Count));
                return;
            }

            if (text == "/colony flighttest" || text.StartsWith("/colony flighttest "))
            {
                double dist = 150;
                var parts = text.Split(' ');
                if (parts.Length >= 3) double.TryParse(parts[2], out dist);
                var player = MyAPIGateway.Session.Player;
                if (player == null) { MyAPIGateway.Utilities.ShowMessage("Colony", "no player context"); return; }
                var ppos = player.GetPosition();
                IMyCubeGrid best = null; double bestSq = double.MaxValue;
                var roster = colony.Assets.Assets;
                for (int i = 0; i < roster.Count; i++)
                {
                    if (roster[i].Status != AssetStatus.Idle) continue;
                    var g = MyAPIGateway.Entities.GetEntityById(roster[i].EntityId) as IMyCubeGrid;
                    if (g == null) continue;
                    double sq = VRageMath.Vector3D.DistanceSquared(g.GetPosition(), ppos);
                    if (sq < bestSq) { bestSq = sq; best = g; }
                }
                if (best == null) { MyAPIGateway.Utilities.ShowMessage("Colony", "no idle registered drone available"); return; }
                if (_executor.BeginFlightTest(best, dist))
                    MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                        "flight test: '{0}' flying {1:F0} m out and back on the new flight core", best.DisplayName, dist));
                else
                    MyAPIGateway.Utilities.ShowMessage("Colony", "that drone is already flight-testing");
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
                for (int i = 0; i < a.Count && i < 8; i++)
                    MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                        "  {0} [{1}] — {2}", a[i].Name, a[i].Status,
                        string.IsNullOrEmpty(a[i].CapabilitySummary) ? "not yet self-tested" : a[i].CapabilitySummary));
                return;
            }

            if (text == "/colony dispatch")
            {
                // Enable autonomy: from now the colony auto-launches assigned missions for these drones
                // (incl. each one's next mission after it recharges) until abort/recall parks them.
                var assets = colony.Assets.Assets;
                for (int i = 0; i < assets.Count; i++) assets[i].AutoDispatchEnabled = true;
                int launched = _dispatch.AutoDispatchAssigned(colony);
                MyAPIGateway.Utilities.ShowMessage("Colony", launched > 0
                    ? string.Format("autonomous mining enabled — dispatched {0} drone(s)", launched)
                    : "autonomous mining enabled — drones launch as missions are assigned");
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

            // Unrecognised subcommand — show the help listing rather than silently ignoring it.
            ShowHelp();
        }

        private void ShowHelp()
        {
            for (int i = 0; i < HelpLines.Length; i++)
                MyAPIGateway.Utilities.ShowMessage("Colony", HelpLines[i]);
        }

        // Phase names per mission type (indexes match each controller's Phase constants).
        private static readonly string[] MinePhases   = { "commission", "transit", "start bore", "mining", "retreat", "docking" };
        private static readonly string[] WeldPhases   = { "commission", "dock/load", "transit", "welding", "return" };
        private static readonly string[] SurveyPhases = { "commission", "transit", "scanning", "return" };

        // Live snapshot for the testing rounds: every active mission (what/who/where in the lifecycle),
        // every asset, and the survey coverage cursor — one command instead of log archaeology.
        private void HandleStatus(Colony colony)
        {
            int shown = 0;
            var ms = colony.Missions.Missions;
            for (int i = 0; i < ms.Count; i++)
            {
                var m = ms[i];
                if (m.Status != MissionStatus.Assigned && m.Status != MissionStatus.InProgress) continue;
                shown++;

                var grid = MyAPIGateway.Entities.GetEntityById(m.AssignedAssetId) as IMyCubeGrid;
                string drone = grid != null ? grid.DisplayName : ("#" + m.AssignedAssetId);

                string what;
                string[] phases;
                if (m.Type == MissionType.Weld) { what = "weld projector " + m.TargetEntityId; phases = WeldPhases; }
                else if (m.Type == MissionType.Survey) { what = m.TargetOre != null ? "survey for " + m.TargetOre : "general survey"; phases = SurveyPhases; }
                else
                {
                    var dep = colony.Deposits.GetById(m.TargetDepositId);
                    what = "mine " + (dep != null ? dep.OreType + " deposit " + dep.Id : "deposit " + m.TargetDepositId);
                    phases = MinePhases;
                }
                string phase = m.Status == MissionStatus.Assigned ? "awaiting launch"
                    : (m.Phase >= 0 && m.Phase < phases.Length ? phases[m.Phase] : "phase " + m.Phase);

                MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                    "mission {0}: '{1}' — {2} ({3})", m.Id, drone, what, phase));
            }
            if (shown == 0) MyAPIGateway.Utilities.ShowMessage("Colony", "no active missions");

            var assets = colony.Assets.Assets;
            for (int i = 0; i < assets.Count; i++)
            {
                var a = assets[i];
                MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                    "asset '{0}': {1}, {2}{3}", a.Name,
                    a.Type == AssetType.Welder ? "welder" : "miner",
                    a.Status == AssetStatus.Idle ? "idle" : a.Status == AssetStatus.Assigned ? "on mission" : "offline",
                    a.AutoDispatchEnabled ? "" : " (parked)"));
            }

            if (colony.State.SurveyedRadius > 0)
                MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                    "survey coverage: {0:F0} m ring at {1:F0}°; deposits known: {2}",
                    colony.State.SurveyedRadius, colony.State.SurveyedAngleDeg, colony.Deposits.Deposits.Count));

            var coreForPower = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as IMyCubeBlock;
            if (coreForPower != null)
            {
                double stored, cap, batOut, reactorOut;
                DroneUtil.GroupPower(coreForPower.CubeGrid, out stored, out cap, out batOut, out reactorOut);
                MyAPIGateway.Utilities.ShowMessage("Colony", string.Format(
                    "base power: {0:F0}% stored ({1:F1}/{2:F1} MWh){3}",
                    cap > 0 ? stored / cap * 100.0 : 0, stored, cap,
                    reactorOut > 0.005 ? string.Format(", reactor {0:F1} MW", reactorOut) : ""));
            }
        }

        // Compare the projector blueprint's component bill-of-materials to current colony stock and report
        // what's missing. Read-only: it does not queue assemblers or move anything (next increment).
        private void HandleBuild(Colony colony)
        {
            var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as IMyCubeBlock;
            if (core == null) { MyAPIGateway.Utilities.ShowMessage("Colony", "no colony core"); return; }

            // Refresh stock first so the comparison is current (same as /colony resources).
            _tracker.Scan(core.CubeGrid, colony.OwnerKey, colony.Resources, MyAPIGateway.Session.GameDateTime.Ticks);

            // Same rollup the autonomous ProductionService uses, so the report matches what it will do.
            var st = _production.Rollup(colony);
            if (!st.Projecting) { MyAPIGateway.Utilities.ShowMessage("Colony", "no active projector found"); return; }

            MyAPIGateway.Utilities.ShowMessage("Colony", string.Format("blueprint: {0} projector(s), {1} blocks", st.Projectors, st.Blocks));
            if (st.MissingComponents.Count == 0)
                MyAPIGateway.Utilities.ShowMessage("Colony", "components fully stocked — blueprint ready to weld");
            else
            {
                MyAPIGateway.Utilities.ShowMessage("Colony", "need components: " + ProductionService.Summarize(st.MissingComponents, ""));
                if (st.MissingOre.Count > 0)
                    MyAPIGateway.Utilities.ShowMessage("Colony", "must mine: " + ProductionService.Summarize(st.MissingOre, " ore"));
                else
                    MyAPIGateway.Utilities.ShowMessage("Colony", "have the raw materials — will queue assemblers shortly");
            }

            VRage.Utils.MyLog.Default.WriteLineAndConsole(string.Format(
                "[ColonyFramework] /colony build: {0} proj, {1} blocks; missing comp {2} | ingot {3} | ore {4}",
                st.Projectors, st.Blocks,
                ProductionService.Summarize(st.MissingComponents, ""),
                ProductionService.Summarize(st.MissingIngots, ""),
                ProductionService.Summarize(st.MissingOre, "")));
        }
    }
}
