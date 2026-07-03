using System;
using System.Collections.Generic;
using Sandbox.Definitions;
using Sandbox.ModAPI;
using VRage;
using VRage.Game;
using VRage.ObjectBuilders;
using VRage.Utils;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;
using IMyCubeBlock = VRage.Game.ModAPI.IMyCubeBlock;

namespace ColonyFramework
{
    // Result of rolling the projector blueprint up the production chain against current colony stock.
    // Each dict is what's still SHORT at that level (qty needed beyond what's in stock / already queued).
    public class ProductionStatus
    {
        public bool Projecting;
        public int Projectors;
        public int Blocks;
        public readonly Dictionary<string, double> MissingComponents = new Dictionary<string, double>();
        public readonly Dictionary<string, double> MissingIngots = new Dictionary<string, double>();
        public readonly Dictionary<string, double> MissingOre = new Dictionary<string, double>();

        public bool ComponentsStocked { get { return Projecting && MissingComponents.Count == 0; } }
        public bool CanBuildNow { get { return Projecting && MissingComponents.Count > 0 && MissingOre.Count == 0; } }
    }

    // Autonomous production: each tick, roll the projected blueprint up the chain (components -> ingots ->
    // ore) vs station stock. If we hold enough raw material, queue the missing components on assemblers
    // (after a short debounce) and keep refineries processing. If we're short on ore, announce it and
    // create demand-driven mining missions. Services layer (touches assemblers/refineries/definitions);
    // calls Domain (Missions/Deposits/Resources). It coordinates vanilla blocks — it never fake-produces.
    public class ProductionService
    {
        private readonly ProjectorReader _projectors = new ProjectorReader();

        private class ColonyProd { public DateTime ReadySince; public DateTime LastChat; public DateTime LastSurvey; }
        private const double SurveyCooldownSecs = 600.0; // min gap between demand-driven survey dispatches
        private readonly Dictionary<long, ColonyProd> _state = new Dictionary<long, ColonyProd>();

        private readonly MyObjectBuilderType _ingotType;
        private readonly MyObjectBuilderType _oreType;

        private const double ReadyDelaySecs   = 15.0; // hold "can build" this long before queuing (user's 15s)
        private const double ChatThrottleSecs = 60.0; // don't repeat the shortage chat more often than this

        public ProductionService()
        {
            MyDefinitionId t;
            if (MyDefinitionId.TryParse("MyObjectBuilder_Ingot/Iron", out t)) _ingotType = t.TypeId;
            if (MyDefinitionId.TryParse("MyObjectBuilder_Ore/Iron", out t)) _oreType = t.TypeId;
        }

        // ── Autonomous orchestration ───────────────────────────────────────────────────────────────
        public void Tick(Colony colony)
        {
            if (colony == null || !colony.Active) return;
            var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as IMyCubeBlock;
            if (core == null) return;
            var grid = core.CubeGrid;

            var status = Rollup(colony, grid);
            if (!status.Projecting) { _state.Remove(colony.OwnerKey); return; }
            var st = GetState(colony.OwnerKey);

            // A projecting blueprint with blocks left gets a Weld mission (one per projector). Welding
            // runs in PARALLEL with component production — CanBuild + cargo gate what's actually weldable.
            var projectors = CollectGroup<IMyProjector>(grid);
            long nowTick = MyAPIGateway.Session.GameDateTime.Ticks;
            for (int i = 0; i < projectors.Count; i++)
            {
                var p = projectors[i];
                if (p == null || !p.IsProjecting || p.RemainingBlocks <= 0) continue;
                if (colony.Missions.EnsureWeldMission(p.EntityId, nowTick))
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] production: weld mission created for projector {0} ({1} blocks remaining)",
                        p.EntityId, p.RemainingBlocks));
            }

            if (status.MissingComponents.Count == 0) { st.ReadySince = default(DateTime); return; } // nothing to make

            if (status.CanBuildNow)
            {
                if (st.ReadySince == default(DateTime)) st.ReadySince = DateTime.UtcNow; // start the debounce
                if ((DateTime.UtcNow - st.ReadySince).TotalSeconds < ReadyDelaySecs) return;

                int assemblers, queued;
                QueueComponents(grid, status.MissingComponents, out assemblers, out queued);
                EnsureRefineries(grid);
                if (queued > 0)
                {
                    Announce(colony, string.Format("queuing {0} component(s) for blueprint on {1} assembler(s)", queued, assemblers), true);
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] production: queued {0} components on {1} assembler(s)", queued, assemblers));
                }
                else if (assemblers == 0)
                    Announce(colony, "have the materials but no working assembler to build the blueprint", false);
            }
            else // short on ore -> target it for mining (general mining already runs; this prioritises the gap)
            {
                st.ReadySince = default(DateTime);

                long tick = MyAPIGateway.Session.GameDateTime.Ticks;
                var basePos = core.GetPosition();
                int created = 0;
                string unknownOre = null; // an ore we need but have NO deposit of anywhere in the DB
                foreach (var kv in status.MissingOre)
                {
                    var dep = colony.Deposits.FindNearestUnclaimed(basePos, kv.Key);
                    if (dep != null && colony.Missions.CreateMineMission(dep.Id, tick)) created++;
                    else if (dep == null && unknownOre == null && !HasAnyDeposit(colony, kv.Key)) unknownOre = kv.Key;
                }

                // A needed ore isn't in the deposit DB AT ALL — send a scout to look for it (cooldown-gated).
                if (unknownOre != null && (DateTime.UtcNow - st.LastSurvey).TotalSeconds > SurveyCooldownSecs
                    && colony.Missions.EnsureSurveyMission(unknownOre, tick))
                {
                    st.LastSurvey = DateTime.UtcNow;
                    if (!MyAPIGateway.Utilities.IsDedicated)
                        MyAPIGateway.Utilities.ShowMessage("Colony", "no known " + unknownOre + " deposits — dispatching a survey drone");
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] production: no {0} in deposit DB — survey mission created", unknownOre));
                }

                // This branch runs every tick — throttle the chat AND log together so it doesn't flood.
                if ((DateTime.UtcNow - st.LastChat).TotalSeconds >= ChatThrottleSecs)
                {
                    st.LastChat = DateTime.UtcNow;
                    if (!MyAPIGateway.Utilities.IsDedicated)
                        MyAPIGateway.Utilities.ShowMessage("Colony", "production waiting on ore: " + Summarize(status.MissingOre, "") + " — mining");
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] production: short ore {0}; {1}",
                        Summarize(status.MissingOre, ""),
                        created > 0 ? ("created " + created + " mine mission(s)") : "matching deposits already claimed/mining"));
                }
            }
        }

        // ── The blueprint -> stock rollup (shared by Tick and /colony build) ─────────────────────────
        public ProductionStatus Rollup(Colony colony)
        {
            var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as IMyCubeBlock;
            return Rollup(colony, core != null ? core.CubeGrid : null);
        }

        private ProductionStatus Rollup(Colony colony, IMyCubeGrid grid)
        {
            var status = new ProductionStatus();
            if (grid == null) return status;

            int nProj, nBlocks;
            var required = _projectors.RequiredComponents(grid, out nProj, out nBlocks);
            status.Projectors = nProj;
            status.Blocks = nBlocks;
            status.Projecting = nProj > 0;
            if (!status.Projecting) return status;

            var haveComp = colony.Resources.Components;
            var haveIngot = colony.Resources.Ingots;
            var haveOre = colony.Resources.Ore;
            var queued = QueuedComponents(grid);

            // Level 1: components short of (stock + already queued).
            foreach (var kv in required)
            {
                double need = kv.Value - Get(haveComp, kv.Key) - Get(queued, kv.Key);
                if (need > 0) status.MissingComponents[kv.Key] = need;
            }
            if (status.MissingComponents.Count == 0) return status;

            // Level 2: ingots needed for the missing components, short of ingot stock.
            var reqIngots = new Dictionary<string, double>();
            foreach (var kv in status.MissingComponents)
            {
                MyDefinitionId id;
                if (TryDefId("MyObjectBuilder_Component", kv.Key, out id))
                    AddPrereqs(id, kv.Value, _ingotType, reqIngots);
            }
            foreach (var kv in reqIngots)
            {
                double need = kv.Value - Get(haveIngot, kv.Key);
                if (need > 0) status.MissingIngots[kv.Key] = need;
            }

            // Level 3: ore needed to refine the missing ingots, short of ore stock.
            var reqOre = new Dictionary<string, double>();
            foreach (var kv in status.MissingIngots)
            {
                MyDefinitionId id;
                if (TryDefId("MyObjectBuilder_Ingot", kv.Key, out id))
                    AddPrereqs(id, kv.Value, _oreType, reqOre);
            }
            foreach (var kv in reqOre)
            {
                double need = kv.Value - Get(haveOre, kv.Key);
                if (need > 0) status.MissingOre[kv.Key] = need;
            }
            return status;
        }

        // Sum the prerequisites (of prereqType only) needed to produce `qty` of the given result, into `into`.
        // Prereqs and results are per production run, so scale by the result's per-run amount.
        private void AddPrereqs(MyDefinitionId resultId, double qty, MyObjectBuilderType prereqType, Dictionary<string, double> into)
        {
            MyBlueprintDefinitionBase bp;
            if (!MyDefinitionManager.Static.TryGetBlueprintDefinitionByResultId(resultId, out bp) || bp == null) return;

            double perRun = 1.0;
            if (bp.Results != null)
                for (int i = 0; i < bp.Results.Length; i++)
                    if (bp.Results[i].Id.SubtypeName == resultId.SubtypeName) { perRun = (double)bp.Results[i].Amount; break; }
            if (perRun <= 0) perRun = 1.0;

            double runs = qty / perRun;
            if (bp.Prerequisites != null)
                for (int i = 0; i < bp.Prerequisites.Length; i++)
                {
                    var p = bp.Prerequisites[i];
                    if (p.Id.TypeId != prereqType) continue;
                    Add(into, p.Id.SubtypeName, runs * (double)p.Amount);
                }
        }

        // ── Assembler / refinery coordination ────────────────────────────────────────────────────────
        private void QueueComponents(IMyCubeGrid grid, Dictionary<string, double> missing, out int assemblers, out int queuedCount)
        {
            assemblers = 0;
            queuedCount = 0;
            var list = CollectGroup<IMyAssembler>(grid);
            IMyAssembler target = null;
            for (int i = 0; i < list.Count; i++)
                if (list[i].Enabled && list[i].IsFunctional) { target = list[i]; break; }
            assemblers = list.Count;
            if (target == null) return;

            target.Mode = Sandbox.ModAPI.Ingame.MyAssemblerMode.Assembly;
            target.Repeating = false;
            target.UseConveyorSystem = true;

            foreach (var kv in missing)
            {
                double amount = Math.Ceiling(kv.Value);
                if (amount <= 0) continue;
                MyDefinitionId id;
                if (!TryDefId("MyObjectBuilder_Component", kv.Key, out id)) continue;
                MyBlueprintDefinitionBase bp;
                if (!MyDefinitionManager.Static.TryGetBlueprintDefinitionByResultId(id, out bp) || bp == null) continue;
                target.AddQueueItem(bp, (MyFixedPoint)(float)amount);
                queuedCount++;
            }
        }

        // What components are already queued across all assemblers (so we don't re-queue them).
        private Dictionary<string, double> QueuedComponents(IMyCubeGrid grid)
        {
            var queued = new Dictionary<string, double>();
            var list = CollectGroup<IMyAssembler>(grid);
            for (int a = 0; a < list.Count; a++)
            {
                var q = list[a].GetQueue();
                if (q == null) continue;
                for (int i = 0; i < q.Count; i++)
                {
                    var bp = q[i].Blueprint as MyBlueprintDefinitionBase;
                    if (bp == null || bp.Results == null || bp.Results.Length == 0) continue;
                    Add(queued, bp.Results[0].Id.SubtypeName, (double)q[i].Amount);
                }
            }
            return queued;
        }

        private void EnsureRefineries(IMyCubeGrid grid)
        {
            var list = CollectGroup<IMyRefinery>(grid);
            for (int i = 0; i < list.Count; i++)
            {
                list[i].Enabled = true;
                list[i].UseConveyorSystem = true; // let the game auto-pull ore from the network and refine it
            }
        }

        // ── helpers ──────────────────────────────────────────────────────────────────────────────────
        private static readonly List<IMyCubeGrid> _groupBuf = new List<IMyCubeGrid>();
        private List<T> CollectGroup<T>(IMyCubeGrid coreGrid) where T : class, Sandbox.ModAPI.Ingame.IMyTerminalBlock
        {
            var result = new List<T>();
            _groupBuf.Clear();
            var group = coreGrid.GetGridGroup(VRage.Game.ModAPI.GridLinkTypeEnum.Physical);
            if (group != null) group.GetGrids(_groupBuf);
            if (_groupBuf.Count == 0) _groupBuf.Add(coreGrid);
            var tmp = new List<T>();
            for (int g = 0; g < _groupBuf.Count; g++)
            {
                var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(_groupBuf[g]);
                if (ts == null) continue;
                tmp.Clear();
                ts.GetBlocksOfType(tmp);
                result.AddRange(tmp);
            }
            return result;
        }

        // Ice is only worth mining when something can USE it: O2/H2 generators consume it (keep a
        // working buffer), and otherwise a small reserve stays on hand so the player can fill
        // hydrogen bottles — but the colony should never dedicate mining runs to ice nobody needs.
        private const double IceReserveBase  = 2000.0;  // no gas equipment: just the bottle reserve
        private const double IceReserveEquip = 10000.0; // gas generators present: keep them fed
        public bool AllowIceMining(Colony colony)
        {
            var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as IMyCubeBlock;
            if (core == null) return false;
            double ice;
            colony.Resources.Ore.TryGetValue("Ice", out ice);
            bool hasGasEquipment = CollectGroup<IMyGasGenerator>(core.CubeGrid).Count > 0;
            return ice < (hasGasEquipment ? IceReserveEquip : IceReserveBase);
        }

        // Any deposit of this ore in the DB at all (claimed, unclaimed, or being mined)? Depleted ones
        // don't count — a fully-mined ore type is as unknown as a never-seen one.
        private static bool HasAnyDeposit(Colony colony, string oreType)
        {
            var deps = colony.Deposits.Deposits;
            for (int i = 0; i < deps.Count; i++)
                if (deps[i].OreType == oreType && deps[i].Status != DepositStatus.Depleted) return true;
            return false;
        }

        private ColonyProd GetState(long ownerKey)
        {
            ColonyProd s;
            if (!_state.TryGetValue(ownerKey, out s)) { s = new ColonyProd(); _state[ownerKey] = s; }
            return s;
        }

        private void Announce(Colony colony, string msg, bool always)
        {
            var st = GetState(colony.OwnerKey);
            if (!always && (DateTime.UtcNow - st.LastChat).TotalSeconds < ChatThrottleSecs) return;
            st.LastChat = DateTime.UtcNow;
            if (!MyAPIGateway.Utilities.IsDedicated) MyAPIGateway.Utilities.ShowMessage("Colony", msg);
        }

        public static string Summarize(Dictionary<string, double> d, string suffix)
        {
            var parts = new List<string>();
            foreach (var kv in d) parts.Add(string.Format("{0:N0} {1}{2}", kv.Value, kv.Key, suffix));
            return parts.Count == 0 ? "none" : string.Join(", ", parts);
        }

        private static bool TryDefId(string typeId, string subtype, out MyDefinitionId id)
        {
            return MyDefinitionId.TryParse(typeId + "/" + subtype, out id);
        }

        private static double Get(Dictionary<string, double> d, string key) { double v; d.TryGetValue(key, out v); return v; }
        private static void Add(Dictionary<string, double> d, string key, double amt) { double v; d.TryGetValue(key, out v); d[key] = v + amt; }
    }
}
