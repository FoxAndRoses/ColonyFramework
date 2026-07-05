using System;
using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.Game.Components;
using VRage.Game.ModAPI;
using VRage.Utils;

namespace ColonyFramework
{
    [MySessionComponentDescriptor(MyUpdateOrder.BeforeSimulation)]
    public class CommandCoreSession : MySessionComponentBase
    {
        public static CommandCoreSession Instance;

        private const string WorldFile = "colony_world.xml";
        private const string LegacyFile = "colony_state.xml";

        private const int ScanIntervalTicks = 120;
        private const int RefreshIntervalTicks = 600;
        private const int GenIntervalTicks = 300;
        private const int AssignIntervalTicks = 180;
        private const int CoreIntervalTicks = 60;
        private const int ResourceIntervalTicks = 1800; // ~30s
        private const int ProductionIntervalTicks = 300; // ~5s
        private const int ExecIntervalTicks = 10; // ~6 Hz

        private WorldState _world;
        private ColonyRegistry _registry;
        private OreScanScheduler _scanScheduler;
        private AssignmentService _assignment;
        private ColonyCommands _commands;
        private readonly Dictionary<long, IMyCubeBlock> _cores = new Dictionary<long, IMyCubeBlock>();
        private readonly ResourceTracker _resourceTracker = new ResourceTracker();
        private readonly DroneExecutor _executor = new DroneExecutor();
        private readonly DispatchService _dispatch = new DispatchService();
        private readonly ProductionService _production = new ProductionService();
        private readonly GpsService _gps = new GpsService();
        private readonly LcdService _lcd = new LcdService();

        private int _tick;
        private long _scanTick;
        private bool _ready;

        public ColonyRegistry Registry { get { return _registry; } }

        public void RegisterCore(IMyCubeBlock core) { if (core != null) _cores[core.EntityId] = core; }
        public void UnregisterCore(IMyCubeBlock core) { if (core != null) _cores.Remove(core.EntityId); }

        private bool IsServer()
        {
            return MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer;
        }

        public override void LoadData()
        {
            Instance = this;

            if (!IsServer())
            {
                _world = new WorldState();
                _registry = new ColonyRegistry(_world);
                return;
            }

            try
            {
                _world = LoadWorld() ?? new WorldState();
                _registry = new ColonyRegistry(_world);

                _scanScheduler = new OreScanScheduler();
                _assignment = new AssignmentService();
                _commands = new ColonyCommands(_registry, _executor);

                if (!MyAPIGateway.Utilities.IsDedicated)
                    MyAPIGateway.Utilities.MessageEntered += OnMessageEntered;

                MyLog.Default.WriteLineAndConsole("[ColonyFramework] Loaded. Colonies: " + _world.Colonies.Count);
                _ready = true;
            }
            catch (Exception e)
            {
                MyLog.Default.WriteLineAndConsole("[ColonyFramework] LoadData error: " + e.Message);
                _world = new WorldState();
                _registry = new ColonyRegistry(_world);
                _scanScheduler = new OreScanScheduler();
                _assignment = new AssignmentService();
                _commands = new ColonyCommands(_registry, _executor);
                _ready = true;
            }
        }

        public override void UpdateBeforeSimulation()
        {
            if (!IsServer() || !_ready) return;
            _tick++;

            if (_tick % CoreIntervalTicks == 0) UpdateColonyActivity();
            if (_tick % ResourceIntervalTicks == 0) UpdateResources();
            if (_tick % RefreshIntervalTicks == 0) _scanScheduler.RefreshDetectors();
            if (_tick % ScanIntervalTicks == 0) _scanScheduler.Step(_registry, ref _scanTick);

            // Mining is DEMAND-DRIVEN: ProductionService creates targeted Mine missions for ores the
            // colony actually needs (blueprint shortfalls + the ice reserve). The old blanket
            // one-mission-per-deposit generator queued the entire deposit DB (692 missions in one
            // tick in testing) and sent drones after ore nobody wanted — retired.

            if (_tick % AssignIntervalTicks == 0)
            {
                foreach (var colony in _registry.Colonies)
                {
                    if (!colony.Active) continue;
                    _assignment.ValidateAndAssign(colony);
                    _dispatch.AutoDispatchAssigned(colony); // autonomous: launch assigned missions (incl. a just-recharged drone's next one)
                }
            }

            if (_tick % ProductionIntervalTicks == 0)
            {
                foreach (var colony in _registry.Colonies)
                {
                    if (!colony.Active) continue;
                    // Isolate the production/definition API (new, higher-surface) so a hiccup can't kill the tick.
                    try { _production.Tick(colony); }
                    catch (Exception e) { MyLog.Default.WriteLineAndConsole("[ColonyFramework] production error: " + e.Message); }
                    try { _gps.Sync(colony); } // HUD markers for active missions (same slow cadence)
                    catch (Exception e) { MyLog.Default.WriteLineAndConsole("[ColonyFramework] gps error: " + e.Message); }
                    try { _lcd.Tick(colony); } // "[Colony]" LCD dashboards + low-power warning
                    catch (Exception e) { MyLog.Default.WriteLineAndConsole("[ColonyFramework] lcd error: " + e.Message); }
                }
            }

            if (_tick % ExecIntervalTicks == 0)
            {
                foreach (var colony in _registry.Colonies)
                {
                    if (!colony.Active) continue;
                    _executor.Tick(colony);
                }
            }
        }

        private void UpdateResources()
        {
            try
            {
                long now = MyAPIGateway.Session.GameDateTime.Ticks;
                foreach (var colony in _registry.Colonies)
                {
                    if (!colony.Active || colony.State.CoreEntityId == 0) continue;
                    var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId) as IMyCubeBlock;
                    if (core == null) continue;
                    _resourceTracker.Scan(core.CubeGrid, colony.OwnerKey, colony.Resources, now);
                }
            }
            catch (Exception e)
            {
                MyLog.Default.WriteLineAndConsole("[ColonyFramework] UpdateResources error: " + e.Message);
            }
        }

        private void UpdateColonyActivity()
        {
            try
            {
                var activeOwners = new HashSet<long>();
                var coreForOwner = new Dictionary<long, long>();
                var founderForOwner = new Dictionary<long, long>();
                foreach (var kv in _cores)
                {
                    var core = kv.Value;
                    if (core == null) continue;
                    var fb = core as IMyFunctionalBlock;
                    if (fb == null || !fb.IsWorking) continue;
                    long ownerKey = Ownership.ResolveOwnerKey(core.OwnerId);
                    activeOwners.Add(ownerKey);
                    long existing;
                    if (!coreForOwner.TryGetValue(ownerKey, out existing) || core.EntityId < existing)
                    {
                        coreForOwner[ownerKey] = core.EntityId;
                        founderForOwner[ownerKey] = core.OwnerId;
                    }
                }

                foreach (var ownerKey in activeOwners)
                {
                    var colony = _registry.GetOrCreate(ownerKey);
                    colony.Active = true;
                    colony.State.CoreEntityId = coreForOwner[ownerKey];
                    if (colony.State.FounderId == 0)
                    {
                        colony.State.FounderId = founderForOwner[ownerKey];
                        colony.State.CreatedGameTicks = MyAPIGateway.Session.GameDateTime.Ticks;
                        if (string.IsNullOrEmpty(colony.State.Name))
                            colony.State.Name = "Colony " + colony.State.ColonyId;
                        MyLog.Default.WriteLineAndConsole(string.Format(
                            "[ColonyFramework] Colony founded: id={0} name='{1}' founder={2}",
                            colony.State.ColonyId, colony.State.Name, colony.State.FounderId));
                    }
                }

                foreach (var colony in _registry.Colonies)
                {
                    if (!activeOwners.Contains(colony.OwnerKey))
                    {
                        colony.Active = false;
                        colony.State.CoreEntityId = 0;
                    }
                }
            }
            catch (Exception e)
            {
                MyLog.Default.WriteLineAndConsole("[ColonyFramework] UpdateColonyActivity error: " + e.Message);
            }
        }

        public override void SaveData()
        {
            if (!IsServer() || _world == null) return;
            try
            {
                var xml = MyAPIGateway.Utilities.SerializeToXML(_world);
                using (var w = MyAPIGateway.Utilities.WriteFileInWorldStorage(WorldFile, typeof(CommandCoreSession)))
                    w.Write(xml);
                MyLog.Default.WriteLineAndConsole("[ColonyFramework] SaveData - colonies: " + _world.Colonies.Count);
            }
            catch (Exception e)
            {
                MyLog.Default.WriteLineAndConsole("[ColonyFramework] SaveData error: " + e.Message);
            }
        }

        private WorldState LoadWorld()
        {
            if (MyAPIGateway.Utilities.FileExistsInWorldStorage(WorldFile, typeof(CommandCoreSession)))
            {
                using (var r = MyAPIGateway.Utilities.ReadFileInWorldStorage(WorldFile, typeof(CommandCoreSession)))
                {
                    var xml = r.ReadToEnd();
                    if (!string.IsNullOrEmpty(xml))
                        return MyAPIGateway.Utilities.SerializeFromXML<WorldState>(xml);
                }
            }
            if (MyAPIGateway.Utilities.FileExistsInWorldStorage(LegacyFile, typeof(CommandCoreSession)))
            {
                using (var r = MyAPIGateway.Utilities.ReadFileInWorldStorage(LegacyFile, typeof(CommandCoreSession)))
                {
                    var xml = r.ReadToEnd();
                    if (!string.IsNullOrEmpty(xml))
                    {
                        var legacy = MyAPIGateway.Utilities.SerializeFromXML<ColonyState>(xml);
                        if (legacy != null)
                        {
                            legacy.OwnerKey = 0;
                            legacy.Active = false;
                            var ws = new WorldState();
                            ws.Colonies.Add(legacy);
                            MyLog.Default.WriteLineAndConsole("[ColonyFramework] Migrated legacy colony_state.xml.");
                            return ws;
                        }
                    }
                }
            }
            return null;
        }

        private void OnMessageEntered(string messageText, ref bool sendToOthers)
        {
            if (_commands != null) _commands.Handle(messageText, ref sendToOthers);
        }

        protected override void UnloadData()
        {
            MyLog.Default.WriteLineAndConsole("[ColonyFramework] UnloadData.");
            if (!MyAPIGateway.Utilities.IsDedicated)
                MyAPIGateway.Utilities.MessageEntered -= OnMessageEntered;
            if (_scanScheduler != null) _scanScheduler.Clear();
            _cores.Clear();
            _scanScheduler = null;
            _assignment = null;
            _commands = null;
            _registry = null;
            _world = null;
            _ready = false;
            Instance = null;
        }
    }
}
