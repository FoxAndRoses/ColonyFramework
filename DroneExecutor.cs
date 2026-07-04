using System.Collections.Generic;
using Sandbox.ModAPI;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

namespace ColonyFramework
{
    // Coordinator: owns one MinerController per in-progress Mine mission and routes ticks to it.
    // All per-mission state and behaviour live in MinerController; this class only manages the
    // controller lifecycle and exposes the command surface used by the host and chat commands.
    public class DroneExecutor
    {
        private const int PhaseRetreat = 4;

        private readonly Dictionary<long, MinerController> _controllers = new Dictionary<long, MinerController>();
        private readonly Dictionary<long, WelderController> _welders = new Dictionary<long, WelderController>();
        private readonly Dictionary<long, SurveyController> _surveys = new Dictionary<long, SurveyController>();
        private readonly Dictionary<long, ParkController> _parkers = new Dictionary<long, ParkController>(); // by ASSET id
        private readonly ConnectorReservations _cons = new ConnectorReservations(); // fleet connector traffic control
        private readonly WeldCoordinator _weldCoord = new WeldCoordinator(); // multi-welder block claims + bubbles
        private readonly BoreController _bore = new BoreController(); // for ReleaseControls only
        private readonly List<long> _stale = new List<long>();

        public void Tick(Colony colony)
        {
            var ms = colony.Missions.Missions;
            var active = new HashSet<long>();
            var activeWeld = new HashSet<long>();
            var activeSurvey = new HashSet<long>();

            for (int i = 0; i < ms.Count; i++)
            {
                var m = ms[i];
                if (m.Status != MissionStatus.InProgress) continue;

                var grid = MyAPIGateway.Entities.GetEntityById(m.AssignedAssetId) as IMyCubeGrid;
                if (grid == null) continue;

                if (m.Type == MissionType.Mine)
                {
                    var deposit = colony.Deposits.GetById(m.TargetDepositId);
                    if (deposit == null) continue;
                    active.Add(m.Id);
                    GetController(m.Id).Advance(colony, m, deposit, grid, _cons);
                }
                else if (m.Type == MissionType.Weld)
                {
                    activeWeld.Add(m.Id);
                    GetWelder(m.Id).Advance(colony, m, grid, _cons, _weldCoord);
                }
                else if (m.Type == MissionType.Survey)
                {
                    activeSurvey.Add(m.Id);
                    GetSurvey(m.Id).Advance(colony, m, grid);
                }
            }

            // Drop controllers whose mission is no longer active in-progress.
            _stale.Clear();
            foreach (var key in _controllers.Keys)
                if (!active.Contains(key)) _stale.Add(key);
            for (int i = 0; i < _stale.Count; i++) _controllers.Remove(_stale[i]);

            _stale.Clear();
            foreach (var key in _welders.Keys)
                if (!activeWeld.Contains(key)) _stale.Add(key);
            for (int i = 0; i < _stale.Count; i++) _welders.Remove(_stale[i]);

            _stale.Clear();
            foreach (var key in _surveys.Keys)
                if (!activeSurvey.Contains(key)) _stale.Add(key);
            for (int i = 0; i < _stale.Count; i++) _surveys.Remove(_stale[i]);

            TickParkers(colony);
        }

        // Idle drones get parked (dock-recharge or land-nearby + power-nap) instead of hovering
        // forever. A parker exists only while its asset is Idle; the moment a mission takes the
        // asset the parker is dropped and PrepareForFlight wakes the drone at commissioning.
        private void TickParkers(Colony colony)
        {
            var assets = colony.Assets.Assets;
            _stale.Clear();
            foreach (var key in _parkers.Keys) _stale.Add(key); // assume stale until seen idle below
            for (int i = 0; i < assets.Count; i++)
            {
                var a = assets[i];
                if (a.Status != AssetStatus.Idle) continue;
                var grid = MyAPIGateway.Entities.GetEntityById(a.EntityId) as IMyCubeGrid;
                if (grid == null) continue;
                _stale.Remove(a.EntityId);
                ParkController p;
                if (!_parkers.TryGetValue(a.EntityId, out p)) { p = new ParkController(); _parkers[a.EntityId] = p; }
                p.Tick(colony, a, grid, _cons);
            }
            for (int i = 0; i < _stale.Count; i++) _parkers.Remove(_stale[i]);
        }

        public void AbortAll(Colony colony)
        {
            var ms = colony.Missions.Missions;
            for (int i = 0; i < ms.Count; i++)
            {
                var m = ms[i];
                if (m.Status != MissionStatus.Assigned && m.Status != MissionStatus.InProgress) continue;
                var grid = MyAPIGateway.Entities.GetEntityById(m.AssignedAssetId) as IMyCubeGrid;
                var asset = colony.Assets.GetByEntityId(m.AssignedAssetId);
                if (asset != null) asset.AutoDispatchEnabled = false; // park: don't auto-relaunch after an abort
                if (m.Type == MissionType.Mine) { GetController(m.Id).Abort(colony, m, grid); _controllers.Remove(m.Id); }
                else if (m.Type == MissionType.Weld) { GetWelder(m.Id).Abort(colony, m, grid); _welders.Remove(m.Id); }
                else if (m.Type == MissionType.Survey) { GetSurvey(m.Id).Abort(colony, m, grid); _surveys.Remove(m.Id); }
            }
        }

        public void RecallAll(Colony colony)
        {
            var ms = colony.Missions.Missions;
            for (int i = 0; i < ms.Count; i++)
            {
                var m = ms[i];
                if (m.Status != MissionStatus.InProgress) continue;
                var grid = MyAPIGateway.Entities.GetEntityById(m.AssignedAssetId) as IMyCubeGrid;
                if (grid == null) continue;
                var asset = colony.Assets.GetByEntityId(m.AssignedAssetId);
                if (asset != null) asset.AutoDispatchEnabled = false; // park after it returns; /colony dispatch to resume

                if (m.Type == MissionType.Mine)
                {
                    if (m.Phase >= PhaseRetreat) continue; // already on its way home
                    var deposit = colony.Deposits.GetById(m.TargetDepositId);
                    if (deposit == null) continue;
                    GetController(m.Id).Recall(m, deposit, grid);
                }
                else if (m.Type == MissionType.Weld) GetWelder(m.Id).Recall(colony, m, grid);
                else if (m.Type == MissionType.Survey) GetSurvey(m.Id).Recall(colony, m, grid);
            }
        }

        // Clears any stale gyro/thrust overrides on a grid (e.g. baked into a spawned/pasted
        // blueprint). Called on registration so a claimed drone won't thrust on its own.
        public void ReleaseControls(IMyCubeGrid grid)
        {
            if (grid != null) _bore.Release(grid);
        }

        private MinerController GetController(long missionId)
        {
            MinerController c;
            if (!_controllers.TryGetValue(missionId, out c)) { c = new MinerController(); _controllers[missionId] = c; }
            return c;
        }

        private WelderController GetWelder(long missionId)
        {
            WelderController c;
            if (!_welders.TryGetValue(missionId, out c)) { c = new WelderController(); _welders[missionId] = c; }
            return c;
        }

        private SurveyController GetSurvey(long missionId)
        {
            SurveyController c;
            if (!_surveys.TryGetValue(missionId, out c)) { c = new SurveyController(); _surveys[missionId] = c; }
            return c;
        }
    }
}
