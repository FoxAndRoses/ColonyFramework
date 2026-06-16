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
        private readonly BoreController _bore = new BoreController(); // for ReleaseControls only
        private readonly List<long> _stale = new List<long>();

        public void Tick(Colony colony)
        {
            var ms = colony.Missions.Missions;
            var active = new HashSet<long>();

            for (int i = 0; i < ms.Count; i++)
            {
                var m = ms[i];
                if (m.Status != MissionStatus.InProgress || m.Type != MissionType.Mine) continue;

                var deposit = colony.Deposits.GetById(m.TargetDepositId);
                var grid = MyAPIGateway.Entities.GetEntityById(m.AssignedAssetId) as IMyCubeGrid;
                if (deposit == null || grid == null) continue;

                active.Add(m.Id);
                GetController(m.Id).Advance(colony, m, deposit, grid);
            }

            // Drop controllers whose mission is no longer an active in-progress mine mission.
            _stale.Clear();
            foreach (var key in _controllers.Keys)
                if (!active.Contains(key)) _stale.Add(key);
            for (int i = 0; i < _stale.Count; i++) _controllers.Remove(_stale[i]);
        }

        public void AbortAll(Colony colony)
        {
            var ms = colony.Missions.Missions;
            for (int i = 0; i < ms.Count; i++)
            {
                var m = ms[i];
                if (m.Type != MissionType.Mine) continue;
                if (m.Status != MissionStatus.Assigned && m.Status != MissionStatus.InProgress) continue;
                var grid = MyAPIGateway.Entities.GetEntityById(m.AssignedAssetId) as IMyCubeGrid;
                var asset = colony.Assets.GetByEntityId(m.AssignedAssetId);
                if (asset != null) asset.AutoDispatchEnabled = false; // park: don't auto-relaunch after an abort
                GetController(m.Id).Abort(colony, m, grid);
                _controllers.Remove(m.Id);
            }
        }

        public void RecallAll(Colony colony)
        {
            var ms = colony.Missions.Missions;
            for (int i = 0; i < ms.Count; i++)
            {
                var m = ms[i];
                if (m.Type != MissionType.Mine) continue;
                if (m.Status != MissionStatus.InProgress || m.Phase >= PhaseRetreat) continue;
                var deposit = colony.Deposits.GetById(m.TargetDepositId);
                var grid = MyAPIGateway.Entities.GetEntityById(m.AssignedAssetId) as IMyCubeGrid;
                if (deposit == null || grid == null) continue;
                var asset = colony.Assets.GetByEntityId(m.AssignedAssetId);
                if (asset != null) asset.AutoDispatchEnabled = false; // park after it returns; /colony dispatch to resume
                GetController(m.Id).Recall(m, deposit, grid);
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
    }
}
