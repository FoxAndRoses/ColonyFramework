using System.Collections.Generic;

namespace ColonyFramework
{
    // Server-side authority over missions. Owns mission state transitions and the
    // mission<->deposit coupling (claim on assign, deplete on complete, release on fail).
    public class MissionManager
    {
        private readonly ColonyState _state;
        private readonly DepositManager _deposits;

        public MissionManager(ColonyState state, DepositManager deposits)
        {
            _state = state;
            _deposits = deposits;
        }

        public IReadOnlyList<Mission> Missions { get { return _state.Missions; } }

        // One Mine mission per Unclaimed deposit with no active mission. Returns count created.
        public int GenerateMineMissions(long tick)
        {
            int created = 0;
            var deps = _deposits.Deposits;
            for (int i = 0; i < deps.Count; i++)
            {
                var d = deps[i];
                if (d.Status != DepositStatus.Unclaimed) continue;
                if (HasActiveMissionFor(d.Id)) continue;

                _state.Missions.Add(new Mission
                {
                    Id = _state.NextMissionId++,
                    Type = MissionType.Mine,
                    TargetDepositId = d.Id,
                    AssignedAssetId = 0,
                    Status = MissionStatus.PendingAssignment,
                    CreatedTick = tick
                });
                created++;
            }
            return created;
        }

        public Mission GetById(long id)
        {
            for (int i = 0; i < _state.Missions.Count; i++)
                if (_state.Missions[i].Id == id) return _state.Missions[i];
            return null;
        }

        // PendingAssignment -> Assigned. Claims the target deposit for the asset.
        public bool Assign(long missionId, long assetEntityId)
        {
            var m = GetById(missionId);
            if (m == null || m.Status != MissionStatus.PendingAssignment) return false;
            if (!_deposits.TryClaim(m.TargetDepositId, assetEntityId)) return false;
            m.AssignedAssetId = assetEntityId;
            m.Status = MissionStatus.Assigned;
            return true;
        }

        public void SetInProgress(long missionId)
        {
            var m = GetById(missionId);
            if (m != null && m.Status == MissionStatus.Assigned)
                m.Status = MissionStatus.InProgress;
        }

        // Mining done: deposit depleted.
        public void Complete(long missionId)
        {
            var m = GetById(missionId);
            if (m == null) return;
            m.Status = MissionStatus.Completed;
            _deposits.MarkDepleted(m.TargetDepositId);
        }

        // Mining aborted: deposit returned to the pool.
        public void Fail(long missionId)
        {
            var m = GetById(missionId);
            if (m == null) return;
            m.Status = MissionStatus.Failed;
            _deposits.Release(m.TargetDepositId);
        }

        private bool HasActiveMissionFor(long depositId)
        {
            for (int i = 0; i < _state.Missions.Count; i++)
            {
                var m = _state.Missions[i];
                if (m.TargetDepositId == depositId &&
                    (m.Status == MissionStatus.PendingAssignment ||
                     m.Status == MissionStatus.Assigned ||
                     m.Status == MissionStatus.InProgress))
                    return true;
            }
            return false;
        }
    }
}
