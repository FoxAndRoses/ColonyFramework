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

        // One Mine mission per Unclaimed deposit with no active mission. Ice deposits are gated by
        // demand (allowIce, decided by the caller from gas equipment + reserve stock) — nobody wants
        // drones dutifully strip-mining an ice sheet no recipe needs. Returns count created.
        public int GenerateMineMissions(long tick, bool allowIce = true)
        {
            int created = 0;
            var deps = _deposits.Deposits;
            for (int i = 0; i < deps.Count; i++)
            {
                var d = deps[i];
                if (d.Status != DepositStatus.Unclaimed) continue;
                if (!allowIce && d.OreType == "Ice") continue;
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

            // Ice demand just went away: retire ice missions still waiting for a drone (assigned /
            // in-flight ones finish normally) so the queue doesn't hold dead weight.
            if (!allowIce)
                for (int i = 0; i < _state.Missions.Count; i++)
                {
                    var m = _state.Missions[i];
                    if (m.Type != MissionType.Mine || m.Status != MissionStatus.PendingAssignment) continue;
                    var d = _deposits.GetById(m.TargetDepositId);
                    if (d != null && d.OreType == "Ice") Fail(m.Id);
                }
            return created;
        }

        // One Weld mission per projector: the welder drone builds the projected blueprint. Created when
        // production sees an active projection with remaining blocks; no-op while one is already active.
        public bool EnsureWeldMission(long projectorEntityId, long tick)
        {
            for (int i = 0; i < _state.Missions.Count; i++)
            {
                var m = _state.Missions[i];
                if (m.Type == MissionType.Weld && m.TargetEntityId == projectorEntityId &&
                    (m.Status == MissionStatus.PendingAssignment ||
                     m.Status == MissionStatus.Assigned ||
                     m.Status == MissionStatus.InProgress))
                    return false;
            }
            _state.Missions.Add(new Mission
            {
                Id = _state.NextMissionId++,
                Type = MissionType.Weld,
                TargetDepositId = 0,
                TargetEntityId = projectorEntityId,
                AssignedAssetId = 0,
                Status = MissionStatus.PendingAssignment,
                CreatedTick = tick
            });
            return true;
        }

        // One Survey mission at a time per colony: a scout flies the next ring segment scanning for ore
        // (oreType = the specific ore production needs, or null for a general first-build survey).
        public bool EnsureSurveyMission(string oreType, long tick)
        {
            for (int i = 0; i < _state.Missions.Count; i++)
            {
                var m = _state.Missions[i];
                if (m.Type == MissionType.Survey &&
                    (m.Status == MissionStatus.PendingAssignment ||
                     m.Status == MissionStatus.Assigned ||
                     m.Status == MissionStatus.InProgress))
                    return false;
            }
            _state.Missions.Add(new Mission
            {
                Id = _state.NextMissionId++,
                Type = MissionType.Survey,
                TargetOre = oreType,
                AssignedAssetId = 0,
                Status = MissionStatus.PendingAssignment,
                CreatedTick = tick
            });
            return true;
        }

        // Demand-driven: create one Mine mission for a specific deposit (e.g. production needs its ore),
        // unless that deposit already has an active mission. Returns true if a mission was created.
        public bool CreateMineMission(long depositId, long tick)
        {
            if (HasActiveMissionFor(depositId)) return false;
            var d = _deposits.GetById(depositId);
            if (d == null || d.Status != DepositStatus.Unclaimed) return false;

            _state.Missions.Add(new Mission
            {
                Id = _state.NextMissionId++,
                Type = MissionType.Mine,
                TargetDepositId = depositId,
                AssignedAssetId = 0,
                Status = MissionStatus.PendingAssignment,
                CreatedTick = tick
            });
            return true;
        }

        public Mission GetById(long id)
        {
            for (int i = 0; i < _state.Missions.Count; i++)
                if (_state.Missions[i].Id == id) return _state.Missions[i];
            return null;
        }

        // PendingAssignment -> Assigned. Mine missions claim their target deposit; Weld missions have
        // no deposit (their target is a projector entity), so they skip the claim.
        public bool Assign(long missionId, long assetEntityId)
        {
            var m = GetById(missionId);
            if (m == null || m.Status != MissionStatus.PendingAssignment) return false;
            if (m.Type == MissionType.Mine && !_deposits.TryClaim(m.TargetDepositId, assetEntityId)) return false;
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

        // This load delivered, but the deposit still has ore: complete the mission and RETURN the deposit
        // to the pool (Claimed → Unclaimed) so it gets re-assigned and finished across more loads.
        public void CompleteAndRelease(long missionId)
        {
            var m = GetById(missionId);
            if (m == null) return;
            m.Status = MissionStatus.Completed;
            _deposits.Release(m.TargetDepositId);
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
