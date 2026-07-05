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

        // Demand-driven cleanup: retire PENDING Mine missions whose ore isn't currently demanded
        // (assigned / in-flight ones finish normally). Also drains the legacy blanket-generated
        // backlog on old saves. Returns count retired.
        public int RetirePendingMineExcept(HashSet<string> demandedOres)
        {
            int retired = 0;
            for (int i = 0; i < _state.Missions.Count; i++)
            {
                var m = _state.Missions[i];
                if (m.Type != MissionType.Mine || m.Status != MissionStatus.PendingAssignment) continue;
                var d = _deposits.GetById(m.TargetDepositId);
                if (d == null || !demandedOres.Contains(d.OreType)) { Fail(m.Id); retired++; }
            }
            return retired;
        }

        // Up to 'allowed' concurrent Weld missions per projector (multi-welder: the count scales with
        // remaining work; the WeldCoordinator's bubbles keep the drones apart on the hull). Returns
        // how many new missions were created this call.
        public int EnsureWeldMissions(long projectorEntityId, long tick, int allowed)
        {
            int active = 0;
            for (int i = 0; i < _state.Missions.Count; i++)
            {
                var m = _state.Missions[i];
                if (m.Type == MissionType.Weld && m.TargetEntityId == projectorEntityId &&
                    (m.Status == MissionStatus.PendingAssignment ||
                     m.Status == MissionStatus.Assigned ||
                     m.Status == MissionStatus.InProgress))
                    active++;
            }
            int created = 0;
            for (; active < allowed; active++, created++)
            {
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
            }
            return created;
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
