using System.Collections.Generic;
using VRageMath;

namespace ColonyFramework
{
    // Server-side authority over the deposit database.
    // Owns ALL DepositRecord state transitions. Nothing else mutates deposits directly.
    public class DepositManager
    {
        private const double MergeRadius = 50.0;
        private const double MergeRadiusSq = MergeRadius * MergeRadius;

        private readonly ColonyState _state;

        public DepositManager(ColonyState state) { _state = state; }

        public IReadOnlyList<DepositRecord> Deposits { get { return _state.Deposits; } }

        // New deposit, or the existing one if a same-ore deposit is within MergeRadius. No duplicates.
        public DepositRecord AddDeposit(string oreType, Vector3D position, long discoveredBy, long tick)
        {
            for (int i = 0; i < _state.Deposits.Count; i++)
            {
                var d = _state.Deposits[i];
                if (d.OreType == oreType && Vector3D.DistanceSquared(d.Position, position) <= MergeRadiusSq)
                    return d;
            }

            var record = new DepositRecord
            {
                Id = _state.NextDepositId++,
                OreType = oreType,
                Position = position,
                DiscoveredByEntityId = discoveredBy,
                DiscoveredTick = tick,
                Status = DepositStatus.Unclaimed,
                ClaimedByEntityId = 0
            };
            _state.Deposits.Add(record);
            return record;
        }

        public DepositRecord GetById(long id)
        {
            for (int i = 0; i < _state.Deposits.Count; i++)
                if (_state.Deposits[i].Id == id) return _state.Deposits[i];
            return null;
        }

        // Unclaimed -> Claimed. False if missing or not Unclaimed.
        public bool TryClaim(long depositId, long assetEntityId)
        {
            var d = GetById(depositId);
            if (d == null || d.Status != DepositStatus.Unclaimed) return false;
            d.Status = DepositStatus.Claimed;
            d.ClaimedByEntityId = assetEntityId;
            return true;
        }

        // Claimed -> Unclaimed. Clears the claim.
        public void Release(long depositId)
        {
            var d = GetById(depositId);
            if (d == null || d.Status != DepositStatus.Claimed) return;
            d.Status = DepositStatus.Unclaimed;
            d.ClaimedByEntityId = 0;
        }

        // -> Depleted. Terminal.
        public void MarkDepleted(long depositId)
        {
            var d = GetById(depositId);
            if (d == null) return;
            d.Status = DepositStatus.Depleted;
            d.ClaimedByEntityId = 0;
        }

        // Nearest Unclaimed deposit, optional ore filter. minDist = exclusion radius around 'from':
        // never hand out a deposit UNDER the base — a drone would happily undermine the colony's
        // own foundation (MISSION.md hygiene).
        public DepositRecord FindNearestUnclaimed(Vector3D from, string oreType = null, double minDist = 0)
        {
            DepositRecord best = null;
            double bestSq = double.MaxValue;
            double minSq = minDist * minDist;
            for (int i = 0; i < _state.Deposits.Count; i++)
            {
                var d = _state.Deposits[i];
                if (d.Status != DepositStatus.Unclaimed) continue;
                if (oreType != null && d.OreType != oreType) continue;
                double sq = Vector3D.DistanceSquared(d.Position, from);
                if (sq < minSq) continue;
                if (sq < bestSq) { bestSq = sq; best = d; }
            }
            return best;
        }
    }
}
