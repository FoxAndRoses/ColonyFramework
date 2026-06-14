using System.Collections.Generic;
using VRageMath;

namespace ColonyFramework
{
    // Server-side authority over the asset registry. Keyed by grid EntityId.
    public class AssetManager
    {
        private readonly ColonyState _state;
        public AssetManager(ColonyState state) { _state = state; }

        public IReadOnlyList<AssetRecord> Assets { get { return _state.Assets; } }

        public AssetRecord GetByEntityId(long entityId)
        {
            for (int i = 0; i < _state.Assets.Count; i++)
                if (_state.Assets[i].EntityId == entityId) return _state.Assets[i];
            return null;
        }

        // Adds a new asset, or refreshes an existing one (dedup by EntityId).
        public AssetRecord Register(long entityId, AssetType type, Vector3D pos, string name)
        {
            var existing = GetByEntityId(entityId);
            if (existing != null)
            {
                existing.LastPosition = pos;
                if (existing.Status == AssetStatus.Offline) existing.Status = AssetStatus.Idle;
                return existing;
            }
            var rec = new AssetRecord
            {
                EntityId = entityId,
                Type = type,
                Status = AssetStatus.Idle,
                AssignedMissionId = 0,
                LastPosition = pos,
                Name = name
            };
            _state.Assets.Add(rec);
            return rec;
        }

        public bool Unregister(long entityId)
        {
            for (int i = 0; i < _state.Assets.Count; i++)
                if (_state.Assets[i].EntityId == entityId) { _state.Assets.RemoveAt(i); return true; }
            return false;
        }
    }
}
