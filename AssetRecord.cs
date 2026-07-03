using ProtoBuf;
using VRage;
using VRageMath;

namespace ColonyFramework
{
    public enum AssetType { Miner = 0, Welder = 1 }

    public enum AssetStatus { Idle = 0, Assigned = 1, Offline = 2 }

    [ProtoContract]
    public class AssetRecord
    {
        [ProtoMember(1)] public long EntityId;
        [ProtoMember(2)] public AssetType Type;
        [ProtoMember(3)] public AssetStatus Status;
        [ProtoMember(4)] public long AssignedMissionId;
        [ProtoMember(5)] public SerializableVector3D LastPosition;
        [ProtoMember(6)] public string Name;
        [ProtoMember(7)] public int OfflineTicks; // incremented each validation tick while entity missing; removed at threshold
        [ProtoMember(8)] public bool AutoDispatchEnabled; // true = colony may auto-launch this drone's missions (set by /colony dispatch; cleared by abort/recall)
    }
}
