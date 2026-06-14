using ProtoBuf;
using VRage;
using VRageMath;

namespace ColonyFramework
{
    // State transitions: Unclaimed -> Claimed -> (Depleted | back to Unclaimed if a miner aborts)
    public enum DepositStatus
    {
        Unclaimed = 0,
        Claimed   = 1,
        Depleted  = 2
    }

    [ProtoContract]
    public class DepositRecord
    {
        [ProtoMember(1)] public long Id;
        [ProtoMember(2)] public string OreType;
        [ProtoMember(3)] public SerializableVector3D Position;
        [ProtoMember(4)] public long DiscoveredByEntityId;
        [ProtoMember(5)] public long DiscoveredTick;
        [ProtoMember(6)] public DepositStatus Status;
        [ProtoMember(7)] public long ClaimedByEntityId;
    }
}
