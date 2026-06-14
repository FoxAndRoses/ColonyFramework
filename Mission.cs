using ProtoBuf;

namespace ColonyFramework
{
    public enum MissionType
    {
        Mine = 0
    }

    // Lifecycle: PendingAssignment -> Assigned -> InProgress -> (Completed | Failed)
    public enum MissionStatus
    {
        PendingAssignment = 0,
        Assigned          = 1,
        InProgress        = 2,
        Completed         = 3,
        Failed            = 4
    }

    [ProtoContract]
    public class Mission
    {
        [ProtoMember(1)] public long Id;
        [ProtoMember(2)] public MissionType Type;
        [ProtoMember(3)] public long TargetDepositId;
        [ProtoMember(4)] public long AssignedAssetId; // 0 = unassigned
        [ProtoMember(5)] public MissionStatus Status;
        [ProtoMember(6)] public long CreatedTick;
        [ProtoMember(7)] public int Phase; // 0=Commission, 1=Transit, 2=StartBore, 3=Mining, 4=Retreat
    }
}
