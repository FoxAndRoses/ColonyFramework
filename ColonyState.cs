using System.Collections.Generic;
using ProtoBuf;

namespace ColonyFramework
{
    [ProtoContract]
    public class ColonyState
    {
        [ProtoMember(1)] public long NextDepositId = 1;
        [ProtoMember(2)] public List<DepositRecord> Deposits = new List<DepositRecord>();
        [ProtoMember(3)] public long NextMissionId = 1;
        [ProtoMember(4)] public List<Mission> Missions = new List<Mission>();
        [ProtoMember(5)] public List<AssetRecord> Assets = new List<AssetRecord>();
        [ProtoMember(6)] public long OwnerKey;
        [ProtoMember(7)] public long CoreEntityId;
        [ProtoMember(8)] public bool Active = true;
        [ProtoMember(9)] public long ColonyId;
        [ProtoMember(10)] public string Name;
        [ProtoMember(11)] public long FounderId;
        [ProtoMember(12)] public long CreatedGameTicks;
        [ProtoMember(13)] public double SurveyedRadius;   // ore-survey ring cursor: how far out we've scanned
        [ProtoMember(14)] public double SurveyedAngleDeg; // ...and how far around the current ring
    }
}
