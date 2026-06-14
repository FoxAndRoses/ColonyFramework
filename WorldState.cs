using System.Collections.Generic;
using ProtoBuf;

namespace ColonyFramework
{
    [ProtoContract]
    public class WorldState
    {
        [ProtoMember(1)] public int SchemaVersion = 1;
        [ProtoMember(2)] public List<ColonyState> Colonies = new List<ColonyState>();
        [ProtoMember(3)] public long NextColonyId = 1;
    }
}
