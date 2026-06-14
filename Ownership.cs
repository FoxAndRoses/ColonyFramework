using Sandbox.ModAPI;

namespace ColonyFramework
{
    // Colony key for an owner: faction id if in a faction, else the identity id. 0 = unowned/default.
    public static class Ownership
    {
        public static long ResolveOwnerKey(long ownerIdentityId)
        {
            if (ownerIdentityId == 0) return 0;
            var faction = MyAPIGateway.Session.Factions.TryGetPlayerFaction(ownerIdentityId);
            return faction != null ? faction.FactionId : ownerIdentityId;
        }
    }
}
