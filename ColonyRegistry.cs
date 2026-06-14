using System.Collections.Generic;

namespace ColonyFramework
{
    // Single source of truth for all colonies, keyed by owner (faction/identity).
    public class ColonyRegistry
    {
        private readonly WorldState _world;
        private readonly Dictionary<long, Colony> _byOwner = new Dictionary<long, Colony>();

        public ColonyRegistry(WorldState world)
        {
            _world = world;
            for (int i = 0; i < world.Colonies.Count; i++)
            {
                var cs = world.Colonies[i];
                if (cs.ColonyId == 0)
                {
                    cs.ColonyId = world.NextColonyId++;     // backfill colonies created before IDs existed
                    cs.Name = "Colony " + cs.ColonyId;       // regenerate auto-name (no user naming yet)
                }
                _byOwner[cs.OwnerKey] = new Colony(cs);
            }
        }

        public IEnumerable<Colony> Colonies { get { return _byOwner.Values; } }

        public Colony Get(long ownerKey)
        {
            Colony c;
            return _byOwner.TryGetValue(ownerKey, out c) ? c : null;
        }

        public Colony GetOrCreate(long ownerKey)
        {
            Colony c;
            if (_byOwner.TryGetValue(ownerKey, out c)) return c;
            var cs = new ColonyState { OwnerKey = ownerKey, Active = true, ColonyId = _world.NextColonyId++ };
            _world.Colonies.Add(cs);
            c = new Colony(cs);
            _byOwner[ownerKey] = c;
            return c;
        }
    }
}
