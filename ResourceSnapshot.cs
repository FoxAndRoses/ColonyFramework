using System.Collections.Generic;

namespace ColonyFramework
{
    // Runtime-only (not persisted). Recomputed by ResourceTracker each cycle.
    public class ResourceSnapshot
    {
        public readonly Dictionary<string, double> Ore = new Dictionary<string, double>();
        public readonly Dictionary<string, double> Ingots = new Dictionary<string, double>();
        public readonly Dictionary<string, double> Components = new Dictionary<string, double>();
        public long LastUpdatedTicks;

        public void Clear()
        {
            Ore.Clear();
            Ingots.Clear();
            Components.Clear();
        }

        public double Total(Dictionary<string, double> dict)
        {
            double t = 0;
            foreach (var v in dict.Values) t += v;
            return t;
        }
    }
}
