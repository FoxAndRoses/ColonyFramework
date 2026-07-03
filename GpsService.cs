using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.Game.ModAPI;
using VRageMath;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

namespace ColonyFramework
{
    // HUD markers for the colony's ACTIVE missions only (never the whole deposit DB — that's
    // hundreds of points). Reconciles on a slow tick: one GPS per Assigned/InProgress mission at
    // its target, added for the colony founder, removed the moment the mission ends. Pure
    // observability — no gameplay effect.
    public class GpsService
    {
        private readonly Dictionary<long, IMyGps> _byMission = new Dictionary<long, IMyGps>(); // missionId -> gps
        private readonly List<long> _stale = new List<long>();

        public void Sync(Colony colony)
        {
            if (MyAPIGateway.Session == null || MyAPIGateway.Session.GPS == null) return;
            long identity = colony.State.FounderId;
            if (identity == 0) return;

            var ms = colony.Missions.Missions;
            var active = new HashSet<long>();
            for (int i = 0; i < ms.Count; i++)
            {
                var m = ms[i];
                if (m.Status != MissionStatus.Assigned && m.Status != MissionStatus.InProgress) continue;

                Vector3D pos;
                string name;
                if (m.Type == MissionType.Weld)
                {
                    var proj = MyAPIGateway.Entities.GetEntityById(m.TargetEntityId);
                    if (proj == null) continue;
                    pos = proj.GetPosition();
                    name = "Colony: welding site";
                }
                else if (m.Type == MissionType.Survey)
                {
                    var grid = MyAPIGateway.Entities.GetEntityById(m.AssignedAssetId) as IMyCubeGrid;
                    if (grid == null) continue;
                    pos = grid.GetPosition(); // the scout itself is the interesting point
                    name = m.TargetOre != null ? "Colony: surveying for " + m.TargetOre : "Colony: surveying";
                }
                else
                {
                    var dep = colony.Deposits.GetById(m.TargetDepositId);
                    if (dep == null) continue;
                    pos = dep.Position;
                    name = "Colony: mining " + dep.OreType;
                }

                active.Add(m.Id);
                IMyGps gps;
                if (!_byMission.TryGetValue(m.Id, out gps))
                {
                    gps = MyAPIGateway.Session.GPS.Create(name, "ColonyFramework mission " + m.Id, pos, true);
                    MyAPIGateway.Session.GPS.AddGps(identity, gps);
                    _byMission[m.Id] = gps;
                }
                else if (m.Type == MissionType.Survey && Vector3D.DistanceSquared(gps.Coords, pos) > 100.0 * 100.0)
                {
                    // The scout marker follows the drone loosely (re-pin when it has moved >100 m).
                    MyAPIGateway.Session.GPS.RemoveGps(identity, gps);
                    gps = MyAPIGateway.Session.GPS.Create(name, "ColonyFramework mission " + m.Id, pos, true);
                    MyAPIGateway.Session.GPS.AddGps(identity, gps);
                    _byMission[m.Id] = gps;
                }
            }

            // Mission ended -> marker gone.
            _stale.Clear();
            foreach (var kv in _byMission)
                if (!active.Contains(kv.Key)) _stale.Add(kv.Key);
            for (int i = 0; i < _stale.Count; i++)
            {
                MyAPIGateway.Session.GPS.RemoveGps(identity, _byMission[_stale[i]]);
                _byMission.Remove(_stale[i]);
            }
        }
    }
}
