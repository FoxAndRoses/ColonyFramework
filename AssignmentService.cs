using Sandbox.ModAPI;
using VRage.ModAPI;
using VRage.Utils;
using VRageMath;

namespace ColonyFramework
{
    public class AssignmentService
    {
        public void ValidateAndAssign(Colony colony)
        {
            ValidateAssets(colony);
            AssignPending(colony);
        }

        private const int OfflineTicksBeforePurge = 2; // 2 validation cycles ≈ 6s (ValidateAssets runs every 180 ticks / 3s)

        private void ValidateAssets(Colony colony)
        {
            var list = colony.Assets.Assets;
            for (int i = list.Count - 1; i >= 0; i--)
            {
                var a = list[i];
                var ent = MyAPIGateway.Entities.GetEntityById(a.EntityId);
                if (ent == null)
                {
                    a.OfflineTicks++;
                    a.Status = AssetStatus.Offline;

                    if (a.OfflineTicks >= OfflineTicksBeforePurge)
                    {
                        if (a.AssignedMissionId != 0)
                        {
                            colony.Missions.Fail(a.AssignedMissionId);
                        }
                        colony.Assets.Unregister(a.EntityId);
                        MyLog.Default.WriteLineAndConsole(string.Format(
                            "[ColonyFramework] Asset '{0}' ({1}) purged after {2} offline ticks (entity gone)",
                            a.Name, a.EntityId, a.OfflineTicks));
                        continue;
                    }

                    if (a.AssignedMissionId != 0 && a.OfflineTicks == 1)
                    {
                        colony.Missions.Fail(a.AssignedMissionId);
                        a.AssignedMissionId = 0;
                    }
                }
                else
                {
                    a.OfflineTicks = 0;
                    a.LastPosition = ent.GetPosition();
                    if (a.Status == AssetStatus.Offline)
                        a.Status = (a.AssignedMissionId != 0) ? AssetStatus.Assigned : AssetStatus.Idle;
                }
            }
        }

        private void AssignPending(Colony colony)
        {
            var ms = colony.Missions.Missions;
            for (int i = 0; i < ms.Count; i++)
            {
                var m = ms[i];
                if (m.Status != MissionStatus.PendingAssignment) continue;

                var dep = colony.Deposits.GetById(m.TargetDepositId);
                if (dep == null) continue;
                Vector3D dpos = dep.Position;

                AssetRecord best = null;
                double bestSq = double.MaxValue;
                var assets = colony.Assets.Assets;
                for (int j = 0; j < assets.Count; j++)
                {
                    var a = assets[j];
                    if (a.Status != AssetStatus.Idle) continue;
                    double sq = Vector3D.DistanceSquared(a.LastPosition, dpos);
                    if (sq < bestSq) { bestSq = sq; best = a; }
                }
                if (best == null) break;

                if (colony.Missions.Assign(m.Id, best.EntityId))
                {
                    best.Status = AssetStatus.Assigned;
                    best.AssignedMissionId = m.Id;
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] Assigned mission {0} (deposit {1}) to asset {2}",
                        m.Id, m.TargetDepositId, best.EntityId));
                }
            }
        }
    }
}
