using Sandbox.ModAPI;
using VRage.ModAPI;
using VRage.Utils;
using VRageMath;

namespace ColonyFramework
{
    public class AssignmentService
    {
        private readonly System.Collections.Generic.Dictionary<long, System.DateTime> _noDepositChat
            = new System.Collections.Generic.Dictionary<long, System.DateTime>();
        private const double NoDepositChatSecs = 120.0;

        public void ValidateAndAssign(Colony colony)
        {
            ValidateAssets(colony);
            AssignPending(colony);
            ReportIfOutOfDeposits(colony);
        }

        // If a drone wants work but the colony has no ore left to mine (no unclaimed deposits and no
        // active missions), tell the player — throttled so it doesn't spam.
        private void ReportIfOutOfDeposits(Colony colony)
        {
            bool idleWanting = false;
            var assets = colony.Assets.Assets;
            for (int i = 0; i < assets.Count; i++)
                if (assets[i].Status == AssetStatus.Idle && assets[i].AutoDispatchEnabled) { idleWanting = true; break; }
            if (!idleWanting) return;

            var deps = colony.Deposits.Deposits;
            for (int i = 0; i < deps.Count; i++)
                if (deps[i].Status == DepositStatus.Unclaimed) return; // ore still available to assign

            var ms = colony.Missions.Missions;
            for (int i = 0; i < ms.Count; i++)
                if (ms[i].Status == MissionStatus.PendingAssignment ||
                    ms[i].Status == MissionStatus.Assigned ||
                    ms[i].Status == MissionStatus.InProgress) return; // work still in flight (may release more)

            System.DateTime last;
            if (_noDepositChat.TryGetValue(colony.OwnerKey, out last) &&
                (System.DateTime.UtcNow - last).TotalSeconds < NoDepositChatSecs) return;
            _noDepositChat[colony.OwnerKey] = System.DateTime.UtcNow;

            // First-build / out-of-ore: send a scout instead of only asking the player.
            long tick = MyAPIGateway.Session.GameDateTime.Ticks;
            bool created = colony.Missions.EnsureSurveyMission(null, tick);
            if (!MyAPIGateway.Utilities.IsDedicated)
                MyAPIGateway.Utilities.ShowMessage("Colony", created
                    ? "no known ore deposits — dispatching a survey drone"
                    : "no known ore deposits remain — scan for more (/colony scan)");
            MyLog.Default.WriteLineAndConsole("[ColonyFramework] colony " + colony.OwnerKey +
                (created ? ": out of deposits — survey mission created" : ": out of deposits, survey already active"));
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

                // Mission target position: deposit for Mine, projector for Weld, the core for Survey.
                // Survey is a CAPABILITY (any idle drone with a working ore detector), not an asset type.
                Vector3D dpos;
                AssetType? wantType;
                if (m.Type == MissionType.Weld)
                {
                    var proj = MyAPIGateway.Entities.GetEntityById(m.TargetEntityId);
                    if (proj == null) continue;
                    dpos = proj.GetPosition();
                    wantType = AssetType.Welder;
                }
                else if (m.Type == MissionType.Survey)
                {
                    var core = MyAPIGateway.Entities.GetEntityById(colony.State.CoreEntityId);
                    if (core == null) continue;
                    dpos = core.GetPosition();
                    wantType = null;
                }
                else
                {
                    var dep = colony.Deposits.GetById(m.TargetDepositId);
                    if (dep == null) continue;
                    dpos = dep.Position;
                    wantType = AssetType.Miner;
                }

                AssetRecord best = null;
                double bestSq = double.MaxValue;
                var assets = colony.Assets.Assets;
                for (int j = 0; j < assets.Count; j++)
                {
                    var a = assets[j];
                    if (a.Status != AssetStatus.Idle) continue;
                    if (wantType.HasValue && a.Type != wantType.Value) continue;
                    if (m.Type == MissionType.Survey)
                    {
                        var g = MyAPIGateway.Entities.GetEntityById(a.EntityId) as VRage.Game.ModAPI.IMyCubeGrid;
                        if (g == null || DroneUtil.FindOreDetector(g) == null) continue; // needs a working detector
                    }
                    double sq = Vector3D.DistanceSquared(a.LastPosition, dpos);
                    if (sq < bestSq) { bestSq = sq; best = a; }
                }
                if (best == null) continue; // no capable idle asset — other mission types may still match

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
