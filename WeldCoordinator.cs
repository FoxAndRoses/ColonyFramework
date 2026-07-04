using System;
using System.Collections.Generic;
using VRageMath;

namespace ColonyFramework
{
    // Multi-welder separation on one projection — mutual bubbles, not progress gates:
    //  - block CLAIMS: no two weld missions ever target the same projected cell;
    //  - a spacing BUBBLE: candidates near another mission's claimed block OR another welder drone's
    //    live position are skipped, so welders spread across different patches of hull and never
    //    converge on one spot (mid-flight separation itself is the AvoidanceProbe's job — welder
    //    drones are ordinary third-party grids to each other).
    // In-memory, owned by DroneExecutor, passed into WelderController ticks. Claims and presence
    // expire on their own, so a dropped controller never wedges a cell.
    public class WeldCoordinator
    {
        public const double WelderBubble = 10.0; // m — min separation between welders' work spots

        private const double ClaimExpirySecs = 120.0;
        private const double PresenceExpirySecs = 30.0;

        private class Claim { public long ProjectorId; public Vector3I Cell; public Vector3D Pos; public DateTime At; }
        private class Presence { public Vector3D Pos; public DateTime At; }

        private readonly Dictionary<long, Claim> _claims = new Dictionary<long, Claim>();       // missionId -> claimed block
        private readonly Dictionary<long, Presence> _presence = new Dictionary<long, Presence>(); // missionId -> drone position
        private readonly List<long> _sweep = new List<long>();

        // Called every welder tick — where this mission's drone currently is.
        public void UpdatePresence(long missionId, Vector3D pos)
        {
            Presence p;
            if (!_presence.TryGetValue(missionId, out p)) { p = new Presence(); _presence[missionId] = p; }
            p.Pos = pos;
            p.At = DateTime.UtcNow;
        }

        // Claim (or refresh) this mission's target block. False if another mission holds the cell.
        public bool TryClaim(long missionId, long projectorId, Vector3I cell, Vector3D worldPos)
        {
            SweepExpired();
            foreach (var kv in _claims)
                if (kv.Key != missionId && kv.Value.ProjectorId == projectorId && kv.Value.Cell == cell)
                    return false;
            Claim c;
            if (!_claims.TryGetValue(missionId, out c)) { c = new Claim(); _claims[missionId] = c; }
            c.ProjectorId = projectorId;
            c.Cell = cell;
            c.Pos = worldPos;
            c.At = DateTime.UtcNow;
            return true;
        }

        public void ReleaseClaim(long missionId)
        {
            _claims.Remove(missionId);
        }

        // Is this candidate inside another welder's bubble (their claimed block or their drone)?
        public bool NearOthers(long missionId, Vector3D worldPos, double radius)
        {
            SweepExpired();
            double r2 = radius * radius;
            foreach (var kv in _claims)
                if (kv.Key != missionId && Vector3D.DistanceSquared(kv.Value.Pos, worldPos) < r2) return true;
            foreach (var kv in _presence)
                if (kv.Key != missionId && (DateTime.UtcNow - kv.Value.At).TotalSeconds < PresenceExpirySecs
                    && Vector3D.DistanceSquared(kv.Value.Pos, worldPos) < r2) return true;
            return false;
        }

        private void SweepExpired()
        {
            _sweep.Clear();
            foreach (var kv in _claims)
                if ((DateTime.UtcNow - kv.Value.At).TotalSeconds > ClaimExpirySecs) _sweep.Add(kv.Key);
            for (int i = 0; i < _sweep.Count; i++) _claims.Remove(_sweep[i]);
        }
    }
}
