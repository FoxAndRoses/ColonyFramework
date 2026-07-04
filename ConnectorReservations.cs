using System;
using System.Collections.Generic;
using Sandbox.ModAPI;
using VRageMath;
using MyShipConnectorStatus = Sandbox.ModAPI.Ingame.MyShipConnectorStatus;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;

namespace ColonyFramework
{
    // Base-connector traffic control for multi-drone fleets. FindFreeConnectorOnGroup only skips
    // LOCKED connectors, so two drones returning at once would both pick the same "free" connector
    // and collide mid-shimmy. This registry hands each drone a distinct connector: Acquire reserves
    // (and re-acquiring refreshes), Release frees on lock/complete/failure, and reservations expire
    // after a few minutes as crash-safety. In-memory, owned by DroneExecutor and passed into
    // controller ticks (no ambient globals).
    public class ConnectorReservations
    {
        private const double ExpirySecs = 300.0;

        private class Entry { public long ConnectorId; public DateTime At; }
        private readonly Dictionary<long, Entry> _byDrone = new Dictionary<long, Entry>(); // droneEntityId -> reservation
        private readonly HashSet<long> _reservedBuf = new HashSet<long>();
        private readonly List<long> _sweep = new List<long>();

        // The connector this drone should dock at: its existing reservation (refreshed) or the
        // nearest connector no other drone has reserved. Null = everything is taken/locked.
        public IMyShipConnector Acquire(IMyCubeGrid coreGrid, Vector3D nearTo, long droneEntityId)
        {
            SweepExpired();

            Entry mine;
            if (_byDrone.TryGetValue(droneEntityId, out mine))
            {
                var kept = MyAPIGateway.Entities.GetEntityById(mine.ConnectorId) as IMyShipConnector;
                if (kept != null && kept.Status != MyShipConnectorStatus.Connected)
                {
                    mine.At = DateTime.UtcNow;
                    return kept;
                }
                _byDrone.Remove(droneEntityId); // reservation went stale (connector gone or taken)
            }

            _reservedBuf.Clear();
            foreach (var kv in _byDrone) _reservedBuf.Add(kv.Value.ConnectorId);

            var con = DroneUtil.FindFreeConnectorOnGroup(coreGrid, nearTo, _reservedBuf);
            if (con != null)
                _byDrone[droneEntityId] = new Entry { ConnectorId = con.EntityId, At = DateTime.UtcNow };
            return con;
        }

        // Non-reserving peek — "is there any point flying home to dock?" (idle parker's decision).
        public bool AnyFree(IMyCubeGrid coreGrid, Vector3D nearTo)
        {
            SweepExpired();
            _reservedBuf.Clear();
            foreach (var kv in _byDrone) _reservedBuf.Add(kv.Value.ConnectorId);
            return DroneUtil.FindFreeConnectorOnGroup(coreGrid, nearTo, _reservedBuf) != null;
        }

        public void Release(long droneEntityId)
        {
            _byDrone.Remove(droneEntityId);
        }

        private void SweepExpired()
        {
            _sweep.Clear();
            foreach (var kv in _byDrone)
                if ((DateTime.UtcNow - kv.Value.At).TotalSeconds > ExpirySecs) _sweep.Add(kv.Key);
            for (int i = 0; i < _sweep.Count; i++) _byDrone.Remove(_sweep[i]);
        }
    }
}
