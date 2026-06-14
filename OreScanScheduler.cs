using System;
using System.Collections.Generic;
using Sandbox.ModAPI;
using VRage.ModAPI;
using VRage.Utils;
using VRageMath;
using IMyCubeGrid = VRage.Game.ModAPI.IMyCubeGrid;
using IMyOreDetector = Sandbox.ModAPI.Ingame.IMyOreDetector;

namespace ColonyFramework
{
    public class OreScanScheduler
    {
        private const double AutoScanRadius = 150.0;
        private const int AutoScanLod = 2;
        private const double MoveGateSq = 10.0 * 10.0;

        private readonly OreScanner _scanner = new OreScanner();
        private readonly List<IMyOreDetector> _detectors = new List<IMyOreDetector>();
        private readonly Dictionary<long, Vector3D> _lastScanPos = new Dictionary<long, Vector3D>();
        private int _rrIndex;

        public void RefreshDetectors()
        {
            try
            {
                _detectors.Clear();
                var ents = new HashSet<IMyEntity>();
                MyAPIGateway.Entities.GetEntities(ents, e => e is IMyCubeGrid);

                var perGrid = new List<IMyOreDetector>();
                foreach (var ent in ents)
                {
                    var grid = ent as IMyCubeGrid;
                    if (grid == null) continue;
                    var ts = MyAPIGateway.TerminalActionsHelper.GetTerminalSystemForGrid(grid);
                    if (ts == null) continue;
                    perGrid.Clear();
                    ts.GetBlocksOfType(perGrid);
                    for (int i = 0; i < perGrid.Count; i++)
                        _detectors.Add(perGrid[i]);
                }
                if (_rrIndex >= _detectors.Count) _rrIndex = 0;
            }
            catch (Exception e)
            {
                MyLog.Default.WriteLineAndConsole("[ColonyFramework] RefreshDetectors error: " + e.Message);
            }
        }

        public void Step(ColonyRegistry registry, ref long scanTick)
        {
            try
            {
                if (_detectors.Count == 0) { RefreshDetectors(); if (_detectors.Count == 0) return; }
                if (_rrIndex >= _detectors.Count) _rrIndex = 0;
                var det = _detectors[_rrIndex];
                _rrIndex++;

                if (det == null || !det.IsWorking) return;

                // Resolve the colony by the detector's owner; only feed an active (cored) colony.
                long ownerKey = Ownership.ResolveOwnerKey(det.OwnerId);
                var colony = registry.Get(ownerKey);
                if (colony == null || !colony.Active) return;

                Vector3D pos = det.GetPosition();
                Vector3D last;
                if (_lastScanPos.TryGetValue(det.EntityId, out last)
                    && Vector3D.DistanceSquared(pos, last) < MoveGateSq)
                    return;

                long owner = det.CubeGrid != null ? det.CubeGrid.EntityId : det.EntityId;
                scanTick++;
                int cells = _scanner.Scan(colony.Deposits, pos, AutoScanRadius, AutoScanLod, owner, scanTick);
                _lastScanPos[det.EntityId] = pos;

                if (cells > 0)
                    MyLog.Default.WriteLineAndConsole(string.Format(
                        "[ColonyFramework] Colony {0} auto-scan detector {1}: {2} cells, deposits: {3}",
                        ownerKey, det.EntityId, cells, colony.Deposits.Deposits.Count));
            }
            catch (Exception e)
            {
                MyLog.Default.WriteLineAndConsole("[ColonyFramework] ScanStep error: " + e.Message);
            }
        }

        public void Clear()
        {
            _detectors.Clear();
            _lastScanPos.Clear();
        }
    }
}
