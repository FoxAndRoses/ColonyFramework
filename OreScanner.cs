using System;
using System.Collections.Generic;
using Sandbox.Definitions;
using Sandbox.ModAPI;
using VRage.Game.ModAPI;
using VRage.ModAPI;
using VRage.Voxels;
using VRageMath;

namespace ColonyFramework
{
    public class OreScanner
    {
        private const byte IsoLevel = 127;

        public int Scan(DepositManager deposits, Vector3D center, double radius, int lod, long sourceEntityId, long tick)
        {
            var matToOre = new Dictionary<byte, string>();
            foreach (var mat in MyDefinitionManager.Static.GetVoxelMaterialDefinitions())
            {
                if (mat.IsRare && !string.IsNullOrEmpty(mat.MinedOre))
                    matToOre[mat.Index] = mat.MinedOre;
            }
            if (matToOre.Count == 0) return 0;

            double scale = 1 << lod;
            var voxels = new List<IMyVoxelBase>();
            MyAPIGateway.Session.VoxelMaps.GetInstances(voxels, x => true);

            int cellHits = 0;
            double rSq = radius * radius;
            var cache = new MyStorageData();

            foreach (var voxel in voxels)
            {
                var storage = voxel.Storage;
                if (storage == null) continue;

                Vector3D localMin = (center - radius) - voxel.PositionLeftBottomCorner;
                Vector3D localMax = (center + radius) - voxel.PositionLeftBottomCorner;

                Vector3I min = new Vector3I(
                    (int)Math.Floor(localMin.X / scale),
                    (int)Math.Floor(localMin.Y / scale),
                    (int)Math.Floor(localMin.Z / scale));
                Vector3I max = new Vector3I(
                    (int)Math.Ceiling(localMax.X / scale),
                    (int)Math.Ceiling(localMax.Y / scale),
                    (int)Math.Ceiling(localMax.Z / scale));

                Vector3I sizeMax = new Vector3I(
                    storage.Size.X >> lod, storage.Size.Y >> lod, storage.Size.Z >> lod) - Vector3I.One;

                min = Vector3I.Clamp(min, Vector3I.Zero, sizeMax);
                max = Vector3I.Clamp(max, Vector3I.Zero, sizeMax);
                if (min.X > max.X || min.Y > max.Y || min.Z > max.Z) continue;

                cache.Resize(max - min + Vector3I.One);
                storage.ReadRange(cache, MyStorageDataTypeFlags.ContentAndMaterial, lod, min, max);

                Vector3I dims = max - min;
                Vector3I p;
                for (p.Z = 0; p.Z <= dims.Z; p.Z++)
                for (p.Y = 0; p.Y <= dims.Y; p.Y++)
                for (p.X = 0; p.X <= dims.X; p.X++)
                {
                    int li = cache.ComputeLinear(ref p);
                    if (cache.Content(li) <= IsoLevel) continue;

                    string ore;
                    if (!matToOre.TryGetValue(cache.Material(li), out ore)) continue;

                    Vector3I vc = min + p;
                    Vector3D world = voxel.PositionLeftBottomCorner + new Vector3D(
                        vc.X * scale + scale * 0.5,
                        vc.Y * scale + scale * 0.5,
                        vc.Z * scale + scale * 0.5);

                    if (Vector3D.DistanceSquared(world, center) > rSq) continue;

                    deposits.AddDeposit(ore, world, sourceEntityId, tick);
                    cellHits++;
                }
            }
            return cellHits;
        }
    }
}
