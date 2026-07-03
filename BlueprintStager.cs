using System.Collections.Generic;
using VRageMath;
using IMySlimBlock = VRage.Game.ModAPI.IMySlimBlock;

namespace ColonyFramework
{
    // Shipyard-style build ordering for a projected blueprint, computed ONCE per mission from pure
    // block-list math (no engine queries):
    //   FRAME (0)     — structural blocks (no FatBlock = armor/beams) with 3+ faces exposed to the
    //                   outside of the blueprint: the skeleton's corners, edges and spars.
    //   INTERNALS (1) — every functional block (reactors, conveyors, thrusters, cockpits...): fitted
    //                   while the hull is still open, so they stay externally reachable.
    //   CLOSURE (2)   — the remaining structural skin (1-2 exposed faces, or fully enclosed): the
    //                   hull plates that would seal the interior go on LAST.
    // The welder picks targets in stage order (CanBuild still gates actual placement, so blocks whose
    // neighbours don't exist yet are skipped gracefully and picked up on a later pass).
    // Approximation note: occupancy uses each block's root cell (multi-cell blocks aren't expanded),
    // which can over-count "exposed" faces next to large blocks — worst case a skin plate is promoted
    // to frame. Ordering imperfection only; nothing breaks.
    public class BlueprintStager
    {
        public const int Frame = 0, Internals = 1, Closure = 2;

        private readonly Dictionary<Vector3I, int> _stageByCell = new Dictionary<Vector3I, int>();
        private bool _built;

        private static readonly Vector3I[] Faces =
        {
            new Vector3I(1, 0, 0), new Vector3I(-1, 0, 0),
            new Vector3I(0, 1, 0), new Vector3I(0, -1, 0),
            new Vector3I(0, 0, 1), new Vector3I(0, 0, -1)
        };

        // Classify every projected block once (call with the full projected block list).
        public void EnsureBuilt(List<IMySlimBlock> blocks)
        {
            if (_built) return;
            _built = true;
            _stageByCell.Clear();

            var occupied = new HashSet<Vector3I>();
            for (int i = 0; i < blocks.Count; i++) occupied.Add(blocks[i].Position);

            for (int i = 0; i < blocks.Count; i++)
            {
                var b = blocks[i];
                if (b.FatBlock != null) { _stageByCell[b.Position] = Internals; continue; } // functional

                int exposed = 0;
                for (int f = 0; f < 6; f++)
                    if (!occupied.Contains(b.Position + Faces[f])) exposed++;
                _stageByCell[b.Position] = exposed >= 3 ? Frame : Closure; // skeleton vs skin
            }
        }

        public int StageOf(IMySlimBlock b)
        {
            int s;
            return _stageByCell.TryGetValue(b.Position, out s) ? s : Internals;
        }

        public static string StageName(int stage)
        {
            return stage == Frame ? "frame" : stage == Internals ? "internals" : "hull closure";
        }
    }
}
