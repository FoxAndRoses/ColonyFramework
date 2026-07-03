# Colony Framework

A Space Engineers mod that coordinates autonomous drones into a believable,
self-managing colony — it surveys for ore, mines it, refines and assembles toward a
projected blueprint, and prints ships in shipyard stages. The intelligence is in the
coordination — everything stays physical (real drills, real welders, real voxels), not faked.

**Status:** In development. The full mining loop (dispatch → mine → return → dock → unload →
recharge → auto-redispatch) and the production pipeline are validated in-game. Obstacle
avoidance, welder drones, and survey drones are built and entering in-game validation.

🌐 **Project site:** https://foxandroses.github.io/ColonyFramework/

## What it does

- **Colony identity** — a Colony Core block founds a colony (name, founder, creation date); multi-colony support keyed by faction or player.
- **Ore discovery** — bounded LOD voxel scans, manual (`/colony scan`), automatic via any faction ore detector (movement-gated round-robin), and **survey drones** that fly expanding low-altitude search rings on demand when production needs an ore the colony doesn't know.
- **Autonomous mining** — commission self-test → cruise transit → gyro-stabilized + bore with **yield-based dynamic depth** (drills until the ore actually runs out), ice/stone **eject excursions** (junk is dumped on site, not hauled home), multi-load deposits (released, not depleted, until mined out), staged connector docking, cargo unload, **draw-derived recharge**, and automatic re-dispatch.
- **Production pipeline** — reads the projector blueprint's bill of materials, rolls it up components → ingots → ore against colony stock, auto-queues assemblers when materials suffice, keeps refineries fed, and creates targeted mining (or survey) missions for what's missing.
- **Welder drones (ship printing)** — load components at base, fly to the build site, and weld the projected blueprint block-by-block with radial outside-in approaches — in **shipyard stages**: frame → internals → hull closure, so interiors go in while the hull is open.
- **Reactive obstacle avoidance** — layered stateless sensing on all transit legs (terrain look-ahead via heightmap, 1 Hz broad-phase entity query, ray fan only when the corridor is blocked); no pathfinding, MP-cheap.
- **Mission system** — Mine / Weld / Survey missions with a full claim → assign → in-progress → complete/fail state machine; assignment matches asset type and capability (ore detector for surveys).
- **Observability** — `/colony status` live snapshot (every mission, drone, phase, survey coverage), HUD **GPS markers** for active mission targets, per-phase log telemetry, and fleet-wide `/colony recall`.
- **Resource tracking** — owner-filtered grid-group scan totalling ore, ingots, and components (30 s cadence + on demand).

## Chat commands

`/colony` — `help`, `info`, `scan`, `missions`, `register`, `assets`, `resources`, `build`,
`status`, `dispatch`, `abort`, `recall`. (`dispatch` enables autonomy; `abort`/`recall` park the fleet.)

## Architecture

Dependencies flow one direction only: **Data → Domain → Services → Host**. Managers are
engine-free and unit-testable; the session is host-only; every state mutation is
server-authoritative; save schema is append-only. See [ARCHITECTURE.md](ARCHITECTURE.md)
and [ROADMAP.md](ROADMAP.md) for detail.

## Build

Built with the MDK² toolchain (`dotnet build` → pack → deploy).

---

*Not affiliated with Keen Software House. Space Engineers is a trademark of Keen Software House.*
