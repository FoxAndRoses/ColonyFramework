# Colony Framework

A Space Engineers mod that coordinates autonomous mining drones into a believable,
self-managing mining colony. The intelligence is in the coordination — mining stays
physical (real drills carving real voxels), not faked.

**Status:** In development. PoCs 1–3 validated in-game; closing the full mine-and-return loop next.

🌐 **Project site:** https://foxandroses.github.io/ColonyFramework/

## What it does

- **Colony identity** — a Colony Core block founds a colony (name, founder, creation date); multi-colony support keyed by faction or player.
- **Ore discovery** — bounded LOD voxel scans, manual (`/colony scan`) and automatic via round-robin powered ore detectors, with a 50 m dedup merge radius.
- **Mission system** — one mine mission per unclaimed deposit, with a full claim → assign → in-progress → complete/fail state machine.
- **Asset assignment** — register miner grids; offline detection and nearest-idle assignment of pending missions.
- **Autonomous drones** — commission self-test → autopilot transit → gyro-stabilized bore → cargo/timeout watchdogs → retreat climb.
- **Resource tracking** — owner-filtered grid-group scan totalling ore, ingots, and components (30 s cadence + on demand).

## Chat commands

`/colony` — `info`, `scan`, `missions`, `register`, `assets`, `resources`, `dispatch`, `abort`, `recall`.

## Architecture

Dependencies flow one direction only: **Data → Domain → Services → Host**. Managers are
engine-free and unit-testable; the session is host-only; every state mutation is
server-authoritative; save schema is append-only. See [ARCHITECTURE.md](ARCHITECTURE.md)
and [ROADMAP.md](ROADMAP.md) for detail.

## Build

Built with the MDK² toolchain (`dotnet build` → pack → deploy).

---

*Not affiliated with Keen Software House. Space Engineers is a trademark of Keen Software House.*
