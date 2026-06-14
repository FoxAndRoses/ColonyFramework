# ColonyFramework Architecture

## Purpose

A Space Engineers mod that creates the **appearance** of an intelligent mining colony through state management and coordination — not real AI. Mining stays physical (real drills, real voxels); the mod coordinates assets and missions, it does not fake-mine. The goal is believable colony behavior driven by scan data, not simulation shortcuts.

## Layering

Dependencies flow in one direction only. No layer may depend on a layer above it.

```
Data → Domain → Services → Host
```

| Layer | What it contains | Engine refs? |
|-------|-----------------|-------------|
| **Data** | Dumb serializable POCOs (`[ProtoContract]`). No logic, no SE API references. | No |
| **Domain** | Managers that operate on Data: enforce state transitions, own business rules. Engine-free. | No |
| **Services** | Touch the SE API; call into Domain to record results. | Yes |
| **Host** | `CommandCoreSession`: lifecycle, tick scheduling, composition root. No business logic. | Yes |

## Key Rules

- **Managers are engine-free.** `DepositManager`, `MissionManager`, `AssetManager` have zero SE API imports. They can be unit-tested without a running game.
- **The session is host-only.** `CommandCoreSession` wires things together and drives the tick; it contains no business logic of its own.
- **Pass Colony explicitly.** Services receive a `Colony` (or `ColonyRegistry`) parameter. No ambient global state (`static` singletons, etc.).
- **ProtoMember tags are append-only.** Never reuse a tag number on a different field. Never renumber existing fields. Old saves must always deserialize safely.
- **Server-authoritative.** Every state mutation is gated behind `IsServer()`. Clients receive a fresh empty state and do not mutate it.

## Single Source of Truth

- **`Colony`** bundles one colony's `ColonyState` with the three managers (`DepositManager`, `MissionManager`, `AssetManager`) that operate on it. It is the unit of per-colony work.
- **`ColonyRegistry`** holds all colonies keyed by `long ownerKey` (faction/player identity). `GetOrCreate(0)` returns the current default colony. Faction-scoped multi-colony support arrives in step 1b.
- Owner key `0` is the implicit default colony used throughout step 1a.

## File Inventory

### Data layer

| File | Purpose |
|------|---------|
| `WorldState.cs` | Serialization root saved to `colony_world.xml`; holds the list of all `ColonyState` entries, a schema version, and the next-colony-ID counter. |
| `ColonyState.cs` | Per-colony persisted bag: ID counters, deposit/mission/asset lists, owner key, core block entity ID, active flag, colony identity (name, founder, creation tick), and `Mission.Phase`. |
| `DepositRecord.cs` | Ore deposit POCO: position, ore type, discovered-by entity/tick, and `DepositStatus` (Unclaimed → Claimed → Depleted). |
| `Mission.cs` | Mining mission POCO: target deposit ID, assigned asset ID, `MissionStatus` lifecycle (PendingAssignment → Assigned → InProgress → Completed/Failed), and `Phase` (0 Transit → 1 StartBore → 2 Boring → 3 Retreating). |
| `AssetRecord.cs` | Registered miner grid POCO: entity ID, type, `AssetStatus` (Idle/Assigned/Offline), assigned mission ID, last known position, and display name. |

### Domain layer

| File | Purpose |
|------|---------|
| `Colony.cs` | Bundles one `ColonyState` with its three managers and a `ResourceSnapshot`; the unit passed to services and the composition boundary for a single colony. |
| `ColonyRegistry.cs` | Dictionary of all colonies keyed by owner; `GetOrCreate` mints new colonies on demand; constructor backfills missing `ColonyId` fields on legacy saves. |
| `DepositManager.cs` | Owns all `DepositRecord` state transitions: merge-radius dedup on add, claim/release/deplete, and nearest-unclaimed spatial query. |
| `MissionManager.cs` | Owns all `Mission` state transitions: generates Mine missions for unclaimed deposits, assigns (claiming the deposit), marks in-progress, completes (depleting), or fails (releasing). |
| `AssetManager.cs` | Owns the asset registry: register/unregister miner grids by entity ID with dedup, status refresh on re-register. |

### Services layer

| File | Purpose |
|------|---------|
| `Ownership.cs` | Resolves a player identity ID to an owner key: faction ID if in a faction, else identity ID, else 0. |
| `OreScanner.cs` | Stateless voxel scanner; reads LOD-parameterized storage ranges and feeds matching rare-ore cells into a given `DepositManager`. |
| `OreScanScheduler.cs` | Round-robin ore detector scan loop: maintains the detector list, applies the move-gate, resolves the target colony via `ColonyRegistry`, and calls `OreScanner.Scan`. |
| `AssignmentService.cs` | Per-tick asset validation (marks offline/restores grids) and nearest-idle-asset assignment of pending missions; calls `MissionManager` and updates `AssetRecord` status. |
| `ResourceSnapshot.cs` | Runtime-only (non-persisted) ore/ingot/component totals for one colony; recomputed by `ResourceTracker` each cycle. |
| `ResourceTracker.cs` | Scans the physical grid group rooted at the colony core, filtered to grids owned by the colony's owner key; accumulates items into a `ResourceSnapshot`. |
| `NavMath.cs` | Stateless navigation helpers: `ComputeStandoff` picks a 120 m standoff point above a deposit, gravity-aware (anti-gravity direction on a planet, approach vector in space). |
| `DroneUtil.cs` | Stateless grid/power helpers shared by the execution services: find RC/drills, cargo fill, reactor/battery checks, battery stored+output sum, the commissioning spike toggle, planet altitude, and grid release (gear unlock + connector disconnect). |
| `DispatchService.cs` | Chat-driven dispatch initiator: finds the first Assigned (or stuck InProgress) mission, validates the asset, and hands off to the executor's Commission phase (does NOT release/fly — the drone stays put until commissioning passes). |
| `BoreController.cs` | Direct flight control (autopilot OFF): damped gyro P-D loop (1.0 gain, 1.5 damping, ±1 rpm clamp) for orientation hold; `Drive` (align + velocity-limited advance), `ThrustAlong` (thrust along a world dir without reorienting), `Release`. |
| `MinerController.cs` | Executes ONE drone's mission (instance per mission): Commission (power self-test, refuse < 10 min runtime) → Transit (RC autopilot to standoff) → StartBore (alignment check) → Mining (+-pattern bore: Center/N/S/E/W to ore-depth+1 m, drills on, exit on pattern-complete or cargo ≥ 80 %) → Retreat (climb to standoff, complete). Flip guard, stuck/timeout watchdogs. |
| `DroneExecutor.cs` | Coordinator: owns one `MinerController` per in-progress Mine mission, routes ticks, prunes finished controllers. Exposes `AbortAll` (hard stop), `RecallAll` (graceful retreat), `ReleaseControls` (clear stale overrides on register). |
| `ColonyCommands.cs` | Chat command handler for all `/colony` commands: `info`, `scan`, `missions`, `register`, `assets`, `resources`, `dispatch`, `abort`, `recall`. |

### Host layer

| File | Purpose |
|------|---------|
| `CommandCoreSession.cs` | `MySessionComponentBase`: composition root, lifecycle (`LoadData`/`SaveData`/`UnloadData`), tick scheduler (colony activity, resources, scan refresh, ore scan, mission generation, assignment, executor), and legacy `colony_state.xml` migration. |
| `CoreBlockLogic.cs` | `MyGameLogicComponent` for the Colony Core block; calls `CommandCoreSession.Instance.RegisterCore` / `UnregisterCore` on init/close so the session tracks which cores are functional. |

## Persistence

- **Save file:** `colony_world.xml` in SE world storage, serialized via `MyAPIGateway.Utilities.SerializeToXML<WorldState>`.
- **One-time migration:** On load, if `colony_world.xml` is absent but the legacy `colony_state.xml` exists, the old `ColonyState` is read, stamped with `OwnerKey = 0`, wrapped in a new `WorldState`, and used. The new file is written on the next `SaveData`. The legacy file is left in place (SE has no delete API).
- **Backwards compatibility:** `ColonyState` tags 1–5 predate the 1a refactor; tags 6–8 (`OwnerKey`, `CoreEntityId`, `Active`) were added append-only. Old saves deserialize to `OwnerKey = 0` and `Active = false` (bool default), so `Active` is explicitly initialized to `true` in the migration path and on `GetOrCreate`.
