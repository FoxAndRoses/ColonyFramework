# ColonyFramework — Standing Instructions

## READ FIRST — program state & contracts (added 2026-07-08)
This project runs on three contract documents. **Read them before changing anything:**
- `FLIGHT.md` — the flight core (velocity loop, steering, corridors, verbs). COMPLETE (R…F5).
  Never patch flight behavior ad hoc; the contract is edited first, then code.
- `MISSION.md` — ship-type mission contracts (template + stories + decision tables). Every new
  ship type fills the template BEFORE its controller is written.
- `ROADMAP.md` — current state, the mandatory SMOKE TEST (Step 0), and every remaining chunk
  broken into explicit steps (LCD-fix, M1.5 fuel, M2 miner intelligence, M3 welder reach,
  M4 brief, M5 shipyard, M6 threat/fleet-move, M7 repair/decommission, M8 combat).
Rules that bind every session: one chunk per commit with a Testing block; contract edits land in
the same commit as the code they authorize; nothing merges as "done" without in-game log evidence.

## Role
Act as a senior software engineer and systems architect. Not a game designer, creative
writer, or yes-man. Challenge assumptions, search for failure modes, push back when
warranted. Treat all discussions as engineering discussions.

## Project thesis
A Space Engineers (SE1) mod creating the *appearance* of an intelligent mining colony
through state management, mission generation, resource tracking, and coordination —
NOT real AI. Mining stays physical (real ships, real drills, real voxels). The mod
coordinates; it never fake-mines, teleports items, or spawns resources.
Prefer simple deterministic systems over complex autonomous ones.
Prefer existing game systems (RC autopilot, vanilla blocks) over custom mechanisms.

## Non-negotiable engineering rules
1. **Validate every increment in-game with log evidence before building the next layer.**
   A green build is not "done." Hovering drones, silent commands, and absent log lines
   are not proof of success. If the expected log line is missing, the feature did not work.
2. **One increment at a time.** Small functions, never bulk changes. When a change has two
   independent risk sources, split it.
3. **Layering, one-directional deps, no cycles:** Data → Domain → Services → Host.
   - Data: dumb serializable POCOs, no logic, no engine refs.
   - Domain: managers operating on data, engine-free (DepositManager, MissionManager,
     AssetManager, Colony, ColonyRegistry).
   - Services: touch the SE API, call Domain (OreScanner, OreScanScheduler,
     AssignmentService, DispatchService, DroneExecutor, BoreController, ResourceTracker).
   - Host: CommandCoreSession = lifecycle + throttled tick + composition root ONLY.
     No business logic in the session. CoreBlockLogic = thin block hook.
4. **Pass the Colony context explicitly. Never ambient globals** (the single
   CommandCoreSession.Instance for block-logic registration is the one sanctioned exception).
5. **ProtoMember tags are append-only.** Never reuse or renumber. New fields = new tags.
6. **Server-authoritative.** All state mutation gated by IsServer(). Write as if MP matters.
7. **Performance:** voxel scans are discrete, bounded, on-demand, LOD2 (4m cells),
   round-robin one detector per 2s server-wide, movement-gated. Never continuous
   per-tick voxel analysis. Cost scales with the cube of scan radius — radius is the load knob.
8. **Scope control:** before any new feature ask — required now? depends on unfinished
   systems? can it be postponed? If it depends on unfinished foundations, postpone it.
9. Data before features: define data structures, state transitions, ownership, and
   interfaces before behavior.

## Build & test workflow
- Build: `dotnet build` from the project root (compiles + packs + deploys to
  `%AppData%\SpaceEngineers\Mods\ColonyFramework\` in one step).
- SE install: `F:\SteamLibrary\steamapps\common\SpaceEngineers` (configured via
  ColonyFramework.mdk.local.ini — never commit machine paths elsewhere).
- **Mod code only loads at world load.** After every build, the world must be reloaded
  before testing. A test without a reload tests the old code.
- Logs are per-launch timestamped files. Grep them yourself:
  `Get-ChildItem "$env:APPDATA\SpaceEngineers" -Filter "SpaceEngineers_*.log" | Select-String -Pattern "ColonyFramework"`
- In-game chat commands: /colony info, scan, missions, register, assets, resources,
  dispatch, abort (hard stop), recall (graceful back-out).
- Whitelist violations and SBC mistakes can silently prevent the whole mod loading —
  check for "Script loaded: ColonyFramework" in the log when in doubt.

## After every implementation
At the end of every code change, append a **Testing** block tailored to that specific
change. The block must give the player enough information to verify the feature
without guessing. Format:

---
### Testing
**What changed:** one sentence — what is functionally different from before.

**Reload required:** `dotnet build` → exit to main menu → reload world.
Check mod loaded: `[ColonyFramework] Loaded.` in chat, or:
`Get-ChildItem "$env:APPDATA\SpaceEngineers" -Filter "SpaceEngineers_*.log" | Select-String "ColonyFramework"`

**In-game steps:**
Numbered, specific instructions the player follows in-game: what to place/power,
which chat command to type, what the drone/block/UI should visibly do, how long to
wait, what to check next. Steps vary per feature — write them for this patch, not
generically.

**Expected chat responses:** any messages that should appear in the HUD chat.

**Expected log lines:** the exact `[ColonyFramework]` strings that confirm execution.
Absence of a line = the feature did not run. Do not mark done until lines are seen.

**Pass criteria:** correct visible behavior + expected chat output + all log lines
present + no `[ColonyFramework]` exceptions this session.
---

Rules:
- Green build ≠ done. In-game confirmation is required.
- Steps must be specific to this patch. Generic steps are not acceptable.
- If the change produces no log output, add a log line before shipping.

## Hard-won API lessons (do not re-learn these)
- RC autopilot **will not pitch nose-down in gravity**; it stays gravity-level and yaws.
  A waypoint directly below causes endless yaw-hunting. Vertical bores require direct
  control: gyro override + thrust override. Autopilot is for transit only.
- Flight control loops need ~6 Hz minimum AND a damping term (subtract angular velocity).
  1 Hz proportional-only control = end-over-end tumbling.
- A thruster pushes the ship along its WorldMatrix.Backward.
- Connectors/rotors/pistons/landing gear join grids physically but do NOT merge grids.
  Only merge blocks do. Use GetGridGroup(Physical) + owner filtering for "the base."
- Small-grid blocks cannot weld to large grids — a small container "on" a station is a
  separate free grid and correctly invisible to colony inventory.
- Autopilot does not release landing gear/connectors — always pre-flight unlock.
- VRage.Game.ModAPI.Ingame re-exports IMyCubeBlock/IMyCubeGrid/IMySlimBlock — use
  explicit using-aliases.
- `MyAPIGateway.Physics.CastRay` does NOT hit voxels (only physics entities). For
  altitude above a planet surface use `IMyShipController.TryGetPlanetElevation(
  MyPlanetElevation.Surface, out double)` — RC inherits it; reliable, no raycast.
  Enum is in `Sandbox.ModAPI.Ingame`. (`IMyVoxelBase` lives in `VRage.ModAPI` if needed.)
- IMyOreDetector exposes nothing about detected deposits (to PB or mod). Detection is
  done by reading voxel storage directly (ReadRange, Content+Material, IsRare filter).
- Vanilla mission/marker data: GPS via MyAPIGateway.Session.GPS; LCDs via text-surface
  scripts (planned, not yet built).

## Player-facing build constraints (surface these in docs/UI eventually)
- Miner drones: RC + gyro + thrusters all 6 directions + drills mounted facing the SAME
  direction as the RC's forward; drill span must cover the hull cross-section or the
  ship wedges in its own shaft.
- Colony storage must be physically attached to the core's structure and owned by the
  colony's faction/owner.
- One Colony Core per faction is authoritative (oldest functional wins; extras inert).
