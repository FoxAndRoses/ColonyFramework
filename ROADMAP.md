# ColonyFramework — Roadmap & State

## Validated and working (in-game, log-evidenced)
- Toolchain: MDK², dotnet build → pack → deploy.
- Persistence: WorldState root (colony_world.xml), legacy migration, append-only schema.
- Multi-colony: ColonyRegistry keyed by owner (faction id, else identity id);
  Colony Core block (donor: Programmable Block, SubtypeId "ColonyCore") activates a
  colony while functional; oldest core wins; colony deactivates on core loss.
- Colony identity: ColonyId, auto-name, founder, creation date (/colony info).
- Discovery: bounded LOD2 voxel scans; manual (/colony scan, 25m) and automatic
  (round-robin powered ore detectors, 150m, movement-gated). Dedup 50m merge radius.
- Missions: one Mine mission per unclaimed deposit; claim/release/deplete state machine.
- Assets: manual /colony register; validation (offline detection, mission fail on loss);
  nearest-idle assignment.
- Resource tracking: physical grid group + owner filter, ore/ingot/component totals,
  30s cadence + on-demand (/colony resources).
- Execution PoCs 1–3: dispatch (RC autopilot to gravity-safe standoff 120m above
  deposit), arrival detection (phase machine on Mission.Phase), bore (damped 6 Hz
  gyro P-D align + bang-bang thrust descent 1.5 m/s, cargo 80% / target / timeout 240s /
  no-progress 20s watchdogs), retreat climb, mission Complete → asset idle.
  /colony abort (hard) and /colony recall (graceful).

## Next: PoC 4 — close the full loop
1. Return home: after retreat, fly to a designated home point (the colony core grid's
   position initially), via autopilot transit.
2. Staged docking: navigate to staging point near a designated home connector →
   autopilot off → precision align (connector world matrix) → connect.
3. Unload: push drone cargo into base storage via connected conveyor (or inventory
   transfer), then undock to a hold point.
4. Auto-dispatch: executor dispatches Assigned missions automatically (retire manual
   /colony dispatch to a debug tool). This completes discover → generate → assign →
   execute → return → unload → repeat.

## Then: Tier 0 completion (Tier 0 = full v1 scope; later tiers gate NEW features only)
- Power tracking: battery charge % only (no consumption — no clean API).
- Colony Advisor: threshold-derived observations only; Event Log capped at 20 entries.
- LCD displays via native text-surface scripts (Resource / Power / Status panels);
  advisory text is content for these panels.
- Construction advisor: projector on the core's grid group only; blueprint component
  requirements vs ResourceSnapshot.
- GPS deposit markers on faction members' HUDs + on/off toggle on the core block.
- Custom "Colony" G-menu tab (GuiBlockCategoryDefinitions) once ≥2 colony blocks exist.
- Deferred from Tier 0: module detection (no modules exist yet), milestone tracking.

## Known debt / accepted simplifications (revisit deliberately)
- Bore is straight-down only; no spiral/stepped patterns; transit is straight-line
  with no terrain avoidance (can clip hills between drone and standoff).
- Mission Complete marks the deposit Depleted after ONE bore — wrong; needs deposit
  quantity estimates and partial-depletion later.
- Deposit DB grows unbounded (300+ in test world); needs Depleted pruning + mission
  generation throttling (one colony generated 50 missions in one pass).
- Mission/deposit lists are linear scans — fine now, index by id if counts grow.
- "Oldest core wins" is invisible to players; needs terminal info on the core block.
- Phase field semantics changed once between builds; persisted missions can resume
  with stale phases after mod updates — consider a phase-reset on schema bump.
- Resource scan diagnostics still log every cycle — quiet them once stable.
- Drone has no home position concept yet; standoff recomputed from gravity each check.
- No network sync layer for client-side richness yet (vanilla replication covers
  LCD/GPS plans); revisit when interactive client UI is wanted.

## Decision log (why things are the way they are)
- Full mod over PB scripts: PB sandbox cannot read voxels/ore data or share across
  grids; mod API can. Mining stays physical regardless (Nanite-style voxel-carving
  rejected: perf trap + violates project thesis).
- PAM integration rejected (built around recorded paths, no external coordinate
  injection). AI Flight/Basic rejected for transit (no scriptable dynamic waypoints).
  RC autopilot chosen for transit; direct gyro/thrust control for the bore phase.
- Drills must face RC-forward: lets autopilot/controller share one alignment convention;
  checked at bore start (dot > 0.7) with mission fail + reason on violation.
- Tier system: mechanism deferred until a tier-1 feature exists; everything current is
  tier 0 by definition.
