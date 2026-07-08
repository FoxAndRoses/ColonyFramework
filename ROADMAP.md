# ROADMAP.md — Handoff & Remaining Work (stepped chunks)

**For any Claude session (or human) picking this up cold. Read in this order:**
1. `CLAUDE.md` — standing rules (layering, testing discipline, API lessons). Non-negotiable.
2. `FLIGHT.md` — the flight core contract. COMPLETE and closed (chunks R…F5). Do not patch flight
   behavior ad hoc; changes go through the contract.
3. `MISSION.md` — the mission-intelligence contract: template + stories + decision tables.
   **Edit the contract BEFORE code when a design needs to change.**
4. This file — what's done, what's next, and the exact steps per chunk.

## State as of commit `1069d63` (2026-07-08)
- Flight program COMPLETE: ShipSelfModel + launch self-test, FlightController velocity loop
  (one actuator owner), context steering + work volumes + camera probe, corridor planning, ALL
  consumers converted (welder, miner incl. 115 m eject excursion, survey, parker), Dock verb,
  legacy (AvoidanceProbe, autopilot cruise helpers) deleted.
- Mission program: MISSION.md written incl. round-2 contracts (blueprint capture via
  `SetProjectedGrid` — confirmed present in Sandbox.Game.dll; repair/decommission; fleet move;
  ship classes; combat placeholder). **M1 shipped**: energy ledger (distance-aware, named aborts),
  Scout type + survey eligibility, ore-under-base exclusion (80 m), warehouse backpressure (90%),
  player-control pause.
- Known open non-chunk item: LCD dashboard renders nothing (no exceptions — needs the diagnostic
  below).
- **NOTHING SINCE COMMIT `9e2ee58` HAS BEEN TESTED IN-GAME.** The entire flight core, M1, and all
  converts are build-green only. CLAUDE.md rule 1: a green build is not "done".

## STEP 0 — the smoke test (DO THIS BEFORE ANY NEW CHUNK)
1. `dotnet build` → exit to main menu → reload world → confirm `[ColonyFramework] Loaded.`
2. `/colony flighttest 300` near an idle registered drone → smooth corridor out-and-back,
   `flighttest … COMPLETE` in log, no jitter, ZERO `deflected` spam.
3. One full miner loop: dispatch → hop self-test (`self-test PASS — lift …`) → bore →
   eject excursion (`eject excursion — flying 115 m off the shaft` … `re-entering to Xm`, X
   increasing per cycle) → return → dock/unload.
4. One welder job on a small projection: `weld: tool on, creeping in` → `weld: block complete (N
   remaining)` counting down; no `deflected` lines.
5. Fix what breaks BEFORE building more. Flight bugs are fixed in FlightController /
   FlightSteering / FlightCorridor (one owner each), never by re-adding logic to mission
   controllers. Pull exact log lines first:
   `Get-ChildItem "$env:APPDATA\SpaceEngineers" -Filter "SpaceEngineers_*.log" | Select-String "ColonyFramework"`

## Remaining chunks — stepped
Rules for every chunk: read its MISSION.md section first; one commit per chunk; append a Testing
block (CLAUDE.md format); add log lines for anything otherwise invisible; contract edits land in
the same commit as the code they authorize; push with your own GitHub auth.

### LCD-fix (tiny, standalone — do anytime)
1. In `LcdService.WriteToTaggedSurfaces`, count matched surfaces; log `lcd: N tagged surface(s)`
   (throttle ~1/min).
2. In-game: N=0 → finder bug (name filter, subgrid membership, ownership); N>0 but blank →
   try direct `IMyTextPanel` cast + `ContentType.TEXT_AND_IMAGE` + `Script = ""`.
3. Acceptance: `[Colony]`-named LCD renders the dashboard and updates ~5 s.

### M1.5 — FuelModel (hydrogen/hybrid) [contract: MISSION.md Story M-C + fleet notes]
1. `ShipSelfModel`: census `IMyGasTank` (H2), gas generators, H2 thrusters → `FuelKind
   { Battery, Hydrogen, Hybrid }` + tank fill/capacity fields.
2. `LaunchSelfTest`: record tank-level delta across the hop → measured burn rate (same pattern as
   the power spike).
3. `MissionLedger.ShouldReturn`: Hydrogen/Hybrid path — return cost in liters vs stored (same
   distance/time formula; battery path unchanged).
4. **Ice-is-fuel bug (real, shipped today):** in `DroneUtil.IsJunkItem`/`EjectJunk`, ice is NOT
   junk when the drone has a gas generator and FuelKind != Battery, until aboard-ice ≥ a
   `FuelIceReserveKg`; eject only the excess.
5. Emergency refuel: ledger trips + generator + ice aboard → land in place (verbs exist), log
   refuel progress while tanks fill, resume mission. Miner first; welder optional.
6. Acceptance: H2 miner keeps reserve ice through ejects; drained tanks mid-mission → lands,
   refuels, resumes — all named in the log.

### M2 — Miner arrival intelligence [contract: Stories M-A, M-B, M-D; tables D1/D2/D3/D5]
Largest chunk; sub-steps are independently commit-able:
1. **D2 helper first (shared with M3):** `ShipSelfModel.HoverThrustAt(Vector3D noseDir)` — rotate
   the body axes so Forward = noseDir; gravity-opposing thrust = Σ axis thrust × max(0,
   axisDir·gravityUp); OK iff ≥ 1.15·m·g. Log the verdict during the self-test.
2. **Depth budget (M-B):** at bore start `D_max = min(oreDepthAlongAxis + 2, D_climb, 60)` with
   `D_climb` by full-cargo TWR band (<1.25→15 m, <1.5→35 m, else 60 m); re-band when mass grows
   ~10%; if current depth > shrunken budget → trigger the existing eject excursion, resume only to
   the budget. Log the budget and which term bound it.
3. **Dump vector (M-D):** eject fly-out picks argmax over 8 compass directions of
   `terrainDrop(115 m, 2 heightmap samples) − basePenalty(within 60° of base bearing) −
   activeShaftPenalty(within 100 m of a claimed deposit)`; tie → current away-from-deposit.
4. **Bore axis selection (M-A)** — keep vertical as the default, gate the rest:
   a. Candidates from the ore centroid: up, 4 horizontal compass dirs, steepest-descent direction.
   b. Rock path per axis: walk `GetClosestSurfacePointGlobal` outward in 4 m steps, cap 120 m.
   c. Feasible = `HoverThrustAt(−axis)` OK ∧ thrust along +axis at full mass ≥ 1.3 × gravity
      component along the axis.
   d. Score = rockPath × (vertical ? 0.8 : 1.0); min wins; none feasible → vertical (invariant I5).
   e. Generalize bookkeeping: mission `downDir` = chosen axis; `_u,_v` ⊥ axis; descend/penetration
      gates use along-axis distance from `_boreContact` (NOT altitude) when non-vertical; entry
      point = surface + axis·(hullLen/2 + margin); approach entry via OrientAndCreep along −axis.
   f. Log: `bore axis: horizontal-N (rock 12 m vs vertical 90 m), entry at (…)`.
5. Acceptance: mountainside deposit bores from the side; deep ore stops at the budget with the
   binding reason logged; junk lands downhill; flat ground still chooses vertical.

### M3 — Welder reach solver [contract: Story W-A/W-B; table D6]
1. Candidates: 6 face normals of the target cell + 4 blends of the two most open faces (≤10).
2. Gates per candidate, cheapest first: REACH (existing CastRay), FIT (3 rays at hull half-width
   along the corridor), ATTITUDE (`HoverThrustAt` from M2 step 1 — build that first if doing M3
   before M2).
3. Order candidates with a claims-bubble penalty (WeldCoordinator query exists).
4. First pass wins → existing approach/creep verbs at that angle. None → defer with per-gate
   counts: `no fitting corridor: 10 candidates — 6 blocked, 2 unfit, 2 tip-risk`.
5. **W-B fix (required):** `FlightSteering.RefreshObstacles` work-volume exemption must NEVER
   apply to registered fleet drones — pass asset ids through FlightController; drone grids stay
   obstacles inside work volumes.
6. Acceptance: an interior block on a partially framed hull gets a visibly angled (pitched)
   approach or the provable defer line; two welders never converge on one spot.

### M4 — /colony brief (stretch; anytime after M2)
Read-only introspection: per active drone print mission/phase, ledger numbers, chosen bore
axis/depth budget/dump vector (miner), current candidate verdicts (welder). Expose small getter
structs through DroneExecutor; no behavior changes.

### M5 — Shipyard + blueprint capture [contract: fleet story F-B; API confirmed]
1. `[verify]` SPIKE FIRST: `grid.GetObjectBuilder()` on a registered drone → sanitize the OB
   (clear inventories, battery charge → default, ownership → colony founder) →
   `IMyProjector.SetProjectedGrid(ob)` on a same-grid-size projector → confirm the projection
   appears and is weldable. A throwaway `/colony capturetest` command is fine; remove after.
2. Persist captures per asset type in world storage (separate files, not colony_world.xml).
3. Recapture on every successful commissioning self-test (the fleet's blueprints track live ships).
4. FleetPlanner (ProductionService cadence): pending missions per type > idle drones of that type,
   sustained 5 min → find idle `[Shipyard]`-named projector of matching grid size →
   SetProjectedGrid → existing weld pipeline builds it → auto-register on completion (scope by
   shipyard name + proximity) → normal self-test commissions it.
5. Ship classes ride along: S/M/L by AABB max dimension (<8 m, <20 m, ≥20 m); pad tags
   `[Pad S|M|L]`; ConnectorReservations matches class ≤ pad class.
6. Acceptance: sustained backlog prints and registers a new miner with no player action beyond
   having built/named the shipyard.

### M6 — Threat response + fleet move [contract: fleet sections]
1. `MyAPIGateway.Session.DamageSystem` handler → drone under fire → named abort ("under attack,
   fleeing") → corridor home at ledger-max speed.
2. No-go volumes: attack site becomes an avoid-sphere in FlightSteering (inverse work volume,
   ~10 min TTL); demand mining skips deposits inside it.
3. `/colony move <here|gps> [all]`: Move mission type; shared corridor + per-drone offset slots.
4. Acceptance: shooting a miner makes it flee and others route around the zone; /colony move
   relocates idle drones in formation.

### M7 — Repair & decommission [contract: Story F-A]
1. Integrity diff (live blocks vs capture) → "damaged" flag on the asset; damaged drones fail
   dispatch until repaired.
2. Repairable (onboard projector alive + capture exists): Repair mission — dock at a repair pad,
   SetProjectedGrid own capture on the ONBOARD projector, welder drone (or player) restores it.
   Verify projection alignment on the pad before enabling welders.
3. Unrepairable → DECOMMISSION: fly (or mark in place if unflyable) to a graveyard ≥150 m from
   base, power down, unregister, GPS `decommissioned — disassembly`; FleetPlanner (M5) queues the
   replacement.
4. Acceptance: grinding a thruster off a drone produces the full named path end-to-end.

### M8+ — Combat drones
CONTRACT FIRST: expand the MISSION.md placeholder into a full template instance (identity, arc,
rules-of-engagement stories, ammo-as-fuel ledger, disengage criteria, escort formations, the
terminal ram story) and get it reviewed. No code before the contract.

## Decision log (preserved — why things are the way they are)
- Full mod over PB scripts: PB sandbox cannot read voxels/ore or share across grids. Mining stays
  physical (voxel-carving/Nanite-style rejected: perf trap + violates thesis).
- RC autopilot retired from precision flight (F1): it is gravity-level-only with an imprecise
  arrival radius — matches PAM/SAM prior art. It survives only inside the miner's legacy dock and
  its recovery hop.
- Drills/welders face RC-forward: one alignment convention shared by all controllers; enforced at
  commissioning/bore start with named failure.
- Blueprint loading: player-side blueprint files are NOT loadable by mods; `SetProjectedGrid` with
  a mod-captured ObjectBuilder is the sanctioned path (M5).
- Cheat policy (user-set): "cheats to force actions" are acceptable when they consume equivalent
  resources (e.g. prefab-spawn a small-grid seed while debiting components) — never create from
  nothing.
