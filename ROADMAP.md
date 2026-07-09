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
6. **W-D auto-complete fallback** [contract: Story W-D]: per-cell strike tracking (2 full defer
   strikes or 3 weld timeouts) → deduct remaining components from stock →
   `MoveItemsToConstructionStockpile` + `IncreaseMountLevel` `[verify]` → Notify with reason and
   deducted count. Brakes: per-construction cap (default 20%, `/colony autocomplete <pct>`, 0
   disables), stock-availability gate, over-cap → named stall ("layout largely unreachable").
7. **W-E no-hoarding** [contract: Story W-E]: welder Complete/Fail/Recall unloads ALL components
   at dock; extend H1's parked-docked drain to ALL cargo types (minus M1.5 fuel-ice reserve).
8. Acceptance: an interior block on a partially framed hull gets a visibly angled (pitched)
   approach or the provable defer line; a truly unreachable block auto-completes with a named
   deduction and the construction FINISHES; two welders never converge; an idle parked welder
   carries zero components.

### M4 — /colony brief (stretch; anytime after M2)
Read-only introspection: per active drone print mission/phase, ledger numbers, chosen bore
axis/depth budget/dump vector (miner), current candidate verdicts (welder). Expose small getter
structs through DroneExecutor; no behavior changes.

### M5 — Blueprint capture + ExpansionPlanner [contract: GROWTH story F-B — user-unified on
FRAME-SPAWN for ALL captures (drones AND buildings); SetProjectedGrid demoted to fallback]
1. `[verify]` SPIKE FIRST (shared with B1): `grid.GetObjectBuilder()` on a registered drone →
   sanitize (inventories, battery charge, ownership) → rebuild the OB with all blocks at frame
   BuildPercent + empty stockpiles → spawn → confirm the scaffold appears and welds to completion.
   Verify the SetProjectedGrid fallback once too. Throwaway `/colony capturetest`; remove after.
2. Persist captures per type in world storage (separate files, not colony_world.xml); recapture on
   every successful commissioning self-test (blueprints track their live ships).
3. Weld-target generalization (shared with B1 and M7 repair): weld missions target "incomplete
   blocks on a designated construction grid" — integrity check replaces CanBuild; stager, slots,
   reach solver unchanged.
4. ExpansionPlanner (ProductionService cadence) per the GROWTH contract: autonomy ladder
   OFF / PROPOSE (default) / FULL set at the core terminal (persisted); "possible" gates (stock
   keep-back 30%, idle welder, surveyed flat site, power margin >20%); "appropriate" signal table
   (mission backlog → +drone; cargo >85% → +storage; refinery queue → +refinery; power margin →
   +power; repair queue → +pad, via B1); ONE build queued at a time; priority power > storage >
   refinery > drones > pads; hard per-type caps (`/colony cap`); siege freeze; every autonomous
   build Notified with its reason ("expanding: +1 miner — 14 pending vs 2 idle").
5. New-drone flow: frame-spawn on a clear pad site near base (mini B-A survey) → welders complete
   → commissioning self-test → auto-register.
6. Ship classes ride along: S/M/L by AABB max dimension (<8 m, <20 m, ≥20 m); pad tags
   `[Pad S|M|L]`; ConnectorReservations matches class ≤ pad class.
7. Acceptance: sustained mining backlog frame-spawns, welds, self-tests, and registers a new miner
   — after `/colony approve` in PROPOSE, automatically with the reason Notified in FULL.

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

### NET-1 — Multiplayer correctness (REQUIRED — user directive: "completely MP friendly")
Known breaks traced in current code (no test needed to confirm the first two):
- `CommandCoreSession.LoadData` registers `MessageEntered` only in the SERVER branch → joined
  clients have no chat handler; commands are dead for everyone but the host.
- Every `ShowMessage` is local-process-only and wrapped in `!IsDedicated` → on a dedicated server
  NOBODY receives any colony feedback, ever.
Steps:
1. Protobuf command packet (`[ProtoContract]`, append-only): { senderIdentityId, command string }.
   Client session (non-server branch) registers MessageEntered → intercepts `/colony …` → sends via
   `MyAPIGateway.Multiplayer.SendMessageToServer` (one registered ushort channel id).
2. Server handler: resolve sender identity → OWNERSHIP CHECK (sender's faction/identity must match
   the colony OwnerKey — same rule as registration) → route into the existing `ColonyCommands.Handle`.
3. Notify service replacing raw ShowMessage everywhere: `Notify(colony, msg)` → server sends a
   packet to the owning faction's online players → tiny client handler shows chat/HUD text. Delete
   the `!IsDedicated` guards (they exist only because ShowMessage was local).
4. Verify-and-fix pass: GPS (`AddGps(identityId, …)` must target each faction player — verify it
   syncs), LCD `WriteText` from server (verify client render), self-test/ledger chat lines all
   through Notify.
5. Acceptance: on a dedicated server, a JOINED (non-host) faction member runs /colony status,
   dispatch, flighttest — everything works and all feedback arrives; a non-faction player's
   commands are rejected by name.

### UI-1 — Terminal controls + hotbar actions (small; after NET-1)
1. `MyAPIGateway.TerminalControls` on the Colony Core block: buttons Dispatch / Recall / Abort /
   Scan, a status multiline label, blueprint/build info. All actions route through the SAME
   server command path as chat (NET-1) — the UI is just another ingress.
2. Block ACTIONS (toolbar-bindable): Dispatch, Recall, Status-to-chat — players put colony
   control on a cockpit hotbar.
3. Acceptance: full colony control from the core's terminal + hotbar with zero chat typing,
   as host AND as joined client.

### UI-2 — RichHud radial menu (user-approved Tier 3; after UI-1 and the POSTURE contract)
UX rules (user directive: thoroughly friendly, sensible nesting):
- Max 8 slots per ring, max 2 levels deep. Hold-key opens, drag+release selects (one motion);
  releasing on a submenu opens it, releasing on centre cancels. Most-used entries sit at the
  cardinal positions of the root.
- Rank filtering HIDES entries (a Member never sees Founder items — server capability list, R1).
- Destructive entries (Abort, Decommission) get a confirm ring (drag through twice), nothing else.
- CONTEXT SENSITIVITY: if the player is AIMING at a registered drone (camera ray), the root gains
  a "This ship ▸" branch scoped to it; otherwise colony-wide entries only.
Menu tree (v1) — SCOPE LAW (user-corrected): the radial commands the player's OWN WING and makes
REQUESTS of the core; the core's strategic state (colony posture, reserve %, ranks, rename) is set
ONLY at the core block terminal (UI-1) — physical presence required.
- Root: Status (instant HUD readout) · **Command ▸** (my wing) · Request ▸ (asks the core) ·
  [This ship ▸] (aimed, own-wing ship)
- **Command ▸** (MISSION.md WING COMMAND — only visible while a wing is assigned): Follow me ·
  Aggressive · Defensive · Stand down · Dismiss/R&R⚠
- Request ▸: Reinforce me (R2) · Scan here · Build status · Move fleet here (Leader+, M6 — the
  core decides from surplus)
- This ship ▸ (aimed own-wing ship): Brief (M4) · Send home · Flight test
UI-1 addendum: the CORE TERMINAL gains the strategic panel — colony posture (Defensive/Aggressive/
Stand down/R&R), reserve %, ranks, rename — rank-gated Leader+/Founder, never available remotely.
Steps:
1. Soft dependency handshake with RichHudFramework; absent → log once, fall back to UI-1.
2. Implement the tree above; every leaf routes through the NET-1 command path (the radial is just
   another ingress); posture leaves require the POSTURE contract's server-side state (small — the
   state field + the per-controller consults land with this chunk if M8 hasn't yet).
3. HUD status readout page (LCD dashboard content, toggleable).
4. Acceptance: full tree usable one-handed as host AND joined client; Member sees a smaller tree;
   aiming at a drone adds the scoped branch; Abort requires the confirm ring; mod loads cleanly
   without RichHud installed.

### V1 — Asset value ledger + protective reserve [contract: MISSION.md VALUE LEDGER]
(after M5's ship classes; before R2)
1. Material-worth table (one static dict: ingot type → weight; Pt/U/Au heavy).
2. ValueScore at commissioning: tonnage term + Σ(block components → ingots → weights); cache on
   AssetRecord (new append-only ProtoMember).
3. Reserve doctrine: `/colony reserve <pct>` (persisted); reserve = highest-value idle assets to
   the pct; `Surplus()` query on the asset manager used by fleet-move/reinforcements/shipyard.
4. Acceptance: /colony assets shows value + reserve flags; a fleet-move of "all" leaves the
   reserve home, by name.

### R1 — Ranks & rank-gated commands [contract: MISSION.md RANKS; after NET-1]
1. Tier resolution: faction Founder/Leader/Member → colony tiers, with `/colony rank set` override
   persisted in colony state (append-only).
2. NET-1's server command router gains per-command tier requirements (one table); rejections named.
3. Server sends each player their capability list on join/rank change → UI-1 buttons and the UI-2
   radial render only permitted commands.
4. Acceptance: a Member sees/can use only their tier's commands on a dedicated server.

### R2 — Reinforcements [contract: Story R-A/R-B; after M6 formations, V1, R1; combat case after M8]
1. `/colony reinforce` (radial/chat): rank tier → budget (Member ≤1×S, Leader ≤3×M, Founder =
   surplus) → nearest class-appropriate surplus assets; per-candidate D2/TWR gate against the
   caller's gravity well BEFORE dispatch.
2. Reinforce mission: DYNAMIC target = caller identity; corridor re-plans re-read live position;
   caller offline/dead → last position, then RTB.
3. Arrival: M6 formation slots around the caller; CONTEXT ADOPTION — combat context (recent damage
   events/hostiles near caller) → M8 doctrine (until M8 exists: hold formation + flee rules);
   calm → follow in formation until `/colony release` or the ledger's RTB trip (D4 measured from
   CURRENT position home, continuously — escorts turn back by name before stranding).
4. Siege lock: active threat zone at base → reserve doctrine pins to 100%, reinforcements refuse
   with a named reason.
5. Acceptance: idle-in-space Leader calls → wing arrives in formation and follows; base under
   attack → the same call is refused by name; escorts RTB on ledger before stranding.

### B1 — Construction sites (prefab buildings) [contract: MISSION.md Story B-A; after M5's capture spike]
1. Prefab library: `/colony capture building <name>` on a player-built structure → sanitized OB in
   world storage (reuses M5's capture code verbatim).
2. `/colony build <name> here`: site survey (footprint flatness via heightmap variance +
   grid-clearance sphere) → named accept/reject.
3. FRAME-SPAWN `[verify first]`: build the grid OB from the capture with every block at frame
   BuildPercent + empty stockpiles; spawn gravity-aligned; DEBIT first-component cost of all
   blocks from stock. (Fallback: 3-block powered anchor + SetProjectedGrid.) No projector needed.
4. Weld pipeline targets "incomplete blocks on the construction grid" (integrity check replaces
   CanBuild in selection; everything else — multi-welder, stager, reach, resupply — unchanged).
4b. Colony-initiated placement: ring-sampled site scoring (flatness, 30-150 m band, clearance, no
   ore under footprint) → `proposed: <name>` GPS + `/colony approve` gate.
4c. BASE TOPOLOGY (contract: MISSION.md attach-vs-district; user-corrected): estate registry
   (transitive proximity graph of owned structures, feeds exclusion/defense/placement).
   ATTACH = SAME-GRID extension: `IMyCubeGrid.AddBlock` `[verify in the B1 spike]` stakes the
   capture's frames directly onto the base grid (coordinate transform + all-or-nothing CubeExists
   pre-validation + N blocks/tick throttle) — conveyors/power connect natively, NO connectors.
   DISTRICTS stay separate frame-spawned grids, self-powered, drone-supplied (no lock dependency
   anywhere; connectors remain drone-docking only). Function table unchanged (logistics attach;
   solar/yard/defense-ring detach); proposals name their placement reasoning.
5. Site zone: no-go volume for non-assigned fleet + `[verify]` mod-created native Safe Zone for
   players/foreign grids (fallback: Notify warning + graceful stall on intrusion).
6. Progress: self-renaming GPS marker (`<name> — NN%`), LCD line; completion dissolves the zone
   and registers the building (mining exclusion from anchor-spawn moment; power/cargo rollups pick
   it up per attachment/proximity rules).
7. Acceptance: one command raises a real building with real welders and visible progress; a player
   walking into the zone is kept out (or warned, per fallback); stock shortage stalls with named
   missing components and resumes.

### M8+ — Combat drones
CONTRACT FIRST: expand the MISSION.md placeholder into a full template instance (identity, arc,
rules-of-engagement stories, ammo-as-fuel ledger, disengage criteria, escort formations, the
terminal ram story) and get it reviewed. No code before the contract.

## Daydream round 3 — common-sense gaps (scenarios played against commit 39fceab)
Each was walked through the actual code; "confirmed" = evidence already exists in code or logs.

**H1 — hygiene chunk (small fixes, one commit, do after Step 0):**
1. **"The stranger's ship"** (CONFIRMED — no ownership check in /colony register): you can register
   any grid within 100 m, including another player's parked ship — the colony then flies it away.
   Fix: majority BigOwners must resolve to the colony's OwnerKey; named rejection otherwise.
2. **"The full drone that never unloads"** (CONFIRMED by code path): DockWaiting's 10-min ceiling
   completes the mission WITH FULL CARGO; the parker then docks the drone for recharge but never
   transfers cargo — ore is stranded aboard indefinitely. Fix: any drone parked AT A CONNECTOR
   drains its cargo to base (parker Nap(docked) runs the existing unload transfer).
3. **"The weld mission treadmill"** (CONFIRMED — earlier logs show `1 weld mission(s) created`
   every production tick): after "nothing buildable"/"construction stalled" completes a weld
   mission, EnsureWeldMissions re-creates it ~5 s later → dispatch → same outcome → infinite
   charge-burning loop. Fix: per-projector cooldown (5 min) after a stall/nothing-buildable
   completion, with a named log line.
4. **"Two drones, one name"** (CONFIRMED in logs — two 'Mining Drone Mk1_3' rows): duplicate
   DisplayNames make logs/GPS/briefs ambiguous. Fix: suffix asset Name with a short id at
   registration (display only; grid name untouched).
5. **"The starving base"**: parker sets docked drones to Recharge — pulling from BASE batteries.
   On a solar base at night, the fleet recharge drains the colony below its own low-power warning.
   Fix: parker skips Recharge mode (batteries Auto, still topping from surplus) when base storage
   < 30% (`DroneUtil.GroupPower` exists).
6. **"The 2% welder"**: dock-load departs when cargo fill > 2% — a welder can fly out with nearly
   nothing and immediately boomerang for resupply. Fix: load target scaled to the job like the
   charge gate (min(components-for-remaining-blocks, CargoLoadTarget)); the 120 s no-components
   stand-down stays.

**Fold into existing chunks:**
7. **"Mining under the shipyard"** (→ M5): the 80 m exclusion is measured from the CORE point;
   detached colony structures (shipyard pads, outposts within range) are unprotected. Fix: exclude
   deposits within 60 m of ANY colony-owned static grid's AABB, not just the core.
8. **"The ceiling connector"** (→ M5 pads): the dock shimmy assumes a roughly-upward connector
   approach; a downward/side-mounted base connector under an overhang breaks the geometry. Fix:
   classify connector facing at reservation time; deprioritize/reject non-upward pads with a named
   reason until pad classes land.
9. **"The speed mod"** (→ mod-compat, rides M5 ship classes): `GameSpeedCap = 95` hardcodes
   vanilla; speed-mod worlds waste capability. Fix: read the world's max ship speed from the
   environment definition at session load; keep 95 as the floor fallback.
10. **"The asteroid"** (→ M2): zero-g mining is designed for (bore axis = toward ore; gravity
    fallbacks exist) but has NEVER been exercised. M2's axis selection covers it naturally —
    add an asteroid case to M2's acceptance tests rather than new code.
11. **"The rival colony"** (→ future, note only): deposit claims are per-colony state — two
    colonies on one planet can both claim the same deposit and send miners to the same hole.
    Multi-colony arbitration is out of scope until multi-colony play is real; documented here so
    nobody is surprised.
12. **"The thousand-deposit database"** (→ housekeeping, anytime): the deposit DB only grows;
    long-lived worlds accumulate depleted entries forever. Fix: prune Depleted deposits older than
    ~48 h game time (append a tombstone log line).

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
