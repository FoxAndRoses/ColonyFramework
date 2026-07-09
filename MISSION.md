# MISSION.md — Ship-Type Mission Contracts

FLIGHT.md made drones fly; this document makes them WORK. It is two things at once:

1. **A template.** Every ship type — present or future (hauler, scout, repair, defense) — is
   specified by filling the five sections below. **No new ship-type controller is written until its
   contract section exists here.** This is the onboarding path for new drone types.
2. **The filled contracts** for the types we have (miner, welder, survey, parker), built the way the
   user directed: as STORIES, played through the real systems (ShipSelfModel numbers, machine
   states, ModAPI limits) until they break. Every failure mode found becomes a design element with
   an owning M-chunk. The stories double as the in-game acceptance scripts.

Rules of use (same as FLIGHT.md): contract edits precede code; `[exists]` marks behavior already
shipped that a story merely validates; every M-chunk implements against a named story.

## Mission invariants (all types)
- **I1** Mission logic never touches actuators — it issues FlightController verbs and reads status.
- **I2** Every mission number derives from ShipSelfModel or a live measurement — never a constant
  pretending to be knowledge. (Constants remain only as hard SAFETY caps.)
- **I3** Every abort/defer is NAMED, with the numbers that caused it — in the log, and in chat when
  the player must act ("returning: 28% stored, return needs 19%").
- **I4** Plans are re-checked at phase boundaries; reality wins over the plan (mass changed, battery
  sagged, target moved → re-derive, don't press on).
- **I5** Failure is graceful by construction: the safe fallback of every decision is the CURRENT
  shipped behavior (a smarter choice may be rejected; a mission may never be stranded by one).

---

## THE TEMPLATE (fill all five for any new ship type)

**1. IDENTITY** — required blocks and capability flags (ties into ShipSelfModel + commissioning
gates): what must exist for this type to commission, and which self-model numbers its decisions use.
**2. MISSION ARC** — the phase sequence; per phase: success, abort (named), defer semantics.
**3. STORIES** — several concrete scenarios walked through the real systems until they break;
each: setup → walkthrough → failure modes found → design element → owning chunk.
**4. DECISION TABLES** — the arithmetic the type runs, inputs annotated with their source.
**5. INVARIANTS** — type-specific rules on top of I1–I5.

---

# MINER contract

## 1. Identity
RC + gyros + 6-axis thrust + drills forward-mounted spanning the hull cross-section (existing build
constraint) + connector(s) + battery/reactor + ore detector optional (survey finds deposits).
Self-model inputs used: mass (live), thrust per axis, TWR at full-cargo mass, braking distances,
cargo capacity/fill, measured full-load draw (commissioning spike), battery capacity.

## 2. Mission arc
Commission(self-test) → Transit(corridor) → **Arrival Analysis (NEW, M2)** → Bore(+ pattern; eject
excursions) → Retreat → Return(corridor) → Dock/Unload/Charge → re-dispatch.
Aborts: self-test deficiency; energy ledger trip (M1); trapped-in-shaft (exists: escalation → named
fail); deposit gone. Defer: none (mining has no per-target defer; the deposit releases for re-claim).

## 3. Stories

### Story M-A — "The core wants iron" (arrival analysis & BORE AXIS SELECTION)
Setup: demand mining dispatches a miner to deposit #107, an iron centroid 38 m inside a mountainside.
Walkthrough today: the drone flies to a standoff ABOVE the deposit and bores STRAIGHT DOWN — through
up to ~90 m of mountain flank that a horizontal approach would skip in 12 m.
**Play it through the systems:** the bore machine is already axis-generic (`Drive`/`ClimbShaft` take
the bore axis; penetration is `Dot(pos − contact, axis)`), so the machine can drill any direction it
can HOVER along. What breaks: (a) depth bookkeeping uses ALTITUDE (TryGetPlanetElevation) for the
descend gate and the surface-penetration cap — meaningless for a horizontal shaft → generalize to
along-axis penetration from the contact point; (b) the + pattern's lateral basis `_u,_v` is built
perpendicular to gravity → build it perpendicular to the BORE AXIS; (c) the eject excursion exits
along −axis then flies level — already fine (the flight core levels from any attitude).
**Axis selection (the decision):** sample the heightmap along ~6 candidate directions from the ore
centroid (up, 4 horizontal compass points, and the steepest-descent direction of local terrain);
rock path per candidate = distance from centroid to surface along that ray (planet
`GetClosestSurfacePointGlobal` walked in 4 m steps, capped 120 m). Feasibility filters from the
self-model: hover-at-attitude (Decision table D2) and back-out thrust along +axis at FULL-cargo
mass ≥ 1.3× the gravity component along the axis. Score = rock path length (shorter wins) with a
1.25× preference multiplier FOR vertical (its junk/eject behavior is best-proven — bias to the
proven path when it's close). **Failure modes found:** horizontal shaft floor accumulates cuttings
the drone re-drills (accept: drills eat them); drone hovering nose-horizontal near a slope clips
terrain on entry (entry point = surface + axis·(hullLength/2 + margin), approach along the axis via
OrientAndCreep — the same tool-first geometry as the welder); heightmap can't see overhangs (camera
raycast along the chosen axis when present, else accept vertical). → **M2.**

### Story M-B — "How deep dare I go" (depth budget)
Setup: pure vertical bore into deep ore; drone at 55 m depth, cargo 70%, battery 46%.
Today: hard 60 m cap and nothing else — the drone that can't climb out at full cargo discovers it by
getting stuck (the escalation tilts and claws; sometimes fails → TrappedFail).
**Play it through:** climbing out is up-thrust vs FULL-cargo mass through a shaft with wall drag and
no dampener help against gravity mispointing. Depth budget BEFORE entering:
`D_max = min(oreDepth + margin, D_climb, HardCap 60)` where `D_climb` maps the full-cargo climb
margin to a depth we trust the escalation to unwind: `TWR_full < 1.25 → 15 m; < 1.5 → 35 m; ≥ 1.5 →
60 m` (table, not formula — the escalation's real behavior is nonlinear; numbers tunable in test).
Mid-bore re-check (I4): cargo mass grows during drilling → when the CURRENT full-mass TWR drops a
band, the budget shrinks; if current depth already exceeds the new budget → stop drilling, eject
excursion, then resume only to the reduced budget. **Failure modes:** gear/thruster snag on shaft
lip during re-entry (exists — reposition centers first); budget shrinks below current depth
mid-drill (handled above); wedging (existing drill-span build constraint — validated, not solved
in code). → **M2** (budget) + existing escalation unchanged.

### Story M-C — "The battery is a fuel tank" (energy ledger — shared with all types)
Setup: deposit 2.1 km out; drone mining at 31% battery. Today: flat 20% abort — which is DEATH at
2.1 km against a headwind of mass (return at full cargo draws more than the outbound leg) and WASTE
at 200 m.
**Play it through:** we have measured full-load draw (commissioning spike, MW), battery stored/cap
(MWh), distance home, cruise speed, and mass. `returnCost ≈ draw × (dist/cruiseSpeed + 120 s
dock reserve)`, evaluated at FULL-cargo mass (draw scales ~linearly with thrust demand; use the
spike draw — it IS the full-load number). Abort-to-return when `stored < returnCost × 1.5`.
Re-evaluated each executor decision (cheap arithmetic), phase boundaries mandatory (I4).
**Failure modes:** draw measured docked ≠ cruise draw (accept ×1.5 margin; refine from live
telemetry later); reactor drones (returnCost ≈ 0 → never trips — correct); battery sag under load
reads low (MinBatteryCharge is stored-energy, not voltage — fine). Chat only when it TRIPS
("'Miner-2' returning: 26% stored, return needs 17%"). → **M1** (replaces LowPowerThreshold in
miner, welder, survey).

### Story M-D — "Where do I throw the junk" (dump vector)
Setup: eject excursion from a shaft on a slope; base 400 m north.
Today: fixed away-from-deposit lateral, 115 m — can throw junk UPHILL (rolls back toward the shaft)
or toward the base pad.
**Play it through:** candidates = 8 compass directions; score = terrain drop over 115 m (heightmap,
2 samples; downhill preferred — junk rolls AWAY) − penalty if within 60° of the base bearing −
penalty if within 100 m of another ACTIVE shaft (deposit claims give their positions). Highest
score wins; tie → current behavior (away from deposit) per I5. **Failure modes:** all directions
uphill (bowl) → dump anyway at the least-bad (junk rolls to the bowl centre, not the shaft — fine);
two miners same deposit — claims already prevent. → **M2** (rides the excursion).

### Story M-E — "The ore is huge" (multi-load cadence) `[exists — validation story]`
Deposit outlives cargo: cargo-full retreat → deliver → deposit RELEASED not depleted → re-claimed →
new mission bores the same field. Validated: the + pattern restarts (accepted cost), depth budget
(M-B) now bounds each pass identically, and the physical shaft persists so re-descent is fast.
Design element: NONE new — story exists to test the loop end-to-end with the M-chunks in place.

## 4. Decision tables (miner)
- **D1 bore axis:** per candidate axis a: `rock(a)` = centroid→surface along a (4 m steps);
  feasible(a) = HoverOK(−a as nose direction, m_full) ∧ `thrustAlong(+a, m_full) ≥ 1.3·m_full·g·|a·ĝ|`;
  choose min rock(a)·(a vertical ? 0.8 : 1.0) over feasible; none feasible → vertical (I5).
- **D2 hover-at-attitude (SHARED, also welder):** nose along n̂ → body axes rotate; gravity-opposing
  thrust `T_up(n̂) = Σ_axes max(0, thrust_axis · (axis_dir(n̂)·ĝ⁻))`; OK iff `T_up ≥ 1.15·m·g`.
  Pure arithmetic on ShipSelfModel.ThrustN — tip-over decided BEFORE flying.
- **D3 depth budget:** `min(oreDepth+2 m, D_climb(TWR_full band), 60 m)`, re-banded when mass grows.
- **D4 energy:** abort-to-return iff `stored < draw_spike × (dist/v_cruise + 120 s) × 1.5` (M1).
- **D5 dump vector:** argmax over 8 compass dirs of `drop(d) − basePenalty(d) − shaftPenalty(d)`.

## 5. Miner invariants
Bore machine mechanics stay untouched (proven) — M2 changes only the TARGETS/axis/limits fed to it.
Vertical remains the default whenever scoring is close (I5). The + pattern is per-axis-frame.

---

# WELDER contract

## 1. Identity
RC + gyros + 6-axis thrust + welder(s) forward-mounted + connector + battery/reactor. Self-model:
welder tool positions/facing, physical radius, thrust per axis (for D2 attitude feasibility),
cargo (components), measured draw.

## 2. Mission arc
Commission → DockLoad(components + job-scaled charge) → Transit(corridor, work volume declared) →
Work loop: Select(claims/stager) → **Reach Solve (NEW, M3)** → Approach(orbit-or-direct) →
OrientAndCreep(weld) → BackOut(SlideTo) → … → Return/Resupply. Defers are per-block (deferred set,
one retry pass). Aborts: ledger trip (M1), welders lost, stall watchdog.

## 3. Stories

### Story W-A — "The reactor behind the frame" (reach solver)
Setup: exoskeleton ~60% built; next stage target is an interior reactor; the volume-centre radial
and one alternative are blocked by frame beams.
Today: ONE radial candidate → reach-test fails → defer → likely "construction stalled" even though
a 40°-down side corridor exists that the drone fits through.
**Play it through:** candidates = the 6 block-face normals of the target cell + 4 slanted blends of
the two most open faces (≤10 candidates, cheap). Per candidate, three gates in cost order:
(1) REACH — CastRay approach-point→block (exists); (2) FIT — 3 rays swept at hull half-width along
the corridor (CastRay against grids works; the projection ghost is not physical and correctly
ignored); (3) ATTITUDE — D2: can the drone hover nose-along-candidate? First candidate passing all
three wins; the creep runs at that angle (the flight core flies any attitude already — deciding was
the missing piece). None pass → defer WITH PROOF: "no fitting corridor: 10 candidates — 6 blocked,
2 unfit, 2 tip-risk". **Failure modes found:** fits going IN but backing out swings the tail into a
beam (SlideTo is attitude-locked — back out along the SAME corridor, exists); candidate passes rays
but clips a diagonal block corner between rays (accept: rays at hull width + WeldStopDist standoff
absorb ~1 block of error; creep is 0.8 m/s — contact at that speed is a nudge, and the weld
timeout defers if truly wedged, I5); block gets welded by the OTHER welder mid-solve (claims
already gate — re-select). → **M3.**

### Story W-B — "Two welders, one hull" `[exists — validation story]`
Claims + 10 m bubbles + presence already spread the fleet; the reach solver adds candidate-level
courtesy: candidates whose corridor passes within the bubble of another welder's CLAIM are
penalized last, not forbidden. Validates: no same-block wells, no mid-air convergence (steering
sees the other drone — it is NOT in the work volume exemption... verify in test: work volume =
construction sphere; the other WELDER inside it would be exempted too → M3 must add drone grids
back as obstacles even inside work volumes — **failure mode found by this story**, design element:
work-volume exemption applies only to STATIC/construction grids, never to registered fleet drones).
→ small fix in **M3**.

### Story W-C — "Empty-handed" `[exists — validation story]`
Components run dry mid-hull → cargo <2% → return/resupply → job-scaled charge (30% for small jobs)
→ back. Validates M1 interplay: the ledger's return trigger must not fight the resupply trigger
(resupply implies return; ledger only preempts if the SITE is farther than the battery allows —
same formula, no special case).

## 4. Decision tables (welder)
- **D2 shared** (hover-at-attitude) — the tip-over gate for angled approaches.
- **D6 reach solve:** first of ≤10 candidates passing reach ∧ fit ∧ D2; claims-bubble penalty
  orders candidates; none → named defer with per-gate counts.

## 5. Welder invariants
Work volume exemption never applies to registered fleet drones (from W-B). The reach solver only
CHOOSES geometry; approach/creep/back-out remain the existing verbs.

---

# SURVEY contract (compact)
**Identity:** RC/gyros/6-axis/ore detector/battery. **Arc:** commission → ring legs (MoveTo) →
scan per point → return (corridor). **Story S-A "The ring outgrows the battery":** ring radius
grows each lap; today MaxWaypoints=12 fixed — at 3 km rings, 12 points may exceed the battery.
M1's ledger replaces the fixed count: fly points until `stored < returnCost×1.5`, then home; the
persisted cursor already makes partial laps lossless. **Story S-B "Nothing out there"** `[exists]`:
demand-survey cooldown + no-deposit escalation validated as-is. **Invariant:** scanning stays
discrete/bounded (CLAUDE.md perf) — the ledger changes WHEN to stop, never scan cost.

# PARKER note (degenerate mission)
Parking already conforms: verbs only (Transit/MoveTo/Dock + touchdown sink), named give-ups with
cooldown, self-knowledge via PrepareForFlight/nap. No stories needed; it is the template's floor.

---

# FLEET-LEVEL contracts (colony-scope, user round-2 additions)

## Blueprint capture — THE UNLOCK `[verify at first use]`
`IMyProjector.SetProjectedGrid(MyObjectBuilder_CubeGrid)` exists (confirmed present in
Sandbox.Game.dll) and mods can serialize any live grid via `GetObjectBuilder()`. Therefore the mod
can CAPTURE a registered drone at commissioning — its known-good, self-test-passing self, with
inventories/battery-state stripped from the OB — persist it in world storage, and paste it into any
same-grid-size projector on demand. This powers three contracts below. Caveats to verify: OB
sanitization (inventories, ownership, battery charge), same-grid-size requirement, MP sync.

## Story F-A — "Missing pieces" (self-repair & decommission; user-directed)
Damage isn't only dents — blocks can be MISSING, and a welder can only restore what a projection
defines. Every drone's captured blueprint (above) is its repair reference.
- Drone self-check (self-model vs live blocks) finds damage → if it carries an ONBOARD projector:
  load own capture, a repair-welder (or itself, if it's a welder and can reach) restores it at a
  repair pad. If the onboard projector is itself dead or the drone is TOO DAMAGED to fly missions
  safely (fails its self-test, or ledger says it can't complete any mission): **DECOMMISSION** —
  fly (or be declared) to a graveyard spot AWAY from base, power down, UNREGISTER, GPS-mark
  "decommissioned — disassembly", and (with blueprint capture) queue a replacement build.
- Failure modes: drone too damaged to REACH the graveyard (decommission in place, mark position);
  repair projection of a moved/rotated drone misaligns (repair happens ON the pad at a fixed
  connector — projector alignment story to play out at implementation). → chunk **M7**.

## Fleet move & formations (user-directed)
`/colony move <gps|here> [wing]` — a Move mission type for N drones: shared corridor, per-drone
lateral/vertical offset slots (formation = anchor + offsets; the flight core already separates
drones via steering, formations make it INTENTIONAL). Prereq for combat wings and base relocation.
→ chunk **M6** (with threat zones — fleeing IS a fleet move).

## Ship classes (user-directed; Elite-style pads)
Class from ShipSelfModel physical size (AABB max dimension): S (<8 m), M (<20 m), L (≥20 m) —
thresholds tunable. Landing pads/connectors advertise capacity by name tag (`[Pad M]`) or
auto-measured clearance; reservation/parking match `class ≤ pad class`. **Mod compatibility is
structural:** every capability is detected by INTERFACE (IMyThrust, IMyShipDrill, IMyShipWelder,
IMyUserControllableGun, IMyGasTank) and measured by its own reported numbers (MaxEffectiveThrust),
so modded thrusters/drills/weapons are covered automatically; blocks that don't implement standard
interfaces are invisible (accepted limit, documented). → rides **M5/M6**.

## Asset VALUE LEDGER + protective reserve (user-directed)
The colony must know what its assets are WORTH to decide what it can afford to send away.
- **ValueScore per asset** (extends ship classes): `tonnage term + material term`. Material term =
  walk the grid's blocks → block definition components → weighted by material rarity (Pt/U/Au/Ag
  heavy, Fe/Si light; weights in one table). Computed at commissioning alongside the capture;
  cached on the AssetRecord (append-only ProtoMember).
- **Protective reserve doctrine:** the colony keeps a configured fraction of total ValueScore
  (default 40%, `/colony reserve <pct>`) at base — reserve members are the HIGHEST-value idle
  assets. Dispatchable surplus = idle ∧ not reserve ∧ not mission-assigned. Every "send ships
  somewhere" decision (reinforcements, fleet move, shipyard timing) draws only from surplus.
- This is the state machine that "knows the value of assets to know when it should act": inputs =
  roster ValueScores + reserve pct + active threats; outputs = surplus list, and a WANTED list for
  the shipyard (M5) when surplus can't meet standing demand.

## RANKS & REINFORCEMENT contract (user-directed)
**Ranks:** SE factions natively expose Founder/Leader/Member (readable per identity). Colony maps
them to command tiers, with per-player overrides persisted in colony state (`/colony rank`).
Tier gates BOTH the command set (the radial menu builds itself from a server-sent capability list —
UI-2 renders only what the server says this player may do) AND the reinforcement budget:
Member → small escort (≤1 ship, ≤S class), Leader → wing (≤3, ≤M), Founder → everything in surplus.
**Reinforcement flow (Story R-A "Send help"):**
player (anywhere) calls via radial/chat → NET-1 packet carries identity + live position → server:
rank tier → budget → ValueLedger surplus selection (nearest-first, class-appropriate) → Reinforce
missions with a DYNAMIC target (the player identity, position re-read each corridor re-plan — the
player moves; corridors already re-plan per leg) → on arrival, drones take formation slots around
the player's ship (M6 offsets) and ADOPT LOCAL CONTEXT:
- combat context (recent DamageSystem events near the player / hostile grids in range) → engage
  per combat doctrine (M8 RoE) — "arrivals join the fight";
- calm context → hold formation, follow the player (Story R-B "Join the fleet") until released
  (`/colony release`) or the ledger sends them home (fuel/energy — D4 applies to the trip HOME
  from wherever the player leads them, checked continuously: an escort that follows too far turns
  back BY NAME before it strands itself).
**Failure modes to play at implementation:** player in a gravity well the ships can't hover in
(D2/TWR gate per candidate before dispatch); player dies/logs off mid-flight (mission retargets to
last position, then RTB); reinforcements pulled while base is under attack (threat zones freeze
the reserve doctrine at 100% — nothing leaves a besieged base); two players calling at once
(surplus is a single pool — first-come, remainder gets a named "insufficient surplus" reply).

## COMBAT DRONE contract (placeholder — full template instance before any code, per the rule)
Identity: turrets/fixed guns (IMyUserControllableGun/IMyLargeTurretBase — interface-detected, mod
weapons included), ammo inventory, speed class. Arc: patrol / escort / intercept / RTB. Doctrine to
write as stories: rules of engagement (fire only on confirmed hostiles that damaged colony assets —
griefing safety), target selection, AMMO AS FUEL in the ledger (D4 generalizes), disengage criteria
(the ledger again: can I fight AND make it home), escort formations (fleet move), and the
self-sacrifice ram as the terminal story (criteria: ledger-dead + no armed help + attacker engaging
others). → chunks **M8+**, contract first.

## Story F-B — "The shipyard" (fleet expansion, upgraded by blueprint capture)
FleetPlanner: sustained mission backlog per type → pick an idle same-grid-size shipyard projector →
`SetProjectedGrid(capture)` → existing weld pipeline builds it → completed grid passes commissioning
self-test → auto-register. No player pre-loading needed. Failure modes: no shipyard projector
(named chat ask), capture stale after player upgrades a drone (recapture on each successful
commission — the fleet's blueprints track their live ships). → chunk **M5**.

# M-chunks (contracts above; each = one commit + Testing block)
| Chunk | Implements | Owner |
|---|---|---|
| M1 MissionLedger + hygiene | D4 (miner/welder/survey, replaces flat 20%) + role eligibility (Scout type, per-mission eligibility) + ore-under-base exclusion + warehouse backpressure + player-control pause | Fable/Opus |
| M1.5 FuelModel | hydrogen/hybrid ledger, ice-is-fuel junk exception, emergency refuel, survival ice demand | Opus w/ contract |
| M2 Miner arrival intelligence | M-A axis selection + M-B depth budget + M-D dump vector; axis-generalized bore bookkeeping | Fable |
| M3 Welder reach solver | W-A candidates/fit/attitude + W-B drone-never-exempt fix | Fable |
| M4 /colony brief | expose ledger/axis/depth/dump reasoning per drone | Opus-able |
| M5 Shipyard + blueprint capture | F-B + capture-at-commission + ship classes (pads) | Fable first (capture `[verify]`), then Opus |
| M6 Threat response + fleet move | flee, no-go volumes, /colony move formations | Opus w/ contract |
| M7 Repair & decommission | F-A | Opus w/ contract |
| M8+ Combat drones | contract instance FIRST, then doctrine chunks | Fable contract |
