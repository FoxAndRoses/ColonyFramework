# FLIGHT.md — ColonyFramework Flight Model: Research, Contracts, and Build Plan

This document is the CONTRACT for the flight-model rebuild. Every implementation chunk is written
against it; any model (Fable, Opus) in any session resumes from it. Rules of use:

- **No chunk starts without reading its contract section here.** If an implementation needs to
  deviate, the deviation is written here FIRST (this doc is the single source of truth).
- **Each chunk = one commit + one in-game test round** with a Testing block (per CLAUDE.md).
- API claims marked `[verify]` must be smoke-checked at the start of the chunk that first uses
  them (one log line proving the value is sane). Claims marked `[proven]` are already validated
  by this project's test rounds — do not re-litigate them.
- ALL existing flight code is FROZEN until its convert chunk. No more patches to the old paths.
- CONVERT STATUS: COMPLETE. All consumers fly the core (welder F4.1, miner + 115 m eject excursion
  F4.2, survey + parker F4.3). F5 wrapped the shared DockMachine as the `Dock` verb (welder + parker
  route through it; the MINER's bespoke dock stays legacy-by-design — proven, and its recovery keeps
  a minimal autopilot hop). AvoidanceProbe and the autopilot cruise helpers are DELETED. Remaining
  intentional direct-control islands: the bore machine, the miner dock, the parker touchdown sink.

---

## 0. Why a rebuild (evidence, not opinion)

Five test rounds produced five classes of structural failure. These are the design inputs;
the new model must make each **impossible by construction**, not merely fixed.

| # | Failure (observed, logged) | Root cause | Killed by |
|---|---|---|---|
| 1 | Welder loops forever: `corridor blocked by 'e' (0–28 m)` → deflect → retry, 2+ min | The under-construction ship is a SEPARATE free grid; the avoidance prober calls it an obstacle while the approach planner steers into it. Two subsystems, two world models. | Work volumes (§5.3) |
| 2 | "Jitter" — visible constant corrections in every precision phase | Bang-bang thrust overrides + autopilot waypoints re-issued every 3 s = two half-controllers fighting over the same actuators | One actuator owner + velocity loop (§3) |
| 3 | Squirm on pad; drones flopping unpowered | State leaks (batteries in Recharge, thrusters off) across dozens of enable/disable sites | FlightController owns actuator state (§5.1) |
| 4 | Eject loop: dump timeout → re-enter full → re-eject; junk rains back into shaft | Magic constants (8 s, 45 s, 15 m) instead of measured capability; dump geometry chosen blind | ShipSelfModel (§4) + verb-composed excursion (§6) |
| 5 | "Gets to a vector then abandons it" — approach never satisfies `dist<3 && v<0.5` | Arrival tolerance vs autopilot's own arrival radius mismatch; churn re-issues reset progress | Arrival semantics with hysteresis (§5.4) |

The meta-lesson: **safety and correctness must live in geometry and ownership, not in reactive
patches.** Where we followed that rule (bore machine, survey legs, dock machine) the code has been
stable for rounds; where we didn't (welder site flight, avoidance detours), every round regressed.

---

## 1. R1 — ModAPI capability audit

### 1.1 Sensing

| Capability | API | Contract & gotchas |
|---|---|---|
| Mass, velocity, gravity at self | `grid.Physics.Mass`, `.LinearVelocity`, `.AngularVelocity`, `.Gravity` | `[proven]` Physics null until streamed-in; always null-check. Mass changes with cargo — re-read per tick, never cache across a load. |
| Gravity at arbitrary point | `MyAPIGateway.Physics.CalculateNaturalGravityAt(pos, out interference)` | `[proven]` Returns vector; zero in space. |
| Altitude above surface | `IMyShipController.TryGetPlanetElevation(MyPlanetElevation.Surface, out d)` | `[proven]` Reliable, no raycast. RC inherits it. Enum in `Sandbox.ModAPI.Ingame`. |
| Terrain height at arbitrary point | `MyPlanet.GetClosestSurfacePointGlobal(ref point)` | `[proven]` The heightmap oracle. Cheap enough per-waypoint; NOT per-tick-per-direction (budget it). Get planet via `MyGamePruningStructure.GetClosestPlanet(pos)`, cache 60 s. |
| Nearby entities (broad phase) | `MyAPIGateway.Entities.GetTopMostEntitiesInSphere(ref sphere)` | `[proven]` Top-most only (subgrid returns its group root... in practice returns physical top-levels; connector-held drones are separate top-levels). Cheap; the grid-obstacle source. |
| Physics ray (entities) | `MyAPIGateway.Physics.CastRay(from, to, out hit)` | `[proven]` **Does NOT hit voxels.** Grids/characters only. Good for structure line-of-sight; useless for terrain. |
| Forward terrain/void sensing | `IMyCameraBlock.EnableRaycast=true`, `.AvailableScanRange`, `.CanScan(d)`, `.Raycast(d, pitchDeg, yawDeg)` → `MyDetectedEntityInfo` | `[verify]` Hits VOXELS and grids — the only true forward terrain sensor. Charges ~2000 m/s while enabled (idle cost); cone limited ±45° from camera facing. `MyDetectedEntityInfo.HitPosition` gives the point. Requires a camera block on the drone (document as build constraint; DEGRADE GRACEFULLY to heightmap-only when absent). |
| Grid extents / block positions | `grid.WorldAABB`, `block.GetPosition()`, `block.WorldMatrix` (Forward/Up per block) | `[proven]` Tool mounts and connector facings come from block WorldMatrix, not the grid's. |
| Battery / power | `IMyBatteryBlock.CurrentStoredPower/MaxStoredPower/CurrentOutput`, `IMyThrust.MaxEffectiveThrust/CurrentThrust` | `[proven]` `MaxEffectiveThrust` already accounts for atmosphere/altitude — use it, not MaxThrust. |

### 1.2 Actuation

| Capability | API | Contract & gotchas |
|---|---|---|
| Thrust | `IMyThrust.ThrustOverride` (N) / `ThrustOverridePercentage` | `[proven]` A thruster pushes along its `WorldMatrix.Backward`. Override 0 returns authority to dampeners. Overrides + dampeners coexist: dampeners brake axes with no override demand. |
| Attitude | `IMyGyro.GyroOverride` + `Pitch/Yaw/Roll` (rad/s, gyro-local) | `[proven]` Needs ≥6 Hz updates AND a damping term (subtract angular velocity) or it tumbles. Existing `BoreController.Face` implements this correctly — reuse, don't rewrite. |
| Braking / hold | `IMyShipController.DampenersOverride` | `[proven]` The free hover/brake. Gravity is always cancelled when dampeners on + sufficient thrust. |
| Autopilot | `IMyRemoteControl` waypoints + `SetAutoPilotEnabled` | `[proven]` Real contract: **gravity-level only (never pitches down; waypoint below = endless yaw-hunt)**, own arrival radius (imprecise vs our 3 m tolerances), releases nothing (gear/connectors), best at long level cruise. In the new model: **retired from precision work**; optional fallback for >1 km cruise legs only. |
| Docking | `IMyShipConnector` Connectable→Connect | `[proven]` DockMachine already extracted and stable. |

### 1.3 What does NOT exist (design around, never attempt)
- No navmesh, no pathfinding service, no voxel raycast via Physics.CastRay.
- No per-tick voxel queries at scale (CLAUDE.md perf rule; scans are discrete/bounded).
- No reliable "is this volume free" test except: heightmap (terrain) + AABB/sphere checks (grids)
  + camera raycast (line sample). Spatial awareness = composition of those three.

---

## 2. R3 — Prior art (what real drones and other games/mods do)

- **Quadrotor stack (PX4/ArduPilot pattern):** cascade of small loops — position error → desired
  velocity (bounded by a trapezoidal profile) → velocity error → desired acceleration/attitude →
  motor mix, with gravity feedforward. Each loop simple; robustness comes from the cascade and
  from real vehicle parameters (mass, max accel), not from a smart planner. **This maps 1:1 to SE**
  (§3) because SE gives us mass, per-axis thrust, and direct force actuators.
- **Context steering (game AI, replaces our detour waypoints):** each decision tick, score a small
  fan of candidate directions by (goal alignment) − (obstruction) − (turn cost); steer along the
  best. Output is a smooth continuously-blended direction, not a teleporting waypoint. No local
  minima oscillation (a blocked direction lowers a score; it doesn't spawn a contradictory goal).
  Our observed deflection loops are the textbook potential-field failure it was invented to fix.
- **SE ingame-script prior art (PAM, SAM):** the mature SE autopilots fly PRECISION phases with
  gyro+thrust overrides and use Keen autopilot for nothing or only coarse legs — independent
  confirmation of §1.2's conclusion. MES/RivalAI similarly wrap custom control, not waypoints.
- **Boids/flocking separation:** drone-vs-drone spacing as a soft steering term (already partially
  achieved via WeldCoordinator bubbles at TARGET level; F2 adds it at the steering level).

---

## 3. R2 — Control architecture (the jitter killer)

One controller per drone, ticked at executor rate (~6 Hz), owning ALL actuators:

```
goal (from verb)                                   ShipSelfModel
   │                                                    │
   ▼                                                    ▼
position error e ──► desired velocity v* = min(vMax, sqrt(2·aBrake·|e|)) · ê   (trapezoid)
   │                                   ▲ capped per-axis by profile + steering (§5.3)
   ▼
velocity error (v* − v) ──► desired accel a* (P gain, clamped to per-axis capability)
   │
   ▼
force F = m·a* + m·g_feedforward(only the component overrides must carry)
   │
   ▼
per-axis thrust allocation: F projected on body axes → ThrustOverride on that axis's thrusters;
axes with |demand| < deadband get override 0 (dampeners hold them — free damping, no chatter)
```

Design rules:
- **Braking-aware speed is the "fast vs slow" answer:** `v* = sqrt(2·aBrake·dist)` means a ship
  with weak thrust automatically approaches slowly and a strong one flies fast — no speed
  constants per phase. `aBrake` comes from ShipSelfModel per axis, derated ×0.7 safety.
- **Gravity feedforward & dampeners (CORRECTED at F1):** dampeners counter-thrust against ALL
  velocity — including velocity we commanded — so they fight overrides harder the faster we fly
  (fine at the dock's 0.25–2 m/s creeps, crippling at cruise). Rule: dampeners are OFF while a
  translation verb actively controls (the verb applies full gravity feedforward `F += −m·g`), and
  ON for Hover-idle, verb completion, Release, and every failure path (dampeners are the safety
  net — any abnormal exit restores them). This matches PAM/SAM practice.
- **Deadbands + hysteresis kill jitter:** no override changes below force epsilon; arrival state
  latches (§5.4) instead of flickering.
- **Attitude:** reuse `BoreController.Face` (damped, proven). The velocity loop must work at ANY
  attitude (thrust allocation is body-frame) — this is what makes 45°-down weld approaches and
  nose-down boring first-class, not special cases.
- Gyro/thrust update budget: ≤6 Hz decision, overrides persist between ticks (no per-frame spam).

---

## 4. ShipSelfModel — "what a ship believes it is" (chunk S1)

Computed at registration, refreshed on block add/remove and on cargo-mass change >10%:

```
Body:        gridSize, worldAABB extents, physicalRadius, mass, centerOfMass
Propulsion:  thrustN[6 body axes] (Σ MaxEffectiveThrust), gyroCount,
             accel[axis] = thrustN/mass, TWR_up = thrustN(up)/(m·g),
             aBrake[axis], brakingDist(v) = v²/(2·aBrake·0.7),
             vSafeApproach, maxClimbRate
Power:       battery capacity, measured active draw (existing commissioning spike, kept),
             endurance estimate
Tools/ports: connectors[] (pos, facing — which is the DOCK one, which face up for eject),
             welders[]/drills[]/oreDetectors[]/cameras[] (pos, facing),
             cargo capacity (m³), current fill
Flags:       canHoverAtFullCargo (TWR_up@maxMass > 1.15), canPitchWork (6-axis thrust),
             hasForwardSensor (camera present)
```

**Launch self-test ritual (replaces blind commissioning; user-specified):**
1. Wake (batteries Auto, thrusters+gyros on) — anchored spike test retained for draw measurement.
2. Undock/unlock, ascend 10 m on the velocity loop, hold 2 s.
3. Compare measured accel vs model (>25% shortfall = fail), measured draw vs battery sustainable.
4. Proceed, or abort with a NAMED deficiency: `self-test FAIL: up-TWR 1.02 at current mass — add
   up thrusters or shed cargo`. Chat + log; asset flagged unfit until re-test.

Persistence: compact capability summary onto `AssetRecord` (append-only ProtoMembers) for
`/colony assets`. The live model itself is in-memory (Services), rebuilt on load.

## 5. FlightController — verbs and invariants (chunks F1–F2)

### 5.1 Invariant: ONE actuator owner
`FlightController` (one per drone) is the only code that touches thrusters, gyros, dampeners,
autopilot, or battery mode. Mission controllers (miner/welder/survey/park) become pure state
machines: they REQUEST a verb + read status; they never see an actuator. All Prepare/Stabilize/
Nap logic moves inside. (Kills failure classes 2 and 3.)

### 5.2 Verb contracts (all: entry conditions, per-tick behavior, completion, failure with reason)
- `Hover()` — hold position/attitude on dampeners + deadbanded corrections.
- `MoveTo(point, profile)` — velocity-profiled translation, any attitude, steering-aware (§5.3).
- `SlideTo(point, keepFacing)` — MoveTo with attitude locked (bore repositioning, shimmy).
- `OrientAndCreep(dir, stopPredicate, vCreep)` — face `dir` (tool-frame optional: pass which
  block's forward must align), creep with braking-aware speed; stop when predicate true
  (weld-in, dock reverse, touchdown).
- `ClimbOut(minAgl)` — vertical extraction with the existing stall-escalation (proven ClimbShaft
  logic, wrapped).
- `Land(spot)` — descend, gear lock, settle (parker logic, wrapped).
- `Dock(connector)` — the existing DockMachine, wrapped (already proven).
- Verb status: `Running | Done | Failed(reason)`. One verb active per drone; issuing a new verb
  cleanly preempts (overrides zeroed, then new verb owns).

### 5.3 Invariant: work volumes + one shared profile (spatial awareness)
- Every verb runs under a `FlightProfile` (per ship-type tunables: cruiseAlt, minAgl, bubble,
  arriveTol, vMax caps). Planner, steering, and verbs read the SAME profile object — a maneuver
  can never request a point the steering layer vetoes (kills failure class 1's cousin: the
  4 m-AGL approach point vs 8 m clearance prober contradiction).
- The active verb may declare **work volumes** — spheres/AABBs that are NOT obstacles (the
  construction being welded incl. any grid intersecting it, the shaft being bored, the base near
  dock). Steering treats work-volume space as free; geometry (radial approach, orbit-slide,
  dock machine) owns safety inside it. (Kills failure class 1 — grid 'e' is inside the declared
  construction volume.)
- Steering (F2): per decision tick score ~16 candidate directions (goal alignment − terrain
  penalty from 2–3 heightmap probes − grid-sphere penalty − turn cost); blend into the velocity
  loop's direction. Camera raycast (when present) samples the chosen direction ahead as a veto.
  Output is continuous steering — never a detour waypoint, never a contradictory goal.

### 5.4 Arrival semantics (kills failure class 5)
`Arrived(point) = |pos−point| < tol` latched with hysteresis (re-arms only if error > 2·tol),
speed criterion from the velocity loop itself (v* has decayed), no external autopilot involved.

## 6. Consumer converts (chunks F4.x) — recorded requirements
- **WELDER (first convert, currently frozen):** transit = MoveTo corridor; site = orbit-or-direct
  (OrbitNav kept) as MoveTo goals inside a declared construction work volume; weld-in =
  `OrientAndCreep(tool=welder, dir=radial any angle incl. 45° down, stop=tool within
  weldReach+blockHalf)`; charge gate stays job-scaled.
- **MINER:** bore machine survives as-is (proven) on SlideTo/ClimbOut/OrientAndCreep; **eject
  excursion (user spec):** ClimbOut → level → MoveTo(≥110 m horizontal dump point, per-drone
  angle) → Hover + EjectJunk until junk GONE (no cargo-room early exit; 90 s cap) → MoveTo back →
  re-enter same shaft at recorded depth.
- **SURVEY / PARKER:** mechanical converts (MoveTo/Land/Dock).
- **LCD (rides with any chunk):** add `lcd: N tagged surface(s)` log line to split "finder finds 0"
  from "writes invisibly"; fallback direct `IMyTextPanel` path if N>0 but blank.

## 7. Build plan & delegation

| Chunk | Content | Owner | Acceptance (in-game) |
|---|---|---|---|
| R (this doc) | Research + contracts | Fable | User sign-off on FLIGHT.md |
| S1 | ShipSelfModel + self-test ritual + /colony assets summary | Opus (spec §4) | `self-test: up-TWR …, brake … — pass` lines; under-thrusted drone aborts with named deficiency |
| F1 | FlightController + velocity loop + Hover/MoveTo/SlideTo + actuator ownership | Opus (spec §3, §5.1–5.2) | A test drone flies pad→point→pad visibly smooth (no jitter), correct fast/slow from its own TWR |
| F2 | Context steering + work volumes + camera probe | Fable | Drone threads terrain + parked grids without deflection loops; log shows steering scores, zero `deflected` spam |
| F3 | Corridors (terrain-clamped polylines) + arrival-speed planning | Opus (spec §5.3) | Long transit hugs terrain profile at cruiseAlt, slows into arrival |
| F4.1 | WELDER convert (unfreeze) | Fable | Multi-block projection completes from multiple angles incl. downward; zero deflection loops |
| F4.2 | MINER convert + 110 m eject excursion | Opus | Eject: level, ≥110 m, ALL junk, same-shaft resume deeper each cycle |
| F4.3 | Survey + Parker converts; delete legacy CruiseTo ×3 + AvoidanceProbe detours | Opus | Regression round green |
| F5 | Dock verb wrapper | Opus | Docking unchanged (already proven) |

Delegation rules: Opus chunks are single-file, contract-complete (this doc + one section), and must
not modify anything outside their listed files; Fable reviews every chunk diff before it merges;
any contract ambiguity found mid-chunk is resolved by EDITING THIS DOC first, then the code.
