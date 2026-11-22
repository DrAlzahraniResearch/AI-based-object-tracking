# Architecture

## Two tiers, one vehicle interface

| | Tier A (this repo, now) | Tier B (milestone 6) |
|---|---|---|
| Vehicles | `KinematicFleet`: vectorised point masses, 100–1000+ | PX4 SITL x500 + Gazebo, ~5–20 |
| Control | velocity commands, first-order lag τ = 0.4 s, PX4 x500 limits | MAVSDK offboard velocity setpoints |
| Speed | 30 min of traffic in 3–10 s of wall time (31–172 flights, one CPU core) | real time at best |
| Purpose | algorithms, stress tests, statistics (plan steps 3–9) | validate Tier A results on real autopilot dynamics |
| Base | Docker `eval` (python:3.13-slim) | Ubuntu 24.04 + Gazebo Harmonic (PX4 v1.16.2's supported pair; images `gz`, `px4gz`) |

Both implement `VehicleAPI` (`swarm_sim/vehicle.py`), so a separation policy or scheduler written for
Tier A runs unchanged on Tier B. Tier A limits come from PX4 parameters: MPC_XY_VEL_MAX 12,
MPC_Z_VEL_MAX_UP 3, MPC_Z_VEL_MAX_DN 1.5, MPC_ACC_HOR 3, MPC_ACC_UP_MAX 4, MPC_ACC_DOWN_MAX 3.

## Data flow

```
OSM extract ─► osm_to_world.py ─► data/worlds/<name>/{airspace.json, world.sdf, preview.png}
                                          │
configs/tier_a.yaml + hubs ─► missions ─► routes (cruise layers, clearance-checked)
                                          │
                                  intents (4D boxes, float + integer mm/ms)
                                          │
                              sim.run(policy) ─► tracks, LoS events, intrusions
                                          │
                     metrics.json ◄───────┴──────► audit_run.py (verified C checker)
```

## Separation thresholds (assumptions, configurable)

* Loss of separation: horizontal < 30 m **and** vertical < 10 m. These are working numbers for small
  UAS, not a published standard; change them in `configs/tier_a.yaml`.
* Terminal area: within 40 m of a hub. LoS events there are counted separately because hub
  sequencing (step 7) handles them, not en-route separation.
* Cruise layers: base 40 m, step 15 m. The step must exceed the 10 m vertical threshold by an
  altitude-keeping tolerance (decision S11).
* Certification buffer for the audit: ⌈√(30² + 10²)⌉ = 32 m. Two points ≥ 32 m apart in 3D cannot be
  in LoS, so a certified pair can never produce an LoS event if both conform.

## Milestones

1. **Done:** OSM world, missions, routing, intents, Tier A, metrics, Theorem 1 + verified checker + audit.
2. Routes: k alternative layers and lateral offsets per mission (step 3).
3. **Done:** decentralized separation, horizontal ORCA with a vertical gate as a `SeparationPolicy`
   (step 4); see the ORCA section below.
4. **Done:** strategic scheduler, first-come-first-served intent deconfliction with ground delay and
   layer changes, gated by Theorem 2 (steps 5–6); see the scheduler section below. Next: priority
   pre-emption for emergencies, and better-than-FCFS ordering.
4.5. **Done:** digital twin (`docs/twin.md`): plants behind one telemetry/command interface (Tier A with
   model mismatch and faults, PX4 SITL via MAVSDK, replay), state estimation, online verified conformance,
   replanning, calibration, prediction by Tier A rollouts, MCAP logs, Rerun viewer, Theorem 3, fidelity
   report. Tier B (PX4 SIH, small scale) is pulled forward from milestone 6.
5. **Done:** hub sequencing (step 7): approach fixes, strategic departures with pad choice, tactical landing
   slots with ring holding, arrival priority over ground departures, compact vs spread hub layouts,
   Theorem 4 (registry protocol); see the hub section below.
5.5. **Done:** landing reservations: the planned arrival certified at admission and kept when no better
   tactical slot exists, tactical plans from `now`, bounded emergency priority; `docs/results_milestone5_5.md`.
6. **Built, Gazebo runs not yet verified:** Tier B in Gazebo, PX4 SITL x500s in the OSM world on Gazebo Harmonic
   (`--plant px4gz`, Ubuntu 24.04 images `gz` and `px4gz`; Harmonic, not Jetty, because PX4 v1.16.2 requires it), with
   vehicles spawned at their pads and removed after landing; one scene model of a twin run drawn by Gazebo
   (`gz_view.py`: replay, live, headless video), Foxglove and Rerun (`docs/visualization.md`). Next: larger fleets,
   Tier A scenario replays in Gazebo.
7. Stress sweep ×1…×10 and centralized (global scheduler) vs decentralized (ORCA + local
   negotiation) comparison on identical seeds (steps 8–9).
8. Optional: OpenUTM Flight Blender as the external USS, with the checker as a runtime monitor.

## ORCA separation (milestone 3)

`swarm_sim/orca.py`, enabled with `--policy orca` or `policy: orca` in the config. Each vehicle above
15 m and outside a hub's 40 m terminal area solves, on its own, a small linear program: stay closest
to the route velocity while keeping every neighbour (perfect broadcast within 250 m, at most 12)
outside a 35 m horizontal radius over a 10 s horizon. A neighbour counts only if it is, or will be
within 10 s, vertically closer than 13 m. Extra rules for UTM traffic:

| Rule | Why |
|---|---|
| Vertical gate 13 m, below the 15 m layer step | adjacent layers are already separated; a wider gate made 51 % of vehicle-steps constrained and caused stand-offs |
| Priority shares 25 / 75 | an emergency flight takes a quarter of the avoidance |
| Hard corridor 15 m around the current leg, with a tangential clip around points | tactical avoidance must not break the intent the audit checks; without the clip a vehicle spiralled 33 m off a vertical leg into a building |
| Right of way after 10 s without progress | breaks stand-offs between flights whose intents overlap (two emergency holds 18 m apart, three inspections of one building) |
| Pass-through waypoints accept 5 m | ORCA can hold a vehicle 1–2 m off a waypoint indefinitely |
| Building / no-fly guard (3 s look-ahead) | falls back to the clearance-checked route velocity |

What ORCA cannot fix is overlapping intents: two flights planned into the same airspace at the same
time. Those need the strategic scheduler (milestone 4).

## Strategic scheduler (milestone 4)

`swarm_sim/scheduler.py`, enabled with `--scheduler fcfs`. Requests are processed in request order.
For each request it tries ground delays 0, 10, 20, … s up to 900 s, and at each delay the cruise-layer
options (outbound and return raised by 0–2 same-direction steps of 30 m, lowest total first). It takes
the first plan whose en-route intent boxes are cylinder-separated (Theorem 2, h = 30 m, v = 10 m)
from every accepted box.

* Emergencies are never delayed. If no layer option certifies, they fly anyway, flagged
  `accepted_uncertified`, and their boxes still block later requests.
* A delivery or inspection with no certified plan within 900 s is rejected.
* Terminal boxes (the take-off and landing columns, within 40 m of the pad) are not deconflicted:
  that is hub sequencing, step 7.

The intents were tightened so that strategic separation is affordable:

| Change | Before | After | Why |
|---|---|---|---|
| Separation test | Theorem 1, 32 m in 3D | Theorem 2, 30 m horizontal or 10 m vertical | matches the LoS definition; certifies adjacent layers |
| Vertical buffer | 10 m | 2.5 m | 15 m layer step = 10 m + 2 × 2.5 m |
| Horizontal piece | 100 m | 50 m | tighter axis-aligned boxes on diagonal legs |
| Vertical piece | whole leg | 10 m slices | a descent to a drop point blocked every layer below it for about 90 s |
| Time model | +3.3 s per leg (biased ~10 s early per mission) | fitted on 230 Tier A legs | slack no longer has to absorb the bias |
| Time slack | 10 s + 6 s/min | 5 s + 2 s/min | boxes were 35–85 s long for about 10 s of flight |
| No-fly clearance | centre line only | 20 m margin | ORCA's corridor let one flight drift into a no-fly zone |
| Emergency hover | 30 m | 25 m | 30 m sat exactly 10 m below the 40 m layer |

The search uses a vectorised Python reference by default (`--verified-search` uses the Lean C
checker; `tests/test_scheduler.py` checks both give identical decisions). The audit re-checks every
accepted pair with the verified checker afterwards, so the search is not trusted.

## Hub sequencing (milestone 5)

Under 30 m / 10 m, every pad of a compact hub is within 30 m of every other, so the hub is one runway: two
operations there are separated only by time or by 10 m of altitude. The terminal area is managed in two
places, both certified with Theorem 2 against the same registry as en-route plans:

```
admission (strategic)                         ~20 s before the approach fix (tactical, HubSequencer)
  en-route boxes + departure climb at a pad     hold time x holding point x landing pad, first certified:
  (scheduler may pick any pad of the hub)        current pos -> ring point -> descend -> hold -> pad at 25 m -> land
  route ends at an approach fix 100 m out       nothing within 60 s holding? move ground departures (arrival priority)
  on the inbound bearing                        still nothing at the fix? hold on the ring (certified), retry every 5 s
                                                after 300 s in total: land anyway (contingent, forced into the registry)
```

* **Ring.** Holding points every 60 deg at 100 m from the hub centre, at 25 m, 40 m, the inbound layer, or one
  same-direction level above it. 100 m keeps a holding box (20 m buffer) 30 m clear of every pad column
  (5 m buffer) even for the spread layout's outer pads, 45 m out.
* **Terminal buffer 5 m** (`IntentParams.terminal_lateral_buffer_m`), from PX4's measured tracking (3.5 m p99).
* **Layouts.** Compact: the mini_town hubs as built (8 m pads). Spread: 40 m pads (30 m + 2 x 5 m), so every pad
  column is certifiable against every other and the hub works as several runways.
* **Bookkeeping.** Each plan version records which of its boxes are certified (`cert`), so the twin audit checks
  terminal operations too. A moved departure gets a second decision; reports count the last one.
* **Reservations (`terminal="reserve"`, milestone 5.5).** Admission also certifies the arrival (fix -> pad),
  and the registry marks those rows (`arr`). At the fix the sequencer searches as above; if nothing certifies,
  the flight keeps its reservation (`reserved`). Tactical boxes start at `now`. An emergency that does not
  certify may move ground departures, displace reservations of flights not yet sequencing (they get an
  `unreserved` plan version and sequence tactically), and wait up to 30 s.
