# Civilian Drone Swarm Simulation

Decentralized multi-drone traffic management for high-density civilian operations (delivery,
emergency, inspection), with machine-checked separation guarantees.

| Plan step | Status | Where |
|---|---|---|
| 1. Civilian environment | done | `swarm_sim/osm.py`, `scripts/osm_to_world.py` (OSM → airspace + Gazebo Jetty SDF) |
| 2. Delivery / emergency / inspection missions | done | `swarm_sim/missions.py` (Poisson demand, priorities) |
| 3. Multiple routes | partial | semicircular cruise layers + clearance-checked out-and-back; alternatives are milestone 2 |
| 4. Autonomous separation | done (milestone 3) | `swarm_sim/orca.py`: decentralized ORCA with corridor, priority and right-of-way rules |
| 5. Traffic scheduling | done (milestone 4) | `swarm_sim/scheduler.py`: FCFS strategic deconfliction (ground delay + layer changes) |
| 6. Detect / avoid conflicts | done | strategic: scheduler gated by Theorem 2; tactical: ORCA; verified audit (`scripts/audit_run.py`) |
| 7. Shared landing hubs | done (milestones 5, 5.5) | `swarm_sim/hubs.py`: approach fixes, certified landing slots with ring holding, arrival priority; landing reservations at admission, emergency priority; compact vs spread layouts; Theorem 4 |
| 8. Stress tests | ready | `run_tier_a.py --rate-scale` |
| 9. Centralized vs decentralized | not started | both will be `SeparationPolicy` + scheduler variants on the same runs |
| Digital twin (milestone 4.5) | done | `swarm_sim/twin/`: telemetry → state → verified conformance → replan / calibrate / predict, against Tier A, PX4 SITL or a replayed log; Theorem 3 (`docs/twin.md`) |

## Quick start (Docker Desktop, from the repo root)

```bash
docker compose -f docker/compose.yaml build eval proofs
docker compose -f docker/compose.yaml run --rm eval                 # Python tests (the FFI test file skips here)
docker compose -f docker/compose.yaml run --rm eval bash scripts/demo.sh
docker compose -f docker/compose.yaml run --rm proofs               # Lean proofs, C library (ABI 3), FFI tests, twin audit test
docker compose -f docker/compose.yaml run --rm proofs bash -c \
  "for r in runs/mini_town_*_s*; do python3 scripts/audit_run.py \$r; done"
```

`eval` and `sim` reuse the images already verified on the MOT repo (same bases, same
`infra/install_stack.sh`). `proofs` is Lean 4.36.0 + mathlib v4.36.0. `px4` (milestone 4.5) is PX4 v1.16.2
SITL + MAVSDK built on the proofs image.

Digital twin (milestone 4.5, `docs/twin.md`):

```bash
docker compose -f docker/compose.yaml build proofs px4               # px4 builds FROM proofs; PX4 build ~10-25 min
docker compose -f docker/compose.yaml run --rm eval bash scripts/twin_demo.sh   # Tier A plant matrix (~40 min)
docker compose -f docker/compose.yaml run --rm proofs bash -c \
  "for r in runs_twin/mini_town_tier_a*; do python3 scripts/audit_twin.py \$r; done"
docker compose -f docker/compose.yaml run --rm px4 python3 scripts/run_twin.py --world data/worlds/mini_town \
  --plant px4 --missions 3 --speed 1 --verified --tag session1
docker compose -f docker/compose.yaml run --rm px4 python3 scripts/fidelity_report.py runs_twin/mini_town_px4_s1_x1_session1
pip install rerun-sdk && python scripts/twin_view.py runs_twin/<run> && rerun runs_twin/<run>/twin.rrd   # 3D, on Windows
```

Hub sequencing (milestone 5):

```bash
docker compose -f docker/compose.yaml run --rm eval bash scripts/hubs_demo.sh         # 63 runs, ~2-3 h
docker compose -f docker/compose.yaml run --rm proofs bash -c \
  "for r in runs_twin/mini_town_tier_a*fix_* runs_twin/mini_town_tier_a*hub{seq,res}*; do python3 scripts/audit_twin.py \$r > /dev/null && echo ok \$r; done"
```

`--hubs sequence` is milestone 5 (tactical landing slots only); `--hubs reserve` adds landing reservations at
admission (milestone 5.5), `--reserve-fallback` admits without one instead of rejecting.

## Results: hub sequencing (milestone 5, mini_town, seeds 1–3)

Same missions in every configuration; 30 m / 10 m separation everywhere, the hubs included. Full tables:
`docs/results_milestone5.md`.

| Demand | Hubs | Requests flown | Terminal LoS | Mean ground delay | Arrival holding mean / p95 |
|---|---|---|---|---|---|
| ×1 | off / compact / spread | 106 / 106 / 106 of 106 | 59 / 3 / 1 | 35 / 129 / 41 s | – / 13 / 2 s, – / 53 / 8 s |
| ×2 | off / compact / spread | 205 / 178 / 205 of 205 | 264 / 29 / 10 | 90 / 319 / 109 s | – / 18 / 6 s, – / 61 / 30 s |
| ×5 | off / compact / spread | 424 / 225 / 383 of 515 | 907 / 63 / 40 | 459 / 464 / 462 s | – / 19 / 10 s, – / 73 / 48 s |

* Terminal-area conflicts fall 94–98 %, and no loss of separation anywhere is between two certified flights:
  every remaining event involves a contingent landing (no certified slot within 300 s) or an uncertified
  emergency. All 30 runs pass the verified audit.
* Under 30 m / 10 m a compact hub (pads 8 m apart) is one runway and caps throughput; 40 m pad spacing
  restores it (×2: everything flies, 6 s mean holding).
* **Theorem 4**: the registry protocol keeps every pair of certified flights separated.

**Milestone 5.5, landing reservations** (`docs/results_milestone5_5.md`). Contingent arrivals all came from
the same cause: a flight was launched without a landing it could fly, and departures certified in the meantime
filled the terminal area. Certifying the planned arrival at admission as a reservation, kept when no better
tactical slot exists, cuts contingent arrivals from 23 / 26 to 1 / 4 at ×2 (compact / spread) and from 40 / 64
to 5 / 6 at ×5. Every remaining case traces to an emergency. Terminal LoS falls from 29 / 10 to 2 / 1 at ×2.
The cost is capacity. With spread hubs at ×2 it is ground delay (201 of 205 flown, mean 289 s). With compact
hubs it is throughput (×2: 144 of 205 flown, against 178). All 33 reservation runs pass the verified audit.

## Results: digital twin (milestone 4.5, mini_town, seeds 1–3)

Plant: the fleet flies 15 % slower than the planner assumes, one vehicle degrades to 4 m/s mid-flight,
telemetry is noisy and late (`configs/twin.yaml`). Full table and discussion: `docs/results_milestone4_5.md`.

| Demand | Twin | Conformance (ground truth) | Landing overruns | 60 s prediction error, median / p95 |
|---|---|---|---|---|
| ×2 | matched plant (control) | 0.9998 | 0 | 0.35 / 2.7 m |
| ×2 | open loop | 0.9809 | 71 | 4.46 / 49.2 m |
| ×2 | replan only / calibrate only | 0.9928 / 0.9947 | 32 / 15 | 4.43 / 49.2 m, 0.38 / 3.0 m |
| ×2 | **full twin** | **0.9989** | **3** | **0.38 / 3.0 m** |

* The twin learns the fleet's real speed (8.5 m/s) from telemetry, replans flights that leave their
  intents from their measured positions, and predicts 60 s ahead by forking Tier A from its own state.
* Conflict prediction (no deconfliction, ×2): all 24 en-route conflicts predicted at least 10 s ahead,
  median lead ~50 s of a 60 s horizon.
* **PX4 SITL** (3 deliveries, two sessions): uncalibrated, conformance 0.935, with PX4 lifting off 3.9 s
  late and cruising at 8.5–9 m/s against 10 commanded. Starting the next session from the first one's
  calibration: conformance 1.000, and PX4 tracks its plan within 3.5 m / 0.36 m (p99).
* All 29 twin runs pass the verified audit (Lean checker): every certification the online scheduler
  claimed is confirmed, and no certified pair lost separation inside en-route intents.
* **Theorem 3** (deviation bounds) turns a measured tracking error into a certificate: plans separated
  with boxes widened by the error stay safe for that autopilot.

## Results: separation policy × strategic scheduler (mini_town, seeds 1–3)

Totals over three seeds per demand level (`scripts/compare_policies.py`; per-run table in
`docs/results_milestone4.md`). Ground delay is the mean over flown requests.

| Demand | Configuration | Requests flown | Rejected | Uncertified emergencies | Mean ground delay | En-route LoS per flight h | Terminal LoS |
|---|---|---|---|---|---|---|---|
| ×1 | none | 106 / 106 | 0 | 0 | 0 s | 1.56 | 63 |
| ×1 | ORCA | 106 / 106 | 0 | 0 | 0 s | 0.31 | 63 |
| ×1 | FCFS | 106 / 106 | 0 | 1 | 33 s | 0 | 61 |
| ×1 | ORCA + FCFS | 106 / 106 | 0 | 1 | 33 s | 0 | 61 |
| ×2 | none | 205 / 205 | 0 | 0 | 0 s | 1.97 | 268 |
| ×2 | ORCA | 205 / 205 | 0 | 0 | 0 s | 0.65 | 269 |
| ×2 | FCFS | 205 / 205 | 0 | 8 | 89 s | 0.29 | 276 |
| ×2 | ORCA + FCFS | 205 / 205 | 0 | 8 | 89 s | 0.07 | 276 |
| ×5 | none | 515 / 515 | 0 | 0 | 0 s | 5.42 | 1667 |
| ×5 | ORCA | 515 / 515 | 0 | 0 | 0 s | 1.86 | 1652 |
| ×5 | FCFS | 430 / 515 | 85 (17 %) | 23 | 442 s | 0.17 | 904 |
| ×5 | ORCA + FCFS | 430 / 515 | 85 (17 %) | 23 | 442 s | 0.03 | 902 |

* All 36 runs: zero building or no-fly intrusions; the verified audit (Theorem 2) found zero
  counterexamples and zero disagreements with the Python reference.
* Every en-route LoS event that remains under the scheduler (11 in 18 runs) involves an emergency that
  could not be certified and flew anyway. No LoS occurred between two certified flights.
* The cost of strategic separation is ground delay, and at ×5 rejections: with these buffers the town
  saturates between ×2 and ×5. About 80 % of the conflicts that cause delay are away from the hubs
  (crossings on the same layer, 52 %; descents to drop points, 28 %).
* ORCA alone loses conformance at ×5 (down to 0.938) because its in-flight delays now exceed the
  tighter intent windows. With the scheduler, ORCA rarely has to act and conformance stays ≥ 0.992.
* Terminal LoS (within 40 m of a hub) is untouched by the scheduler alone; milestone 5 (hub sequencing) takes it on.
* The no-policy and ORCA numbers differ from milestone 3 because routes now keep 20 m from no-fly zones
  and emergencies hover at 25 m (decision S18), which changes the generated missions.

## Layout

```
swarm_sim/   osm, vehicle (Tier A fleet + VehicleAPI), missions, intents, sim, orca, scheduler, hubs, metrics, plot
swarm_sim/twin/  telemetry (plants: Tier A, replay), px4 (PX4 SITL via MAVSDK), state (estimation, conformance,
             calibration), core (the loop, prediction), log (MCAP), figures
proofs/      Lean: UtmChecker.lean (executable Int checker) + UtmProofs/ (spec, theorems 1-3, refinement, registry: theorem 4)
runtime/     C shim + header + Makefile → libutm_checker.{so,a}; ctypes binding utm_checker.py
scripts/     osm_to_world, run_tier_a, audit_run, compare_policies, demo.sh;
             run_twin, audit_twin, fidelity_report, twin_view (Rerun), compare_twin, twin_demo.sh;
             compare_hubs, hubs_demo.sh
configs/     tier_a.yaml, twin.yaml, hubs_mini_town.yaml
docker/      eval, proofs, px4 (PX4 SITL + MAVSDK), sim (Gazebo Jetty) images + compose
docs/        architecture, twin, proofs, osm, decisions, results_milestone4, results_milestone4_5, results_milestone5
```

See `docs/architecture.md` for the two-tier design and milestones and `docs/proofs.md` for exactly
what is proven and what is trusted.
