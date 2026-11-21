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
| 7. Shared landing hubs | hubs + pads | `configs/hubs_*.yaml`; pad scheduling next |
| 8. Stress tests | ready | `run_tier_a.py --rate-scale` |
| 9. Centralized vs decentralized | not started | both will be `SeparationPolicy` + scheduler variants on the same runs |

## Quick start (Docker Desktop, from the repo root)

```bash
docker compose -f docker/compose.yaml build eval proofs
docker compose -f docker/compose.yaml run --rm eval                 # Python tests (the FFI test file skips here)
docker compose -f docker/compose.yaml run --rm eval bash scripts/demo.sh
docker compose -f docker/compose.yaml run --rm proofs               # Lean proofs, C library (ABI 2), 10 FFI tests
docker compose -f docker/compose.yaml run --rm proofs bash -c \
  "for r in runs/mini_town_*_s*; do python3 scripts/audit_run.py \$r; done"
```

`eval` and `sim` reuse the images already verified on the MOT repo (same bases, same
`infra/install_stack.sh`). `proofs` is new: Lean 4.36.0 + mathlib v4.36.0.

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
* Terminal LoS (within 40 m of a hub) is untouched by design; it is the next milestone (step 7).
* The no-policy and ORCA numbers differ from milestone 3 because routes now keep 20 m from no-fly zones
  and emergencies hover at 25 m (decision S18), which changes the generated missions.

## Layout

```
swarm_sim/   osm, vehicle (Tier A fleet + VehicleAPI), missions, intents, sim, orca, scheduler, metrics, plot
proofs/      Lean: UtmChecker.lean (executable Int checker) + UtmProofs/ (spec, theorems 1 and 2, refinement)
runtime/     C shim + header + Makefile → libutm_checker.{so,a}; ctypes binding utm_checker.py
scripts/     osm_to_world, run_tier_a, audit_run, compare_policies, demo.sh
configs/     tier_a.yaml, hubs_mini_town.yaml
docker/      eval, proofs, sim (Gazebo Jetty) images + compose
docs/        architecture, proofs, osm, decisions
```

See `docs/architecture.md` for the two-tier design and milestones and `docs/proofs.md` for exactly
what is proven and what is trusted.
