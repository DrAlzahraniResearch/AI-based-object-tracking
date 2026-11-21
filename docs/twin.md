# Digital twin (milestone 4.5)

Milestones 1–4 are a simulator: everything is generated at t = 0 and runs open-loop. A digital twin
adds a live link to a system that it does not control internally, a state estimate kept synchronised
with it, and a loop where the model predicts, compares and acts. This milestone builds that loop and
runs it against three plants.

## What the twin is a twin of

There is no real fleet. The "physical" side is one of:

| Plant | What it stands for | Ground truth |
|---|---|---|
| `TierAPlant` | a fleet whose dynamics differ from the planning model: configurable speed limits, GNSS-like noise (0.7 m / 0.5 m), 0.25 s link latency, injected faults | yes (used only to score the twin) |
| `Px4Plant` | PX4 v1.16.2 SITL with its built-in simulator (SIH), one instance per vehicle, flown through MAVSDK in offboard mode | PX4's own estimate |
| `ReplayPlant` | a recorded twin log, replayed in shadow mode (commands are recorded, not applied) | the log |

The twin only sees `Telemetry` samples (timestamped, noisy, late, with mission progress and a route
version) and talks back through `dispatch`, `set_route` and `cancel`. Swapping the plant does not
change a line of the twin. The same interface takes real flight logs, Remote ID (ASTM F3411) or a live
UTM feed later. The accurate claim is: a twin of PX4-in-SIH traffic, architecture-ready for real
aircraft, not a twin of real traffic.

## The loop (`swarm_sim/twin/core.py`, every 0.2 s of plant time)

1. **Ingest.** `TwinState` keeps an alpha-beta filter per vehicle and extrapolates over the link
   latency. Progress fields from a route version older than the twin's are ignored (after a replan,
   late samples from the old route are still in flight).
2. **Admit.** Requests arrive at their filing time. The scheduler's incremental `admit()` certifies a
   plan against every intent already in force (Theorem 2, as in milestone 4) and the plan is
   dispatched, or the request is cancelled. Batch FCFS is now a loop over `admit()`; milestone-4 runs
   reproduce bit for bit.
3. **Conformance.** `ConformanceMonitor` checks each estimate against the vehicle's current intent
   boxes with the Lean-verified `inside` (or the Python reference when the library is not built). A
   vehicle is flagged after 2 s of continuous misses and cleared after 2 s of hits.
4. **Replan.** A flagged vehicle is re-planned from its measured position: its old boxes are withdrawn,
   the remaining route gets fresh time windows, and the scheduler's `replan()` tries airborne holds of
   0, 10, … 120 s until the new boxes certify against everyone else (`replanned`), else flies it
   anyway and registers it (`contingent`). If the vehicle is measurably slower than the plan, its own
   measured speed is used. Not replanned: the final descent at the pad (a `landing_overrun` alert,
   which is hub sequencing, step 7).
5. **Calibrate.** `SpeedCalibrator` measures cruise ground speed (mid-leg) and climb and descent rates
   per vehicle. The fleet model is the median of per-vehicle medians over the 20 most recent
   vehicles (at least 3), so one degraded vehicle cannot drag the fleet. A drift above 3 % updates the
   planning model used for new plans, replans and predictions.
6. **Predict.** Every 10 s, `Predictor` forks a Tier A simulation from the estimated state (positions,
   velocities, waypoint progress, holds, per-vehicle speed caps, flights about to take off) and rolls
   it 60 s ahead with the same separation policy. Predicted positions at +10/30/60 s are scored
   against what happens, and predicted en-route LoS against the plant's LoS events.

With `--hubs sequence` (milestone 5) the loop also sequences arrivals: about 20 s before its approach fix, each
inbound flight gets a certified landing slot from `swarm_sim/hubs.py` (holding on a ring around the hub if
needed, ground departures moved if an arrival would otherwise hold over 60 s), logged on `/twin/arrival`.
See `docs/architecture.md` (hub sequencing).

Everything goes to `twin_log.mcap` (MCAP, the rosbag2 storage format; JSON messages):
`/twin/telemetry`, `/twin/state`, `/twin/decision`, `/twin/alert`, `/twin/replan`,
`/twin/calibration`, `/twin/prediction`, `/twin/arrival`, `/plant/truth`.

## Verification

* **Online:** conformance decisions come from the verified `inside`, and admission can use the
  verified separation check (`--verified`).
* **Offline:** `scripts/audit_twin.py` works on plan versions (each version of a flight's boxes is in
  force over [valid_from, valid_to)). Whenever two versions were in force together, the later one was
  certified against the earlier one online. The audit re-checks every such claim with the verified
  checker, clipped to the shared window. It then checks every ground-truth sample: a sample where two
  flights are in loss of separation while both are inside en-route boxes of mutually certified
  versions would contradict Theorem 2.
* **Theorem 3** (`proofs/UtmProofs/Spec.lean`, `no_los_of_deviation`; checker `separatedDev`, ABI 3)
  covers the sim-to-real gap. If each aircraft stays within (eh, ev, δ) of a point of its nominal
  volume, cylinder separation of the volumes widened by (eh, ev, δ) rules out loss of separation;
  equivalently, the nominal gaps must grow by twice the error (`sepHV_inflate_iff`).
  `scripts/fidelity_report.py` measures that error for a plant.

## Commands

```bash
python scripts/run_twin.py --world data/worlds/mini_town [--seed 2] [--rate-scale 2] [--verified]
python scripts/run_twin.py --world data/worlds/mini_town --no-faults           # matched plant
python scripts/run_twin.py --world data/worlds/mini_town --no-replan --no-calibrate   # open loop
python scripts/run_twin.py --world data/worlds/mini_town --no-faults --admit-all --policy none  # prediction benchmark
python scripts/run_twin.py --world data/worlds/mini_town --plant px4 --missions 3 --speed 1     # Tier B (px4 image)
python scripts/run_twin.py --world data/worlds/mini_town --plant replay --replay runs_twin/<run>/twin_log.mcap
python scripts/audit_twin.py runs_twin/<run>          # verified (proofs image)
python scripts/fidelity_report.py runs_twin/<run>     # Theorem 3 tracking error, Tier A vs plant, leg timing
python scripts/twin_view.py runs_twin/<run>           # -> twin.rrd; `rerun twin.rrd` (pip install rerun-sdk)
bash scripts/twin_demo.sh                             # the milestone-4.5 matrix + audits + comparison
```

The scenario knobs (plant speed limit, noise, latency, faults; PX4 vehicles and speed factor) are in
`configs/twin.yaml`.

## Results and limits

Results: `docs/results_milestone4_5.md`. Known limits:

* The twin is of PX4-in-SIH traffic, not of real aircraft (no wind, no GNSS degradation, no comms loss).
* Calibration targets steady mid-leg speed. For PX4, which keeps accelerating along a leg, it hunts
  between 8.4 and 8.7 m/s; fitting leg times (speed + overhead) is the better target for an autopilot.
* Shadow mode (replay) does not adopt the replans recorded in the log, so a flight replanned during the
  live run is judged against its original plan.
* Replanning only adds airborne holds. Rerouting and layer changes in the air come with hub
  sequencing (step 7) and the better scheduler.
* Prediction is checked every 10 s over 60 s. A conflict that develops in under 10 s is ORCA's job.
