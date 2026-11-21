# Milestone 4.5 results: the digital twin

All runs on `mini_town`, ORCA + FCFS scheduler unless noted, verified checker online (`--verified`).
Scenario (`configs/twin.yaml`, an assumption chosen to give the twin something to detect): the plant's
fleet is limited to 8.5 m/s while the planner assumes 10 m/s cruise, one airborne vehicle drops to
4 m/s at t = 600 s, telemetry has 0.7 m / 0.5 m noise and 0.25 s latency. "Matched plant" removes the
mismatch and the fault. Ground truth comes from the plant and is used only for scoring and the audit.

## Tier A plant: totals over seeds 1-3

| Demand | Configuration | Flights | Conformance (truth) | Non-conformance alerts | Landing overruns | Replans (contingent) | 60 s prediction error, median / p95 | En-route LoS | Verified audit |
|---|---|---|---|---|---|---|---|---|---|
| x1 | matched plant | 106 | 1.0000 | 0 | 0 | 0 (0) | 0.33 / 2.5 m | 0 | 0 violations, 0 counterexamples |
| x1 | open loop | 106 | 0.9813 | 7 | 39 | 0 (0) | 5.68 / 50.8 m | 0 | 0 / 0 |
| x1 | **full twin** | 106 | **0.9992** | 2 | 0 | 1 (1) | **0.36 / 4.7 m** | 0 | 0 / 0 |
| x2 | matched plant | 205 | 0.9998 | 2 | 0 | 0 (2) | 0.35 / 2.7 m | 1 | 0 / 0 |
| x2 | open loop | 205 | 0.9809 | 16 | 71 | 0 (0) | 4.46 / 49.2 m | 1 | 0 / 0 |
| x2 | replan only | 205 | 0.9928 | 10 | 32 | 5 (5) | 4.43 / 49.2 m | 1 | 0 / 0 |
| x2 | calibrate only | 205 | 0.9947 | 7 | 15 | 0 (0) | 0.38 / 3.0 m | 1 | 0 / 0 |
| x2 | **full twin** | 205 | **0.9989** | 5 | 3 | 2 (3) | **0.38 / 3.0 m** | 1 | 0 / 0 |

Medians and p95 are means over seeds. Per-run table below; figure `docs/figures/milestone4_5_twin_tier_a.png`.

* **Closing the loop recovers almost all the conformance the mismatch costs.** Open loop, a 15 %
  slower fleet leaves 1.9 % of position samples outside their intents and 39-71 flights overrun their
  landing window. The full twin is within 0.1 % of the matched plant.
* **Calibration and replanning do different jobs.** Calibration learns the fleet's speed 90-270 s into
  the run (8.48-8.55 m/s in every run, the median of per-vehicle medians) and fixes every flight planned
  afterwards; it also fixes prediction (60 s p95 error 49 -> 3 m). Replanning rescues the flights that
  were planned before calibration and are already in the air, and the degraded vehicle. Either alone
  leaves 0.5-0.7 % of samples non-conforming; together, 0.1 %.
* **The degraded vehicle.** In 3 of the 6 full-twin runs the twin flagged it 35-37 s after the fault,
  measured its own speed (3.96-4.03 m/s) and replanned it once with that speed; it conformed again
  2 s later. In the other 3 the fault hit a vehicle on its last short leg home or already descending to
  its pad: the delay stayed inside its timing slack, it never left its intent, and the twin did nothing.
* **Replans.** 1 of 2 replans at x1 and 2 of 5 at x2 certified against everyone else; the rest flew
  `contingent`, counted and registered so that later admissions avoid them.
* **Prediction.** A 60 s Tier A rollout forked from the twin's state every 10 s predicts positions to a
  0.3-0.4 m median once calibrated. In the prediction benchmark (no strategic deconfliction, no ORCA,
  matched plant, x2) it predicted all 24 en-route LoS events at least 10 s ahead, median lead 49-54 s
  (the horizon is 60 s), precision 0.67-0.82. Of the 7 predicted events that did not match, 5 were real
  LoS between the same pair that the plant classified as terminal (it started inside the 40 m hub
  radius), and 2 were near misses that bottomed out just outside the thresholds (30.4 m / 10.3 m and
  31.5 m / 11.6 m). The one en-route LoS at x2 seed 3 (an emergency that could not be certified,
  as in milestone 4) was predicted 52-60 s ahead in every configuration.
* **Verified audit: 27 runs, 0 claim violations, 0 counterexamples.** Every pair of plan versions in
  force together that the online scheduler claimed to certify (517 claims at x1, 2,275 at x2 for the
  full twin) is confirmed by the Lean checker, and no ground-truth sample has two mutually certified
  flights in loss of separation inside en-route intents. Truth conformance matches the twin's own
  estimate-based conformance to within 0.0004 in every run.
* Speed: the twin runs 22-64x faster than real time with Tier A as the plant (one CPU core).

## PX4 SITL plant (Tier B): 3 deliveries, real time, two sessions

PX4 v1.16.2, SIH simulator, one instance per vehicle, offboard velocity setpoints from the shared
guidance law, PX4 LAND mode at the pad. Session 1 starts from the Tier A model. Session 2 flies the
same three missions with `--prior` = session 1's calibration (`docs/figures/milestone4_5_px4.png`).

| Session | Conformance twin / truth | Alerts (landing) | Theorem 3 premise at intent slack | p99 tracking error at 5 s: h / v | Lift-off lag | Cruise leg - plan | 60 s prediction median / p95 | Audit |
|---|---|---|---|---|---|---|---|---|
| 1 (Tier A model) | 0.9353 / 0.9440 | 2 (2) | 0.839 | 102.7 / 15.3 m | 3.94 s | +4.9 s | 3.3 / 43.4 m | CONSISTENT |
| 2 (session 1 prior) | **1.0000 / 1.0000** | 0 (0) | **1.000** | **3.5 / 0.36 m** | 3.97 s | +0.8 s | 2.3 / 15.5 m | CONSISTENT |

**Independent reproduction** (Docker `px4` image on a Windows ThinkPad, WSL2; same seed and missions;
session 2 from that machine's own session 1 calibration: 8.65 m/s cruise, 1.52 m/s descent, 3.93 s
lift-off lag; both sessions audited CONSISTENT):

| Session | Conformance twin / truth | Alerts (landing) | Theorem 3 premise at intent slack | p99 tracking error at 5 s: h / v | Lift-off lag | Cruise leg - plan | 60 s prediction median / p95 |
|---|---|---|---|---|---|---|---|
| 1 (Tier A model) | 0.9338 / 0.9460 | 3 (2) | 0.764 | 107.2 / 18.0 m | 4.03 s | +5.1 s | 2.7 / 35.7 m |
| 2 (session 1 prior) | 1.0000 / 1.0000 | 0 (0) | 1.000 | 3.2 / 0.36 m | 3.83 s | +0.6 s | 2.4 / 14.1 m |

Calibrated, the two machines agree to within a few tenths of a metre and second. Uncalibrated, they
differ more (premise 0.76 vs 0.84), because the misses depend on how timing errors accumulate.

What the fidelity report found about PX4 against the Tier A model:

* **Take-off lag, 3.9 s.** Even pre-armed, PX4 lifts off ~3.9 s after the release time (spool-up plus
  the `MPC_TKO_RAMP_T` = 3 s take-off ramp). Tier A climbs at once. This alone uses most of the 5 s
  timing slack and is the largest single source of session 1's misses. It is now a model parameter
  (`IntentParams.takeoff_s`) that the twin calibrates (3.94 s from three flights).
* **Slow speed convergence.** Commanded 10 m/s, PX4's velocity loop with SIH drag reaches 8-9 m/s and
  is still creeping up at the end of a 290 m leg (`docs/figures/milestone4_5_px4_session1_fidelity.png`);
  Tier A reaches 10 m/s in ~2 s. Cruise legs take 4.9 s longer than planned; the calibrated mid-leg speed
  (8.7 m/s) brings that to 0.8 s. A leg-time fit gives 9.0-9.2 m/s plus 3.4-4.3 s.
* **Landing takes 8-9 s longer than planned** (PX4 LAND slows to its landing speed near the ground and
  waits for the land detector). With the take-off lag in the model the whole schedule shifts by 4 s and
  the landing overruns disappear in session 2; this is hub sequencing territory (step 7) anyway.
* **Climb and descent rates match** (climb legs +0.25 s, drop descents within 0.1 s).
* **Lateral tracking is excellent.** Once the timing is right (session 2), PX4 stays within 3.5 m
  horizontally and 0.36 m vertically of the plan (p99, delta = 5 s), so the 20 m lateral buffer has
  ~5x headroom for this autopilot. Theorem 3 makes that a certificate: plans certified with boxes
  widened by the measured (eh, ev, delta) are safe for this autopilot.
* Whole missions take 23-24 s longer on PX4 than in Tier A re-flying the same plan (8-14 %, session 1).

Limitations: three vehicles, one seed, a 2-core sandbox (real time, so no speed-up), PX4's SIH model
(not Gazebo physics, no wind, no GNSS degradation). Session 1's calibration hunts between 8.4 and
8.7 m/s because PX4 keeps accelerating along a leg, so the mid-leg median depends on leg length; a
leg-time fit would be the better calibration target for an autopilot like this.

## Replay (shadow mode)

`--plant replay` re-runs the twin on a recorded log: the same 31 admissions, conformance 0.9888 (lower
than live because shadow mode cannot apply its replans, so a replanned flight is judged against its
original plan), prediction median 0.75 m. Commands are recorded and ignored (31).

## Per-run table (Tier A plant)

| scale | seed | configuration | requests (rejected) | conformance twin / truth | alerts (landing) | replans (contingent) | cruise model m/s | en-route LoS (terminal) | pred. error 60 s median / p95 m | LoS recall (lead >= 10 s) / precision, median lead s | audit |
|---|---|---|---|---|---|---|---|---|---|---|---|
| x1 | 1 | matched plant | 31 (0) | 1.0000 / 1.0000 | 0 (0) | 0 (0) | 10.00 | 0 (19) | 0.34 / 2.4 | - | 0 viol, 0 cex |
| x1 | 2 | matched plant | 37 (0) | 1.0000 / 1.0000 | 0 (0) | 0 (0) | 10.00 | 0 (19) | 0.29 / 2.6 | - | 0 viol, 0 cex |
| x1 | 3 | matched plant | 38 (0) | 1.0000 / 1.0000 | 0 (0) | 0 (0) | 10.00 | 0 (23) | 0.35 / 2.4 | - | 0 viol, 0 cex |
| x1 | 1 | full twin | 31 (0) | 0.9997 / 0.9998 | 1 (0) | 1 (0) | 8.49 | 0 (20) | 0.36 / 5.6 | - | 0 viol, 0 cex |
| x1 | 2 | full twin | 37 (0) | 0.9978 / 0.9979 | 1 (0) | 0 (1) | 8.53 | 0 (13) | 0.27 / 4.6 | - | 0 viol, 0 cex |
| x1 | 3 | full twin | 38 (0) | 0.9999 / 0.9999 | 0 (0) | 0 (0) | 8.48 | 0 (20) | 0.44 / 3.8 | - | 0 viol, 0 cex |
| x1 | 1 | open loop | 31 (0) | 0.9775 / 0.9777 | 10 (9) | 0 (0) | 10.00 | 0 (19) | 5.71 / 54.3 | - | 0 viol, 0 cex |
| x1 | 2 | open loop | 37 (0) | 0.9745 / 0.9747 | 26 (23) | 0 (0) | 10.00 | 0 (17) | 6.38 / 51.2 | - | 0 viol, 0 cex |
| x1 | 3 | open loop | 38 (0) | 0.9913 / 0.9916 | 10 (7) | 0 (0) | 10.00 | 0 (24) | 4.94 / 46.9 | - | 0 viol, 0 cex |
| x2 | 1 | matched plant | 65 (0) | 0.9994 / 0.9994 | 2 (0) | 0 (2) | 10.00 | 0 (83) | 0.35 / 3.1 | - | 0 viol, 0 cex |
| x2 | 2 | matched plant | 76 (0) | 1.0000 / 1.0000 | 0 (0) | 0 (0) | 10.00 | 0 (101) | 0.35 / 2.5 | - | 0 viol, 0 cex |
| x2 | 3 | matched plant | 64 (0) | 1.0000 / 1.0000 | 0 (0) | 0 (0) | 10.00 | 1 (85) | 0.35 / 2.5 | 1.00 (1.00) / 1.00, 60 | 0 viol, 0 cex |
| x2 | 1 | full twin | 65 (0) | 0.9996 / 0.9997 | 2 (0) | 0 (2) | 8.51 | 0 (87) | 0.31 / 2.9 | - | 0 viol, 0 cex |
| x2 | 2 | full twin | 76 (0) | 0.9989 / 0.9988 | 1 (0) | 1 (0) | 8.49 | 0 (106) | 0.37 / 2.9 | - | 0 viol, 0 cex |
| x2 | 3 | full twin | 64 (0) | 0.9981 / 0.9981 | 5 (3) | 1 (1) | 8.55 | 1 (94) | 0.45 / 3.3 | 1.00 (1.00) / 1.00, 52 | 0 viol, 0 cex |
| x2 | 1 | replan only | 65 (0) | 0.9906 / 0.9907 | 21 (16) | 1 (4) | 10.00 | 0 (74) | 4.55 / 47.1 | - | 0 viol, 0 cex |
| x2 | 2 | replan only | 76 (0) | 0.9928 / 0.9932 | 11 (9) | 2 (0) | 10.00 | 0 (101) | 4.00 / 49.8 | - | 0 viol, 0 cex |
| x2 | 3 | replan only | 64 (0) | 0.9943 / 0.9944 | 10 (7) | 2 (1) | 10.00 | 1 (85) | 4.73 / 50.7 | 1.00 (1.00) / 1.00, 59 | 0 viol, 0 cex |
| x2 | 1 | calibrate only | 65 (0) | 0.9938 / 0.9938 | 3 (0) | 0 (0) | 8.51 | 0 (87) | 0.31 / 2.9 | - | 0 viol, 0 cex |
| x2 | 2 | calibrate only | 76 (0) | 0.9978 / 0.9977 | 8 (7) | 0 (0) | 8.49 | 0 (106) | 0.37 / 2.9 | - | 0 viol, 0 cex |
| x2 | 3 | calibrate only | 64 (0) | 0.9925 / 0.9925 | 11 (8) | 0 (0) | 8.55 | 1 (94) | 0.45 / 3.3 | 1.00 (1.00) / 1.00, 52 | 0 viol, 0 cex |
| x2 | 1 | open loop | 65 (0) | 0.9663 / 0.9664 | 36 (28) | 0 (0) | 10.00 | 0 (73) | 4.64 / 46.9 | - | 0 viol, 0 cex |
| x2 | 2 | open loop | 76 (0) | 0.9904 / 0.9908 | 25 (23) | 0 (0) | 10.00 | 0 (101) | 4.00 / 49.8 | - | 0 viol, 0 cex |
| x2 | 3 | open loop | 64 (0) | 0.9852 / 0.9855 | 26 (20) | 0 (0) | 10.00 | 1 (85) | 4.74 / 50.7 | 1.00 (1.00) / 1.00, 59 | 0 viol, 0 cex |
| x2 | 1 | prediction benchmark | 65 (0) | 1.0000 / 1.0000 | 0 (0) | 0 (0) | 10.00 | 9 (84) | 0.32 / 4.3 | 1.00 (1.00) / 0.82, 54 | 0 viol, 0 cex |
| x2 | 2 | prediction benchmark | 76 (0) | 1.0000 / 1.0000 | 0 (0) | 0 (0) | 10.00 | 6 (107) | 0.30 / 2.6 | 1.00 (1.00) / 0.67, 49 | 0 viol, 0 cex |
| x2 | 3 | prediction benchmark | 64 (0) | 1.0000 / 1.0000 | 0 (0) | 0 (0) | 10.00 | 9 (77) | 0.30 / 2.7 | 1.00 (1.00) / 0.82, 53 | 0 viol, 0 cex |
