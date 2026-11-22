# Decisions log

**S1. Two tiers behind one `VehicleAPI`.** PX4 SITL cannot run hundreds of vehicles on a laptop, and
step 8 needs hundreds. Tier A (kinematic, PX4 limits) does the statistics; Tier B (PX4 + Gazebo)
validates a subset.

**S2. Reuse the verified MOT Docker images.** `eval` (python:3.13-slim) and `sim` (Ubuntu 26.04 + Jetty
via `infra/install_stack.sh`) are unchanged except for requirements and paths. Tier B will use
Ubuntu 24.04, because that is the pair PX4 supports with Jetty.

**S3. Operational intents as axis-aligned 4D boxes.** This follows ASTM F3548-style volumes and keeps
the checker to linear integer comparisons, which makes the proofs short and the C code trivial. The
cost is conservatism on diagonal legs, reduced by splitting legs into ≤ 100 m pieces.

**S4. Integers at the boundary (mm, ms), boxes rounded outwards.** The checker is exact on `Int`; floats
never enter verified code.

**S5. Squared distance in the spec.** This avoids `Real.sqrt` and keeps the proof to `nlinarith` on squares.

**S6. Checker without mathlib, spec with mathlib.** The compiled library depends only on Lean core
and the runtime; mathlib is needed only to check the proofs.

**S7. Two library flavours.** Lean's static runtime uses local-exec TLS, so the shared library links
`libleanshared.so` and the static archive is for executables.

**S8. LoS = 30 m horizontal and 10 m vertical; terminal LoS counted separately.** These are working
assumptions, not a standard. Hub-area conflicts belong to step 7.

**S9. Conformance is audited, not assumed.** Theorem 1 needs it, so every run reports it.

**S10. Semicircular cruise layers.** The layer depends on track angle (base 40 m, 15 m step since S11),
so opposite headings are vertically separated by construction before any policy runs.

**S11. Layer step 15 m, not 10 m.** With 10 m, opposite layers were exactly at the 10 m vertical LoS
threshold, so millimetres of altitude-tracking error produced "LoS" events between flights that were
nominally separated (32 of the 36 en-route events in the seed-1 ×1 baseline). 15 m leaves 5 m for altitude-hold error,
which real PX4 needs too. This changed the baseline numbers reported before milestone 3.

**S12. ORCA in the horizontal plane with a vertical gate.** PX4 vertical limits are 4–8× below the
horizontal ones, and the layers already separate vertically. The gate (13 m) must stay below the
layer step.

**S13. Conformance beats tactical avoidance.** The corridor is a hard constraint, plus a tangential
clip around points. When avoidance and the corridor conflict, the vehicle stays in its intent and the
loss of separation is recorded. Otherwise the strategic guarantee (Theorem 1) would silently lose its
premise.

**S14. Right of way instead of endless waiting.** A vehicle without progress for 10 s gets right of
way (priority, then index): it drops the constraint and the other vehicle takes the whole avoidance.
The rule is symmetric, so a pair always agrees. Overlapping intents then show up as recorded LoS events
for the scheduler to remove, not as 250 s hidden delays.

**S15. Audit counterexamples require conformance.** An LoS event between a certified pair counts
against Theorem 1 only if both flights were inside their intents at the time; otherwise it is
reported as `certified_but_nonconforming`, since the theorem's premise failed.

**S16. Theorem 2 (cylinder form) for scheduling and audit.** Theorem 1 needs the 3D buffer along one
axis, so certifying two layers 15 m apart would need boxes about 32 m apart vertically. Theorem 2
matches the LoS definition (30 m horizontal or 10 m vertical) and certifies adjacent layers. Both
theorems stay in the library; the ABI went to version 2.

**S17. Strategic deconfliction covers en-route boxes only.** Boxes within 40 m of the pad (take-off
and landing columns) are terminal, left to step 7. The audit reports certification over all boxes
too, and classifies LoS events where a flight was only in terminal boxes separately.

**S18. Intent tightening.** The changes and reasons are in the table in architecture.md. Each was
driven by a measured cause of ground delay, not by tuning to a target.

**S19. Emergencies are never ground-delayed.** If they can't be certified, they fly uncertified and
are counted. Every en-route LoS event left under the scheduler involves one of them. Next: pre-empt
lower-priority flights instead.

**S20. Reject rather than fly uncertified (non-emergency).** After 900 s without a certified plan,
a delivery or inspection is rejected. Throughput lost is reported as rejections.

**S21. The search is untrusted; the audit is not.** The scheduler searches with a vectorised Python
reference for speed. Every accepted plan is re-checked by the Lean-verified checker in the audit, and
a test checks that a verified-checker search makes identical decisions.

## Milestone 4.5: digital twin

**S22. The twin never reads a simulator's internals.** It sees timestamped, noisy, late `Telemetry`
and acts through `dispatch` / `set_route` / `cancel`. Tier A, PX4 and a replayed log are interchangeable
plants. Ground truth stays inside sim plants and is used only to score the twin afterwards.

**S23. Tier A as the stand-in plant, with a deliberate mismatch.** The plant's fleet is limited to
8.5 m/s while the planner assumes 10 m/s cruise (a payload variant or a configuration mismatch), one
vehicle drops to 4 m/s at t = 600 s, and telemetry has 0.7 m / 0.5 m noise and 0.25 s latency. This
is a scenario assumption (`configs/twin.yaml`), chosen so that the twin has something to detect,
learn and correct. `--no-faults` gives the matched control.

**S24. Tier B is PX4 SIH, flown with the same guidance as Tier A.** PX4 v1.16.2's built-in simulator
needs no Gazebo and runs headless and faster than real time. The bridge runs the shared waypoint logic
(`sim.follow`) and guidance law and sends PX4 offboard velocity setpoints, so a Tier A vs Tier B
difference is the autopilot (controllers, estimator, SIH dynamics), not the mission logic. PX4's LAND
mode does the final touchdown. Gazebo stays for rendering and sensors (milestone 6).

**S25. Conformance online, on estimates, with the verified checker.** The monitor uses the Lean
`inside` on filtered estimates (2 s to flag, 2 s to clear, to ride out noise at box faces). An estimate
below ground is clamped to 0. The offline audit re-judges conformance on ground truth.

**S26. Replan = withdraw, then re-certify from the measured position.** Airborne holds of 0–120 s are
tried until the new boxes certify against every other flight's boxes in force; otherwise the flight is
`contingent` (flown, registered, counted). If still non-conforming after the 20 s cooldown, the twin
tries again. Not replanned: the final descent at the pad (`landing_overrun`, hub sequencing).

**S27. Calibration is one vote per vehicle.** Fleet speeds are the median of per-vehicle medians over
the 20 most recent vehicles (at least 3). A first version used one pooled median, and in light traffic a
single degraded vehicle pulled the fleet model to 4 m/s, after which every healthy flight ran early
against its plan. A vehicle measurably slower than the model (> 3 %) is replanned and predicted with its
own speed. Lift-off lag (release to 0.5 m) is calibrated the same way when it exceeds 1 s.

**S28. Prediction forks Tier A from the twin's state.** Positions, velocities, waypoint progress,
holds, per-vehicle speed caps and imminent take-offs seed a 60 s rollout every 10 s with the same
separation policy. Scored against what happens: position error at +10/30/60 s, and LoS recall
(with lead time) and precision against the plant's en-route LoS events. The prediction benchmark
disables strategic deconfliction (`--admit-all`) and ORCA, otherwise there are no conflicts to predict.

**S29. Plan versions are the unit of audit.** Every admission and replan registers a version with its
validity window. Two versions in force together were checked online by whichever came later; the
audit re-checks each such claim with the verified checker over the shared window, then checks every
ground-truth sample.

**S30. Theorem 3 (deviation bounds) makes the fidelity gap a certificate.** Measured tracking error
(eh, ev, δ) of a real autopilot against its plan, plus cylinder separation of the plan boxes widened by
that error, implies no loss of separation (`no_los_of_deviation`, checker `separatedDev`, ABI 3).

**S31. MCAP for the log, Rerun for 3D, both optional at the edges.** MCAP is rosbag2's storage format,
readable without ROS (and by Foxglove). The Rerun SDK is 175 MB, so it is not in the images: export runs
anywhere `rerun-sdk` is installed, including natively on Windows.

**S32. PX4's take-off ramp is a model parameter, not a bridge hack.** Lift-off comes ~3.9 s after the
release time even when the vehicle is pre-armed (PX4 `MPC_TKO_RAMP_T` = 3 s plus spool-up), which uses
most of the 5 s timing slack. `IntentParams.takeoff_s` (default 0, so milestone 4 is unchanged) holds
the pad for that long; the twin calibrates it, and `--prior` starts a session from an earlier one's
calibration.

## Milestone 5: hub sequencing

**S33. Under 30 m / 10 m, a compact hub is one runway.** Every pad of both mini_town hubs is within 30 m
of every other (8 m spacing), so two simultaneous operations at a hub are a loss of separation unless
they are 10 m apart vertically. The milestone does not relax the standard inside the terminal area;
it sequences the hub, and measures what a spread layout (40 m pads) buys.

**S34. Departures strategic, arrivals tactical.** A departure's pad climb is known at admission with a
5 s slack, so the scheduler certifies it with the rest of the plan (and may pick another pad). An
arrival's timing is known only near the end of a flight (slack 5 s + 2 s/min), so certifying it at
admission would reserve the hub for minutes. Inbound flights end their strategic plan at an approach
fix 100 m out on the inbound bearing and ask the hub sequencer for a landing slot ~20 s before it.

**S35. Holding on a ring, descending outside the hub.** Holding points sit on the ring every 60 deg at
25 m or 40 m. The ring radius must keep a holding box (20 m buffer) 30 m clear of every pad column (5 m
buffer): farthest pad + 55 m, so 66 m for the compact hubs but 100 m for the spread layout, whose outer
pads are 45 m out. A first run with a 70 m ring had a quarter of the spread layout's arrivals fail,
because every descent on the ring conflicted with departures from the outer pads. All milestone-5 runs
use 100 m, so every layout flies the same missions. Arrivals
descend on the ring and cross to the pad at 25 m: a landing occupies the hub column for ~17 s instead of
30-80 s from the cruise layers (descent is 1.5 m/s). Every candidate route is checked for building
clearance.

**S36. Terminal boxes use a 5 m lateral buffer.** The 20 m en-route buffer made a pad column 40 m wide,
so no two pads (even 32 m apart) could ever operate together, and ring holding points 36 deg apart
conflicted. PX4 tracked its plan within 3.5 m (p99) once calibrated (milestone 4.5), and Theorem 3 is
what licenses sizing buffers from measured tracking error. Default unchanged (milestone 4 reproduces).

**S37. Arrival priority over ground departures.** A drone on the ground waits for free; one in the air
burns battery. If no landing slot needs at most 60 s of holding, departures not yet airborne (and not
within 5 s of lift-off, and not emergencies) may be moved: the sequencer ignores their bookings, and the
twin re-admits each displaced departure for a later slot.

**S38. Never fly into the hub uncertified while there is battery to wait.** A flight with no slot at the
fix holds there (outside the terminal area, registered so later plans avoid it) and retries every 5 s; it
lands uncertified (`contingent`) only after 300 s in total. Early versions gave up after one search and
flew the strategic approach into a busy hub, which caused most of the remaining terminal LoS.

**S39. Theorem 4: the registry protocol keeps the fleet separated.** Withdraw, check against every other
flight with `separatedHV`, register as certified only if all pass; force-register what must fly anyway;
prune expired boxes. Proved in Lean that any sequence of these keeps every pair of certified boxes from
different flights cylinder-separated, so with Theorem 2, two flights inside certified boxes are never in
LoS. A first version of the model had no forced entries, which the Python registry has (uncertified
emergencies, contingent landings); the model was extended rather than the claim left too strong. The Python implementation follows the protocol and
is tested and audited, not extracted.

## Milestone 5.5: landing reservations

**S40. Why arrivals went contingent: they were never promised a landing.** Instrumenting every failed final
attempt at x2 (compact seed 1, spread seed 2) showed that none failed after holding: all 20 went contingent at
their first attempt at the fix, with no landing slot and no free holding box anywhere on the ring. The
blocking boxes were departures climbing out through the arrival's corridor and flights inbound on the same
bearing, all certified at admission against a plan whose landing was not. Under S34 a departure is admitted
against the airspace an arrival will need minutes later without knowing it, so by the time the arrival asks,
the terminal area can be full. Holding stacks or longer leads would not fix that; the arrival has to be part of
the strategic plan.

**S41. Certify the planned arrival at admission as a reservation (`terminal="reserve"`).** The whole plan,
approach fix to pad, is certified when the request is admitted, with the usual growing time slack; no flight
is launched without a landing it can fly. The sequencer still looks for a tactical slot ~20 s before the fix
(tighter windows, other pads and holding points). If none certifies, the flight keeps its reservation
(`reserved`) instead of holding or landing uncertified. Reservation rows are marked in the registry so that
an emergency can displace them; the displaced flight falls back to tactical sequencing (S34) and a new plan
version records that its arrival is no longer certified. This reverses S34's choice for arrivals: at x2/x5 the
cost in reserved hub time, which S34 expected, is real (below) and is the price of having no contingent
arrivals. `terminal="sequence"` (milestone 5) is unchanged and reproduces bit for bit.

**S42. Tactical plans start now.** A plan made from the measured position at time `now` only claims the
airspace from `now`; its first box used to extend 5 s of slack into the past and "conflict" with flights that
had already passed. With reservations, tactical boxes are clipped to start at `now`. The audit already checks
each version only from its `valid_from`; versions are in force on `[valid_from, valid_to)`, and the audit now
treats that interval as half-open (a flight that landed at t and one admitted at t were counted as concurrent
at the single instant t).

**S43. Emergency priority, bounded.** An emergency that cannot be certified as filed may move departures still
on the ground (as arrivals may, S37), displace the landing reservations of airborne flights that have not yet
started sequencing, and wait up to 30 s on the ground (`priority_max_delay_s`; emergencies otherwise never
wait, S19). If that still certifies nothing it flies uncertified as before. The displacement is modelled in
Theorem 4 as a removal (`drop`) followed by an ordinary checked admission, which is what the implementation's
result equals.

**S44. Rejected options.** Periodically re-certifying conforming flights from their measured state (tighter
slack on the rest of the plan) added 2-8 % throughput at x5 in a one-seed trial and was dropped. Admitting
without a reservation when none exists within the ground-delay limit (`reserve_fallback`) is kept as an
option, not the default: it recovers throughput but brings back flights that can go contingent.
