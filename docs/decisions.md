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
