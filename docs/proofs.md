# Proofs and the verified checker

## What is proven (Lean 4.36.0, mathlib v4.36.0)

**Spec** (`proofs/UtmProofs/Spec.lean`, over ℝ). An `RBox` is an axis-aligned 4D box
[x0,x1]×[y0,y1]×[z0,z1]×[t0,t1]. `SepSpec a b buf` holds if the time windows are disjoint, or if on some
spatial axis one box ends at least `buf` before the other begins (6 axis cases + 2 time cases).

**Theorem 1** `sep_of_spec`: if `0 ≤ buf`, `SepSpec a b buf`, point p is in box a at time t and q is
in box b at the same time t, then `buf² ≤ ‖p − q‖²`. Squared distance avoids `sqrt`; it is the same
statement as `buf ≤ ‖p − q‖` because both sides are non-negative.

**Theorem 2** `sep_hv_of_spec` / `no_los_of_spec` (cylinder form, added in milestone 4). `SepHV a b h v`
holds if the time windows are disjoint, or the boxes are at least `h` apart along x or y, or at least
`v` apart along z. Then two points, one in each box at the same instant, satisfy
`h² ≤ horizontal distance²` or `v² ≤ vertical distance²`, i.e. they are **not in loss of separation**
(`LoS p q h v := hdist² < h² ∧ dz² < v²`). This is the LoS definition the simulator uses (30 m
horizontal, 10 m vertical). Theorem 1 with `buf = √(h² + v²)` also rules out LoS, but it needs that
gap along a single axis, so it can never certify two cruise layers 15 m apart. Theorem 2 can, and the
scheduler and audit use it.

**Theorem 3** `no_los_of_deviation` (deviation bounds, milestone 4.5). `RBox.inflate b eh ev δ` widens a
box by `eh` along x and y, `ev` along z and `δ` in time, on every side. `Tracks p t p0 t0 eh ev δ` says
the actual sample (p, t) is within `eh` per horizontal axis, `ev` vertically and `δ` in time of the
nominal sample (p0, t0). If `SepHV (a.inflate eh ev δ) (b.inflate eh ev δ) h v`, and each aircraft
tracks some point of its own nominal box within the bound, the two aircraft are not in loss of
separation. `mem_inflate_of_tracks` is the step from tracking to membership; `sepHV_inflate_iff` shows
the condition is exactly "nominal gaps of `h + 2 eh` horizontally, `v + 2 ev` vertically, or more than
`2 δ` in time". This is the sim-to-real statement: a planner's boxes plus a measured tracking-error
bound for the real autopilot give a certificate, and `scripts/fidelity_report.py` measures the bound
(per-axis error is the hypothesis, so a Euclidean horizontal bound `≤ eh` implies it).

**Theorem 4** (`proofs/UtmProofs/Registry.lean`, milestone 5: the registry protocol). A registry entry is a
box, its flight, and whether it was certified or forced (an emergency that could not be certified, a landing
after the holding limit: registered without the check so that later plans avoid it). `Inv h v r`: any two
certified boxes of different flights are `separatedHV`. `admit` registers a flight's candidate boxes as
certified only if each is `separatedHV` from every registered box (certified or forced) of every other flight;
`force` registers without the check; `withdraw` removes a flight; `prune` drops expired boxes. `inv_admit`,
`inv_force`, `inv_withdraw`, `inv_prune`: each preserves `Inv`; `inv_steps`: so does any sequence of them from
the empty registry; `registry_no_los` (with Theorem 2): two flights, each inside one of its certified
registered boxes at the same instant, are not in loss of separation. `separatedHV_comm` (symmetry) is the one
lemma the proof needs. This is a model of the protocol that the scheduler (`admit`, `replan`) and the hub
sequencer follow, not of their Python code, which is tested and audited instead.

**Checker** (`proofs/UtmChecker.lean`, Int only, no mathlib). `separated` and `inside` are the
functions compiled to C and exported as `utm_separated` and `utm_inside`; `separatedHV` is exported as
`utm_separated_hv`; `separatedDev a b h v eh ev dt` (`separatedHV` on `Box.inflate`d boxes) as
`utm_separated_dev` (ABI version 3).

**Refinement** (`proofs/UtmProofs/Refinement.lean`):

* `separated_sound`: `separated a b buf = true → SepSpec a.toR b.toR buf`.
* `inside_sound`: with margin `m ≥ 0`, `inside b m x y z t = true → b.toR.mem ⟨x,y,z⟩ t`.
* `checker_separation`: checker accepts + both points in their boxes ⇒ `buf² ≤ distSq`.
* `checker_separation_samples`: the same, with both positions given as integer samples accepted by
  `inside`.
* `separatedHV_sound`, `checker_no_los`, `checker_no_los_samples`: the same chain for Theorem 2.
  `checker_no_los_samples` is the statement `scripts/audit_run.py` and `scripts/audit_twin.py` rely on.
* `Box.inflate_toR` (integer widening commutes with the cast), `separatedDev_sound`,
  `checker_no_los_dev`: the chain for Theorem 3.

`proofs/UtmProofs/Axioms.lean` prints the axioms used: only `propext`, `Classical.choice`,
`Quot.sound` (no `sorry`, no extra axioms).

## Trust boundary (what is *not* proven)

1. **Int64 vs Int.** The theorems are about `Int`. The exported C functions take `Int64` and convert
   with `.toInt`, which is exact, and the arithmetic then runs on Lean's arbitrary-precision `Int`, so
   no overflow is possible inside the checker. `tests/test_checker_ffi.py::test_no_int64_overflow_false_positive`
   shows the case where naive int64 code would wrongly accept and the checker correctly rejects.
2. **The Lean compiler, the C compiler, the Lean runtime and ctypes** are trusted.
3. **Float → integer export.** `Box4.as_int` rounds every box *outwards* to mm/ms, so the integer box
   contains the float box. Position samples are rounded to the nearest mm/ms; with margin 0 this can
   misclassify a sample within 0.5 mm of a face. Use margin ≥ 1 mm when that matters.
4. **Conformance is an assumption of Theorems 1–3, not a consequence.** The theorem says nothing about a
   flight that leaves its intent. The audit therefore checks conformance on every logged sample,
   reports it separately, and counts an LoS event between a certified pair as a counterexample only if
   both flights conformed during it (otherwise `certified_but_nonconforming`). ORCA keeps flights
   inside a 15 m corridor for exactly this reason.
5. **Twin estimates.** The twin's online monitor judges estimates (filtered, noisy telemetry), not truth.
   An estimate below ground is clamped to 0 (a parked vehicle's estimator reads a few cm negative).
   The offline twin audit re-judges conformance on the plant's ground truth.
6. **Plan versions.** A replan withdraws a flight's boxes and certifies new ones from the replan time
   on. The twin audit checks every pair of versions over the window in which both were in force.
7. **Between samples.** Logs are at 2 Hz; the theorem covers positions that are in the box. Saying the
   vehicle stayed inside *between* samples needs a speed bound (future theorem 2: box margin ≥ v·Δt/2).
8. **Assumed thresholds.** 30 m / 10 m LoS and the 32 m buffer are project assumptions.

## Building

Docker (recommended, uses the mathlib cache):

```bash
docker compose -f docker/compose.yaml build proofs
docker compose -f docker/compose.yaml run --rm proofs
```

This runs `lake exe cache get`, `lake build`, prints the axioms, builds `runtime/build/libutm_checker.so`
and `.a`, runs the C smoke test against both, and runs the FFI tests (20 000 random cases agree with the
Python reference, extreme values, sampled soundness).

Natively: install elan, then `cd proofs && lake exe cache get && lake build`, then `make -C runtime test`.
Without the cache, mathlib's needed subset builds from source in about an hour.

## Library flavours

* `libutm_checker.so`: for Python/ctypes. Links the toolchain's `libleanshared.so` (Lean's static
  runtime uses local-exec TLS and cannot go into a shared object).
* `libutm_checker.a`: for C/C++ executables, e.g. a ROS 2 monitor node; link statically as `runtime/Makefile`
  does for `utm_smoke`.

C API (`runtime/utm_checker.h`, ABI 3): call `utm_init()` once, then `utm_check_separated`,
`utm_check_separated_hv`, `utm_check_separated_dev`, `utm_check_inside`, `utm_count_conflicts`,
`utm_count_conflicts_hv`, `utm_count_against_hv` (candidate boxes against an accepted set, used by the
scheduler's verified search), `utm_count_against_dev`. Boxes are `int64_t[8]` = x0 x1 y0 y1 z0 z1 t0 t1 in mm and ms.

## Next theorems

* **Per-flight error bounds:** Theorem 3 with a different `(eh, ev, δ)` per aircraft (a degraded vehicle
  has a larger timing error than the fleet), and `δ` growing with time since release, as the intents'
  slack does.
* **Sampling:** if consecutive samples are inside a box shrunk by v_max·Δt/2, the continuous
  trajectory between them is inside the box.
* **Scheduler invariant, extracted:** Theorem 4 proves the registry protocol. Exporting `admit` itself through
  the C ABI and calling it from the scheduler would put the protocol's implementation, not only its model,
  under the proof.
* **ORCA:** a reciprocal-avoidance step keeps the predicted pairwise distance ≥ r over the horizon τ
  (as an invariant over ℝ, without the executable part).
