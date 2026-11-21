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

**Checker** (`proofs/UtmChecker.lean`, Int only, no mathlib). `separated` and `inside` are the
functions compiled to C and exported as `utm_separated` and `utm_inside`; `separatedHV` is exported as
`utm_separated_hv` (ABI version 2).

**Refinement** (`proofs/UtmProofs/Refinement.lean`):

* `separated_sound`: `separated a b buf = true → SepSpec a.toR b.toR buf`.
* `inside_sound`: with margin `m ≥ 0`, `inside b m x y z t = true → b.toR.mem ⟨x,y,z⟩ t`.
* `checker_separation`: checker accepts + both points in their boxes ⇒ `buf² ≤ distSq`.
* `checker_separation_samples`: the same, with both positions given as integer samples accepted by
  `inside`.
* `separatedHV_sound`, `checker_no_los`, `checker_no_los_samples`: the same chain for Theorem 2.
  `checker_no_los_samples` is the statement `scripts/audit_run.py` relies on.

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
4. **Conformance is an assumption of Theorem 1, not a consequence.** The theorem says nothing about a
   flight that leaves its intent. The audit therefore checks conformance on every logged sample,
   reports it separately, and counts an LoS event between a certified pair as a counterexample only if
   both flights conformed during it (otherwise `certified_but_nonconforming`). ORCA keeps flights
   inside a 15 m corridor for exactly this reason.
5. **Between samples.** Logs are at 2 Hz; the theorem covers positions that are in the box. Saying the
   vehicle stayed inside *between* samples needs a speed bound (future theorem 2: box margin ≥ v·Δt/2).
6. **Assumed thresholds.** 30 m / 10 m LoS and the 32 m buffer are project assumptions.

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

C API (`runtime/utm_checker.h`, ABI 2): call `utm_init()` once, then `utm_check_separated`,
`utm_check_separated_hv`, `utm_check_inside`, `utm_count_conflicts`, `utm_count_conflicts_hv`,
`utm_count_against_hv` (candidate boxes against an accepted set, used by the scheduler's verified search). Boxes are `int64_t[8]` = x0 x1 y0 y1 z0 z1 t0 t1 in mm and ms.

## Next theorems

* **Sampling:** if consecutive samples are inside a box shrunk by v_max·Δt/2, the continuous
  trajectory between them is inside the box.
* **Scheduler invariant:** the FCFS scheduler only admits an intent set whose pairs are all
  `separatedHV` (except emergencies it flags as uncertified), so with conformance the whole fleet
  stays out of LoS en route. Today this is checked after the fact by the audit and by
  `tests/test_scheduler.py`; the proof would be over the accept loop.
* **ORCA:** a reciprocal-avoidance step keeps the predicted pairwise distance ≥ r over the horizon τ
  (as an invariant over ℝ, without the executable part).
