import UtmProofs.Spec
import UtmChecker

/-!
# Refinement: the integer checker implies the ℝ specification

`Box.toR` casts the checker's integer volume (mm, ms) to ℝ. Then:

* `separated_sound` — `separated a b buf = true → SepSpec a.toR b.toR buf`
* `inside_sound`    — `inside b m x y z t = true` with `m ≥ 0` → the sample is in `b.toR`
* `checker_separation` — composing both with Theorem 1: if the checker accepts
  the pair of volumes, and both aircraft positions conform at time `t`, they
  are at least `buf` apart at `t`.

What runs in C is exactly `Utm.separated` / `Utm.inside` (`UtmChecker`), so
these theorems are about the deployed code, up to the `Int64 → Int`
conversion at the FFI boundary (`Int64.toInt`, exact).
-/

namespace Utm

/-- Integer volume (mm, ms) as a real volume in the same units. -/
def Box.toR (b : Box) : RBox :=
  ⟨b.x0, b.x1, b.y0, b.y1, b.z0, b.z1, b.t0, b.t1⟩

theorem separated_sound {a b : Box} {buf : Int} (h : separated a b buf = true) :
    SepSpec a.toR b.toR (buf : ℝ) := by
  simp only [separated, Bool.or_eq_true, decide_eq_true_eq] at h
  simp only [SepSpec, Box.toR]
  rcases h with ((((((h | h) | h) | h) | h) | h) | h) | h
  · exact Or.inl (by exact_mod_cast h)
  · exact Or.inr (Or.inl (by exact_mod_cast h))
  · exact Or.inr (Or.inr (Or.inl (by exact_mod_cast h)))
  · exact Or.inr (Or.inr (Or.inr (Or.inl (by exact_mod_cast h))))
  · exact Or.inr (Or.inr (Or.inr (Or.inr (Or.inl (by exact_mod_cast h)))))
  · exact Or.inr (Or.inr (Or.inr (Or.inr (Or.inr (Or.inl (by exact_mod_cast h))))))
  · exact Or.inr (Or.inr (Or.inr (Or.inr (Or.inr (Or.inr (Or.inl (by exact_mod_cast h)))))))
  · exact Or.inr (Or.inr (Or.inr (Or.inr (Or.inr (Or.inr (Or.inr (by exact_mod_cast h)))))))

theorem separatedHV_sound {a b : Box} {h v : Int} (hs : separatedHV a b h v = true) :
    SepHV a.toR b.toR (h : ℝ) (v : ℝ) := by
  simp only [separatedHV, Bool.or_eq_true, decide_eq_true_eq] at hs
  simp only [SepHV, Box.toR]
  rcases hs with ((((((hs | hs) | hs) | hs) | hs) | hs) | hs) | hs
  · exact Or.inl (by exact_mod_cast hs)
  · exact Or.inr (Or.inl (by exact_mod_cast hs))
  · exact Or.inr (Or.inr (Or.inl (by exact_mod_cast hs)))
  · exact Or.inr (Or.inr (Or.inr (Or.inl (by exact_mod_cast hs))))
  · exact Or.inr (Or.inr (Or.inr (Or.inr (Or.inl (by exact_mod_cast hs)))))
  · exact Or.inr (Or.inr (Or.inr (Or.inr (Or.inr (Or.inl (by exact_mod_cast hs))))))
  · exact Or.inr (Or.inr (Or.inr (Or.inr (Or.inr (Or.inr (Or.inl (by exact_mod_cast hs)))))))
  · exact Or.inr (Or.inr (Or.inr (Or.inr (Or.inr (Or.inr (Or.inr (by exact_mod_cast hs)))))))

theorem inside_sound {b : Box} {m x y z t : Int} (hm : 0 ≤ m)
    (h : inside b m x y z t = true) :
    b.toR.mem ⟨x, y, z⟩ (t : ℝ) := by
  simp only [inside, Bool.and_eq_true, decide_eq_true_eq] at h
  obtain ⟨⟨⟨⟨⟨⟨⟨hx0, hx1⟩, hy0⟩, hy1⟩, hz0⟩, hz1⟩, ht0⟩, ht1⟩ := h
  simp only [RBox.mem, Box.toR]
  refine ⟨?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_⟩ <;>
    first
    | (exact_mod_cast ht0) | (exact_mod_cast ht1)
    | (have : ((b.x0 : ℤ) : ℝ) + m ≤ x := by exact_mod_cast hx0
       have : (0 : ℝ) ≤ m := by exact_mod_cast hm
       linarith)
    | (have : (x : ℝ) + m ≤ b.x1 := by exact_mod_cast hx1
       have : (0 : ℝ) ≤ m := by exact_mod_cast hm
       linarith)
    | (have : ((b.y0 : ℤ) : ℝ) + m ≤ y := by exact_mod_cast hy0
       have : (0 : ℝ) ≤ m := by exact_mod_cast hm
       linarith)
    | (have : (y : ℝ) + m ≤ b.y1 := by exact_mod_cast hy1
       have : (0 : ℝ) ≤ m := by exact_mod_cast hm
       linarith)
    | (have : ((b.z0 : ℤ) : ℝ) + m ≤ z := by exact_mod_cast hz0
       have : (0 : ℝ) ≤ m := by exact_mod_cast hm
       linarith)
    | (have : (z : ℝ) + m ≤ b.z1 := by exact_mod_cast hz1
       have : (0 : ℝ) ≤ m := by exact_mod_cast hm
       linarith)

/-- **Checker-level safety.** If the deployed checker accepts the two volumes
with buffer `buf ≥ 0`, and each aircraft's position conforms to its own
volume at time `t`, the two aircraft are at least `buf` apart at `t`. -/
theorem checker_separation {a b : Box} {buf : Int} (hbuf : 0 ≤ buf)
    (hs : separated a b buf = true) {p q : P3} {t : ℝ}
    (hp : a.toR.mem p t) (hq : b.toR.mem q t) :
    ((buf : ℝ)) ^ 2 ≤ distSq p q :=
  sep_of_spec (by exact_mod_cast hbuf) (separated_sound hs) hp hq

/-- Same, with both positions given as conforming integer samples. -/
theorem checker_separation_samples {a b : Box} {buf m : Int} (hbuf : 0 ≤ buf) (hm : 0 ≤ m)
    (hs : separated a b buf = true) {px py pz qx qy qz t : Int}
    (hp : inside a m px py pz t = true) (hq : inside b m qx qy qz t = true) :
    ((buf : ℝ)) ^ 2 ≤ distSq ⟨px, py, pz⟩ ⟨qx, qy, qz⟩ :=
  checker_separation hbuf hs (inside_sound hm hp) (inside_sound hm hq)

/-- **Checker-level safety, cylinder form.** If the deployed checker accepts the two
volumes with thresholds `h, v ≥ 0` and both aircraft conform at time `t`, they are
not in loss of separation at `t`. -/
theorem checker_no_los {a b : Box} {h v : Int} (hh : 0 ≤ h) (hv : 0 ≤ v)
    (hs : separatedHV a b h v = true) {p q : P3} {t : ℝ}
    (hp : a.toR.mem p t) (hq : b.toR.mem q t) : ¬ LoS p q (h : ℝ) (v : ℝ) :=
  no_los_of_spec (by exact_mod_cast hh) (by exact_mod_cast hv) (separatedHV_sound hs) hp hq

/-- Same, with both positions given as conforming integer samples. -/
theorem checker_no_los_samples {a b : Box} {h v m : Int} (hh : 0 ≤ h) (hv : 0 ≤ v) (hm : 0 ≤ m)
    (hs : separatedHV a b h v = true) {px py pz qx qy qz t : Int}
    (hp : inside a m px py pz t = true) (hq : inside b m qx qy qz t = true) :
    ¬ LoS ⟨px, py, pz⟩ ⟨qx, qy, qz⟩ (h : ℝ) (v : ℝ) :=
  checker_no_los hh hv hs (inside_sound hm hp) (inside_sound hm hq)

/-- Integer widening commutes with the cast to ℝ. -/
theorem Box.inflate_toR (b : Box) (eh ev dt : Int) :
    (b.inflate eh ev dt).toR = b.toR.inflate (eh : ℝ) (ev : ℝ) (dt : ℝ) := by
  simp only [Box.inflate, Box.toR, RBox.inflate, Int.cast_sub, Int.cast_add]

theorem separatedDev_sound {a b : Box} {h v eh ev dt : Int} (hs : separatedDev a b h v eh ev dt = true) :
    SepHV (a.toR.inflate eh ev dt) (b.toR.inflate eh ev dt) (h : ℝ) (v : ℝ) := by
  have := separatedHV_sound hs
  rwa [Box.inflate_toR, Box.inflate_toR] at this

/-- **Checker-level Theorem 3.** If the deployed checker accepts the two nominal volumes with
thresholds `h, v ≥ 0` and error bound `(eh, ev, dt)`, and each aircraft tracks a point of its own
nominal volume within that bound, they are not in loss of separation. -/
theorem checker_no_los_dev {a b : Box} {h v eh ev dt : Int} (hh : 0 ≤ h) (hv : 0 ≤ v)
    (hs : separatedDev a b h v eh ev dt = true) {p q p0 q0 : P3} {t t0 s0 : ℝ}
    (hp0 : a.toR.mem p0 t0) (hq0 : b.toR.mem q0 s0)
    (hp : Tracks p t p0 t0 eh ev dt) (hq : Tracks q t q0 s0 eh ev dt) : ¬ LoS p q (h : ℝ) (v : ℝ) :=
  no_los_of_deviation (by exact_mod_cast hh) (by exact_mod_cast hv) (separatedDev_sound hs) hp0 hq0 hp hq

end Utm
