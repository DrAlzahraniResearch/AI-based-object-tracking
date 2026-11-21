import Mathlib.Basic.Real.Basic
import Mathlib.Tactic.Linarith
import Mathlib.Tactic.Positivity

/-!
# Specification over ℝ and Theorem 1

Theorem 1 (`Utm.sep_of_spec`): if two 4D operational-intent volumes are
buffer-separated (never occupied at the same time, or at least `buf` apart
along some spatial axis) and two aircraft are each inside their own volume at
the same time `t`, then they are at least `buf` apart.

Distances are stated squared (`buf ^ 2 ≤ distSq p q`, with `0 ≤ buf`), which is
equivalent to `buf ≤ ‖p - q‖` for the Euclidean norm and avoids `Real.sqrt`.
-/

namespace Utm

/-- A point in the local ENU frame, metres (or any consistent unit). -/
structure P3 where
  x : ℝ
  y : ℝ
  z : ℝ

/-- Axis-aligned 4D volume over ℝ, closed intervals. -/
structure RBox where
  x0 : ℝ
  x1 : ℝ
  y0 : ℝ
  y1 : ℝ
  z0 : ℝ
  z1 : ℝ
  t0 : ℝ
  t1 : ℝ

/-- Aircraft position `p` at time `t` lies in volume `b`. -/
def RBox.mem (b : RBox) (p : P3) (t : ℝ) : Prop :=
  b.x0 ≤ p.x ∧ p.x ≤ b.x1 ∧ b.y0 ≤ p.y ∧ p.y ≤ b.y1 ∧
  b.z0 ≤ p.z ∧ p.z ≤ b.z1 ∧ b.t0 ≤ t ∧ t ≤ b.t1

/-- Squared Euclidean distance. -/
def distSq (p q : P3) : ℝ := (p.x - q.x) ^ 2 + (p.y - q.y) ^ 2 + (p.z - q.z) ^ 2

/-- Buffer separation of two volumes (the property the integer checker decides). -/
def SepSpec (a b : RBox) (buf : ℝ) : Prop :=
  a.t1 < b.t0 ∨ b.t1 < a.t0 ∨
  a.x1 + buf ≤ b.x0 ∨ b.x1 + buf ≤ a.x0 ∨
  a.y1 + buf ≤ b.y0 ∨ b.y1 + buf ≤ a.y0 ∨
  a.z1 + buf ≤ b.z0 ∨ b.z1 + buf ≤ a.z0

/-- A gap of at least `buf ≥ 0` along one axis bounds the squared distance. -/
private lemma sq_le_of_gap {u v buf r s : ℝ} (hbuf : 0 ≤ buf) (h : buf ≤ v - u) :
    buf ^ 2 ≤ (u - v) ^ 2 + r ^ 2 + s ^ 2 := by
  nlinarith [mul_le_mul h h hbuf (by linarith), sq_nonneg r, sq_nonneg s]

/-- **Theorem 1.** Buffer-separated volumes keep their occupants `buf` apart. -/
theorem sep_of_spec {a b : RBox} {buf : ℝ} (hbuf : 0 ≤ buf) (h : SepSpec a b buf)
    {p q : P3} {t : ℝ} (hp : a.mem p t) (hq : b.mem q t) : buf ^ 2 ≤ distSq p q := by
  obtain ⟨hpx0, hpx1, hpy0, hpy1, hpz0, hpz1, hpt0, hpt1⟩ := hp
  obtain ⟨hqx0, hqx1, hqy0, hqy1, hqz0, hqz1, hqt0, hqt1⟩ := hq
  unfold distSq
  rcases h with h | h | h | h | h | h | h | h
  · exact absurd (lt_of_le_of_lt hpt1 h) (not_lt.mpr hqt0)
  · exact absurd (lt_of_le_of_lt hqt1 h) (not_lt.mpr hpt0)
  · exact sq_le_of_gap hbuf (by linarith)
  · have := sq_le_of_gap (u := q.x) (v := p.x) (r := p.y - q.y) (s := p.z - q.z) hbuf (by linarith)
    nlinarith [sq_nonneg (p.x - q.x)]
  · have := sq_le_of_gap (u := p.y) (v := q.y) (r := p.x - q.x) (s := p.z - q.z) hbuf (by linarith)
    nlinarith
  · have := sq_le_of_gap (u := q.y) (v := p.y) (r := p.x - q.x) (s := p.z - q.z) hbuf (by linarith)
    nlinarith
  · have := sq_le_of_gap (u := p.z) (v := q.z) (r := p.x - q.x) (s := p.y - q.y) hbuf (by linarith)
    nlinarith
  · have := sq_le_of_gap (u := q.z) (v := p.z) (r := p.x - q.x) (s := p.y - q.y) hbuf (by linarith)
    nlinarith

/-! ## Theorem 2: cylinder separation (the loss-of-separation definition)

Loss of separation between two aircraft is horizontal distance `< h` AND vertical
distance `< v` (a cylinder, not a sphere). Theorem 1 with `buf = √(h² + v²)`
would imply no LoS but is far more conservative: it cannot certify two cruise
layers that are `v` apart unless their boxes are `√(h² + v²)` apart vertically.
-/

/-- Squared horizontal distance. -/
def hdistSq (p q : P3) : ℝ := (p.x - q.x) ^ 2 + (p.y - q.y) ^ 2

/-- Loss of separation with horizontal threshold `h` and vertical threshold `v`. -/
def LoS (p q : P3) (h v : ℝ) : Prop := hdistSq p q < h ^ 2 ∧ (p.z - q.z) ^ 2 < v ^ 2

/-- Cylinder separation of two volumes (the property `separatedHV` decides). -/
def SepHV (a b : RBox) (h v : ℝ) : Prop :=
  a.t1 < b.t0 ∨ b.t1 < a.t0 ∨
  a.x1 + h ≤ b.x0 ∨ b.x1 + h ≤ a.x0 ∨
  a.y1 + h ≤ b.y0 ∨ b.y1 + h ≤ a.y0 ∨
  a.z1 + v ≤ b.z0 ∨ b.z1 + v ≤ a.z0

private lemma sq_le_of_gap1 {u w g : ℝ} (hg : 0 ≤ g) (h : g ≤ w - u) : g ^ 2 ≤ (u - w) ^ 2 := by
  nlinarith [mul_le_mul h h hg (by linarith)]

/-- **Theorem 2.** Two aircraft inside cylinder-separated volumes at the same time
are at least `h` apart horizontally or at least `v` apart vertically. -/
theorem sep_hv_of_spec {a b : RBox} {h v : ℝ} (hh : 0 ≤ h) (hv : 0 ≤ v) (hs : SepHV a b h v)
    {p q : P3} {t : ℝ} (hp : a.mem p t) (hq : b.mem q t) :
    h ^ 2 ≤ hdistSq p q ∨ v ^ 2 ≤ (p.z - q.z) ^ 2 := by
  obtain ⟨hpx0, hpx1, hpy0, hpy1, hpz0, hpz1, hpt0, hpt1⟩ := hp
  obtain ⟨hqx0, hqx1, hqy0, hqy1, hqz0, hqz1, hqt0, hqt1⟩ := hq
  unfold hdistSq
  rcases hs with hs | hs | hs | hs | hs | hs | hs | hs
  · exact absurd (lt_of_le_of_lt hpt1 hs) (not_lt.mpr hqt0)
  · exact absurd (lt_of_le_of_lt hqt1 hs) (not_lt.mpr hpt0)
  · left; nlinarith [sq_le_of_gap1 (u := p.x) (w := q.x) hh (by linarith), sq_nonneg (p.y - q.y)]
  · left; nlinarith [sq_le_of_gap1 (u := q.x) (w := p.x) hh (by linarith), sq_nonneg (p.y - q.y)]
  · left; nlinarith [sq_le_of_gap1 (u := p.y) (w := q.y) hh (by linarith), sq_nonneg (p.x - q.x)]
  · left; nlinarith [sq_le_of_gap1 (u := q.y) (w := p.y) hh (by linarith), sq_nonneg (p.x - q.x)]
  · right; nlinarith [sq_le_of_gap1 (u := p.z) (w := q.z) hv (by linarith)]
  · right; nlinarith [sq_le_of_gap1 (u := q.z) (w := p.z) hv (by linarith)]

/-- Theorem 2, stated as the safety property: no loss of separation. -/
theorem no_los_of_spec {a b : RBox} {h v : ℝ} (hh : 0 ≤ h) (hv : 0 ≤ v) (hs : SepHV a b h v)
    {p q : P3} {t : ℝ} (hp : a.mem p t) (hq : b.mem q t) : ¬ LoS p q h v := by
  rintro ⟨h1, h2⟩
  rcases sep_hv_of_spec hh hv hs hp hq with h3 | h3 <;> linarith

/-! ## Theorem 3: deviation bounds (sim-to-real)

The volumes above are a plan. A real autopilot tracks its plan with some error. If every
aircraft stays within `eh` horizontally (per axis), `ev` vertically and `δ` in time of some
point of its own nominal volume, it is inside that volume widened by `(eh, ev, δ)`, so
cylinder separation of the widened volumes rules out loss of separation. Equivalently,
the nominal volumes need gaps enlarged by twice the error (`sepHV_inflate_iff`). The error
bound is what the fidelity report measures (Tier A vs PX4).
-/

/-- Volume widened by `eh` along x and y, `ev` along z and `δ` in time, on every side. -/
def RBox.inflate (b : RBox) (eh ev δ : ℝ) : RBox :=
  ⟨b.x0 - eh, b.x1 + eh, b.y0 - eh, b.y1 + eh, b.z0 - ev, b.z1 + ev, b.t0 - δ, b.t1 + δ⟩

/-- The actual sample `(p, t)` is within the error bound of the nominal sample `(p0, t0)`. -/
def Tracks (p : P3) (t : ℝ) (p0 : P3) (t0 : ℝ) (eh ev δ : ℝ) : Prop :=
  |p.x - p0.x| ≤ eh ∧ |p.y - p0.y| ≤ eh ∧ |p.z - p0.z| ≤ ev ∧ |t - t0| ≤ δ

/-- Tracking a point of `b` within the bound puts the actual sample in the widened volume. -/
theorem mem_inflate_of_tracks {b : RBox} {p p0 : P3} {t t0 eh ev δ : ℝ}
    (h0 : b.mem p0 t0) (ht : Tracks p t p0 t0 eh ev δ) : (b.inflate eh ev δ).mem p t := by
  obtain ⟨hx0, hx1, hy0, hy1, hz0, hz1, ht0, ht1⟩ := h0
  obtain ⟨ex, ey, ez, et⟩ := ht
  rw [abs_le] at ex ey ez et
  simp only [RBox.mem, RBox.inflate]
  refine ⟨?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_⟩ <;> linarith [ex.1, ex.2, ey.1, ey.2, ez.1, ez.2, et.1, et.2]

/-- Widened volumes are cylinder-separated iff the nominal ones have gaps enlarged by twice the
error: `h + 2 eh` horizontally, `v + 2 ev` vertically, or more than `2 δ` in time. -/
theorem sepHV_inflate_iff (a b : RBox) (h v eh ev δ : ℝ) :
    SepHV (a.inflate eh ev δ) (b.inflate eh ev δ) h v ↔
      (a.t1 + 2 * δ < b.t0 ∨ b.t1 + 2 * δ < a.t0 ∨
       a.x1 + (h + 2 * eh) ≤ b.x0 ∨ b.x1 + (h + 2 * eh) ≤ a.x0 ∨
       a.y1 + (h + 2 * eh) ≤ b.y0 ∨ b.y1 + (h + 2 * eh) ≤ a.y0 ∨
       a.z1 + (v + 2 * ev) ≤ b.z0 ∨ b.z1 + (v + 2 * ev) ≤ a.z0) := by
  simp only [SepHV, RBox.inflate]
  refine or_congr ?_ (or_congr ?_ (or_congr ?_ (or_congr ?_ (or_congr ?_ (or_congr ?_
    (or_congr ?_ ?_)))))) <;> constructor <;> intro hx <;> linarith

/-- **Theorem 3.** If the widened volumes are cylinder-separated and each aircraft tracks a point
of its own nominal volume within `(eh, ev, δ)`, the aircraft are not in loss of separation. -/
theorem no_los_of_deviation {a b : RBox} {h v eh ev δ : ℝ} (hh : 0 ≤ h) (hv : 0 ≤ v)
    (hs : SepHV (a.inflate eh ev δ) (b.inflate eh ev δ) h v)
    {p q p0 q0 : P3} {t t0 s0 : ℝ} (hp0 : a.mem p0 t0) (hq0 : b.mem q0 s0)
    (hp : Tracks p t p0 t0 eh ev δ) (hq : Tracks q t q0 s0 eh ev δ) : ¬ LoS p q h v :=
  no_los_of_spec hh hv hs (mem_inflate_of_tracks hp0 hp) (mem_inflate_of_tracks hq0 hq)

end Utm
