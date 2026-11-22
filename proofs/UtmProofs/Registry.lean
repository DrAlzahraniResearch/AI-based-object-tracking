import UtmProofs.Refinement

/-!
# Theorem 4: the intent registry stays pairwise separated

The scheduler (admission, replanning) and the hub sequencer share one discipline over the registry of
accepted boxes, each tagged with its flight: withdraw the flight's own boxes, check every candidate box
against every box of every other flight with `separatedHV`, and register the candidates only if all checks
pass. Some flights are registered without the check (an emergency that could not be certified, a landing
after the holding limit) so that later plans avoid them; they are marked forced. Expired boxes are pruned.
This file proves that the discipline keeps the registry invariant: any two certified boxes of different
flights are cylinder-separated, whatever was forced in between. With Theorem 2 (`checker_no_los`), two
flights both inside registered boxes at the same instant are therefore never in loss of separation.

Milestone 5.5 adds removals of part of a flight's entries: a landing reservation displaced by an emergency
(`unreserve`), and the displaced rows of an emergency's priority admission. The implementation checks the
emergency against the registry without the displaceable rows, then removes the rows of every displaceable
flight it conflicts with; the result is the same as removing those rows first (`drop`) and then admitting
through the full check, which is how the model states it. Any removal preserves the invariant (`inv_filter`).
A flight that keeps its reservation after a failed slot search leaves the registry as it was.

This is a model of the registry protocol, not of the Python code that implements it (`FcfsScheduler.admit`,
`replan`, `HubSequencer.sequence`, `register` / `withdraw` / `unreserve` / `prune`); that code follows the
protocol and is tested, and every run is audited afterwards with the verified checker.
-/

namespace Utm

/-- `separatedHV` is symmetric. -/
theorem separatedHV_comm (a b : Box) (h v : Int) : separatedHV a b h v = separatedHV b a h v := by
  simp only [separatedHV]
  ac_rfl

/-- A registry entry: the owning flight, whether the entry was certified (admitted through the check) or
forced (an emergency flown uncertified, a contingent landing: registered so that later plans avoid it),
and the box. -/
structure Entry where
  flight : Nat
  cert : Bool
  box : Box

abbrev Registry := List Entry

/-- The invariant: certified boxes of different flights are cylinder-separated. Forced boxes are exempt. -/
def Inv (h v : Int) (r : Registry) : Prop :=
  ∀ p ∈ r, ∀ q ∈ r, p.cert = true → q.cert = true → p.flight ≠ q.flight → separatedHV p.box q.box h v = true

/-- The admission check: every candidate box against every registered box (certified or forced) of every
other flight. -/
def compatible (h v : Int) (k : Nat) (cand : List Box) (r : Registry) : Bool :=
  cand.all fun a => r.all fun q => q.flight == k || separatedHV a q.box h v

/-- Register flight `k`'s candidate boxes as certified if they are compatible; otherwise leave the registry
unchanged. -/
def admit (h v : Int) (k : Nat) (cand : List Box) (r : Registry) : Option Registry :=
  if compatible h v k cand r then some (r ++ cand.map (⟨k, true, ·⟩)) else none

/-- Register flight `k`'s boxes without a check, as forced. -/
def force (k : Nat) (cand : List Box) (r : Registry) : Registry := r ++ cand.map (⟨k, false, ·⟩)

/-- Withdraw flight `k` (replan, sequencing, landing). -/
def withdraw (k : Nat) (r : Registry) : Registry := r.filter fun q => q.flight != k

/-- Drop boxes that ended before time `t`. -/
def prune (t : Int) (r : Registry) : Registry := r.filter fun q => decide (t ≤ q.box.t1)

theorem inv_nil (h v : Int) : Inv h v [] := by simp [Inv]

theorem inv_filter {h v : Int} {r : Registry} (f : Entry → Bool) (hr : Inv h v r) :
    Inv h v (r.filter f) := by
  intro p hp q hq
  exact hr p (List.mem_of_mem_filter hp) q (List.mem_of_mem_filter hq)

theorem inv_withdraw {h v : Int} {r : Registry} (k : Nat) (hr : Inv h v r) : Inv h v (withdraw k r) :=
  inv_filter _ hr

theorem inv_prune {h v : Int} {r : Registry} (t : Int) (hr : Inv h v r) : Inv h v (prune t r) :=
  inv_filter _ hr

/-- Forced entries do not touch the invariant (they are exempt from it). -/
theorem inv_force {h v : Int} {r : Registry} (k : Nat) (cand : List Box) (hr : Inv h v r) :
    Inv h v (force k cand r) := by
  intro p hp q hq hpc hqc hne
  rcases List.mem_append.1 hp with hp | hp <;> rcases List.mem_append.1 hq with hq | hq
  · exact hr p hp q hq hpc hqc hne
  · obtain ⟨b, _, rfl⟩ := List.mem_map.1 hq; simp at hqc
  · obtain ⟨a, _, rfl⟩ := List.mem_map.1 hp; simp at hpc
  · obtain ⟨a, _, rfl⟩ := List.mem_map.1 hp; simp at hpc

/-- **Theorem 4.** Admission preserves the invariant. -/
theorem inv_admit {h v : Int} {r r' : Registry} {k : Nat} {cand : List Box}
    (hr : Inv h v r) (ha : admit h v k cand r = some r') : Inv h v r' := by
  unfold admit at ha
  split at ha
  · rename_i hc
    cases ha
    simp only [compatible, List.all_eq_true, Bool.or_eq_true, beq_iff_eq] at hc
    intro p hp q hq hpc hqc hne
    rcases List.mem_append.1 hp with hp | hp <;> rcases List.mem_append.1 hq with hq | hq
    · exact hr p hp q hq hpc hqc hne
    · obtain ⟨b, hb, rfl⟩ := List.mem_map.1 hq
      rcases hc b hb p hp with h1 | h1
      · exact absurd h1 hne
      · rw [separatedHV_comm]; exact h1
    · obtain ⟨a, ha', rfl⟩ := List.mem_map.1 hp
      rcases hc a ha' q hq with h1 | h1
      · exact absurd h1.symm hne
      · exact h1
    · obtain ⟨a, _, rfl⟩ := List.mem_map.1 hp
      obtain ⟨b, _, rfl⟩ := List.mem_map.1 hq
      exact absurd rfl hne
  · cases ha

/-- Any sequence of the operations, from the empty registry. `drop f` removes the entries `f` rejects
(a displaced landing reservation, displaced rows before a priority admission). -/
inductive Op where
  | admit (k : Nat) (cand : List Box)
  | force (k : Nat) (cand : List Box)
  | withdraw (k : Nat)
  | prune (t : Int)
  | drop (f : Entry → Bool)

def step (h v : Int) (r : Registry) : Op → Registry
  | .admit k c => (admit h v k c r).getD r
  | .force k c => force k c r
  | .withdraw k => withdraw k r
  | .prune t => prune t r
  | .drop f => r.filter f

theorem inv_steps (h v : Int) (ops : List Op) : Inv h v (ops.foldl (step h v) []) := by
  suffices ∀ r, Inv h v r → Inv h v (ops.foldl (step h v) r) from this [] (inv_nil h v)
  induction ops with
  | nil => intro r hr; exact hr
  | cons o os ih =>
    intro r hr
    apply ih
    cases o with
    | admit k c =>
      simp only [step]
      cases ha : admit h v k c r with
      | none => simpa using hr
      | some r' => simpa using inv_admit hr ha
    | force k c => exact inv_force k c hr
    | withdraw k => exact inv_withdraw k hr
    | prune t => exact inv_prune t hr
    | drop f => exact inv_filter f hr

/-- **Fleet safety.** Two flights, each inside one of its certified registered boxes at the same instant,
are not in loss of separation (Theorem 4 + Theorem 2). -/
theorem registry_no_los {h v : Int} (hh : 0 ≤ h) (hv : 0 ≤ v) {r : Registry} (hr : Inv h v r)
    {p q : Entry} (hp : p ∈ r) (hq : q ∈ r) (hpc : p.cert = true) (hqc : q.cert = true)
    (hne : p.flight ≠ q.flight) {x y : P3} {t : ℝ}
    (hx : p.box.toR.mem x t) (hy : q.box.toR.mem y t) : ¬ LoS x y (h : ℝ) (v : ℝ) :=
  checker_no_los hh hv (hr p hp q hq hpc hqc hne) hx hy

end Utm
