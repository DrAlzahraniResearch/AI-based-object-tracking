/-!
# Executable separation checker (Lean core only)

Operational intents are approximated by axis-aligned 4D volumes ("boxes") in
fixed-point integers: millimetres for x, y, z (local ENU frame) and
milliseconds for t. Integers keep the checker exact, so it can be proved
correct against the real-number specification in `UtmProofs`
(`Utm.separated_sound`, `Utm.checker_separation`). No `Float` is involved.

This module deliberately imports nothing from mathlib: it is what gets
compiled to C and linked into the runtime library (`runtime/`).
-/

namespace Utm

/-- Axis-aligned 4D volume, closed intervals, integer mm and ms. -/
structure Box where
  x0 : Int
  x1 : Int
  y0 : Int
  y1 : Int
  z0 : Int
  z1 : Int
  t0 : Int
  t1 : Int
  deriving Repr, DecidableEq

/-- `true` iff the two volumes are never occupied at the same time, or are at
least `buf` apart along at least one spatial axis. Proved sound in
`UtmProofs.Refinement`: then any two points, one in each volume at the same
time, are at least `buf` apart (Euclidean). -/
def separated (a b : Box) (buf : Int) : Bool :=
  decide (a.t1 < b.t0) || decide (b.t1 < a.t0) ||
  decide (a.x1 + buf ≤ b.x0) || decide (b.x1 + buf ≤ a.x0) ||
  decide (a.y1 + buf ≤ b.y0) || decide (b.y1 + buf ≤ a.y0) ||
  decide (a.z1 + buf ≤ b.z0) || decide (b.z1 + buf ≤ a.z0)

/-- Cylinder version, matching the loss-of-separation definition (horizontal < `h`
AND vertical < `v`): `true` iff the volumes are never occupied at the same
time, or are at least `h` apart along x or y, or at least `v` apart along z.
Proved sound in `UtmProofs.Refinement` (`checker_no_los`): two occupants at
the same time are then never in loss of separation. -/
def separatedHV (a b : Box) (h v : Int) : Bool :=
  decide (a.t1 < b.t0) || decide (b.t1 < a.t0) ||
  decide (a.x1 + h ≤ b.x0) || decide (b.x1 + h ≤ a.x0) ||
  decide (a.y1 + h ≤ b.y0) || decide (b.y1 + h ≤ a.y0) ||
  decide (a.z1 + v ≤ b.z0) || decide (b.z1 + v ≤ a.z0)

/-- `true` iff the sample (x, y, z) at time t lies in `b` with at least `m`
of spatial margin on every side (conformance of a position sample). -/
def inside (b : Box) (m x y z t : Int) : Bool :=
  decide (b.x0 + m ≤ x) && decide (x + m ≤ b.x1) &&
  decide (b.y0 + m ≤ y) && decide (y + m ≤ b.y1) &&
  decide (b.z0 + m ≤ z) && decide (z + m ≤ b.z1) &&
  decide (b.t0 ≤ t) && decide (t ≤ b.t1)

/-! ## C ABI

`Int64` crosses the FFI boundary as a plain 64-bit integer and `UInt8` as
`uint8_t`. Values are converted to unbounded `Int` before any arithmetic, so
there is no overflow inside the check. -/

/-- C: `uint8_t utm_separated(int64_t ax0, ..., int64_t bt1, int64_t buf)` -/
@[export utm_separated]
def separatedC (ax0 ax1 ay0 ay1 az0 az1 at0 at1
                bx0 bx1 by0 by1 bz0 bz1 bt0 bt1 buf : Int64) : UInt8 :=
  if separated ⟨ax0.toInt, ax1.toInt, ay0.toInt, ay1.toInt, az0.toInt, az1.toInt, at0.toInt, at1.toInt⟩
               ⟨bx0.toInt, bx1.toInt, by0.toInt, by1.toInt, bz0.toInt, bz1.toInt, bt0.toInt, bt1.toInt⟩
               buf.toInt
  then 1 else 0

/-- C: `uint8_t utm_separated_hv(int64_t ax0, ..., int64_t bt1, int64_t h, int64_t v)` -/
@[export utm_separated_hv]
def separatedHVC (ax0 ax1 ay0 ay1 az0 az1 at0 at1
                  bx0 bx1 by0 by1 bz0 bz1 bt0 bt1 h v : Int64) : UInt8 :=
  if separatedHV ⟨ax0.toInt, ax1.toInt, ay0.toInt, ay1.toInt, az0.toInt, az1.toInt, at0.toInt, at1.toInt⟩
                 ⟨bx0.toInt, bx1.toInt, by0.toInt, by1.toInt, bz0.toInt, bz1.toInt, bt0.toInt, bt1.toInt⟩
                 h.toInt v.toInt
  then 1 else 0

/-- C: `uint8_t utm_inside(int64_t x0, ..., int64_t t1, int64_t m, int64_t x, int64_t y, int64_t z, int64_t t)` -/
@[export utm_inside]
def insideC (x0 x1 y0 y1 z0 z1 t0 t1 m x y z t : Int64) : UInt8 :=
  if inside ⟨x0.toInt, x1.toInt, y0.toInt, y1.toInt, z0.toInt, z1.toInt, t0.toInt, t1.toInt⟩
            m.toInt x.toInt y.toInt z.toInt t.toInt
  then 1 else 0

/-- ABI version, bumped whenever an exported signature changes. -/
@[export utm_abi_version]
def abiVersion (_ : Unit) : UInt32 := 2

end Utm
