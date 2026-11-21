"""Operational intents: a planned flight as a set of 4D volumes (ASTM F3548 style).

Each leg of the route is split into pieces of at most `max_piece_m` horizontally
and `max_piece_z_m` vertically (so a climb or descent through the cruise layers
blocks one altitude slice at a time, not the whole column); each piece
becomes an axis-aligned box (the piece's bounding box inflated by lateral and
vertical buffers) with a time window from the kinematic plan, widened by a
slack that grows with time since release (later = less certain).

Boxes are also exported in the checker's integer units (mm, ms) so that the
Lean-verified checker (`proofs/UtmChecker.lean`) can decide pairwise
separation and position conformance exactly.

Each box has a phase: "terminal" if the whole piece lies within
`terminal_radius_m` of the mission's pad (take-off and landing columns),
else "en_route". The strategic scheduler deconflicts en-route boxes; terminal
boxes are the hub sequencer's job (plan step 7).

The vertical buffer (2.5 m) is tied to the 15 m layer step: boxes on adjacent
layers are then exactly 10 m apart vertically, which is what the cylinder
checker (Theorem 2, v = 10 m) needs to certify them.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .missions import Mission
from .vehicle import VehicleParams


@dataclass(frozen=True)
class IntentParams:
    lateral_buffer_m: float = 20.0
    vertical_buffer_m: float = 2.5      # layer step 15 = v_sep 10 + 2 x 2.5
    terminal_radius_m: float = 40.0
    slack_s: float = 5.0                # fixed time slack each side
    slack_per_min: float = 2.0          # extra slack (s) per minute since release
    max_piece_m: float = 50.0           # horizontal piece length (shorter = tighter boxes on diagonals)
    max_piece_z_m: float = 10.0         # vertical piece height: a descent blocks one 10 m slice at a time
    takeoff_s: float = 0.0              # release -> lift-off lag on the pad (PX4: spool-up + MPC_TKO_RAMP_T ramp, ~4 s)
    terminal_lateral_buffer_m: float | None = None   # lateral buffer of terminal boxes (None: lateral_buffer_m).
                                        # Milestone 5 uses 5 m: PX4 tracked its plan within 3.5 m (p99, milestone 4.5)


@dataclass
class Box4:
    mission: int
    leg: int
    x0: float
    x1: float
    y0: float
    y1: float
    z0: float
    z1: float
    t0: float
    t1: float
    phase: str = "en_route"             # en_route | terminal
    flow: str = ""                      # "dep": the departure climb at the pad (leading terminal boxes);
                                        # "arr": after the approach fix (or trailing terminal boxes); "": en route

    def as_int(self) -> tuple:
        """Checker units: millimetres and milliseconds, rounded outwards (conservative)."""
        f, c = math.floor, math.ceil
        return (f(self.x0 * 1000), c(self.x1 * 1000), f(self.y0 * 1000), c(self.y1 * 1000),
                f(self.z0 * 1000), c(self.z1 * 1000), f(self.t0 * 1000), c(self.t1 * 1000))

    def contains(self, x, y, z, t) -> bool:
        return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1 and self.z0 <= z <= self.z1 and self.t0 <= t <= self.t1


def _leg_time(a, b, vp: VehicleParams) -> float:
    """Planned leg duration. Overheads fitted to Tier A (230 legs, mini_town seed 2, no policy):
    horizontal d/v + 2.0 s (0.6 v/a), climb dz/v_up + 0.15 s, descent dz/v_down (no overhead).
    Refit for Tier B (PX4) before using its intents."""
    dxy = math.hypot(b[0] - a[0], b[1] - a[1])
    dz = b[2] - a[2]
    th = dxy / vp.cruise_xy + (0.6 * vp.cruise_xy / vp.a_xy_max if dxy > 1 else 0.0)
    tv = abs(dz) / (vp.v_up_max if dz > 0 else vp.v_down_max) + (0.15 if dz > 0.5 else 0.0)
    return max(th, tv) + 0.5 * min(th, tv)    # partially overlapped climb and cruise in the guidance law


def planned_schedule(m: Mission, vp: VehicleParams, takeoff_s: float = 0.0) -> list[tuple]:
    """[(a_xyz, b_xyz, t_start, t_end)] per leg, plus holds as zero-length legs. A take-off lag (flights
    that start on their pad, not mid-air replans) is a zero-length leg at the pad."""
    return [leg[:4] for leg in _schedule(m, vp, takeoff_s)]


def _schedule(m: Mission, vp: VehicleParams, takeoff_s: float = 0.0) -> list[tuple]:
    """planned_schedule with the waypoint index of each leg as a fifth element (-1: take-off lag)."""
    start = tuple(m.meta["start"])
    t = m.release_t
    out, prev = [], start
    pad = m.meta.get("pad", m.meta["start"])
    if takeoff_s > 0 and start[2] <= pad[2] + 0.5 and math.dist(start[:2], pad[:2]) < 1.0:
        out.append((start, start, t, t + takeoff_s, -1))
        t += takeoff_s
    for wi, w in enumerate(m.waypoints):
        cur = (w.x, w.y, w.z)
        dt = _leg_time(prev, cur, vp)
        out.append((prev, cur, t, t + dt, wi))
        t += dt
        if w.hold_s > 0:
            out.append((cur, cur, t, t + w.hold_s, wi))
            t += w.hold_s
        prev = cur
    return out


def mission_intents(m: Mission, vp: VehicleParams, ip: IntentParams = IntentParams()) -> list[Box4]:
    boxes, wis = [], []
    pad = np.asarray(m.meta.get("pad", m.meta["start"])[:2], float)    # replans start mid-air
    for k, (a, b, ta, tb, wi) in enumerate(_schedule(m, vp, ip.takeoff_s)):
        L = math.dist(a[:2], b[:2])
        n = max(1, math.ceil(L / ip.max_piece_m), math.ceil(abs(b[2] - a[2]) / ip.max_piece_z_m))
        for i in range(n):
            u0, u1 = i / n, (i + 1) / n
            p0 = np.add(a, np.multiply(np.subtract(b, a), u0))
            p1 = np.add(a, np.multiply(np.subtract(b, a), u1))
            t0, t1 = ta + (tb - ta) * u0, ta + (tb - ta) * u1
            slack = ip.slack_s + ip.slack_per_min * (t1 - m.release_t) / 60.0
            term = max(np.hypot(*(p0[:2] - pad)), np.hypot(*(p1[:2] - pad))) <= ip.terminal_radius_m
            lb = ip.terminal_lateral_buffer_m if (term and ip.terminal_lateral_buffer_m is not None) else ip.lateral_buffer_m
            boxes.append(Box4(m.id, k,
                              min(p0[0], p1[0]) - lb, max(p0[0], p1[0]) + lb,
                              min(p0[1], p1[1]) - lb, max(p0[1], p1[1]) + lb,
                              max(0.0, min(p0[2], p1[2]) - ip.vertical_buffer_m),
                              max(p0[2], p1[2]) + ip.vertical_buffer_m,
                              t0 - slack, t1 + slack, "terminal" if term else "en_route"))
            wis.append(wi)
    _flows(m, boxes, wis)
    return boxes


def _flows(m: Mission, boxes: list[Box4], wis: list[int]) -> None:
    """Tag departure boxes (terminal boxes before the first en-route box) and arrival boxes (legs after
    an approach fix, or terminal boxes after the last en-route box when the route has no fix)."""
    fix = next((i for i, w in enumerate(m.waypoints) if w.tag == "fix"), None)
    first_enr = next((k for k, b in enumerate(boxes) if b.phase == "en_route"), len(boxes))
    last_enr = max((k for k, b in enumerate(boxes) if b.phase == "en_route"), default=-1)
    for k, b in enumerate(boxes):
        if fix is not None and wis[k] > fix:
            b.flow = "arr"
        elif k < first_enr and b.phase == "terminal":
            b.flow = "dep"
        elif fix is None and k > last_enr and b.phase == "terminal":
            b.flow = "arr"


# ------------------------------------------------------------------ Python reference checker
def separated_ref(a: tuple, b: tuple, buf: int) -> bool:
    """Same decision as Utm.separated (proofs/UtmChecker.lean), on integer tuples (x0,x1,y0,y1,z0,z1,t0,t1)."""
    ax0, ax1, ay0, ay1, az0, az1, at0, at1 = a
    bx0, bx1, by0, by1, bz0, bz1, bt0, bt1 = b
    return (at1 < bt0 or bt1 < at0 or ax1 + buf <= bx0 or bx1 + buf <= ax0 or
            ay1 + buf <= by0 or by1 + buf <= ay0 or az1 + buf <= bz0 or bz1 + buf <= az0)


def separated_hv_ref(a: tuple, b: tuple, h: int, v: int) -> bool:
    """Same decision as Utm.separatedHV (cylinder form, Theorem 2)."""
    ax0, ax1, ay0, ay1, az0, az1, at0, at1 = a
    bx0, bx1, by0, by1, bz0, bz1, bt0, bt1 = b
    return (at1 < bt0 or bt1 < at0 or ax1 + h <= bx0 or bx1 + h <= ax0 or
            ay1 + h <= by0 or by1 + h <= ay0 or az1 + v <= bz0 or bz1 + v <= az0)


def separated_hv_matrix(A: np.ndarray, B: np.ndarray, h: int, v: int) -> np.ndarray:
    """Vectorised separated_hv_ref: (m,8) x (n,8) int64 -> (m,n) bool. Used by the scheduler's search;
    every accepted plan is re-checked by the Lean-verified checker in the audit."""
    A = np.asarray(A, np.int64).reshape(-1, 8)[:, None, :]
    B = np.asarray(B, np.int64).reshape(-1, 8)[None, :, :]
    return ((A[..., 7] < B[..., 6]) | (B[..., 7] < A[..., 6]) |
            (A[..., 1] + h <= B[..., 0]) | (B[..., 1] + h <= A[..., 0]) |
            (A[..., 3] + h <= B[..., 2]) | (B[..., 3] + h <= A[..., 2]) |
            (A[..., 5] + v <= B[..., 4]) | (B[..., 5] + v <= A[..., 4]))


def inflate_ref(b: tuple, eh: int, ev: int, dt: int) -> tuple:
    """Same as Utm.Box.inflate."""
    x0, x1, y0, y1, z0, z1, t0, t1 = b
    return (x0 - eh, x1 + eh, y0 - eh, y1 + eh, z0 - ev, z1 + ev, t0 - dt, t1 + dt)


def separated_dev_ref(a: tuple, b: tuple, h: int, v: int, eh: int, ev: int, dt: int) -> bool:
    """Same decision as Utm.separatedDev (Theorem 3): cylinder check on both boxes widened by the error bound."""
    return separated_hv_ref(inflate_ref(a, eh, ev, dt), inflate_ref(b, eh, ev, dt), h, v)


def inside_ref(b: tuple, m: int, x: int, y: int, z: int, t: int) -> bool:
    """Same decision as Utm.inside."""
    x0, x1, y0, y1, z0, z1, t0, t1 = b
    return (x0 + m <= x and x + m <= x1 and y0 + m <= y and y + m <= y1 and
            z0 + m <= z and z + m <= z1 and t0 <= t <= t1)
