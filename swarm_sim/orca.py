"""Decentralized separation: ORCA (Optimal Reciprocal Collision Avoidance) in the horizontal plane.

Each airborne vehicle independently picks the velocity closest to its route-following velocity that lies
outside every neighbour's velocity obstacle over a horizon `tau`, assuming every neighbour does the
same and takes its share of the avoidance (van den Berg, Guy, Lin & Manocha, "Reciprocal n-body
collision avoidance", ISRR 2009; the linear programs follow the RVO2 library).

Choices for UTM traffic (all in `OrcaParams`):

* Horizontal ORCA, vertical gate. Cruise layers already separate opposite headings vertically, and
  PX4's vertical limits are asymmetric and 4-8x smaller than the horizontal ones. A neighbour counts
  only if it is (or will be within `tau`) vertically closer than `v_sep_m + v_margin_m`. The gate must
  stay below the layer step (15 m), or ORCA treats traffic on the adjacent layer, which is already
  separated, as a conflict (that made 51 % of agent-steps constrained and caused stand-offs).
* Combined radius `h_sep_m + h_margin_m`: ORCA keeps neighbours outside the loss-of-separation
  cylinder, not merely outside a collision radius.
* Priority shares: a higher-priority flight (emergency) takes `priority_share` of the avoidance, the
  other flight the rest; equal priorities split 50/50 (classic ORCA).
* Conformance corridor: a hard half-plane keeps the horizontal distance from the current leg
  segment (a point for vertical legs and holds) under `corridor_m` (inside the 20 m lateral intent
  buffer and the 15 m building-clearance corridor of the router), with a barrier-style pull back of
  gain `corridor_gain`. Around a point (vertical legs, holds, leg ends) a half-plane still allows
  sliding sideways, which spirals outwards, so the tangential part of the velocity is also clipped
  so that the offset predicted `corridor_lookahead_s` ahead stays within max(offset, corridor).
  Tactical avoidance must not break the strategic contract the audit checks: where the two conflict,
  conformance wins and the loss of separation is recorded.
* Terminal exclusion: vehicles below `min_alt_m` or within `terminal_radius_m` of a hub (take-off,
  landing, pad work) are neither controlled nor avoided; hub sequencing is plan step 7.
* Right of way when stalled: a vehicle that wants to move (> `stall_want_ms`) but has not come
  `stall_progress_m` closer to its current waypoint for `stall_s` is "stalled" (this catches both
  stand-offs and jitter). Against a non-stalled neighbour it gets right of
  way; if both are stalled, the higher-priority one (then the lower mission index) does. The vehicle
  with right of way drops that neighbour's constraint and proceeds; the other takes the whole
  avoidance. The rule is symmetric, so both vehicles agree. Without it, two vehicles whose corridors
  overlap (e.g. two emergency holds 18 m apart) block each other indefinitely; with it the flights
  progress and, if the yielding vehicle cannot clear inside its corridor, the loss of separation is
  recorded. Overlapping intents like that are the scheduler's job (milestone 4), not ORCA's.
* Building / no-fly guard: if the new velocity would put the vehicle inside a building or no-fly
  zone within `guard_s`, the route velocity (which is clearance-checked) is used instead.

Neighbour state comes from the fleet, i.e. perfect broadcast (Remote ID / ADS-B-like) with no
latency or noise. That is an idealisation to relax later.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np
from scipy.spatial import cKDTree

EPS = 1e-9


@dataclass(frozen=True)
class OrcaParams:
    tau_s: float = 10.0             # look-ahead for neighbours
    sense_m: float = 250.0          # neighbour (broadcast) range
    max_neighbours: int = 12
    h_sep_m: float = 30.0           # = SimParams.h_sep_m
    h_margin_m: float = 5.0
    v_sep_m: float = 10.0           # = SimParams.v_sep_m
    v_margin_m: float = 3.0         # gate = v_sep + margin = 13 m < 15 m layer step
    min_alt_m: float = 15.0
    terminal_radius_m: float = 40.0 # = SimParams.terminal_radius_m
    priority_share: float = 0.25    # share of avoidance taken by the higher-priority vehicle
    corridor_m: float = 15.0        # max cross-track offset (intent lateral buffer is 20 m)
    corridor_gain: float = 0.5      # 1/s
    corridor_lookahead_s: float = 1.0
    guard_s: float = 3.0
    stall_s: float = 10.0
    stall_progress_m: float = 1.0
    stall_want_ms: float = 1.0


@dataclass
class Line:
    point: np.ndarray
    direction: np.ndarray


def _det(a, b) -> float:
    return a[0] * b[1] - a[1] * b[0]


# -------------------------------------------------------------- linear programs (RVO2)
def _lp1(lines, k, radius, opt, direction_opt):
    pt, d = lines[k].point, lines[k].direction
    dot = pt @ d
    disc = dot * dot + radius * radius - pt @ pt
    if disc < 0:
        return None
    s = math.sqrt(disc)
    t_left, t_right = -dot - s, -dot + s
    for i in range(k):
        den = _det(d, lines[i].direction)
        num = _det(lines[i].direction, pt - lines[i].point)
        if abs(den) <= EPS:
            if num < 0:
                return None
            continue
        t = num / den
        if den >= 0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)
        if t_left > t_right:
            return None
    if direction_opt:
        return pt + (t_right if opt @ d > 0 else t_left) * d
    t = d @ (opt - pt)
    return pt + min(max(t, t_left), t_right) * d


def _lp2(lines, radius, opt, direction_opt):
    """Returns (index of first failing line or len(lines), result)."""
    if direction_opt:
        result = opt * radius
    elif opt @ opt > radius * radius:
        result = opt / np.linalg.norm(opt) * radius
    else:
        result = opt.copy()
    for i, ln in enumerate(lines):
        if _det(ln.direction, ln.point - result) > 0:
            r = _lp1(lines, i, radius, opt, direction_opt)
            if r is None:
                return i, result
            result = r
    return len(lines), result


def _lp3(lines, n_hard, begin, radius, result):
    """Infeasible case: minimise the maximum violation of the soft lines, keeping the first n_hard."""
    distance = 0.0
    for i in range(begin, len(lines)):
        li = lines[i]
        if _det(li.direction, li.point - result) > distance:
            proj = list(lines[:n_hard])
            for j in range(n_hard, i):
                lj = lines[j]
                den = _det(li.direction, lj.direction)
                if abs(den) <= EPS:
                    if li.direction @ lj.direction > 0:
                        continue
                    point = 0.5 * (li.point + lj.point)
                else:
                    point = li.point + (_det(lj.direction, li.point - lj.point) / den) * li.direction
                dn = lj.direction - li.direction
                proj.append(Line(point, dn / max(np.linalg.norm(dn), EPS)))
            fail, r = _lp2(proj, radius, np.array([-li.direction[1], li.direction[0]]), True)
            if fail >= len(proj):
                result = r
            distance = _det(li.direction, li.point - result)
    return result


def solve(hard: list[Line], soft: list[Line], radius: float, opt: np.ndarray) -> tuple[np.ndarray, bool]:
    """Velocity closest to `opt` within |v| <= radius satisfying all lines (left of each direction).
    Hard lines are never relaxed; if soft lines cannot all hold, their worst violation is minimised.
    Returns (velocity, feasible)."""
    lines = hard + soft
    fail, result = _lp2(lines, radius, opt, False)
    if fail < len(lines):
        return _lp3(lines, len(hard), fail, radius, result), False
    return result, True


def orca_line(p_i, v_i, p_j, v_j, r, tau, dt, share=0.5) -> Line:
    """ORCA half-plane for agent i induced by j (2D). `share`: fraction of the avoidance i takes."""
    rel_p = p_j - p_i
    rel_v = v_i - v_j
    dist2 = rel_p @ rel_p
    r2 = r * r
    if dist2 > r2:
        w = rel_v - rel_p / tau
        w2 = w @ w
        dot1 = w @ rel_p
        if dot1 < 0 and dot1 * dot1 > r2 * w2:          # project on the cut-off circle
            wl = math.sqrt(w2)
            uw = w / max(wl, EPS)
            direction = np.array([uw[1], -uw[0]])
            u = (r / tau - wl) * uw
        else:                                           # project on a leg of the cone
            leg = math.sqrt(dist2 - r2)
            if _det(rel_p, w) > 0:
                direction = np.array([rel_p[0] * leg - rel_p[1] * r, rel_p[0] * r + rel_p[1] * leg]) / dist2
            else:
                direction = -np.array([rel_p[0] * leg + rel_p[1] * r, -rel_p[0] * r + rel_p[1] * leg]) / dist2
            u = (rel_v @ direction) * direction - rel_v
    else:                                               # already inside: resolve within one step
        w = rel_v - rel_p / dt
        wl = math.sqrt(w @ w)
        uw = w / max(wl, EPS)
        direction = np.array([uw[1], -uw[0]])
        u = (r / dt - wl) * uw
    return Line(v_i + share * u, direction)


def halfplane_max(n: np.ndarray, c: float) -> Line:
    """Hard constraint v . n <= c (n unit)."""
    return Line(c * n, np.array([-n[1], n[0]]))


# -------------------------------------------------------------- policy
class OrcaPolicy:
    name = "orca"

    def __init__(self, params: OrcaParams = OrcaParams(), dt: float = 0.1):
        self.p = params
        self.dt = dt
        self.stats = {"agent_steps": 0, "constrained": 0, "infeasible": 0, "corridor_active": 0, "corridor_clip": 0,
                      "guard_fallback": 0,
                      "stalled_agent_steps": 0, "right_of_way_pairs": 0}
        self.stall_t = None        # per vehicle: seconds without progress
        self.best_d = None         # per vehicle: closest distance to the current target so far
        self.target = None

    def _share(self, i, j, pr_i, pr_j, stalled) -> float:
        """Fraction of the avoidance taken by i against j; share(i,j) + share(j,i) == 1."""
        si, sj = bool(stalled[i]), bool(stalled[j])
        if si != sj:
            return 0.0 if si else 1.0
        if si and sj:
            i_first = (pr_i, i) < (pr_j, j)
            return 0.0 if i_first else 1.0
        if pr_i < pr_j:
            return self.p.priority_share
        if pr_i > pr_j:
            return 1.0 - self.p.priority_share
        return 0.5

    def adjust(self, fleet, ids, desired, ctx=None):
        p = self.p
        ctx = ctx or {}
        out = np.array(desired, float, copy=True)
        if len(ids) == 0:
            return out
        pos, vel = fleet.pos, fleet.vel
        n = len(pos)
        if self.stall_t is None or len(self.stall_t) != n:
            self.stall_t, self.best_d, self.target = np.zeros(n), np.full(n, np.inf), np.full((n, 3), np.nan)
        want = np.linalg.norm(out[:, :2], axis=1) > p.stall_want_ms
        tgt = ctx.get("leg_b")
        if tgt is not None:
            new = ~np.all(np.isclose(self.target[ids], tgt), axis=1)
            self.target[ids] = tgt
            self.best_d[ids[new]] = np.inf
            self.stall_t[ids[new]] = 0.0
            d = np.linalg.norm(pos[ids] - tgt, axis=1)
            better = d < self.best_d[ids] - p.stall_progress_m
            self.best_d[ids] = np.where(better, d, self.best_d[ids])
            self.stall_t[ids] = np.where(want & ~better, self.stall_t[ids] + self.dt, 0.0)
        stalled = self.stall_t >= p.stall_s
        self.stats["stalled_agent_steps"] += int(stalled[ids].sum())
        sel = pos[ids, 2] >= p.min_alt_m                      # controlled and visible vehicles
        hubs = ctx.get("hubs_xy")
        if hubs is not None and len(hubs):
            dh = np.min(np.linalg.norm(pos[ids, None, :2] - hubs[None, :, :], axis=2), axis=1)
            sel &= dh >= p.terminal_radius_m
        ctrl = ids[sel]
        if len(ctrl) == 0:
            return out
        tree = cKDTree(pos[ctrl, :2])
        k_of = {int(i): k for k, i in enumerate(ids)}
        prio = ctx.get("priority")
        leg_a, leg_b = ctx.get("leg_a"), ctx.get("leg_b")
        air = ctx.get("airspace")
        r = p.h_sep_m + p.h_margin_m
        vmax = fleet.p.v_xy_max
        for a, i in enumerate(ctrl):
            k = k_of[int(i)]
            self.stats["agent_steps"] += 1
            pi, vi = pos[i, :2], vel[i, :2]
            soft = []
            nb = tree.query_ball_point(pi, p.sense_m)
            cands = []
            for b in nb:
                if b == a:
                    continue
                j = ctrl[b]
                dz = pos[j, 2] - pos[i, 2]
                dvz = vel[j, 2] - vel[i, 2]
                closing = max(0.0, -math.copysign(1.0, dz) * dvz) * p.tau_s
                if abs(dz) - closing >= p.v_sep_m + p.v_margin_m:
                    continue
                cands.append((float(np.sum((pos[j, :2] - pi) ** 2)), int(j)))
            for _, j in sorted(cands)[:p.max_neighbours]:
                pa, pb = (prio[k], prio[k_of[j]]) if prio is not None else (0, 0)
                share = self._share(int(i), j, pa, pb, stalled)
                if stalled[i] or stalled[j]:
                    self.stats["right_of_way_pairs"] += 1
                    if share == 0.0:
                        continue          # right of way: proceed; j takes the whole avoidance
                soft.append(orca_line(pi, vi, pos[j, :2], vel[j, :2], r, p.tau_s, self.dt, share))
            hard = []
            clip = None                                            # (offset vector, offset) around a point
            if leg_a is not None:
                A, B = leg_a[k, :2], leg_b[k, :2]
                d = B - A
                L2 = float(d @ d)
                u = 0.0 if L2 < 1e-6 else min(1.0, max(0.0, float((pi - A) @ d) / L2))
                ev = pi - (A + u * d)                          # from the closest point of the leg
                e = float(np.hypot(*ev))
                if e > 1e-3:
                    c = max(p.corridor_gain * (p.corridor_m - e), -0.5 * vmax)
                    if e > 0.5 * p.corridor_m:
                        self.stats["corridor_active"] += 1
                    hard.append(halfplane_max(ev / e, c))
                    if u <= 0.0 or u >= 1.0:
                        clip = (ev, e)
            if not soft:
                if hard:
                    v2, _ = solve(hard, [], vmax, out[k, :2])
                    out[k, :2] = self._clip(v2, clip)
                continue
            self.stats["constrained"] += 1
            v2, ok = solve(hard, soft, vmax, out[k, :2])
            if not ok:
                self.stats["infeasible"] += 1
            v2 = self._clip(v2, clip)
            if air is not None:
                q = pos[i] + np.array([v2[0], v2[1], out[k, 2]]) * p.guard_s
                if air.inside_building(*q) or air.in_no_fly(*q):
                    self.stats["guard_fallback"] += 1
                    continue
            out[k, :2] = v2
        return out

    def _clip(self, v, clip):
        """Limit the tangential velocity around a point so the predicted offset does not grow past
        max(current offset, corridor)."""
        if clip is None:
            return v
        ev, e = clip
        h = self.p.corridor_lookahead_s
        r_hat = ev / e
        vr = float(v @ r_hat)
        vt = v - vr * r_hat
        rm = e + vr * h
        lim = max(e, self.p.corridor_m)
        tt = float(np.hypot(*vt)) * h
        if rm * rm + tt * tt <= lim * lim or tt < 1e-9:
            return v
        self.stats["corridor_clip"] += 1
        s = math.sqrt(max(0.0, lim * lim - rm * rm)) / tt
        return vr * r_hat + s * vt

    def info(self) -> dict:
        return {"params": asdict(self.p), "stats": dict(self.stats)}
