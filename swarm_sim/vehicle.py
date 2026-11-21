"""Tier A vehicle model and the vehicle interface shared with Tier B.

Tier A: vectorised point-mass multicopters that track a velocity setpoint
under PX4-like limits (horizontal/vertical speed, acceleration, a first-order
response). Deterministic, headless, hundreds of vehicles on a laptop CPU.

Defaults follow PX4 multicopter position-control parameters for an x500-class
quadrotor (MPC_XY_VEL_MAX 12 m/s, MPC_Z_VEL_MAX_UP 3, MPC_Z_VEL_MAX_DN 1.5,
MPC_ACC_HOR 3 m/s^2, MPC_ACC_UP_MAX 4, MPC_ACC_DOWN_MAX 3). They are a
starting point: calibrate them against PX4 SITL logs (Tier B) before trusting
Tier A numbers quantitatively.

The traffic algorithms only see `VehicleAPI`, so the same routing,
separation and scheduling code drives Tier A and Tier B (PX4 via MAVSDK).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VehicleParams:
    v_xy_max: float = 12.0      # m/s  (MPC_XY_VEL_MAX)
    v_up_max: float = 3.0       # m/s  (MPC_Z_VEL_MAX_UP)
    v_down_max: float = 1.5     # m/s  (MPC_Z_VEL_MAX_DN)
    a_xy_max: float = 3.0       # m/s^2 (MPC_ACC_HOR)
    a_up_max: float = 4.0       # m/s^2 (MPC_ACC_UP_MAX)
    a_down_max: float = 3.0     # m/s^2 (MPC_ACC_DOWN_MAX)
    tau: float = 0.4            # s, first-order velocity response
    cruise_xy: float = 10.0     # m/s, nominal cruise used for planning


class VehicleAPI:
    """What traffic algorithms may use. Tier A and Tier B both implement it."""

    def positions(self) -> np.ndarray: ...            # (N,3) ENU metres
    def velocities(self) -> np.ndarray: ...           # (N,3) m/s
    def active(self) -> np.ndarray: ...               # (N,) bool: airborne / in mission
    def command_velocity(self, ids, v) -> None: ...   # velocity setpoints, ENU m/s
    def time(self) -> float: ...


class KinematicFleet(VehicleAPI):
    """Tier A. State arrays for all N vehicles; `step` integrates dt seconds."""

    def __init__(self, n: int, params: VehicleParams = VehicleParams(), start=None):
        self.p = VehicleParams() if params is None else params
        self.pos = np.zeros((n, 3)) if start is None else np.array(start, float).reshape(n, 3)
        self.vel = np.zeros((n, 3))
        self.cmd = np.zeros((n, 3))
        self.on = np.zeros(n, bool)
        self.v_cap = np.full(n, np.inf)      # per-vehicle horizontal speed cap (degraded vehicles; twin rollouts)
        self.t = 0.0

    # -------------------------------------------------------------- API
    def positions(self):
        return self.pos

    def velocities(self):
        return self.vel

    def active(self):
        return self.on

    def time(self):
        return self.t

    def command_velocity(self, ids, v):
        self.cmd[ids] = self._limit_velocity(np.atleast_2d(np.asarray(v, float)), ids)

    # -------------------------------------------------------------- dynamics
    def _limit_velocity(self, v, ids=None):
        v = v.copy()
        sp = np.linalg.norm(v[:, :2], axis=1, keepdims=True)
        vmax = self.p.v_xy_max if ids is None else np.minimum(self.p.v_xy_max, self.v_cap[ids])[:, None]
        v[:, :2] *= np.minimum(1.0, vmax / np.maximum(sp, 1e-9))
        v[:, 2] = np.clip(v[:, 2], -self.p.v_down_max, self.p.v_up_max)
        return v

    def step(self, dt: float):
        a = (self.cmd - self.vel) / max(self.p.tau, dt)
        an = np.linalg.norm(a[:, :2], axis=1, keepdims=True)
        a[:, :2] *= np.minimum(1.0, self.p.a_xy_max / np.maximum(an, 1e-9))
        a[:, 2] = np.clip(a[:, 2], -self.p.a_down_max, self.p.a_up_max)
        a[~self.on] = 0.0
        self.vel += a * dt
        self.vel[~self.on] = 0.0
        self.pos += self.vel * dt
        self.pos[:, 2] = np.maximum(self.pos[:, 2], 0.0)       # the ground
        self.t += dt


class MavsdkFleet(VehicleAPI):  # pragma: no cover - Tier B
    """Superseded by `swarm_sim.twin.px4.Px4Plant` (milestone 4.5): PX4 SITL instances flown through MAVSDK
    behind the twin's plant interface (telemetry in, dispatch / set_route out)."""

    def __init__(self, *_, **__):
        raise NotImplementedError("use swarm_sim.twin.px4.Px4Plant (see docs/twin.md)")
