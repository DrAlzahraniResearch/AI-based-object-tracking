"""Tier B plant: PX4 SITL (SIH, no Gazebo) flown through MAVSDK.

One PX4 instance per vehicle slot (`px4 -i k`, MAVLink offboard port 14540 + k). Each vehicle runs the
same waypoint logic and guidance law as Tier A (`swarm_sim.sim.follow`, `_guidance`) and sends the
result to PX4 as offboard velocity setpoints, so the difference between a Tier A track and a Tier B
track is the autopilot (PX4's position/velocity controllers, estimator and SIH dynamics), not the
mission logic. Vehicles arm and hold zero velocity in offboard `arm_lead_s` before their release time (PX4 needs ~4 s
from the arm command to lift-off). Landing uses PX4's own LAND mode at the final waypoint.

Frames: PX4 local NED with its origin where the instance booted; the slot is mapped to the mission's
pad, so ENU = pad + (east, north, -down).

Clock: PX4 SIH runs in lockstep at `speed_factor` x real time. The plant clock is instance 0's
simulation time (MAVLink odometry time_usec); other instances are aligned to it once, when they
connect. Telemetry samples carry that clock.

Needs: a PX4-Autopilot build (`make px4_sitl_default`), `pip install mavsdk`. Docker: service `px4`.
"""
from __future__ import annotations

import asyncio
import math
import os
import queue
import shutil
import subprocess
import threading
import time
from pathlib import Path

import numpy as np

from ..missions import Mission
from ..sim import SimParams, _guidance, follow
from ..vehicle import VehicleParams
from .telemetry import Plant, Telemetry


def px4_root() -> Path:
    for c in (os.environ.get("PX4_ROOT"), "/opt/px4", str(Path.home() / "px4")):
        if c and (Path(c) / "build/px4_sitl_default/bin/px4").exists():
            return Path(c)
    raise FileNotFoundError("PX4 SITL build not found: set PX4_ROOT to a PX4-Autopilot tree built with "
                            "`make px4_sitl_default` (Docker: service px4)")


class Px4Plant(Plant):
    name = "px4"

    def __init__(self, n: int, vp: VehicleParams, sp: SimParams, speed_factor: float = 1.0,
                 rate_hz: float = 10.0, setpoint_hz: float = 20.0, workdir: str | os.PathLike = "/tmp/px4_twin",
                 model: str = "sihsim_quadx", launch: bool = True, arm_lead_s: float = 4.0, log=print,
                 start: bool = True):
        self.n, self.vp, self.sp = n, vp, sp
        self.speed, self.rate_hz, self.sp_hz = speed_factor, rate_hz, setpoint_hz
        self.workdir, self.model, self.log = Path(workdir), model, log
        self.arm_lead = arm_lead_s                     # arm and hold in offboard this long before release (spool-up)
        self.procs: list[subprocess.Popen] = []
        self.q: queue.Queue = queue.Queue()
        self.missions: dict[int, Mission] = {}
        self.route_ver = np.zeros(n, int)
        self.state = ["idle"] * n                      # idle | waiting | flying | landed | cancelled
        self.clock_off = [None] * n                    # instance sim time - plant time
        self.t_plant = 0.0
        self.t_end = math.inf
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.err: list = []
        self.track: list = []                          # (t, vid, x, y, z) at telemetry rate: the Tier B "truth"
        self.origin: dict[int, tuple] = {}             # ENU of each instance's local NED origin
        self.last_pos: dict[int, np.ndarray] = {}
        self.external_clock = False                    # True: t_plant comes from the simulator (Gazebo), not PX4
        self.keep_streams = True                       # keep a landed vehicle's telemetry (SIH: instance 0 is the clock)
        self.launch_flag = launch
        if start:
            self._start()

    def _start(self):
        if self.launch_flag:
            self._launch()
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        if not self.ready.wait(180):
            self.close()
            raise TimeoutError(f"PX4 instances not ready: {self.err}")

    # ------------------------------------------------------------------ processes
    def _launch(self):
        root = px4_root()
        try:
            subprocess.run(["pkill", "-x", "px4"], check=False)       # stale instances from an aborted run
            time.sleep(1.0)
        except FileNotFoundError:
            pass
        env = os.environ | {"PX4_SIM_MODEL": self.model, "PX4_SIM_SPEED_FACTOR": f"{self.speed:g}", "HEADLESS": "1"}
        for k in range(self.n):
            wd = self.workdir / f"instance_{k}"
            shutil.rmtree(wd, ignore_errors=True)
            wd.mkdir(parents=True)
            self.procs.append(subprocess.Popen(
                [str(root / "build/px4_sitl_default/bin/px4"), "-i", str(k), "-d",
                 str(root / "build/px4_sitl_default/etc")], cwd=wd, env=env,
                stdout=open(wd / "out.log", "w"), stderr=subprocess.STDOUT))

    def close(self):
        try:
            self.loop.call_soon_threadsafe(self.loop.stop)
        except RuntimeError:
            pass
        for p in self.procs:
            p.terminate()
        for p in self.procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()

    # ------------------------------------------------------------------ plant API (twin thread)
    def now(self):
        with self.lock:
            return self.t_plant

    def advance(self) -> bool:
        time.sleep(self.sp.dt / self.speed)            # the twin ticks on the plant clock
        if self.err:
            raise RuntimeError(f"PX4 plant: {self.err}")
        with self.lock:
            done = all(s in ("landed", "cancelled", "idle") for s in self.state)
            return not (done and self.t_plant >= self.t_end)

    def poll(self):
        out = []
        while True:
            try:
                out.append(self.q.get_nowait())
            except queue.Empty:
                return out

    def dispatch(self, vid, mission):
        with self.lock:
            self.missions[vid] = mission
            self.route_ver[vid] += 1
            self.state[vid] = "waiting"

    def set_route(self, vid, mission):
        with self.lock:
            self.missions[vid] = mission
            self.route_ver[vid] += 1

    def cancel(self, vid):
        with self.lock:
            if self.state[vid] in ("idle", "waiting"):
                self.state[vid] = "cancelled"

    def truth(self):
        return {}

    # ------------------------------------------------------------------ MAVSDK side (asyncio thread)
    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._main())
        except Exception as e:  # noqa: BLE001
            self.err.append(repr(e))
            self.ready.set()

    async def _connect(self, k: int):
        """MAVSDK link to instance k (offboard port 14540 + k), once it is healthy enough to arm."""
        from mavsdk import System
        d = System(port=50051 + k)
        await d.connect(system_address=f"udpin://0.0.0.0:{14540 + k}")
        async for s in d.core.connection_state():
            if s.is_connected:
                break
        async for h in d.telemetry.health():
            if h.is_local_position_ok and h.is_home_position_ok and h.is_armable:
                break
        await d.telemetry.set_rate_position_velocity_ned(self.rate_hz)
        await d.telemetry.set_rate_odometry(self.rate_hz)
        return d

    async def _main(self):
        systems = [await self._connect(k) for k in range(self.n)]
        self.ready.set()
        self.log(f"PX4 plant: {self.n} instance(s) ready, speed x{self.speed:g}")
        await asyncio.gather(*(self._vehicle(k, d) for k, d in enumerate(systems)))

    async def _vehicle(self, k: int, d):
        from mavsdk.offboard import VelocityNedYaw
        latest = {"pv": None, "t": None, "in_air": False}

        async def odo():
            async for o in d.telemetry.odometry():
                ts = o.time_usec / 1e6
                with self.lock:
                    if self.clock_off[k] is None:
                        self.clock_off[k] = ts - (self.t_plant if (k or self.external_clock) else 0.0)
                    latest["t"] = ts - self.clock_off[k]
                    if k == 0 and not self.external_clock:
                        self.t_plant = latest["t"]

        async def pvs():
            async for pv in d.telemetry.position_velocity_ned():
                latest["pv"] = pv

        async def air():
            async for a in d.telemetry.in_air():
                latest["in_air"] = a

        tasks = [asyncio.ensure_future(f()) for f in (odo, pvs, air)]
        while latest["t"] is None or latest["pv"] is None:
            await asyncio.sleep(0.05)
        # wait for a mission and its release time
        while True:
            with self.lock:
                st, m = self.state[k], self.missions.get(k)
            if st == "cancelled":
                await self._end_streams(tasks)                            # SIH: keep the clock running
                return
            if st == "waiting" and m is not None and latest["t"] >= m.release_t - self.arm_lead:
                break
            await asyncio.sleep(0.05)
        await d.action.arm()                                   # pre-flight: arm, hold zero velocity until release
        await d.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
        await d.offboard.start()
        while latest["t"] < m.release_t:
            await d.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
            await asyncio.sleep(0.05 / self.speed)

        def enu():
            pv = latest["pv"]
            pad = self.origin[k]
            return (np.array([pad[0] + pv.position.east_m, pad[1] + pv.position.north_m, pad[2] - pv.position.down_m]),
                    np.array([pv.velocity.east_m_s, pv.velocity.north_m_s, -pv.velocity.down_m_s]))

        with self.lock:
            pad = tuple(self.missions[k].meta.get("pad", self.missions[k].meta["start"]))
            o = self.origin.setdefault(k, pad)             # SIH: the instance is wherever its mission starts
            if math.dist(o[:2], pad[:2]) > 1.0:
                self.log(f"PX4 plant: vehicle {k} stands at {o[:2]}, its plan starts at {pad[:2]} "
                         f"({math.dist(o[:2], pad[:2]):.1f} m): expect a conformance alert at take-off")
            self.state[k] = "flying"
            ver = self.route_ver[k]
            m = self.missions[k]
        wi, hold_until = 0, -1.0
        period = 1.0 / self.sp_hz / self.speed
        last_tx = -1.0
        while True:
            with self.lock:
                if self.route_ver[k] != ver:                   # replanned by the twin
                    ver, m = self.route_ver[k], self.missions[k]
                    wi, hold_until = 0, -1.0
            t = latest["t"]
            p, v = enu()
            wi, hold_until, _adv = follow(m, wi, hold_until, p, t, self.sp)
            if wi >= len(m.waypoints) or (wi == len(m.waypoints) - 1 and m.waypoints[-1].tag == "land"
                                         and math.hypot(p[0] - m.waypoints[-1].x, p[1] - m.waypoints[-1].y) < 1.5
                                         and p[2] - m.waypoints[-1].z < 3.0):
                break
            w = m.waypoints[wi]
            cmd = _guidance(p, (w.x, w.y, w.z), self.vp)
            await d.offboard.set_velocity_ned(VelocityNedYaw(cmd[1], cmd[0], -cmd[2], 0.0))
            if t - last_tx >= 1.0 / self.rate_hz - 1e-6:
                last_tx = t
                self.q.put(Telemetry(k, t, p, v, "flying", wi, hold_until, int(ver)))
                self.track.append((t, k, *p))
                self.last_pos[k] = p
            await asyncio.sleep(period)
        # final approach: PX4 LAND mode
        await d.offboard.stop()
        await d.action.land()
        while latest["in_air"]:
            p, v = enu()
            t = latest["t"]
            if t - last_tx >= 1.0 / self.rate_hz - 1e-6:
                last_tx = t
                self.q.put(Telemetry(k, t, p, v, "flying", len(m.waypoints) - 1, -1.0, int(ver)))
                self.track.append((t, k, *p))
                self.last_pos[k] = p
            await asyncio.sleep(period)
        p, v = enu()
        self.q.put(Telemetry(k, latest["t"], p, np.zeros(3), "landed", len(m.waypoints), -1.0, int(ver)))
        with self.lock:
            self.state[k] = "landed"
            self.last_pos[k] = p
        await self._end_streams(tasks)

    async def _end_streams(self, tasks):
        """SIH: keep the telemetry streams alive, instance 0's odometry is the plant clock for everyone.
        Gazebo: the clock is the world's, so a finished vehicle's streams stop (and the instance is removed)."""
        if self.keep_streams:
            await asyncio.gather(*tasks, return_exceptions=True)
            return
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
