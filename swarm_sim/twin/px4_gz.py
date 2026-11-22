"""Tier B in Gazebo: PX4 SITL x500s flying in the OSM world (Gazebo Harmonic), flown through MAVSDK.

Same bridge as `Px4Plant` (shared waypoint logic and guidance, offboard velocity setpoints, PX4 LAND mode at the
pad); what changes is the simulator and the vehicles' life cycle:

* The world is `world_sdf(air, px4=True)`: the OSM buildings with collision geometry, hubs, no-fly volumes, and the
  systems PX4's sensors need. Its `<spherical_coordinates>` put PX4's GPS where the OSM extract is.
* The plant clock is the world's sim time (`/world/<w>/clock`), not PX4's: vehicles come and go.
* A vehicle is spawned `spawn_lead_s` before its filed release time, at its filed pad, once the pad is free (no
  other vehicle standing on it or below 5 m within `pad_clear_m`): one PX4 instance per vehicle with
  PX4_GZ_STANDALONE=1 and PX4_GZ_MODEL_POSE. Its local NED origin is the spawn point.
* When it has landed (or its request is cancelled), its PX4 instance stops and its model is removed, which frees the
  pad for the next one: a delivery that returns to its hub would otherwise sit on a pad for the rest of the run.
* Collisions are real: an x500 that leaves its intent into a building hits it.

If the scheduler moves a flight to another pad of the same hub after the vehicle has been spawned, the vehicle takes
off from where it stands; the plant logs it and the twin sees it as non-conformance at take-off.

Needs: a PX4-Autopilot build with the Gazebo bridge (`make px4_sitl_default` with gz-harmonic installed), the `gz`
CLI, MAVSDK. Docker: service `px4gz`.
"""
from __future__ import annotations

import asyncio
import math
import os
import shutil
import subprocess
import time
from pathlib import Path

from ..missions import Mission
from ..osm import Airspace, world_sdf
from ..sim import SimParams
from ..vehicle import VehicleParams
from .px4 import Px4Plant, px4_root


def pad_free(pad, others: dict, clear_m: float, alt_m: float = 5.0) -> bool:
    """No other vehicle within clear_m of the pad horizontally and below alt_m above it."""
    for p in others.values():
        if math.hypot(p[0] - pad[0], p[1] - pad[1]) < clear_m and p[2] < pad[2] + alt_m:
            return False
    return True


def instance_env(k: int, pos, model: str, speed: float, gz_env: dict) -> dict:
    return gz_env | {"PX4_SIM_MODEL": model, "PX4_GZ_STANDALONE": "1", "HEADLESS": "1",
                     "PX4_GZ_MODEL_POSE": ",".join(f"{v:.3f}" for v in pos), "PX4_SIM_SPEED_FACTOR": f"{speed:g}"}


class Px4GzPlant(Px4Plant):
    name = "px4gz"
    poll_s = 0.2                                        # wall seconds between spawn checks

    def __init__(self, requests: list[Mission], air: Airspace, vp: VehicleParams, sp: SimParams,
                 speed_factor: float = 1.0, rate_hz: float = 10.0, setpoint_hz: float = 20.0,
                 workdir: str | os.PathLike = "/tmp/px4gz_twin", model: str = "gz_x500", spawn_lead_s: float = 45.0,
                 pad_clear_m: float = 4.0, spawn_dz_m: float = 0.2, gui: bool = False, arm_lead_s: float = 4.0,
                 log=print):
        super().__init__(len(requests), vp, sp, speed_factor, rate_hz, setpoint_hz, workdir, model,
                         launch=True, arm_lead_s=arm_lead_s, log=log, start=False)
        if sorted(m.id for m in requests) != list(range(len(requests))):
            raise ValueError("Px4GzPlant: request ids must be 0..n-1 (one PX4 instance per request)")
        self.requests = {m.id: m for m in requests}
        self.air, self.world = air, air.name
        self.spawn_lead, self.pad_clear, self.spawn_dz, self.gui = spawn_lead_s, pad_clear_m, spawn_dz_m, gui
        self.external_clock, self.keep_streams = True, False
        self.px4: dict[int, subprocess.Popen] = {}
        self.spawned: dict[int, tuple] = {}             # vehicles currently in the world -> spawn point
        self.removed: list[int] = []
        self.gz_procs: list = []
        self.clock = None
        self._start()

    @property
    def model_base(self) -> str:
        return self.model.removeprefix("gz_")

    # ------------------------------------------------------------------ processes
    def _launch(self):
        from ..gz import launch
        from ..gz.transport import GzClock
        self.root = px4_root()
        self.gz_env = launch.px4_gz_env(self.root)
        if not (Path(self.gz_env["PX4_GZ_MODELS"]) / self.model_base / "model.sdf").exists():
            raise FileNotFoundError(f"PX4 Gazebo model {self.model_base} not found under {self.gz_env['PX4_GZ_MODELS']}"
                                    " (the Tools/simulation/gz submodule)")
        subprocess.run(["pkill", "-x", "px4"], check=False)
        self.workdir.mkdir(parents=True, exist_ok=True)
        wpath = self.workdir / f"{self.world}.sdf"
        wpath.write_text(world_sdf(self.air, px4=True, real_time_factor=self.speed))
        self.gz_procs = launch.start(wpath, self.world, run=True, gui=self.gui, env=self.gz_env, logdir=self.workdir)
        self.clock = GzClock(self.world, self._on_clock)
        t0 = time.monotonic()
        while self.clock.t <= 0.0:
            if time.monotonic() - t0 > 30:
                raise TimeoutError(f"no clock from Gazebo world {self.world}")
            time.sleep(0.1)
        self.log(f"PX4/Gazebo plant: world {self.world} up ({wpath}), x{self.speed:g}")

    def _on_clock(self, t: float):
        with self.lock:
            self.t_plant = t

    def _spawn_px4(self, k: int, pos):
        wd = self.workdir / f"instance_{k}"
        shutil.rmtree(wd, ignore_errors=True)
        wd.mkdir(parents=True)
        env = os.environ | instance_env(k, pos, self.model, self.speed, self.gz_env)
        self.px4[k] = subprocess.Popen(
            [str(self.root / "build/px4_sitl_default/bin/px4"), "-i", str(k), "-d",
             str(self.root / "build/px4_sitl_default/etc")], cwd=wd, env=env,
            stdout=open(wd / "out.log", "w"), stderr=subprocess.STDOUT)

    def _despawn(self, k: int):
        from ..gz.transport import cli_remove_model
        p = self.px4.pop(k, None)
        if p is not None:
            p.terminate()
            try:
                p.wait(10)
            except subprocess.TimeoutExpired:
                p.kill()
        if not cli_remove_model(self.world, f"{self.model_base}_{k}"):
            self.log(f"PX4/Gazebo plant: could not remove {self.model_base}_{k}")
        with self.lock:
            self.spawned.pop(k, None)
            self.removed.append(k)

    def advance(self) -> bool:
        """As Px4Plant, but not finished while a landed vehicle is still being removed: otherwise the run ends and
        the world shuts down under the removal."""
        alive = super().advance()
        if not alive:
            with self.lock:
                return bool(self.spawned)
        return True

    def close(self):
        try:
            self.loop.call_soon_threadsafe(self.loop.stop)
        except (RuntimeError, AttributeError):
            pass
        for p in self.px4.values():
            p.terminate()
        for p in self.px4.values():
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
        if self.clock is not None:
            self.clock.close()
        from ..gz import launch
        launch.stop(self.gz_procs)

    # ------------------------------------------------------------------ life cycle (asyncio thread)
    async def _main(self):
        self.ready.set()
        await asyncio.gather(*(self._lifecycle(k) for k in range(self.n)))

    def _occupied(self, k: int) -> dict:
        return {j: self.last_pos.get(j, pos) for j, pos in self.spawned.items() if j != k}

    async def _lifecycle(self, k: int):
        m = self.requests[k]
        pad = tuple(float(v) for v in m.meta.get("pad", m.meta["start"]))
        spawn_at = m.release_t - self.spawn_lead
        waited = False
        while True:
            with self.lock:
                st, t = self.state[k], self.t_plant
                free = pad_free(pad, self._occupied(k), self.pad_clear)
            if st == "cancelled":
                return
            if t >= spawn_at:
                if free:
                    break
                if not waited:
                    self.log(f"PX4/Gazebo plant: vehicle {k} waits for its pad {pad[:2]} to clear")
                    waited = True
            await asyncio.sleep(self.poll_s)
        pos = (pad[0], pad[1], pad[2] + self.spawn_dz)
        with self.lock:
            self.spawned[k] = pad
            self.origin[k] = pad
        await asyncio.to_thread(self._spawn_px4, k, pos)
        self.log(f"PX4/Gazebo plant: t={self.t_plant:7.1f}  spawned {self.model_base}_{k} at {pad}")
        try:
            d = await self._connect(k)
            await self._vehicle(k, d)
        finally:
            await asyncio.to_thread(self._despawn, k)
