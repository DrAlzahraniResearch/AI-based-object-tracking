"""Starting `gz sim` (server, optional GUI) and PX4's Gazebo environment."""
from __future__ import annotations

import os
import subprocess
from pathlib import Path

from .transport import wait_for_world


def px4_gz_env(px4_root: str | os.PathLike, build: str = "px4_sitl_default") -> dict:
    """What PX4's gz_env.sh exports, minus GZ_SIM_SERVER_CONFIG_PATH: our worlds list their own systems."""
    root = Path(px4_root)
    models = root / "Tools/simulation/gz/models"
    worlds = root / "Tools/simulation/gz/worlds"
    plugins = root / "build" / build / "src/modules/simulation/gz_plugins"

    def join(var, *paths):
        return os.pathsep.join([p for p in (os.environ.get(var, ""), *map(str, paths)) if p])
    return {"PX4_GZ_MODELS": str(models), "PX4_GZ_WORLDS": str(worlds), "PX4_GZ_PLUGINS": str(plugins),
            "GZ_SIM_RESOURCE_PATH": join("GZ_SIM_RESOURCE_PATH", models, worlds),
            "GZ_SIM_SYSTEM_PLUGIN_PATH": join("GZ_SIM_SYSTEM_PLUGIN_PATH", plugins)}


def server_cmd(world_sdf: str | os.PathLike, run: bool = True, headless_rendering: bool = True,
               verbose: int = 1) -> list[str]:
    cmd = ["gz", "sim", "-s", f"-v{verbose}"]
    if run:
        cmd.append("-r")
    if headless_rendering:
        cmd.append("--headless-rendering")       # EGL, no display: camera sensors render in a container
    return cmd + [str(world_sdf)]


def start(world_sdf: str | os.PathLike, world_name: str, run: bool = True, gui: bool = False,
          headless_rendering: bool = True, env: dict | None = None, logdir: str | os.PathLike | None = None,
          timeout_s: float = 180.0) -> list[subprocess.Popen]:
    """Start the server (and the GUI client) and wait until the world answers. Returns the processes."""
    env = os.environ | (env or {})
    logdir = Path(logdir) if logdir else Path(world_sdf).parent
    logdir.mkdir(parents=True, exist_ok=True)
    procs = [subprocess.Popen(server_cmd(world_sdf, run, headless_rendering), env=env,
                              stdout=open(logdir / "gz_server.log", "w"), stderr=subprocess.STDOUT)]
    try:
        wait_for_world(world_name, timeout_s, procs)
    except Exception:
        stop(procs)
        raise
    if gui:
        procs.append(subprocess.Popen(["gz", "sim", "-g", "-v1"], env=env,
                                      stdout=open(logdir / "gz_gui.log", "w"), stderr=subprocess.STDOUT))
    return procs


def stop(procs: list[subprocess.Popen]):
    for p in reversed(procs):
        if p.poll() is None:
            p.terminate()
    for p in procs:
        try:
            p.wait(10)
        except subprocess.TimeoutExpired:
            p.kill()
