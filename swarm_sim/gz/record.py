"""Headless video of a Gazebo world: a camera sensor, a paused world stepped once per frame, ffmpeg.

Each video frame: move the vehicles (GzView.apply) and the camera, step the world by one camera period
(`1 / fps` s of sim time, i.e. `steps` physics iterations), wait for the camera's image, pipe it to ffmpeg. The
result does not depend on how fast the machine renders (CPU llvmpipe in Docker is slow but deterministic).
GUI markers (intent boxes) are not in the image: the camera renders server entities only.
"""
from __future__ import annotations

import subprocess
import threading
from pathlib import Path

import numpy as np

from .models import camera_rig_sdf, rpy_to_quat

CAMERA = "swarm_camera"
TOPIC = "/swarm/camera"


def ffmpeg_cmd(out: str | Path, width: int, height: int, fps: float) -> list[str]:
    return ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
            "-r", f"{fps:g}", "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", str(out)]


def image_rgb(width: int, height: int, step: int, data: bytes) -> bytes:
    """Tightly packed RGB rows from a gz Image (rows may be padded to `step` bytes)."""
    if step in (0, width * 3):
        return bytes(data[: width * height * 3])
    a = np.frombuffer(data, np.uint8)[: step * height].reshape(height, step)[:, : width * 3]
    return a.tobytes()


class Recorder:
    def __init__(self, gz, out: str | Path, width: int = 1280, height: int = 720, fps: float = 10.0,
                 step_size_s: float = 0.004, pose=((0, 0, 100), (0, 0.5, 0)), timeout_s: float = 120.0):
        self.gz, self.w, self.h, self.fps = gz, width, height, fps
        self.steps = max(1, round(1.0 / fps / step_size_s))
        self.timeout = timeout_s
        self.frames = 0
        self._img = None
        self._new = threading.Event()
        gz.control(pause=True)
        pos, rpy = pose
        if not gz.create(camera_rig_sdf(CAMERA, TOPIC, width, height, fps), CAMERA, pos, rpy_to_quat(*rpy)):
            raise RuntimeError("could not spawn the camera rig")
        gz.subscribe(TOPIC, "image", "Image", self._on_image)
        self.ff = subprocess.Popen(ffmpeg_cmd(out, width, height, fps), stdin=subprocess.PIPE)
        self.out = Path(out)

    def _on_image(self, msg):
        self._img = (msg.width, msg.height, msg.step, bytes(msg.data))
        self._new.set()

    def camera(self, pos, rpy):
        self.gz.set_poses([(CAMERA, tuple(pos), rpy_to_quat(*rpy))])

    def capture(self) -> bool:
        self._new.clear()
        self.gz.control(multi_step=self.steps)
        if not self._new.wait(self.timeout):
            return False
        w, h, step, data = self._img
        if (w, h) != (self.w, self.h):
            return False
        self.ff.stdin.write(image_rgb(w, h, step, data))
        self.frames += 1
        return True

    def close(self):
        if self.ff.stdin:
            self.ff.stdin.close()
        self.ff.wait()
        try:
            self.gz.remove(CAMERA)
        except Exception:  # noqa: BLE001
            pass
