"""SDF snippets spawned at run time, and camera placement.

Vehicle markers are visual-only (no collision), non-static links with gravity off: the viewer teleports them with
`set_pose_vector` every frame and physics leaves them where they were put. They are drawn `scale` x the size of a
real 0.5 m quadrotor so that they stay visible from an overview camera over a 1-2 km town.
"""
from __future__ import annotations

import math

import numpy as np


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) for SDF's fixed-axis roll-pitch-yaw."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def _mat(rgb, alpha=1.0, emissive=0.25):
    r, g, b = (c / 255.0 for c in rgb)
    return (f"<material><ambient>{r:.3f} {g:.3f} {b:.3f} {alpha}</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} {alpha}</diffuse>"
            f"<emissive>{r * emissive:.3f} {g * emissive:.3f} {b * emissive:.3f} 1</emissive></material>")


def drone_marker_sdf(name: str, rgb=(40, 180, 80), scale: float = 4.0) -> str:
    """A quadrotor-shaped marker: body, two crossed arms, four rotor discs, in one colour."""
    s = scale
    arm = 0.25 * s
    rotors = "".join(
        f'<visual name="rotor{i}"><pose>{arm * math.cos(a):.3f} {arm * math.sin(a):.3f} {0.05 * s:.3f} 0 0 0</pose>'
        f'<geometry><cylinder><radius>{0.12 * s:.3f}</radius><length>{0.01 * s:.3f}</length></cylinder></geometry>'
        f'{_mat((40, 40, 40), 0.8, 0)}<cast_shadows>false</cast_shadows></visual>'
        for i, a in enumerate(np.deg2rad([45, 135, 225, 315])))
    arms = "".join(
        f'<visual name="arm{i}"><pose>0 0 {0.02 * s:.3f} 0 0 {yaw:.4f}</pose>'
        f'<geometry><box><size>{0.75 * s:.3f} {0.04 * s:.3f} {0.03 * s:.3f}</size></box></geometry>{_mat((60, 60, 60))}</visual>'
        for i, yaw in enumerate((math.pi / 4, -math.pi / 4)))
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>false</static>
    <link name="body">
      <gravity>false</gravity>
      <inertial><mass>1.0</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
      <visual name="hull"><geometry><box><size>{0.22 * s:.3f} {0.22 * s:.3f} {0.1 * s:.3f}</size></box></geometry>{_mat(rgb)}</visual>
      <visual name="nose"><pose>{0.14 * s:.3f} 0 0 0 0 0</pose><geometry><sphere><radius>{0.05 * s:.3f}</radius></sphere></geometry>{_mat(rgb)}</visual>
      {arms}{rotors}
    </link>
  </model>
</sdf>
"""


def camera_rig_sdf(name: str, topic: str, width: int, height: int, fps: float, hfov: float = 1.1) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <gravity>false</gravity>
      <inertial><mass>1.0</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
      <sensor name="camera" type="camera">
        <always_on>true</always_on>
        <update_rate>{fps:g}</update_rate>
        <topic>{topic}</topic>
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
          <clip><near>0.5</near><far>5000</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""


def look_at(eye, target) -> tuple[np.ndarray, tuple[float, float, float]]:
    """Pose (position, rpy) of a camera at `eye` looking at `target` (a camera looks along its +X)."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    d = target - eye
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))      # positive pitch tips +X downwards
    return eye, (0.0, pitch, yaw)


def overview_pose(bounds, elevation_deg: float = 35.0, margin: float = 0.15):
    """An oblique view from the south-west that fits the whole area."""
    xmin, ymin, xmax, ymax = bounds
    cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
    span = max(xmax - xmin, ymax - ymin) * (1 + margin)
    dist = span / (2 * math.tan(1.1 / 2)) * 1.05
    el = math.radians(elevation_deg)
    d = dist * np.array([-math.cos(el) * math.sqrt(0.5), -math.cos(el) * math.sqrt(0.5), math.sin(el)])
    return look_at(np.array([cx, cy, 0.0]) + d, (cx, cy, 0.0))


def follow_pose(pos, vel, back_m: float = 35.0, up_m: float = 15.0):
    """Chase camera behind a vehicle along its horizontal velocity (or from the south when hovering)."""
    pos = np.asarray(pos, float)
    v = np.asarray(vel, float)[:2]
    h = v / np.linalg.norm(v) if np.linalg.norm(v) > 0.5 else np.array([0.0, 1.0])
    eye = pos - back_m * np.array([h[0], h[1], 0.0]) + np.array([0.0, 0.0, up_m])
    return look_at(eye, pos)
