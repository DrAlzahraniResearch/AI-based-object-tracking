# Visualization and Gazebo (milestone 6)

One scene model, three viewers, and PX4 flying in the OSM world.

```
twin run (twin_log.mcap + intents.json)  ──► swarm_sim/viz/scene.py: Scene.frame(t)
live twin (run_twin.py --gz-publish) ──gz topic /swarm/twin──┘        │
                                                                     ├─► Gazebo   scripts/gz_view.py      3D world, vehicle models, intent markers, MP4
                                                                     ├─► Foxglove scripts/foxglove_export.py  3D + event log + plots, one MCAP
                                                                     └─► Rerun    scripts/twin_view.py    scrubbable 3D timeline
PX4 SITL x500s in Gazebo Harmonic (swarm_sim/twin/px4_gz.py) ──► the same twin, plant "px4gz"
```

A frame at time t holds the twin's vehicle estimates (interpolated between the 1 Hz `/twin/state` samples) with
conformance, the plant's truth (sim plants), the latest 60 s prediction, every intent box in force at t (from
the plan versions valid at t, certified or not) and the events since the previous frame. All three viewers draw
the same frames, so they agree with each other and with the audit's inputs.

## Which viewer for what

| Question | Viewer |
|---|---|
| What does the traffic look like in the town? Demo, paper video | Gazebo (`gz_view.py --record`), or the Gazebo GUI |
| Did PX4 really fly it, buildings and all? | Gazebo, Tier B in Gazebo (`--plant px4gz`) |
| Why was this flight replanned? Which boxes were in force? | Rerun or Foxglove 3D, with the event log |
| How do non-conformance, flying count, certified boxes evolve? | Foxglove Plot on `/metrics` |
| Nothing installed but Python | Foxglove (web app, open the MCAP) |

## Gazebo viewer (`scripts/gz_view.py`, `swarm_sim/gz/`)

```bash
docker compose -f docker/compose.yaml build gz
# headless MP4: the world is paused and stepped once per video frame, so CPU rendering is fine (just slow)
docker compose -f docker/compose.yaml run --rm gz python3 scripts/gz_view.py runs_twin/<run> --launch \
  --record runs_twin/<run>/gazebo.mp4 --rate 8 --camera overview
docker compose -f docker/compose.yaml run --rm gz python3 scripts/gz_view.py runs_twin/<run> --launch \
  --record runs_twin/<run>/follow.mp4 --camera follow:3 --start 120 --end 400
# GUI (Linux: xhost +local: first; Windows: see the gz-gui service in docker/compose.yaml)
docker compose -f docker/compose.yaml --profile gui run --rm gz-gui python3 scripts/gz_view.py runs_twin/<run> --launch --gui
# no Gazebo: write the models, poses and markers that would be sent, as JSON Lines
python scripts/gz_view.py runs_twin/<run> --dump /tmp/frames.jsonl
```

* **Vehicles** are models, so the camera sensor sees them: a green and a red quadrotor marker per vehicle, the
  one not in use parked 200 m below ground. They are drawn 4x life size (`--scale`) so they are visible over a
  town, yawed along the velocity and tilted with speed. A vehicle absent for 20 s is removed.
* **Markers** (GUI only, `/marker_array`): intent boxes in force in the vehicle's colour (faint when uncertified),
  plant truth (grey), 60 s predictions (yellow, with a line from the vehicle), labels (`3 !` = non-conforming).
  Camera sensors render server entities only, so recorded video shows vehicles and the world, not the boxes.
  Use Rerun or Foxglove for the boxes.
* **Live:** `run_twin.py ... --gz-publish` publishes every logged message, and each plan version as it is made
  or withdrawn, as JSON on the gz topic `/swarm/twin`; `gz_view.py --live --world data/worlds/<name>` draws it.
  Both must share `GZ_PARTITION` (the images set `swarm`) and should run in one container, since gz-transport
  discovery does not cross container networks by default. The link is one-way: the twin still never reads the
  simulator (decision S22). `scripts/live_demo.sh` runs both in one container:

  ```bash
  # Tier A twin held to 4x real time (--pace), headless: status lines, a summary, exit code 0 if the link works
  docker compose -f docker/compose.yaml run --rm px4gz bash scripts/live_demo.sh
  # the same in the Gazebo GUI
  docker compose -f docker/compose.yaml --profile gui run --rm gz-gui bash scripts/live_demo.sh --gui
  # PX4 x500s in Gazebo: the twin owns the world, the viewer adds only markers (--no-vehicles)
  docker compose -f docker/compose.yaml run --rm px4gz bash scripts/live_demo.sh --plant px4gz --missions 3
  ```

  With Tier A the twin is stopped after `--duration` wall seconds (default 120; `0` for the whole run) and still
  writes its report. Live frames are drawn at each new `/twin/state` (1 Hz of sim time; `twin.log_state_hz` in
  `configs/twin.yaml` for smoother motion). Without a GUI, markers fail; the viewer then skips them for 30 s at a
  time instead of waiting for each request's timeout.
* Needs the gz-transport Python bindings (`python3-gz-transport13 python3-gz-msgs10` on Harmonic; Ionic and
  Jetty module names are tried too), except with `--dump`.

## Tier B in Gazebo (`--plant px4gz`, `swarm_sim/twin/px4_gz.py`)

```bash
docker compose -f docker/compose.yaml build px4gz                   # builds gz and proofs first, then PX4 + gz bridge
docker compose -f docker/compose.yaml run --rm px4gz python3 scripts/run_twin.py --world data/worlds/mini_town \
  --plant px4gz --missions 3 --verified --tag gz1
docker compose -f docker/compose.yaml run --rm px4gz python3 scripts/fidelity_report.py runs_twin/mini_town_px4gz_s1_x1_gz1
docker compose -f docker/compose.yaml run --rm gz python3 scripts/gz_view.py runs_twin/mini_town_px4gz_s1_x1_gz1 --launch --record ...
```

* **Image.** PX4 v1.16.2 requires Gazebo Harmonic (gz-sim >= 8; `px4-rc.gzsim` checks) and supports it on
  Ubuntu 24.04, so `gz` is 24.04 + Harmonic and `px4gz` builds PX4 with its Gazebo bridge on top of it. The
  older `sim` image (26.04 + Jetty) is kept for the MOT-compatible ROS stack.
* **World.** `world_sdf(air, px4=True)`: the OSM world with PX4's system list (Contact, AirPressure, AirSpeed,
  ApplyLinkWrench, Magnetometer, NavSat, IMU, Sensors, ...), ODE at 250 Hz, the magnetic field and atmosphere.
  Its `<spherical_coordinates>` put PX4's GPS at the OSM origin. Buildings have collision geometry.
* **Vehicles come and go.** Each x500 is spawned 45 s before its filed release, at its filed pad, once nobody
  stands on or hovers low over that pad (one PX4 instance per vehicle, `PX4_GZ_STANDALONE=1`,
  `PX4_GZ_MODEL_POSE`), and removed once landed. A delivery returns to its hub, so without removal pads would
  fill up. The plant clock is the world's (`/world/<w>/clock`), not instance 0's as with SIH.
* **Frames.** A vehicle's local NED origin is its spawn point; ENU = spawn + (east, north, -down), as in SIH.
  If the scheduler later moves the flight to another pad of the same hub, the vehicle takes off from where it
  stands, the plant logs it, and the twin sees non-conformance at take-off.
* **Cost.** An x500 with Gazebo sensors needs about one core at real time; keep `missions x speed_factor`
  within the core count. Settings: `plant_px4_gz` in `configs/twin.yaml`.

## Foxglove (`scripts/foxglove_export.py`, `swarm_sim/viz/foxglove.py`)

```bash
python scripts/foxglove_export.py runs_twin/<run>      # -> runs_twin/<run>/foxglove.mcap (eval image is enough)
```

Open the file in Foxglove (web or desktop). Channels use Foxglove's own JSON schemas (vendored in
`swarm_sim/viz/schemas/`), so panels work without configuration:

| Channel | Schema | Panel |
|---|---|---|
| `/world` | foxglove.SceneUpdate | 3D: buildings, hub pads and names, no-fly outlines (frame `world`) |
| `/scene` | foxglove.SceneUpdate | 3D: vehicles (green / red, labelled), truth, predictions, intent boxes in force |
| `/events` | foxglove.Log | Log: alerts, replans, decisions, arrivals, calibration (WARNING when they matter) |
| `/metrics` | JSON | Plot: `flying`, `nonconforming`, `intents_certified`, `intents_uncertified` |

The twin's own log (`twin_log.mcap`) also opens in Foxglove, for the raw messages.

## Tests and what is verified

`tests/test_viz.py` (no Gazebo needed): PX4 world systems and physics; marker and camera SDF; camera aiming
(the camera's +X axis points at the target); scene interpolation, conformance, intents in force and events on
a synthetic run; the Gazebo viewer's calls (spawn, park, remove, marker id reuse and deletion) through a
recording client and through `gz_view.py --dump`; the live link reproducing the offline scene message for
message; Foxglove export validated against Foxglove's schemas; clock parsing, image row padding, pad
occupancy and PX4 instance environment.

Opt-in, against real software: `RUN_GZ=1` (service `gz`) starts a world and checks the vehicle models appear;
`RUN_PX4GZ=1` (service `px4gz`) flies one PX4 delivery in Gazebo and checks it lands.

```bash
docker compose -f docker/compose.yaml run --rm -e RUN_GZ=1 gz python3 -m pytest -q tests/test_viz.py
docker compose -f docker/compose.yaml run --rm -e RUN_PX4GZ=1 px4gz python3 -m pytest -q tests/test_viz.py -k px4
```

Verified so far (Docker Desktop on Windows, CPU only): all three images build; `RUN_GZ=1` passes (Harmonic
world started, vehicle models created and moved through the Python bindings); `RUN_PX4GZ=1` passes (one x500
delivery spawned, flown under the twin and landed, about 5 minutes); `--record --camera follow:0` of that run
gives a correct 59 s, 1280x720 video (589 frames: take-off from the pad, climb over the town, cruise, return,
landing; conforming throughout). CPU rendering takes a few seconds per frame, so a recording takes several times
the flight's length; it prints progress every 25 frames. Three x500s (`--missions 3`, two from the west depot
at once, one from the east) flew their deliveries at real time with conformance 1.0, no alerts or replans, and
the twin calibrated a 4.07 s lift-off lag (3.9 s with SIH); `audit_twin.py` with the verified checker is
CONSISTENT (3 certified concurrent pairs confirmed, 3,330 PX4 position samples all inside their boxes, no loss of
separation). `scripts/live_demo.sh` (Tier A at 4x, headless): the viewer received all 935 messages of 474 s of
traffic (up to 4 drones, 18 intent boxes in force) and drew 377 frames. Not yet tried: the GUI from a container.

## Limits

* Recorded video has no intent boxes (camera sensors do not render GUI markers).
* The Gazebo viewer shows the twin's estimates, not ground truth, as models; truth is a GUI marker.
* Live viewing across containers needs gz-transport discovery configured (`GZ_IP`, host networking); one
  container is the simple case.
* `px4gz` spawns at the filed pad; a pad change made at admission after the spawn shows up as non-conformance.
