"""Show a twin run in Gazebo: replay a finished run, follow a live twin, or record a video.

    # replay in the GUI (starts gz sim with the run's world; 4x speed)
    python scripts/gz_view.py runs_twin/<run> --launch --gui --rate 4
    # headless MP4 (paused world, stepped once per frame; works with CPU rendering)
    python scripts/gz_view.py runs_twin/<run> --launch --record runs_twin/<run>/gazebo.mp4 --camera overview
    python scripts/gz_view.py runs_twin/<run> --launch --record follow3.mp4 --camera follow:3 --start 120 --end 300
    # live: run the twin with --gz-publish in another shell (same GZ_PARTITION)
    python scripts/gz_view.py --live --world data/worlds/mini_town --launch --gui
    # no Gazebo at all: write what would be sent (models, poses, markers) as JSON Lines
    python scripts/gz_view.py runs_twin/<run> --dump /tmp/frames.jsonl

Vehicles are the twin's estimates (green conforming, red not), drawn --scale x life size. With the GUI, markers add
the intent boxes in force (faint = uncertified), plant truth (grey), 60 s predictions (yellow) and labels.
Needs the gz-transport Python bindings (Docker: service gz) except with --dump.
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_sim.gz.view import GzView, RecordingClient, ViewOptions  # noqa: E402
from swarm_sim.osm import Airspace, world_sdf  # noqa: E402
from swarm_sim.viz.scene import Scene  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("run", type=Path, nargs="?", help="twin run directory (runs_twin/<run>)")
ap.add_argument("--live", action="store_true", help="follow a running twin (run_twin.py --gz-publish)")
ap.add_argument("--world", type=Path, help="world directory (data/worlds/<name>); default: the run's")
ap.add_argument("--world-dir", type=Path, default=ROOT / "data/worlds")
ap.add_argument("--launch", action="store_true", help="start gz sim with the world (else attach to a running one)")
ap.add_argument("--gui", action="store_true", help="with --launch: also start the Gazebo GUI")
ap.add_argument("--rate", type=float, default=4.0, help="replay speed, x sim time")
ap.add_argument("--every", type=float, help="sim seconds between frames (default: rate / fps)")
ap.add_argument("--fps", type=float, default=10.0, help="frames per wall second (and video fps)")
ap.add_argument("--start", type=float)
ap.add_argument("--end", type=float)
ap.add_argument("--record", type=Path, help="write an MP4 from a camera sensor (headless)")
ap.add_argument("--camera", default="overview", help="overview | follow:<vid> | follow (first non-conforming, else first)")
ap.add_argument("--size", default="1280x720")
ap.add_argument("--scale", type=float, default=4.0, help="vehicle marker size, x life size")
ap.add_argument("--no-intents", action="store_true")
ap.add_argument("--no-truth", action="store_true")
ap.add_argument("--no-predictions", action="store_true")
ap.add_argument("--no-labels", action="store_true")
ap.add_argument("--certified-only", action="store_true", help="draw certified intent boxes only")
ap.add_argument("--dump", type=Path, help="no Gazebo: write the calls as JSON Lines")
ap.add_argument("--quiet", action="store_true", help="do not print events")
args = ap.parse_args()
if not args.live and args.run is None:
    ap.error("give a run directory or --live")
if args.live and args.world is None:
    ap.error("--live needs --world")
if args.record and args.live:
    ap.error("--record replays a finished run (record live runs afterwards from their directory)")

# ------------------------------------------------------------------ scene and world
if args.live:
    air = Airspace.load(args.world / "airspace.json")
    scene = Scene(air)
    world_path = args.world / "world.sdf"
else:
    scene = Scene.from_run(args.run, args.world.parent if args.world else args.world_dir)
    air = scene.air
    world_path = (args.world or args.world_dir / air.name) / "world.sdf"
every = args.every or args.rate / args.fps
opt = ViewOptions(scale=args.scale, intents=not args.no_intents, truth=not args.no_truth,
                  predictions=not args.no_predictions, labels=not args.no_labels, certified_only=args.certified_only)

procs = []
gz = None
if args.dump:
    client = RecordingClient()
else:
    from swarm_sim.gz import launch
    from swarm_sim.gz.transport import Gz
    if args.launch:
        if not world_path.exists():
            world_path.write_text(world_sdf(air))
        procs = launch.start(world_path, air.name, run=not args.record, gui=args.gui,
                             logdir=(args.run or Path("/tmp")) / "gz_logs" if not args.live else Path("/tmp/gz_view"))
        print(f"gz sim: world {air.name} up ({world_path})")
    gz = client = Gz(air.name)
view = GzView(client, opt)

recorder = None
if args.record:
    from swarm_sim.gz.models import overview_pose
    from swarm_sim.gz.record import Recorder
    w, h = (int(x) for x in args.size.split("x"))
    pos, rpy = overview_pose(air.bounds)
    recorder = Recorder(gz, args.record, w, h, args.fps, pose=(pos, rpy))

follow_vid = None
if args.camera.startswith("follow:"):
    follow_vid = int(args.camera.split(":", 1)[1])


def camera_for(frame):
    from swarm_sim.gz.models import follow_pose, overview_pose
    if args.camera == "overview" or not frame.vehicles:
        return overview_pose(air.bounds)
    vid = follow_vid
    if vid is None:
        nc = [k for k, v in frame.vehicles.items() if not v.conforming]
        vid = min(nc) if nc else min(frame.vehicles)
    v = frame.vehicles.get(vid)
    return follow_pose(v.pos, v.vel) if v is not None else overview_pose(air.bounds)


def show_events(frame):
    if args.quiet:
        return
    for e in frame.events:
        print(f"  t={e.t:7.1f}  {'!' if e.warn else ' '} {e.text}")


dump = open(args.dump, "w") if args.dump else None
n = 0
try:
    if args.live:
        from swarm_sim.gz.live import GzLiveFeed
        feed = GzLiveFeed(gz)
        print("waiting for the twin on /swarm/twin (run_twin.py --gz-publish) ...")
        latest = None
        while True:
            for topic, t, msg in feed.drain():
                scene.feed(topic, t, msg)
                if topic == "/twin/state":
                    latest = t if latest is None else max(latest, t)
            if latest is not None:
                f = scene.frame(latest)
                view.apply(f)
                show_events(f)
                n += 1
            time.sleep(1.0 / args.fps)
    else:
        times = scene.times(every, args.start, args.end)
        print(f"replaying {args.run.name}: {len(times)} frames, t = {times[0]:.0f}..{times[-1]:.0f} s"
              if times else "nothing to replay")
        wall0 = time.monotonic()
        for k, t in enumerate(times):
            f = scene.frame(t)
            view.apply(f)
            show_events(f)
            if recorder is not None:
                recorder.camera(*camera_for(f))
                if not recorder.capture():
                    print(f"WARNING: no camera image at t={t:.1f}; stopping")
                    break
            elif dump is None:
                lag = wall0 + k / args.fps - time.monotonic()
                if lag > 0:
                    time.sleep(lag)
            if dump is not None:
                for c in client.calls:
                    dump.write(json.dumps({"t": round(t, 3), "call": c[0], "args": c[1:]}, default=float) + "\n")
                client.calls.clear()
            n += 1
except KeyboardInterrupt:
    pass
finally:
    if recorder is not None:
        recorder.close()
        print(f"-> {recorder.out}  ({recorder.frames} frames at {args.fps:g} fps, {args.rate:g}x)")
    if dump is not None:
        dump.close()
        print(f"-> {args.dump}  ({n} frames)")
    if procs:
        from swarm_sim.gz import launch
        launch.stop(procs)
