"""Run the digital twin against a plant.

    python scripts/run_twin.py --world data/worlds/mini_town                     # Tier A plant (mismatch + fault)
    python scripts/run_twin.py --world data/worlds/mini_town --plant px4         # PX4 SITL (SIH) via MAVSDK
    python scripts/run_twin.py --world data/worlds/mini_town --plant px4gz       # PX4 SITL x500s in Gazebo (px4gz image)
    python scripts/run_twin.py --world data/worlds/mini_town --plant replay --replay runs_twin/<run>/twin_log.mcap

Writes runs_twin/<world>_<plant>_s<seed>_x<scale>/: twin_log.mcap, report.json, intents.json (every plan
version, for scripts/audit_twin.py), routes.json, truth.npz + events.json (sim plants), twin.png.
"""
import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_sim.intents import IntentParams  # noqa: E402
from swarm_sim.missions import RoutingParams, generate_missions  # noqa: E402
from swarm_sim.orca import OrcaParams, OrcaPolicy  # noqa: E402
from swarm_sim.osm import Airspace  # noqa: E402
from swarm_sim.scheduler import FcfsScheduler, SchedulerParams  # noqa: E402
from swarm_sim.sim import SeparationPolicy, SimParams  # noqa: E402
from swarm_sim.twin.core import DigitalTwin, TwinParams, score_los_predictions  # noqa: E402
from swarm_sim.twin.log import TwinLog, read_log  # noqa: E402
from swarm_sim.twin.telemetry import PlantParams, ReplayPlant, Telemetry, TierAPlant  # noqa: E402
from swarm_sim.vehicle import VehicleParams  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--world", type=Path, required=True)
ap.add_argument("--config", type=Path, default=ROOT / "configs/tier_a.yaml")
ap.add_argument("--twin-config", type=Path, default=ROOT / "configs/twin.yaml")
ap.add_argument("--plant", choices=["tier_a", "px4", "px4gz", "replay"], default="tier_a")
ap.add_argument("--replay", type=Path, help="twin_log.mcap to replay (--plant replay)")
ap.add_argument("--seed", type=int)
ap.add_argument("--rate-scale", type=float, default=1.0)
ap.add_argument("--policy", choices=["none", "orca"], default="orca")
ap.add_argument("--no-replan", action="store_true")
ap.add_argument("--no-calibrate", action="store_true")
ap.add_argument("--no-faults", action="store_true", help="Tier A plant: no model mismatch, no fault (control run)")
ap.add_argument("--admit-all", action="store_true",
                help="no strategic deconfliction: every request flies as filed (benchmark for conflict prediction)")
ap.add_argument("--missions", type=int, help="px4, px4gz: number of vehicles/missions (overrides config)")
ap.add_argument("--speed", type=float, help="px4, px4gz: simulation speed factor (overrides config)")
ap.add_argument("--gz-gui", action="store_true", help="px4gz: also open the Gazebo GUI")
ap.add_argument("--gz-publish", action="store_true",
                help="publish the twin live on the gz topic /swarm/twin (scripts/gz_view.py --live)")
ap.add_argument("--pace", type=float,
                help="tier_a, replay: hold the plant to this many sim seconds per wall second (for --gz-publish)")
ap.add_argument("--verified", action="store_true", help="Lean-verified checker for conformance and scheduling")
ap.add_argument("--hubs", choices=["off", "sequence", "reserve"],
                help="hub sequencing; reserve: plus landing reservations at admission (overrides configs/twin.yaml hubs.mode)")
ap.add_argument("--reserve-fallback", action="store_true",
                help="hubs reserve: admit without a landing reservation instead of rejecting")
ap.add_argument("--approach-fix", action="store_true", help="route inbound flights via approach fixes even with --hubs off")
ap.add_argument("--pad-spacing", type=float, help="override every hub's pad spacing (m)")
ap.add_argument("--prior", type=Path, help="report.json of an earlier twin run: start from its calibrated model")
ap.add_argument("--tag", default="")
ap.add_argument("--out", type=Path, default=ROOT / "runs_twin")
args = ap.parse_args()

cfg = yaml.safe_load(args.config.read_text())
tcfg = yaml.safe_load(args.twin_config.read_text())
seed = args.seed if args.seed is not None else cfg["seed"]
air = Airspace.load(args.world / "airspace.json")
hcfg = tcfg.get("hubs") or {}
hub_mode = args.hubs or hcfg.get("mode", "off")
pad_spacing = args.pad_spacing or hcfg.get("pad_spacing_m")
if pad_spacing:
    air.hubs = [replace(h, pad_spacing_m=float(pad_spacing)) for h in air.hubs]
vp = VehicleParams(**cfg.get("vehicle") or {})
rp = RoutingParams(**cfg.get("routing") or {})
ip = IntentParams(**cfg.get("intent") or {})
hub_seq = hub_mode in ("sequence", "reserve")
if hub_seq or args.approach_fix:
    rp = replace(rp, approach_fix_m=float(hcfg.get("approach_fix_m", 70)))
    if hcfg.get("terminal_lateral_buffer_m") is not None:
        ip = replace(ip, terminal_lateral_buffer_m=float(hcfg["terminal_lateral_buffer_m"]))
sp = SimParams(duration_s=cfg["duration_s"], **cfg.get("sim") or {})
tp = TwinParams(**(tcfg.get("twin") or {}))
if args.no_replan:
    tp = replace(tp, replan=False)
if args.no_calibrate:
    tp = replace(tp, calibrate=False)
if args.admit_all:
    tp = replace(tp, strategic=False)
orca_p = OrcaParams(**({"h_sep_m": sp.h_sep_m, "v_sep_m": sp.v_sep_m, "terminal_radius_m": sp.terminal_radius_m}
                       | (cfg.get("orca") or {})))


def policy_factory(dt):
    return OrcaPolicy(orca_p, dt) if args.policy == "orca" else SeparationPolicy()


vp_cfg = vp            # the vehicles' configuration: the plant's truth (Tier A) and the commands (PX4) never take a prior
prior = None
if args.prior:
    prior = json.loads(args.prior.read_text())["calibration"]["final"]
    vp = replace(vp, **{k: v for k, v in prior.items() if k != "takeoff_s"})
    ip = replace(ip, takeoff_s=prior.get("takeoff_s", 0.0))
    print(f"prior calibration from {args.prior}: {prior}")
checker = None
if args.verified:
    from runtime.utm_checker import VerifiedChecker
    checker = VerifiedChecker()
spar = SchedulerParams(**({"h_sep_m": sp.h_sep_m, "v_sep_m": sp.v_sep_m} | (cfg.get("scheduler_params") or {})
                          | ({"terminal": hub_mode} if hub_seq else {})
                          | ({"reserve_fallback": True} if args.reserve_fallback else {})))
sched = FcfsScheduler(air, vp, ip, rp, spar, checker)
sequencer = None
if hub_seq:
    from swarm_sim.hubs import HubParams, HubSequencer
    sequencer = HubSequencer(sched, rp, HubParams(**{k: (tuple(v) if isinstance(v, list) else v)
                                                     for k, v in hcfg.items() if k in HubParams.__dataclass_fields__}))
rates = {k: v * args.rate_scale for k, v in cfg["demand_per_hour"].items()}
requests = generate_missions(air, rates, cfg["duration_s"], seed, rp)

plant_name = args.plant
if args.plant == "tier_a":
    pp = PlantParams(**{k: (tuple(v) if k == "faults" else v) for k, v in (tcfg.get("plant_tier_a") or {}).items()})
    if args.no_faults:
        pp = replace(pp, vehicle={}, faults=())
        plant_name = "tier_a-nominal"
    plant = TierAPlant(air, requests, vp_cfg, sp, policy_factory(sp.dt), pp)
    max_t = sp.duration_s + sp.max_extra_s
elif args.plant in ("px4", "px4gz"):
    pc = tcfg.get("plant_px4" if args.plant == "px4" else "plant_px4_gz") or {}
    k = args.missions or pc.get("missions", 1)
    picked = [m for m in requests if m.kind == "delivery"][:k]
    for i, m in enumerate(picked):
        m.id = i
        m.release_t = pc.get("first_release_s", 5.0) + i * pc.get("stagger_s", 20)
    requests = picked
    speed = args.speed or pc.get("speed_factor", 1)
    if args.plant == "px4":
        from swarm_sim.twin.px4 import Px4Plant
        plant = Px4Plant(len(requests), vp_cfg, sp, speed_factor=speed,
                         rate_hz=pc.get("rate_hz", 10), setpoint_hz=pc.get("setpoint_hz", 20))
    else:
        from swarm_sim.twin.px4_gz import Px4GzPlant
        plant = Px4GzPlant(requests, air, vp_cfg, sp, speed_factor=speed, rate_hz=pc.get("rate_hz", 10),
                           setpoint_hz=pc.get("setpoint_hz", 20), model=pc.get("model", "gz_x500"),
                           spawn_lead_s=pc.get("spawn_lead_s", 45), pad_clear_m=pc.get("pad_clear_m", 4.0),
                           gui=args.gz_gui)
    plant.t_end = max(m.release_t for m in requests) + 1.0
    max_t = 3600.0
else:
    frames = {}
    for _topic, _t, msg in read_log(args.replay, ["/twin/telemetry"]):
        for s in msg["samples"]:
            frames.setdefault(s["t"], []).append(Telemetry(s["vid"], s["t"], np.array(s["pos"]), np.array(s["vel"]),
                                                           s["state"], s["wp_idx"], s["hold_until"], s.get("route_ver", 1)))
    plant = ReplayPlant(sorted(frames.items()), sp.dt)
    tp = replace(tp, replan=False)                 # shadow mode: the past cannot be changed
    max_t = max(frames) + 1.0

mode = "-".join(x for x, on in (("open", args.no_replan and args.no_calibrate), ("noreplan", args.no_replan and not args.no_calibrate),
                                  ("nocal", args.no_calibrate and not args.no_replan), ("admitall", args.admit_all),
                                  (args.policy, args.policy != "orca"), ("hubseq", hub_mode == "sequence"),
                                  ("hubres", hub_mode == "reserve"), ("fb", args.reserve_fallback), ("fix", args.approach_fix and not hub_seq),
                                  (f"pads{pad_spacing:g}" if pad_spacing else "", bool(pad_spacing))) if on)
name = (f"{air.name}_{plant_name}{('-' + mode) if mode else ''}_s{seed}_x{args.rate_scale:g}"
        f"{('_' + args.tag) if args.tag else ''}")
out = args.out / name
out.mkdir(parents=True, exist_ok=True)
log = TwinLog(out / "twin_log")
print(f"twin: plant={plant_name} requests={len(requests)} checker={'verified' if checker else 'python-reference'}"
      f" -> {out}")
twin_plant = plant
if args.pace:
    if args.plant in ("px4", "px4gz"):
        ap.error("--pace is for tier_a and replay; PX4 plants run at their own speed factor (--speed)")
    from swarm_sim.twin.telemetry import PacedPlant
    twin_plant = PacedPlant(plant, args.pace)
twin = DigitalTwin(air, twin_plant, requests, sched, vp, ip, sp, policy_factory, tp, checker, log, sequencer=sequencer,
                   vp_cfg=vp_cfg)
if args.gz_publish:
    from swarm_sim.gz.live import TOPIC, GzLiveTap
    from swarm_sim.gz.transport import Gz
    _gz = Gz(air.name)
    log.taps.append(GzLiveTap(lambda data: _gz.publish_string(TOPIC, data), twin))
    print(f"publishing the twin on gz topic {TOPIC} (scripts/gz_view.py --live --world {args.world})")
try:
    rep = twin.run(max_t)
except KeyboardInterrupt:                      # Ctrl+C, or `timeout --signal=INT` (scripts/live_demo.sh)
    print(f"interrupted at t={plant.now():.1f} s: writing what the twin has so far")
    for v in twin.intent_versions:
        if v["valid_to"] is None:
            v["valid_to"] = round(plant.now(), 3)
    rep = twin.report()
    rep["interrupted_at_s"] = round(plant.now(), 1)
finally:
    log.close()
    if args.plant in ("px4", "px4gz"):
        plant.close()

rep |= {"world": air.name, "seed": seed, "rate_scale": args.rate_scale, "policy": args.policy, "log": log.file.name,
        "mode": {"replan": tp.replan, "calibrate": tp.calibrate, "strategic": tp.strategic, "plant": plant_name,
                 "prior": str(args.prior) if args.prior else None, "prior_model": prior,
                 "hubs": hub_mode, "approach_fix_m": rp.approach_fix_m, "pad_spacing_m": pad_spacing},
        "params": {"twin": tp.__dict__, "intent": twin.ip.__dict__}}
if isinstance(plant, TierAPlant):
    res = plant.sim.result()
    ev = res.los_events
    rep["plant_truth"] = {
        "vehicle": plant.vp_true.__dict__, "faults": plant.fault_log,
        "completed": int(sum(s["state"] == "landed" for s in res.status)),
        "los_events": {"en_route": sum(not e["terminal"] for e in ev), "terminal": sum(e["terminal"] for e in ev)},
        "intrusions": res.intrusions}
    rep["prediction"]["los"] = score_los_predictions(twin.predictions, ev)
    np.savez_compressed(out / "truth.npz", times=res.times, tracks=res.tracks)
    (out / "events.json").write_text(json.dumps({"los": ev, "intrusions": res.intrusions, "status": res.status}))
elif args.plant in ("px4", "px4gz"):
    tr = np.array(plant.track) if plant.track else np.zeros((0, 5))
    np.savez_compressed(out / "truth.npz", px4_track=tr)
    rep["plant_truth"] = {"completed": sum(s == "landed" for s in plant.state), "speed_factor": plant.speed,
                          "simulator": "gazebo" if args.plant == "px4gz" else "sih",
                          "note": "Tier B truth = PX4 EKF local position at telemetry rate"}
else:
    rep["shadow_mode_commands_ignored"] = len(plant.ignored)
(out / "report.json").write_text(json.dumps(rep, indent=1, default=float))
(out / "intents.json").write_text(json.dumps({"units": {"int": "mm, ms"}, "versions": twin.intent_versions}))
(out / "routes.json").write_text(json.dumps({str(k): m.to_json() for k, m in twin.routes.items()}))
(out / "legs.json").write_text(json.dumps(twin.legs))
from swarm_sim.twin.figures import twin_figure  # noqa: E402
twin_figure(out, rep, twin)
show = {k: rep[k] for k in ("plant", "sim_s", "wall_s", "realtime_factor", "decisions", "conformance", "alerts",
                            "replans", "hubs", "calibration", "prediction") if k in rep and rep[k] is not None}
show["conformance"] = {k: show["conformance"][k] for k in ("rate", "checker")}
show["calibration"] = {"updates": len(rep["calibration"]["updates"]), "final": rep["calibration"]["final"]}
if "plant_truth" in rep:
    show["plant_truth"] = {k: v for k, v in rep["plant_truth"].items() if k != "vehicle"}
print(json.dumps(show, indent=1))
print(f"-> {out}")
