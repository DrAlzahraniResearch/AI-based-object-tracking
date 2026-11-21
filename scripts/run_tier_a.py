"""Run one Tier A simulation.

    python scripts/run_tier_a.py --world data/worlds/mini_town --config configs/tier_a.yaml [--seed 3] [--rate-scale 2]

Writes runs/<world>_<policy>[-<scheduler>]_s<seed>_x<scale>/: missions.json, intents.json (float +
checker ints + phase), tracks.npz, events.json, metrics.json, traffic.png, and schedule.json with a scheduler.
"""
import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_sim.intents import IntentParams, mission_intents  # noqa: E402
from swarm_sim.metrics import summarise  # noqa: E402
from swarm_sim.missions import RoutingParams, generate_missions  # noqa: E402
from swarm_sim.orca import OrcaParams, OrcaPolicy  # noqa: E402
from swarm_sim.scheduler import FcfsScheduler, SchedulerParams  # noqa: E402
from swarm_sim.osm import Airspace  # noqa: E402
from swarm_sim.plot import plot_traffic  # noqa: E402
from swarm_sim.sim import SeparationPolicy, SimParams, run  # noqa: E402
from swarm_sim.vehicle import VehicleParams  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--world", type=Path, required=True, help="data/worlds/<name> (from osm_to_world.py)")
ap.add_argument("--config", type=Path, default=ROOT / "configs/tier_a.yaml")
ap.add_argument("--seed", type=int)
ap.add_argument("--rate-scale", type=float, default=1.0, help="multiply all demand rates (stress tests)")
ap.add_argument("--policy", choices=["none", "orca"], help="overrides the config's policy")
ap.add_argument("--scheduler", choices=["none", "fcfs"], help="overrides the config's scheduler")
ap.add_argument("--verified-search", action="store_true",
                help="scheduler uses the Lean-verified C checker for its search (needs runtime/build)")
ap.add_argument("--out", type=Path, default=ROOT / "runs")
args = ap.parse_args()

cfg = yaml.safe_load(args.config.read_text())
seed = args.seed if args.seed is not None else cfg["seed"]
air = Airspace.load(args.world / "airspace.json")
vp = VehicleParams(**cfg.get("vehicle") or {})
rp = RoutingParams(**cfg.get("routing") or {})
ip = IntentParams(**cfg.get("intent") or {})
sp = SimParams(duration_s=cfg["duration_s"], **cfg.get("sim") or {})
pname = args.policy or cfg.get("policy", "none")
if pname == "orca":
    policy = OrcaPolicy(OrcaParams(**({"h_sep_m": sp.h_sep_m, "v_sep_m": sp.v_sep_m, "terminal_radius_m": sp.terminal_radius_m} | (cfg.get("orca") or {}))), sp.dt)
else:
    policy = SeparationPolicy()
rates = {k: v * args.rate_scale for k, v in cfg["demand_per_hour"].items()}
missions = generate_missions(air, rates, cfg["duration_s"], seed, rp)
sname = args.scheduler or cfg.get("scheduler", "none")
sched_report = None
if sname == "fcfs":
    chk = None
    if args.verified_search:
        from runtime.utm_checker import VerifiedChecker
        chk = VerifiedChecker()
    spar = SchedulerParams(**({"h_sep_m": sp.h_sep_m, "v_sep_m": sp.v_sep_m} | (cfg.get("scheduler_params") or {})))
    missions, sched_report = FcfsScheduler(air, vp, ip, rp, spar, chk).schedule(missions)
res = run(air, missions, vp, sp, policy)
met = summarise(res, vp, ip, sp.h_sep_m, sp.v_sep_m)
if sched_report is not None:
    met["schedule"] = {k: v for k, v in sched_report.items() if k != "decisions"}
met |= {"world": air.name, "seed": seed, "rate_scale": args.rate_scale, "rates_per_h": rates,
        "assumptions": {"h_sep_m": sp.h_sep_m, "v_sep_m": sp.v_sep_m, "terminal_radius_m": sp.terminal_radius_m}}

tag = res.params["policy"] + ("" if sname == "none" else f"-{sname}")
out = args.out / f"{air.name}_{tag}_s{seed}_x{args.rate_scale:g}"
out.mkdir(parents=True, exist_ok=True)
if sched_report is not None:
    (out / "schedule.json").write_text(json.dumps(sched_report, indent=1))
(out / "missions.json").write_text(json.dumps([m.to_json() for m in missions]))
intents = {m.id: mission_intents(m, vp, ip) for m in missions}
(out / "intents.json").write_text(json.dumps({
    "units": {"float": "m, s", "int": "mm, ms"},
    "missions": {str(k): {"float": [asdict(b) for b in v], "int": [b.as_int() for b in v],
                          "phase": [b.phase for b in v]} for k, v in intents.items()}}))
np.savez_compressed(out / "tracks.npz", times=res.times, tracks=res.tracks)
(out / "events.json").write_text(json.dumps({"los": res.los_events, "intrusions": res.intrusions, "status": res.status}))
(out / "metrics.json").write_text(json.dumps(met | {"params": res.params}, indent=1))
tracks = {i: res.tracks[~np.isnan(res.tracks[:, i, 0]), i] for i in range(len(missions))}
plot_traffic(air, {i: t for i, t in tracks.items() if len(t)}, out / "traffic.png",
             [e for e in res.los_events if not e["terminal"]],
             f"{air.name}: {met['completed']}/{met['missions']} missions, "
             f"{met['los_events']['en_route']} en-route LoS, {tag}")
keys = ("missions", "completed", "flight_hours", "delay_s_vs_plan", "ground_delay_s", "los_events", "intrusions",
        "strategic_conflicts", "conformance", "wall_s")
print(json.dumps({k: met[k] for k in keys} | ({"schedule": {k: met["schedule"][k] for k in
      ("requests", "accepted", "accepted_uncertified", "rejected", "layer_bumped", "wall_s")}} if sched_report else {}),
      indent=1))
print(f"-> {out}")
