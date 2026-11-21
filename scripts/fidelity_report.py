"""Fidelity report for a twin run: how far does the plant (PX4, or the Tier A stand-in) stray from the
plan, and from what Tier A predicts?

    python scripts/fidelity_report.py runs_twin/mini_town_px4_s1_x1

1. Theorem 3 tracking error. For every truth sample (t, p) and the plan version in force, the nominal
   position p0(t0) comes from the plan's kinematic schedule (swarm_sim.intents.planned_schedule). For a
   timing tolerance delta, t0 ranges over [t - delta, t + delta] and is chosen to minimise the error
   normalised by the intent buffers (lateral, vertical). Reported per delta: the fraction of samples within
   the buffers (Theorem 3's premise with (eh, ev, delta) = intent buffers), and percentiles of the
   horizontal (per-axis, Chebyshev) and vertical error at that t0. Split en-route / terminal.
2. Tier A vs plant. Each flight's admitted plan is re-flown in Tier A alone (no traffic, nominal model) and
   compared with the plant's track up to its first replan.
3. Leg timing: measured vs planned durations by leg type, and a fitted cruise model t = d / v + c.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from swarm_sim.intents import planned_schedule  # noqa: E402
from swarm_sim.missions import Mission, Waypoint  # noqa: E402
from swarm_sim.osm import Airspace  # noqa: E402
from swarm_sim.sim import SimParams, Simulation  # noqa: E402
from swarm_sim.vehicle import VehicleParams  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("run", type=Path)
ap.add_argument("--world-dir", type=Path, default=ROOT / "data/worlds")
ap.add_argument("--deltas", default="0,2,5,10")
args = ap.parse_args()

rep = json.loads((args.run / "report.json").read_text())
ip = rep["params"]["intent"]
EH, EV, TERM = ip["lateral_buffer_m"], ip["vertical_buffer_m"], ip["terminal_radius_m"]
deltas = [float(x) for x in args.deltas.split(",")]
vers = json.loads((args.run / "intents.json").read_text())["versions"]
air = Airspace.load(args.world_dir / rep["world"] / "airspace.json")


def mission_of(d) -> Mission:
    m = Mission(**{k: v for k, v in d.items() if k != "waypoints"})
    m.waypoints = [Waypoint(**w) for w in d["waypoints"]]
    return m


for x in vers:
    x["m"] = mission_of(x["mission"])
    x["vp"] = VehicleParams(**x["vehicle"])
    legs = planned_schedule(x["m"], x["vp"], x.get("takeoff_s", 0.0))
    x["T"] = np.array([legs[0][2]] + [b[3] for b in legs])
    x["P"] = np.array([legs[0][0]] + [b[1] for b in legs], float)
    x["pad"] = np.array(x["mission"]["meta"].get("pad", x["mission"]["meta"]["start"])[:2], float)
by_vid = {}
for x in vers:
    by_vid.setdefault(x["vid"], []).append(x)


def nominal(x, t0):
    """Planned position at times t0 (array): piecewise linear through the leg endpoints, clamped."""
    return np.stack([np.interp(t0, x["T"], x["P"][:, k]) for k in range(3)], axis=-1)


# ------------------------------------------------------------------ truth samples
tr = np.load(args.run / "truth.npz")
S = []                                   # (t, vid, x, y, z)
if "tracks" in tr:
    times_, tracks_ = tr["times"], tr["tracks"]            # load once: NpzFile decompresses on every access
    for k, t in enumerate(times_):
        row = tracks_[k].copy()
        for i in np.flatnonzero(~np.isnan(row[:, 0])):
            S.append((float(t), int(i), *row[i]))
else:
    S = [tuple(r) for r in tr["px4_track"]]
S = np.array(S, float).reshape(-1, 5)


def version_at(vid, t):
    hit = None
    for x in by_vid.get(vid, []):
        if x["valid_from"] <= t <= x["valid_to"]:
            hit = x
    return hit


# ------------------------------------------------------------------ 1. Theorem 3 tracking error
res = {}
for d in deltas + ["slack"]:
    rows = {"en_route": [], "terminal": []}
    for t, vid, *p in S:
        if p[2] < 0.5:
            continue
        x = version_at(int(vid), t)
        if x is None:
            continue
        dd = (ip["slack_s"] + ip["slack_per_min"] * max(0.0, t - x["mission"]["release_t"]) / 60.0) if d == "slack" else d
        offs = np.arange(-dd, dd + 1e-9, 0.25) if dd > 0 else np.zeros(1)
        p0 = nominal(x, t + offs)
        dx, dy, dz = (np.abs(p0[:, k] - p[k]) for k in range(3))
        score = np.maximum(np.maximum(dx, dy) / EH, dz / EV)
        j = int(np.argmin(score))
        ph = "terminal" if math.hypot(p[0] - x["pad"][0], p[1] - x["pad"][1]) <= TERM else "en_route"
        rows[ph].append((max(dx[j], dy[j]), dz[j], score[j] <= 1.0))
    out = {}
    for ph, r in rows.items():
        if not r:
            continue
        a = np.array(r, float)
        out[ph] = {"samples": len(a), "within_buffers": round(float(a[:, 2].mean()), 4),
                   "eh_m": {"p50": round(float(np.percentile(a[:, 0], 50)), 2), "p99": round(float(np.percentile(a[:, 0], 99)), 2),
                            "max": round(float(a[:, 0].max()), 2)},
                   "ev_m": {"p50": round(float(np.percentile(a[:, 1], 50)), 2), "p99": round(float(np.percentile(a[:, 1], 99)), 2),
                            "max": round(float(a[:, 1].max()), 2)}}
    res["delta_intent_slack" if d == "slack" else f"delta_{d:g}s"] = out

# ------------------------------------------------------------------ 2. Tier A re-flight of each admitted plan
sp = SimParams(dt=0.1, log_hz=10, duration_s=0, max_extra_s=3600)
cmp_rows, tracks_a = [], {}
for vid, vs in by_vid.items():
    x = vs[0]
    t_cut = vs[1]["valid_from"] if len(vs) > 1 else math.inf
    m = mission_of(x["mission"])
    m.id = 0
    sim = Simulation(air, [m], x["vp"], sp, None, monitors=False)
    sim.fleet.t = 0.0
    while sim.step() and sim.fleet.t < m.release_t + 3600:
        pass
    ta = np.array(sim.times)
    pa = np.array(sim.tracks)[:, 0, :]
    ok = ~np.isnan(pa[:, 0])
    tracks_a[vid] = (ta[ok], pa[ok])
    B = S[(S[:, 1] == vid) & (S[:, 0] <= t_cut) & (S[:, 0] >= m.release_t)]
    if len(B) == 0 or ok.sum() == 0:
        continue
    pa_i = np.stack([np.interp(B[:, 0], ta[ok], pa[ok, k], left=np.nan, right=np.nan) for k in range(3)], -1)
    good = ~np.isnan(pa_i[:, 0])
    eh = np.hypot(*(B[good, 2:4] - pa_i[good, :2]).T)
    ev = np.abs(B[good, 4] - pa_i[good, 2])
    land_a = float(ta[ok][-1])
    land_b = float(B[:, 0].max()) if t_cut == math.inf else None
    cmp_rows.append({"vid": vid, "compared_s": round(float(B[good, 0].max() - B[good, 0].min()), 1) if good.any() else 0,
                     "until": "landing" if t_cut == math.inf else f"first replan at {t_cut:g} s",
                     "horizontal_m": {"p50": round(float(np.median(eh)), 2), "max": round(float(eh.max()), 2)},
                     "vertical_m": {"p50": round(float(np.median(ev)), 2), "max": round(float(ev.max()), 2)},
                     "mission_time_s": {"tier_a": round(land_a - m.release_t, 1),
                                        "plant": round(land_b - m.release_t, 1) if land_b else None}})

# ------------------------------------------------------------------ 3. leg timing
legs = json.loads((args.run / "legs.json").read_text()) if (args.run / "legs.json").exists() else []
timing = {}
for tag in sorted({x["tag"] for x in legs}):
    L = [x for x in legs if x["tag"] == tag]
    e = np.array([x["measured_s"] - x["planned_s"] for x in L])
    timing[tag] = {"n": len(L), "measured_minus_planned_s": {"mean": round(float(e.mean()), 2),
                                                             "max_abs": round(float(abs(e).max()), 2)}}
cr = [x for x in legs if x["tag"] == "cruise" and x["dxy"] > 50]
if len(cr) >= 2 and len({x["dxy"] for x in cr}) >= 2:
    A = np.c_[[x["dxy"] for x in cr], np.ones(len(cr))]
    (inv_v, c), *_ = np.linalg.lstsq(A, np.array([x["measured_s"] for x in cr]), rcond=None)
    timing["cruise_fit"] = {"v_eff_ms": round(1 / inv_v, 2), "overhead_s": round(float(c), 2), "n": len(cr)}
elif cr:
    v_model = vers[0]["vehicle"]["cruise_xy"]
    timing["cruise_fit"] = {"note": "one leg length only: overhead at the model's speed",
                            "overhead_at_model_speed_s": round(float(np.mean([x["measured_s"] - x["dxy"] / v_model for x in cr])), 2)}

lags = []                                # time to pass 5 m above the pad minus the model's climb time to 5 m
for vid, vs in by_vid.items():
    pz = vs[0]["mission"]["meta"].get("pad", vs[0]["mission"]["meta"]["start"])[2]
    B = S[(S[:, 1] == vid) & (S[:, 4] > pz + 5.0)]
    if len(B):
        lags.append(float(B[:, 0].min()) - vs[0]["mission"]["release_t"] - 5.0 / vs[0]["vehicle"]["v_up_max"])
worst = res[f"delta_{max(deltas):g}s"].get("en_route", {})
premise = res["delta_intent_slack"].get("en_route", {}).get("within_buffers")
report = {
    "run": str(args.run), "plant": rep["plant"],
    "budget": {"lateral_buffer_m": EH, "vertical_buffer_m": EV, "slack_s": f"{ip['slack_s']} + {ip['slack_per_min']}/min"},
    "theorem3_tracking": res,
    "theorem3_premise_en_route_at_intent_slack": premise,
    "liftoff_lag_s": {"mean": round(float(np.mean(lags)), 2), "max": round(float(np.max(lags)), 2)} if lags else None,
    "tier_a_vs_plant": cmp_rows,
    "leg_timing": timing,
    "recommendation": {
        "lateral_buffer_m_min": round(worst.get("eh_m", {}).get("max", 0) + 2, 1) if worst else None,
        "vertical_buffer_m_min": round(worst.get("ev_m", {}).get("max", 0) + 0.5, 1) if worst else None,
        "note": "smallest buffers covering every en-route sample at the largest delta, plus margin"},
}
(args.run / "fidelity.json").write_text(json.dumps(report, indent=1))

# ------------------------------------------------------------------ figure
fig, ax = plt.subplots(1, 3, figsize=(16, 4.8))
fig.suptitle(f"Fidelity: {rep['plant']} vs plan vs Tier A ({args.run.name})", fontsize=11)
for vid, vs in list(by_vid.items())[:6]:
    B = S[S[:, 1] == vid]
    x = vs[0]
    ax[0].plot(x["P"][:, 0], x["P"][:, 1], color="0.6", lw=3, alpha=0.5)
    if vid in tracks_a:
        ax[0].plot(tracks_a[vid][1][:, 0], tracks_a[vid][1][:, 1], "--", lw=1, color="#2471a3")
    ax[0].plot(B[:, 2], B[:, 3], lw=1, color="#c0392b")
ax[0].plot([], [], color="0.6", lw=3, alpha=0.5, label="plan")
ax[0].plot([], [], "--", color="#2471a3", label="Tier A")
ax[0].plot([], [], color="#c0392b", label=rep["plant"])
ax[0].set_aspect("equal")
ax[0].legend(fontsize=8, frameon=False)
ax[0].set_title("Tracks (x, y)")
vid0 = next(iter(by_vid))
B = S[S[:, 1] == vid0]
sb = np.hypot(np.gradient(B[:, 2], B[:, 0]), np.gradient(B[:, 3], B[:, 0]))
w = max(1, int(round(1.0 / max(np.median(np.diff(B[:, 0])), 1e-3))))          # 1 s moving average
sb = np.convolve(sb, np.ones(w) / w, mode="same")
ax[1].plot(B[:, 0], sb, color="#c0392b", lw=1, label=rep["plant"])
if vid0 in tracks_a:
    ta, pa = tracks_a[vid0]
    ax[1].plot(ta, np.hypot(np.gradient(pa[:, 0], ta), np.gradient(pa[:, 1], ta)), "--", color="#2471a3", lw=1, label="Tier A")
ax[1].axhline(by_vid[vid0][0]["vehicle"]["cruise_xy"], color="0.5", ls=":", lw=1, label="commanded cruise")
ax[1].set_xlabel("time (s)")
ax[1].set_ylabel("horizontal speed (m/s)")
ax[1].set_title(f"Speed profile, flight {vid0}")
ax[1].legend(fontsize=8, frameon=False)
xs = [f"{d:g}" for d in deltas] + ["intent\nslack"]
for ph, col in (("en_route", "#2471a3"), ("terminal", "#c0392b")):
    ys = [res[f"delta_{d:g}s"].get(ph, {}).get("within_buffers", np.nan) for d in deltas] + \
         [res["delta_intent_slack"].get(ph, {}).get("within_buffers", np.nan)]
    ax[2].plot(xs, ys, "o-", color=col, label=ph)
ax[2].set_ylim(0, 1.05)
ax[2].set_xlabel("timing tolerance delta (s)")
ax[2].set_ylabel(f"fraction within buffers ({EH:g} m, {EV:g} m)")
ax[2].set_title("Theorem 3 premise: tracking error within the buffers")
ax[2].legend(fontsize=8, frameon=False)
fig.tight_layout()
fig.savefig(args.run / "fidelity.png", dpi=110)
print(json.dumps({k: report[k] for k in ("theorem3_premise_en_route_at_intent_slack", "liftoff_lag_s", "tier_a_vs_plant", "leg_timing",
                                         "recommendation")}, indent=1))
print(json.dumps(res, indent=1))
