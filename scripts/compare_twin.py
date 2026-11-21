"""Table + figure over twin runs (runs_twin/*/report.json, audit.json and fidelity.json when present).

    python scripts/compare_twin.py runs_twin --world mini_town
"""
import argparse
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_sim.twin.core import score_los_predictions  # noqa: E402
from swarm_sim.twin.log import read_log  # noqa: E402
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("runs", type=Path)
ap.add_argument("--world", default="mini_town")
ap.add_argument("--out", type=Path)
args = ap.parse_args()

LABEL = {"tier_a-nominal": "matched plant", "tier_a": "full twin", "tier_a-nocal": "replan only",
         "tier_a-noreplan": "calibrate only", "tier_a-open": "open loop",
         "tier_a-nominal-admitall-none": "prediction benchmark", "px4 (session1)": "PX4, session 1",
         "px4 (session2)": "PX4, session 2 (prior)", "replay (shadow)": "replay (shadow mode)"}
rows = []
for d in sorted(args.runs.glob(f"{args.world}_*")):
    f = d / "report.json"
    if not f.exists():
        continue
    r = json.loads(f.read_text())
    a = json.loads((d / "audit.json").read_text()) if (d / "audit.json").exists() else None
    fi = json.loads((d / "fidelity.json").read_text()) if (d / "fidelity.json").exists() else None
    mm = re.match(r"(.+)_s(\d+)_x([0-9.]+)(?:_(.+))?$", d.name[len(args.world) + 1:])
    if not mm:
        continue
    cfg = mm.group(1) + (f" ({mm.group(4)})" if mm.group(4) else "")
    pt = r.get("plant_truth", {})
    p60 = r["prediction"]["position_error_m"].get("60s", {})
    los = r["prediction"].get("los") or {}
    if (d / "events.json").exists() and (d / "twin_log.mcap").exists() and "tracks" in np.load(d / "truth.npz"):
        preds = [{"t": t, "los": m["los"]} for _, t, m in read_log(d / "twin_log.mcap", ["/twin/prediction"])]
        los = score_los_predictions(preds, json.loads((d / "events.json").read_text())["los"])   # rescored from the log
    dec = r["decisions"]
    rows.append({
        "cfg": cfg, "label": LABEL.get(cfg, cfg), "seed": r["seed"], "scale": r["rate_scale"],
        "requests": r["requests"], "rejected": dec.get("rejected", 0),
        "conf": r["conformance"]["rate"], "conf_truth": a["truth_conformance"]["rate"] if a else None,
        "alerts": r["alerts"]["nonconforming"], "overruns": r["alerts"].get("landing_overrun", 0),
        "replans": r["replans"].get("replanned", 0), "contingent": r["replans"].get("contingent", 0),
        "cruise": r["calibration"]["final"]["cruise_xy"],
        "los_er": pt.get("los_events", {}).get("en_route"), "los_term": pt.get("los_events", {}).get("terminal"),
        "p60_med": p60.get("median"), "p60_p95": p60.get("p95"),
        "recall10": los.get("recall_lead_10s"), "recall": los.get("recall_any_lead"), "precision": los.get("precision"),
        "lead": los.get("lead_s_median"), "actual_los": los.get("actual_en_route"),
        "audit": (f"{a['claims']['violations']} viol, {a['los']['counterexample_samples']} cex" if a else "-"),
        "premise": fi.get("theorem3_premise_en_route_at_intent_slack") if fi else None,
        "rtf": r.get("realtime_factor"),
    })


def f(x, nd=3):
    return "-" if x is None else (f"{x:.{nd}f}" if isinstance(x, float) else str(x))


hdr = ("| scale | seed | configuration | requests (rejected) | conformance twin / truth | alerts (landing) | "
       "replans (contingent) | cruise model m/s | en-route LoS (terminal) | pred. error 60 s median / p95 m | "
       "LoS recall (lead >= 10 s) / precision, median lead s | audit |")
print(hdr)
print("|" + "---|" * hdr.count("|")[:-1] if False else "|" + "|".join(["---"] * (hdr.count("|") - 1)) + "|")
for x in sorted(rows, key=lambda r: (r["scale"], list(LABEL).index(r["cfg"]) if r["cfg"] in LABEL else 99, r["seed"])):
    recall = (f"{f(x['recall'], 2)} ({f(x['recall10'], 2)}) / {f(x['precision'], 2)}, {f(x['lead'], 0)}"
              if x["actual_los"] else "-")
    print(f"| x{x['scale']:g} | {x['seed']} | {x['label']} | {x['requests']} ({x['rejected']}) | "
          f"{f(x['conf'], 4)} / {f(x['conf_truth'], 4)} | {x['alerts']} ({x['overruns']}) | {x['replans']} ({x['contingent']}) | "
          f"{f(x['cruise'], 2)} | {f(x['los_er'])} ({f(x['los_term'])}) | {f(x['p60_med'], 2)} / {f(x['p60_p95'], 1)} | "
          f"{recall} | {x['audit']} |")

# figure: conformance by configuration, and prediction error
g = defaultdict(list)
for x in rows:
    g[(x["label"], x["scale"])].append(x)
keys = [k for k in sorted(g, key=lambda k: (k[1], list(LABEL.values()).index(k[0]) if k[0] in LABEL.values() else 99))
        if k[0] in ("matched plant", "open loop", "calibrate only", "replan only", "full twin")]
fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
cols = {"matched plant": "0.55", "open loop": "#c0392b", "calibrate only": "#e59866", "replan only": "#7fb3d5",
        "full twin": "#2471a3", "PX4 (Tier B)": "#7d3c98"}
xs = np.arange(len(keys))
vals = [np.mean([r["conf_truth"] if r["conf_truth"] is not None else r["conf"] for r in g[k]]) for k in keys]
ax[0].bar(xs, vals, color=[cols.get(k[0], "0.3") for k in keys])
for i, v in enumerate(vals):
    ax[0].text(i, v + 0.002, f"{v:.3f}", ha="center", fontsize=8)
ax[0].set_xticks(xs, [f"{k[0]}\nx{k[1]:g}" for k in keys], fontsize=8)
ax[0].set_ylim(min(vals + [0.9]) - 0.02, 1.01)
ax[0].set_ylabel("conformance (truth, mean over seeds)")
ax[0].set_title("Conformance (ground truth). All but the matched plant: fleet 15 % slower\nthan planned, one vehicle degraded to 4 m/s at t=600 s", fontsize=10)
pk = [k for k in keys if any(r["p60_med"] is not None for r in g[k])]
for i, k in enumerate(pk):
    med = [r["p60_med"] for r in g[k] if r["p60_med"] is not None]
    p95 = [r["p60_p95"] for r in g[k] if r["p60_p95"] is not None]
    ax[1].bar(i - 0.2, np.mean(med), 0.4, color=cols.get(k[0], "0.3"))
    ax[1].bar(i + 0.2, np.mean(p95), 0.4, color=cols.get(k[0], "0.3"), alpha=0.45)
ax[1].set_xticks(range(len(pk)), [f"{k[0]}\nx{k[1]:g}" for k in pk], fontsize=8)
ax[1].set_ylabel("60 s position prediction error (m)")
ax[1].set_title("Prediction: median (solid) and p95 (light)")
fig.tight_layout()
out = args.out or args.runs / f"twin_comparison_{args.world}.png"
fig.savefig(out, dpi=110)
print(f"-> {out}")
