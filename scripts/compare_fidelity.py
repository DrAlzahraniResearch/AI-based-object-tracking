"""Compare fidelity reports of several twin runs (e.g. PX4 session 1 vs session 2 with --prior).

    python scripts/compare_fidelity.py runs_twin/mini_town_px4_s1_x1_session1 runs_twin/mini_town_px4_s1_x1_session2 \
        --labels "session 1 (uncalibrated)" "session 2 (prior)" --out docs/figures/milestone4_5_px4.png
"""
import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("runs", type=Path, nargs="+")
ap.add_argument("--labels", nargs="+")
ap.add_argument("--out", type=Path, required=True)
args = ap.parse_args()
labels = args.labels or [r.name for r in args.runs]
cols = ["#c0392b", "#2471a3", "#27ae60", "#7d3c98"]

fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))
print("| run | conformance twin / truth | alerts (landing) | T3 premise at intent slack | p99 error at 5 s: h / v m | "
      "lift-off lag s | cruise leg - plan s | 60 s prediction median / p95 m |")
print("|---|---|---|---|---|---|---|---|")
for k, (run, lab) in enumerate(zip(args.runs, labels)):
    f = json.loads((run / "fidelity.json").read_text())
    r = json.loads((run / "report.json").read_text())
    a = json.loads((run / "audit.json").read_text()) if (run / "audit.json").exists() else None
    tt = f["theorem3_tracking"]
    ds = [d for d in tt if d.startswith("delta_") and d.endswith("s") and d != "delta_intent_slack"]
    xs = [float(d[6:-1]) for d in ds]
    ax[0].plot(xs, [tt[d]["en_route"]["within_buffers"] for d in ds], "o-", color=cols[k], label=lab)
    ax[0].axhline(f["theorem3_premise_en_route_at_intent_slack"], color=cols[k], ls=":", lw=1)
    ax[1].plot(xs, [tt[d]["en_route"]["eh_m"]["p99"] for d in ds], "o-", color=cols[k], label=f"{lab}: horizontal")
    ax[1].plot(xs, [tt[d]["en_route"]["ev_m"]["p99"] for d in ds], "s--", color=cols[k], label=f"{lab}: vertical")
    hs = sorted(r["prediction"]["position_error_m"], key=lambda h: float(h[:-1]))
    off = (k - (len(args.runs) - 1) / 2) * 0.35
    ax[2].bar(np.arange(len(hs)) + off, [r["prediction"]["position_error_m"][h]["p95"] for h in hs], 0.35,
              color=cols[k], alpha=0.5, label=f"{lab}: p95")
    ax[2].bar(np.arange(len(hs)) + off, [r["prediction"]["position_error_m"][h]["median"] for h in hs], 0.35,
              color=cols[k], label=f"{lab}: median")
    ax[2].set_xticks(range(len(hs)), hs)
    p60 = r["prediction"]["position_error_m"]["60s"]
    e5 = tt["delta_5s"]["en_route"]
    print(f"| {lab} | {r['conformance']['rate']:.4f} / {a['truth_conformance']['rate']:.4f} | "
          f"{r['alerts']['nonconforming']} ({r['alerts'].get('landing_overrun', 0)}) | "
          f"{f['theorem3_premise_en_route_at_intent_slack']:.3f} | {e5['eh_m']['p99']} / {e5['ev_m']['p99']} | "
          f"{f['liftoff_lag_s']['mean']} | {f['leg_timing']['cruise']['measured_minus_planned_s']['mean']} | "
          f"{p60['median']} / {p60['p95']} |")
ax[0].set_xlabel("timing tolerance delta (s)")
ax[0].set_ylabel("en-route samples within the buffers (20 m, 2.5 m)")
ax[0].set_title("Theorem 3 premise (dotted: at the intents' own slack)")
ax[0].set_ylim(0, 1.05)
ax[0].legend(fontsize=8, frameon=False)
ax[1].set_yscale("log")
ax[1].axhline(20, color="0.5", lw=0.8)
ax[1].axhline(2.5, color="0.5", lw=0.8, ls="--")
ax[1].set_xlabel("timing tolerance delta (s)")
ax[1].set_ylabel("p99 tracking error (m, log)")
ax[1].set_title("Tracking error vs plan (grey: lateral / vertical buffers)")
ax[1].legend(fontsize=7, frameon=False)
ax[2].set_xlabel("prediction horizon")
ax[2].set_ylabel("position error (m)")
ax[2].set_title("Twin's Tier A prediction of PX4")
ax[2].legend(fontsize=7, frameon=False)
fig.suptitle("PX4 SITL (SIH) under the twin: same 3 deliveries, before and after calibration", fontsize=11)
fig.tight_layout()
args.out.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(args.out, dpi=110)
print(f"-> {args.out}")
