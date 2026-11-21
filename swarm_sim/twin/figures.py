"""Summary figure for a twin run: what the twin saw, how it reacted, how well it predicted."""
from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def twin_figure(out, rep: dict, twin) -> None:
    fig, ax = plt.subplots(2, 2, figsize=(13, 8.5))
    fig.suptitle(f"Digital twin: {rep['world']} / plant {rep['plant']} / seed {rep['seed']} x{rep['rate_scale']:g}  "
                 f"(conformance {rep['conformance']['rate']}, {rep['replans']['total']} replans)", fontsize=11)

    # 1. flights in the air vs time, with alerts and replans
    a = ax[0, 0]
    ts = sorted({round(d, 1) for d in np.arange(0, rep["sim_s"], 5.0)})
    ver = twin.intent_versions
    airborne = [len({v["vid"] for v in ver if v["valid_from"] <= t <= v["valid_to"]}) for t in ts]
    a.plot(ts, airborne, color="0.35", lw=1, label="flights with an active plan")
    nc = [x["t"] for x in twin.alerts if x["event"] == "nonconforming"]
    a.scatter(nc, np.full(len(nc), max(airborne or [1]) * 1.05), marker="v", color="#c0392b", s=18,
              label=f"non-conformance alerts ({len(nc)})")
    rp = [r["t"] for r in twin.replans]
    a.scatter(rp, np.full(len(rp), max(airborne or [1]) * 1.12), marker="o", color="#2471a3", s=14,
              label=f"replans ({len(rp)})")
    a.set_xlabel("plant time (s)")
    a.set_ylabel("flights")
    a.set_title("Flights, alerts and replans")
    a.legend(loc="lower right", fontsize=8, frameon=False)

    # 2. calibration
    a = ax[0, 1]
    m0 = rep["calibration"]["model"]["cruise_xy"]
    a.axhline(m0, color="0.5", ls="--", lw=1, label=f"planning model {m0:g} m/s")
    up = [u for u in rep["calibration"]["updates"] if "cruise_xy" in u]
    if up:
        a.step([0] + [u["t"] for u in up] + [rep["sim_s"]], [m0] + [u["cruise_xy"] for u in up] + [up[-1]["cruise_xy"]],
               where="post", color="#2471a3", lw=2, label="twin's calibrated cruise speed")
    tv = rep.get("plant_truth", {}).get("vehicle", {}).get("v_xy_max")
    if tv:
        a.axhline(tv, color="#c0392b", lw=1, ls=":", label=f"plant's true limit {tv:g} m/s")
    a.set_ylim(0, max(m0, tv or 0) * 1.25)
    a.set_xlabel("plant time (s)")
    a.set_ylabel("m/s")
    a.set_title("Online calibration (horizontal cruise)")
    a.legend(loc="lower right", fontsize=8, frameon=False)

    # 3. prediction error vs horizon
    a = ax[1, 0]
    hs = sorted(twin.pos_err)
    if hs:
        a.boxplot([twin.pos_err[h] for h in hs], showfliers=False)
        a.set_xticks(range(1, len(hs) + 1), [f"{h:g} s" for h in hs])
    a.set_ylabel("position error (m)")
    a.set_xlabel("prediction horizon")
    los = rep["prediction"].get("los")
    extra = (f"\nLoS: recall {los['recall_any_lead']} (lead >= 10 s: {los['recall_lead_10s']}), precision {los['precision']}"
             if los and los["actual_en_route"] else "")
    a.set_title("Prediction (Tier A rollout from twin state)" + extra, fontsize=10)

    # 4. leg-time error before / after calibration
    a = ax[1, 1]
    legs = [x for x in twin.legs if x["tag"] == "cruise"]
    if legs:
        e0 = [x["measured_s"] - x["planned_s"] for x in legs]
        e1 = [x["measured_s"] - x["planned_cal_s"] for x in legs]
        bins = np.linspace(min(e0 + e1) - 1, max(e0 + e1) + 1, 30)
        a.hist(e0, bins, color="0.6", alpha=0.8, label="vs planning model")
        a.hist(e1, bins, color="#2471a3", alpha=0.6, label="vs model at completion (calibrated)")
        a.axvline(0, color="k", lw=0.8)
        a.legend(fontsize=8, frameon=False)
    a.set_xlabel("measured - planned cruise leg time (s)")
    a.set_ylabel("legs")
    a.set_title("Cruise leg timing")
    fig.tight_layout()
    fig.savefig(out / "twin.png", dpi=110)
    plt.close(fig)
