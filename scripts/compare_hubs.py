"""Milestones 5 / 5.5 table + figure: hub sequencing off / compact hubs / spread hubs, tactical only or with landing
reservations (runs_twin/*/report.json, audit.json).

    python scripts/compare_hubs.py runs_twin --world mini_town [--out docs/figures/milestone5_hubs.png]
"""
import argparse
import json
import re
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("runs", type=Path)
ap.add_argument("--world", default="mini_town")
ap.add_argument("--out", type=Path)
args = ap.parse_args()

LABEL = {"tier_a-nominal-fix": "off", "tier_a-nominal-hubseq": "sequenced, compact hubs",
         "tier_a-nominal-hubseq-pads40": "sequenced, spread hubs (40 m)", "tier_a-hubseq": "sequenced, mismatched plant",
         "tier_a-nominal-hubres": "reserved, compact hubs", "tier_a-nominal-hubres-pads40": "reserved, spread hubs (40 m)",
         "tier_a-hubres": "reserved, mismatched plant",
         "tier_a-nominal-hubres-fb": "reserved or tactical, compact", "tier_a-nominal-hubres-fb-pads40": "reserved or tactical, spread"}
ORDER = list(LABEL)
rows = []
for d in sorted(args.runs.glob(f"{args.world}_*")):
    mm = re.match(r"(.+)_s(\d+)_x([0-9.]+)$", d.name[len(args.world) + 1:])
    if not mm or mm.group(1) not in LABEL or not (d / "report.json").exists():
        continue
    r = json.loads((d / "report.json").read_text())
    a = json.loads((d / "audit.json").read_text()) if (d / "audit.json").exists() else None
    ev = json.loads((d / "events.json").read_text())["los"]
    h = r.get("hubs") or {}
    sched = r["decisions"]
    req = r["requests"]
    # ground delay of flown requests: last decision per request (moved departures included)
    log = d / "twin_log.mcap"
    gd = []
    if log.exists():
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from swarm_sim.twin.log import read_log
        last = {}
        for _, _, x in read_log(log, ["/twin/decision"]):
            last[x["mission"]] = x
        gd = [x["ground_delay_s"] for x in last.values() if x["ground_delay_s"] is not None]
    rows.append({
        "cfg": mm.group(1), "label": LABEL[mm.group(1)], "seed": int(mm.group(2)), "scale": float(mm.group(3)),
        "requests": req, "rejected": sched.get("rejected", 0), "uncert": sched.get("accepted_uncertified", 0),
        "moved": h.get("departures_moved", 0), "gd_mean": float(np.mean(gd)) if gd else 0.0,
        "gd_p95": float(np.percentile(gd, 95)) if gd else 0.0,
        "arr": h.get("arrivals"), "seq": h.get("sequenced"), "cont": h.get("contingent"), "kept": h.get("reserved", 0),
        "prio": len(h.get("emergency_priority") or []),
        "hold_mean": (h.get("hold_s") or {}).get("mean"), "hold_p95": (h.get("hold_s") or {}).get("p95"),
        "hold_max": (h.get("hold_s") or {}).get("max"),
        "los_term": sum(e["terminal"] for e in ev), "los_er": sum(not e["terminal"] for e in ev),
        "completed": r["plant_truth"]["completed"], "conf": r["conformance"]["rate"],
        "conf_truth": a["truth_conformance"]["rate"] if a else None,
        "audit": (f"{a['claims']['violations']} viol, {a['los']['counterexample_samples']} cex" if a else "-"),
        "los_cls": a["los"]["events_by_class"] if a else None, "wall": r["wall_s"],
    })


def f(x, nd=0):
    return "-" if x is None else (f"{x:.{nd}f}" if isinstance(x, float) else str(x))


print("| scale | seed | hubs | flown / requests | rejected | departures moved | ground delay mean / p95 s | "
      "arrivals sequenced + kept reservation (contingent) | holding mean / p95 / max s | terminal LoS | en-route LoS | "
      "emergencies uncertified (priority) | conformance (truth) | audit |")
print("|" + "|".join(["---"] * 14) + "|")
for x in sorted(rows, key=lambda r: (r["scale"], ORDER.index(r["cfg"]), r["seed"])):
    arr = f"{x['seq']} + {x['kept']} ({x['cont']})" if x["arr"] is not None else "-"
    hold = f"{f(x['hold_mean'])} / {f(x['hold_p95'])} / {f(x['hold_max'])}" if x["arr"] else "-"
    print(f"| x{x['scale']:g} | {x['seed']} | {x['label']} | {x['completed']} / {x['requests']} | {x['rejected']} | "
          f"{x['moved']} | {x['gd_mean']:.0f} / {x['gd_p95']:.0f} | {arr} | {hold} | {x['los_term']} | {x['los_er']} | "
          f"{x['uncert']} ({x['prio']}) | "
          f"{f(x['conf_truth'] if x['conf_truth'] is not None else x['conf'], 4)} | {x['audit']} |")

g = defaultdict(list)
for x in rows:
    g[(x["scale"], x["cfg"])].append(x)
print("\n| scale | hubs | runs | flown / requests | rejected | terminal LoS | en-route LoS | arrivals certified | "
      "contingent | emergencies uncertified | holding mean / p95 s | ground delay mean s | departures moved |")
print("|" + "|".join(["---"] * 13) + "|")
for k in sorted(g, key=lambda k: (k[0], ORDER.index(k[1]))):
    xs = g[k]
    arr = sum(x["arr"] or 0 for x in xs)
    print(f"| x{k[0]:g} | {LABEL[k[1]]} | {len(xs)} | {sum(x['completed'] for x in xs)} / {sum(x['requests'] for x in xs)} | "
          f"{sum(x['rejected'] for x in xs)} | {sum(x['los_term'] for x in xs)} | {sum(x['los_er'] for x in xs)} | "
          + (f"{sum(x['seq'] + x['kept'] for x in xs)} / {arr} | {sum(x['cont'] for x in xs)} | " if arr else "- | - | ")
          + f"{sum(x['uncert'] for x in xs)} | "
          + (f"{np.mean([x['hold_mean'] for x in xs if x['hold_mean'] is not None]):.0f} / "
             f"{np.mean([x['hold_p95'] for x in xs if x['hold_p95'] is not None]):.0f} | " if arr else "- | ")
          + f"{np.mean([x['gd_mean'] for x in xs]):.0f} | {sum(x['moved'] for x in xs)} |")

if rows:
    scales = sorted({x["scale"] for x in rows})
    cfgs = [c for c in ORDER if any(x["cfg"] == c for x in rows)]
    cols = {"tier_a-nominal-fix": "#c0392b", "tier_a-nominal-hubseq": "#2471a3",
            "tier_a-nominal-hubseq-pads40": "#27ae60", "tier_a-hubseq": "#7d3c98",
            "tier_a-nominal-hubres": "#85c1e9", "tier_a-nominal-hubres-pads40": "#82e0aa", "tier_a-hubres": "#c39bd3",
            "tier_a-nominal-hubres-fb": "#1b4f72", "tier_a-nominal-hubres-fb-pads40": "#145a32"}
    fig, ax = plt.subplots(1, 5, figsize=(24, 4.6))
    w = 0.8 / len(cfgs)
    for i, c in enumerate(cfgs):
        xs = np.arange(len(scales)) + (i - (len(cfgs) - 1) / 2) * w
        ax[0].bar(xs, [np.mean([x["los_term"] for x in g[(s, c)]]) if g[(s, c)] else 0 for s in scales], w,
                  color=cols[c], label=LABEL[c])
        ax[1].bar(xs, [100 * sum(x["completed"] for x in g[(s, c)]) / sum(x["requests"] for x in g[(s, c)])
                       if g[(s, c)] else 0 for s in scales], w, color=cols[c], label=LABEL[c])
        ax[2].bar(xs, [np.mean([x["gd_mean"] for x in g[(s, c)]]) if g[(s, c)] else 0 for s in scales], w,
                  color=cols[c], label=LABEL[c])
        if c != "tier_a-nominal-fix":
            ax[4].bar(xs, [np.mean([x["cont"] or 0 for x in g[(s, c)]]) if g[(s, c)] else 0 for s in scales], w,
                      color=cols[c], label=LABEL[c])
            ax[3].bar(xs, [np.mean([x["hold_p95"] or 0 for x in g[(s, c)]]) if g[(s, c)] else 0 for s in scales], w,
                      color=cols[c], alpha=0.5)
            ax[3].bar(xs, [np.mean([x["hold_mean"] or 0 for x in g[(s, c)]]) if g[(s, c)] else 0 for s in scales], w,
                      color=cols[c], label=LABEL[c])
    ax[1].set_ylim(0, 105)
    for a_, t in zip(ax, ("Terminal-area loss of separation (per run)", "Requests flown (%)", "Mean ground delay (s)",
                          "Arrival holding: mean (solid), p95 (light), s", "Contingent arrivals (per run)")):
        a_.set_xticks(range(len(scales)), [f"x{s:g}" for s in scales])
        a_.set_title(t, fontsize=10)
        a_.set_xlabel("demand")
    ax[0].legend(fontsize=8, frameon=False)
    fig.suptitle("Milestones 5 / 5.5: hub sequencing, tactical / with landing reservations (mini_town, ORCA + FCFS, seeds 1-3; matched plant unless noted)", fontsize=11)
    fig.tight_layout()
    out = args.out or args.runs / f"hubs_comparison_{args.world}.png"
    fig.savefig(out, dpi=110)
    print(f"-> {out}")
