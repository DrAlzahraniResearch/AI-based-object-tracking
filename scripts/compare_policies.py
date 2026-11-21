"""Compare separation policies on identical seeds / demand.

    python scripts/compare_policies.py runs/ --world mini_town [--out runs/compare_mini_town]

Reads runs/<world>_<policy>_s<seed>_x<scale>/metrics.json (+ audit.json if present) and writes
comparison.json, comparison.md and comparison.png.
"""
import argparse
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("runs", type=Path)
ap.add_argument("--world", required=True)
ap.add_argument("--out", type=Path)
args = ap.parse_args()
out = args.out or args.runs / f"compare_{args.world}"
out.mkdir(parents=True, exist_ok=True)

pat = re.compile(rf"^{re.escape(args.world)}_(?P<policy>[a-z0-9-]+)_s(?P<seed>\d+)_x(?P<scale>[0-9.]+)$")
rows = []
for d in sorted(args.runs.iterdir()):
    m = pat.match(d.name)
    if not m or not (d / "metrics.json").exists():
        continue
    met = json.loads((d / "metrics.json").read_text())
    aud = json.loads((d / "audit.json").read_text()) if (d / "audit.json").exists() else None
    stats = met["params"].get("policy_info", {}).get("stats", {})
    sch = met.get("schedule")
    rows.append({
        "policy": m["policy"], "seed": int(m["seed"]), "scale": float(m["scale"]),
        "missions": met["missions"], "completed": met["completed"], "flight_h": met["flight_hours"],
        "los_en_route": met["los_events"]["en_route"], "los_terminal": met["los_events"]["terminal"],
        "los_en_route_per_fh": met["los_events"]["en_route_per_flight_hour"],
        "min_dist_m": met["los_events"]["min_dist_m"],
        "delay_p95_s": met["delay_s_vs_plan"]["p95"], "delay_max_s": met["delay_s_vs_plan"]["max"],
        "intrusions": met["intrusions"]["building"] + met["intrusions"]["no_fly"],
        "conformance": met["conformance"]["rate"],
        "infeasible_share": round(stats["infeasible"] / stats["agent_steps"], 4) if stats.get("agent_steps") else None,
        "requests": sch["requests"] if sch else met["missions"],
        "rejected": sch["rejected"] if sch else 0,
        "uncertified": sch["accepted_uncertified"] if sch else 0,
        "ground_delay_p95_s": sch["ground_delay_s"]["p95"] if sch else 0.0,
        "ground_delay_mean_s": sch["ground_delay_s"]["mean"] if sch else 0.0,
        "audit": None if aud is None else {"certified_rate": aud["strategic"]["certified_rate"],
                                           "theorem": aud.get("theorem", "1"),
                                           "counterexamples": aud["los_events"]["between_certified_pairs"],
                                           "mismatches": aud["strategic"]["reference_mismatches"]
                                           + aud["conformance"]["reference_mismatches"]},
    })
if not rows:
    sys.exit(f"no runs for world {args.world!r} in {args.runs}")
rows.sort(key=lambda r: (r["scale"], r["seed"], r["policy"]))
(out / "comparison.json").write_text(json.dumps(rows, indent=1))

hdr = ("| scale | seed | policy | done / requested | rejected | uncert. | ground delay mean / p95 s | en-route LoS | "
       "per flight h | terminal LoS | air delay p95 s | intrusions | conformance | certified | audit |")
lines = [hdr, "|" + "---|" * (hdr.count("|") - 1)]
for r in rows:
    a = r["audit"]
    audit = "-" if a is None else f"{a['counterexamples']} cex, {a['mismatches']} mism"
    cert = "-" if a is None else f"{100 * a['certified_rate']:.1f} %"
    lines.append(f"| ×{r['scale']:g} | {r['seed']} | {r['policy']} | {r['completed']}/{r['requests']} | {r['rejected']} | "
                 f"{r['uncertified']} | {r['ground_delay_mean_s'] or 0:.0f} / {r['ground_delay_p95_s'] or 0:.0f} | "
                 f"{r['los_en_route']} | {r['los_en_route_per_fh']:.2f} | {r['los_terminal']} | "
                 f"{r['delay_p95_s']:.1f} | {r['intrusions']} | {r['conformance']:.4f} | {cert} | {audit} |")
(out / "comparison.md").write_text("\n".join(lines) + "\n")
print("\n".join(lines))

agg = defaultdict(lambda: defaultdict(list))
for r in rows:
    agg[r["policy"]][r["scale"]].append(r["los_en_route_per_fh"])
gd = defaultdict(lambda: defaultdict(list))
for r in rows:
    gd[r["policy"]][r["scale"]].append(r["ground_delay_mean_s"] or 0.0)
fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11, 4), dpi=150)
for pol, by in sorted(gd.items()):
    xs = sorted(by)
    ax2.plot(xs, [sum(by[x]) / len(by[x]) for x in xs], marker="o", label=pol,
             ls="--" if pol.startswith("none") else "-", lw=2.5 if pol.startswith("none") else 1.5)
ax2.set_xlabel("demand scale (× baseline rates)")
ax2.set_ylabel("mean ground delay per request (s)")
ax2.set_title("cost of strategic deconfliction (lines overlap per scheduler)")
ax2.grid(alpha=0.3)
ax2.legend()
for pol, by in sorted(agg.items()):
    xs = sorted(by)
    line, = ax.plot(xs, [sum(by[x]) / len(by[x]) for x in xs], marker="o", label=f"{pol} (mean of seeds)")
    for x in xs:
        ax.scatter([x] * len(by[x]), by[x], s=12, alpha=0.5, color=line.get_color())
ax.set_xlabel("demand scale (× baseline rates)")
ax.set_ylabel("en-route LoS events per flight hour")
ax.set_title(f"{args.world}: separation policies (dots = seeds)")
ax.grid(alpha=0.3)
ax.legend()
fig.tight_layout()
fig.savefig(out / "comparison.png")
print(f"-> {out}")
