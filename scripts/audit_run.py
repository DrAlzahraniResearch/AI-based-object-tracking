"""Offline audit of a Tier A run with the Lean-verified checker (compiled C, via ctypes).

    make -C runtime                                   # once
    python scripts/audit_run.py runs/mini_town_orca-fcfs_s1_x1

Uses Theorem 2 (cylinder form, proofs/UtmProofs/Spec.lean: `no_los_of_spec`; checker:
`Utm.separatedHV`; refinement: `checker_no_los_samples`) with the run's own LoS thresholds
h = h_sep_m, v = v_sep_m.

1. Strategic separation. A mission pair is CERTIFIED if the verified checker accepts every pair of
   their EN-ROUTE intent boxes (terminal boxes, the take-off / landing columns at the pad, are hub
   sequencing, plan step 7). By Theorem 2, two certified flights that are both inside en-route boxes
   at the same instant are not in loss of separation. Also reported: certification over all boxes.
2. Conformance. Every logged position sample is checked (verified `inside`) against its own boxes:
   any box, and en-route boxes only.
3. Consistency. An LoS event between a certified pair is a COUNTEREXAMPLE if both flights were inside
   en-route boxes at every logged sample of the event: that would contradict Theorem 2, so it means
   bad assumptions or a bug. Otherwise the theorem's premise failed, reported apart as
   `certified_premise_terminal` (a flight was only in terminal boxes) or
   `certified_premise_nonconforming` (a flight was outside all its boxes).
Every verified decision is cross-checked against the Python reference (swarm_sim.intents).
"""
import argparse
import itertools
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from runtime.utm_checker import VerifiedChecker  # noqa: E402
from swarm_sim.intents import inside_ref, separated_hv_matrix  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("run", type=Path)
args = ap.parse_args()

chk = VerifiedChecker()
met = json.loads((args.run / "metrics.json").read_text())
h, v = met["assumptions"]["h_sep_m"], met["assumptions"]["v_sep_m"]
H, V = int(round(h * 1000)), int(round(v * 1000))
raw = json.loads((args.run / "intents.json").read_text())["missions"]
boxes = {int(k): [tuple(b) for b in d["int"]] for k, d in raw.items()}
phase = {int(k): d.get("phase", ["en_route"] * len(d["int"])) for k, d in raw.items()}
enr = {m: [b for b, ph in zip(boxes[m], phase[m]) if ph == "en_route"] for m in boxes}
tr = np.load(args.run / "tracks.npz")
times, tracks = tr["times"], tr["tracks"]
events = json.loads((args.run / "events.json").read_text())["los"]


def span(bx):
    return (min(b[6] for b in bx), max(b[7] for b in bx)) if bx else None


# ---------------------------------------------------------------- 1. strategic
t0 = time.time()
cert, cert_all, n_pairs, mism = set(), set(), 0, 0
sp_e = {m: span(b) for m, b in enr.items()}
sp_a = {m: span(b) for m, b in boxes.items()}
for a, b in itertools.combinations(sorted(boxes), 2):
    n_pairs += 1
    for src, sp, out in ((enr, sp_e, cert), (boxes, sp_a, cert_all)):
        A, B = src[a], src[b]
        if not A or not B or sp[a][1] < sp[b][0] or sp[b][1] < sp[a][0]:
            out.add((a, b))
            continue
        bad = chk.count_against_hv(A, B, H, V)
        mism += bad != int((~separated_hv_matrix(A, B, H, V)).sum())
        if bad == 0:
            out.add((a, b))
t_sep = time.time() - t0

# ---------------------------------------------------------------- 2. conformance
t0 = time.time()
state = {}                      # (mission, k) -> "en_route" | "terminal" | "none"
conf_tot = conf_any = conf_enr = conf_mism = 0
for m in sorted(boxes):
    for k, t in enumerate(times):
        p = tracks[k, m]
        if np.isnan(p[0]) or p[2] < 0.5:
            continue
        xi, yi, zi, ti = (int(round(p[0] * 1000)), int(round(p[1] * 1000)), int(round(p[2] * 1000)),
                          int(round(t * 1000)))
        hits = [chk.inside(b, 0, xi, yi, zi, ti) for b in boxes[m]]
        conf_mism += hits != [inside_ref(b, 0, xi, yi, zi, ti) for b in boxes[m]]
        in_e = any(hh for hh, ph in zip(hits, phase[m]) if ph == "en_route")
        in_any = any(hits)
        conf_tot += 1
        conf_any += in_any
        conf_enr += in_e
        state[(m, k)] = "en_route" if in_e else ("terminal" if in_any else "none")
t_conf = time.time() - t0

# ---------------------------------------------------------------- 3. consistency
cex, prem_term, prem_nonconf = [], [], []
for e in events:
    pair = (min(e["a"], e["b"]), max(e["a"], e["b"]))
    if pair not in cert:
        continue
    ks = np.flatnonzero((times >= e["t_start"] - 0.5) & (times <= e["t_end"] + 0.5))
    st = [state[(m, int(k))] for m in pair for k in ks if (m, int(k)) in state]   # airborne samples only
    if "none" in st:
        prem_nonconf.append(e)
    elif "terminal" in st:
        prem_term.append(e)
    else:
        cex.append(e)

report = {
    "run": str(args.run), "theorem": "2 (cylinder: horizontal >= h or vertical >= v)",
    "thresholds_m": {"h": h, "v": v},
    "checker": {"backend": "Lean UtmChecker.separatedHV via libutm_checker.so", "abi": chk.lib.utm_version()},
    "strategic": {"mission_pairs": n_pairs,
                  "certified_en_route": len(cert), "certified_rate": round(len(cert) / n_pairs, 4) if n_pairs else None,
                  "certified_all_boxes": len(cert_all),
                  "certified_all_rate": round(len(cert_all) / n_pairs, 4) if n_pairs else None,
                  "reference_mismatches": mism, "seconds": round(t_sep, 2)},
    "conformance": {"samples": conf_tot, "in_any_box": conf_any, "in_en_route_box": conf_enr,
                    "rate": round(conf_any / conf_tot, 6) if conf_tot else None,
                    "reference_mismatches": conf_mism, "seconds": round(t_conf, 2)},
    "los_events": {"total": len(events), "en_route": sum(not e["terminal"] for e in events),
                   "between_certified_pairs": len(cex),
                   "certified_premise_terminal": len(prem_term),
                   "certified_premise_nonconforming": len(prem_nonconf)},
    "counterexamples": cex[:20],
    "premise_nonconforming": prem_nonconf[:20],
    "verdict": ("CONSISTENT: no loss of separation between certified pairs inside en-route intents"
                if not cex else "COUNTEREXAMPLES FOUND: check assumptions or code"),
}
(args.run / "audit.json").write_text(json.dumps(report, indent=1))
print(json.dumps({k: report[k] for k in ("thresholds_m", "strategic", "conformance", "los_events", "verdict")}, indent=1))
sys.exit(0 if not cex and not mism and not conf_mism else 1)
