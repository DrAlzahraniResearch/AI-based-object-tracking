"""Offline audit of a digital-twin run with the Lean-verified checker.

    python scripts/audit_twin.py runs_twin/mini_town_tier_a_s1_x1

The twin's plans change while it runs (admission, then replans), so the audit works on plan VERSIONS
(intents.json): each version of a flight's intent boxes is in force over [valid_from, valid_to).

1. Claims. Whenever two versions were in force together, the one registered later was checked online
   against the other. If the later one was certified ("accepted" / "replanned"), the verified checker
   must agree that the two versions' en-route boxes, clipped to the window where both were in force, are
   cylinder-separated (Theorem 2). A disagreement is a CLAIM VIOLATION (a bug in the online search).
   Which boxes of a version were certified is recorded with it (`cert`): en-route boxes, plus with hub
   sequencing (milestone 5) the departure climb and, once sequenced, the arrival.
2. Conformance against GROUND TRUTH (the plant's true positions, not the twin's estimates): each sample
   is checked with the verified `inside` against the version in force at that time.
3. Consistency. A sample where two flights are in loss of separation, both inside certified boxes of
   versions that are certified against each other, contradicts Theorem 2: a COUNTEREXAMPLE. LoS events
   (from the plant) are also classified as in audit_run.py: uncertified, premise_terminal,
   premise_nonconforming, counterexample.
Every verified decision is cross-checked against the Python reference.
"""
import argparse
import itertools
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from runtime.utm_checker import VerifiedChecker  # noqa: E402
from swarm_sim.intents import inside_ref, separated_hv_matrix  # noqa: E402

CERTIFIED = {"accepted", "replanned", "sequenced", "awaiting"}

ap = argparse.ArgumentParser()
ap.add_argument("run", type=Path)
ap.add_argument("--h", type=float, default=30.0)
ap.add_argument("--v", type=float, default=10.0)
args = ap.parse_args()

chk = VerifiedChecker()
H, V = int(round(args.h * 1000)), int(round(args.v * 1000))
vers = json.loads((args.run / "intents.json").read_text())["versions"]
for k, x in enumerate(vers):
    x["k"] = k
    cert = x.get("cert") or [ph == "en_route" for ph in x["phase"]]       # 4.5 logs: en-route boxes certified
    x["cert_mask"] = cert
    x["enr"] = np.array([b for b, c in zip(x["int"], cert) if c], np.int64).reshape(-1, 8)
    x["all"] = np.array(x["int"], np.int64).reshape(-1, 8)


def clip(B, lo_s, hi_s):
    if len(B) == 0:
        return B
    lo, hi = math.floor(lo_s * 1000), math.ceil(hi_s * 1000)
    C = B.copy()
    C[:, 6] = np.maximum(C[:, 6], lo)
    C[:, 7] = np.minimum(C[:, 7], hi)
    return C[C[:, 6] <= C[:, 7]]


# ---------------------------------------------------------------- 1. claims
t0 = time.time()
pair_cert = {}                  # (k_x, k_y) -> separated within the common window
claims = violations = mism = checked = 0
viol_list = []
for x, y in itertools.combinations(vers, 2):
    if x["vid"] == y["vid"]:
        continue
    lo, hi = max(x["valid_from"], y["valid_from"]), min(x["valid_to"], y["valid_to"])
    if lo > hi:
        continue
    A, B = clip(x["enr"], lo, hi), clip(y["enr"], lo, hi)
    later = y if (y["valid_from"], y["k"]) > (x["valid_from"], x["k"]) else x
    if len(A) == 0 or len(B) == 0:
        ok = True
    else:
        checked += 1
        bad = chk.count_against_hv(A, B, H, V)
        mism += bad != int((~separated_hv_matrix(A, B, H, V)).sum())
        ok = bad == 0
    pair_cert[(x["k"], y["k"])] = pair_cert[(y["k"], x["k"])] = ok
    if later["status"] in CERTIFIED:
        claims += 1
        if not ok:
            violations += 1
            viol_list.append({"vids": [x["vid"], y["vid"]], "window": [lo, hi], "later_status": later["status"]})
t_claims = time.time() - t0

# ---------------------------------------------------------------- 2. truth samples
tr = np.load(args.run / "truth.npz")
samples = {}                    # t -> {vid: pos}
if "tracks" in tr:
    times_, tracks_ = tr["times"], tr["tracks"]            # load once: NpzFile decompresses on every access
    for k, t in enumerate(times_):
        row = tracks_[k].copy()
        samples[round(float(t), 3)] = {int(i): row[i] for i in np.flatnonzero(~np.isnan(row[:, 0]))}
else:
    for t, vid, x, y, z in np.array(tr["px4_track"]):
        samples.setdefault(round(float(t), 1), {})[int(vid)] = np.array([x, y, z])
by_vid = {}
for x in vers:
    by_vid.setdefault(x["vid"], []).append(x)


def version_at(vid, t):
    for x in by_vid.get(vid, []):
        if x["valid_from"] <= t < x["valid_to"] or (t == x["valid_to"] and x is by_vid[vid][-1]):
            return x
    return None


t0 = time.time()
state = {}
tot = in_any = in_enr = in_cert = cmism = 0
for t, pts in samples.items():
    for vid, p in pts.items():
        if p[2] < 0.5:
            continue
        x = version_at(vid, t)
        if x is None:
            continue
        xi, yi, zi, ti = (int(round(c * 1000)) for c in (p[0], p[1], max(p[2], 0.0), t))   # below ground = on it
        live = np.flatnonzero((x["all"][:, 6] <= ti) & (ti <= x["all"][:, 7]))
        hits = {int(j): chk.inside(x["all"][j], 0, xi, yi, zi, ti) for j in live}
        cmism += any(h != inside_ref(tuple(int(c) for c in x["all"][j]), 0, xi, yi, zi, ti) for j, h in hits.items())
        enr = any(h for j, h in hits.items() if x["phase"][j] == "en_route")
        crt = any(h for j, h in hits.items() if x["cert_mask"][j])
        tot += 1
        in_any += any(hits.values())
        in_enr += enr
        in_cert += crt
        state[(vid, t)] = ("certified" if crt else "uncertified_box" if any(hits.values()) else "none", x["k"])
t_conf = time.time() - t0

# ---------------------------------------------------------------- 3. consistency (every sample, exact)
cex_samples, los_samples = [], 0
for t, pts in samples.items():
    ids = [v for v in pts if pts[v][2] >= 0.5]
    for a, b in itertools.combinations(sorted(ids), 2):
        pa, pb = pts[a], pts[b]
        if math.hypot(pa[0] - pb[0], pa[1] - pb[1]) >= args.h or abs(pa[2] - pb[2]) >= args.v:
            continue
        los_samples += 1
        sa, sb = state.get((a, t)), state.get((b, t))
        if sa and sb and sa[0] == sb[0] == "certified" and pair_cert.get((sa[1], sb[1]), False):
            cex_samples.append({"t": t, "vids": [a, b], "pos": [pa.round(2).tolist(), pb.round(2).tolist()]})

ev_path = args.run / "events.json"
events = json.loads(ev_path.read_text())["los"] if ev_path.exists() else []
cls = {"uncertified": 0, "premise_terminal": 0, "premise_nonconforming": 0, "counterexample": 0, "unsampled": 0}
ts = np.array(sorted(samples))
for e in events:
    ks = ts[(ts >= e["t_start"] - 0.5) & (ts <= e["t_end"] + 0.5)]
    st = [(state.get((e["a"], t)), state.get((e["b"], t))) for t in ks]
    st = [s for s in st if s[0] and s[1]]
    if not st:
        cls["unsampled"] += 1
    elif not all(pair_cert.get((sa[1], sb[1]), False) for sa, sb in st):
        cls["uncertified"] += 1
    elif any("none" in (sa[0], sb[0]) for sa, sb in st):
        cls["premise_nonconforming"] += 1
    elif any("uncertified_box" in (sa[0], sb[0]) for sa, sb in st):
        cls["premise_terminal"] += 1          # inside its plan, but in boxes nobody certified (terminal, not sequenced)
    else:
        cls["counterexample"] += 1

report = {
    "run": str(args.run), "theorem": "2 (cylinder), per plan version",
    "thresholds_m": {"h": args.h, "v": args.v},
    "checker": {"backend": "Lean UtmChecker via libutm_checker.so", "abi": chk.lib.utm_version()},
    "versions": {"total": len(vers), "by_status": {s: sum(x["status"] == s for x in vers)
                                                    for s in sorted({x["status"] for x in vers})}},
    "claims": {"concurrent_pairs_with_certified_claim": claims, "checked_with_boxes": checked,
               "violations": violations, "reference_mismatches": mism, "seconds": round(t_claims, 2)},
    "truth_conformance": {"samples": tot, "in_any_box": in_any, "in_en_route_box": in_enr, "in_certified_box": in_cert,
                          "rate": round(in_any / tot, 6) if tot else None, "reference_mismatches": cmism,
                          "seconds": round(t_conf, 2)},
    "los": {"samples_in_los": los_samples, "counterexample_samples": len(cex_samples),
            "events": len(events), "events_by_class": cls},
    "claim_violations": viol_list[:20], "counterexamples": cex_samples[:20],
    "verdict": ("CONSISTENT: online certifications confirmed by the verified checker; no loss of separation "
                "between certified versions inside their certified boxes"
                if not violations and not cex_samples and not cls["counterexample"] else
                "INCONSISTENT: see claim_violations / counterexamples"),
}
(args.run / "audit.json").write_text(json.dumps(report, indent=1))
print(json.dumps({k: report[k] for k in ("versions", "claims", "truth_conformance", "los", "verdict")}, indent=1))
sys.exit(0 if not violations and not cex_samples and not cls["counterexample"] and not mism and not cmism else 1)
