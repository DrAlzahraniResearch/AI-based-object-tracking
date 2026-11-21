#!/usr/bin/env bash
# Milestone 5: hub sequencing off / compact hubs / spread hubs (40 m pads) at x1, x2, x5 demand (seeds 1-3),
# matched plant, ORCA + FCFS; plus sequencing on the mismatched plant (fleet 15 % slow, a degraded vehicle) at x2.
# Every configuration flies the same missions (approach fixes 100 m out). Runs JOBS at a time (default 2).
#   docker compose -f docker/compose.yaml run --rm eval bash scripts/hubs_demo.sh            # ~1-2 h
#   docker compose -f docker/compose.yaml run --rm proofs bash -c \
#     "for r in runs_twin/mini_town_tier_a*fix* runs_twin/mini_town_tier_a*hubseq*; do python3 scripts/audit_twin.py \$r > /dev/null && echo ok \$r; done"
set -euo pipefail
cd "$(dirname "$0")/.."
[ -f data/worlds/mini_town/airspace.json ] || python scripts/osm_to_world.py --osm tests/fixtures/mini_town.osm \
  --name mini_town --hubs configs/hubs_mini_town.yaml
PY=${PYTHON:-python}
W="--world data/worlds/mini_town"
{
  for scale in 5 2 1; do
    for s in 1 2 3; do
      echo "$W --seed $s --rate-scale $scale --no-faults --approach-fix"
      echo "$W --seed $s --rate-scale $scale --no-faults --hubs sequence"
      echo "$W --seed $s --rate-scale $scale --no-faults --hubs sequence --pad-spacing 40"
    done
  done
  for s in 1 2 3; do echo "$W --seed $s --rate-scale 2 --hubs sequence"; done
} | xargs -P "${JOBS:-2}" -I{} sh -c "$PY scripts/run_twin.py {} > /dev/null 2>&1 && echo done: {}"
if $PY -c "from runtime.utm_checker import VerifiedChecker; VerifiedChecker()" 2>/dev/null; then
  for r in runs_twin/mini_town_tier_a*fix_* runs_twin/mini_town_tier_a*hubseq*; do
    $PY scripts/audit_twin.py "$r" > /dev/null && echo "audit ok: $r"
  done
fi
$PY scripts/compare_hubs.py runs_twin --world mini_town
