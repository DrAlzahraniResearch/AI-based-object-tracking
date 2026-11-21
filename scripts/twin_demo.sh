#!/usr/bin/env bash
# Milestone 4.5: the digital twin against the Tier A plant (model mismatch + a mid-flight fault), with
# ablations, plus a conflict-prediction benchmark; verified audits when the checker is built here.
#   docker compose -f docker/compose.yaml run --rm eval bash scripts/twin_demo.sh            # ~30-40 min
#   docker compose -f docker/compose.yaml run --rm proofs bash -c \
#     "for r in runs_twin/mini_town_*; do python3 scripts/audit_twin.py \$r; done"         # verified audits
# The PX4 (Tier B) run is separate: docker compose -f docker/compose.yaml run --rm px4
set -euo pipefail
cd "$(dirname "$0")/.."
[ -f data/worlds/mini_town/airspace.json ] || python scripts/osm_to_world.py --osm tests/fixtures/mini_town.osm \
  --name mini_town --hubs configs/hubs_mini_town.yaml
PY=${PYTHON:-python}
V=""
if $PY -c "from runtime.utm_checker import VerifiedChecker; VerifiedChecker()" 2>/dev/null; then V="--verified"; fi
W="--world data/worlds/mini_town"
for scale in 1 2; do
  for s in 1 2 3; do
    $PY scripts/run_twin.py $W --seed $s --rate-scale $scale $V --no-faults               # matched plant
    $PY scripts/run_twin.py $W --seed $s --rate-scale $scale $V --no-replan --no-calibrate # open loop
    $PY scripts/run_twin.py $W --seed $s --rate-scale $scale $V                           # full twin
  done
done
for s in 1 2 3; do
  $PY scripts/run_twin.py $W --seed $s --rate-scale 2 $V --no-calibrate                   # replan only
  $PY scripts/run_twin.py $W --seed $s --rate-scale 2 $V --no-replan                      # calibrate only
  $PY scripts/run_twin.py $W --seed $s --rate-scale 2 $V --no-faults --admit-all --policy none   # prediction benchmark
done
if [ -n "$V" ]; then
  for r in runs_twin/mini_town_tier_a*; do $PY scripts/audit_twin.py "$r" > /dev/null && echo "audit ok: $r"; done
else
  echo "verified checker not built here: run scripts/audit_twin.py in the proofs container"
fi
$PY scripts/compare_twin.py runs_twin --world mini_town
