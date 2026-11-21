#!/usr/bin/env bash
# End-to-end Tier A demo on the bundled synthetic town (no network needed):
# OSM -> world, then {no policy, ORCA} x {no scheduler, FCFS} on seeds 1-3 at x1 and x5 demand,
# audits (if the verified checker is built here), comparison table.
set -euo pipefail
cd "$(dirname "$0")/.."
python scripts/osm_to_world.py --osm tests/fixtures/mini_town.osm --name mini_town \
  --hubs configs/hubs_mini_town.yaml
for scale in 1 5; do
  for s in 1 2 3; do
    for policy in none orca; do
      for sched in none fcfs; do
        python scripts/run_tier_a.py --world data/worlds/mini_town --config configs/tier_a.yaml \
          --seed "$s" --rate-scale "$scale" --policy "$policy" --scheduler "$sched"
      done
    done
  done
done
if python -c "from runtime.utm_checker import VerifiedChecker; VerifiedChecker()" 2>/dev/null; then
  for r in runs/mini_town_*_s*_x*; do python scripts/audit_run.py "$r"; done
else
  echo "verified checker not built here: run the audits in the proofs container"
fi
python scripts/compare_policies.py runs --world mini_town
