#!/usr/bin/env bash
# The twin, live in Gazebo: run_twin.py publishes on the gz topic /swarm/twin (--gz-publish) and gz_view.py --live
# draws it while it runs. Both in one container, so gz-transport discovery needs no network setup.
#
# From the repo root (images: px4gz for either plant; gz is enough for tier_a):
#   docker compose -f docker/compose.yaml run --rm px4gz bash scripts/live_demo.sh
#       Tier A plant at 4x real time, headless: prints what the viewer receives and draws, then a summary
#   docker compose -f docker/compose.yaml --profile gui run --rm gz-gui bash scripts/live_demo.sh --gui
#       ... and watch it in the Gazebo GUI (vehicles, intent boxes in force, predictions)
#   docker compose -f docker/compose.yaml run --rm px4gz bash scripts/live_demo.sh --plant px4gz --missions 3
#       PX4 x500s in Gazebo; the viewer adds only markers (boxes, predictions, labels) around the real drones
#
# Options: --plant tier_a|px4gz  --world DIR  --pace X (tier_a: sim s per wall s, default 4)  --gui
#          --duration S (tier_a: stop the twin after S wall seconds, default 120; 0 = whole run)
# Anything else goes to run_twin.py (e.g. --rate-scale 2, --hubs reserve, --missions 3, --verified).
# Exit code: 0 if the viewer received the twin's messages, non-zero otherwise.
set -euo pipefail
cd "$(dirname "$0")/.."

WORLD=data/worlds/mini_town PLANT=tier_a PACE=4 GUI=0 DURATION=120
TWIN_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --plant) PLANT="$2"; shift ;;
    --world) WORLD="$2"; shift ;;
    --pace) PACE="$2"; shift ;;
    --duration) DURATION="$2"; shift ;;
    --gui) GUI=1 ;;
    *) TWIN_ARGS+=("$1") ;;
  esac
  shift
done
[[ "$PLANT" == tier_a || "$PLANT" == px4gz ]] || { echo "--plant must be tier_a or px4gz" >&2; exit 2; }

if [[ ! -f "$WORLD/airspace.json" ]]; then
  if [[ "$(basename "$WORLD")" == mini_town ]]; then
    echo "== building $WORLD from the test fixture"
    python3 scripts/osm_to_world.py --osm tests/fixtures/mini_town.osm --name mini_town \
      --hubs configs/hubs_mini_town.yaml --out "$(dirname "$WORLD")" > /dev/null
  else
    echo "no world at $WORLD (scripts/osm_to_world.py)" >&2; exit 2
  fi
fi

TMP=$(mktemp -d /tmp/live_demo.XXXX)
READY="$TMP/viewer.ready"
VIEW_PID="" TWIN_PID=""
cleanup() {
  for p in $TWIN_PID $VIEW_PID; do kill "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

VIEW_ARGS=(--live --world "$WORLD" --ready-file "$READY" --exit-after-idle 20 --first-wait 600 --status-every 10)
GUI_ARGS=()
[[ $GUI == 1 ]] && GUI_ARGS=(--gui)

if [[ "$PLANT" == tier_a ]]; then
  # The viewer owns the world; the twin's Tier A plant needs none.
  echo "== viewer: starting Gazebo with $WORLD/world.sdf"
  python3 -u scripts/gz_view.py "${VIEW_ARGS[@]}" --launch "${GUI_ARGS[@]}" > >(sed -u 's/^/[viewer] /') 2>&1 &
  VIEW_PID=$!
  for _ in $(seq 180); do [[ -f "$READY" ]] && break; sleep 1; done
  [[ -f "$READY" ]] || { echo "viewer did not come up (see /tmp/gz_view/gz_server.log)" >&2; exit 1; }
  echo "== twin: Tier A plant at ${PACE}x real time, publishing on /swarm/twin"
  TWIN_CMD=(python3 -u scripts/run_twin.py --world "$WORLD" --gz-publish --pace "$PACE" --tag live "${TWIN_ARGS[@]}")
  if [[ "$DURATION" != 0 ]]; then
    echo "   (stopping the twin after ${DURATION} s of wall time, report kept; --duration 0 for the whole run)"
    timeout --signal=INT "$DURATION" "${TWIN_CMD[@]}" > >(sed -u 's/^/[twin] /') 2>&1 || true
  else
    "${TWIN_CMD[@]}" > >(sed -u 's/^/[twin] /') 2>&1
  fi
else
  # The twin's PX4 plant owns the world (with the x500s); the viewer only adds markers to it.
  [[ $GUI == 1 ]] && TWIN_ARGS+=(--gz-gui)
  echo "== twin: PX4 x500s in Gazebo, publishing on /swarm/twin"
  python3 -u scripts/run_twin.py --world "$WORLD" --plant px4gz --gz-publish --tag live "${TWIN_ARGS[@]}" \
    > >(sed -u 's/^/[twin] /') 2>&1 &
  TWIN_PID=$!
  python3 -u scripts/gz_view.py "${VIEW_ARGS[@]}" --no-vehicles > >(sed -u 's/^/[viewer] /') 2>&1 &
  VIEW_PID=$!
  wait "$TWIN_PID" || true
  TWIN_PID=""
fi

echo "== twin done; waiting for the viewer to drain (it stops 20 s after the last message)"
set +e
wait "$VIEW_PID"
RC=$?
set -e
VIEW_PID=""
sleep 0.5                                   # let the prefixing sed flush the viewer's last lines
if [[ $RC == 0 ]]; then
  echo "== live link OK: the viewer received and drew the twin (summary line above)"
else
  echo "== live link FAILED: the viewer received nothing from the twin (exit $RC)"
fi
exit $RC
