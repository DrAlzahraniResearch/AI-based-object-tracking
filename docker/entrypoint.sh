#!/usr/bin/env bash
# Source ROS 2 (if installed) and run the given command inside the twin venv.
set -e
if [[ -f /opt/ros/lyrical/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/lyrical/setup.bash
fi
exec "$@"
