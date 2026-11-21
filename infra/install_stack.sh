#!/usr/bin/env bash
# Install the simulation stack on Ubuntu 26.04 LTS (Resolute Raccoon).
# Used by docker/sim.Dockerfile AND vbox/provision.sh, so the container and the VM match.
#
#   ROS 2 Lyrical Luth (LTS, Tier 1 on 26.04)   ros-lyrical-ros-base + foxglove-bridge
#   Gazebo Jetty (the Lyrical pairing)          gz-jetty + Python bindings for gz-transport / gz-msgs
#   Python venv (--system-site-packages)        repo requirements; sees rclpy and the gz bindings
#
# Usage: infra/install_stack.sh [--no-ros] [--no-gazebo] [--with-ros-gz] [--venv /opt/twin-venv] [--repo DIR]
# --with-ros-gz installs ros-lyrical-ros-gz (vendored Gazebo) for the Week-5 live pipeline. Do not mix it
# with the OSRF gz-jetty packages in one shell environment without checking GZ_VERSION; see docs/digital_twin_wam.md.
set -euo pipefail

WITH_ROS=1 WITH_GZ=1 WITH_ROS_GZ=0 VENV=/opt/twin-venv REPO=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-ros) WITH_ROS=0 ;;
    --no-gazebo) WITH_GZ=0 ;;
    --with-ros-gz) WITH_ROS_GZ=1 ;;
    --venv) VENV="$2"; shift ;;
    --repo) REPO="$2"; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
  shift
done

SUDO=""
[[ $(id -u) -ne 0 ]] && SUDO="sudo"
export DEBIAN_FRONTEND=noninteractive

. /etc/os-release
CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"
if [[ "${CODENAME}" != "resolute" ]]; then
  echo "WARNING: expected Ubuntu 26.04 (resolute), found ${PRETTY_NAME}. Lyrical binaries are Tier 1 only on 26.04." >&2
fi

$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends \
  ca-certificates curl gnupg lsb-release locales software-properties-common \
  git build-essential python3 python3-venv python3-pip python3-dev \
  ffmpeg libgl1 libegl1 libgl1-mesa-dri mesa-utils
$SUDO locale-gen en_US en_US.UTF-8
$SUDO update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$SUDO add-apt-repository -y universe

if [[ $WITH_ROS -eq 1 ]]; then
  # Official method (docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html): ros2-apt-source package
  ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
  $SUDO dpkg -i /tmp/ros2-apt-source.deb
  $SUDO apt-get update
  # ros-base already contains tf2_ros, sensor_msgs and visualization_msgs used by tracking_twin.ros_bridge
  $SUDO apt-get install -y --no-install-recommends ros-dev-tools ros-lyrical-ros-base
  $SUDO apt-get install -y --no-install-recommends ros-lyrical-foxglove-bridge \
    || echo "WARNING: ros-lyrical-foxglove-bridge not available; build it from source or use rosbridge" >&2
  if [[ $WITH_ROS_GZ -eq 1 ]]; then
    $SUDO apt-get install -y --no-install-recommends ros-lyrical-ros-gz
  fi
fi

if [[ $WITH_GZ -eq 1 ]]; then
  # Gazebo Jetty from packages.osrfoundation.org (gazebosim.org/docs/jetty/install_ubuntu)
  $SUDO curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable ${CODENAME} main" \
    | $SUDO tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
  $SUDO apt-get update
  $SUDO apt-get install -y --no-install-recommends gz-jetty
  # Jetty dropped version numbers from package names; fall back to versioned names if needed.
  $SUDO apt-get install -y --no-install-recommends python3-gz-transport python3-gz-msgs \
    || $SUDO apt-get install -y --no-install-recommends python3-gz-transport15 python3-gz-msgs12 \
    || echo "WARNING: gz Python bindings not found as apt packages; gz_render_flight.py will not run" >&2
fi

# Python environment for the repo (system site packages -> rclpy + gz bindings visible)
VSUDO=""
mkdir -p "$(dirname "$VENV")" 2>/dev/null || true
[[ -w "$(dirname "$VENV")" ]] || VSUDO="$SUDO"
$VSUDO python3 -m venv --system-site-packages "$VENV"
$VSUDO "$VENV/bin/pip" install --upgrade pip
for req in requirements.txt requirements-twin.txt; do   # swarm repo uses requirements.txt
  if [[ -n "$REPO" && -f "$REPO/$req" ]]; then
    $VSUDO "$VENV/bin/pip" install -r "$REPO/$req"
  fi
done

$SUDO apt-get clean
echo "install_stack: done (ROS=${WITH_ROS} Gazebo=${WITH_GZ} ros_gz=${WITH_ROS_GZ} venv=${VENV})"
