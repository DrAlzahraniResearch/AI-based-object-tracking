# Gazebo image for Tier B and the viewer: Ubuntu 24.04 + Gazebo Harmonic (the pair PX4 v1.16.2 supports; its
# px4-rc.gzsim requires gz-sim >= 8 and its Tools/setup/ubuntu.sh installs gz-harmonic on 24.04).
# No ROS. The `sim` image (26.04 + Jetty) stays for the MOT-compatible stack; this one is what px4gz builds FROM.
#
#   docker compose -f docker/compose.yaml build gz
#   docker compose -f docker/compose.yaml run --rm gz python3 scripts/gz_view.py runs_twin/<run> --launch \
#       --record runs_twin/<run>/gazebo.mp4                       # headless video (CPU rendering, EGL)
#   docker compose -f docker/compose.yaml run --rm gz-gui python3 scripts/gz_view.py runs_twin/<run> --launch --gui
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg lsb-release git python3 python3-pip python3-venv python3-dev build-essential \
      ffmpeg libgl1 libegl1 libgl1-mesa-dri mesa-utils procps \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends gz-harmonic \
    && (apt-get install -y --no-install-recommends python3-gz-transport13 python3-gz-msgs10 \
        || echo "WARNING: gz-transport Python bindings not installed: gz_view.py and --gz-publish will not run" >&2) \
    && gz sim --versions \
    && rm -rf /var/lib/apt/lists/*

# Repo requirements in a venv that still sees the apt-installed gz bindings.
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m venv --system-site-packages /opt/swarm-venv \
    && /opt/swarm-venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt \
    && (/opt/swarm-venv/bin/python -c "import gz.transport13, gz.msgs10.stringmsg_pb2; print('gz.transport13 ok')" \
        || echo "WARNING: gz.transport13 not importable from the venv" >&2)

ENV VIRTUAL_ENV=/opt/swarm-venv \
    PATH=/opt/swarm-venv/bin:$PATH \
    PYTHONPATH=/work \
    GZ_PARTITION=swarm
WORKDIR /work
CMD ["bash"]
