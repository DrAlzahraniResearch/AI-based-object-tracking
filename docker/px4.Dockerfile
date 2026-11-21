# Tier B: PX4 SITL v1.16.2 with its built-in simulator (SIH: no Gazebo, headless) + MAVSDK, built on the
# proofs image so the twin can use the Lean-verified checker (libleanshared.so is already there).
#   docker compose -f docker/compose.yaml build proofs px4        # proofs first: px4 builds FROM it
#   docker compose -f docker/compose.yaml run --rm proofs          # builds runtime/build/libutm_checker.so
#   docker compose -f docker/compose.yaml run --rm px4             # twin + 2 PX4 vehicles (see CMD)
#
# The PX4 build takes 10-25 min the first time. One PX4 instance per vehicle; each needs ~1 CPU core at
# speed factor 1, so on a 4-core laptop keep missions x speed <= ~3.
FROM civilian-swarm/proofs:latest

# The build subset of PX4's Tools/setup/ubuntu.sh (no simulators, no NuttX toolchain).
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake file g++ gcc libssl-dev libxml2-dev libxml2-utils ninja-build \
      python3-dev python3-setuptools python3-wheel rsync unzip zip procps \
    && rm -rf /var/lib/apt/lists/*

ARG PX4_TAG=v1.16.2
# Shallow clone. The ModalAI VOXL2 board submodule (GitLab) is not needed for SITL. The version script
# wants a NuttX release tag, which a shallow submodule clone does not carry.
RUN git clone --depth 1 --branch ${PX4_TAG} https://github.com/PX4/PX4-Autopilot.git /opt/px4 \
    && cd /opt/px4 \
    && git -c submodule."boards/modalai/voxl2/libfc-sensor-api".update=none \
         submodule update --init --recursive --depth 1 --jobs 8 \
    && (git -C platforms/nuttx/NuttX/nuttx tag nuttx-12.12.0 || true)
RUN pip3 install -r /opt/px4/Tools/setup/requirements.txt "empy==3.3.4" "mavsdk==3.0.1" \
    && cd /opt/px4 && make px4_sitl_default \
    && test -x /opt/px4/build/px4_sitl_default/bin/px4

ENV PX4_ROOT=/opt/px4
WORKDIR /work
CMD ["python3", "scripts/run_twin.py", "--world", "data/worlds/mini_town", "--plant", "px4", "--verified"]
