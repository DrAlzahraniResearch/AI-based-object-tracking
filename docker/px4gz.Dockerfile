# Tier B in Gazebo: PX4 SITL v1.16.2 built with its Gazebo (Harmonic) bridge + MAVSDK, on the gz image
# (Ubuntu 24.04 + Gazebo Harmonic). The verified checker's runtime (libleanshared.so) is copied from the proofs
# image, which is also 24.04, so --verified works here as in the px4 image.
#
#   docker compose -f docker/compose.yaml build px4gz              # builds gz and proofs first (compose build contexts)
#   docker compose -f docker/compose.yaml run --rm proofs          # builds runtime/build/libutm_checker.so (once)
#   docker compose -f docker/compose.yaml run --rm px4gz           # twin + 3 x500s in the OSM world (see CMD)
#
# The PX4 build takes 10-30 min the first time. Each x500 needs about one CPU core at real time.
# `gz` and `proofs` are compose build contexts (docker/compose.yaml: additional_contexts), i.e. those services' images.
FROM proofs AS lean
FROM gz

# The build and simulation subsets of PX4's Tools/setup/ubuntu.sh (no NuttX toolchain, no gz install: the base
# image has gz-harmonic). OpenCV and GStreamer are for PX4's optical-flow and camera Gazebo plugins.
RUN apt-get update && apt-get install -y --no-install-recommends \
      cmake file g++ gcc libssl-dev libxml2-dev libxml2-utils ninja-build python3-setuptools python3-wheel \
      rsync unzip zip bc cppzmq-dev libunwind-dev libeigen3-dev libopencv-dev protobuf-compiler pkg-config \
      gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-libav \
      libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

ARG PX4_TAG=v1.16.2
# Shallow clone, as in px4.Dockerfile; Tools/simulation/gz (PX4-gazebo-models: x500, worlds) is a submodule.
RUN git clone --depth 1 --branch ${PX4_TAG} https://github.com/PX4/PX4-Autopilot.git /opt/px4 \
    && cd /opt/px4 \
    && git -c submodule."boards/modalai/voxl2/libfc-sensor-api".update=none \
         submodule update --init --recursive --depth 1 --jobs 8 \
    && (git -C platforms/nuttx/NuttX/nuttx tag nuttx-12.12.0 || true) \
    && test -f Tools/simulation/gz/models/x500/model.sdf
ENV GZ_DISTRO=harmonic
RUN pip install --no-cache-dir -r /opt/px4/Tools/setup/requirements.txt "empy==3.3.4" "mavsdk==3.0.1" \
    && cd /opt/px4 && make px4_sitl_default \
    && test -x /opt/px4/build/px4_sitl_default/bin/px4 \
    && test -e /opt/px4/build/px4_sitl_default/bin/px4-gz_bridge      # the Gazebo bridge was built

# Verified checker runtime (libutm_checker.so in /work/runtime/build has its rpath here).
COPY --from=lean /root/.elan/toolchains/leanprover--lean4---v4.36.0/lib/lean/libleanshared.so \
     /root/.elan/toolchains/leanprover--lean4---v4.36.0/lib/lean/libleanshared.so

ENV PX4_ROOT=/opt/px4
WORKDIR /work
CMD ["python3", "scripts/run_twin.py", "--world", "data/worlds/mini_town", "--plant", "px4gz", "--verified"]
