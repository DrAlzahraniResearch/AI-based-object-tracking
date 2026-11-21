# Gazebo image: Ubuntu 26.04 LTS, ROS 2 Lyrical, Gazebo Jetty. Same install script and base as the
# image already verified on the MOT repo (Jetty 10.7.0). Used here to load and render the OSM worlds.
# PX4 SITL (Tier B, milestone 6) targets Ubuntu 24.04 + Jetty and will get its own image.
#
#   docker compose -f docker/compose.yaml build sim
#   docker compose -f docker/compose.yaml run --rm sim gz sim -s -r --iterations 100 data/worlds/mini_town/world.sdf
#
# CPU rendering uses Mesa llvmpipe (slow but deterministic). With an NVIDIA GPU
# and the NVIDIA Container Toolkit, use the "sim-gpu" service instead.
FROM ubuntu:26.04

ARG WITH_ROS_GZ=0
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 DEBIAN_FRONTEND=noninteractive

COPY infra/install_stack.sh /tmp/install_stack.sh
COPY requirements.txt /tmp/repo/
RUN bash /tmp/install_stack.sh --venv /opt/twin-venv --repo /tmp/repo \
      $( [ "$WITH_ROS_GZ" = "1" ] && echo --with-ros-gz ) \
    && rm -rf /var/lib/apt/lists/* /tmp/repo

ENV VIRTUAL_ENV=/opt/twin-venv \
    PATH=/opt/twin-venv/bin:$PATH \
    PYTHONPATH=/work \
    GZ_PARTITION=swarm \
    QT_QPA_PLATFORM=offscreen

WORKDIR /work
COPY docker/entrypoint.sh /usr/local/bin/twin-entrypoint
RUN chmod +x /usr/local/bin/twin-entrypoint
ENTRYPOINT ["twin-entrypoint"]
CMD ["bash"]
