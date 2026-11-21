# Tier A + analysis image: no ROS, no Gazebo, no Lean. Same base as the verified MOT eval image.
#   docker compose -f docker/compose.yaml run --rm eval                 # pytest (FFI tests skip without the library)
#   docker compose -f docker/compose.yaml run --rm eval python scripts/run_tier_a.py --world data/worlds/mini_town
FROM python:3.13-slim-bookworm

ENV PYTHONDONTWRITEBYTECODE=1 PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1 MPLBACKEND=Agg
RUN apt-get update && apt-get install -y --no-install-recommends git make \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
COPY requirements.txt /tmp/req/
RUN pip install -r /tmp/req/requirements.txt
ENV PYTHONPATH=/work
CMD ["pytest", "-q"]
