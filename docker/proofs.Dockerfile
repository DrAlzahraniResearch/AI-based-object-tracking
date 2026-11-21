# Lean 4.36.0 + mathlib v4.36.0: checks the proofs, builds libutm_checker.{so,a}, runs the FFI tests.
#   docker compose -f docker/compose.yaml build proofs
#   docker compose -f docker/compose.yaml run --rm proofs            # lake build + make test + pytest
#
# The mathlib cache (`lake exe cache get`) downloads prebuilt .olean files, so the first
# build takes minutes, not the ~1 h a from-source mathlib build needs. The .lake directory lives
# in a named volume so it survives container restarts.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 PIP_BREAK_SYSTEM_PACKAGES=1
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl git make gcc libc6-dev python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSfL https://raw.githubusercontent.com/leanprover/elan/master/elan-init.sh \
      | sh -s -- -y --default-toolchain leanprover/lean4:v4.36.0
ENV PATH=/root/.elan/bin:$PATH
# Bake the toolchain into the image (elan otherwise downloads it lazily into the container, where it is
# lost with --rm; libutm_checker.so's rpath points at this directory).
RUN elan toolchain install leanprover/lean4:v4.36.0 \
    && elan default leanprover/lean4:v4.36.0 \
    && test -f /root/.elan/toolchains/leanprover--lean4---v4.36.0/lib/lean/libleanshared.so

COPY requirements.txt /tmp/req/
RUN pip3 install -r /tmp/req/requirements.txt

WORKDIR /work
ENV PYTHONPATH=/work
CMD ["bash", "-c", "cd proofs && lake exe cache get && lake build && lake env lean UtmProofs/Axioms.lean && cd .. \
     && make -C runtime test \
     && pytest -q tests/test_checker_ffi.py tests/test_scheduler.py::test_verified_search_matches_reference"]
