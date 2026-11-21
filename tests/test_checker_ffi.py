"""The compiled Lean checker (via C/ctypes) agrees with the Python reference, and is sound on random cases.

Skipped when runtime/build/libutm_checker.so has not been built (`make -C runtime`).
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from runtime.utm_checker import CheckerUnavailable, VerifiedChecker  # noqa: E402
from swarm_sim.intents import (inside_ref, separated_dev_ref, separated_hv_matrix, separated_hv_ref,  # noqa: E402
                               separated_ref)  # noqa: E402

try:
    CHK = VerifiedChecker()
except CheckerUnavailable as e:  # pragma: no cover
    pytest.skip(f"verified checker not built: {e}", allow_module_level=True)


def _rand_box(rng, span=10_000, size=4_000):
    lo = rng.integers(-span, span, 4)
    hi = lo + rng.integers(0, size, 4)
    return tuple(int(v) for pair in zip(lo, hi) for v in pair)


def test_known_cases():
    a = (0, 10, 0, 10, 0, 10, 0, 10)
    assert CHK.separated(a, (15, 20, 0, 10, 0, 10, 0, 10), 5)
    assert not CHK.separated(a, (14, 20, 0, 10, 0, 10, 0, 10), 5)
    assert CHK.separated(a, (0, 10, 0, 10, 0, 10, 11, 20), 5)
    assert CHK.inside(a, 2, 5, 5, 5, 5) and not CHK.inside(a, 2, 9, 5, 5, 5)


def test_agrees_with_python_reference():
    rng = np.random.default_rng(42)
    for _ in range(20_000):
        a, b = _rand_box(rng), _rand_box(rng)
        buf = int(rng.integers(-100, 2_000))          # negative buffers too: the checker must not care
        assert CHK.separated(a, b, buf) == separated_ref(a, b, buf)
        m = int(rng.integers(0, 500))
        x, y, z, t = (int(v) for v in rng.integers(-12_000, 12_000, 4))
        assert CHK.inside(a, m, x, y, z, t) == inside_ref(a, m, x, y, z, t)


def test_extreme_int64_values_cross_the_abi():
    big = 2 ** 62
    a = (-big, -big + 10, 0, 1, 0, 1, 0, 1)
    b = (big - 10, big, 0, 1, 0, 1, 0, 1)
    assert CHK.separated(a, b, big)                    # gap ~ 2^63 crosses the boundary intact
    with pytest.raises(ValueError):
        CHK.separated((2 ** 63,) + a[1:], b, 0)


def test_no_int64_overflow_false_positive():
    """Overlapping boxes, huge buffer: b.x1 + buf exceeds int64. Naive C int64 arithmetic wraps to a
    negative number and would wrongly report 'separated'; the Lean checker computes in unbounded Int."""
    big, buf = 2 ** 62, 2 ** 63 - 1
    a = (-big, 0, 0, 1, 0, 1, 0, 1)
    b = (0, big, 0, 1, 0, 1, 0, 1)
    wrapped = ((b[1] + buf + 2 ** 63) % 2 ** 64) - 2 ** 63     # what int64 addition would produce
    assert wrapped <= a[0]                                     # i.e. a naive check would say True
    assert not separated_ref(a, b, buf)
    assert not CHK.separated(a, b, buf)


def test_soundness_by_sampling():
    """Theorem 1 as a property: accepted pairs never have co-temporal points closer than buf."""
    rng = np.random.default_rng(7)
    accepted = 0
    for _ in range(5_000):
        a, b = _rand_box(rng, 3_000, 2_000), _rand_box(rng, 3_000, 2_000)
        buf = int(rng.integers(0, 1_500))
        if not CHK.separated(a, b, buf):
            continue
        t_lo, t_hi = max(a[6], b[6]), min(a[7], b[7])
        if t_lo > t_hi:
            continue
        accepted += 1
        for _ in range(10):
            p = [rng.uniform(a[2 * k], a[2 * k + 1]) for k in range(3)]
            q = [rng.uniform(b[2 * k], b[2 * k + 1]) for k in range(3)]
            assert math.dist(p, q) >= buf - 1e-6
    assert accepted > 50


def test_count_conflicts_matches_pairwise():
    rng = np.random.default_rng(3)
    boxes = [_rand_box(rng, 5_000, 3_000) for _ in range(120)]
    groups = rng.integers(0, 30, len(boxes))
    expected = sum(1 for i in range(len(boxes)) for j in range(i + 1, len(boxes))
                   if groups[i] != groups[j] and not separated_ref(boxes[i], boxes[j], 250))
    assert CHK.count_conflicts(boxes, groups, 250) == expected


# ------------------------------------------------------------------ Theorem 2 (cylinder form)
def test_hv_known_cases():
    a = (0, 10, 0, 10, 0, 10, 0, 10)
    above = (0, 10, 0, 10, 15, 20, 0, 10)                        # z gap 5
    beside = (15, 20, 0, 10, 0, 10, 0, 10)                       # x gap 5
    assert CHK.separated_hv(a, above, 100, 5) and not CHK.separated_hv(a, above, 100, 6)
    assert CHK.separated_hv(a, beside, 5, 100) and not CHK.separated_hv(a, beside, 6, 100)
    assert not CHK.separated(a, above, 6)                        # Theorem 1 needs the gap on one axis >= buf


def test_hv_agrees_with_references():
    rng = np.random.default_rng(11)
    A = [_rand_box(rng) for _ in range(300)]
    B = [_rand_box(rng) for _ in range(300)]
    for _ in range(3):
        h, v = int(rng.integers(0, 3_000)), int(rng.integers(0, 3_000))
        M = separated_hv_matrix(A, B, h, v)
        for i in range(0, 300, 7):
            for j in range(300):
                assert CHK.separated_hv(A[i], B[j], h, v) == separated_hv_ref(A[i], B[j], h, v) == bool(M[i, j])
        assert CHK.count_against_hv(A, B, h, v) == int((~M).sum())


def test_hv_soundness_no_loss_of_separation():
    """Theorem 2 as a property: accepted pairs never have co-temporal points in LoS."""
    rng = np.random.default_rng(5)
    accepted = 0
    for _ in range(5_000):
        a, b = _rand_box(rng, 3_000, 2_000), _rand_box(rng, 3_000, 2_000)
        h, v = int(rng.integers(0, 1_500)), int(rng.integers(0, 800))
        if not CHK.separated_hv(a, b, h, v) or max(a[6], b[6]) > min(a[7], b[7]):
            continue
        accepted += 1
        for _ in range(10):
            p = [rng.uniform(a[2 * k], a[2 * k + 1]) for k in range(3)]
            q = [rng.uniform(b[2 * k], b[2 * k + 1]) for k in range(3)]
            assert math.hypot(p[0] - q[0], p[1] - q[1]) >= h - 1e-6 or abs(p[2] - q[2]) >= v - 1e-6
    assert accepted > 50


def test_count_conflicts_hv_matches_pairwise():
    rng = np.random.default_rng(9)
    boxes = [_rand_box(rng, 5_000, 3_000) for _ in range(120)]
    groups = rng.integers(0, 30, len(boxes))
    expected = sum(1 for i in range(len(boxes)) for j in range(i + 1, len(boxes))
                   if groups[i] != groups[j] and not separated_hv_ref(boxes[i], boxes[j], 300, 100))
    assert CHK.count_conflicts_hv(boxes, groups, 300, 100) == expected


def test_dev_known_cases_and_reference():
    a, beside = (0, 10, 0, 10, 0, 10, 0, 10), (15, 20, 0, 10, 0, 10, 0, 10)        # x gap 5
    later = (0, 10, 0, 10, 0, 10, 11, 20)                                            # time gap 1
    assert CHK.separated_dev(a, beside, 1, 100, 2, 0, 0) and not CHK.separated_dev(a, beside, 1, 100, 3, 0, 0)
    assert CHK.separated_dev(a, later, 5, 5, 0, 0, 0) and not CHK.separated_dev(a, later, 5, 5, 0, 0, 1)
    rng = np.random.default_rng(13)
    for _ in range(3_000):
        x, y = _rand_box(rng), _rand_box(rng)
        h, v, eh, ev, dt = (int(rng.integers(0, 2_000)) for _ in range(5))
        assert CHK.separated_dev(x, y, h, v, eh, ev, dt) == separated_dev_ref(x, y, h, v, eh, ev, dt)
    A = [_rand_box(rng) for _ in range(40)]
    B = [_rand_box(rng) for _ in range(60)]
    assert CHK.count_against_dev(A, B, 300, 100, 50, 20, 10) == sum(
        not separated_dev_ref(p, q, 300, 100, 50, 20, 10) for p in A for q in B)


def test_dev_soundness_tracking_error():
    """Theorem 3 as a property: nominal points in accepted boxes, actual points within the error bound
    (and timing error within dt), never in loss of separation."""
    rng = np.random.default_rng(17)
    accepted = 0
    for _ in range(5_000):
        a, b = _rand_box(rng, 3_000, 1_500), _rand_box(rng, 3_000, 1_500)
        h, v, eh, ev, dt = (int(rng.integers(0, 600)) for _ in range(5))
        if not CHK.separated_dev(a, b, h, v, eh, ev, dt):
            continue
        accepted += 1
        for _ in range(10):
            t = rng.uniform(min(a[6], b[6]) - dt, max(a[7], b[7]) + dt)
            ta, tb = t + rng.uniform(-dt, dt), t + rng.uniform(-dt, dt)        # nominal times within dt
            if not (a[6] <= ta <= a[7] and b[6] <= tb <= b[7]):
                continue
            p0 = [rng.uniform(a[2 * k], a[2 * k + 1]) for k in range(3)]
            q0 = [rng.uniform(b[2 * k], b[2 * k + 1]) for k in range(3)]
            p = [p0[0] + rng.uniform(-eh, eh), p0[1] + rng.uniform(-eh, eh), p0[2] + rng.uniform(-ev, ev)]
            q = [q0[0] + rng.uniform(-eh, eh), q0[1] + rng.uniform(-eh, eh), q0[2] + rng.uniform(-ev, ev)]
            assert math.hypot(p[0] - q[0], p[1] - q[1]) >= h - 1e-6 or abs(p[2] - q[2]) >= v - 1e-6
    assert accepted > 50
