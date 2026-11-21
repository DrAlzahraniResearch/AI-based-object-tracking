"""Python binding (ctypes) for the Lean-verified separation checker.

    from runtime.utm_checker import VerifiedChecker
    chk = VerifiedChecker()                       # loads runtime/build/libutm_checker.so, starts the Lean runtime
    chk.separated(a, b, buf_mm)                   # a, b: (x0,x1,y0,y1,z0,z1,t0,t1) in mm / ms  (Theorem 1)
    chk.separated_hv(a, b, h_mm, v_mm)            # cylinder form: no LoS                       (Theorem 2)
    chk.separated_dev(a, b, h_mm, v_mm, eh_mm, ev_mm, dt_ms)   # nominal boxes + tracking error (Theorem 3)
    chk.inside(box, margin_mm, x, y, z, t)
    chk.count_conflicts(boxes, groups, buf_mm)    # pairs from different groups that are NOT separated

Build the library first: `make -C runtime` (needs the Lean toolchain; see docs/proofs.md).
Results are bool; the decision itself is the Lean function `Utm.separated` / `Utm.inside`,
whose soundness is proved in proofs/UtmProofs/Refinement.lean.
"""
from __future__ import annotations

import ctypes
import os
from pathlib import Path

import numpy as np

_DEFAULT = Path(__file__).resolve().parent / "build" / "libutm_checker.so"
_I64x8 = ctypes.c_int64 * 8
ABI_VERSION = 3
INT64_MIN, INT64_MAX = -(2 ** 63), 2 ** 63 - 1


class CheckerUnavailable(RuntimeError):
    pass


def _lean_libdir() -> Path | None:
    """Directory holding libleanshared.so: $LEAN_PREFIX, else the toolchain pinned by proofs/."""
    prefix = os.environ.get("LEAN_PREFIX")
    if not prefix:
        import shutil
        import subprocess
        if shutil.which("lean") is None:
            return None
        try:
            prefix = subprocess.run(["lean", "--print-prefix"], cwd=Path(__file__).resolve().parents[1] / "proofs",
                                    capture_output=True, text=True, timeout=60, check=True).stdout.strip()
        except (subprocess.SubprocessError, OSError):
            return None
    d = Path(prefix) / "lib" / "lean"
    return d if (d / "libleanshared.so").exists() else None


def _load(p: Path) -> ctypes.CDLL:
    """Load the library; if the baked-in rpath to libleanshared.so is stale, find the runtime and preload it."""
    try:
        return ctypes.CDLL(str(p))
    except OSError as e:
        if "libleanshared" not in str(e):
            raise CheckerUnavailable(str(e)) from e
        d = _lean_libdir()
        if d is None:
            raise CheckerUnavailable(
                "libleanshared.so (Lean runtime) not found. Install the toolchain pinned in proofs/lean-toolchain, "
                "or set LEAN_PREFIX to it.") from e
        ctypes.CDLL(str(d / "libleanshared.so"), mode=ctypes.RTLD_GLOBAL)
        return ctypes.CDLL(str(p))


class VerifiedChecker:
    _lib = None

    def __init__(self, path: str | os.PathLike | None = None):
        if VerifiedChecker._lib is None:
            p = Path(path or os.environ.get("UTM_CHECKER_LIB", _DEFAULT))
            if not p.exists():
                raise CheckerUnavailable(f"{p} not found; build it with `make -C runtime`")
            lib = _load(p)
            lib.utm_init.restype = ctypes.c_int
            lib.utm_version.restype = ctypes.c_uint32
            lib.utm_check_separated.argtypes = [_I64x8, _I64x8, ctypes.c_int64]
            lib.utm_check_separated.restype = ctypes.c_int
            lib.utm_check_separated_hv.argtypes = [_I64x8, _I64x8, ctypes.c_int64, ctypes.c_int64]
            lib.utm_check_separated_hv.restype = ctypes.c_int
            P = ctypes.POINTER(ctypes.c_int64)
            lib.utm_count_conflicts_hv.argtypes = [P, P, ctypes.c_int64, ctypes.c_int64, ctypes.c_int64]
            lib.utm_count_conflicts_hv.restype = ctypes.c_int64
            lib.utm_count_against_hv.argtypes = [P, ctypes.c_int64, P, ctypes.c_int64, ctypes.c_int64, ctypes.c_int64]
            lib.utm_count_against_hv.restype = ctypes.c_int64
            lib.utm_check_separated_dev.argtypes = [_I64x8, _I64x8] + [ctypes.c_int64] * 5
            lib.utm_check_separated_dev.restype = ctypes.c_int
            lib.utm_count_against_dev.argtypes = [P, ctypes.c_int64, P, ctypes.c_int64] + [ctypes.c_int64] * 5
            lib.utm_count_against_dev.restype = ctypes.c_int64
            lib.utm_check_inside.argtypes = [_I64x8] + [ctypes.c_int64] * 5
            lib.utm_check_inside.restype = ctypes.c_int
            lib.utm_count_conflicts.argtypes = [ctypes.POINTER(ctypes.c_int64), ctypes.POINTER(ctypes.c_int64),
                                                ctypes.c_int64, ctypes.c_int64]
            lib.utm_count_conflicts.restype = ctypes.c_int64
            rc = lib.utm_init()
            if rc != 0:
                raise CheckerUnavailable(f"utm_init failed ({rc})")
            if lib.utm_version() != ABI_VERSION:
                raise CheckerUnavailable(f"ABI {lib.utm_version()} != {ABI_VERSION}; rebuild runtime/")
            VerifiedChecker._lib = lib
        self.lib = VerifiedChecker._lib

    @staticmethod
    def _box(b) -> _I64x8:
        vals = [int(v) for v in b]
        if len(vals) != 8 or not all(INT64_MIN <= v <= INT64_MAX for v in vals):
            raise ValueError(f"box must be 8 int64 values, got {b}")
        return _I64x8(*vals)

    def separated(self, a, b, buf: int) -> bool:
        r = self.lib.utm_check_separated(self._box(a), self._box(b), int(buf))
        if r < 0:
            raise CheckerUnavailable("checker not initialised")
        return bool(r)

    def inside(self, box, margin: int, x: int, y: int, z: int, t: int) -> bool:
        r = self.lib.utm_check_inside(self._box(box), int(margin), int(x), int(y), int(z), int(t))
        if r < 0:
            raise CheckerUnavailable("checker not initialised")
        return bool(r)

    def count_conflicts(self, boxes, groups, buf: int) -> int:
        B = np.ascontiguousarray(np.asarray(boxes, dtype=np.int64).reshape(-1, 8))
        G = np.ascontiguousarray(np.asarray(groups, dtype=np.int64).reshape(-1))
        if len(B) != len(G):
            raise ValueError("boxes and groups differ in length")
        P = ctypes.POINTER(ctypes.c_int64)
        return int(self.lib.utm_count_conflicts(B.ctypes.data_as(P), G.ctypes.data_as(P), len(B), int(buf)))

    def separated_hv(self, a, b, h: int, v: int) -> bool:
        r = self.lib.utm_check_separated_hv(self._box(a), self._box(b), int(h), int(v))
        if r < 0:
            raise CheckerUnavailable("checker not initialised")
        return bool(r)

    @staticmethod
    def _arr(boxes):
        return np.ascontiguousarray(np.asarray(boxes, dtype=np.int64).reshape(-1, 8))

    def count_conflicts_hv(self, boxes, groups, h: int, v: int) -> int:
        B = self._arr(boxes)
        G = np.ascontiguousarray(np.asarray(groups, dtype=np.int64).reshape(-1))
        if len(B) != len(G):
            raise ValueError("boxes and groups differ in length")
        P = ctypes.POINTER(ctypes.c_int64)
        return int(self.lib.utm_count_conflicts_hv(B.ctypes.data_as(P), G.ctypes.data_as(P), len(B), int(h), int(v)))

    def count_against_hv(self, cand, others, h: int, v: int) -> int:
        """Pairs (candidate box, other box) that are NOT cylinder-separated."""
        C, O = self._arr(cand), self._arr(others)
        P = ctypes.POINTER(ctypes.c_int64)
        return int(self.lib.utm_count_against_hv(C.ctypes.data_as(P), len(C), O.ctypes.data_as(P), len(O),
                                                 int(h), int(v)))

    def separated_dev(self, a, b, h: int, v: int, eh: int, ev: int, dt: int) -> bool:
        """Theorem 3: nominal boxes a, b; tracking error eh (per horizontal axis), ev (vertical), dt (time)."""
        r = self.lib.utm_check_separated_dev(self._box(a), self._box(b), int(h), int(v), int(eh), int(ev), int(dt))
        if r < 0:
            raise CheckerUnavailable("checker not initialised")
        return bool(r)

    def count_against_dev(self, cand, others, h: int, v: int, eh: int, ev: int, dt: int) -> int:
        C, O = self._arr(cand), self._arr(others)
        P = ctypes.POINTER(ctypes.c_int64)
        return int(self.lib.utm_count_against_dev(C.ctypes.data_as(P), len(C), O.ctypes.data_as(P), len(O),
                                                  int(h), int(v), int(eh), int(ev), int(dt)))

    def inside_many(self, boxes, margin: int, x: int, y: int, z: int, t: int) -> np.ndarray:
        """inside() for each row of `boxes` (n x 8) -> (n,) bool."""
        return np.array([self.inside(b, margin, x, y, z, t) for b in self._arr(boxes)], bool)
