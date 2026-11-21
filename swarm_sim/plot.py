"""Top-down plots of an airspace and simulated traffic (matplotlib, headless)."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402


def _base(ax, air):
    if air.buildings:
        hs = np.array([b.height for b in air.buildings])
        pc = PolyCollection([np.array(b.polygon) for b in air.buildings], array=hs, cmap="Greys",
                            edgecolors="0.3", linewidths=0.3, alpha=0.9)
        pc.set_clim(0, max(hs.max(), 1))
        ax.add_collection(pc)
        plt.colorbar(pc, ax=ax, fraction=0.035, pad=0.01, label="building height (m)")
    for r in air.roads:
        a = np.array(r.line)
        ax.plot(a[:, 0], a[:, 1], color="0.75", lw=1.2 if r.kind in ("primary", "secondary", "tertiary") else 0.6,
                zorder=0)
    for n in air.no_fly:
        a = np.array(n.polygon)
        ax.fill(a[:, 0], a[:, 1], color="red", alpha=0.15, lw=0)
        ax.plot(a[:, 0], a[:, 1], color="red", lw=0.8)
        ax.text(a[:, 0].mean(), a[:, 1].mean(), "no-fly", color="red", ha="center", fontsize=7)
    for h in air.hubs:
        pads = np.array(h.pad_positions())
        ax.scatter(pads[:, 0], pads[:, 1], marker="s", s=18, color="orange", edgecolors="k", lw=0.4, zorder=5)
        ax.text(h.x, h.y + 12, h.name, ha="center", fontsize=7, zorder=6)
    xmin, ymin, xmax, ymax = air.bounds
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")


def plot_airspace(air, path: Path):
    fig, ax = plt.subplots(figsize=(8, 8), dpi=110)
    _base(ax, air)
    ax.set_title(f"{air.name}: {len(air.buildings)} buildings, {len(air.no_fly)} no-fly, {len(air.hubs)} hubs",
                 fontsize=9)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_traffic(air, tracks: dict, path: Path, events: list | None = None, title: str = ""):
    """tracks: {drone_id: (T,3) array}; events: [{"x":, "y":, ...}] loss-of-separation locations."""
    fig, ax = plt.subplots(figsize=(8, 8), dpi=110)
    _base(ax, air)
    cmap = plt.get_cmap("tab20")
    for k, (did, tr) in enumerate(sorted(tracks.items())):
        tr = np.asarray(tr)
        ax.plot(tr[:, 0], tr[:, 1], lw=0.8, color=cmap(k % 20), alpha=0.85)
    if events:
        ax.scatter([e["x"] for e in events], [e["y"] for e in events], marker="x", color="red", s=30, lw=1.2,
                   zorder=7, label=f"loss of separation ({len(events)})")
        ax.legend(loc="lower right", fontsize=7)
    ax.set_title(title, fontsize=9)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
