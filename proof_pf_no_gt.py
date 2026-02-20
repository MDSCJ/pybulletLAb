#!/usr/bin/env python3
"""
proof_pf_no_gt.py — Proof that the PF-based autonomy works WITHOUT ground-truth.

Runs TWO headless autonomous navigation sessions and produces:
  1. pf_proof.png — Dual-panel comparison (maze vs warehouse)
  2. Console summary table of key metrics for each map.

Usage:
    python proof_pf_no_gt.py
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np

from lab4.config import SimConfig
from lab4.simulation import Simulation
from shared.utils.map_picker import pick_map


# ── Helpers ──────────────────────────────────────────────────────────────────
def _load_pf_csv(run_dir: Path) -> dict[str, np.ndarray]:
    pf_csv = run_dir / "pf_agent0.csv"
    if not pf_csv.exists():
        print(f"[ERROR] PF log not found: {pf_csv}")
        sys.exit(1)
    raw: dict[str, list] = {}
    with open(pf_csv) as f:
        for row in csv.DictReader(f):
            for k, v in row.items():
                raw.setdefault(k, []).append(float(v))
    return {k: np.array(v) for k, v in raw.items()}


def _print_summary(label: str, d: dict[str, np.ndarray]) -> None:
    t, err_xy = d["t"], d["err_xy"]
    err_th = d.get("err_th", np.zeros_like(t))
    neff = d["neff"]
    err_odom = d.get("err_odom", np.zeros_like(t))
    print("\n" + "=" * 60)
    print(f"  PF METRICS — {label}  (no GT in control)")
    print("=" * 60)
    print(f"  {'Metric':<30s} {'Value':>10s}")
    print(f"  {'-'*30} {'-'*10}")
    print(f"  {'Mean PF error (m)':<30s} {np.mean(err_xy):>10.4f}")
    print(f"  {'Max  PF error (m)':<30s} {np.max(err_xy):>10.4f}")
    print(f"  {'Final PF error (m)':<30s} {err_xy[-1]:>10.4f}")
    print(f"  {'Mean odom error (m)':<30s} {np.mean(err_odom):>10.4f}")
    print(f"  {'Final odom error (m)':<30s} {err_odom[-1]:>10.4f}")
    print(f"  {'Mean heading error (°)':<30s} {np.degrees(np.mean(err_th)):>10.1f}")
    print(f"  {'Mean Neff':<30s} {np.mean(neff):>10.1f}")
    print(f"  {'Min  Neff':<30s} {np.min(neff):>10.1f}")
    print(f"  {'Samples':<30s} {len(t):>10d}")
    print(f"  {'Duration (s)':<30s} {t[-1]:>10.2f}")
    print("=" * 60)


def _run_one(map_name: str, n_particles: int = 800, n_rays: int = 72,
             max_s: float = 60.0, seed: int = 42, n_humans: int = 2,
             inject_pct: float = 0.01, dense_interval: int = 5,
             dense_rays: int = 120) -> dict[str, np.ndarray]:
    map_path = pick_map("shared/maps", direct=map_name)

    cfg = SimConfig(
        map_path=map_path,
        cell_size=0.5,
        hz=50.0,
        n_particles=n_particles,
        scan_period=0.2,
        n_lidar_rays=n_rays,
        nav_enabled=True,
        goal_mode="random",
        replan_interval=3.0,
        max_sim_s=max_s,
        direct=True,
        seed=seed,
        n_robots=1,
        n_humans=n_humans,
        speed_multiplier=8,
        pf_random_inject_pct=inject_pct,
        pf_dense_scan_interval=dense_interval,
        pf_dense_scan_rays=dense_rays,
    )

    print("=" * 60)
    print(f"  RUN: {map_name}")
    print(f"  Particles: {n_particles}  |  Rays: {n_rays}")
    print(f"  Random inject: {inject_pct*100:.0f}% | Dense every {dense_interval} ({dense_rays} rays)")
    print("=" * 60)

    sim = Simulation(cfg)
    try:
        result = sim.run()
    finally:
        run_dir = Path(sim.logger.run_dir)
        sim.shutdown()

    print(f"\nResult ({map_name}): dist={result.get('distance',0):.1f}m  "
          f"pf_err={result.get('mean_pf_error',0):.2f}m  "
          f"gt_use={result.get('gt_usage_pct',0):.1f}%")

    return _load_pf_csv(run_dir)


# ── Main ─────────────────────────────────────────────────────────────────────
def run_proof():
    # 1) Maze — distinct corridors → PF converges well
    d_maze = _run_one("maze_lab4", n_particles=800, n_rays=72,
                      max_s=60.0, n_humans=0, seed=42)
    _print_summary("maze_lab4", d_maze)

    # 2) Warehouse — highly symmetric shelves → PF struggles (expected)
    d_wh = _run_one("warehouse_small", n_particles=800, n_rays=72,
                    max_s=60.0, n_humans=2, seed=42)
    _print_summary("warehouse_small", d_wh)

    # ── Plot ─────────────────────────────────────────────────────────────────
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib not available — skipping plot.")
        return

    fig, axes = plt.subplots(3, 2, figsize=(14, 9), sharex="col")

    for col, (label, d) in enumerate([("maze_lab4", d_maze),
                                       ("warehouse_small", d_wh)]):
        t = d["t"]
        err_xy = d["err_xy"]
        err_odom = d.get("err_odom", np.zeros_like(t))
        err_th = d.get("err_th", np.zeros_like(t))
        neff = d["neff"]

        # Row 0: position error
        axes[0, col].plot(t, err_xy, "r-", lw=0.8,
                          label=f"PF (mean={np.mean(err_xy):.2f} m)")
        axes[0, col].plot(t, err_odom, "b-", lw=0.8, alpha=0.6,
                          label=f"Odom (mean={np.mean(err_odom):.2f} m)")
        axes[0, col].set_ylabel("Pos error (m)")
        axes[0, col].set_title(label, fontweight="bold")
        axes[0, col].legend(fontsize=7)
        axes[0, col].grid(True, alpha=0.3)

        # Row 1: heading error
        axes[1, col].plot(t, np.degrees(err_th), "m-", lw=0.8)
        axes[1, col].axhline(np.degrees(np.mean(err_th)), color="grey",
                              ls="--", lw=0.6,
                              label=f"mean={np.degrees(np.mean(err_th)):.1f}°")
        axes[1, col].set_ylabel("Heading error (°)")
        axes[1, col].legend(fontsize=7)
        axes[1, col].grid(True, alpha=0.3)

        # Row 2: Neff
        axes[2, col].plot(t, neff, "b-", lw=0.8)
        axes[2, col].axhline(np.mean(neff), color="grey", ls="--", lw=0.6,
                              label=f"mean={np.mean(neff):.0f}")
        axes[2, col].set_ylabel("Neff")
        axes[2, col].set_xlabel("Time (s)")
        axes[2, col].legend(fontsize=7)
        axes[2, col].grid(True, alpha=0.3)

    fig.suptitle("PF-Corrected Odometry — No Ground-Truth in Control Loop\n"
                 "(800 particles, 72 rays, cluster estimate, position-only fusion)",
                 fontsize=11)
    fig.tight_layout()

    out_path = Path("pf_proof.png")
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"\nPlot saved: {out_path.resolve()}")


if __name__ == "__main__":
    run_proof()
