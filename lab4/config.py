"""
config.py — Simulation configuration dataclass.
"""
from dataclasses import dataclass
from typing import Optional, Tuple

@dataclass
class SimConfig:
    """All tunable simulation parameters."""
    # World
    map_path: str = ""
    cell_size: float = 0.5
    hz: float = 50.0
    friction: float = 0.05

    # Particle filter
    n_particles: int = 800
    scan_period: float = 0.2
    n_lidar_rays: int = 72
    pf_init_std_xy: float = 0.3
    pf_init_std_th: float = 0.2

    # Navigation
    nav_enabled: bool = False
    goal_mode: str = "random"    # "random", "top", "rc"
    goal_rc: Optional[Tuple[int, int]] = None
    goal_margin: int = 1
    replan_interval: float = 4.0

    # PF degraded-mode thresholds
    pf_degraded_enter: float = 1.5   # enter degraded if PF err > this
    pf_degraded_exit: float = 0.8    # exit degraded if PF err < this

    # Emergency recovery
    emergency_idle_s: float = 5.0
    emergency_forward_s: float = 1.2
    emergency_pf_runs: int = 3

    # Kidnapped-robot PF recovery
    pf_random_inject_pct: float = 0.01    # fraction of particles replaced with random free-space samples each resample
    pf_dense_scan_interval: int = 5       # every N scans, use a denser lidar sweep
    pf_dense_scan_rays: int = 120         # number of rays for the dense sweep

    # Autotest scenario
    autotest: bool = False
    max_sim_s: float = 60.0
    v_straight: float = 1.4
    w_turn: float = 1.3
    v_curve: float = 1.0
    w_curve: float = 0.8
    curve_s: float = 15.0

    # Display
    direct: bool = False   # headless mode

    # Seed
    seed: Optional[int] = None

    # Multi-job (Phase 3)
    n_jobs: int = 0

    # Humans (Phase 4)
    n_humans: int = 0

    # Multi-robot (Phase 5)
    n_robots: int = 1

    # Speed multiplier
    speed_multiplier: int = 8

    # SLAM (Phase 6)
    slam_enabled: bool = False
