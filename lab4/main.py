"""
main.py — CLI entry point for the AMR warehouse simulation.
Usage:
    python -m lab4.main --map warehouse_small --nav --goal_random --direct --seed 42
    python -m lab4.main --nav --goal_random          # interactive map picker + GUI
    python -m lab4.main                               # teleop mode
"""
from __future__ import annotations

import argparse
from shared.utils.map_picker import pick_map
from lab4.config import SimConfig
from lab4.simulation import Simulation


def main():
    parser = argparse.ArgumentParser(description="AMR Warehouse Simulation")

    # ── World ──
    parser.add_argument("--map", type=str, default=None,
                        help="Map file (name, filename, or full path)")
    parser.add_argument("--cell_size", type=float, default=0.5)
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--friction", type=float, default=0.05)

    # ── Particle filter ──
    parser.add_argument("--n", type=int, default=800, help="Number of particles")
    parser.add_argument("--scan_period", type=float, default=0.2)
    parser.add_argument("--angles_n", type=int, default=72, help="Number of lidar rays")

    # ── Navigation ──
    parser.add_argument("--nav", action="store_true", help="Enable autonomous navigation")
    parser.add_argument("--goal_rc", type=str, default=None, help="Goal as 'row,col'")
    parser.add_argument("--goal_top", action="store_true")
    parser.add_argument("--goal_random", action="store_true")
    parser.add_argument("--goal_margin", type=int, default=1)
    parser.add_argument("--replan_s", type=float, default=2.0)

    # ── Emergency ──
    parser.add_argument("--emergency_idle_s", type=float, default=5.0)
    parser.add_argument("--emergency_forward_s", type=float, default=1.2)
    parser.add_argument("--emergency_pf_runs", type=int, default=3)

    # ── Autotest ──
    parser.add_argument("--autotest", action="store_true")
    parser.add_argument("--max_sim_s", type=float, default=60.0)
    parser.add_argument("--v_straight", type=float, default=3.5)
    parser.add_argument("--w_turn", type=float, default=3.5)
    parser.add_argument("--v_curve", type=float, default=2.5)
    parser.add_argument("--w_curve", type=float, default=2.5)
    parser.add_argument("--curve_s", type=float, default=15.0)

    # ── Display / mode ──
    parser.add_argument("--direct", action="store_true", help="Headless mode (no GUI)")
    parser.add_argument("--seed", type=int, default=None)

    # ── Multi-job (Phase 3) ──
    parser.add_argument("--jobs", type=int, default=0, help="Number of pick/drop jobs")

    # ── Humans (Phase 4) ──
    parser.add_argument("--humans", type=int, default=0, help="Number of simulated humans")

    # ── Multi-robot (Phase 5) ──
    parser.add_argument("--robots", type=int, default=1, help="Number of robots")

    # ── SLAM (Phase 6) ──
    parser.add_argument("--slam", action="store_true", help="Enable SLAM mode")

    # ── Speed ──
    parser.add_argument("--speed", type=int, default=8, help="Physics speed multiplier")

    args = parser.parse_args()

    # ── Resolve map ──
    if args.direct and args.map:
        map_path = pick_map("shared/maps", direct=args.map)
    elif args.map:
        map_path = pick_map("shared/maps", direct=args.map)
    else:
        map_path = pick_map("shared/maps", default="warehouse_small.txt")

    # ── Determine goal mode ──
    goal_mode = "random"
    goal_rc = None
    if args.goal_rc:
        goal_mode = "rc"
        rr, cc = args.goal_rc.split(",")
        goal_rc = (int(rr), int(cc))
    elif args.goal_top:
        goal_mode = "top"

    # ── Build config ──
    cfg = SimConfig(
        map_path=map_path,
        cell_size=args.cell_size,
        hz=args.hz,
        friction=args.friction,
        n_particles=args.n,
        scan_period=args.scan_period,
        n_lidar_rays=args.angles_n,
        nav_enabled=args.nav,
        goal_mode=goal_mode,
        goal_rc=goal_rc,
        goal_margin=args.goal_margin,
        replan_interval=args.replan_s,
        emergency_idle_s=args.emergency_idle_s,
        emergency_forward_s=args.emergency_forward_s,
        emergency_pf_runs=args.emergency_pf_runs,
        autotest=args.autotest,
        max_sim_s=args.max_sim_s,
        v_straight=args.v_straight,
        w_turn=args.w_turn,
        v_curve=args.v_curve,
        w_curve=args.w_curve,
        curve_s=args.curve_s,
        direct=args.direct,
        seed=args.seed,
        n_jobs=args.jobs,
        n_humans=args.humans,
        n_robots=args.robots,
        slam_enabled=args.slam,
        speed_multiplier=args.speed,
    )

    # ── Run ──
    sim = Simulation(cfg)
    try:
        result = sim.run()
        print(f"\n=== RESULT ===")
        print(f"  success:       {result['success']}")
        print(f"  sim_time:      {result['sim_time']:.2f}s")
        print(f"  distance:      {result['distance']:.2f}m")
        print(f"  mean_pf_error: {result['mean_pf_error']:.3f}m")
        print(f"  gt_usage:      {result.get('gt_usage_pct', 0):.1f}%")
    finally:
        sim.shutdown()


if __name__ == "__main__":
    main()
