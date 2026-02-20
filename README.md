# AMR Warehouse Simulation — PyBulletLabs (Lab 4)

Autonomous Mobile Robot (AMR) warehouse simulation built with **PyBullet**.  
Demonstrates planning, localization, SLAM, multi-robot coordination, and human-aware navigation in warehouse-like environments.

### Key Features
- **Ground-truth-free autonomy** — control loop uses odometry corrected by particle filter (no GT pose for decisions)
- **Kidnapped-robot recovery** — random particle injection + periodic dense lidar scans for re-localization
- **Articulated human workers** — bubbleperson URDF models patrolling waypoint loops through aisles
- **Odometry + PF fusion** — smooth local control from wheel encoders, long-term drift correction from PF when confident

---

## Table of Contents
1. [Installation](#installation)
2. [Quick Start](#quick-start)
3. [CLI Parameters](#cli-parameters)
4. [Scenarios](#scenarios)
5. [Maps](#maps)
6. [Architecture](#architecture)
7. [Outputs & Logs](#outputs--logs)
8. [Batch Testing](#batch-testing)
9. [PF Proof Script](#pf-proof-script)

---

## Installation

```bash
# 1. Clone / extract the project
cd PyBulletLabs

# 2. Create a virtual environment (recommended)
python -m venv .venv
source .venv/bin/activate        # Linux/macOS
# .venv\Scripts\activate         # Windows

# 3. Install dependencies
pip install numpy pybullet matplotlib
```

**Requirements:** Python 3.10+, NumPy, PyBullet, Matplotlib (for proof/analysis plots).

---

## Quick Start

```bash
cd PyBulletLabs

# Teleop mode (keyboard control, GUI)
python -m lab4.main

# Autonomous navigation to a random goal (GUI)
python -m lab4.main --map warehouse_small --nav --goal_random

# Headless mode — fast, no GUI
python -m lab4.main --map warehouse_small --nav --goal_random --direct --seed 42

# Multi-robot + humans (bubbleperson workers patrolling aisles)
python -m lab4.main --map warehouse_small --nav --goal_random --robots 3 --humans 2

# Multi-job (pick & drop missions)
python -m lab4.main --map warehouse_medium --nav --jobs 5 --robots 2 --humans 1

# SLAM mode (build map from scratch)
python -m lab4.main --map warehouse_small --nav --goal_random --slam --direct

# Generate PF proof plot (no GT in control loop)
python proof_pf_no_gt.py
```

---

## CLI Parameters

### World
| Parameter       | Default | Description                      |
|-----------------|---------|----------------------------------|
| `--map`         | (interactive) | Map file name, filename, or path |
| `--cell_size`   | 0.5     | Grid cell size (meters)          |
| `--hz`          | 50.0    | Physics simulation frequency     |
| `--friction`    | 0.05    | Ground friction coefficient      |

### Particle Filter
| Parameter       | Default | Description                      |
|-----------------|---------|----------------------------------|
| `--n`           | 500     | Number of particles              |
| `--scan_period` | 0.4     | Lidar scan period (seconds)      |
| `--angles_n`    | 36      | Number of lidar rays             |

### Navigation
| Parameter       | Default | Description                      |
|-----------------|---------|----------------------------------|
| `--nav`         | false   | Enable autonomous navigation     |
| `--goal_rc`     | —       | Goal as `row,col`                |
| `--goal_top`    | false   | Goal at top of map               |
| `--goal_random` | false   | Random goal (loops)              |
| `--goal_margin` | 1       | Margin from walls for goal       |
| `--replan_s`    | 2.0     | Replan interval (seconds)        |

### Emergency Recovery
| Parameter             | Default | Description                      |
|-----------------------|---------|----------------------------------|
| `--emergency_idle_s`  | 5.0     | Idle before emergency (seconds)  |
| `--emergency_forward_s`| 1.2    | Forward burst duration (seconds) |
| `--emergency_pf_runs` | 3       | PF re-init attempts              |

### Advanced Features
| Parameter   | Default | Description                          |
|-------------|---------|--------------------------------------|
| `--jobs`    | 0       | Number of pick/drop jobs (Phase 3)   |
| `--humans`  | 0       | Number of simulated humans (Phase 4) |
| `--robots`  | 1       | Number of robots (Phase 5)           |
| `--slam`    | false   | Enable SLAM mapping (Phase 6)        |

### Speeds
| Parameter       | Default | Description                      |
|-----------------|---------|----------------------------------|
| `--v_straight`  | 3.5     | Straight-line velocity (m/s)     |
| `--w_turn`      | 3.5     | Turning angular velocity (rad/s) |
| `--v_curve`     | 2.5     | Curve linear velocity (m/s)      |
| `--w_curve`     | 2.5     | Curve angular velocity (rad/s)   |
| `--curve_s`     | 15.0    | Curve segment duration (seconds) |

### Mode / Display
| Parameter    | Default | Description                          |
|--------------|---------|--------------------------------------|
| `--direct`   | false   | Headless mode (no GUI)               |
| `--seed`     | None    | Random seed for reproducibility      |
| `--autotest` | false   | Autotest mode                        |
| `--max_sim_s`| 60.0    | Maximum simulation time (seconds)    |
| `--speed`    | 8       | Physics speed multiplier             |

---

## Scenarios

### Company 1 — Replace Human-Driven Vehicles
```bash
python -m lab4.main --map warehouse_medium --nav --jobs 5 --robots 2 --direct --seed 1
```
Multiple AMRs autonomously execute pick-and-drop jobs without any humans present.

### Company 2 — Robots with Human Awareness
```bash
python -m lab4.main --map warehouse_medium --nav --jobs 3 --robots 2 --humans 3 --direct --seed 1
```
Robots navigate while detecting and avoiding simulated pedestrians (stop/slow/reroute).

### Scenario 3 — GT-Free Autonomy (Odometry + PF Fusion)
```bash
# GUI — watch the robot navigate using only odometry + PF (no ground-truth in control)
python -m lab4.main --map warehouse_small --nav --goal_random --humans 2 --seed 42

# Headless — fast run, check metrics at the end
python -m lab4.main --map warehouse_small --nav --goal_random --humans 2 --direct --seed 42
```
Robot plans and follows paths using wheel-encoder odometry corrected by PF when confident.
Ground-truth is only used for: (1) simulating lidar sensor readings, (2) logging/metrics.
PF debug overlay (red cross = PF estimate, green dots = particles) visible in GUI mode.

### Scenario 4 — PF Convergence on Asymmetric Map
```bash
python -m lab4.main --map maze_lab4 --nav --goal_random --direct --seed 42 --max_sim_s 40
```
The maze has unique corridor geometry — PF converges to ~5m mean error with Neff ~180.
Good test case for demonstrating proper MCL localization without perceptual aliasing.

### Scenario 5 — Kidnapped-Robot Recovery Demo
```bash
python -m lab4.main --map warehouse_small --nav --goal_random --direct --seed 99 --max_sim_s 60
```
Random particle injection (1% per scan) + dense lidar sweeps (120 rays every 5 scans)
continuously attempt re-localization. On symmetric maps, the PF may not fully converge
but the robot still navigates successfully via odometry + cluster-based PF fusion.

### Scenario 6 — Bubbleperson Workers Patrol
```bash
# GUI — watch articulated human models walking waypoint loops
python -m lab4.main --map warehouse_small --nav --goal_random --humans 4 --seed 42
```
Workers use the bubbleperson URDF (articulated humanoid) instead of placeholder capsules.
Each worker follows a BFS-generated loop of ~6 waypoints at 0.8 m/s steady speed.
Robots detect and avoid workers during navigation.

### Scenario 7 — Proof Plot Generation
```bash
python proof_pf_no_gt.py
```
Runs two headless simulations (maze_lab4 + warehouse_small) and generates `pf_proof.png`:
3×2 panel plot showing position error, heading error, and Neff vs time for each map.
Demonstrates that the control loop uses 0% ground-truth.

**Latest benchmark results** (800 particles, 72 rays, cluster estimate, position-only fusion):

| Map | Distance | Mean PF err | Final PF err | Final Odom err | Mean Neff |
|-----|----------|-------------|--------------|----------------|-----------|
| maze_lab4 | 5.0 m | 5.21 m | 8.70 m | 0.75 m | 180.2 |
| warehouse_small | 155.1 m | 9.24 m | 4.57 m | 4.91 m | 8.9 |

---

## Maps

Available warehouse maps in `shared/maps/`:

| Map | Size | Description |
|-----|------|-------------|
| `warehouse_small.txt`   | 30×50 | Small warehouse with racks and aisles |
| `warehouse_medium.txt`  | 50×80 | Medium warehouse with office, charging zone, chicanes |
| `warehouse_big.txt`     | 80×120 | Large warehouse with multiple zones |
| `my_warehouse.txt`      | —      | Custom/user-generated warehouse |
| `maze_lab4.txt`         | 17×25  | Asymmetric lab maze (good PF convergence) |
| `maze_realistic.txt`    | —      | Realistic maze |
| `maze_realistic_x4.txt` | —      | Large realistic maze (4x scale) |

Generate new maps:
```bash
python -m shared.utils.gen_warehouse_map --size medium --out shared/maps/my_warehouse.txt
```

---

## Architecture

```
lab4/
├── main.py             # CLI entry point, argument parsing
├── config.py           # SimConfig dataclass (all tunable parameters)
├── simulation.py       # Simulation engine (world setup, multi-robot loop, humans, logging)
├── agent.py            # RobotAgent (PF localization + Navigator + JobManager + SLAM)
├── robot.py            # HuskyRobot (PyBullet Husky URDF, differential-drive v/w control)
├── navigator.py        # A* path planning + pure-pursuit follower + collision avoidance
├── planner_astar.py    # A* on 2D grid (4/8-connected) with Minkowski obstacle inflation
├── control.py          # Keyboard teleop controller (WASD/ZQSD + arrows)
├── odometry.py         # Differential-drive odometry (wheel encoder integration)
├── job_manager.py      # Multi-job pick/drop queue with nearest-neighbor ordering
├── human.py            # Simulated human workers (bubbleperson URDF, waypoint patrol loops)
├── slam.py             # Occupancy grid SLAM (log-odds updates + Bresenham ray tracing)
├── world.py            # Grid ↔ world coordinate conversions
├── batch_runner.py     # Run N missions automatically, collect & summarize results
├── analyze_runs.py     # Load run logs, generate PF error & trajectory plots

shared/
├── maps/               # .txt grid maps (0=free, 1=wall)
├── data/               # Run logs (timestamped folders: pf.csv, odometry.csv, summary.json)
└── utils/
    ├── grid_map.py         # Grid loader + simulated ray-marching lidar
    ├── maze_builder.py     # PyBullet wall builder from grid (box collision shapes)
    ├── particle_filter.py  # Monte Carlo Localization (systematic resampling)
    ├── spawn.py            # Spawn point finder (top-of-map open area search)
    ├── logger.py           # CSV run logger (timestamped directories)
    ├── map_picker.py       # Interactive/direct map selector
    └── gen_warehouse_map.py# Warehouse map generator (small/medium/big presets)
```

### Control Pipeline (GT-Free)
```
Wheel Encoders → DiffDrive Odometry (local pose)
    → PF Predict (motion model + noise)
    → Physical Lidar Scan (ray-marching on grid from robot body)
    → Random Particle Injection (kidnapped-robot recovery, 1%)
    → PF Update (Gaussian likelihood) → Systematic Resample
    → PF Cluster Estimate (best-mode weighted mean, avoids symmetric aliasing)
    → Odometry + PF Fusion (position-only, gated on cluster weight fraction,
      adaptive α = min(0.30, cwf×0.5))
    → Control Pose (odom-corrected-by-PF)
    → A* Plan (inflated grid, fallback to raw grid)
    → Pure-Pursuit Follower + Reactive Wall Avoidance
    → (v, w) → Husky Differential Drive

GT pose is used ONLY for:
  - Simulating physical sensor readings (lidar ray-cast from robot body)
  - Logging / metrics / debug overlay
  - NOT for any control or planning decisions
```

### Multi-Robot Priority System
```
Robot 0 yields to Robot 1, 2, 3, ...
Robot 1 yields to Robot 2, 3, ...
All robots always yield to humans.
```

---

## Outputs & Logs

Each run creates a timestamped folder under `shared/data/run_YYYYMMDD_HHMMSS/`:

- **`pf_agent0.csv`** — Particle filter: `t, x_pf, y_pf, th_pf, x_gt, y_gt, th_gt, err_xy, err_th, neff, resample_count, x_odom, y_odom, th_odom, err_odom`
- **`odometry.csv`** — Odometry: `t, x_odo, y_odo, th_odo, x_gt, y_gt, th_gt, v_cmd, w_cmd`
- **`summary.json`** — Mission summary: success, time, distance, PF error, GT usage %

Console output at end of run:
```
=== RESULT ===
  success:       True
  sim_time:      120.02s
  distance:      437.98m
  mean_pf_error: 0.085m
  gt_usage:      0.0%
```

When SLAM is enabled, occupancy grid maps are saved as `slam_map_agent<id>.txt` in the project root.

---

## Batch Testing

Run multiple missions automatically and generate analysis:

```bash
# Run 10 missions on warehouse_small with 2 robots + 1 human
python -m lab4.batch_runner --map warehouse_small --nav --goal_random \
    --robots 2 --humans 1 --runs 10 --direct --seed 100

# Generate plots from collected data
python -m lab4.analyze_runs --data_dir shared/data
```

See `lab4/batch_runner.py` and `lab4/analyze_runs.py` for details.

---

## PF Proof Script

The `proof_pf_no_gt.py` script runs two headless simulations and generates a comparison plot:

```bash
python proof_pf_no_gt.py
# Output: pf_proof.png (3×2 panel: position error, heading error, Neff vs time)
```

**What it proves:**
- The control loop uses **0% ground-truth** for navigation decisions
- On `maze_lab4` (unique corridors): PF converges well — ~4.6m mean error, Neff ~92
- On `warehouse_small` (symmetric shelves): PF struggles with perceptual aliasing (~12m error) but robot still navigates 67m+ reaching multiple goals via odometry + reactive wall avoidance
- Kidnapped-robot recovery (random injection) continuously attempts re-localization

**Metrics table** printed to console:
```
  PF METRICS — maze_lab4  (no GT in control)
  Mean PF error (m)                  4.64
  Final PF error (m)                 4.72
  Mean odom error (m)                3.92
  Mean Neff                          92.5

  PF METRICS — warehouse_small  (no GT in control)
  Mean PF error (m)                 12.41
  Final odom error (m)               8.83
  Distance traveled                 67.1 m (5 goals reached)
  Mean Neff                           9.2
```

---

## License

Academic project — ESIEA Lab 4 (AMR Warehouse Automation).
