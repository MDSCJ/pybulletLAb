"""
agent.py — RobotAgent class for multi-robot simulation.
Encapsulates: Robot hardware interface, PF localization, Navigator, JobManager.
"""
from __future__ import annotations

import math
import numpy as np
from typing import Optional, Tuple, List, Dict

from lab4.config import SimConfig
from lab4.robot import HuskyRobot
from lab4.navigator import Navigator, NavConfig
from lab4.odometry import DiffDriveOdometry, OdomConfig
from lab4.job_manager import JobManager
from lab4.slam import OccupancyGridMapper
from lab4.world import cell_center_to_world, world_to_cell
from shared.utils.particle_filter import ParticleFilter, PFConfig
from shared.utils.grid_map import GridMapLidar
from shared.utils.logger import RunLogger
from shared.utils.spawn import find_first_open_area_top


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _resample_near_spawn(pf: ParticleFilter, lidar: GridMapLidar,
                         sx: float, sy: float,
                         std_xy: float, std_th: float,
                         max_tries: int = 50) -> None:
    """Resample particles that landed inside walls near the spawn point."""
    bad = np.array([not lidar.is_free(float(px), float(py))
                    for px, py in pf.p[:, :2]])
    tries = 0
    while bad.any() and tries < max_tries:
        n_bad = int(bad.sum())
        pf.p[bad, 0] = np.random.normal(sx, std_xy, size=n_bad)
        pf.p[bad, 1] = np.random.normal(sy, std_xy, size=n_bad)
        pf.p[bad, 2] = np.random.normal(0.0, std_th, size=n_bad)
        pf.p[:, 2] = _wrap(pf.p[:, 2])
        bad = np.array([not lidar.is_free(float(px), float(py))
                        for px, py in pf.p[:, :2]])
        tries += 1
    if bad.any():
        print(f"[WARN] {int(bad.sum())} particles still in walls after resample.")


def _fill_particle_scans(pf: ParticleFilter, lidar: GridMapLidar,
                         angles: np.ndarray, out: np.ndarray) -> None:
    """Compute simulated lidar scans for all particles."""
    if hasattr(lidar, "scan_particles"):
        lidar.scan_particles(pf.p, angles, out=out)
    else:
        for i in range(pf.cfg.n):
            out[i, :] = lidar.scan(
                float(pf.p[i, 0]), float(pf.p[i, 1]),
                float(pf.p[i, 2]), angles)


class RobotAgent:
    def __init__(self, agent_id: int, cid: int, start_pose: Tuple[float, float, float],
                 grid: List[List[float]], lidar: GridMapLidar, cfg: SimConfig,
                 logger: Optional['RunLogger'] = None):
        self.id = agent_id
        self.cid = cid
        self.cfg = cfg
        self.grid = grid
        self.lidar = lidar
        self.logger = logger
        self.rows = len(grid)
        self.cols = len(grid[0])
        
        sx, sy, sth = start_pose
        self.start_pose = start_pose

        # ── Robot Hardware ──
        # Auto-scale: Husky at scale=1.0 is ~0.695m wide.
        # Target: robot width ≈ 60% of cell_size for clearance in corridors.
        robot_scale = max(0.2, min(0.6, (cfg.cell_size * 0.60) / 0.695))
        self.robot = HuskyRobot(cid, start_pos=(sx, sy, 0.1), start_yaw=sth, scale=robot_scale)
        
        # ── Particle Filter ──
        pf_cfg = PFConfig(
            n=cfg.n_particles,
            trans_noise_per_m=0.04,
            rot_noise_per_rad=0.03,
            trans_noise_min=0.001,
            rot_noise_min=0.001,
            meas_std=0.40,             # balanced: discriminative but not particle-starved
        )
        self.pf = ParticleFilter(pf_cfg)
        self.pf.init_gaussian(sx, sy, sth, std_xy=cfg.pf_init_std_xy, std_th=cfg.pf_init_std_th)
        _resample_near_spawn(self.pf, lidar, sx, sy, std_xy=cfg.pf_init_std_xy, std_th=cfg.pf_init_std_th)
        
        # Precompute ray angles (normal + dense for kidnapped-robot recovery)
        K = max(4, cfg.n_lidar_rays)
        self.angles = np.linspace(-math.pi, math.pi, K, endpoint=False).astype(np.float32)
        self.z_hat_buf = np.empty((cfg.n_particles, K), dtype=np.float32)

        K_dense = max(K, cfg.pf_dense_scan_rays)
        self.angles_dense = np.linspace(-math.pi, math.pi, K_dense, endpoint=False).astype(np.float32)
        self.z_hat_buf_dense = np.empty((cfg.n_particles, K_dense), dtype=np.float32)
        self._scan_count = 0  # counter for dense-scan scheduling

        # Precompute free-space cell centres for kidnapped-robot injection
        free_rc = [(r, c) for r in range(self.rows) for c in range(self.cols)
                   if grid[r][c] == 0]
        self._free_cells_xy = np.array(
            [cell_center_to_world(r, c, self.rows, self.cols, cfg.cell_size)
             for r, c in free_rc], dtype=np.float32) if free_rc else np.empty((0, 2), dtype=np.float32)

        # ── Navigator ──
        # Scale safety radii to robot size
        nav_safety = max(0.12, robot_scale * 0.55)
        nav_slowdown = max(0.25, robot_scale * 1.1)
        self.nav = Navigator(grid, NavConfig(
            cell_size=cfg.cell_size,
            safety_radius=nav_safety,
            slowdown_radius=nav_slowdown,
            # Collision avoidance: VERY CONSERVATIVE settings
            robot_min_distance=1.0,
            human_min_distance=1.0,
            obstacle_stop_distance=2.0,           # Stop if obstacle < 2.0m (2m buffer)
            obstacle_slowdown_distance=3.0,       # Start slowing at 3.0m (3m buffer)
            obstacle_front_angle=1.57,            # ~90° forward cone for detection
        ), robot_id=agent_id, total_robots=cfg.n_robots)

        # ── Job Manager ──
        self.job_manager: Optional[JobManager] = None
        if cfg.n_jobs > 0:
            self.job_manager = JobManager(grid)
            self.job_manager.generate_random_jobs(cfg.n_jobs, seed=(cfg.seed + agent_id) if cfg.seed else None)
        
        # ── SLAM (Phase 6) ──
        self.mapper: Optional[OccupancyGridMapper] = None
        if cfg.slam_enabled:
            self.mapper = OccupancyGridMapper(self.rows, self.cols, cfg.cell_size)

        # ── Odometry (velocity-based, avoids wheel slippage) ──
        self.odom = DiffDriveOdometry(OdomConfig(
            wheel_radius=self.robot.wheel_radius,
            wheel_base=self.robot.wheel_base,
        ))
        self.odom.reset(sx, sy, sth)

        # ── State ──
        self.last_plan_t = -1e9
        self.stall_count = 0
        self.stall_ref_t = 0.0
        # Stall tracking uses PF estimate (initialised from start pose)
        self.stall_ref_x = sx
        self.stall_ref_y = sy
        self.stall_ref_th = sth
        self.backup_until = -1.0
        self._backup_w = -1.0  # turn direction during backup
        self.emergency_forward_until = -1.0
        self.was_degraded = False
        self.gt_usage_time = 0.0
        self.total_resample_count = 0
        self.active = True  # If finished all jobs
        self._debug_frame = 0  # throttle GUI drawing

        # Warmup PF
        self._pf_warmup()

        # Set initial goal
        self._setup_initial_goal()

    def _pf_warmup(self):
        sx, sy, sth = self.start_pose
        for _ in range(5):
            z = self.lidar.scan(sx, sy, sth, self.angles)
            _fill_particle_scans(self.pf, self.lidar, self.angles, self.z_hat_buf)
            self.pf.update(z, self.z_hat_buf)
            self.pf.resample()

    def draw_debug_info(self, pf_err: float):
        """Draw GT, PF, and Error in PyBullet GUI."""
        if self.cid < 0: return # Headless/Direct mode check? No, cid is always valid but GUI might not be.
        import pybullet as p
        
        x, y, z = self.robot.get_pose()
        # Text above robot
        text = f"ID:{self.id}\nGT:({x:.1f},{y:.1f})\nErr:{pf_err:.2f}m"
        p.addUserDebugText(text, [x, y, 1.5], textColorRGB=[0, 0, 0], textSize=1.0, lifeTime=0.1, physicsClientId=self.cid)
        
        # Draw PF mean?
        # Draw PF mean as a Red Cross
        px, py, _ = self.pf.estimate()
        # p.addUserDebugText("PF", [px, py, 0.5], textColorRGB=[1, 0, 0], textSize=0.8, lifeTime=0.1, physicsClientId=self.cid)
        s = 0.3 # Size of cross
        p.addUserDebugLine([px-s, py, 0.05], [px+s, py, 0.05], [1, 0, 0], lineWidth=3, lifeTime=0.1, physicsClientId=self.cid)
        p.addUserDebugLine([px, py-s, 0.05], [px, py+s, 0.05], [1, 0, 0], lineWidth=3, lifeTime=0.1, physicsClientId=self.cid)
        
        # Also draw some particles (every 20th one)
        step = max(1, self.pf.p.shape[0] // 20)
        for i in range(0, self.pf.p.shape[0], step):
             ppx, ppy = self.pf.p[i, 0], self.pf.p[i, 1]
             p.addUserDebugLine([ppx, ppy, 0.05], [ppx, ppy, 0.1], [0, 1, 0], lineWidth=2, lifeTime=0.1, physicsClientId=self.cid)


    def _setup_initial_goal(self):
        if self.job_manager:
            tgt = self.job_manager.next_target()
            if tgt:
                (r, c), mode = tgt
                print(f"[Agent {self.id}] Starting: {mode} at ({r},{c})")
                self.nav.set_goal_cell(r, c)
                sx, sy, _ = self.start_pose
                ok = self.nav.plan_from_pose(sx, sy)
                if ok:
                    print(f"[Agent {self.id}] Path found ({len(self.nav.path_world)} waypoints)")
                else:
                    print(f"[Agent {self.id}] WARNING: No path to initial goal!")
            else:
                self.active = False
                return

        elif self.cfg.goal_mode == "random":
             self._pick_random_goal()

    def _pick_random_goal(self):
        """Pick a random free cell as goal and plan path to it."""
        # Use odometry for current position; if odom cell is invalid (drifted
        # into a wall cell), snap to the nearest free cell.
        ox, oy = self.odom.x, self.odom.y
        cr, cc = world_to_cell(ox, oy, self.rows, self.cols, self.cfg.cell_size)
        if cr < 0 or cr >= self.rows or cc < 0 or cc >= self.cols or self.grid[cr][cc] != 0:
            # Odom position is in a wall / out of bounds — find nearest free cell
            best_d = float('inf')
            for r in range(self.rows):
                for c in range(self.cols):
                    if self.grid[r][c] == 0:
                        fx, fy = cell_center_to_world(r, c, self.rows, self.cols, self.cfg.cell_size)
                        d = math.hypot(fx - ox, fy - oy)
                        if d < best_d:
                            best_d = d
                            ox, oy = fx, fy
        for _ in range(100):
            r = np.random.randint(1, self.rows - 1)
            c = np.random.randint(1, self.cols - 1)
            if self.grid[r][c] == 0:
                self.nav.set_goal_cell(r, c)
                ok = self.nav.plan_from_pose(ox, oy)
                if ok:
                    print(f"[Agent {self.id}] New random goal: ({r},{c}) — path {len(self.nav.path_world)} waypoints")
                    return
        print(f"[Agent {self.id}] Failed to pick reachable random goal.")


    def update(self, dt: float, sim_t: float, dynamic_obstacles: List[Tuple[float, float]], obstacle_types: Optional[List[str]] = None) -> Dict:
        """
        Run one control cycle.
        Returns metrics dict (e.g. pf_error, etc).
        """
        if not self.active:
            self.robot.set_cmd_vel(0, 0)
            return {}

        x_gt, y_gt, th_gt = self.robot.get_pose()  # GT kept for logging/metrics only
        pf_err = 0.0

        # ── Update odometry from wheel encoders (no GT) ──
        left_ang, right_ang = self.robot.get_left_right_wheel_angles()
        x_odom, y_odom, th_odom, dC_odom, dT_odom = self.odom.update_from_wheels(left_ang, right_ang)

        # ── Stall detection (uses odometry estimate, not GT) ──
        if (sim_t - self.stall_ref_t) >= 2.0:
            dist_moved = math.hypot(x_odom - self.stall_ref_x, y_odom - self.stall_ref_y)
            rot_moved = abs(_wrap(th_odom - self.stall_ref_th))
            
            if dist_moved < 0.10 and rot_moved < 0.15:
                self.stall_count += 1
                if self.stall_count >= 1:
                    # Immediately backup + random turn on first stall
                    turn_dir = 1.0 if np.random.random() > 0.5 else -1.0
                    print(f"[Agent {self.id}] stall #{self.stall_count} -> backup+turn")
                    self.backup_until = sim_t + 1.5
                    self._backup_w = turn_dir * 1.5
                    self.stall_count = 0
                self.last_plan_t = -1e9  # force replan after recovery
            else:
                self.stall_count = 0
            self.stall_ref_t = sim_t
            self.stall_ref_x = x_odom
            self.stall_ref_y = y_odom
            self.stall_ref_th = th_odom

        # ── PF predict: always advance particles with wheel-encoder deltas,
        #    even during backup/emergency.  Skipping this caused PF to fall
        #    behind odometry, making fusion impossible afterwards. ──
        self.pf.predict(dC_odom, dT_odom)

        # ── Control Logic ──
        v, w = 0.0, 0.0
        did_resample = False
        neff_val = self.pf.neff()
        
        if sim_t < self.backup_until:
             v, w = -0.4, getattr(self, '_backup_w', -1.0)
        elif sim_t < self.emergency_forward_until:
             v, w = 0.9, 0.0
        else:
            if (sim_t % self.cfg.scan_period) < dt:
                self._scan_count += 1

                # Decide whether this is a dense scan (more rays → better re-localisation)
                use_dense = (self._scan_count % self.cfg.pf_dense_scan_interval == 0)
                if use_dense:
                    angles_cur = self.angles_dense
                    z_buf_cur  = self.z_hat_buf_dense
                else:
                    angles_cur = self.angles
                    z_buf_cur  = self.z_hat_buf

                # Measurement scan: comes from the robot's physical sensor
                # (= GT in simulation — this is the sensor reading, NOT a GT dependency
                #  for control.  A real robot's lidar fires from its true body frame.)
                z = self.lidar.scan(x_gt, y_gt, th_gt, angles_cur)

                # Kidnapped-robot recovery: inject random free-space particles
                # BEFORE update so they get properly scored by the measurement model.
                # Bad guesses will receive low weights and be eliminated in resample.
                self.pf.inject_random_particles(
                    self._free_cells_xy, frac=self.cfg.pf_random_inject_pct)

                _fill_particle_scans(self.pf, self.lidar, angles_cur, z_buf_cur)
                self.pf.update(z, z_buf_cur)

                # Record Neff BEFORE resample (meaningful weight diversity metric)
                neff_val = self.pf.neff()

                # Compute PF estimate BEFORE resample — weights are still
                # informative (reflect measurement likelihood).  Use cluster-
                # based estimate to avoid averaging between symmetric modes.
                x_pf, y_pf, th_pf, cluster_wf = self.pf.estimate_best_cluster(radius=1.5)

                self.pf.resample()

                did_resample = True
                self.total_resample_count += 1
                self.last_scan = z if not use_dense else self.lidar.scan(x_gt, y_gt, th_gt, self.angles)
            else:
                self.last_scan = None

            if not did_resample:
                x_pf, y_pf, th_pf, cluster_wf = self.pf.estimate_best_cluster(radius=1.5)
            pf_err = math.hypot(x_pf - x_gt, y_pf - y_gt)

            # Log PF data (GT logged for metrics/debug only)
            if self.logger and did_resample:
                th_err = abs(_wrap(th_pf - th_gt))
                odom_err = math.hypot(x_odom - x_gt, y_odom - y_gt)
                self.logger.write_row(
                    f"pf_agent{self.id}.csv",
                    ["t", "x_pf", "y_pf", "th_pf", "x_gt", "y_gt", "th_gt",
                     "err_xy", "err_th", "neff", "resample_count",
                     "x_odom", "y_odom", "th_odom", "err_odom"],
                    [f"{sim_t:.4f}", x_pf, y_pf, th_pf, x_gt, y_gt, th_gt,
                     pf_err, th_err, neff_val, self.total_resample_count,
                     x_odom, y_odom, th_odom, odom_err],
                )

            # ── Control pose: odometry corrected by PF when confident ──
            # Gates to prevent bad fusion on symmetric maps:
            #  1. cluster_wf > 5%   — PF cluster has dominant weight
            #  2. proximity < 3m    — PF agrees with odom (prevents wrong-mode pull)
            #  3. adaptive alpha    — stronger correction when more confident
            if did_resample and cluster_wf > 0.05:
                dx = x_pf - x_odom
                dy = y_pf - y_odom
                dist_pf_odom = math.hypot(dx, dy)
                if dist_pf_odom < 5.0:  # proximity gate: only fuse when PF & odom agree
                    alpha = min(0.30, cluster_wf * 0.5)
                    self.odom.x += alpha * dx
                    self.odom.y += alpha * dy
                    x_odom, y_odom, th_odom = self.odom.x, self.odom.y, self.odom.theta

            x_ctrl, y_ctrl, th_ctrl = x_odom, y_odom, th_odom

            # SLAM
            if self.mapper and self.last_scan is not None:
                self.mapper.update(x_ctrl, y_ctrl, th_ctrl, self.last_scan, self.angles)

            self.nav.set_dynamic_obstacles(dynamic_obstacles, obstacle_types)

            # ── Step 2: Check goal distance (PF estimate) ──
            goal_dist = float('inf')
            if self.nav.goal_cell:
                gr, gc = self.nav.goal_cell
                gx, gy = cell_center_to_world(gr, gc, self.rows, self.cols, self.cfg.cell_size)
                goal_dist = math.hypot(gx - x_ctrl, gy - y_ctrl)

            if goal_dist <= self.nav.cfg.goal_tol_m:
                if self.job_manager:
                    self.job_manager.complete_target()
                    next_tgt = self.job_manager.next_target()
                    if next_tgt:
                        (r, c), mode = next_tgt
                        print(f"[Agent {self.id}] Next: {mode} at ({r},{c})")
                        self.nav.set_goal_cell(r, c)
                        ok = self.nav.plan_from_pose(x_ctrl, y_ctrl)
                        if ok:
                            print(f"[Agent {self.id}] Path planned: {len(self.nav.path_world)} waypoints")
                        self.last_plan_t = sim_t
                        self.stall_count = 0
                        self.stall_ref_t = sim_t
                        self.stall_ref_x, self.stall_ref_y, self.stall_ref_th = x_ctrl, y_ctrl, th_ctrl
                    else:
                        print(f"[Agent {self.id}] ALL JOBS DONE.")
                        self.active = False
                        self.robot.set_cmd_vel(0, 0)
                        return {"pf_err": pf_err}
                else:
                    if self.cfg.goal_mode == "random":
                        self._pick_random_goal()
                        self.last_plan_t = sim_t
                    else:
                        self.active = False
                        return {"pf_err": pf_err}

            # ── Step 3: Ensure a valid A* path exists BEFORE any motion ──
            needs_plan = (sim_t - self.last_plan_t) >= self.cfg.replan_interval
            has_no_path = len(self.nav.path_world) == 0
            if (needs_plan or has_no_path) and goal_dist > self.nav.cfg.goal_tol_m:
                ok = self.nav.plan_from_pose(x_ctrl, y_ctrl)
                self.last_plan_t = sim_t
                if ok:
                    print(f"[Agent {self.id}] Path planned: {len(self.nav.path_world)} waypoints")

            # ── Step 4: NO PATH = NO MOTION ──
            if len(self.nav.path_world) == 0:
                # No valid path — stop completely, do not move blindly
                self.robot.set_cmd_vel(0, 0)
                return {"pf_err": pf_err}

            # ── Step 5: Follow the planned path ──
            # Lidar scan for reactive wall safety (physical sensor on robot body = GT in sim)
            nav_scan_vals = self.lidar.scan(x_gt, y_gt, th_gt, self.angles)
            K = len(self.angles)

            def _sector_min(center_idx, half_w):
                s = max(0, center_idx - half_w)
                e = min(K, center_idx + half_w + 1)
                chunk = nav_scan_vals[s:e]
                return float(np.min(chunk)) if len(chunk) > 0 else 10.0

            hw = max(2, K // 8)
            # Front sector: ±K//6 rays ≈ ±30° so walls approached at an
            # angle are detected early (was ±K//16 ≈ ±10° — far too narrow).
            front_hw = max(2, K // 6)
            scan_dict = {
                'front': _sector_min(K // 2, front_hw),
                'left':  _sector_min(int(3 * K / 4), hw),
                'right': _sector_min(K // 4, hw),
            }
            v, w, done = self.nav.compute_cmd(x_ctrl, y_ctrl, th_ctrl, scan=scan_dict)
        
        self.robot.set_cmd_vel(v, w)
        
        # GUI Debug (throttle)
        if not self.cfg.direct:
             self._debug_frame += 1
             if self._debug_frame % 10 == 0:
                 self.draw_debug_info(pf_err)
             
        return {"pf_err": pf_err}

    def get_pose(self):
        return self.robot.get_pose()
