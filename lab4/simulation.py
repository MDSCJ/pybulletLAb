"""
simulation.py — Clean simulation engine for the AMR warehouse project.
Handles: world setup, multi-robot management, human obstacles, logging.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict

import numpy as np
import pybullet as p

from shared.utils.grid_map import load_grid, GridMapLidar, GridMapConfig
from shared.utils.logger import RunLogger
from shared.utils.maze_builder import MazeBuilder, MazePhysicsConfig
from shared.utils.spawn import find_first_open_area_top
from lab4.agent import RobotAgent
from lab4.human import SimulatedHuman
from lab4.config import SimConfig
from lab4.world import cell_center_to_world


class Simulation:
    """
    Main simulation engine for the AMR warehouse project.
    Manages multiple robots (RobotAgents) and humans.
    """

    def __init__(self, cfg: SimConfig):
        self.cfg = cfg
        self.rng = np.random.default_rng(cfg.seed)
        if cfg.seed is not None:
            np.random.seed(cfg.seed)

        # ── Connect PyBullet ──
        self.cid = p.connect(p.DIRECT if cfg.direct else p.GUI)
        if not cfg.direct:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
            p.resetDebugVisualizerCamera(
                cameraDistance=7.0, cameraYaw=45, cameraPitch=-55,
                cameraTargetPosition=(0, 0, 0), physicsClientId=self.cid)

        p.resetSimulation(physicsClientId=self.cid)
        p.setGravity(0, 0, -9.81, physicsClientId=self.cid)

        self.dt = 1.0 / float(cfg.hz)
        p.setTimeStep(self.dt, physicsClientId=self.cid)

        # ── Build world ──
        self.grid = load_grid(cfg.map_path)
        maze = MazeBuilder(self.cid, MazePhysicsConfig(cell_size=cfg.cell_size, friction=cfg.friction))
        maze.add_ground()
        maze.build(self.grid)
        self.lidar = GridMapLidar(self.grid, GridMapConfig(cell_size=cfg.cell_size))
        self.rows = len(self.grid)
        self.cols = len(self.grid[0])

        # ── Logger (created early so agents can use it) ──
        self.logger = RunLogger.create("shared/data")
        print(f"MAP: {cfg.map_path}")
        print(f"Logs in: {self.logger.run_dir}")

        # ── Agents (Phase 5) ──
        self.agents: List[RobotAgent] = []
        n_robots = max(1, cfg.n_robots)
        print(f"[SIM] Spawning {n_robots} robots...")
        
        for i in range(n_robots):
            # Find spawn point
            # Use different seed or offset for each robot? 
            # find_first_open_area_top returns deterministic result if unmodified grid?
            # We can use 'free_block' logic with margin.
            # Or use 'find_open_area' with random search if 'top' is occupied?
            # Basic implementation: try to spawn effectively.
            # If multiple robots, random spawn is better to avoid collision at start.
            
            spawn_r, spawn_c = 0, 0
            if i == 0:
                 # First robot at top (standard)
                try:
                     spawn_r, spawn_c = find_first_open_area_top(self.grid, free_block=6, margin=1)
                except RuntimeError:
                     try:
                         spawn_r, spawn_c = find_first_open_area_top(self.grid, free_block=4, margin=1)
                     except RuntimeError:
                         # Fallback for tight mazes
                         try:
                             spawn_r, spawn_c = find_first_open_area_top(self.grid, free_block=2, margin=0)
                         except RuntimeError:
                             # Last resort: find ANY free cell
                             print("[SIM] Warning: random spawn fallback!")
                             while True:
                                 spawn_r = self.rng.integers(1, self.rows-2)
                                 spawn_c = self.rng.integers(1, self.cols-2)
                                 if self.grid[spawn_r][spawn_c] == 0:
                                     break
            else:
                 # Subsequent robots random specific logic
                 spawn_r = self.rng.integers(1, self.rows-2)
                 spawn_c = self.rng.integers(1, self.cols-2)
                 while self.grid[spawn_r][spawn_c] != 0:
                      spawn_r = self.rng.integers(1, self.rows-2)
                      spawn_c = self.rng.integers(1, self.cols-2)

            sx, sy = cell_center_to_world(spawn_r, spawn_c, self.rows, self.cols, cfg.cell_size)
            
            # Create agent
            agent = RobotAgent(i, self.cid, (sx, sy, 0.0), self.grid, self.lidar, cfg,
                               logger=self.logger)
            self.agents.append(agent)

        # ── Humans (Phase 4) ──
        self.humans: List[SimulatedHuman] = []
        if cfg.n_humans > 0:
            print(f"[SIM] Spawning {cfg.n_humans} humans...")
            for i in range(cfg.n_humans):
                # Retry logic for spawning humans away from robots is tricky with multiple robots.
                # Simplified: just spawn random free.
                hr = self.rng.integers(1, self.rows-2)
                hc = self.rng.integers(1, self.cols-2)
                if self.grid[hr][hc] == 0:
                     hx, hy = cell_center_to_world(hr, hc, self.rows, self.cols, cfg.cell_size)
                     self.humans.append(SimulatedHuman(
                             self.cid, hx, hy, self.grid, cfg.cell_size, seed=i))

        # ── Draw pick/drop markers in GUI ──
        if not cfg.direct:
            self._draw_job_markers()

        # ── Results ──
        self.result = {
            "success": False,
            "sim_time": 0.0,
            "distance": 0.0,
            "mean_pf_error": 0.0,
            "gt_usage_pct": 0.0,
        }

    def _draw_job_markers(self):
        """Draw pick (green) and drop (red) markers in the PyBullet GUI."""
        for agent in self.agents:
            if agent.job_manager is None:
                continue
            # Collect all jobs (current + queued)
            all_jobs = []
            if agent.job_manager.current_job is not None:
                all_jobs.append(agent.job_manager.current_job)
            all_jobs.extend(agent.job_manager.jobs)

            for job in all_jobs:
                pr, pc = job.pick_cell
                dr, dc = job.drop_cell
                px, py = cell_center_to_world(pr, pc, self.rows, self.cols, self.cfg.cell_size)
                dx, dy = cell_center_to_world(dr, dc, self.rows, self.cols, self.cfg.cell_size)

                # Pick marker — GREEN sphere + pillar
                pick_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.15,
                                              rgbaColor=[0, 1, 0, 0.85],
                                              physicsClientId=self.cid)
                p.createMultiBody(baseMass=0, baseVisualShapeIndex=pick_vis,
                                  basePosition=[px, py, 0.4],
                                  physicsClientId=self.cid)
                # Pillar line from ground to sphere
                p.addUserDebugLine([px, py, 0.0], [px, py, 0.4],
                                   lineColorRGB=[0, 0.8, 0], lineWidth=2,
                                   lifeTime=0, physicsClientId=self.cid)
                p.addUserDebugText(f"P{job.id}", [px, py, 0.65],
                                   textColorRGB=[0, 0.6, 0], textSize=1.2,
                                   lifeTime=0, physicsClientId=self.cid)

                # Drop marker — RED sphere + pillar
                drop_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.15,
                                              rgbaColor=[1, 0, 0, 0.85],
                                              physicsClientId=self.cid)
                p.createMultiBody(baseMass=0, baseVisualShapeIndex=drop_vis,
                                  basePosition=[dx, dy, 0.4],
                                  physicsClientId=self.cid)
                p.addUserDebugLine([dx, dy, 0.0], [dx, dy, 0.4],
                                   lineColorRGB=[0.8, 0, 0], lineWidth=2,
                                   lifeTime=0, physicsClientId=self.cid)
                p.addUserDebugText(f"D{job.id}", [dx, dy, 0.65],
                                   textColorRGB=[0.6, 0, 0], textSize=1.2,
                                   lifeTime=0, physicsClientId=self.cid)

        print("[SIM] Pick (green) and Drop (red) markers drawn.")

    def shutdown(self):
        p.disconnect(self.cid)

    def run(self) -> dict:
        """Run the simulation. Returns a result dict."""
        cfg = self.cfg
        sim_t = 0.0
        
        # Tracking metrics
        total_pf_errors = []
        last_poses = {}
        total_distance = 0.0
        
        for agent in self.agents:
            ax, ay, _ = agent.get_pose()
            last_poses[agent.id] = (ax, ay)
        
        while True:
            # ── Time-out check ──
            if cfg.autotest and sim_t >= cfg.max_sim_s:
                break
            if cfg.nav_enabled and sim_t >= cfg.max_sim_s:
                print(f"[NAV] timeout at {sim_t:.1f}s")
                break

            # ── Update Humans ──
            human_positions = []
            for h in self.humans:
                h.update(self.dt)
                hx, hy, _, _ = h.get_pose()
                human_positions.append((hx, hy))

            # ── Update Agents ──
            # Basic traffic coordination: Robot sees OTHER robots + humans as obstacles
            
            # First collect all robot poses
            agent_poses = []
            for agent in self.agents:
                ax, ay, _ = agent.get_pose()
                agent_poses.append((ax, ay))
            
            # Now update each agent
            all_done = True
            for i, agent in enumerate(self.agents):
                if not agent.active:
                    continue
                all_done = False
                
                # Construct obstacles list for this agent
                # Humans + other robots with their types
                obstacles = list(human_positions)
                obstacle_types = ['human'] * len(human_positions)
                
                for j, (ax, ay) in enumerate(agent_poses):
                    if i != j:
                        obstacles.append((ax, ay))
                        obstacle_types.append(f'robot_{j}')
                
                # Update agent with priority-based obstacle avoidance
                metrics = agent.update(self.dt, sim_t, obstacles, obstacle_types)
                
                if "pf_err" in metrics:
                    total_pf_errors.append(metrics["pf_err"])
                
                # Update distance
                ax_new, ay_new, _ = agent.get_pose()
                lx, ly = last_poses[agent.id]
                dist = math.hypot(ax_new - lx, ay_new - ly)
                total_distance += dist
                last_poses[agent.id] = (ax_new, ay_new)

            # ── Step Physics (speed multiplier) ──
            for _ in range(cfg.speed_multiplier):
                p.stepSimulation(physicsClientId=self.cid)
            if not cfg.direct:
                time.sleep(self.dt / max(1, cfg.speed_multiplier))
            
            sim_t += self.dt
            
            # Check completion
            if all_done:
                print(f"[SIM] All agents finished at {sim_t:.2f}s")
                self.result["success"] = True
                break

        self.result["sim_time"] = sim_t
        self.result["distance"] = total_distance
        if total_pf_errors:
            self.result["mean_pf_error"] = float(np.mean(total_pf_errors))
            
        # ── Save SLAM Maps ──
        for agent in self.agents:
            if hasattr(agent, "mapper") and agent.mapper:
                outfile = f"slam_map_agent{agent.id}.txt"
                print(f"[SIM] Saving SLAM map to {outfile}...")
                agent.mapper.save(outfile)

        return self.result
