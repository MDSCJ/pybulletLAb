"""
human.py — Simulated human agent (bubbleperson URDF) with waypoint-loop patrol.
"""
from __future__ import annotations

import math
import os
import random
from collections import deque

import numpy as np
import pybullet as p
from typing import List, Tuple, Optional

from lab4.world import cell_center_to_world


# ── Path to the bubbleperson URDF (relative to project root) ──
_URDF_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          "models", "bubbleperson.urdf")


class SimulatedHuman:
    """
    Worker entity using the bubbleperson articulated URDF.
    Follows a closed waypoint loop at a steady speed.
    """

    # Class-level default patrol speed (m/s)
    PATROL_SPEED = 0.8
    # Number of waypoints to generate for the patrol loop
    N_WAYPOINTS = 6
    # How close (m) to a waypoint before advancing to next
    WP_TOLERANCE = 0.35

    def __init__(self, cid: int, start_x: float, start_y: float,
                 grid: List[List[float]], cell_size: float, seed: int = None):
        self.cid = cid
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.cell_size = cell_size
        self.rng = random.Random(seed) if seed is not None else random.Random()

        # ── Load bubbleperson URDF ──
        # URDF body parts are already 2× larger; scale to fit corridors
        global_scale = max(0.25, cell_size * 0.50)
        self.body_id = p.loadURDF(
            _URDF_PATH,
            basePosition=[start_x, start_y, 0.55 * global_scale],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            globalScaling=global_scale,
            useFixedBase=True,           # kinematic — no physics response
            physicsClientId=cid,
        )
        self._scale = global_scale

        # ── Build a closed waypoint loop ──
        self.speed = self.PATROL_SPEED
        self.waypoints: List[Tuple[float, float]] = []
        self._build_patrol_loop(start_x, start_y)
        self._wp_idx = 0  # current target waypoint index

    # ─────────────────────────── patrol loop builder ───────────────────────────
    def _build_patrol_loop(self, sx: float, sy: float):
        """
        Generate a closed loop of free-space waypoints near the spawn point.
        Strategy: BFS outward from spawn cell to collect reachable free cells,
        then pick N roughly-evenly-spaced cells to form a loop.
        """
        sr, sc = self._world_to_cell(sx, sy)
        # BFS to collect reachable free cells (up to 200)
        visited = set()
        queue = deque()
        queue.append((sr, sc))
        visited.add((sr, sc))
        free_cells: List[Tuple[int, int]] = []

        while queue and len(free_cells) < 200:
            r, c = queue.popleft()
            free_cells.append((r, c))
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols and (nr, nc) not in visited:
                    if self.grid[nr][nc] == 0:
                        visited.add((nr, nc))
                        queue.append((nr, nc))

        if len(free_cells) < 3:
            # Tiny space — just stay put
            wx, wy = cell_center_to_world(sr, sc, self.rows, self.cols, self.cell_size)
            self.waypoints = [(wx, wy)]
            return

        # Pick N_WAYPOINTS spread across the reachable set
        n_wp = min(self.N_WAYPOINTS, len(free_cells))
        step = max(1, len(free_cells) // n_wp)
        chosen = [free_cells[i * step] for i in range(n_wp)]

        # Convert to world coordinates
        self.waypoints = [
            cell_center_to_world(r, c, self.rows, self.cols, self.cell_size)
            for r, c in chosen
        ]

    def _world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        world_w = self.cols * self.cell_size
        world_h = self.rows * self.cell_size
        col = int((x + world_w / 2.0) / self.cell_size)
        row = int((y + world_h / 2.0) / self.cell_size)
        row = max(0, min(self.rows - 1, row))
        col = max(0, min(self.cols - 1, col))
        return row, col

    # ─────────────────────────── update each tick ─────────────────────────────
    def update(self, dt: float):
        """Move along the waypoint loop at steady speed (kinematic)."""
        if not self.waypoints:
            return

        pos, _ = p.getBasePositionAndOrientation(self.body_id, physicsClientId=self.cid)
        x, y = pos[0], pos[1]

        tx, ty = self.waypoints[self._wp_idx]
        dist = math.hypot(tx - x, ty - y)

        # Advance to next waypoint when close enough (loop around)
        if dist < self.WP_TOLERANCE:
            self._wp_idx = (self._wp_idx + 1) % len(self.waypoints)
            tx, ty = self.waypoints[self._wp_idx]
            dist = math.hypot(tx - x, ty - y)
            if dist < 1e-6:
                return

        # Heading toward current waypoint
        heading = math.atan2(ty - y, tx - x)
        step = min(self.speed * dt, dist)  # don't overshoot
        nx = x + step * math.cos(heading)
        ny = y + step * math.sin(heading)

        # ── Wall check: only move if the target cell is free ──
        nr, nc = self._world_to_cell(nx, ny)
        if self.grid[nr][nc] != 0:
            # Blocked — skip to next waypoint instead of walking through
            self._wp_idx = (self._wp_idx + 1) % len(self.waypoints)
            return

        # Also check intermediate point to avoid cutting corners through walls
        mx = (x + nx) * 0.5
        my = (y + ny) * 0.5
        mr, mc = self._world_to_cell(mx, my)
        if self.grid[mr][mc] != 0:
            self._wp_idx = (self._wp_idx + 1) % len(self.waypoints)
            return

        # Kinematic move — directly set position
        z_stand = 0.55 * self._scale
        orn = p.getQuaternionFromEuler([0, 0, heading])
        p.resetBasePositionAndOrientation(self.body_id, [nx, ny, z_stand], orn,
                                          physicsClientId=self.cid)

    def get_pose(self) -> Tuple[float, float, float, float]:
        """Returns x, y, vx, vy."""
        pos, _ = p.getBasePositionAndOrientation(self.body_id, physicsClientId=self.cid)
        vel, _ = p.getBaseVelocity(self.body_id, physicsClientId=self.cid)
        return pos[0], pos[1], vel[0], vel[1]
