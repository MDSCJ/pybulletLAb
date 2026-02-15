"""
human.py — Simulated human agent for dynamic obstacle testing.
"""
from __future__ import annotations

import math
import random
import numpy as np
import pybullet as p
from typing import List, Tuple, Optional

from lab4.world import cell_center_to_world

class SimulatedHuman:
    def __init__(self, cid: int, start_x: float, start_y: float, 
                 grid: List[List[float]], cell_size: float, seed: int = None):
        self.cid = cid
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.cell_size = cell_size
        self.rng = random.Random(seed) if seed is not None else random.Random()

        # Physics body (Capsule)
        radius = 0.25
        height = 1.7
        visual_shape = p.createVisualShape(
            p.GEOM_CAPSULE, radius=radius, length=height, 
            rgbaColor=[0.2, 0.8, 0.2, 1], physicsClientId=cid)
        col_shape = p.createCollisionShape(
            p.GEOM_CAPSULE, radius=radius, height=height, physicsClientId=cid)
        
        self.body_id = p.createMultiBody(
            baseMass=60.0,
            baseCollisionShapeIndex=col_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=[start_x, start_y, height/2 + 0.1],
            physicsClientId=cid
        )
        
        # Behavior
        self.speed = 1.0  # m/s
        self.target: Optional[Tuple[float, float]] = None
        self._pick_new_target()

    def update(self, dt: float):
        """Move towards target."""
        pos, rot = p.getBasePositionAndOrientation(self.body_id, physicsClientId=self.cid)
        x, y = pos[0], pos[1]
        
        if self.target is None:
            self._pick_new_target()
            return

        tx, ty = self.target
        dist = math.hypot(tx - x, ty - y)
        
        if dist < 0.5:
            self._pick_new_target()
            return
            
        # Move towards target
        heading = math.atan2(ty - y, tx - x)
        vx = self.speed * math.cos(heading)
        vy = self.speed * math.sin(heading)
        
        # Set velocity (kinematic-ish control via linear velocity)
        # Keep Z velocity existing (gravity)
        lin_vel, ang_vel = p.getBaseVelocity(self.body_id, physicsClientId=self.cid)
        p.resetBaseVelocity(self.body_id, [vx, vy, lin_vel[2]], [0, 0, 0], physicsClientId=self.cid)
        
        # Keep upright
        p.resetBasePositionAndOrientation(self.body_id, [x, y, pos[2]], [0, 0, 0, 1], physicsClientId=self.cid)

    def get_pose(self) -> Tuple[float, float, float, float]:
        """Returns x, y, vx, vy."""
        pos, _ = p.getBasePositionAndOrientation(self.body_id, physicsClientId=self.cid)
        vel, _ = p.getBaseVelocity(self.body_id, physicsClientId=self.cid)
        return pos[0], pos[1], vel[0], vel[1]

    def _pick_new_target(self):
        """Pick a random free cell as target."""
        for _ in range(100):
            r = self.rng.randint(1, self.rows - 2)
            c = self.rng.randint(1, self.cols - 2)
            if self.grid[r][c] == 0:
                tx, ty = cell_center_to_world(r, c, self.rows, self.cols, self.cell_size)
                # Check line of sight? Nah, just go. Physics deals with collisions.
                self.target = (tx, ty)
                return
        # Fallback: stay put
        self.target = None
