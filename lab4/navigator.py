"""
navigator.py — Path planner + follower for the AMR.
Pipeline: PF estimated pose -> grid cell -> A* -> world waypoints -> (v, w) commands.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from lab4.planner_astar import astar, inflate_grid
from lab4.world import cell_center_to_world, world_to_cell


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class NavConfig:
    cell_size: float = 0.5
    lookahead_m: float = 0.6
    v_nom: float = 2.0              # Fast speed for 60s target
    w_max: float = 3.0
    k_heading: float = 3.5
    goal_tol_m: float = 0.6
    # Dynamic obstacle avoidance
    safety_radius: float = 0.35
    slowdown_radius: float = 0.70
    # Wall avoidance thresholds (will be scaled to cell_size in Navigator.__init__)
    wall_front_stop: float = 0.35     # Full stop if wall ahead closer than this
    wall_front_slow: float = 0.60     # Start slowing if wall ahead closer than this
    wall_side_push: float = 0.30      # Start side-push if wall closer than this


class Navigator:
    """
    Pipeline: estimated pose (PF) -> cell -> A* -> world path -> follow (v, w).
    Supports dynamic obstacle (human) avoidance via set_dynamic_obstacles().
    """

    def __init__(self, grid, cfg: NavConfig):
        self.grid = grid
        self.cfg = cfg
        self.rows = len(grid)
        self.cols = len(grid[0])

        # Inflated grid for A* planning (robot clearance)
        self._inflated = inflate_grid(grid, radius=1)

        self.goal_cell: Optional[Tuple[int, int]] = None
        self.path_cells: List[Tuple[int, int]] = []
        self.path_world: List[Tuple[float, float]] = []
        self._path_idx = 0

        # Dynamic obstacles (positions updated each tick)
        self._dyn_obstacles: List[Tuple[float, float]] = []

    def set_goal_cell(self, r: int, c: int):
        """Set goal cell and reset path."""
        self.goal_cell = (r, c)
        self.path_cells = []
        self.path_world = []
        self._path_idx = 0

    def set_dynamic_obstacles(self, positions: List[Tuple[float, float]]):
        """Update list of dynamic obstacle positions (e.g., humans)."""
        self._dyn_obstacles = list(positions)

    def plan_from_pose(self, x: float, y: float) -> bool:
        """Plan a path from world position (x, y) to goal cell using A*.
        Uses inflated grid first (for robot clearance), falls back to raw grid."""
        if self.goal_cell is None:
            return False

        start = world_to_cell(x, y, self.rows, self.cols, self.cfg.cell_size)

        # Try inflated grid first (safer paths)
        path = astar(self._inflated, start, self.goal_cell, eight_connected=True)
        if not path:
            # Fallback to raw grid (goal/start might be in inflated zone)
            path = astar(self.grid, start, self.goal_cell, eight_connected=True)
        if not path:
            self.path_cells = []
            self.path_world = []
            self._path_idx = 0
            return False

        self.path_cells = path
        raw_world = [
            cell_center_to_world(r, c, self.rows, self.cols, self.cfg.cell_size)
            for (r, c) in path
        ]
        # Keep all waypoints — smoothing can clip corners through walls
        self.path_world = raw_world
        self._path_idx = 0
        return True

    def _pick_lookahead(self, x: float, y: float) -> Tuple[float, float]:
        """Advance path index to find the next lookahead point."""
        if not self.path_world:
            return (x, y)

        # Advance index past nearby points
        while self._path_idx < len(self.path_world) - 1:
            px, py = self.path_world[self._path_idx]
            if math.hypot(px - x, py - y) > self.cfg.lookahead_m * 0.5:
                break
            self._path_idx += 1

        # Find first point beyond lookahead distance
        best = self.path_world[self._path_idx]
        for j in range(self._path_idx, len(self.path_world)):
            px, py = self.path_world[j]
            if math.hypot(px - x, py - y) >= self.cfg.lookahead_m:
                best = (px, py)
                break
            best = (px, py)
        return best

    def _nearest_obstacle_dist(self, x: float, y: float) -> float:
        """Distance to nearest dynamic obstacle."""
        if not self._dyn_obstacles:
            return float("inf")
        return min(math.hypot(ox - x, oy - y)
                   for ox, oy in self._dyn_obstacles)

    def compute_cmd(self, x: float, y: float, th: float, scan: Optional[Dict[str, float]] = None) -> Tuple[float, float, bool]:
        """
        Compute velocity command (v, w) and whether the goal is reached.
        Returns: (v, w, done)
        """
        if self.goal_cell is None:
            return 0.0, 0.0, True

        if not self.path_world:
            return 0.0, 0.0, False

        gx, gy = self.path_world[-1]
        dist_to_goal = math.hypot(gx - x, gy - y)

        # Check goal reached
        if dist_to_goal <= self.cfg.goal_tol_m:
            return 0.0, 0.0, True

        # Dynamic obstacle check
        obs_dist = self._nearest_obstacle_dist(x, y)
        if obs_dist < self.cfg.safety_radius:
            return 0.0, 0.0, False   # full stop
        obs_factor = 1.0
        if obs_dist < self.cfg.slowdown_radius:
            obs_factor = max(0.2, (obs_dist - self.cfg.safety_radius)
                             / (self.cfg.slowdown_radius - self.cfg.safety_radius))
            # print(f"[NAV] Dynamic Obstacle Avoidance: dist={obs_dist:.2f}m factor={obs_factor:.2f}")

        # Near end of path: aim directly at goal
        if dist_to_goal < 1.2:
            tx, ty = gx, gy
        else:
            tx, ty = self._pick_lookahead(x, y)

        target_heading = math.atan2(ty - y, tx - x)
        err = _wrap(target_heading - th)

        # Calculate base commands
        is_turning_in_place = False
        if abs(err) > (math.pi / 4):
            # Large error: rotate in place
            is_turning_in_place = True
            v = 0.0
            w = _clamp(self.cfg.k_heading * err, -self.cfg.w_max, self.cfg.w_max)
        else:
            # Normal tracking
            w = _clamp(self.cfg.k_heading * err, -self.cfg.w_max, self.cfg.w_max)
            heading_factor = max(0.1, (1.0 - abs(err) / (math.pi / 4)) ** 2)
            v = self.cfg.v_nom * heading_factor * obs_factor

            # Slow down near goal
            if dist_to_goal < 0.8:
                v *= max(0.3, dist_to_goal / 0.8)

        # Reactive wall avoidance
        if scan:
            front_d = scan.get('front', 10.0)
            left_d = scan.get('left', 10.0)
            right_d = scan.get('right', 10.0)

            # ── Front wall: stop and turn away ──
            if front_d < self.cfg.wall_front_stop:
                # Wall very close ahead — stop forward motion & turn toward open side
                v = 0.0
                if not is_turning_in_place:
                    # Turn toward whichever side has more space
                    if left_d > right_d:
                        w = self.cfg.w_max * 0.7   # turn left
                    else:
                        w = -self.cfg.w_max * 0.7  # turn right
            elif front_d < self.cfg.wall_front_slow:
                # Wall approaching — proportional slowdown
                slow_factor = (front_d - self.cfg.wall_front_stop) / (
                    self.cfg.wall_front_slow - self.cfg.wall_front_stop)
                v *= _clamp(slow_factor, 0.1, 1.0)

            # ── Side walls: gentle push away ──
            if not is_turning_in_place:
                if left_d < self.cfg.wall_side_push:
                    push = 2.0 * (self.cfg.wall_side_push - left_d) / self.cfg.wall_side_push
                    w -= push
                if right_d < self.cfg.wall_side_push:
                    push = 2.0 * (self.cfg.wall_side_push - right_d) / self.cfg.wall_side_push
                    w += push

        # Clamp w finally
        w = _clamp(w, -self.cfg.w_max, self.cfg.w_max)

        return float(v), float(w), False


def _smooth_path(pts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Remove collinear intermediate waypoints to reduce unnecessary turns."""
    if len(pts) <= 2:
        return pts
    result = [pts[0]]
    for i in range(1, len(pts) - 1):
        x0, y0 = result[-1]
        x1, y1 = pts[i]
        x2, y2 = pts[i + 1]
        # Cross product to detect collinearity
        cross = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
        if abs(cross) > 1e-6:
            result.append(pts[i])
    result.append(pts[-1])
    return result
