"""
UAV-X Phase 3 – Fast-Planner Bridge with ESDF + A* + B-Spline
==============================================================
Implements a self-contained trajectory planner suitable for UAV navigation
in the UAV-X disaster-response scenario.

Architecture
------------
1. **ESDF (Euclidean Signed Distance Field)**
   A 2-D numpy grid where each cell stores the *distance to the nearest
   obstacle*.  Built using multi-source BFS from all obstacle cells.
   Provides instant obstacle-proximity lookup during planning.

2. **A* Planner**
   Grid-based A* search from start to goal.  Cell cost is augmented by an
   obstacle-proximity penalty so paths naturally stay away from buildings.

3. **B-Spline Smoothing**
   The raw A* waypoint list (grid-aligned, staircase-shaped) is smoothed
   into a continuous curve using:
     - ``scipy.interpolate.splprep`` / ``splev`` (if scipy installed), OR
     - A simple moving-average smoother (pure-Python / numpy fallback).

Usage
-----
>>> planner = FastPlannerBridge(world_size=(2200, 1400), resolution=20.0)
>>> planner.update_esdf(
...     obstacle_positions=[(500, 300), (900, 100), (700, 900)],
...     radius=30.0,
... )
>>> traj = planner.plan_trajectory(
...     start=(0, 0, 30),
...     goal=(1200, 600, 30),
...     max_vel=12.0,
... )
>>> smooth = planner.get_smoothed_path(traj, num_points=100)

Dependencies
------------
  numpy   >= 1.21  (optional but strongly recommended)
  scipy   >= 1.7   (optional, for B-spline smoothing)
"""

import heapq
import logging
import math
from collections import deque
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger("FastPlannerBridge")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)

# ---------------------------------------------------------------------------
# Optional numpy import
# ---------------------------------------------------------------------------
try:
    import numpy as np
    _NP = True
except ImportError:
    np = None  # type: ignore[assignment]
    _NP = False
    logger.warning(
        "numpy not installed.  FastPlannerBridge will use pure-Python "
        "arrays (slower).  Install with:  pip install numpy"
    )

# ---------------------------------------------------------------------------
# Optional scipy import (for B-spline smoothing)
# ---------------------------------------------------------------------------
try:
    from scipy.interpolate import splprep, splev  # type: ignore
    _SCIPY = True
except ImportError:
    splprep = None  # type: ignore[assignment]
    splev  = None   # type: ignore[assignment]
    _SCIPY = False
    logger.warning(
        "scipy not installed.  B-spline smoothing will use moving-average "
        "fallback.  Install with:  pip install scipy"
    )


# ===========================================================================
# Constants
# ===========================================================================

#: Cost multiplier applied to cells near obstacles in A*
_OBSTACLE_COST_WEIGHT: float = 8.0

#: Minimum safe distance from obstacle centre for zero extra cost (metres)
_SAFE_DIST_M: float = 40.0

#: A* heuristic: Euclidean distance (admissible)
_HEURISTIC = "euclidean"

#: Very large cost used to mark impassable cells
_INF_COST: float = float("inf")


# ===========================================================================
# FastPlannerBridge
# ===========================================================================

class FastPlannerBridge:
    """
    Self-contained trajectory planner combining ESDF, A*, and B-spline smoothing.

    Parameters
    ----------
    world_size : tuple[float, float]
        ``(width_m, height_m)`` of the simulated world in metres.
        Corresponds to the X and Y extents.  E.g. ``(2200, 1400)`` for the
        UAV-X disaster zone.
    resolution : float
        Grid cell size in metres.  Coarser = faster; finer = more accurate.
        Default 20.0 m (so a 2200×1400 m world → 110×70 cells).

    Attributes
    ----------
    grid_w : int
        Number of grid cells in the X direction.
    grid_h : int
        Number of grid cells in the Y direction.
    esdf : numpy.ndarray or list[list[float]]
        2-D array ``[gx][gy]`` of distance-to-nearest-obstacle (metres).
        Initialised to *resolution* × max_dim (i.e. fully free space).

    Examples
    --------
    >>> planner = FastPlannerBridge(world_size=(2200, 1400), resolution=20.0)
    >>> planner.update_esdf([(500, 300, 0), (900, 100, 0)], radius=30.0)
    >>> path = planner.plan_trajectory((0, 0, 30), (1100, 600, 30))
    >>> smooth = planner.get_smoothed_path(path, num_points=80)
    """

    def __init__(
        self,
        world_size: Tuple[float, float] = (2200.0, 1400.0),
        resolution: float = 20.0,
    ) -> None:
        self.world_w, self.world_h = float(world_size[0]), float(world_size[1])
        self.resolution = float(resolution)

        # Grid dimensions (number of cells)
        self.grid_w = max(1, int(math.ceil(self.world_w / self.resolution)))
        self.grid_h = max(1, int(math.ceil(self.world_h / self.resolution)))

        # Large "free-space" distance initialisation
        _free = self.world_w + self.world_h

        if _NP:
            # ESDF: shape (grid_w, grid_h), dtype float32
            self.esdf: "np.ndarray" = np.full(
                (self.grid_w, self.grid_h), _free, dtype=np.float32
            )
            # Occupancy grid: True = occupied
            self._occupied: "np.ndarray" = np.zeros(
                (self.grid_w, self.grid_h), dtype=bool
            )
        else:
            # Pure-Python fallback: 2-D list
            self.esdf = [
                [_free] * self.grid_h for _ in range(self.grid_w)
            ]
            self._occupied = [
                [False] * self.grid_h for _ in range(self.grid_w)
            ]

        logger.info(
            "FastPlannerBridge: world=(%.0f×%.0f m), resolution=%.1f m, "
            "grid=(%d×%d cells)",
            self.world_w, self.world_h,
            self.resolution,
            self.grid_w, self.grid_h,
        )

    # ------------------------------------------------------------------
    # Coordinate conversion helpers
    # ------------------------------------------------------------------

    def _world_to_grid(
        self, x: float, y: float
    ) -> Tuple[int, int]:
        """
        Convert world coordinates (metres) to grid cell indices.

        Clamps to valid grid range.
        """
        gx = int(x / self.resolution)
        gy = int(y / self.resolution)
        gx = max(0, min(self.grid_w - 1, gx))
        gy = max(0, min(self.grid_h - 1, gy))
        return gx, gy

    def _grid_to_world(
        self, gx: int, gy: int
    ) -> Tuple[float, float]:
        """
        Convert grid cell centre to world coordinates (metres).
        """
        x = (gx + 0.5) * self.resolution
        y = (gy + 0.5) * self.resolution
        return x, y

    def _esdf_get(self, gx: int, gy: int) -> float:
        """Get ESDF value at grid cell (gx, gy)."""
        if _NP:
            return float(self.esdf[gx, gy])
        return self.esdf[gx][gy]

    def _esdf_set(self, gx: int, gy: int, val: float) -> None:
        """Set ESDF value at grid cell (gx, gy)."""
        if _NP:
            self.esdf[gx, gy] = val
        else:
            self.esdf[gx][gy] = val

    def _occ_get(self, gx: int, gy: int) -> bool:
        """Return True if cell (gx, gy) is occupied."""
        if _NP:
            return bool(self._occupied[gx, gy])
        return self._occupied[gx][gy]

    def _occ_set(self, gx: int, gy: int, val: bool) -> None:
        """Mark cell (gx, gy) as occupied/free."""
        if _NP:
            self._occupied[gx, gy] = val
        else:
            self._occupied[gx][gy] = val

    # ------------------------------------------------------------------
    # ESDF
    # ------------------------------------------------------------------

    def update_esdf(
        self,
        obstacle_positions: List[Tuple],
        radius: float = 30.0,
    ) -> None:
        """
        Mark obstacle cells and recompute the full ESDF via multi-source BFS.

        The BFS propagates outward from every obstacle cell, setting each
        reached cell's ESDF value to its distance to the nearest obstacle.
        This is the standard method used in Fast-Planner / EGO-Swarm.

        Parameters
        ----------
        obstacle_positions : list
            Each element is either ``(x, y)`` or ``(x, y, z)`` in world
            metres.  The Z component is ignored (2-D grid).
        radius : float
            Radius (metres) around each obstacle centre to mark as occupied.
            All cells whose centres lie within this radius are marked.

        Examples
        --------
        >>> planner.update_esdf(
        ...     [(500, 300, 0), (900, 100, 0), (700, 900, 0)],
        ...     radius=30.0,
        ... )
        """
        if not obstacle_positions:
            logger.debug("update_esdf: no obstacles provided.")
            return

        # Radius in grid cells (ceiling to be conservative)
        r_cells = int(math.ceil(radius / self.resolution))

        # Reset occupancy grid
        if _NP:
            self._occupied[:] = False
            self.esdf[:] = self.world_w + self.world_h
        else:
            for gx in range(self.grid_w):
                for gy in range(self.grid_h):
                    self._occ_set(gx, gy, False)
                    self._esdf_set(gx, gy, self.world_w + self.world_h)

        # ---- Phase 1: mark obstacle cells --------------------------------
        seed_cells: List[Tuple[int, int]] = []

        for obs in obstacle_positions:
            ox, oy = float(obs[0]), float(obs[1])
            c_gx, c_gy = self._world_to_grid(ox, oy)

            for dgx in range(-r_cells, r_cells + 1):
                for dgy in range(-r_cells, r_cells + 1):
                    gx = c_gx + dgx
                    gy = c_gy + dgy
                    if not (0 <= gx < self.grid_w and 0 <= gy < self.grid_h):
                        continue
                    # Check actual world distance
                    wx, wy = self._grid_to_world(gx, gy)
                    dist = math.sqrt((wx - ox) ** 2 + (wy - oy) ** 2)
                    if dist <= radius:
                        self._occ_set(gx, gy, True)
                        self._esdf_set(gx, gy, 0.0)
                        seed_cells.append((gx, gy))

        # ---- Phase 2: multi-source BFS to compute ESDF ------------------
        # Each cell stores the Euclidean distance to the nearest obstacle
        # cell (in world metres).
        queue: deque = deque(seed_cells)
        # 8-connected neighbourhood
        neighbours = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ]

        while queue:
            gx, gy = queue.popleft()
            current_dist = self._esdf_get(gx, gy)

            for dgx, dgy in neighbours:
                ngx = gx + dgx
                ngy = gy + dgy
                if not (0 <= ngx < self.grid_w and 0 <= ngy < self.grid_h):
                    continue

                # Distance from neighbour to current obstacle-boundary cell
                # (diagonal cells are √2 × resolution away)
                step_dist = math.sqrt(dgx ** 2 + dgy ** 2) * self.resolution
                new_dist = current_dist + step_dist

                if new_dist < self._esdf_get(ngx, ngy):
                    self._esdf_set(ngx, ngy, new_dist)
                    queue.append((ngx, ngy))

        n_obs_cells = len(set(seed_cells))
        logger.info(
            "ESDF updated: %d obstacle positions, %d occupied cells, "
            "BFS propagated.",
            len(obstacle_positions), n_obs_cells,
        )

    # ------------------------------------------------------------------
    # A* Planner
    # ------------------------------------------------------------------

    def plan_trajectory(
        self,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
        max_vel: float = 12.0,
    ) -> List[Tuple[float, float, float]]:
        """
        Plan a collision-avoiding trajectory from *start* to *goal*.

        Uses A* on the 2-D ESDF grid with an obstacle-proximity cost term.
        The Z component is preserved from *start* and linearly interpolated
        to *goal* along the path.

        Parameters
        ----------
        start : (x, y, z)
            Starting world position in metres.
        goal : (x, y, z)
            Goal world position in metres.
        max_vel : float
            Maximum flight velocity (m/s).  Currently stored as metadata
            for downstream velocity profiling; not used in A* itself.

        Returns
        -------
        list of (x, y, z) waypoints from *start* to *goal*.
        Returns ``[start, goal]`` if A* fails (no path found).

        Examples
        --------
        >>> traj = planner.plan_trajectory((0, 0, 30), (1100, 600, 40))
        """
        sx, sy, sz = float(start[0]), float(start[1]), float(start[2])
        gx_w, gy_w, gz_w = float(goal[0]), float(goal[1]), float(goal[2])

        s_cell = self._world_to_grid(sx, sy)
        g_cell = self._world_to_grid(gx_w, gy_w)

        if s_cell == g_cell:
            return [start, goal]

        # ---- A* search --------------------------------------------------
        # Priority queue: (f_cost, g_cost, (gx, gy))
        open_heap: List[Tuple[float, float, Tuple[int, int]]] = []
        g_cost_map: Dict[Tuple[int, int], float] = {s_cell: 0.0}
        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {
            s_cell: None
        }

        def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
            # Euclidean heuristic (admissible)
            return math.sqrt(
                (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
            ) * self.resolution

        start_h = heuristic(s_cell, g_cell)
        heapq.heappush(open_heap, (start_h, 0.0, s_cell))

        # 8-connected neighbours
        neighbours_offsets = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
        ]

        found = False
        max_iterations = self.grid_w * self.grid_h * 4

        for _ in range(max_iterations):
            if not open_heap:
                break
            _, g_cost, current = heapq.heappop(open_heap)

            if current == g_cell:
                found = True
                break

            # Skip if we already have a cheaper path to this cell
            if g_cost > g_cost_map.get(current, _INF_COST):
                continue

            cx, cy = current
            for dx, dy, move_cost in neighbours_offsets:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < self.grid_w and 0 <= ny < self.grid_h):
                    continue

                # Cells deeply inside obstacles are impassable
                esdf_val = self._esdf_get(nx, ny)
                if esdf_val < 1.0:
                    continue

                # Step cost: movement distance + obstacle proximity penalty
                proximity_penalty = 0.0
                if esdf_val < _SAFE_DIST_M:
                    proximity_penalty = (
                        _OBSTACLE_COST_WEIGHT
                        * (_SAFE_DIST_M - esdf_val)
                        / _SAFE_DIST_M
                    )

                step = move_cost * self.resolution + proximity_penalty
                new_g = g_cost + step

                if new_g < g_cost_map.get((nx, ny), _INF_COST):
                    g_cost_map[(nx, ny)] = new_g
                    came_from[(nx, ny)] = current
                    f_cost = new_g + heuristic((nx, ny), g_cell)
                    heapq.heappush(open_heap, (f_cost, new_g, (nx, ny)))

        if not found:
            logger.warning(
                "A* could not find path from %s to %s – returning straight line.",
                s_cell, g_cell,
            )
            return [start, goal]

        # ---- Reconstruct path -------------------------------------------
        path_cells: List[Tuple[int, int]] = []
        current: Optional[Tuple[int, int]] = g_cell
        while current is not None:
            path_cells.append(current)
            current = came_from.get(current)
        path_cells.reverse()

        # ---- Convert to world coordinates with Z interpolation ----------
        n = len(path_cells)
        waypoints: List[Tuple[float, float, float]] = []

        for i, (gx, gy) in enumerate(path_cells):
            wx, wy = self._grid_to_world(gx, gy)
            # Linear altitude interpolation from start_z to goal_z
            t = i / max(1, n - 1)
            wz = sz + t * (gz_w - sz)
            waypoints.append((wx, wy, wz))

        # Force exact start/end coordinates
        if waypoints:
            waypoints[0] = start
            waypoints[-1] = goal

        logger.info(
            "A* path found: %d cells → %d waypoints (max_vel=%.1f m/s).",
            len(path_cells), len(waypoints), max_vel,
        )
        return waypoints

    # ------------------------------------------------------------------
    # B-Spline / Moving-Average Smoothing
    # ------------------------------------------------------------------

    def get_smoothed_path(
        self,
        waypoints: List[Tuple[float, float, float]],
        num_points: int = 50,
    ) -> List[Tuple[float, float, float]]:
        """
        Smooth a raw waypoint list into a continuous curve.

        Uses ``scipy.interpolate.splprep`` / ``splev`` (cubic B-spline) if
        scipy is available; otherwise uses a simple weighted moving-average
        smoother over a 5-point window.

        Parameters
        ----------
        waypoints : list[(x, y, z)]
            Raw waypoints (e.g. from :meth:`plan_trajectory`).
        num_points : int
            Number of output sample points along the smoothed curve.

        Returns
        -------
        list of (x, y, z) tuples representing the smoothed path.
        Returns *waypoints* unchanged if fewer than 3 points provided.

        Examples
        --------
        >>> raw_path = planner.plan_trajectory((0, 0, 30), (1100, 600, 30))
        >>> smooth = planner.get_smoothed_path(raw_path, num_points=100)
        """
        if len(waypoints) < 3:
            return list(waypoints)

        if _SCIPY:
            return self._bspline_scipy(waypoints, num_points)
        else:
            return self._moving_average_smooth(waypoints, num_points)

    def _bspline_scipy(
        self,
        waypoints: List[Tuple[float, float, float]],
        num_points: int,
    ) -> List[Tuple[float, float, float]]:
        """
        Fit a cubic B-spline through *waypoints* using scipy and sample it
        at *num_points* evenly-spaced parameter values.

        ``splprep`` finds the B-spline representation ``(tck)`` of a parametric
        curve passing through the data points.  ``splev`` evaluates it.

        Smoothing factor ``s`` is set to ``len(waypoints) * 5`` – this allows
        the spline to deviate slightly from input points (global smoothing),
        preventing the wavy artefacts of exact interpolation on noisy paths.
        """
        xs = [float(w[0]) for w in waypoints]
        ys = [float(w[1]) for w in waypoints]
        zs = [float(w[2]) for w in waypoints]

        try:
            s_factor = float(len(waypoints)) * 5.0
            # k=3 → cubic spline; s → smoothing factor
            tck, _ = splprep([xs, ys, zs], s=s_factor, k=min(3, len(waypoints) - 1))
            u_new = [i / (num_points - 1) for i in range(num_points)]
            out = splev(u_new, tck)
            smooth = list(zip(out[0], out[1], out[2]))
            logger.debug(
                "B-spline smoothing (scipy): %d → %d points.",
                len(waypoints), len(smooth),
            )
            return smooth
        except Exception as exc:
            logger.warning("scipy splprep failed (%s) – using moving average.", exc)
            return self._moving_average_smooth(waypoints, num_points)

    def _moving_average_smooth(
        self,
        waypoints: List[Tuple[float, float, float]],
        num_points: int,
        window: int = 5,
    ) -> List[Tuple[float, float, float]]:
        """
        Smooth *waypoints* using a weighted moving-average kernel, then
        re-sample to exactly *num_points* points via linear interpolation.

        Window weights: [1, 2, 4, 2, 1] (Gaussian-like, sums to 10).

        The first and last waypoints are kept fixed (boundary conditions).
        """
        pts = list(waypoints)
        n = len(pts)

        if n < window:
            # Not enough points for the chosen window; use plain average
            window = max(3, n // 2 * 2 + 1)

        # Weighted kernel
        kernel = [1.0, 2.0, 4.0, 2.0, 1.0][:window]
        k_sum = sum(kernel)

        smoothed = [pts[0]]  # Keep start fixed

        half_w = window // 2
        for i in range(1, n - 1):
            sx = sy = sz = 0.0
            for ki, w in enumerate(kernel):
                idx = i - half_w + ki
                idx = max(0, min(n - 1, idx))  # Clamp to valid range
                sx += pts[idx][0] * w
                sy += pts[idx][1] * w
                sz += pts[idx][2] * w
            smoothed.append((sx / k_sum, sy / k_sum, sz / k_sum))

        smoothed.append(pts[-1])  # Keep end fixed

        # ---- Re-sample to num_points via linear interpolation -----------
        if num_points <= len(smoothed):
            # Downsample: pick evenly-spaced indices
            step = (len(smoothed) - 1) / max(1, num_points - 1)
            resampled = []
            for i in range(num_points):
                idx = min(int(i * step), len(smoothed) - 1)
                resampled.append(smoothed[idx])
            return resampled

        # Upsample: linear interpolation between consecutive smoothed points
        resampled = []
        for i in range(num_points):
            t = i / (num_points - 1) * (len(smoothed) - 1)
            lo = int(t)
            hi = min(lo + 1, len(smoothed) - 1)
            alpha = t - lo
            p0 = smoothed[lo]
            p1 = smoothed[hi]
            resampled.append((
                p0[0] + alpha * (p1[0] - p0[0]),
                p0[1] + alpha * (p1[1] - p0[1]),
                p0[2] + alpha * (p1[2] - p0[2]),
            ))

        logger.debug(
            "Moving-average smoothing: %d → %d points.",
            len(waypoints), len(resampled),
        )
        return resampled

    def get_esdf_value(
        self, x: float, y: float
    ) -> float:
        """
        Query the ESDF at a world position ``(x, y)``.

        Returns the distance (metres) from ``(x, y)`` to the nearest
        obstacle, as stored in the ESDF grid.

        Parameters
        ----------
        x, y : float
            World coordinates in metres.

        Returns
        -------
        float
            Distance to nearest obstacle in metres.
        """
        gx, gy = self._world_to_grid(x, y)
        return self._esdf_get(gx, gy)

    def is_free(self, x: float, y: float, clearance: float = 5.0) -> bool:
        """
        Return True if position ``(x, y)`` is farther than *clearance*
        metres from any obstacle.

        Parameters
        ----------
        x, y : float
            World coordinates in metres.
        clearance : float
            Minimum required distance to obstacles (metres).

        Returns
        -------
        bool
        """
        return self.get_esdf_value(x, y) > clearance

    def __repr__(self) -> str:
        return (
            f"FastPlannerBridge("
            f"world=({self.world_w:.0f}×{self.world_h:.0f} m), "
            f"resolution={self.resolution:.1f} m, "
            f"grid=({self.grid_w}×{self.grid_h}))"
        )


# ===========================================================================
# Module demo / self-test
# ===========================================================================

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("=== FastPlannerBridge demo ===")

    planner = FastPlannerBridge(world_size=(2200.0, 1400.0), resolution=20.0)
    logger.info("Planner: %s", planner)

    # Register obstacles (collapsed buildings from disaster_zone.sdf)
    obstacle_positions = [
        (500.0,  300.0,  0.0),
        (900.0,  100.0,  0.0),
        (700.0,  900.0,  0.0),
        (1200.0, 600.0,  0.0),
        (300.0,  1100.0, 0.0),
    ]
    planner.update_esdf(obstacle_positions, radius=30.0)

    # Plan trajectories for several UAVs
    scenarios = [
        ("uav_0", (10.0, 10.0, 30.0),   (500.0,  300.0, 30.0)),
        ("uav_1", (10.0, 10.0, 40.0),   (900.0,  100.0, 40.0)),
        ("uav_2", (10.0, 10.0, 35.0),   (1200.0, 600.0, 35.0)),
    ]

    for uid, start, goal in scenarios:
        raw_path = planner.plan_trajectory(start, goal, max_vel=12.0)
        smooth_path = planner.get_smoothed_path(raw_path, num_points=50)

        logger.info(
            "[%s] raw=%d wpts → smooth=%d pts | start=%s goal=%s",
            uid, len(raw_path), len(smooth_path), raw_path[0], raw_path[-1],
        )

        # Check that the smoothed path start/end are close to desired
        s_err = math.sqrt(sum((a - b) ** 2 for a, b in zip(smooth_path[0], start)))
        g_err = math.sqrt(sum((a - b) ** 2 for a, b in zip(smooth_path[-1], goal)))
        logger.info(
            "  Endpoint errors: start=%.1f m, goal=%.1f m", s_err, g_err
        )

    logger.info("Demo complete.")
