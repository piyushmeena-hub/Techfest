"""
sim/obstacles.py: 3D Axis-Aligned Bounding Box (AABB) Obstacles & Vectorized Ray-Slab Occlusion Engine.

Implements the Williams et al. (2005) 3D Ray-AABB slab intersection algorithm,
exact penetration distance calculation, NLoS RF attenuation modeling,
and Khatib APF obstacle repulsion forces.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
import numpy as np


@dataclass
class RayIntersectionResult:
    """Detailed geometric outcome of a 3D ray segment intersection with an obstacle."""
    hit: bool
    t_enter: float
    t_exit: float
    penetration_distance: float
    entry_point: Optional[np.ndarray] = None
    exit_point: Optional[np.ndarray] = None
    obstacle_id: Optional[str] = None
    attenuation_db: float = 0.0


@dataclass
class ObstacleAABB:
    """
    3D Axis-Aligned Bounding Box obstacle representing disaster structures and debris.
    """
    id: str
    name: str
    min_pt: np.ndarray  # [x_min, y_min, z_min]
    max_pt: np.ndarray  # [x_max, y_max, z_max]
    material: str = "reinforced_concrete"
    base_attenuation_db: float = 22.0
    attenuation_db_per_meter: float = 1.5

    def __post_init__(self) -> None:
        self.min_pt = np.asarray(self.min_pt, dtype=np.float64)
        self.max_pt = np.asarray(self.max_pt, dtype=np.float64)

        if self.min_pt.shape != (3,) or self.max_pt.shape != (3,):
            raise ValueError(
                f"Obstacle corner vectors must have shape (3,), got min={self.min_pt.shape}, max={self.max_pt.shape}"
            )
        if np.any(self.min_pt >= self.max_pt):
            raise ValueError(
                f"Obstacle {self.id}: min_pt must be strictly less than max_pt. Got min={self.min_pt}, max={self.max_pt}"
            )
        if self.min_pt[2] < 0.0:
            raise ValueError(f"Obstacle {self.id}: z_min cannot be below ground (0.0). Got {self.min_pt[2]}")

    @property
    def min_bound(self) -> np.ndarray:
        """Alias for min_pt."""
        return self.min_pt

    @property
    def max_bound(self) -> np.ndarray:
        """Alias for max_pt."""
        return self.max_pt

    @property
    def center(self) -> np.ndarray:
        """Returns 3D centroid of AABB."""
        return 0.5 * (self.min_pt + self.max_pt)

    @property
    def extents(self) -> np.ndarray:
        """Returns [dx, dy, dz] dimensions in meters."""
        return self.max_pt - self.min_pt

    @property
    def volume(self) -> float:
        """Returns enclosed volume in cubic meters."""
        e = self.extents
        return float(e[0] * e[1] * e[2])

    @property
    def height(self) -> float:
        """Returns vertical height in meters."""
        return float(self.max_pt[2] - self.min_pt[2])

    def contains_point(self, point: np.ndarray, margin: float = 0.0) -> bool:
        """Checks if 3D point lies inside or on the boundary of the AABB."""
        p = np.asarray(point, dtype=np.float64)
        return bool(np.all((p >= self.min_pt - margin) & (p <= self.max_pt + margin)))

    def closest_point(self, point: np.ndarray) -> np.ndarray:
        """Returns the point on or inside the AABB closest to the given point."""
        p = np.asarray(point, dtype=np.float64)
        return np.clip(p, self.min_pt, self.max_pt)

    def distance_to_point(self, point: np.ndarray) -> float:
        """
        Returns Euclidean distance from point to the AABB surface.
        Returns 0.0 if point is strictly inside the AABB.
        """
        p = np.asarray(point, dtype=np.float64)
        closest = self.closest_point(p)
        return float(np.linalg.norm(p - closest))

    def distance_and_closest_point(self, point: np.ndarray) -> Tuple[float, np.ndarray]:
        """Returns tuple of (distance, closest_point)."""
        p = np.asarray(point, dtype=np.float64)
        closest = np.clip(p, self.min_pt, self.max_pt)
        dist = float(np.linalg.norm(p - closest))
        return dist, closest

    def surface_normal(self, point: np.ndarray) -> np.ndarray:
        """
        Computes the outward unit normal vector from the AABB face closest to point.
        """
        p = np.asarray(point, dtype=np.float64)
        c = self.center
        e = self.extents * 0.5
        d = p - c

        # Determine dominant axis direction relative to half-extents
        ratio = d / np.maximum(e, 1e-6)
        abs_ratio = np.abs(ratio)
        max_axis = int(np.argmax(abs_ratio))

        normal = np.zeros(3, dtype=np.float64)
        normal[max_axis] = 1.0 if d[max_axis] >= 0.0 else -1.0
        return normal

    def intersect_ray_segment(self, p_src: np.ndarray, p_dst: np.ndarray) -> RayIntersectionResult:
        """
        Executes robust Williams et al. slab intersection test on line segment [p_src, p_dst].
        Calculates exact penetration distance and RF attenuation.
        """
        src = np.asarray(p_src, dtype=np.float64)
        dst = np.asarray(p_dst, dtype=np.float64)
        d = dst - src
        length = float(np.linalg.norm(d))

        if length < 1e-9:
            inside = self.contains_point(src)
            return RayIntersectionResult(
                hit=inside,
                t_enter=0.0,
                t_exit=0.0,
                penetration_distance=0.0,
                entry_point=src.copy() if inside else None,
                exit_point=src.copy() if inside else None,
                obstacle_id=self.id if inside else None,
                attenuation_db=self.base_attenuation_db if inside else 0.0,
            )

        t_near = np.zeros(3, dtype=np.float64)
        t_far = np.zeros(3, dtype=np.float64)

        for i in range(3):
            if abs(d[i]) < 1e-12:
                # Parallel to axis slab i
                if src[i] < self.min_pt[i] or src[i] > self.max_pt[i]:
                    return RayIntersectionResult(hit=False, t_enter=0.0, t_exit=0.0, penetration_distance=0.0)
                t_near[i] = -np.inf
                t_far[i] = np.inf
            else:
                inv = 1.0 / d[i]
                t1 = (self.min_pt[i] - src[i]) * inv
                t2 = (self.max_pt[i] - src[i]) * inv
                t_near[i] = min(t1, t2)
                t_far[i] = max(t1, t2)

        t_enter = float(np.max(t_near))
        t_exit = float(np.min(t_far))

        # Check line segment [0, 1] intersection
        hit = (t_enter <= t_exit) and (t_exit >= 0.0) and (t_enter <= 1.0)
        if not hit:
            return RayIntersectionResult(hit=False, t_enter=t_enter, t_exit=t_exit, penetration_distance=0.0)

        # Clamped penetration parameters
        t_in = max(0.0, t_enter)
        t_out = min(1.0, t_exit)
        pen_dist = (t_out - t_in) * length

        pt_entry = src + t_in * d
        pt_exit = src + t_out * d
        attenuation = self.base_attenuation_db + self.attenuation_db_per_meter * pen_dist

        return RayIntersectionResult(
            hit=True,
            t_enter=t_enter,
            t_exit=t_exit,
            penetration_distance=pen_dist,
            entry_point=pt_entry,
            exit_point=pt_exit,
            obstacle_id=self.id,
            attenuation_db=attenuation,
        )

    def intersects_ray(self, p_src: np.ndarray, p_dst: np.ndarray) -> bool:
        """Fast boolean check for line segment intersection."""
        return self.intersect_ray_segment(p_src, p_dst).hit

    def to_dict(self) -> Dict[str, Any]:
        """Serializes obstacle metadata into dictionary."""
        return {
            "id": self.id,
            "name": self.name,
            "min_pt": self.min_pt.tolist(),
            "max_pt": self.max_pt.tolist(),
            "center": self.center.tolist(),
            "extents": self.extents.tolist(),
            "height": self.height,
            "material": self.material,
            "base_attenuation_db": self.base_attenuation_db,
            "attenuation_db_per_meter": self.attenuation_db_per_meter,
        }


# Alias for compatibility with explorer M1-3 and top-level package
Obstacle = ObstacleAABB


class ObstacleManager:
    """
    Manages collection of 3D obstacles, vectorized ray casting, APF repulsion, and batch evaluation.
    """

    def __init__(self, obstacles: Optional[List[ObstacleAABB]] = None) -> None:
        self.obstacles: List[ObstacleAABB] = []
        self._obstacle_map: Dict[str, ObstacleAABB] = {}

        # Cache array buffers for vectorization
        self._cached_min = np.empty((0, 3), dtype=np.float64)
        self._cached_max = np.empty((0, 3), dtype=np.float64)
        self._cached_base_att = np.empty(0, dtype=np.float64)
        self._cached_per_m_att = np.empty(0, dtype=np.float64)

        if obstacles:
            for obs in obstacles:
                self.add_obstacle(obs)

    def add_obstacle(self, obstacle: ObstacleAABB) -> None:
        """Registers a new obstacle and refreshes vectorized cache arrays."""
        if obstacle.id in self._obstacle_map:
            raise ValueError(f"Duplicate obstacle ID '{obstacle.id}'")
        self.obstacles.append(obstacle)
        self._obstacle_map[obstacle.id] = obstacle
        self._rebuild_cache()

    def _rebuild_cache(self) -> None:
        """Rebuilds contiguous NumPy matrices for vectorized operations."""
        M = len(self.obstacles)
        if M == 0:
            self._cached_min = np.empty((0, 3), dtype=np.float64)
            self._cached_max = np.empty((0, 3), dtype=np.float64)
            self._cached_base_att = np.empty(0, dtype=np.float64)
            self._cached_per_m_att = np.empty(0, dtype=np.float64)
            return

        self._cached_min = np.array([o.min_pt for o in self.obstacles], dtype=np.float64)
        self._cached_max = np.array([o.max_pt for o in self.obstacles], dtype=np.float64)
        self._cached_base_att = np.array([o.base_attenuation_db for o in self.obstacles], dtype=np.float64)
        self._cached_per_m_att = np.array([o.attenuation_db_per_meter for o in self.obstacles], dtype=np.float64)

    def get_obstacle(self, obstacle_id: str) -> Optional[ObstacleAABB]:
        """Retrieve obstacle by unique ID."""
        return self._obstacle_map.get(obstacle_id)

    def check_los(self, p_src: np.ndarray, p_dst: np.ndarray) -> Tuple[bool, float, float, List[str]]:
        """
        Evaluates Line-of-Sight between p_src and p_dst against all registered obstacles.
        Returns:
            is_los_clear: bool (True if NO obstacle occludes the ray)
            total_penetration: float (summed penetration distance in meters)
            total_attenuation: float (summed dB attenuation)
            hit_obstacle_ids: List[str] (IDs of all blocking obstacles)
        """
        M = len(self.obstacles)
        if M == 0:
            return True, 0.0, 0.0, []

        src = np.asarray(p_src, dtype=np.float64)
        dst = np.asarray(p_dst, dtype=np.float64)
        d = dst - src
        length = float(np.linalg.norm(d))

        if length < 1e-9:
            # Check point containment
            inside = np.all((src >= self._cached_min) & (src <= self._cached_max), axis=1)
            hits_idx = np.where(inside)[0]
            if len(hits_idx) > 0:
                hit_ids = [self.obstacles[i].id for i in hits_idx]
                att = float(np.sum(self._cached_base_att[hits_idx]))
                return False, 0.0, att, hit_ids
            return True, 0.0, 0.0, []

        t_near = np.full((M, 3), -np.inf, dtype=np.float64)
        t_far = np.full((M, 3), np.inf, dtype=np.float64)
        parallel_miss = np.zeros(M, dtype=bool)

        for i in range(3):
            if abs(d[i]) < 1e-12:
                miss = (src[i] < self._cached_min[:, i]) | (src[i] > self._cached_max[:, i])
                parallel_miss |= miss
            else:
                inv = 1.0 / d[i]
                t1 = (self._cached_min[:, i] - src[i]) * inv
                t2 = (self._cached_max[:, i] - src[i]) * inv
                t_near[:, i] = np.minimum(t1, t2)
                t_far[:, i] = np.maximum(t1, t2)

        t_enter = np.max(t_near, axis=1)
        t_exit = np.min(t_far, axis=1)

        hits = (t_enter <= t_exit) & (t_exit >= 0.0) & (t_enter <= 1.0) & (~parallel_miss)

        if not np.any(hits):
            return True, 0.0, 0.0, []

        t_in = np.maximum(0.0, t_enter[hits])
        t_out = np.minimum(1.0, t_exit[hits])
        pen_dists = (t_out - t_in) * length

        total_pen = float(np.sum(pen_dists))
        hit_indices = np.where(hits)[0]
        hit_ids = [self.obstacles[idx].id for idx in hit_indices]

        # Calculate attenuation
        base_losses = self._cached_base_att[hits]
        per_m_losses = self._cached_per_m_att[hits]
        total_att = float(np.sum(base_losses + per_m_losses * pen_dists))

        return False, total_pen, total_att, hit_ids

    def check_los_batch(
        self, sources: np.ndarray, targets: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Vectorized evaluation of K rays against all M obstacles.
        Args:
            sources: shape (K, 3)
            targets: shape (K, 3)
        Returns:
            los_clear: boolean array of shape (K,)
            total_penetration: float array of shape (K,)
            total_attenuation: float array of shape (K,)
        """
        K = sources.shape[0]
        M = len(self.obstacles)
        if M == 0 or K == 0:
            return np.ones(K, dtype=bool), np.zeros(K, dtype=np.float64), np.zeros(K, dtype=np.float64)

        d = targets - sources  # (K, 3)
        lengths = np.linalg.norm(d, axis=1)  # (K,)

        orig_exp = sources[:, np.newaxis, :]   # (K, 1, 3)
        d_exp = d[:, np.newaxis, :]             # (K, 1, 3)
        bmin_exp = self._cached_min[np.newaxis, :, :] # (1, M, 3)
        bmax_exp = self._cached_max[np.newaxis, :, :] # (1, M, 3)

        is_parallel = np.abs(d_exp) < 1e-12
        safe_d = np.where(is_parallel, 1.0, d_exp)
        inv_d = np.where(is_parallel, 0.0, 1.0 / safe_d)

        t1 = (bmin_exp - orig_exp) * inv_d
        t2 = (bmax_exp - orig_exp) * inv_d

        parallel_miss = is_parallel & ((orig_exp < bmin_exp) | (orig_exp > bmax_exp))
        any_parallel_miss = np.any(parallel_miss, axis=2)  # (K, M)

        t_near_dim = np.where(is_parallel, -np.inf, np.minimum(t1, t2))
        t_far_dim = np.where(is_parallel, np.inf, np.maximum(t1, t2))

        t_enter = np.max(t_near_dim, axis=2)  # (K, M)
        t_exit = np.min(t_far_dim, axis=2)    # (K, M)

        hits = (t_enter <= t_exit) & (t_exit >= 0.0) & (t_enter <= 1.0) & (~any_parallel_miss)

        # Zero-length handling
        zero_len = lengths < 1e-9
        if np.any(zero_len):
            inside = np.all((orig_exp >= bmin_exp) & (orig_exp <= bmax_exp), axis=2)
            hits[zero_len] = inside[zero_len]

        t_in = np.maximum(0.0, t_enter)
        t_out = np.minimum(1.0, t_exit)

        len_exp = lengths[:, np.newaxis]
        pen_dists = np.where(hits, (t_out - t_in) * len_exp, 0.0)  # (K, M)

        total_pen = np.sum(pen_dists, axis=1)  # (K,)
        los_clear = ~np.any(hits, axis=1)      # (K,)

        base_att_exp = self._cached_base_att[np.newaxis, :]   # (1, M)
        per_m_att_exp = self._cached_per_m_att[np.newaxis, :] # (1, M)
        att_matrix = np.where(hits, base_att_exp + per_m_att_exp * pen_dists, 0.0)
        total_att = np.sum(att_matrix, axis=1)  # (K,)

        return los_clear, total_pen, total_att

    def compute_repulsion_force(
        self,
        drone_pos: np.ndarray,
        influence_radius: float = 8.0,
        safe_margin: float = 1.5,
        k_rep: float = 40.0,
    ) -> np.ndarray:
        """
        Computes collective Artificial Potential Field (APF) repulsive force on drone.
        """
        p = np.asarray(drone_pos, dtype=np.float64)
        f_rep = np.zeros(3, dtype=np.float64)

        for obs in self.obstacles:
            closest_pt = obs.closest_point(p)
            diff = p - closest_pt
            dist = float(np.linalg.norm(diff))

            if dist < influence_radius:
                if dist <= safe_margin:
                    # Inside safe buffer: strong outward repulsion
                    normal = obs.surface_normal(p)
                    f_rep += normal * k_rep * 10.0
                else:
                    direction = diff / max(dist, 1e-6)
                    # Khatib repulsion
                    mag = k_rep * (1.0 / (dist - safe_margin) - 1.0 / influence_radius) * (1.0 / (dist - safe_margin) ** 2)
                    f_rep += direction * mag

        return f_rep

    def check_point_collision(self, point: np.ndarray, radius: float = 0.5) -> Tuple[bool, Optional[str]]:
        """Checks if a sphere of given radius centered at point penetrates any obstacle."""
        p = np.asarray(point, dtype=np.float64)
        for obs in self.obstacles:
            if obs.distance_to_point(p) < radius:
                return True, obs.id
        return False, None

    def to_dict_list(self) -> List[Dict[str, Any]]:
        """Serializes all obstacles for JSON telemetry / Three.js 3D viewport."""
        return [obs.to_dict() for obs in self.obstacles]


def create_default_disaster_obstacles() -> List[ObstacleAABB]:
    """
    Constructs the standard urban disaster obstacle layout:
    8 distinct collapsed structures and rubble piles strategically placed to induce
    NLoS RF occlusions between distant sectors and the GCS at [0, 0, 0].
    """
    return [
        ObstacleAABB(
            id="OBS_BLD_ALPHA",
            name="Collapsed High-Rise Alpha",
            min_pt=np.array([30.0, 40.0, 0.0]),
            max_pt=np.array([90.0, 110.0, 55.0]),
            material="reinforced_concrete",
            base_attenuation_db=22.0,
            attenuation_db_per_meter=1.5,
        ),
        ObstacleAABB(
            id="OBS_BLD_BETA",
            name="Damaged Tower Beta",
            min_pt=np.array([-120.0, 60.0, 0.0]),
            max_pt=np.array([-50.0, 130.0, 65.0]),
            material="steel_concrete",
            base_attenuation_db=24.0,
            attenuation_db_per_meter=1.8,
        ),
        ObstacleAABB(
            id="OBS_BLD_GAMMA",
            name="Residential Complex Gamma",
            min_pt=np.array([110.0, -100.0, 0.0]),
            max_pt=np.array([180.0, -30.0, 42.0]),
            material="brick_masonry",
            base_attenuation_db=20.0,
            attenuation_db_per_meter=1.2,
        ),
        ObstacleAABB(
            id="OBS_BLD_DELTA",
            name="Commercial Center Delta",
            min_pt=np.array([-180.0, -120.0, 0.0]),
            max_pt=np.array([-90.0, -50.0, 35.0]),
            material="concrete_debris",
            base_attenuation_db=22.0,
            attenuation_db_per_meter=1.4,
        ),
        ObstacleAABB(
            id="OBS_RUBBLE_NORTH",
            name="North Rubble Pile",
            min_pt=np.array([-30.0, 150.0, 0.0]),
            max_pt=np.array([40.0, 200.0, 22.0]),
            material="dense_rubble",
            base_attenuation_db=18.0,
            attenuation_db_per_meter=2.0,
        ),
        ObstacleAABB(
            id="OBS_RUBBLE_SOUTH",
            name="Overpass Collapse South",
            min_pt=np.array([-40.0, -90.0, 0.0]),
            max_pt=np.array([50.0, -40.0, 18.0]),
            material="asphalt_prestressed_concrete",
            base_attenuation_db=19.0,
            attenuation_db_per_meter=1.6,
        ),
        ObstacleAABB(
            id="OBS_HOSPITAL_WEST",
            name="Damaged Hospital Wing",
            min_pt=np.array([-210.0, 20.0, 0.0]),
            max_pt=np.array([-140.0, 80.0, 48.0]),
            material="heavy_concrete",
            base_attenuation_db=25.0,
            attenuation_db_per_meter=1.7,
        ),
        ObstacleAABB(
            id="OBS_SILO_EAST",
            name="Industrial Ruins East",
            min_pt=np.array([140.0, 80.0, 0.0]),
            max_pt=np.array([200.0, 150.0, 38.0]),
            material="metal_concrete_ruins",
            base_attenuation_db=23.0,
            attenuation_db_per_meter=1.5,
        ),
    ]
