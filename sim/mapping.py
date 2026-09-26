"""
sim/mapping.py: 3D LiDAR Perception & Occupancy Voxel Grid Mapping Engine.

Simulates onboard multi-channel rotating LiDAR sensors and OctoMap-style 3D
occupancy voxel grid mapping for autonomous UAV swarm perception and SLAM.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Dict, List, Optional, Set, Tuple
import numpy as np

from sim.obstacles import ObstacleAABB


@dataclass
class LiDARPoint:
    """A single 3D point return from a LiDAR scan ray."""
    x: float
    y: float
    z: float
    range_m: float
    intensity: float  # [0.0, 1.0] reflection intensity
    obstacle_id: Optional[str] = None


@dataclass
class LiDARScan:
    """A full spherical / cylindrical sweep of LiDAR points at a given timestamp."""
    timestamp: float
    drone_id: str
    sensor_origin: np.ndarray  # [x, y, z] in world frame
    points: List[LiDARPoint] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": round(float(self.timestamp), 3),
            "drone_id": self.drone_id,
            "origin": [round(float(c), 2) for c in self.sensor_origin],
            "num_points": len(self.points),
            # Downsampled/compact point list for fast WebSocket transport [x, y, z, intensity]
            "points": [
                [
                    round(p.x, 2),
                    round(p.y, 2),
                    round(p.z, 2),
                    round(p.intensity, 2)
                ]
                for p in self.points
            ],
        }


class LiDARScanner:
    """
    Simulates a multi-beam rotating 3D LiDAR sensor (e.g., Velodyne VLP-16 / Ouster OS1 equivalent).
    
    Features:
    - Configurable horizontal & vertical FOV with azimuth/elevation steps.
    - Ray-casting against 3D AABB building obstacles and ground terrain.
    - Gaussian range noise model N(0, sigma^2).
    - Ray incident angle reflection intensity modeling.
    """

    def __init__(
        self,
        max_range_m: float = 65.0,
        min_range_m: float = 0.5,
        horizontal_fov_deg: float = 360.0,
        horizontal_resolution_deg: float = 15.0,  # 24 azimuth beams per ring
        vertical_fov_deg: Tuple[float, float] = (-30.0, 15.0),  # -30 deg downward to +15 deg upward
        vertical_channels: int = 8,  # 8 elevation rings
        range_noise_std_m: float = 0.03,  # 3cm range measurement noise
    ) -> None:
        self.max_range = float(max_range_m)
        self.min_range = float(min_range_m)
        self.horizontal_fov_deg = float(horizontal_fov_deg)
        self.horizontal_resolution_deg = float(horizontal_resolution_deg)
        self.vertical_fov_min_deg, self.vertical_fov_max_deg = vertical_fov_deg
        self.vertical_channels = int(vertical_channels)
        self.range_noise_std = float(range_noise_std_m)

        # Precompute ray unit vectors in sensor body frame
        self._body_ray_dirs = self._precompute_ray_directions()

    def _precompute_ray_directions(self) -> np.ndarray:
        """Precomputes unit ray direction vectors in sensor frame [N_rays, 3]."""
        num_azimuth = int(round(self.horizontal_fov_deg / self.horizontal_resolution_deg))
        azimuths = np.linspace(-np.pi, np.pi, num_azimuth, endpoint=False)
        elevations = np.linspace(
            np.radians(self.vertical_fov_min_deg),
            np.radians(self.vertical_fov_max_deg),
            self.vertical_channels
        )

        dirs = []
        for el in elevations:
            cos_el = np.cos(el)
            sin_el = np.sin(el)
            for az in azimuths:
                dx = cos_el * np.cos(az)
                dy = cos_el * np.sin(az)
                dz = sin_el
                dirs.append([dx, dy, dz])

        return np.asarray(dirs, dtype=np.float64)

    def scan(
        self,
        drone_id: str,
        position: np.ndarray,
        attitude: np.ndarray,  # [roll, pitch, yaw] radians
        obstacles: List[ObstacleAABB],
        sim_time: float = 0.0,
    ) -> LiDARScan:
        """
        Executes a 3D LiDAR scan sweep from position with given attitude.
        Rays are tested against all obstacles and ground plane (z = 0).
        """
        pos = np.asarray(position, dtype=np.float64)
        roll, pitch, yaw = attitude[0], attitude[1], attitude[2]

        # Rotation matrix from body to world frame
        # Z-Y-X Euler angle rotation
        cz, sz = np.cos(yaw), np.sin(yaw)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cx, sx = np.cos(roll), np.sin(roll)

        R_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
        R_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
        R_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
        R_body_to_world = R_z @ R_y @ R_x

        # Rotate ray directions into world coordinates
        world_ray_dirs = self._body_ray_dirs @ R_body_to_world.T

        scan = LiDARScan(timestamp=sim_time, drone_id=drone_id, sensor_origin=pos)

        # Broadphase filter: only test obstacles within max_range + bounding radius
        active_obstacles: List[ObstacleAABB] = []
        for obs in obstacles:
            if obs.distance_to_point(pos) <= self.max_range:
                active_obstacles.append(obs)

        for ray_dir in world_ray_dirs:
            closest_dist = self.max_range
            hit_obstacle_id: Optional[str] = None
            hit_normal = np.array([0.0, 0.0, 1.0])

            # 1. Ground plane intersection (z = 0)
            if ray_dir[2] < -1e-5:
                t_ground = -pos[2] / ray_dir[2]
                if self.min_range <= t_ground < closest_dist:
                    closest_dist = t_ground
                    hit_obstacle_id = "GROUND"
                    hit_normal = np.array([0.0, 0.0, 1.0])

            # 2. Obstacles intersection (Ray-AABB slab test)
            p_dst = pos + ray_dir * closest_dist
            for obs in active_obstacles:
                res = obs.intersect_ray_segment(pos, p_dst)
                if res.hit and res.t_enter >= 0.0:
                    dist = res.t_enter * closest_dist
                    if self.min_range <= dist < closest_dist:
                        closest_dist = dist
                        hit_obstacle_id = obs.id
                        if res.entry_point is not None:
                            hit_normal = obs.surface_normal(res.entry_point)
                            p_dst = pos + ray_dir * closest_dist

            # Record hit point if within max range
            if closest_dist < self.max_range and hit_obstacle_id is not None:
                # Add measurement noise
                noise = np.random.normal(0.0, self.range_noise_std)
                measured_dist = max(self.min_range, closest_dist + noise)
                hit_pos = pos + ray_dir * measured_dist

                # Reflection intensity based on Lambertian cosine of incident angle
                cos_incidence = max(0.1, float(abs(np.dot(-ray_dir, hit_normal))))
                # Range decay factor (1 / r^2 normalized)
                range_factor = max(0.2, 1.0 - (measured_dist / self.max_range) * 0.5)
                intensity = min(1.0, cos_incidence * range_factor)

                scan.points.append(
                    LiDARPoint(
                        x=float(hit_pos[0]),
                        y=float(hit_pos[1]),
                        z=float(hit_pos[2]),
                        range_m=float(measured_dist),
                        intensity=float(intensity),
                        obstacle_id=hit_obstacle_id,
                    )
                )

        return scan


@dataclass
class VoxelNode:
    """A single 3D occupancy voxel cell."""
    ix: int
    iy: int
    iz: int
    center: np.ndarray
    log_odds: float = 0.0  # L(m) log-odds of occupancy

    @property
    def occupancy_prob(self) -> float:
        """P(m) = 1 / (1 + exp(-L))"""
        return 1.0 / (1.0 + math.exp(-max(-20.0, min(20.0, self.log_odds))))

    @property
    def is_occupied(self) -> bool:
        """True if probability exceeds standard occupancy threshold (e.g. 0.65)."""
        return self.occupancy_prob >= 0.65

    @property
    def is_free(self) -> bool:
        """True if probability is below free space threshold (e.g. 0.35)."""
        return self.occupancy_prob <= 0.35


class OccupancyGridMap3D:
    """
    3D Occupancy Voxel Grid based on OctoMap log-odds formulation.
    
    Provides:
    - Sparse spatial hashing for infinite/large disaster zones without memory bloat.
    - Ray-marching free-space and hit updates:
        L_new = clamp(L_prev + l_occ, L_min, L_max) for hit voxels
        L_new = clamp(L_prev + l_free, L_min, L_max) for traversed ray voxels
    - Coverage calculation: total volume surveyed and obstacle surface volume.
    """

    def __init__(
        self,
        voxel_size_m: float = 4.0,  # 4m resolution for disaster-scale map
        l_occ: float = 0.85,        # Log-odds increment for hit
        l_free: float = -0.35,      # Log-odds decrement for free ray traversal
        l_min: float = -2.5,        # Clamping lower bound
        l_max: float = 3.5,         # Clamping upper bound
        bounds_x: Tuple[float, float] = (-200.0, 200.0),
        bounds_y: Tuple[float, float] = (-200.0, 200.0),
        bounds_z: Tuple[float, float] = (0.0, 100.0),
    ) -> None:
        self.voxel_size = float(voxel_size_m)
        self.inv_voxel_size = 1.0 / self.voxel_size
        self.l_occ = float(l_occ)
        self.l_free = float(l_free)
        self.l_min = float(l_min)
        self.l_max = float(l_max)
        self.bounds_x = bounds_x
        self.bounds_y = bounds_y
        self.bounds_z = bounds_z

        # Sparse dictionary of active voxels keyed by (ix, iy, iz)
        self.voxels: Dict[Tuple[int, int, int], VoxelNode] = {}
        self.total_surveyed_points: int = 0

    def world_to_grid(self, pt: np.ndarray) -> Tuple[int, int, int]:
        """Maps 3D world coordinates [x, y, z] to discrete integer grid indices."""
        ix = int(math.floor(pt[0] * self.inv_voxel_size))
        iy = int(math.floor(pt[1] * self.inv_voxel_size))
        iz = int(math.floor(pt[2] * self.inv_voxel_size))
        return (ix, iy, iz)

    def grid_to_world(self, key: Tuple[int, int, int]) -> np.ndarray:
        """Returns the continuous 3D centroid of a voxel cell."""
        ix, iy, iz = key
        return np.array([
            (ix + 0.5) * self.voxel_size,
            (iy + 0.5) * self.voxel_size,
            (iz + 0.5) * self.voxel_size,
        ], dtype=np.float64)

    def is_in_bounds(self, pt: np.ndarray) -> bool:
        """Checks if world point is within operational volume."""
        return (
            self.bounds_x[0] <= pt[0] <= self.bounds_x[1]
            and self.bounds_y[0] <= pt[1] <= self.bounds_y[1]
            and self.bounds_z[0] <= pt[2] <= self.bounds_z[1]
        )

    def insert_scan(self, scan: LiDARScan, max_traversal_steps: int = 15) -> None:
        """
        Integrates a LiDAR scan into the 3D occupancy map using log-odds updates.
        Free space voxels along the ray are decremented; hit endpoints are incremented.
        """
        origin = scan.sensor_origin
        if not self.is_in_bounds(origin):
            return

        self.total_surveyed_points += len(scan.points)

        for pt in scan.points:
            hit_world = np.array([pt.x, pt.y, pt.z], dtype=np.float64)
            if not self.is_in_bounds(hit_world):
                continue

            # 1. Update Hit Voxel
            hit_key = self.world_to_grid(hit_world)
            if hit_key not in self.voxels:
                self.voxels[hit_key] = VoxelNode(
                    ix=hit_key[0],
                    iy=hit_key[1],
                    iz=hit_key[2],
                    center=self.grid_to_world(hit_key),
                    log_odds=0.0
                )
            voxel = self.voxels[hit_key]
            voxel.log_odds = min(self.l_max, max(self.l_min, voxel.log_odds + self.l_occ))

            # 2. Sample Free-Space Voxels along ray (Bresenham / coarse ray-march)
            ray_vec = hit_world - origin
            dist = float(np.linalg.norm(ray_vec))
            if dist > self.voxel_size:
                step_size = self.voxel_size * 1.5
                num_steps = min(max_traversal_steps, int(dist / step_size))
                for s in range(1, num_steps):
                    sample_pt = origin + ray_vec * (s * step_size / dist)
                    free_key = self.world_to_grid(sample_pt)
                    if free_key == hit_key:
                        break
                    if free_key not in self.voxels:
                        self.voxels[free_key] = VoxelNode(
                            ix=free_key[0],
                            iy=free_key[1],
                            iz=free_key[2],
                            center=self.grid_to_world(free_key),
                            log_odds=0.0
                        )
                    free_vox = self.voxels[free_key]
                    free_vox.log_odds = min(self.l_max, max(self.l_min, free_vox.log_odds + self.l_free))

    def get_occupied_voxels(self, max_count: int = 1500) -> List[Dict[str, Any]]:
        """
        Returns list of occupied voxels formatted for 3D Three.js client visualization.
        """
        occupied = []
        for key, v in self.voxels.items():
            if v.is_occupied:
                occupied.append({
                    "pos": [round(float(c), 2) for c in v.center],
                    "prob": round(float(v.occupancy_prob), 2),
                    "size": self.voxel_size,
                })
                if len(occupied) >= max_count:
                    break
        return occupied

    def compute_metrics(self) -> Dict[str, float]:
        """Calculates quantitative SLAM mapping metrics for analytics and reporting."""
        occupied_count = sum(1 for v in self.voxels.values() if v.is_occupied)
        free_count = sum(1 for v in self.voxels.values() if v.is_free)
        total_cells = len(self.voxels)

        voxel_vol = self.voxel_size ** 3
        mapped_volume_m3 = total_cells * voxel_vol
        obstacle_volume_m3 = occupied_count * voxel_vol

        # Theater boundary volume
        total_vol = (
            (self.bounds_x[1] - self.bounds_x[0])
            * (self.bounds_y[1] - self.bounds_y[0])
            * (self.bounds_z[1] - self.bounds_z[0])
        )
        coverage_pct = min(100.0, (mapped_volume_m3 / max(1.0, total_vol)) * 100.0 * 20.0)  # scaled explored ratio

        return {
            "occupied_voxels": float(occupied_count),
            "free_voxels": float(free_count),
            "total_mapped_cells": float(total_cells),
            "mapped_volume_m3": round(float(mapped_volume_m3), 1),
            "obstacle_volume_m3": round(float(obstacle_volume_m3), 1),
            "coverage_pct": round(float(coverage_pct), 1),
        }
