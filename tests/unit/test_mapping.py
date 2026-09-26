"""
tests/unit/test_mapping.py: Unit tests for 3D LiDAR perception and Occupancy Voxel Grid mapping.
"""

from __future__ import annotations

import numpy as np
import pytest

from sim.mapping import LiDARScanner, OccupancyGridMap3D, LiDARPoint, LiDARScan
from sim.obstacles import ObstacleAABB


@pytest.fixture
def sample_obstacles():
    """Create a sample building obstacle for LiDAR testing."""
    return [
        ObstacleAABB(
            id="BLD_TEST",
            name="Test Tower",
            min_pt=np.array([20.0, 20.0, 0.0]),
            max_pt=np.array([40.0, 40.0, 30.0]),
            material="reinforced_concrete"
        )
    ]


def test_lidar_scanner_initialization():
    """Verify LiDAR scanner initializes with valid ray precomputations."""
    scanner = LiDARScanner(
        max_range_m=50.0,
        horizontal_fov_deg=360.0,
        horizontal_resolution_deg=30.0,
        vertical_channels=4
    )
    assert scanner.max_range == 50.0
    assert len(scanner._body_ray_dirs) == 12 * 4  # 12 azimuth * 4 vertical
    # Check that ray direction vectors are unit length
    norms = np.linalg.norm(scanner._body_ray_dirs, axis=1)
    np.testing.assert_allclose(norms, 1.0, atol=1e-5)


def test_lidar_scanner_raycast_hits(sample_obstacles):
    """Verify LiDAR scanner raycasts hit obstacles and ground correctly."""
    scanner = LiDARScanner(
        max_range_m=60.0,
        horizontal_fov_deg=360.0,
        horizontal_resolution_deg=15.0,
        vertical_channels=8
    )

    # Place drone at [10, 30, 15] looking directly at the building at [20, 20, 0]..[40, 40, 30]
    pos = np.array([10.0, 30.0, 15.0])
    attitude = np.array([0.0, 0.0, 0.0])

    scan = scanner.scan("UAV_1", pos, attitude, sample_obstacles, sim_time=1.0)

    assert isinstance(scan, LiDARScan)
    assert scan.drone_id == "UAV_1"
    assert len(scan.points) > 0

    # Ensure some points hit the obstacle
    obs_hits = [p for p in scan.points if p.obstacle_id == "BLD_TEST"]
    assert len(obs_hits) > 0

    # Ensure range is within bounds
    for p in scan.points:
        assert scanner.min_range <= p.range_m <= scanner.max_range
        assert 0.0 <= p.intensity <= 1.0

    # Check to_dict() serialization
    data = scan.to_dict()
    assert data["drone_id"] == "UAV_1"
    assert "points" in data
    assert len(data["points"]) == len(scan.points)


def test_occupancy_grid_map_log_odds_updates(sample_obstacles):
    """Verify 3D occupancy voxel grid updates log-odds and computes probabilities."""
    grid = OccupancyGridMap3D(voxel_size_m=4.0)

    # Create synthetic scan hitting obstacle at [25, 25, 10]
    scan = LiDARScan(
        timestamp=0.5,
        drone_id="UAV_1",
        sensor_origin=np.array([0.0, 0.0, 10.0]),
        points=[
            LiDARPoint(x=25.0, y=25.0, z=10.0, range_m=35.35, intensity=0.9, obstacle_id="BLD_TEST")
        ]
    )

    grid.insert_scan(scan)

    # Check that hit voxel is recorded and probability increased
    hit_key = grid.world_to_grid(np.array([25.0, 25.0, 10.0]))
    assert hit_key in grid.voxels
    assert grid.voxels[hit_key].log_odds > 0.0
    assert grid.voxels[hit_key].occupancy_prob > 0.5
    assert grid.voxels[hit_key].is_occupied

    # Check metrics computation
    metrics = grid.compute_metrics()
    assert metrics["occupied_voxels"] >= 1.0
    assert metrics["mapped_volume_m3"] > 0.0

    # Check get_occupied_voxels serialization
    occ_list = grid.get_occupied_voxels()
    assert len(occ_list) >= 1
    assert "pos" in occ_list[0]
    assert "prob" in occ_list[0]
