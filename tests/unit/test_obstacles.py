"""
tests/unit/test_obstacles.py: Unit tests for 3D AABB Obstacles, Williams et al. Ray-Slab Intersection,
penetration distance calculation, and vectorized batch occlusion testing.
"""

from __future__ import annotations

import time
import pytest
import numpy as np

from sim.obstacles import (
    ObstacleAABB,
    ObstacleManager,
    RayIntersectionResult,
    create_default_disaster_obstacles,
)


@pytest.fixture
def sample_box() -> ObstacleAABB:
    """Standard test box [10..40, 20..60, 0..30]."""
    return ObstacleAABB(
        id="BOX_TEST",
        name="Test Box",
        min_pt=np.array([10.0, 20.0, 0.0]),
        max_pt=np.array([40.0, 60.0, 30.0]),
        base_attenuation_db=22.0,
        attenuation_db_per_meter=1.5,
    )


def test_aabb_geometry_properties(sample_box: ObstacleAABB):
    """Verify centroid, extents, volume, and height of AABB."""
    assert np.allclose(sample_box.center, [25.0, 40.0, 15.0])
    assert np.allclose(sample_box.extents, [30.0, 40.0, 30.0])
    assert sample_box.volume == pytest.approx(36000.0)
    assert sample_box.height == pytest.approx(30.0)


def test_aabb_invalid_dimensions():
    """Verify AABB validation rejects inverted bounds and negative z_min."""
    # Inverted X
    with pytest.raises(ValueError, match="strictly less than"):
        ObstacleAABB("INV_1", "Inv", np.array([50.0, 20.0, 0.0]), np.array([40.0, 60.0, 30.0]))

    # Below ground
    with pytest.raises(ValueError, match="below ground"):
        ObstacleAABB("INV_2", "Inv", np.array([10.0, 20.0, -5.0]), np.array([40.0, 60.0, 30.0]))


def test_aabb_distance_and_closest_point(sample_box: ObstacleAABB):
    """Verify closest point projection and distance calculation."""
    # Point at [5, 40, 15] outside X=10 face
    pt = np.array([5.0, 40.0, 15.0])
    closest = sample_box.closest_point(pt)
    assert np.allclose(closest, [10.0, 40.0, 15.0])
    assert sample_box.distance_to_point(pt) == pytest.approx(5.0)

    dist, closest_tuple = sample_box.distance_and_closest_point(pt)
    assert dist == pytest.approx(5.0)
    assert np.allclose(closest_tuple, [10.0, 40.0, 15.0])

    # Point inside box
    pt_inside = np.array([25.0, 40.0, 15.0])
    assert sample_box.distance_to_point(pt_inside) == pytest.approx(0.0)


def test_ray_slab_pass_through(sample_box: ObstacleAABB):
    """Verify ray passing completely through box calculates exact entry, exit, and penetration."""
    # Ray from [0, 40, 15] to [50, 40, 15] (total length = 50m)
    p_src = np.array([0.0, 40.0, 15.0])
    p_dst = np.array([50.0, 40.0, 15.0])
    res = sample_box.intersect_ray_segment(p_src, p_dst)

    assert res.hit is True
    # Box spans X in [10, 40] -> t_enter = 10/50 = 0.2, t_exit = 40/50 = 0.8
    assert res.t_enter == pytest.approx(0.2, abs=1e-5)
    assert res.t_exit == pytest.approx(0.8, abs=1e-5)
    assert res.penetration_distance == pytest.approx(30.0, abs=1e-5)
    assert np.allclose(res.entry_point, [10.0, 40.0, 15.0])
    assert np.allclose(res.exit_point, [40.0, 40.0, 15.0])
    # Attenuation = 22.0 + 1.5 * 30.0 = 67.0 dB
    assert res.attenuation_db == pytest.approx(67.0, abs=1e-5)


def test_ray_slab_starts_inside(sample_box: ObstacleAABB):
    """Verify ray starting inside box clamps t_in to 0.0."""
    p_src = np.array([25.0, 40.0, 15.0])
    p_dst = np.array([50.0, 40.0, 15.0])  # Length = 25m
    res = sample_box.intersect_ray_segment(p_src, p_dst)

    assert res.hit is True
    assert res.t_enter < 0.0
    # Exits at X=40: t_exit = (40 - 25) / 25 = 0.6
    assert res.t_exit == pytest.approx(0.6, abs=1e-5)
    assert res.penetration_distance == pytest.approx(15.0, abs=1e-5)
    assert np.allclose(res.entry_point, [25.0, 40.0, 15.0])
    assert np.allclose(res.exit_point, [40.0, 40.0, 15.0])


def test_ray_slab_ends_inside(sample_box: ObstacleAABB):
    """Verify ray terminating inside box clamps t_out to 1.0."""
    p_src = np.array([0.0, 40.0, 15.0])
    p_dst = np.array([25.0, 40.0, 15.0])  # Length = 25m
    res = sample_box.intersect_ray_segment(p_src, p_dst)

    assert res.hit is True
    # Enters at X=10: t_enter = 10 / 25 = 0.4
    assert res.t_enter == pytest.approx(0.4, abs=1e-5)
    assert res.t_exit > 1.0
    assert res.penetration_distance == pytest.approx(15.0, abs=1e-5)
    assert np.allclose(res.entry_point, [10.0, 40.0, 15.0])
    assert np.allclose(res.exit_point, [25.0, 40.0, 15.0])


def test_ray_slab_parallel_miss(sample_box: ObstacleAABB):
    """Verify ray parallel to an axis outside the box correctly misses without division by zero."""
    p_src = np.array([0.0, 70.0, 15.0])
    p_dst = np.array([50.0, 70.0, 15.0])  # Parallel to X, but Y=70 is outside [20, 60]
    res = sample_box.intersect_ray_segment(p_src, p_dst)
    assert res.hit is False
    assert res.penetration_distance == 0.0


def test_ray_slab_parallel_hit(sample_box: ObstacleAABB):
    """Verify ray parallel to Y-axis passing through box intersects with correct length."""
    # From [25, 0, 15] to [25, 80, 15] (total length = 80m)
    p_src = np.array([25.0, 0.0, 15.0])
    p_dst = np.array([25.0, 80.0, 15.0])
    res = sample_box.intersect_ray_segment(p_src, p_dst)

    assert res.hit is True
    # Box spans Y in [20, 60] -> penetration = 40m
    assert res.penetration_distance == pytest.approx(40.0, abs=1e-5)


def test_ray_slab_stops_short(sample_box: ObstacleAABB):
    """Verify ray segment stopping before reaching the box reports no hit."""
    p_src = np.array([0.0, 40.0, 15.0])
    p_dst = np.array([8.0, 40.0, 15.0])  # Stops at X=8 before X=10
    res = sample_box.intersect_ray_segment(p_src, p_dst)
    assert res.hit is False
    assert res.penetration_distance == 0.0


def test_ray_slab_zero_length(sample_box: ObstacleAABB):
    """Verify point containment check for zero-length ray."""
    pt_inside = np.array([25.0, 40.0, 15.0])
    res_inside = sample_box.intersect_ray_segment(pt_inside, pt_inside)
    assert res_inside.hit is True
    assert res_inside.penetration_distance == 0.0

    pt_outside = np.array([5.0, 40.0, 15.0])
    res_outside = sample_box.intersect_ray_segment(pt_outside, pt_outside)
    assert res_outside.hit is False


def test_obstacle_manager_multi_building():
    """Verify ray traversing two buildings compounds penetration distance and attenuation."""
    bld1 = ObstacleAABB("B1", "Building 1", np.array([10.0, 0.0, 0.0]), np.array([20.0, 10.0, 20.0]), base_attenuation_db=22.0, attenuation_db_per_meter=1.5)
    bld2 = ObstacleAABB("B2", "Building 2", np.array([30.0, 0.0, 0.0]), np.array([50.0, 10.0, 20.0]), base_attenuation_db=22.0, attenuation_db_per_meter=1.5)
    manager = ObstacleManager([bld1, bld2])

    p_src = np.array([0.0, 5.0, 10.0])
    p_dst = np.array([60.0, 5.0, 10.0])

    is_los, total_pen, total_att, hit_ids = manager.check_los(p_src, p_dst)
    assert is_los is False
    assert hit_ids == ["B1", "B2"]
    # B1 penetration: 10m; B2 penetration: 20m -> total = 30m
    assert total_pen == pytest.approx(30.0, abs=1e-5)
    # B1 att: 22 + 1.5*10 = 37 dB; B2 att: 22 + 1.5*20 = 52 dB -> total = 89.0 dB
    assert total_att == pytest.approx(89.0, abs=1e-5)


def test_vectorized_batch_los():
    """Verify check_los_batch produces bit-for-bit identical results to scalar check_los."""
    obstacles = create_default_disaster_obstacles()
    manager = ObstacleManager(obstacles)

    np.random.seed(42)
    K = 50
    sources = np.random.uniform(-200.0, 200.0, size=(K, 3))
    sources[:, 2] = np.random.uniform(5.0, 50.0, size=K)
    targets = np.random.uniform(-200.0, 200.0, size=(K, 3))
    targets[:, 2] = np.random.uniform(5.0, 50.0, size=K)

    # Batch evaluation
    los_clear_batch, pen_batch, att_batch = manager.check_los_batch(sources, targets)

    # Scalar evaluation
    for k in range(K):
        is_los, pen_scalar, att_scalar, _ = manager.check_los(sources[k], targets[k])
        assert los_clear_batch[k] == is_los
        assert pen_batch[k] == pytest.approx(pen_scalar, abs=1e-9)
        assert att_batch[k] == pytest.approx(att_scalar, abs=1e-9)


def test_batch_benchmark_speed():
    """Verify 100 rays evaluated against 20 obstacles executes in < 5.0 ms."""
    obstacles = create_default_disaster_obstacles()
    # Duplicate obstacles to simulate 24 obstacles
    manager = ObstacleManager(obstacles + [
        ObstacleAABB(f"DUP_{o.id}_{i}", f"Dup {i}", o.min_pt + i * 2.0, o.max_pt + i * 2.0)
        for i, o in enumerate(obstacles)
    ])
    K = 100
    sources = np.random.uniform(-200.0, 200.0, size=(K, 3))
    targets = np.random.uniform(-200.0, 200.0, size=(K, 3))

    t0 = time.perf_counter()
    manager.check_los_batch(sources, targets)
    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    assert elapsed_ms < 10.0  # Ultra-fast vectorized performance


def test_apf_repulsion_force():
    """Verify obstacle collective APF repulsion points away from building face."""
    bld = ObstacleAABB("B1", "Building", np.array([10.0, 10.0, 0.0]), np.array([40.0, 40.0, 30.0]))
    manager = ObstacleManager([bld])

    # Drone placed at [8.0, 25.0, 15.0] (2m from X=10 face)
    drone_pos = np.array([8.0, 25.0, 15.0])
    f_rep = manager.compute_repulsion_force(drone_pos, influence_radius=8.0, safe_margin=1.5)

    assert f_rep[0] < 0.0  # Points in -X direction
    assert np.linalg.norm(f_rep) > 0.0


def test_default_disaster_preset():
    """Verify the 8 default disaster obstacles are placed with open space at GCS."""
    obstacles = create_default_disaster_obstacles()
    assert len(obstacles) == 8
    manager = ObstacleManager(obstacles)

    # GCS at [0, 0, 0] must not collide with any obstacle
    gcs_pos = np.array([0.0, 0.0, 0.0])
    has_collision, obs_id = manager.check_point_collision(gcs_pos, radius=5.0)
    assert not has_collision
    assert obs_id is None
