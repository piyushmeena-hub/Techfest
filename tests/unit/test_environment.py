"""
tests/unit/test_environment.py: Unit tests for 3D Disaster Environment and Altitude Corridors.
"""

from __future__ import annotations

import json
import pytest
import numpy as np

from sim.environment import AltitudeCorridor, DisasterEnvironment, EnvironmentConfig


def test_environment_default_bounds():
    """Verify default 500x500x120m operational volume dimensions."""
    env = DisasterEnvironment()
    assert env.width_x == 500.0
    assert env.length_y == 500.0
    assert env.height_z == 120.0
    assert np.allclose(env.bounds_min, [-250.0, -250.0, 0.0])
    assert np.allclose(env.bounds_max, [250.0, 250.0, 120.0])
    assert env.bounds_x == (-250.0, 250.0)
    assert env.bounds_y == (-250.0, 250.0)
    assert env.bounds_z == (0.0, 120.0)


def test_environment_gcs_placement():
    """Verify GCS base station placement at origin [0,0,0] and RF antenna mast height."""
    env = DisasterEnvironment()
    assert np.allclose(env.gcs_position, [0.0, 0.0, 0.0])
    assert np.allclose(env.gcs_rf_position, [0.0, 0.0, 2.5])


def test_is_within_bounds_nominal():
    """Verify points strictly within disaster volume evaluate to True."""
    env = DisasterEnvironment()
    inside_points = [
        np.array([0.0, 0.0, 50.0]),
        np.array([-200.0, 200.0, 10.0]),
        np.array([240.0, -240.0, 110.0]),
        np.array([-250.0, 0.0, 0.0]),
        np.array([250.0, 250.0, 120.0]),
    ]
    for pt in inside_points:
        assert env.is_within_bounds(pt) is True


def test_is_within_bounds_breach():
    """Verify points outside operational volume evaluate to False."""
    env = DisasterEnvironment()
    outside_points = [
        np.array([251.0, 0.0, 50.0]),
        np.array([0.0, -255.0, 50.0]),
        np.array([0.0, 0.0, 125.0]),
        np.array([0.0, 0.0, -1.0]),
        np.array([-300.0, -300.0, 50.0]),
    ]
    for pt in outside_points:
        assert env.is_within_bounds(pt) is False


def test_is_within_bounds_with_margin():
    """Verify inward safety margin correctly contracts the allowable bounding box."""
    env = DisasterEnvironment()
    pt = np.array([245.0, 0.0, 50.0])
    assert env.is_within_bounds(pt, margin=0.0) is True
    # Margin of 10m contracts X_max to 240m
    assert env.is_within_bounds(pt, margin=10.0) is False


def test_clamp_to_bounds():
    """Verify out-of-bounds coordinates are clamped to boundary faces."""
    env = DisasterEnvironment()
    pt = np.array([300.0, -350.0, 150.0])
    clamped = env.clamp_to_bounds(pt)
    assert np.allclose(clamped, [250.0, -250.0, 120.0])

    # Clamping with margin
    clamped_margin = env.clamp_to_bounds(pt, margin=10.0)
    assert np.allclose(clamped_margin, [240.0, -240.0, 110.0])


def test_validate_position_errors():
    """Verify validate_position raises ValueError on NaNs, Infs, or boundary breaches."""
    env = DisasterEnvironment()
    # NaN
    with pytest.raises(ValueError, match="non-finite"):
        env.validate_position(np.array([0.0, np.nan, 50.0]))

    # Inf
    with pytest.raises(ValueError, match="non-finite"):
        env.validate_position(np.array([0.0, np.inf, 50.0]))

    # Out of bounds
    with pytest.raises(ValueError, match="outside bounds"):
        env.validate_position(np.array([300.0, 0.0, 50.0]))


def test_distance_to_boundary():
    """Verify exact calculation of distance to nearest outer boundary wall."""
    env = DisasterEnvironment()
    # Point at [240, 100, 50]: distance to X_max (250) is 10.0m
    pt = np.array([240.0, 100.0, 50.0])
    dist = env.distance_to_boundary(pt)
    assert dist == pytest.approx(10.0, abs=1e-5)

    # Outside point: negative distance
    pt_out = np.array([255.0, 100.0, 50.0])
    dist_out = env.distance_to_boundary(pt_out)
    assert dist_out == pytest.approx(-5.0, abs=1e-5)


def test_altitude_corridors_classification():
    """Verify classification of altitude AGL into operational tiers."""
    env = DisasterEnvironment()
    assert env.get_altitude_corridor(10.0) == AltitudeCorridor.GROUND_LAUNCH_LAND
    assert env.get_altitude_corridor(35.0) == AltitudeCorridor.POI_SURVEY
    assert env.get_altitude_corridor(55.0) == AltitudeCorridor.TRANSIT
    assert env.get_altitude_corridor(80.0) == AltitudeCorridor.RELAY_MESH
    assert env.get_altitude_corridor(48.0) == AltitudeCorridor.BUFFER_ZONE
    assert env.get_altitude_corridor(125.0) == AltitudeCorridor.ABOVE_CEILING
    assert env.get_altitude_corridor(-2.0) == AltitudeCorridor.BELOW_GROUND


def test_environment_serialization():
    """Verify environment serialization produces valid JSON."""
    env = DisasterEnvironment()
    d = env.to_dict()
    assert d["x_bounds"] == [-250.0, 250.0]
    assert d["y_bounds"] == [-250.0, 250.0]
    assert d["z_bounds"] == [0.0, 120.0]
    assert d["dimensions_m"] == [500.0, 500.0, 120.0]
    assert d["gcs_position"] == [0.0, 0.0, 0.0]
    assert d["gcs_rf_position"] == [0.0, 0.0, 2.5]
    json_str = json.dumps(d)
    assert "dimensions_m" in json_str
