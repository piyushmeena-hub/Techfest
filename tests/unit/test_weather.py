"""
tests/unit/test_weather.py: Unit tests for Dryden Atmospheric Turbulence & Wind Shear Engine.
"""

import numpy as np
import pytest

from sim.obstacles import ObstacleAABB
from sim.weather import DrydenTurbulenceModel, WindConfig


def test_wind_boundary_layer_shear():
    """Verify altitude-dependent boundary layer wind shear increases with height."""
    config = WindConfig(mean_speed_mps=5.0, direction_deg=0.0, ref_altitude_m=10.0, shear_exponent=0.143)
    model = DrydenTurbulenceModel(config)

    v_10m = model.get_mean_wind(10.0)
    v_50m = model.get_mean_wind(50.0)
    v_100m = model.get_mean_wind(100.0)

    spd_10m = float(np.linalg.norm(v_10m))
    spd_50m = float(np.linalg.norm(v_50m))
    spd_100m = float(np.linalg.norm(v_100m))

    assert spd_10m == pytest.approx(5.0, abs=0.1)
    assert spd_50m > spd_10m
    assert spd_100m > spd_50m


def test_dryden_turbulence_bounded_and_finite():
    """Verify Dryden turbulence step outputs are finite and stay within physical bounds."""
    config = WindConfig(mean_speed_mps=6.0, turbulence_intensity="MODERATE")
    model = DrydenTurbulenceModel(config)

    dt = 0.05
    for _ in range(200):
        turb = model.step_turbulence(altitude_m=35.0, dt=dt)
        assert turb.shape == (3,)
        assert np.all(np.isfinite(turb))
        # 3-sigma turbulence shouldn't exceed 15 m/s under moderate conditions
        assert np.linalg.norm(turb) < 15.0


def test_building_wake_leeward_downdraft():
    """Verify building downwind wake creates downward velocity suction."""
    config = WindConfig(mean_speed_mps=8.0, direction_deg=180.0) # Wind from South blowing North (+Y)
    model = DrydenTurbulenceModel(config)

    building = ObstacleAABB(
        id="TOWER",
        name="Tower",
        min_pt=np.array([-20.0, -20.0, 0.0]),
        max_pt=np.array([20.0, 20.0, 40.0]),
    )

    mean_wind = np.array([0.0, 8.0, 0.0]) # Blowing North (+Y)

    # Position 1: Upwind (y = -30m) -> no wake downdraft
    p_upwind = np.array([0.0, -30.0, 20.0])
    wake_upwind = model.compute_building_wake(p_upwind, mean_wind, [building])
    assert wake_upwind[2] == 0.0

    # Position 2: Leeward downwind (y = +35m) -> downward suction (w < 0)
    p_downwind = np.array([0.0, 35.0, 20.0])
    wake_downwind = model.compute_building_wake(p_downwind, mean_wind, [building])
    assert wake_downwind[2] < 0.0  # Downdraft present!


def test_composite_wind_field():
    """Verify total composite wind vector is clean and valid."""
    model = DrydenTurbulenceModel()
    pos = [15.0, 40.0, 30.0]
    w = model.get_wind_at(pos, dt=0.05)
    assert w.shape == (3,)
    assert np.all(np.isfinite(w))
