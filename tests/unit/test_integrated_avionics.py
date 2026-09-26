"""
tests/unit/test_integrated_avionics.py: Unit tests for Drone integrated SensorSuite, EKF state estimation,
and atmospheric wind dynamics.
"""

from __future__ import annotations

import math
import pytest
import numpy as np

from sim.core import SimulationConfig, SwarmSimulationCore
from sim.drone import Drone
from sim.types import DroneLimits, DroneRole, FlightMode
from sim.weather import WindConfig


def test_drone_ekf_tracking_accuracy():
    """Verify that DroneEKF successfully tracks true drone position during waypoint navigation."""
    drone = Drone("UAV_EKF_TEST", initial_position=np.array([0.0, 0.0, 20.0]))
    drone.set_flight_mode(FlightMode.TRANSIT)
    drone.set_target_waypoint(np.array([40.0, 40.0, 25.0]))

    dt = 0.05
    for _ in range(100):  # 5.0 seconds
        drone.step(dt)

    true_pos = drone.position
    est_pos = drone.ekf.estimated_position
    err = float(np.linalg.norm(true_pos - est_pos))

    # EKF should track ground truth with sub-meter or near-meter precision under noisy sensors
    assert err < 2.0
    st = drone.get_state()
    assert st.estimated_position is not None
    assert np.allclose(st.estimated_position, est_pos)


def test_drone_atmospheric_wind_drift():
    """Verify that ambient wind creates relative airspeed drag opposing relative motion."""
    drone = Drone("UAV_WIND", initial_position=np.array([0.0, 0.0, 30.0]))
    drone.set_flight_mode(FlightMode.TRANSIT)
    
    # Hover with zero commanded force, but strong Eastward wind (vx = +10 m/s)
    headwind = np.array([10.0, 0.0, 0.0])
    dt = 0.05
    for _ in range(40):  # 2.0 seconds
        drone.step_physics(dt=dt, commanded_force=np.zeros(3), ambient_wind=headwind)

    # Drone should experience aerodynamic drag accelerating it in the direction of the wind (positive X)
    assert drone.velocity[0] > 0.5
    assert drone.position[0] > 0.5


def test_core_weather_simulation():
    """Verify SwarmSimulationCore with enable_weather=True generates valid weather telemetry."""
    config = SimulationConfig(
        dt=0.05,
        enable_weather=True,
        wind_config=WindConfig(mean_speed_mps=5.0, direction_deg=90.0),
    )
    core = SwarmSimulationCore(config=config)
    d = Drone("UAV_1", initial_position=np.array([0.0, 0.0, 30.0]))
    d.set_flight_mode(FlightMode.TRANSIT)
    d.set_target_waypoint(np.array([20.0, 20.0, 30.0]))
    core.add_drone(d)

    snap = core.step()
    assert snap.weather is not None
    assert snap.weather["enabled"] is True
    assert snap.weather["mean_speed_mps"] == pytest.approx(5.0)
    assert snap.weather["direction_deg"] == pytest.approx(90.0)

    # Verify telemetry dictionary includes weather
    telemetry_dict = core.to_dict()
    assert "weather" in telemetry_dict
    assert telemetry_dict["weather"]["enabled"] is True
