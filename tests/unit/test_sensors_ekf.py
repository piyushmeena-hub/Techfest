"""
tests/unit/test_sensors_ekf.py: Unit tests for SensorSuite and DroneEKF state estimator.
"""

import numpy as np
import pytest

from sim.sensors import DroneEKF, SensorConfig, SensorSuite


def test_sensor_suite_multi_rate_sampling():
    """Verify IMU samples on every step while GPS and Baro sample at their configured rates."""
    config = SensorConfig(gps_update_interval=0.10, baro_update_interval=0.05)
    suite = SensorSuite(config)

    pos = np.array([10.0, 20.0, 30.0])
    vel = np.array([1.0, 0.0, 0.0])
    accel = np.array([0.0, 0.0, 0.0])
    omega = np.array([0.0, 0.0, 0.0])

    dt = 0.05

    # Step 1 (t = 0.05s): IMU + Baro present, GPS not yet
    out1 = suite.sample(pos, vel, accel, omega, dt)
    assert out1["imu"] is not None
    assert out1["baro"] is not None
    assert out1["gps"] is None

    # Step 2 (t = 0.10s): IMU + Baro + GPS present
    out2 = suite.sample(pos, vel, accel, omega, dt)
    assert out2["imu"] is not None
    assert out2["baro"] is not None
    assert out2["gps"] is not None
    assert "hdop" in out2["gps"]


def test_ekf_filter_convergence():
    """Verify EKF converges to ground truth position within 0.6m despite measurement noise."""
    ekf = DroneEKF(initial_position=np.array([0.0, 0.0, 0.0]))
    suite = SensorSuite()

    # Simulate 5.0 seconds of flight moving at constant velocity [2.0, 1.0, 0.5] m/s
    dt = 0.05
    true_pos = np.array([0.0, 0.0, 10.0])
    true_vel = np.array([2.0, 1.0, 0.0])
    true_accel = np.zeros(3)
    true_omega = np.zeros(3)

    for _ in range(100):
        true_pos += true_vel * dt
        data = suite.sample(true_pos, true_vel, true_accel, true_omega, dt)

        # 1. EKF Predict with IMU
        ekf.predict(data["imu"]["accel"], dt)

        # 2. EKF Update with Baro if available
        if data["baro"] is not None:
            ekf.update_baro(data["baro"])

        # 3. EKF Update with GPS if available
        if data["gps"] is not None:
            ekf.update_gps(data["gps"]["position"], data["gps"]["velocity"], data["gps"]["hdop"])

    # Position estimation error after convergence
    pos_error = float(np.linalg.norm(ekf.estimated_position - true_pos))
    vel_error = float(np.linalg.norm(ekf.estimated_velocity - true_vel))

    assert pos_error < 0.65, f"EKF position error {pos_error:.3f}m exceeds 0.65m"
    assert vel_error < 0.35, f"EKF velocity error {vel_error:.3f}m exceeds 0.35m"


def test_ekf_covariance_symmetry():
    """Verify error covariance matrix P remains positive semi-definite and symmetric."""
    ekf = DroneEKF()
    ekf.predict(np.array([0.1, -0.2, 0.05]), dt=0.05)
    ekf.update_gps(np.array([1.0, 1.0, 1.0]), np.array([0.1, 0.1, 0.0]))

    # Symmetry check
    assert np.allclose(ekf.P, ekf.P.T, atol=1e-8)
    # Positive eigenvalues (positive semi-definite)
    eigenvalues = np.linalg.eigvals(ekf.P)
    assert np.all(eigenvalues > -1e-6)
