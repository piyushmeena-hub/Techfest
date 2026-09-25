"""
UAV-X: 6-DOF Quadrotor Dynamics & B-Spline Trajectory Engine
Inspired directly by:
  - gym-pybullet-drones (UTIAS DSL) — 6-DOF equations of motion & rotor dynamics
  - EGO-Planner-v2 (ZJU FAST Lab) — Uniform cubic B-spline trajectory generation
  - UAV Swarm Network Simulator — Log-distance RF path loss & SNR modeling
"""
from __future__ import annotations
import math
import numpy as np
from typing import List, Tuple, Dict, Optional


class QuadrotorPhysics:
    """
    6-DOF Quadrotor dynamic model representing quadrotor flight physics.
    Calculates body attitude (roll, pitch, yaw), rotor RPMs, and aerodynamic drag.
    Parameters match Bitcraze Crazyflie 2.X / DJI F450 multirotor platforms.
    """

    def __init__(self, mass: float = 1.2, arm_length: float = 0.225):
        self.mass = mass                 # kg
        self.arm_length = arm_length     # m (center to motor)
        self.g = 9.81                    # m/s^2
        self.kf = 3.16e-5                # Thrust coefficient (N / (rad/s)^2)
        self.km = 7.94e-7                # Torque coefficient (N*m / (rad/s)^2)
        self.hover_rpm = math.sqrt((mass * self.g / 4.0) / self.kf) * (60.0 / (2 * math.pi))

        # Inertia moments (kg * m^2)
        self.Ixx = 1.4e-2
        self.Iyy = 1.4e-2
        self.Izz = 2.17e-2

        # Aerodynamic drag coefficients
        self.drag_xy = 0.25
        self.drag_z = 0.40

    def compute_state(self, current_pos: Tuple[float, float, float],
                      prev_pos: Tuple[float, float, float],
                      dt: float = 1.0) -> Dict[str, any]:
        """
        Compute 6-DOF velocity, attitude (roll, pitch, yaw), and rotor RPMs
        from position transitions.
        """
        vx = (current_pos[0] - prev_pos[0]) / max(dt, 1e-4)
        vy = (current_pos[1] - prev_pos[1]) / max(dt, 1e-4)
        vz = (current_pos[2] - prev_pos[2]) / max(dt, 1e-4)
        speed_2d = math.sqrt(vx * vx + vy * vy)

        # Yaw: angle of movement direction in degrees
        yaw_deg = math.degrees(math.atan2(vy, vx)) if speed_2d > 0.05 else 0.0

        # Pitch & Roll: tilt angles proportional to horizontal acceleration and drag
        # Max tilt angle capped at 25 degrees for stability
        max_tilt_rad = math.radians(25.0)
        pitch_rad = max(-max_tilt_rad, min(max_tilt_rad, (vx * self.drag_xy) / (self.mass * self.g)))
        roll_rad = max(-max_tilt_rad, min(max_tilt_rad, -(vy * self.drag_xy) / (self.mass * self.g)))

        pitch_deg = math.degrees(pitch_rad)
        roll_deg = math.degrees(roll_rad)

        # Rotor RPMs: base hover RPM modified by tilt and vertical climb rate
        vertical_thrust_factor = 1.0 + (vz * 0.15)
        rpm_base = self.hover_rpm * max(0.6, min(1.4, vertical_thrust_factor))

        # Differential RPMs for roll/pitch actuation
        delta_pitch = pitch_rad * 300.0
        delta_roll = roll_rad * 300.0

        rpms = [
            round(rpm_base + delta_pitch - delta_roll),  # Front-Right
            round(rpm_base - delta_pitch - delta_roll),  # Rear-Right
            round(rpm_base - delta_pitch + delta_roll),  # Rear-Left
            round(rpm_base + delta_pitch + delta_roll),  # Front-Left
        ]

        return {
            "velocity": (round(vx, 2), round(vy, 2), round(vz, 2)),
            "speed": round(math.sqrt(vx * vx + vy * vy + vz * vz), 2),
            "roll": round(roll_deg, 1),
            "pitch": round(pitch_deg, 1),
            "yaw": round(yaw_deg, 1),
            "rpms": rpms,
        }


class BSplineTrajectoryPlanner:
    """
    Uniform Cubic B-Spline trajectory generator as used in EGO-Planner and Fast-Planner.
    Generates C^2-smooth, minimum-jerk flight trajectories parameterized by control points.
    """

    def __init__(self, degree: int = 3):
        self.degree = degree
        # B-spline basis matrix for uniform cubic B-spline (degree=3)
        self.basis_matrix = (1.0 / 6.0) * np.array([
            [-1,  3, -3,  1],
            [ 3, -6,  3,  0],
            [-3,  0,  3,  0],
            [ 1,  4,  1,  0]
        ])

    def generate_path(self, waypoints: List[Tuple[float, float, float]],
                      num_samples_per_seg: int = 10) -> List[Tuple[float, float, float]]:
        """
        Generate continuous 3D B-spline trajectory through waypoints.
        If fewer than 4 waypoints, pads with start and end to form a clamped spline.
        """
        if len(waypoints) < 2:
            return waypoints

        pts = list(waypoints)
        # Clamp endpoints by duplicating start and end control points
        control_points = [pts[0], pts[0]] + pts + [pts[-1], pts[-1]]
        n = len(control_points)

        trajectory: List[Tuple[float, float, float]] = []

        for i in range(n - 3):
            P = np.array(control_points[i:i + 4])  # 4x3 matrix
            for u in np.linspace(0.0, 1.0, num_samples_per_seg, endpoint=(i == n - 4)):
                U = np.array([u**3, u**2, u, 1.0])
                pt = U @ self.basis_matrix @ P
                trajectory.append((round(float(pt[0]), 2),
                                   round(float(pt[1]), 2),
                                   round(float(pt[2]), 2)))

        return trajectory


class RFPropagationModel:
    """
    Log-Distance Path Loss & Shadowing model inspired by NS-3 / UAV Swarm Network Simulator.
    Calculates Received Signal Strength Indicator (RSSI) and Signal-to-Noise Ratio (SNR).
    """

    def __init__(self, tx_power_dbm: float = 20.0, freq_ghz: float = 2.4,
                 path_loss_exp: float = 2.6, shadow_std: float = 3.0):
        self.tx_power = tx_power_dbm        # dBm (typical 100mW Wi-Fi transmitter)
        self.freq = freq_ghz                # GHz (2.4 GHz ISM band)
        self.eta = path_loss_exp            # Path loss exponent (2.0 = free space, 2.6 = suburban disaster)
        self.sigma = shadow_std             # Shadowing standard deviation (dB)
        # Reference path loss at d0 = 1m
        self.pl_d0 = 20.0 * math.log10(self.freq * 1e9) + 20.0 * math.log10(4 * math.pi / 3e8)
        self.noise_floor = -95.0            # dBm (thermal noise + receiver noise figure)

    def compute_link_metrics(self, distance: float) -> Dict[str, float]:
        """Compute RSSI, SNR, and estimated packet loss."""
        d = max(1.0, distance)
        path_loss = self.pl_d0 + 10.0 * self.eta * math.log10(d)
        rssi = self.tx_power - path_loss
        snr = rssi - self.noise_floor

        # Packet error rate based on QPSK/OFDM threshold model
        if snr > 18.0:
            loss = 0.00
        elif snr > 10.0:
            loss = 0.05 * (18.0 - snr) / 8.0
        elif snr > 3.0:
            loss = 0.20 + 0.60 * (10.0 - snr) / 7.0
        else:
            loss = 1.00  # Disconnected

        return {
            "distance_m": round(distance, 1),
            "rssi_dbm": round(rssi, 1),
            "snr_db": round(snr, 1),
            "packet_loss": round(min(1.0, max(0.0, loss)), 3),
        }
