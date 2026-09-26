"""
tests/unit/test_adversarial_m1.py: Adversarial Physics, Kinematics Bounds & Stress Test Suite.

Adversarial stress challenges targeting:
- High-velocity head-on drone encounters
- Multi-drone collinear compression
- Extreme waypoint jumps (kinematic bounds enforcement)
- Downwash cone penetration (symmetric vs asymmetric, zero-offset lateral push, NameError detection)
- Ground/ceiling and boundary clamping robustness
- High-speed 3D obstacle avoidance penetration
"""

import math
import numpy as np
import pytest

from sim.core import SimulationConfig, SwarmSimulationCore
from sim.drone import Drone
from sim.environment import DisasterEnvironment, EnvironmentConfig
from sim.obstacles import ObstacleAABB
from sim.types import DroneLimits, DroneRole, FlightMode


class TestAdversarialKinematicBounds:
    """Stress-test kinematic and dynamic limits under extreme inputs."""

    def test_extreme_waypoint_jump_kinematics(self):
        """
        Verify that extreme waypoint jumps (even 1,000,000 meters away)
        strictly enforce:
        - Linear acceleration <= 4.0 m/s^2
        - Horizontal velocity <= 10.0 m/s
        - Vertical climb rate <= 3.5 m/s
        - Vertical descent rate <= 2.5 m/s
        """
        drone = Drone("UAV_STRESS_1", role=DroneRole.SURVEY, initial_pos=np.array([0.0, 0.0, 30.0]))
        drone.set_flight_mode(FlightMode.TRANSIT)

        extreme_waypoints = [
            np.array([1e6, 1e6, 100.0]),
            np.array([-1e6, -1e6, 10.0]),
            np.array([0.0, 0.0, 1e6]),
            np.array([0.0, 0.0, -1e6]),
            np.array([500.0, -500.0, 40.0]),
        ]

        dt = 0.05
        for wp in extreme_waypoints:
            drone.set_target_waypoint(wp)
            for _ in range(100):
                drone.step(dt)
                accel_mag = float(np.linalg.norm(drone.acceleration))
                v_xy = float(np.linalg.norm(drone.velocity[:2]))
                vz = float(drone.velocity[2])

                assert accel_mag <= drone.limits.max_accel + 1e-6, f"Acceleration {accel_mag} exceeded {drone.limits.max_accel}"
                assert v_xy <= drone.limits.max_speed_xy + 1e-6, f"Horizontal velocity {v_xy} exceeded {drone.limits.max_speed_xy}"
                assert vz <= drone.limits.max_speed_z_up + 1e-6, f"Climb rate {vz} exceeded {drone.limits.max_speed_z_up}"
                assert vz >= -drone.limits.max_speed_z_down - 1e-6, f"Descent rate {vz} exceeded {-drone.limits.max_speed_z_down}"
                assert not np.any(np.isnan(drone.position)), "Position contains NaN"
                assert not np.any(np.isnan(drone.velocity)), "Velocity contains NaN"
                assert not np.any(np.isnan(drone.quaternion)), "Quaternion contains NaN"

    def test_direct_excessive_acceleration_injection(self):
        """
        Verify that even if commanded acceleration is directly set to massive values (e.g. 1e8),
        drone.step saturates it to limits.max_accel.
        """
        drone = Drone("UAV_STRESS_ACCEL", initial_pos=np.array([0.0, 0.0, 20.0]))
        drone.set_flight_mode(FlightMode.TRANSIT)

        massive_accels = [
            np.array([1e7, 0.0, 0.0]),
            np.array([0.0, 1e7, 0.0]),
            np.array([0.0, 0.0, 1e7]),
            np.array([-5e6, -5e6, -5e6]),
        ]

        for a_cmd in massive_accels:
            drone.step(0.05, desired_accel=a_cmd)
            mag = float(np.linalg.norm(drone.acceleration))
            assert mag <= drone.limits.max_accel + 1e-6, f"Direct acceleration {mag} > {drone.limits.max_accel}"


class TestAdversarialDownwash:
    """Stress-test aerodynamic downwash cone behavior."""

    def test_downwash_asymmetry_and_lateral_push(self):
        """
        Test two drones: upper drone at (0, 0, 40) and lower drone at (0.2, 0.0, 35).
        Lower drone is inside downwash cone (dz = 5.0m, r_cone = 5.0 * tan(25 deg) ~ 2.33m).
        Upper drone must feel 0 downwash force.
        Lower drone must feel positive lateral repulsion pushing it away from (0, 0)
        and downward force.
        """
        upper = Drone("UAV_UPPER", initial_pos=np.array([0.0, 0.0, 40.0]))
        lower = Drone("UAV_LOWER", initial_pos=np.array([0.2, 0.0, 35.0]))
        upper.set_flight_mode(FlightMode.TRANSIT)
        lower.set_flight_mode(FlightMode.TRANSIT)

        f_dw_upper = upper.compute_downwash_repulsion([lower])
        f_dw_lower = lower.compute_downwash_repulsion([upper])

        assert np.allclose(f_dw_upper, 0.0), f"Upper drone felt downwash force: {f_dw_upper}"
        assert f_dw_lower[0] > 0.0, f"Lower drone did not receive positive lateral X push: {f_dw_lower}"
        assert f_dw_lower[2] < 0.0, f"Lower drone did not receive downward push: {f_dw_lower}"

    def test_downwash_execution_and_lateral_escape_in_core(self):
        """
        Adversarial test:
        1. Verifies that downwash calculation in SwarmSimulationCore executes without crashing
           (detects missing 'import math' NameError bug in sim/core.py:183).
        2. Verifies that when lower drone is positioned under upper drone, it receives a non-zero
           lateral escape force (detects vanishing lateral escape bug).
        """
        sim = SwarmSimulationCore()
        d_top = Drone("TOP", initial_pos=np.array([0.0, 0.0, 35.0]))
        d_bot = Drone("BOT", initial_pos=np.array([0.0, 0.0, 30.0]))
        d_top.set_flight_mode(FlightMode.SURVEYING)
        d_bot.set_flight_mode(FlightMode.SURVEYING)

        sim.add_drone(d_top)
        sim.add_drone(d_bot)

        # In unpatched code, this will raise NameError: name 'math' is not defined
        forces_bot = sim.compute_steering_forces(d_bot)
        lat_force_mag = float(np.linalg.norm(forces_bot[:2]))
        assert lat_force_mag > 0.1, f"Zero lateral escape force on lower drone: {forces_bot}"


class TestAdversarialBoundaryAndGroundClamping:
    """Stress-test ground contact, ceiling bounds, and boundary containment."""

    def test_high_speed_ground_impact_no_penetration_or_nan(self):
        """
        Drop drone at extreme downward velocity towards the ground.
        Verify z >= 0 at all times, velocity[2] >= 0 after collision, and no NaN.
        """
        drone = Drone("UAV_DIVE", initial_pos=np.array([0.0, 0.0, 2.0]))
        drone.set_flight_mode(FlightMode.TRANSIT)
        drone.velocity = np.array([0.0, 0.0, -2.5], dtype=np.float64)

        env = DisasterEnvironment()
        dt = 0.05
        for _ in range(50):
            drone.step(dt, desired_accel=np.array([0.0, 0.0, -4.0]))
            env.enforce_bounds(drone)

            assert drone.position[2] >= 0.0, f"Ground penetration: z={drone.position[2]}"
            assert not math.isnan(drone.position[2]), "z is NaN"
            assert not math.isnan(drone.velocity[2]), "vz is NaN"

        assert drone.position[2] == 0.0
        assert drone.velocity[2] >= 0.0

    def test_world_boundary_containment_under_full_thrust(self):
        """
        Fly drone at max speed straight into positive X boundary (x_max = 250.0).
        Verify coordinates never exceed bounds and remain numerically stable.
        """
        env = DisasterEnvironment(bounds_x=(-250.0, 250.0), bounds_y=(-250.0, 250.0), bounds_z=(0.0, 120.0))
        drone = Drone("UAV_BOUND_CRASH", initial_pos=np.array([245.0, 0.0, 50.0]))
        drone.set_flight_mode(FlightMode.TRANSIT)
        drone.velocity = np.array([10.0, 0.0, 0.0], dtype=np.float64)
        drone.set_target_waypoint(np.array([500.0, 0.0, 50.0]))

        dt = 0.05
        for _ in range(100):
            drone.step(dt, desired_accel=np.array([4.0, 0.0, 0.0]))
            env.enforce_bounds(drone)

            assert drone.position[0] <= 250.0, f"X boundary exceeded: {drone.position[0]}"
            assert drone.position[1] <= 250.0 and drone.position[1] >= -250.0
            assert drone.position[2] <= 120.0 and drone.position[2] >= 0.0
            assert np.all(np.isfinite(drone.position)), "Position contains non-finite values"


class TestAdversarialCollisionsAndSeparation:
    """Stress-test collision avoidance in head-on, multi-drone compression, and obstacles."""

    def test_head_on_collision_encounter(self):
        """
        Two drones flying head-on at max velocity (10 m/s each, closing at 20 m/s).
        Dispatch requirement: Verify APF/Reynolds separation forces prevent penetration.
        Expected: Drones must maintain separation >= 1.5m and not cross.
        """
        sim = SwarmSimulationCore(SimulationConfig(dt=0.05))
        d1 = Drone("D1", role=DroneRole.SURVEY, initial_pos=np.array([-40.0, 0.0, 30.0]))
        d2 = Drone("D2", role=DroneRole.SURVEY, initial_pos=np.array([40.0, 0.0, 30.0]))

        d1.set_flight_mode(FlightMode.TRANSIT)
        d2.set_flight_mode(FlightMode.TRANSIT)

        d1.velocity = np.array([10.0, 0.0, 0.0], dtype=np.float64)
        d2.velocity = np.array([-10.0, 0.0, 0.0], dtype=np.float64)

        d1.set_target_waypoint(np.array([100.0, 0.0, 30.0]))
        d2.set_target_waypoint(np.array([-100.0, 0.0, 30.0]))

        sim.add_drone(d1)
        sim.add_drone(d2)

        min_dist = float("inf")
        crossed = False

        for step in range(100):
            sim.step()
            dist = float(np.linalg.norm(d1.position - d2.position))
            if dist < min_dist:
                min_dist = dist
            if d1.position[0] > d2.position[0]:
                crossed = True

        assert not crossed, f"Catastrophic head-on collision: D1 crossed D2! Min distance was {min_dist:.4f}m"
        assert min_dist >= 1.5, f"Separation safety bubble breached: min distance {min_dist:.4f}m < 1.5m"

    def test_multi_drone_collinear_compression(self):
        """
        Three drones collinear on X axis:
        D1 at -15.0, D2 at 0.0, D3 at +15.0.
        D1 commanded to +50.0 (pushing right), D3 commanded to -50.0 (pushing left),
        D2 commanded to stay at 0.0.
        Dispatch requirement: Verify separation distance >= 1.5m.
        """
        sim = SwarmSimulationCore(SimulationConfig(dt=0.05))
        d1 = Drone("D1", initial_pos=np.array([-15.0, 0.0, 30.0]))
        d2 = Drone("D2", initial_pos=np.array([0.0, 0.0, 30.0]))
        d3 = Drone("D3", initial_pos=np.array([15.0, 0.0, 30.0]))

        for d in [d1, d2, d3]:
            d.set_flight_mode(FlightMode.TRANSIT)

        d1.set_target_waypoint(np.array([50.0, 0.0, 30.0]))
        d2.set_target_waypoint(np.array([0.0, 0.0, 30.0]))
        d3.set_target_waypoint(np.array([-50.0, 0.0, 30.0]))

        sim.add_drone(d1)
        sim.add_drone(d2)
        sim.add_drone(d3)

        min_sep = float("inf")
        for _ in range(120):
            sim.step()
            d12 = float(np.linalg.norm(d1.position - d2.position))
            d23 = float(np.linalg.norm(d2.position - d3.position))
            min_sep = min(min_sep, d12, d23)

        assert min_sep >= 1.5, f"Collinear compression failure: minimum separation was {min_sep:.4f}m < 1.5m"

    def test_high_speed_obstacle_penetration(self):
        """
        Drone flying at full cruising speed (10 m/s) towards a solid 3D AABB building.
        Verify drone does not penetrate inside the obstacle volume.
        """
        sim = SwarmSimulationCore()
        obs = ObstacleAABB("OBS1", "Building", np.array([30.0, 40.0, 0.0]), np.array([90.0, 110.0, 55.0]))
        sim.add_obstacle(obs)

        d = Drone("D1", initial_pos=np.array([20.0, 75.0, 30.0]))
        d.set_flight_mode(FlightMode.TRANSIT)
        d.velocity = np.array([10.0, 0.0, 0.0])
        d.set_target_waypoint(np.array([120.0, 75.0, 30.0]))
        sim.add_drone(d)

        penetrated = False
        for _ in range(60):
            sim.step()
            if obs.contains_point(d.position):
                penetrated = True
                break

        assert not penetrated, f"Obstacle collision! Drone penetrated solid building at {d.position}"
