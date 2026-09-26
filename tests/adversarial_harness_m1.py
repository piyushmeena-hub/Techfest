"""
tests/adversarial_harness_m1.py: Empirical Stress Harness for Milestone 1.

Executes precise simulations across:
1. High-velocity head-on encounters (collinear and small lateral offsets)
2. Multi-drone collinear compression (3, 4, and 5 drones)
3. Extreme waypoint jumps (acceleration and velocity bounds checks)
4. Vertical downwash cone penetration (with zero offset and small offset)
5. Ground collision and boundary clamping stress
"""

import math
import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.core import SimulationConfig, SwarmSimulationCore
from sim.drone import Drone
from sim.environment import DisasterEnvironment
from sim.types import DroneLimits, DroneRole, FlightMode


def run_head_on_encounter(initial_offset_y=0.0, dt=0.05, speed=10.0):
    """
    Two drones initialized at x = -40 and +40 moving towards each other at speed.
    """
    sim = SwarmSimulationCore(SimulationConfig(dt=dt))
    d1 = Drone("D1", role=DroneRole.SURVEY, initial_pos=np.array([-40.0, 0.0, 30.0]))
    d2 = Drone("D2", role=DroneRole.SURVEY, initial_pos=np.array([40.0, initial_offset_y, 30.0]))

    d1.set_flight_mode(FlightMode.TRANSIT)
    d2.set_flight_mode(FlightMode.TRANSIT)

    d1.velocity = np.array([speed, 0.0, 0.0], dtype=np.float64)
    d2.velocity = np.array([-speed, 0.0, 0.0], dtype=np.float64)

    d1.set_target_waypoint(np.array([100.0, 0.0, 30.0]))
    d2.set_target_waypoint(np.array([-100.0, initial_offset_y, 30.0]))

    sim.add_drone(d1)
    sim.add_drone(d2)

    min_dist = float("inf")
    trace = []
    crossed = False

    for step in range(120):
        t = step * dt
        p1 = d1.position.copy()
        p2 = d2.position.copy()
        dist = float(np.linalg.norm(p1 - p2))
        if dist < min_dist:
            min_dist = dist
        if p1[0] > p2[0]:
            crossed = True

        v1 = d1.velocity.copy()
        v2 = d2.velocity.copy()
        trace.append((t, dist, p1[0], p2[0], v1[0], v2[0]))
        sim.step()

    return {
        "min_dist": min_dist,
        "crossed": crossed,
        "trace": trace,
    }


def run_collinear_compression(num_drones=3, dt=0.05):
    """
    num_drones collinear on X axis.
    Outer drones commanded to push inward through the inner drones.
    """
    sim = SwarmSimulationCore(SimulationConfig(dt=dt))
    drones = []
    spacing = 10.0
    start_x = -((num_drones - 1) / 2.0) * spacing

    for i in range(num_drones):
        x = start_x + i * spacing
        d = Drone(f"D_{i}", role=DroneRole.SURVEY, initial_pos=np.array([x, 0.0, 30.0]))
        d.set_flight_mode(FlightMode.TRANSIT)
        # Leftmost wants to go far right, rightmost wants to go far left, middle wants to stay at origin
        if i == 0:
            d.set_target_waypoint(np.array([50.0, 0.0, 30.0]))
        elif i == num_drones - 1:
            d.set_target_waypoint(np.array([-50.0, 0.0, 30.0]))
        else:
            d.set_target_waypoint(np.array([0.0, 0.0, 30.0]))
        drones.append(d)
        sim.add_drone(d)

    min_pairwise = float("inf")
    critical_pair = None

    for step in range(150):
        sim.step()
        for i in range(num_drones):
            for j in range(i + 1, num_drones):
                d_ij = float(np.linalg.norm(drones[i].position - drones[j].position))
                if d_ij < min_pairwise:
                    min_pairwise = d_ij
                    critical_pair = (drones[i].id, drones[j].id)

    return {
        "num_drones": num_drones,
        "min_pairwise_distance": min_pairwise,
        "critical_pair": critical_pair,
        "final_positions": [d.position.tolist() for d in drones],
    }


def run_waypoint_jump_stress():
    """
    Test extreme waypoint jump kinematics bounds:
    - Acceleration <= 4.0 m/s^2
    - Velocity <= 10.0 m/s
    """
    drone = Drone("UAV_WP", initial_pos=np.array([0.0, 0.0, 20.0]))
    drone.set_flight_mode(FlightMode.TRANSIT)

    waypoints = [
        np.array([1000.0, 0.0, 20.0]),
        np.array([-1000.0, 1000.0, 60.0]),
        np.array([0.0, -1000.0, 10.0]),
        np.array([1e5, 1e5, 100.0]),
        np.array([-1e5, -1e5, 5.0]),
    ]

    max_accel_observed = 0.0
    max_vel_xy_observed = 0.0
    max_vz_observed = -float("inf")
    min_vz_observed = float("inf")
    dt = 0.05

    for wp in waypoints:
        drone.set_target_waypoint(wp)
        for _ in range(60):
            drone.step(dt)
            a_mag = float(np.linalg.norm(drone.acceleration))
            v_xy = float(np.linalg.norm(drone.velocity[:2]))
            vz = float(drone.velocity[2])

            max_accel_observed = max(max_accel_observed, a_mag)
            max_vel_xy_observed = max(max_vel_xy_observed, v_xy)
            max_vz_observed = max(max_vz_observed, vz)
            min_vz_observed = min(min_vz_observed, vz)

    return {
        "max_accel": max_accel_observed,
        "max_vel_xy": max_vel_xy_observed,
        "max_vz": max_vz_observed,
        "min_vz": min_vz_observed,
        "accel_bound_ok": max_accel_observed <= 4.0 + 1e-6,
        "vel_xy_bound_ok": max_vel_xy_observed <= 10.0 + 1e-6,
        "climb_bound_ok": max_vz_observed <= 3.5 + 1e-6,
        "descent_bound_ok": min_vz_observed >= -2.5 - 1e-6,
    }


def run_downwash_penetration_stress():
    """
    Test downwash cone behavior:
    1. Lower drone with zero lateral offset: (0, 0, 30) below upper at (0, 0, 35)
    2. Lower drone with 0.5m lateral offset: (0.5, 0, 30) below upper at (0, 0, 35)
    3. Check upper drone reaction
    """
    # Test in SwarmSimulationCore
    sim_zero = SwarmSimulationCore()
    d_top0 = Drone("TOP0", initial_pos=np.array([0.0, 0.0, 35.0]))
    d_bot0 = Drone("BOT0", initial_pos=np.array([0.0, 0.0, 30.0]))
    d_top0.set_flight_mode(FlightMode.SURVEYING)
    d_bot0.set_flight_mode(FlightMode.SURVEYING)
    sim_zero.add_drone(d_top0)
    sim_zero.add_drone(d_bot0)

    f_bot0 = sim_zero.compute_steering_forces(d_bot0)
    f_top0 = sim_zero.compute_steering_forces(d_top0)

    # Offset simulation
    sim_offset = SwarmSimulationCore()
    d_top1 = Drone("TOP1", initial_pos=np.array([0.0, 0.0, 35.0]))
    d_bot1 = Drone("BOT1", initial_pos=np.array([0.5, 0.0, 30.0]))
    d_top1.set_flight_mode(FlightMode.SURVEYING)
    d_bot1.set_flight_mode(FlightMode.SURVEYING)
    sim_offset.add_drone(d_top1)
    sim_offset.add_drone(d_bot1)

    f_bot1 = sim_offset.compute_steering_forces(d_bot1)
    f_top1 = sim_offset.compute_steering_forces(d_top1)

    # Multi-step dynamic run of offset downwash
    offset_trajectories = []
    for _ in range(50):
        sim_offset.step()
        offset_trajectories.append({
            "bot_pos": d_bot1.position.tolist(),
            "bot_vel": d_bot1.velocity.tolist(),
            "top_pos": d_top1.position.tolist(),
        })

    # Multi-step dynamic run of zero offset downwash
    zero_trajectories = []
    for _ in range(50):
        sim_zero.step()
        zero_trajectories.append({
            "bot_pos": d_bot0.position.tolist(),
            "bot_vel": d_bot0.velocity.tolist(),
            "top_pos": d_top0.position.tolist(),
        })

    return {
        "zero_offset_forces": {
            "bot": f_bot0.tolist(),
            "top": f_top0.tolist(),
        },
        "offset_forces": {
            "bot": f_bot1.tolist(),
            "top": f_top1.tolist(),
        },
        "zero_trajectories_end": zero_trajectories[-1],
        "offset_trajectories_end": offset_trajectories[-1],
    }


def run_boundary_and_ground_stress():
    """
    Stress-test boundary clamping and ground contact.
    """
    env = DisasterEnvironment()
    d_ground = Drone("GROUND", initial_pos=np.array([0.0, 0.0, 1.0]))
    d_ground.set_flight_mode(FlightMode.TRANSIT)
    d_ground.velocity = np.array([0.0, 0.0, -2.5])

    ground_z_history = []
    ground_vz_history = []
    dt = 0.05
    for _ in range(40):
        d_ground.step(dt, desired_accel=np.array([0.0, 0.0, -4.0]))
        env.enforce_bounds(d_ground)
        ground_z_history.append(float(d_ground.position[2]))
        ground_vz_history.append(float(d_ground.velocity[2]))

    # Ceiling test
    d_ceil = Drone("CEIL", initial_pos=np.array([0.0, 0.0, 118.0]))
    d_ceil.set_flight_mode(FlightMode.TRANSIT)
    d_ceil.velocity = np.array([0.0, 0.0, 3.5])
    ceil_z_history = []
    for _ in range(40):
        d_ceil.step(dt, desired_accel=np.array([0.0, 0.0, 4.0]))
        env.enforce_bounds(d_ceil)
        ceil_z_history.append(float(d_ceil.position[2]))

    # Lateral boundary test
    d_wall = Drone("WALL", initial_pos=np.array([248.0, 0.0, 50.0]))
    d_wall.set_flight_mode(FlightMode.TRANSIT)
    d_wall.velocity = np.array([10.0, 0.0, 0.0])
    wall_x_history = []
    for _ in range(40):
        d_wall.step(dt, desired_accel=np.array([4.0, 0.0, 0.0]))
        env.enforce_bounds(d_wall)
        wall_x_history.append(float(d_wall.position[0]))

    return {
        "min_ground_z": min(ground_z_history),
        "final_ground_z": ground_z_history[-1],
        "final_ground_vz": ground_vz_history[-1],
        "max_ceil_z": max(ceil_z_history),
        "max_wall_x": max(wall_x_history),
    }


if __name__ == "__main__":
    print("=== Adversarial Physics Stress Harness ===")

    print("\n--- 1. High-Velocity Head-On Encounters ---")
    res_head_on_collinear = run_head_on_encounter(initial_offset_y=0.0)
    print(f"Collinear Head-on: min_dist = {res_head_on_collinear['min_dist']:.4f} m, crossed = {res_head_on_collinear['crossed']}")

    res_head_on_offset = run_head_on_encounter(initial_offset_y=0.2)
    print(f"Offset 0.2m Head-on: min_dist = {res_head_on_offset['min_dist']:.4f} m, crossed = {res_head_on_offset['crossed']}")

    res_head_on_offset1 = run_head_on_encounter(initial_offset_y=1.0)
    print(f"Offset 1.0m Head-on: min_dist = {res_head_on_offset1['min_dist']:.4f} m, crossed = {res_head_on_offset1['crossed']}")

    print("\n--- 2. Multi-Drone Collinear Compression ---")
    for n in [3, 4, 5]:
        res_comp = run_collinear_compression(num_drones=n)
        print(f"Compression N={n}: min_pairwise_distance = {res_comp['min_pairwise_distance']:.4f} m, critical_pair = {res_comp['critical_pair']}")

    print("\n--- 3. Extreme Waypoint Jumps ---")
    res_wp = run_waypoint_jump_stress()
    print(f"Max accel: {res_wp['max_accel']:.4f} m/s^2 (ok={res_wp['accel_bound_ok']})")
    print(f"Max vel_xy: {res_wp['max_vel_xy']:.4f} m/s (ok={res_wp['vel_xy_bound_ok']})")
    print(f"Climb: {res_wp['max_vz']:.4f} m/s (ok={res_wp['climb_bound_ok']}), Descent: {res_wp['min_vz']:.4f} m/s (ok={res_wp['descent_bound_ok']})")

    print("\n--- 4. Downwash Cone Penetration ---")
    try:
        res_dw = run_downwash_penetration_stress()
        print(f"Zero offset forces on bot: {res_dw['zero_offset_forces']['bot']}")
        print(f"Zero offset forces on top: {res_dw['zero_offset_forces']['top']}")
        print(f"Offset forces on bot: {res_dw['offset_forces']['bot']}")
        print(f"Offset forces on top: {res_dw['offset_forces']['top']}")
        print(f"Zero offset end pos bot: {res_dw['zero_trajectories_end']['bot_pos']}")
        print(f"Offset end pos bot: {res_dw['offset_trajectories_end']['bot_pos']}")
    except Exception as e:
        print(f"CRASH in Downwash Cone Penetration: {type(e).__name__}: {e}")

    print("\n--- 5. Boundary and Ground Clamping ---")
    try:
        res_bound = run_boundary_and_ground_stress()
        print(f"Min ground z: {res_bound['min_ground_z']:.4f}, final z: {res_bound['final_ground_z']:.4f}, final vz: {res_bound['final_ground_vz']:.4f}")
        print(f"Max ceil z: {res_bound['max_ceil_z']:.4f}")
        print(f"Max wall x: {res_bound['max_wall_x']:.4f}")
    except Exception as e:
        print(f"CRASH in Boundary and Ground Clamping: {type(e).__name__}: {e}")
