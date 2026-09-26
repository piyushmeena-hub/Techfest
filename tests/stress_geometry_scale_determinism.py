"""
tests/stress_geometry_scale_determinism.py

Empirical stress testing harness for:
1. Degenerate 3D Ray-AABB Geometry (vertex grazing, coplanar face grazing, zero-length, surface start/end)
2. Vectorized vs Scalar Numerical Divergence across 10,000+ random rays
3. Large Swarm Scaling (25 & 50 drones, 500 ticks in SwarmSimulationCore)
4. Deterministic Reproducibility across independent Python processes
"""

import json
import math
import os
import subprocess
import sys
import time
from typing import Any, Dict, List, Tuple
import numpy as np

# Ensure root directory is on python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sim.core import SimulationConfig, SwarmSimulationCore
from sim.drone import Drone
from sim.environment import DisasterEnvironment, EnvironmentConfig
from sim.obstacles import ObstacleAABB, ObstacleManager, create_default_disaster_obstacles
from sim.types import DroneLimits, DroneRole, FlightMode


def run_degenerate_geometry_tests() -> Dict[str, Any]:
    print("=" * 70)
    print("SUITE 1: DEGENERATE 3D RAY-AABB GEOMETRY STRESS TESTS")
    print("=" * 70)

    results = {}
    box = ObstacleAABB(
        id="BOX_TEST",
        name="Test Box",
        min_pt=np.array([0.0, 0.0, 0.0]),
        max_pt=np.array([10.0, 10.0, 10.0]),
        base_attenuation_db=22.0,
        attenuation_db_per_meter=1.5,
    )
    manager = ObstacleManager([box])

    # -------------------------------------------------------------
    # 1.1 Vertex Grazing Ray (touches vertex without interior penetration)
    # Corner at (10, 10, 10). Ray from (11, 9, 10) to (9, 11, 10).
    # Midpoint t=0.5 is (10, 10, 10).
    # -------------------------------------------------------------
    p_src = np.array([11.0, 9.0, 10.0])
    p_dst = np.array([9.0, 11.0, 10.0])
    res_scalar = box.intersect_ray_segment(p_src, p_dst)
    clear_scalar, pen_scalar, att_scalar, hits_scalar = manager.check_los(p_src, p_dst)
    clear_vec, pen_vec, att_vec = manager.check_los_batch(p_src[np.newaxis, :], p_dst[np.newaxis, :])

    print(f"\n[1.1 Vertex Grazing] Ray: {p_src} -> {p_dst}")
    print(f"  Scalar AABB: hit={res_scalar.hit}, t_enter={res_scalar.t_enter:.4f}, t_exit={res_scalar.t_exit:.4f}, pen={res_scalar.penetration_distance:.4f}m, att={res_scalar.attenuation_db:.2f}dB")
    print(f"  Manager Scalar: los_clear={clear_scalar}, pen={pen_scalar:.4f}m, att={att_scalar:.2f}dB")
    print(f"  Manager Batch:  los_clear={bool(clear_vec[0])}, pen={float(pen_vec[0]):.4f}m, att={float(att_vec[0]):.2f}dB")

    results["vertex_grazing"] = {
        "hit": res_scalar.hit,
        "pen_scalar": pen_scalar,
        "att_scalar": att_scalar,
        "pen_vec": float(pen_vec[0]),
        "att_vec": float(att_vec[0]),
        "vec_scalar_match": (clear_scalar == bool(clear_vec[0])) and math.isclose(pen_scalar, float(pen_vec[0]), abs_tol=1e-6),
    }

    # -------------------------------------------------------------
    # 1.2 Coplanar Face Grazing (ray lies entirely in the outer plane z=10.0)
    # Ray from (-5, 5, 10) to (15, 5, 10)
    # -------------------------------------------------------------
    p_src_face = np.array([-5.0, 5.0, 10.0])
    p_dst_face = np.array([15.0, 5.0, 10.0])
    res_face = box.intersect_ray_segment(p_src_face, p_dst_face)
    clear_face_s, pen_face_s, att_face_s, _ = manager.check_los(p_src_face, p_dst_face)
    clear_face_v, pen_face_v, att_face_v = manager.check_los_batch(p_src_face[np.newaxis, :], p_dst_face[np.newaxis, :])

    # Perturbed ray by 1e-10 outside the face (z=10.0000000001)
    p_src_perturbed = np.array([-5.0, 5.0, 10.0 + 1e-10])
    p_dst_perturbed = np.array([15.0, 5.0, 10.0 + 1e-10])
    res_perturbed = box.intersect_ray_segment(p_src_perturbed, p_dst_perturbed)

    print(f"\n[1.2 Coplanar Face Grazing] Ray: {p_src_face} -> {p_dst_face} (on z=10 face)")
    print(f"  Scalar AABB: hit={res_face.hit}, pen={res_face.penetration_distance:.4f}m, att={res_face.attenuation_db:.2f}dB")
    print(f"  Manager Scalar: los_clear={clear_face_s}, pen={pen_face_s:.4f}m, att={att_face_s:.2f}dB")
    print(f"  Manager Batch:  los_clear={bool(clear_face_v[0])}, pen={float(pen_face_v[0]):.4f}m, att={float(att_face_v[0]):.2f}dB")
    print(f"  Perturbed +1e-10m: hit={res_perturbed.hit}, pen={res_perturbed.penetration_distance:.4f}m, att={res_perturbed.attenuation_db:.2f}dB")

    results["coplanar_face_grazing"] = {
        "hit": res_face.hit,
        "pen": res_face.penetration_distance,
        "att": res_face.attenuation_db,
        "perturbed_hit": res_perturbed.hit,
        "vec_scalar_match": (clear_face_s == bool(clear_face_v[0])) and math.isclose(pen_face_s, float(pen_face_v[0]), abs_tol=1e-6),
    }

    # -------------------------------------------------------------
    # 1.3 Zero-Length Rays (p_src == p_dst)
    # Cases: inside, outside, on boundary surface, on edge, on vertex
    # -------------------------------------------------------------
    zero_cases = {
        "zero_inside": np.array([5.0, 5.0, 5.0]),
        "zero_outside": np.array([20.0, 20.0, 20.0]),
        "zero_surface": np.array([10.0, 5.0, 5.0]),
        "zero_edge": np.array([10.0, 10.0, 5.0]),
        "zero_vertex": np.array([10.0, 10.0, 10.0]),
    }

    results["zero_length"] = {}
    print(f"\n[1.3 Zero-Length Rays]")
    for name, pt in zero_cases.items():
        res_z = box.intersect_ray_segment(pt, pt)
        c_s, p_s, a_s, _ = manager.check_los(pt, pt)
        c_v, p_v, a_v = manager.check_los_batch(pt[np.newaxis, :], pt[np.newaxis, :])
        match = (c_s == bool(c_v[0])) and math.isclose(p_s, float(p_v[0]), abs_tol=1e-6) and math.isclose(a_s, float(a_v[0]), abs_tol=1e-6)
        print(f"  {name:12s} at {pt}: hit={res_z.hit}, los_clear={c_s}, pen={p_s:.4f}m, att={a_s:.2f}dB, vec_match={match}")
        results["zero_length"][name] = {
            "hit": res_z.hit,
            "los_clear": c_s,
            "pen": p_s,
            "att": a_s,
            "vec_match": match,
        }

    # -------------------------------------------------------------
    # 1.4 Rays Starting / Ending on Surface
    # -------------------------------------------------------------
    print(f"\n[1.4 Rays Starting / Ending on Surface]")
    # Starting on face (x=0), pointing OUTWARD into open space (-x direction)
    p_start_out_src = np.array([0.0, 5.0, 5.0])
    p_start_out_dst = np.array([-10.0, 5.0, 5.0])
    res_start_out = box.intersect_ray_segment(p_start_out_src, p_start_out_dst)
    c_s, p_s, a_s, _ = manager.check_los(p_start_out_src, p_start_out_dst)
    c_v, p_v, a_v = manager.check_los_batch(p_start_out_src[np.newaxis, :], p_start_out_dst[np.newaxis, :])
    print(f"  Start on surface, pointing AWAY: hit={res_start_out.hit}, los_clear={c_s}, pen={p_s:.4f}m, att={a_s:.2f}dB, vec_match={(c_s == bool(c_v[0]))}")

    # Starting on face (x=0), pointing INWARD into box (+x direction)
    p_start_in_src = np.array([0.0, 5.0, 5.0])
    p_start_in_dst = np.array([10.0, 5.0, 5.0])
    res_start_in = box.intersect_ray_segment(p_start_in_src, p_start_in_dst)
    c_si, p_si, a_si, _ = manager.check_los(p_start_in_src, p_start_in_dst)
    c_vi, p_vi, a_vi = manager.check_los_batch(p_start_in_src[np.newaxis, :], p_start_in_dst[np.newaxis, :])
    print(f"  Start on surface, pointing INWARD: hit={res_start_in.hit}, los_clear={c_si}, pen={p_si:.4f}m, att={a_si:.2f}dB, vec_match={(c_si == bool(c_vi[0]))}")

    # Ending on surface (x=0), coming from open space (-x direction)
    p_end_out_src = np.array([-10.0, 5.0, 5.0])
    p_end_out_dst = np.array([0.0, 5.0, 5.0])
    res_end_out = box.intersect_ray_segment(p_end_out_src, p_end_out_dst)
    c_se, p_se, a_se, _ = manager.check_los(p_end_out_src, p_end_out_dst)
    c_ve, p_ve, a_ve = manager.check_los_batch(p_end_out_src[np.newaxis, :], p_end_out_dst[np.newaxis, :])
    print(f"  End on surface, coming from OUTSIDE: hit={res_end_out.hit}, los_clear={c_se}, pen={p_se:.4f}m, att={a_se:.2f}dB, vec_match={(c_se == bool(c_ve[0]))}")

    results["surface_start_end"] = {
        "start_away_hit": res_start_out.hit,
        "start_away_pen": p_s,
        "start_away_att": a_s,
        "start_in_hit": res_start_in.hit,
        "start_in_pen": p_si,
        "start_in_att": a_si,
        "end_out_hit": res_end_out.hit,
        "end_out_pen": p_se,
        "end_out_att": a_se,
    }

    return results


def run_vectorized_vs_scalar_stress_test(num_rays: int = 10000) -> Dict[str, Any]:
    print("\n" + "=" * 70)
    print(f"SUITE 2: VECTORIZED VS SCALAR DIVERGENCE STRESS TEST ({num_rays:,} RAYS)")
    print("=" * 70)

    obstacles = create_default_disaster_obstacles()
    manager = ObstacleManager(obstacles)

    np.random.seed(42)

    # 1. 70% random rays distributed across theater volume [-250, 250]x[-250, 250]x[0, 120]
    n_rand = int(num_rays * 0.70)
    src_rand = np.random.uniform([-250.0, -250.0, 0.0], [250.0, 250.0, 120.0], size=(n_rand, 3))
    dst_rand = np.random.uniform([-250.0, -250.0, 0.0], [250.0, 250.0, 120.0], size=(n_rand, 3))

    # 2. 20% obstacle-targeted rays (guaranteed hits and near-misses)
    n_targ = int(num_rays * 0.20)
    src_targ = []
    dst_targ = []
    for _ in range(n_targ):
        obs = obstacles[np.random.randint(0, len(obstacles))]
        # Source outside, target inside obstacle
        s = np.random.uniform([-250.0, -250.0, 0.0], [250.0, 250.0, 120.0])
        t = np.random.uniform(obs.min_pt, obs.max_pt)
        src_targ.append(s)
        dst_targ.append(t)
    src_targ = np.array(src_targ)
    dst_targ = np.array(dst_targ)

    # 3. 5% near-parallel/grazing axis-aligned rays
    n_axis = int(num_rays * 0.05)
    src_axis = []
    dst_axis = []
    for _ in range(n_axis):
        obs = obstacles[np.random.randint(0, len(obstacles))]
        axis = np.random.randint(0, 3)
        s = obs.center.copy()
        s[axis] = -240.0
        d = obs.center.copy()
        d[axis] = 240.0
        # Perturb slightly
        offset = (np.random.rand(3) - 0.5) * 5.0
        src_axis.append(s + offset)
        dst_axis.append(d + offset)
    src_axis = np.array(src_axis)
    dst_axis = np.array(dst_axis)

    # 4. 5% zero-length and near-zero length rays
    n_zero = num_rays - (n_rand + n_targ + n_axis)
    src_zero = np.random.uniform([-250.0, -250.0, 0.0], [250.0, 250.0, 120.0], size=(n_zero, 3))
    # Small jitter between 0 and 1e-10
    jitter = np.random.uniform(0.0, 1e-10, size=(n_zero, 3))
    dst_zero = src_zero + jitter

    all_sources = np.vstack([src_rand, src_targ, src_axis, src_zero])
    all_targets = np.vstack([dst_rand, dst_targ, dst_axis, dst_zero])

    total_k = all_sources.shape[0]
    print(f"Generated {total_k} adversarial ray pairs ({n_rand} random, {n_targ} obstacle-targeted, {n_axis} axis-aligned, {n_zero} near-zero length)")

    # Execute Scalar Evaluation
    print(f"Running scalar check_los on {total_k} rays...")
    t0_scalar = time.perf_counter()
    scalar_clear = np.zeros(total_k, dtype=bool)
    scalar_pen = np.zeros(total_k, dtype=np.float64)
    scalar_att = np.zeros(total_k, dtype=np.float64)

    for i in range(total_k):
        c, p, a, _ = manager.check_los(all_sources[i], all_targets[i])
        scalar_clear[i] = c
        scalar_pen[i] = p
        scalar_att[i] = a
    t1_scalar = time.perf_counter()
    scalar_time = t1_scalar - t0_scalar

    # Execute Vectorized Evaluation
    print(f"Running vectorized check_los_batch on {total_k} rays...")
    t0_vec = time.perf_counter()
    vec_clear, vec_pen, vec_att = manager.check_los_batch(all_sources, all_targets)
    t1_vec = time.perf_counter()
    vec_time = t1_vec - t0_vec

    speedup = scalar_time / max(vec_time, 1e-9)

    # Compare
    clear_matches = np.sum(scalar_clear == vec_clear)
    clear_mismatches = total_k - clear_matches

    pen_abs_diff = np.abs(scalar_pen - vec_pen)
    max_pen_diff = float(np.max(pen_abs_diff))
    mean_pen_diff = float(np.mean(pen_abs_diff))

    att_abs_diff = np.abs(scalar_att - vec_att)
    max_att_diff = float(np.max(att_abs_diff))
    mean_att_diff = float(np.mean(att_abs_diff))

    print(f"\n--- Verification Metrics across {total_k:,} Rays ---")
    print(f"  Scalar runtime:     {scalar_time:.4f} s ({total_k / scalar_time:.1f} rays/sec)")
    print(f"  Vectorized runtime: {vec_time:.4f} s ({total_k / vec_time:.1f} rays/sec)")
    print(f"  Vectorized Speedup: {speedup:.2f}x")
    print(f"  Boolean LoS Match:  {clear_matches} / {total_k} ({clear_matches / total_k * 100:.3f}%)")
    print(f"  Boolean Mismatches: {clear_mismatches}")
    print(f"  Penetration Max Diff:  {max_pen_diff:.6e} m")
    print(f"  Penetration Mean Diff: {mean_pen_diff:.6e} m")
    print(f"  Attenuation Max Diff:  {max_att_diff:.6e} dB")
    print(f"  Attenuation Mean Diff: {mean_att_diff:.6e} dB")

    if clear_mismatches > 0:
        bad_idx = np.where(scalar_clear != vec_clear)[0]
        print(f"\nFirst 3 mismatched ray indices: {bad_idx[:3]}")
        for idx in bad_idx[:3]:
            print(f"  Ray {idx}: src={all_sources[idx]}, dst={all_targets[idx]}")
            print(f"    Scalar: clear={scalar_clear[idx]}, pen={scalar_pen[idx]}, att={scalar_att[idx]}")
            print(f"    Vector: clear={vec_clear[idx]}, pen={vec_pen[idx]}, att={vec_att[idx]}")

    return {
        "num_rays": total_k,
        "scalar_time_sec": scalar_time,
        "vector_time_sec": vec_time,
        "speedup": speedup,
        "clear_matches": int(clear_matches),
        "clear_mismatches": int(clear_mismatches),
        "max_pen_diff": max_pen_diff,
        "mean_pen_diff": mean_pen_diff,
        "max_att_diff": max_att_diff,
        "mean_att_diff": mean_att_diff,
    }


def run_swarm_scale_benchmarks() -> Dict[str, Any]:
    print("\n" + "=" * 70)
    print("SUITE 3: LARGE SWARM SCALING BENCHMARKS (25 & 50 DRONES, 500 TICKS)")
    print("=" * 70)

    results = {}

    # Test 3.1: Check whether default enable_downwash=True crashes on 25 drones
    print("\n[3.1 Downwash Defect Reproduction Test (enable_downwash=True)]")
    core_crash = SwarmSimulationCore(SimulationConfig(enable_downwash=True))
    for i in range(25):
        # Spaced out in a grid
        x = -50.0 + (i % 5) * 25.0
        y = -50.0 + (i // 5) * 25.0
        z = 10.0 + (i % 4) * 5.0
        d = Drone(f"UAV_CRASH_{i:02d}", initial_pos=[x, y, z])
        d.set_flight_mode(FlightMode.TRANSIT)
        d.set_target_waypoint([0.0, 0.0, 30.0])  # All fly towards center -> triggers downwash!
        core_crash.add_drone(d)

    crash_occurred = False
    crash_step = None
    crash_error = None
    try:
        for tick in range(100):
            core_crash.step(0.05)
    except NameError as ne:
        crash_occurred = True
        crash_step = core_crash.step_count
        crash_error = str(ne)
        print(f"  CONFIRMED BUG: SwarmSimulationCore crashed at tick {crash_step} with NameError: {crash_error}")
    except Exception as e:
        crash_occurred = True
        crash_step = core_crash.step_count
        crash_error = f"{type(e).__name__}: {str(e)}"
        print(f"  Crashed at tick {crash_step} with {crash_error}")

    results["downwash_crash_test"] = {
        "crash_occurred": crash_occurred,
        "crash_step": crash_step,
        "crash_error": crash_error,
    }

    # Now benchmark with sim.core.math monkey-patched to evaluate true physics & scaling performance
    import sim.core
    sim.core.math = math

    print("\n[3.2 Large Swarm Scaling Benchmark (sim.core.math imported/patched)]")

    for drone_count in [25, 50]:
        print(f"\n--- Benchmarking {drone_count} Drones running 500 ticks (dt=0.05s, 25.0s sim time) ---")
        cfg = SimulationConfig(dt=0.05, enable_downwash=True, enable_vsm_relays=True)
        core = SwarmSimulationCore(config=cfg)

        for obs in create_default_disaster_obstacles():
            core.add_obstacle(obs)

        # Distribute drones across operational area
        np.random.seed(1337)
        for i in range(drone_count):
            role = DroneRole.RELAY if (i % 4 == 0) else DroneRole.SURVEY
            init_x = float(np.random.uniform(-150.0, 150.0))
            init_y = float(np.random.uniform(-150.0, 150.0))
            init_z = float(np.random.uniform(5.0, 40.0))
            d = Drone(f"UAV_{i:02d}", role=role, initial_pos=[init_x, init_y, init_z])
            d.set_flight_mode(FlightMode.TRANSIT)
            # Give target waypoint
            tgt_x = float(np.random.uniform(-200.0, 200.0))
            tgt_y = float(np.random.uniform(-200.0, 200.0))
            tgt_z = float(np.random.uniform(25.0, 45.0) if role == DroneRole.SURVEY else np.random.uniform(70.0, 90.0))
            d.set_target_waypoint([tgt_x, tgt_y, tgt_z])
            core.add_drone(d)

        step_times = []
        t_start_total = time.perf_counter()

        for tick in range(500):
            t0 = time.perf_counter()
            snap = core.step(0.05)
            t1 = time.perf_counter()
            step_times.append(t1 - t0)

        t_end_total = time.perf_counter()
        total_wall_time = t_end_total - t_start_total
        step_times = np.array(step_times)

        mean_step_ms = float(np.mean(step_times) * 1000.0)
        p95_step_ms = float(np.percentile(step_times, 95) * 1000.0)
        max_step_ms = float(np.max(step_times) * 1000.0)
        sim_hz = float(500.0 / total_wall_time)
        realtime_ratio = 25.0 / total_wall_time  # 500 * 0.05 = 25.0s sim duration

        # Verify numerical stability
        all_finite = True
        max_speed = 0.0
        max_accel = 0.0
        for d in core.drones.values():
            if not np.all(np.isfinite(d.position)) or not np.all(np.isfinite(d.velocity)) or not np.all(np.isfinite(d.acceleration)):
                all_finite = False
            spd = float(np.linalg.norm(d.velocity))
            acc = float(np.linalg.norm(d.acceleration))
            max_speed = max(max_speed, spd)
            max_accel = max(max_accel, acc)

        print(f"  Drones:              {drone_count}")
        print(f"  Total Wall Time:     {total_wall_time:.3f} s")
        print(f"  Mean Step Time:      {mean_step_ms:.2f} ms")
        print(f"  P95 Step Time:       {p95_step_ms:.2f} ms")
        print(f"  Max Step Time:       {max_step_ms:.2f} ms")
        print(f"  Effective Sim Rate:  {sim_hz:.1f} Hz (target: 20 Hz)")
        print(f"  Real-time Multiplier:{realtime_ratio:.2f}x faster than real-time")
        print(f"  Numerical Stability: {'PASS (All Finite)' if all_finite else 'FAIL (NaN/Inf detected)'}")
        print(f"  Peak Speed Observed: {max_speed:.2f} m/s (limit: {DroneLimits().max_speed_xy} m/s)")
        print(f"  Peak Accel Observed: {max_accel:.2f} m/s^2 (limit: {DroneLimits().max_accel} m/s^2)")

        results[f"scale_{drone_count}_drones"] = {
            "drones": drone_count,
            "total_wall_time_s": total_wall_time,
            "mean_step_ms": mean_step_ms,
            "p95_step_ms": p95_step_ms,
            "max_step_ms": max_step_ms,
            "sim_hz": sim_hz,
            "realtime_ratio": realtime_ratio,
            "all_finite": all_finite,
            "max_speed": max_speed,
            "max_accel": max_accel,
        }

    return results


def run_determinism_stress_test() -> Dict[str, Any]:
    print("\n" + "=" * 70)
    print("SUITE 4: CROSS-PROCESS DETERMINISTIC REPRODUCIBILITY TEST")
    print("=" * 70)

    # We will write a small python sub-script and execute it in two completely separate python processes.
    subscript_code = """
import sys
import os
import json
import math
import numpy as np

sys.path.insert(0, os.path.abspath(r"{repo_root}"))

import sim.core
sim.core.math = math

from sim.core import SwarmSimulationCore, SimulationConfig
from sim.drone import Drone
from sim.types import DroneRole, FlightMode
from sim.obstacles import create_default_disaster_obstacles

core = SwarmSimulationCore(SimulationConfig(dt=0.05, enable_downwash=True, enable_vsm_relays=True))
for obs in create_default_disaster_obstacles():
    core.add_obstacle(obs)

# Fixed seed
np.random.seed(9999)
for i in range(10):
    role = DroneRole.RELAY if (i % 3 == 0) else DroneRole.SURVEY
    p = np.random.uniform([-100, -100, 10], [100, 100, 40])
    d = Drone(f"DRONE_{i:02d}", role=role, initial_pos=p)
    d.set_flight_mode(FlightMode.TRANSIT)
    t = np.random.uniform([-150, -150, 20], [150, 150, 80])
    d.set_target_waypoint(t)
    core.add_drone(d)

trajectory = []
for tick in range(200):
    snap = core.step(0.05)
    if tick in (0, 50, 100, 199):
        frame = dict()
        for d in core.drones.values():
            frame[d.id] = dict(
                pos=d.position.tolist(),
                vel=d.velocity.tolist(),
                att=d.attitude.tolist(),
                soc=float(d.battery.soc),
            )
        trajectory.append(dict(tick=tick, state=frame))

print(json.dumps(trajectory))
"""

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..")).replace("\\", "/")
    script_content = subscript_code.replace("{repo_root}", repo_root)

    print("Launching Process 1...")
    proc1 = subprocess.run([sys.executable, "-c", script_content], capture_output=True, text=True)
    if proc1.returncode != 0:
        print(f"Process 1 FAILED with stderr:\n{proc1.stderr}")
        return {"error": "Process 1 failed", "stderr": proc1.stderr}

    print("Launching Process 2...")
    proc2 = subprocess.run([sys.executable, "-c", script_content], capture_output=True, text=True)
    if proc2.returncode != 0:
        print(f"Process 2 FAILED with stderr:\n{proc2.stderr}")
        return {"error": "Process 2 failed", "stderr": proc2.stderr}

    traj1 = json.loads(proc1.stdout.strip())
    traj2 = json.loads(proc2.stdout.strip())

    # Deep bit-for-bit comparison
    ticks_compared = len(traj1)
    identical = (traj1 == traj2)
    max_diff = 0.0

    for f1, f2 in zip(traj1, traj2):
        t = f1["tick"]
        s1 = f1["state"]
        s2 = f2["state"]
        for did in s1:
            p1 = np.array(s1[did]["pos"])
            p2 = np.array(s2[did]["pos"])
            v1 = np.array(s1[did]["vel"])
            v2 = np.array(s2[did]["vel"])
            diff = max(np.max(np.abs(p1 - p2)), np.max(np.abs(v1 - v2)))
            max_diff = max(max_diff, diff)

    print(f"Compared {ticks_compared} keyframe snapshots across 10 drones (200 ticks)")
    print(f"Strict Identical Equality: {identical}")
    print(f"Maximum Coordinate Difference: {max_diff:.10e}")

    return {
        "strictly_identical": identical,
        "max_diff": max_diff,
        "ticks_sampled": [0, 50, 100, 199],
        "drones": 10,
    }


def main():
    print("=" * 80)
    print("CHALLENGER M1-2: GEOMETRY, SCALE & DETERMINISM ADVERSARIAL STRESS HARNESS")
    print("=" * 80)

    res_geo = run_degenerate_geometry_tests()
    res_vec = run_vectorized_vs_scalar_stress_test(10000)
    res_scale = run_swarm_scale_benchmarks()
    res_det = run_determinism_stress_test()

    print("\n" + "=" * 80)
    print("ALL STRESS TEST PHASES COMPLETED.")
    print("=" * 80)


if __name__ == "__main__":
    main()
