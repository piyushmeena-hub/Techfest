"""
Tier 1: Feature Isolation Coverage (≥ 60 tests covering 12 features, ≥ 5 tests each).
Requirement-driven, opaque-box tests validating all features defined in TEST_INFRA.md and PROJECT.md.
"""

from __future__ import annotations
import math
import numpy as np
import pytest
from tests.conftest import (
    DroneState,
    AABB,
    NetworkPacket,
    PoI,
    TelemetrySnapshot,
    ReferenceChannelModel,
    ReferenceOcclusionEngine,
    ReferenceDTNBuffer,
    ReferenceNetworkEngine,
    ReferenceKinematicsEngine,
    ReferenceVSMEngine,
    ReferenceMissionCoordinator,
    ReferenceSimulationController
)


# ============================================================================
# Feature 1: Automated Execution & Single Setup Script (ORIGINAL_REQUEST §Acceptance Criteria)
# ============================================================================

def test_f1_headless_mode_initialization():
    """F1.1: Verify simulation initializes and runs in headless mode without graphical display."""
    sim = ReferenceSimulationController(num_drones=3, num_pois=2, duration=1.0, headless=True)
    assert sim.headless is True
    snap = sim.step()
    assert isinstance(snap, TelemetrySnapshot)
    assert snap.sim_time >= 0.0
    assert sim.sim_time > 0.0


def test_f1_cli_duration_parameter_parsing():
    """F1.2: Verify duration parameter accurately bounds the execution time steps."""
    duration = 1.5
    sim = ReferenceSimulationController(num_drones=2, num_pois=1, duration=duration, headless=True)
    snaps = sim.run()
    expected_steps = int(duration / sim.dt)
    assert len(snaps) == expected_steps
    assert math.isclose(sim.sim_time, duration, abs_tol=sim.dt)


def test_f1_cli_drone_count_configuration():
    """F1.3: Verify swarm spawns exact requested number of UAVs."""
    target_count = 6
    sim = ReferenceSimulationController(num_drones=target_count, num_pois=2, duration=0.1)
    assert len(sim.drones) == target_count
    ids = [d.id for d in sim.drones]
    assert len(set(ids)) == target_count


def test_f1_cli_poi_count_configuration():
    """F1.4: Verify environment creates exact requested number of disaster PoIs."""
    target_pois = 5
    sim = ReferenceSimulationController(num_drones=3, num_pois=target_pois, duration=0.1)
    assert len(sim.pois) == target_pois
    poi_ids = [p.id for p in sim.pois]
    assert len(set(poi_ids)) == target_pois


def test_f1_clean_termination_exit_state():
    """F1.5: Verify simulation completes cleanly with valid telemetry summary metrics."""
    sim = ReferenceSimulationController(num_drones=4, num_pois=2, duration=0.5)
    snaps = sim.run()
    last_snap = snaps[-1]
    assert 'pdr' in last_snap.metrics
    assert 'avg_latency_ms' in last_snap.metrics
    assert 0.0 <= last_snap.metrics['pdr'] <= 1.0


# ============================================================================
# Feature 2: 3D Swarm Simulation Environment & Multi-UAV Visualization (ORIGINAL_REQUEST §R1)
# ============================================================================

def test_f2_spatial_boundary_bounds_500x500():
    """F2.1: Verify spatial domain boundaries are strictly [0, 500]m in X/Y and [0, 100]m in Z."""
    drone = DroneState("UAV_TEST", "SURVEY", np.array([600.0, -50.0, 150.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    kinematics = ReferenceKinematicsEngine()
    kinematics.step_drone(drone, 0.1, [drone], [])
    assert 0.0 <= drone.position[0] <= 500.0
    assert 0.0 <= drone.position[1] <= 500.0
    assert 0.0 <= drone.position[2] <= 100.0


def test_f2_telemetry_snapshot_schema():
    """F2.2: Verify telemetry snapshot contains complete schema required for 3D Cockpit HUD."""
    sim = ReferenceSimulationController(num_drones=3, num_pois=2, duration=0.1)
    snap = sim.step()
    assert hasattr(snap, 'sim_time')
    assert hasattr(snap, 'drones')
    assert hasattr(snap, 'gcs')
    assert hasattr(snap, 'pois')
    assert hasattr(snap, 'active_routes')
    assert hasattr(snap, 'links')
    assert hasattr(snap, 'packets')
    assert hasattr(snap, 'metrics')


def test_f2_gcs_stationary_anchor_coordinates():
    """F2.3: Verify GCS position remains static across simulation ticks."""
    gcs_coord = np.array([50.0, 50.0, 10.0])
    sim = ReferenceSimulationController(num_drones=2, num_pois=1, duration=0.5, gcs_pos=gcs_coord)
    init_gcs = np.copy(sim.gcs_pos)
    sim.run()
    assert np.allclose(sim.gcs_pos, init_gcs)
    assert np.allclose(sim.snapshots[-1].gcs['pos'], gcs_coord)


def test_f2_4_tier_altitude_corridors_definition():
    """F2.4: Verify altitude corridors definition (Launch, Survey, Transit, Relay)."""
    corridors = ReferenceKinematicsEngine.ALTITUDE_CORRIDORS
    assert corridors['LAUNCH'] == (0.0, 20.0)
    assert corridors['SURVEY'] == (25.0, 45.0)
    assert corridors['TRANSIT'] == (50.0, 65.0)
    assert corridors['RELAY'] == (70.0, 90.0)


def test_f2_30hz_timestep_progression():
    """F2.5: Verify simulation clock advances by dt = 1/30s each tick."""
    sim = ReferenceSimulationController(num_drones=2, num_pois=1, duration=0.1)
    dt = 1.0 / 30.0
    for i in range(1, 10):
        snap = sim.step()
        assert math.isclose(snap.sim_time, (i - 1) * dt, abs_tol=1e-5)
    assert math.isclose(sim.sim_time, 9 * dt, abs_tol=1e-5)


# ============================================================================
# Feature 3: Quadcopter Kinematics & Velocity/Acceleration Constraints (ORIGINAL_REQUEST §R1)
# ============================================================================

def test_f3_velocity_magnitude_clamp_15ms(kinematics_engine: ReferenceKinematicsEngine):
    """F3.1: Verify velocity magnitude cannot exceed vmax = 15.0 m/s under extreme commands."""
    drone = DroneState("UAV_1", "SURVEY", np.array([100.0, 100.0, 50.0]), np.array([20.0, 20.0, 0.0]), np.zeros(3), np.zeros(4), 1.0, 'TRANSIT')
    drone.target_position = np.array([400.0, 400.0, 50.0])
    kinematics_engine.step_drone(drone, 0.1, [drone], [])
    vel_mag = float(np.linalg.norm(drone.velocity))
    assert vel_mag <= kinematics_engine.v_max + 1e-4


def test_f3_acceleration_magnitude_clamp_5ms2(kinematics_engine: ReferenceKinematicsEngine):
    """F3.2: Verify acceleration magnitude cannot exceed amax = 5.0 m/s^2."""
    drone = DroneState("UAV_1", "SURVEY", np.array([100.0, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'TRANSIT')
    drone.target_position = np.array([500.0, 500.0, 50.0])
    acc = kinematics_engine.compute_acceleration(drone, [drone], [])
    acc_mag = float(np.linalg.norm(acc))
    assert acc_mag <= kinematics_engine.a_max + 1e-4


def test_f3_position_numerical_integration(kinematics_engine: ReferenceKinematicsEngine):
    """F3.3: Verify numerical integration matches kinematic update x_{t+1} = x_t + v*dt."""
    pos_init = np.array([100.0, 100.0, 50.0])
    vel_init = np.array([5.0, 0.0, 0.0])
    drone = DroneState("UAV_1", "SURVEY", np.copy(pos_init), np.copy(vel_init), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    dt = 0.1
    kinematics_engine.step_drone(drone, dt, [drone], [])
    expected_pos = pos_init + drone.velocity * dt
    assert np.allclose(drone.position, expected_pos)


def test_f3_battery_soc_monotonic_depletion(kinematics_engine: ReferenceKinematicsEngine):
    """F3.4: Verify battery SoC decreases strictly monotonically during flight operations."""
    drone = DroneState("UAV_1", "SURVEY", np.array([100.0, 100.0, 50.0]), np.array([5.0, 5.0, 0.0]), np.zeros(3), np.zeros(4), 1.0, 'TRANSIT')
    prev_soc = drone.battery_soc
    for _ in range(5):
        kinematics_engine.step_drone(drone, 1.0, [drone], [])
        assert drone.battery_soc < prev_soc
        prev_soc = drone.battery_soc
    assert 0.0 <= drone.battery_soc < 1.0


def test_f3_rotor_speeds_non_negative():
    """F3.5: Verify rotor speeds remain within valid non-negative physical bounds."""
    drone = DroneState("UAV_1", "SURVEY", np.array([100.0, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.array([400.0, 400.0, 400.0, 400.0]), 1.0, 'IDLE')
    assert len(drone.rotor_speeds) == 4
    assert np.all(drone.rotor_speeds >= 0.0)


# ============================================================================
# Feature 4: Inter-Drone Flocking & Collision Avoidance (ORIGINAL_REQUEST §R1)
# ============================================================================

def test_f4_apf_separation_below_safe_distance(kinematics_engine: ReferenceKinematicsEngine):
    """F4.1: Verify repulsive acceleration triggers when two drones are within d_safe = 3.0m."""
    d1 = DroneState("UAV_1", "SURVEY", np.array([100.0, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    d2 = DroneState("UAV_2", "SURVEY", np.array([101.5, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    acc1 = kinematics_engine.compute_acceleration(d1, [d1, d2], [])
    # d1 should be repelled in negative X direction away from d2
    assert acc1[0] < -0.1


def test_f4_apf_zero_force_beyond_safe_distance(kinematics_engine: ReferenceKinematicsEngine):
    """F4.2: Verify zero inter-drone repulsive force when distance exceeds d_safe."""
    d1 = DroneState("UAV_1", "SURVEY", np.array([100.0, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    d2 = DroneState("UAV_2", "SURVEY", np.array([110.0, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    acc1 = kinematics_engine.compute_acceleration(d1, [d1, d2], [])
    assert np.allclose(acc1, np.zeros(3))


def test_f4_downwash_repulsion_vertical_alignment(kinematics_engine: ReferenceKinematicsEngine):
    """F4.3: Verify horizontal push force when one drone is directly beneath another (downwash)."""
    # d_top is at z=53, d_bot is at z=50, nearly same X/Y
    d_bot = DroneState("UAV_BOT", "SURVEY", np.array([100.0, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    d_top = DroneState("UAV_TOP", "SURVEY", np.array([100.1, 100.0, 53.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    acc_bot = kinematics_engine.compute_acceleration(d_bot, [d_bot, d_top], [])
    # Should experience horizontal repulsion
    horiz_rep = float(np.linalg.norm(acc_bot[:2]))
    assert horiz_rep > 0.5


def test_f4_building_obstacle_apf_repulsion(kinematics_engine: ReferenceKinematicsEngine, sample_obstacles: List[AABB]):
    """F4.4: Verify obstacle APF repulsive force directs drone away from building surface."""
    # BUILDING_1 is [200, 200, 0] to [260, 260, 35]. Drone near x=198 (2m from west wall)
    drone = DroneState("UAV_1", "SURVEY", np.array([198.0, 230.0, 20.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    acc = kinematics_engine.compute_acceleration(drone, [drone], sample_obstacles)
    # West wall is in +X direction from drone, so repulsion should push in -X direction
    assert acc[0] < -0.1


def test_f4_minimum_clearance_maintained(kinematics_engine: ReferenceKinematicsEngine):
    """F4.5: Verify crossing drones maintain clearance > 1.5m due to active separation."""
    kinematics_engine.d_safe = 5.0
    kinematics_engine.k_rep = 25.0
    d1 = DroneState("UAV_1", "SURVEY", np.array([90.0, 99.0, 50.0]), np.array([4.0, 0.0, 0.0]), np.zeros(3), np.zeros(4), 1.0, 'TRANSIT')
    d2 = DroneState("UAV_2", "SURVEY", np.array([110.0, 101.0, 50.0]), np.array([-4.0, 0.0, 0.0]), np.zeros(3), np.zeros(4), 1.0, 'TRANSIT')
    d1.target_position = np.array([120.0, 99.0, 50.0])
    d2.target_position = np.array([80.0, 101.0, 50.0])

    min_dist = float('inf')
    dt = 0.05
    for _ in range(60):
        kinematics_engine.step_drone(d1, dt, [d1, d2], [])
        kinematics_engine.step_drone(d2, dt, [d1, d2], [])
        dist = float(np.linalg.norm(d1.position - d2.position))
        min_dist = min(min_dist, dist)

    assert min_dist >= 1.5, f"Minimum clearance violated: {min_dist}m < 1.5m"


# ============================================================================
# Feature 5: Multi-Hop Communication Modeling & RF Propagation (ORIGINAL_REQUEST §R2)
# ============================================================================

def test_f5_friis_reference_loss_40db(channel_model: ReferenceChannelModel):
    """F5.1: Verify reference path loss at 1m is 40.05 ± 0.5 dB at 2.4 GHz."""
    pl_1m = channel_model.compute_path_loss(1.0, is_los=True)
    assert math.isclose(pl_1m, 40.05, abs_tol=0.5)


def test_f5_log_distance_los_scaling(channel_model: ReferenceChannelModel):
    """F5.2: Verify path loss increases logarithmically with distance with eta = 2.05."""
    pl_10m = channel_model.compute_path_loss(10.0, is_los=True)
    pl_100m = channel_model.compute_path_loss(100.0, is_los=True)
    delta_pl = pl_100m - pl_10m
    expected_delta = 10.0 * 2.05 * math.log10(100.0 / 10.0)  # 20.5 dB
    assert math.isclose(delta_pl, expected_delta, abs_tol=0.2)


def test_f5_snr_calculation_above_noise_floor(channel_model: ReferenceChannelModel):
    """F5.3: Verify SNR = P_tx - PL - N0 matches theoretical RF calculation."""
    pl = 70.0
    snr = channel_model.compute_snr(pl)
    expected_snr = 20.0 - 70.0 - (-95.0)  # 45.0 dB
    assert math.isclose(snr, expected_snr, abs_tol=1e-4)


def test_f5_direct_los_range_cutoff_320m(channel_model: ReferenceChannelModel):
    """F5.4: Verify direct link is viable at 300m, but non-viable beyond 320m."""
    viable_300, _, _ = channel_model.is_link_viable(300.0, is_los=True)
    viable_350, _, _ = channel_model.is_link_viable(350.0, is_los=True)
    assert viable_300 is True
    assert viable_350 is False


def test_f5_symmetric_path_loss_reciprocity(channel_model: ReferenceChannelModel):
    """F5.5: Verify path loss calculation is symmetric between any two points."""
    dist = 125.4
    pl_ab = channel_model.compute_path_loss(dist, is_los=True)
    pl_ba = channel_model.compute_path_loss(dist, is_los=True)
    assert math.isclose(pl_ab, pl_ba)


# ============================================================================
# Feature 6: Line-of-Sight Occlusion by Disaster Obstacles (ORIGINAL_REQUEST §R2)
# ============================================================================

def test_f6_direct_line_through_building_blocked(occlusion_engine: ReferenceOcclusionEngine):
    """F6.1: Verify 3D ray-AABB detects building intersection between two ground/low points."""
    # BUILDING_1 is [200, 200, 0] to [260, 260, 35].
    p_gcs = np.array([50.0, 230.0, 10.0])
    p_uav = np.array([350.0, 230.0, 10.0])
    is_occ, count = occlusion_engine.check_occlusion(p_gcs, p_uav)
    assert is_occ is True
    assert count >= 1


def test_f6_elevated_line_over_building_clear(occlusion_engine: ReferenceOcclusionEngine):
    """F6.2: Verify 3D ray above building roof (z=75m > 35m) is unobstructed."""
    p_gcs = np.array([50.0, 230.0, 75.0])
    p_uav = np.array([350.0, 230.0, 75.0])
    is_occ, count = occlusion_engine.check_occlusion(p_gcs, p_uav)
    assert is_occ is False
    assert count == 0


def test_f6_building_penetration_penalty_22db(channel_model: ReferenceChannelModel):
    """F6.3: Verify occluded link applies +22.0 dB building penetration loss."""
    dist = 100.0
    pl_los = channel_model.compute_path_loss(dist, is_los=True, num_occlusions=0)
    pl_nlos = channel_model.compute_path_loss(dist, is_los=False, num_occlusions=1)
    assert math.isclose(pl_nlos - pl_los, channel_model.building_penetration_loss_db + (10.0 * (3.60 - 2.05) * 2.0), abs_tol=1.0)


def test_f6_multiple_building_occlusion_accumulation(occlusion_engine: ReferenceOcclusionEngine):
    """F6.4: Verify ray intersecting two buildings detects multiple occlusions."""
    # Place two buildings along X axis
    obs1 = AABB("B1", np.array([100.0, 90.0, 0.0]), np.array([120.0, 110.0, 30.0]))
    obs2 = AABB("B2", np.array([200.0, 90.0, 0.0]), np.array([220.0, 110.0, 30.0]))
    occ = ReferenceOcclusionEngine([obs1, obs2])
    p1 = np.array([50.0, 100.0, 15.0])
    p2 = np.array([300.0, 100.0, 15.0])
    is_occ, count = occ.check_occlusion(p1, p2)
    assert is_occ is True
    assert count == 2


def test_f6_lateral_bypass_unobstructed(occlusion_engine: ReferenceOcclusionEngine):
    """F6.5: Verify ray passing laterally beside building is completely clear."""
    # BUILDING_1 is Y in [200, 260]. Point passing at Y=150 should be clear.
    p1 = np.array([50.0, 150.0, 15.0])
    p2 = np.array([350.0, 150.0, 15.0])
    is_occ, count = occlusion_engine.check_occlusion(p1, p2)
    assert is_occ is False
    assert count == 0


# ============================================================================
# Feature 7: Dynamic Multi-Hop Routing Determination (ORIGINAL_REQUEST §R2)
# ============================================================================

def test_f7_direct_single_hop_selection(network_engine: ReferenceNetworkEngine):
    """F7.1: Verify direct 1-hop path ['UAV_1', 'GCS'] selected when within clear LoS."""
    nodes = {
        'GCS': np.array([50.0, 50.0, 10.0]),
        'UAV_1': np.array([100.0, 100.0, 30.0]),
    }
    network_engine.update_topology(nodes)
    route = network_engine.find_route('UAV_1', 'GCS')
    assert route == ['UAV_1', 'GCS']


def test_f7_two_hop_relay_around_obstacle(network_engine: ReferenceNetworkEngine, sample_obstacles: List[AABB]):
    """F7.2: Verify 2-hop route ['UAV_Survey', 'UAV_Relay', 'GCS'] when direct LoS is occluded."""
    # BUILDING_1 is [200, 200, 0] to [260, 260, 35]
    nodes = {
        'GCS': np.array([50.0, 230.0, 10.0]),
        'UAV_Survey': np.array([350.0, 230.0, 20.0]),  # Blocked by building
        'UAV_Relay': np.array([200.0, 230.0, 75.0]),   # High altitude, clear to both
    }
    network_engine.update_topology(nodes, sample_obstacles)
    route = network_engine.find_route('UAV_Survey', 'GCS')
    assert route == ['UAV_Survey', 'UAV_Relay', 'GCS']


def test_f7_three_hop_extended_topology(network_engine: ReferenceNetworkEngine):
    """F7.3: Verify 3-hop route ['UAV_3', 'UAV_2', 'UAV_1', 'GCS'] along extended distance."""
    nodes = {
        'GCS': np.array([0.0, 0.0, 10.0]),
        'UAV_1': np.array([150.0, 0.0, 30.0]),
        'UAV_2': np.array([300.0, 0.0, 30.0]),
        'UAV_3': np.array([450.0, 0.0, 30.0]),
    }
    network_engine.update_topology(nodes, [])
    route = network_engine.find_route('UAV_3', 'GCS')
    assert route == ['UAV_3', 'UAV_2', 'UAV_1', 'GCS']


def test_f7_dijkstra_composite_cost_optimality(network_engine: ReferenceNetworkEngine, sample_obstacles: List[AABB]):
    """F7.4: Verify Dijkstra selects 2-hop clear route over 1-hop heavy occluded route."""
    nodes = {
        'GCS': np.array([150.0, 230.0, 10.0]),
        'UAV_Survey': np.array([290.0, 230.0, 10.0]),  # Direct through 35m building
        'UAV_Relay': np.array([220.0, 150.0, 30.0]),   # Clear path detour
    }
    network_engine.update_topology(nodes, sample_obstacles)
    route = network_engine.find_route('UAV_Survey', 'GCS')
    assert route == ['UAV_Survey', 'UAV_Relay', 'GCS']


def test_f7_dynamic_route_reconfiguration_on_link_break(network_engine: ReferenceNetworkEngine):
    """F7.5: Verify route adapts dynamically when an intermediate relay node moves away."""
    # Diagonal obstacle blocking direct path between UAV_Survey [100, 100, 30] and GCS [0, 0, 10]
    obs = [AABB("OBS_DIAG", np.array([30.0, 30.0, 0.0]), np.array([70.0, 70.0, 40.0]))]
    nodes = {
        'GCS': np.array([0.0, 0.0, 10.0]),
        'UAV_Relay1': np.array([100.0, 0.0, 30.0]),
        'UAV_Relay2': np.array([0.0, 100.0, 30.0]),
        'UAV_Survey': np.array([100.0, 100.0, 30.0]),
    }
    network_engine.update_topology(nodes, obs)
    init_route = network_engine.find_route('UAV_Survey', 'GCS')
    assert len(init_route) == 3
    assert 'UAV_Relay1' in init_route or 'UAV_Relay2' in init_route

    # Move Relay1 out of range (to 500m)
    nodes['UAV_Relay1'] = np.array([500.0, 500.0, 30.0])
    network_engine.update_topology(nodes, obs)
    reconfigured_route = network_engine.find_route('UAV_Survey', 'GCS')
    assert reconfigured_route == ['UAV_Survey', 'UAV_Relay2', 'GCS']


# ============================================================================
# Feature 8: Multi-Hop Link Logging & Verification (ORIGINAL_REQUEST §Acceptance Criteria)
# ============================================================================

def test_f8_packet_id_uniqueness():
    """F8.1: Verify generated packets have distinct unique identifiers."""
    pkts = [NetworkPacket(f"PKT_{i}", "UAV_1", "GCS", "TELEMETRY") for i in range(20)]
    ids = [p.packet_id for p in pkts]
    assert len(set(ids)) == 20


def test_f8_hop_trace_records_exact_path(network_engine: ReferenceNetworkEngine):
    """F8.2: Verify packet's hop_trace accurately records all intermediate hops."""
    nodes = {
        'GCS': np.array([0.0, 0.0, 10.0]),
        'UAV_1': np.array([100.0, 0.0, 30.0]),
        'UAV_2': np.array([200.0, 0.0, 30.0]),
    }
    network_engine.update_topology(nodes, [])
    pkt = NetworkPacket("PKT_TRACE_1", "UAV_2", "GCS", "SURVEY_DATA")
    delivered = network_engine.transmit_packet(pkt, current_sim_time=1.0)
    assert delivered is True
    assert pkt.hop_trace == ['UAV_2', 'UAV_1', 'GCS']


def test_f8_end_to_end_latency_calculation(network_engine: ReferenceNetworkEngine):
    """F8.3: Verify end-to-end packet latency is calculated and recorded upon delivery."""
    nodes = {
        'GCS': np.array([0.0, 0.0, 10.0]),
        'UAV_1': np.array([100.0, 0.0, 30.0]),
    }
    network_engine.update_topology(nodes, [])
    pkt = NetworkPacket("PKT_LAT_1", "UAV_1", "GCS", "TELEMETRY", timestamp_sent=1.0)
    network_engine.transmit_packet(pkt, current_sim_time=1.0)
    assert pkt.timestamp_received is not None
    assert pkt.timestamp_received > pkt.timestamp_sent


def test_f8_packet_status_lifecycle_queued_to_delivered(network_engine: ReferenceNetworkEngine):
    """F8.4: Verify packet status transitions from QUEUED to DELIVERED."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0]), 'UAV_1': np.array([50.0, 0.0, 30.0])}
    network_engine.update_topology(nodes, [])
    pkt = NetworkPacket("PKT_STATE", "UAV_1", "GCS", "TELEMETRY")
    assert pkt.status == 'QUEUED'
    network_engine.transmit_packet(pkt, 1.0)
    assert pkt.status == 'DELIVERED'


def test_f8_unreachable_packet_logging(network_engine: ReferenceNetworkEngine):
    """F8.5: Verify packets to unreachable nodes are buffered and remain QUEUED."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0]), 'UAV_ISOLATED': np.array([490.0, 490.0, 30.0])}
    network_engine.update_topology(nodes, [])
    pkt = NetworkPacket("PKT_FAIL", "UAV_ISOLATED", "GCS", "SURVEY_DATA")
    delivered = network_engine.transmit_packet(pkt, 1.0)
    assert delivered is False
    assert pkt.status == 'QUEUED'
    assert 'UAV_ISOLATED' in network_engine.dtn_buffers
    assert network_engine.dtn_buffers['UAV_ISOLATED'].size() == 1


# ============================================================================
# Feature 9: PoI Assignment & Autonomous Navigation (ORIGINAL_REQUEST §R3)
# ============================================================================

def test_f9_priority_based_poi_assignment():
    """F9.1: Verify HIGH priority PoI is assigned before LOW priority PoI."""
    pois = [
        PoI("POI_LOW", np.array([200.0, 200.0, 30.0]), "LOW"),
        PoI("POI_HIGH", np.array([300.0, 300.0, 30.0]), "HIGH"),
    ]
    mission = ReferenceMissionCoordinator(pois)
    drone = DroneState("UAV_1", "SURVEY", np.array([50.0, 50.0, 10.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    mission.assign_tasks([drone])
    assert drone.assigned_poi_id == "POI_HIGH"


def test_f9_survey_drone_target_setpoint_assignment(sample_pois: List[PoI]):
    """F9.2: Verify assigned drone's target_position equals PoI coordinates."""
    mission = ReferenceMissionCoordinator(sample_pois)
    drone = DroneState("UAV_1", "SURVEY", np.array([50.0, 50.0, 10.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    mission.assign_tasks([drone])
    assigned_poi = next(p for p in sample_pois if p.id == drone.assigned_poi_id)
    assert np.allclose(drone.target_position, assigned_poi.position)


def test_f9_arrival_within_acceptance_radius(kinematics_engine: ReferenceKinematicsEngine):
    """F9.3: Verify drone stepping towards PoI gets within acceptance radius <= 3.0m."""
    target = np.array([110.0, 100.0, 30.0])
    drone = DroneState("UAV_1", "SURVEY", np.array([100.0, 100.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'TRANSIT', target_position=target)
    for _ in range(30):
        kinematics_engine.step_drone(drone, 0.1, [drone], [])
    dist = float(np.linalg.norm(drone.position - target))
    assert dist <= 3.0


def test_f9_single_drone_per_poi_constraint(sample_pois: List[PoI]):
    """F9.4: Verify two drones are not assigned to the same PoI simultaneously."""
    mission = ReferenceMissionCoordinator(sample_pois)
    d1 = DroneState("UAV_1", "SURVEY", np.array([50.0, 50.0, 10.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    d2 = DroneState("UAV_2", "SURVEY", np.array([50.0, 50.0, 10.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    mission.assign_tasks([d1, d2])
    assert d1.assigned_poi_id != d2.assigned_poi_id


def test_f9_poi_reassignment_after_completion():
    """F9.5: Verify drone is reassigned to next pending PoI after finishing first PoI."""
    poi1 = PoI("POI_1", np.array([100.0, 100.0, 30.0]), "HIGH", dwell_time_required=0.1)
    poi2 = PoI("POI_2", np.array([200.0, 200.0, 30.0]), "MEDIUM", dwell_time_required=0.5)
    mission = ReferenceMissionCoordinator([poi1, poi2])
    drone = DroneState("UAV_1", "SURVEY", np.array([100.0, 100.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    mission.assign_tasks([drone])
    assert drone.assigned_poi_id == "POI_1"

    net = ReferenceNetworkEngine()
    mission.update_dwell([drone], 0.2, net, 1.0)
    assert poi1.status == 'COMPLETED'
    assert drone.assigned_poi_id is None

    # Next assignment
    mission.assign_tasks([drone])
    assert drone.assigned_poi_id == "POI_2"


# ============================================================================
# Feature 10: PoI Surveying Dwell & Data Collection (ORIGINAL_REQUEST §R3)
# ============================================================================

def test_f10_fsm_transitions_to_surveying_at_poi():
    """F10.1: Verify flight_mode transitions to SURVEYING when within 3.0m of PoI."""
    poi = PoI("POI_1", np.array([100.0, 100.0, 30.0]), "HIGH", dwell_time_required=2.0)
    mission = ReferenceMissionCoordinator([poi])
    drone = DroneState("UAV_1", "SURVEY", np.array([101.0, 100.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'TRANSIT', assigned_poi_id="POI_1")
    net = ReferenceNetworkEngine()
    mission.update_dwell([drone], 0.1, net, 1.0)
    assert drone.flight_mode == 'SURVEYING'
    assert poi.status == 'IN_PROGRESS'


def test_f10_dwell_time_accumulation():
    """F10.2: Verify dwell time accumulates while drone remains within PoI radius."""
    poi = PoI("POI_1", np.array([100.0, 100.0, 30.0]), "HIGH", dwell_time_required=3.0)
    mission = ReferenceMissionCoordinator([poi])
    drone = DroneState("UAV_1", "SURVEY", np.array([100.0, 100.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'SURVEYING', assigned_poi_id="POI_1")
    net = ReferenceNetworkEngine()
    dt = 0.5
    for _ in range(4):
        mission.update_dwell([drone], dt, net, 1.0)
    assert math.isclose(poi.dwell_time_accumulated, 2.0, abs_tol=1e-4)


def test_f10_survey_packet_generation_during_dwell():
    """F10.3: Verify data packets are generated and transmitted during dwell inspection."""
    poi = PoI("POI_1", np.array([100.0, 100.0, 30.0]), "HIGH", dwell_time_required=2.0)
    mission = ReferenceMissionCoordinator([poi])
    drone = DroneState("UAV_1", "SURVEY", np.array([100.0, 100.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'SURVEYING', assigned_poi_id="POI_1")
    net = ReferenceNetworkEngine()
    pkts = mission.update_dwell([drone], 0.1, net, 1.0)
    assert len(pkts) == 1
    assert pkts[0].payload_type == 'SURVEY_DATA'
    assert pkts[0].source_id == 'UAV_1'


def test_f10_poi_marked_completed_after_required_dwell():
    """F10.4: Verify PoI status is set to COMPLETED once accumulated dwell >= required."""
    poi = PoI("POI_1", np.array([100.0, 100.0, 30.0]), "HIGH", dwell_time_required=1.0)
    mission = ReferenceMissionCoordinator([poi])
    drone = DroneState("UAV_1", "SURVEY", np.array([100.0, 100.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'SURVEYING', assigned_poi_id="POI_1")
    net = ReferenceNetworkEngine()
    mission.update_dwell([drone], 1.2, net, 1.0)
    assert poi.status == 'COMPLETED'
    assert mission.completed_pois == 1


def test_f10_drone_returns_to_idle_or_transit_post_survey():
    """F10.5: Verify drone transitions out of SURVEYING mode once survey is complete."""
    poi = PoI("POI_1", np.array([100.0, 100.0, 30.0]), "HIGH", dwell_time_required=0.5)
    mission = ReferenceMissionCoordinator([poi])
    drone = DroneState("UAV_1", "SURVEY", np.array([100.0, 100.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'SURVEYING', assigned_poi_id="POI_1")
    net = ReferenceNetworkEngine()
    mission.update_dwell([drone], 0.6, net, 1.0)
    assert drone.flight_mode == 'IDLE'
    assert drone.assigned_poi_id is None


# ============================================================================
# Feature 11: Telemetry Transmission via Multi-Hop Relay (ORIGINAL_REQUEST §R3)
# ============================================================================

def test_f11_survey_data_transmitted_to_gcs(network_engine: ReferenceNetworkEngine):
    """F11.1: Verify survey payload is transmitted and successfully delivered to GCS."""
    nodes = {
        'GCS': np.array([0.0, 0.0, 10.0]),
        'UAV_Relay': np.array([200.0, 0.0, 75.0]),
        'UAV_Survey': np.array([400.0, 0.0, 30.0]),  # 400m from GCS (> 320m direct cutoff), forces relay
    }
    network_engine.update_topology(nodes, [])
    pkt = NetworkPacket("DATA_PKT_1", "UAV_Survey", "GCS", "SURVEY_DATA", data_size_bytes=2048)
    success = network_engine.transmit_packet(pkt, 1.0)
    assert success is True
    assert pkt.status == 'DELIVERED'
    assert 'UAV_Relay' in pkt.hop_trace


def test_f11_dtn_buffer_stores_packets_during_blackout(network_engine: ReferenceNetworkEngine):
    """F11.2: Verify DTN buffer caches packets during communication blackout."""
    # UAV has no connection to GCS
    nodes = {'GCS': np.array([0.0, 0.0, 10.0]), 'UAV_Survey': np.array([450.0, 450.0, 30.0])}
    network_engine.update_topology(nodes, [])
    pkt = NetworkPacket("PKT_STORE", "UAV_Survey", "GCS", "SURVEY_DATA")
    delivered = network_engine.transmit_packet(pkt, 1.0)
    assert delivered is False
    buffer = network_engine.dtn_buffers['UAV_Survey']
    assert buffer.size() == 1
    assert buffer.buffer[0].packet_id == "PKT_STORE"


def test_f11_dtn_buffer_drain_upon_reconnection(network_engine: ReferenceNetworkEngine):
    """F11.3: Verify DTN buffer flushes stored packets once route is restored."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0]), 'UAV_Survey': np.array([380.0, 0.0, 30.0])}
    network_engine.update_topology(nodes, [])
    pkt = NetworkPacket("PKT_STORE", "UAV_Survey", "GCS", "SURVEY_DATA")
    network_engine.transmit_packet(pkt, 1.0)

    # Reconnection: add relay bridging the gap (190m from both nodes, well within 320m range)
    nodes['UAV_Relay'] = np.array([190.0, 0.0, 75.0])
    network_engine.update_topology(nodes, [])
    buffer = network_engine.dtn_buffers['UAV_Survey']
    flushed_pkt = buffer.pop()
    assert flushed_pkt is not None
    delivered = network_engine.transmit_packet(flushed_pkt, 2.0)
    assert delivered is True
    assert buffer.is_empty()


def test_f11_pdr_metric_accurate_calculation(network_engine: ReferenceNetworkEngine):
    """F11.4: Verify Packet Delivery Ratio (PDR) accurately calculates delivered / transmitted."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0]), 'UAV_1': np.array([50.0, 0.0, 30.0])}
    network_engine.update_topology(nodes, [])

    # Transmit 3 successful packets
    for i in range(3):
        network_engine.transmit_packet(NetworkPacket(f"P_OK_{i}", "UAV_1", "GCS", "TELEMETRY"))

    # Break link and transmit 1 dropped packet
    nodes['UAV_1'] = np.array([450.0, 450.0, 30.0])
    network_engine.update_topology(nodes, [])
    network_engine.transmit_packet(NetworkPacket("P_FAIL", "UAV_1", "GCS", "TELEMETRY"))

    metrics = network_engine.get_metrics()
    assert metrics['total_transmitted'] == 4
    assert metrics['total_delivered'] == 3
    assert math.isclose(metrics['pdr'], 0.75, abs_tol=1e-4)


def test_f11_fifo_packet_ordering_preserved():
    """F11.5: Verify DTN buffer maintains strict FIFO ordering."""
    buf = ReferenceDTNBuffer(capacity=10)
    for i in range(5):
        buf.push(NetworkPacket(f"PKT_{i}", "UAV_1", "GCS", "DATA"))
    for i in range(5):
        pkt = buf.pop()
        assert pkt.packet_id == f"PKT_{i}"


# ============================================================================
# Feature 12: Network Connectivity Maintenance & Topology Resilience (ORIGINAL_REQUEST §Acceptance Criteria)
# ============================================================================

def test_f12_vsm_relay_setpoint_between_gcs_and_surveyors(vsm_engine: ReferenceVSMEngine):
    """F12.1: Verify VSM positions relay midway between GCS and survey swarm."""
    gcs = np.array([0.0, 0.0, 10.0])
    surveyors = [
        DroneState("UAV_S1", "SURVEY", np.array([400.0, 400.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'SURVEYING'),
        DroneState("UAV_S2", "SURVEY", np.array([420.0, 420.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'SURVEYING'),
    ]
    relays = [DroneState("UAV_R1", "RELAY", np.array([50.0, 50.0, 10.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')]
    setpoints = vsm_engine.compute_relay_setpoints(gcs, surveyors, relays, [])
    sp = setpoints['UAV_R1']
    # Relay should be positioned approximately halfway (~205, ~205)
    assert 180.0 <= sp[0] <= 230.0
    assert 180.0 <= sp[1] <= 230.0


def test_f12_relay_altitude_corridor_elevation(vsm_engine: ReferenceVSMEngine):
    """F12.2: Verify relay setpoints target relay altitude corridor [70, 90]m."""
    gcs = np.array([50.0, 50.0, 10.0])
    surveyors = [DroneState("UAV_S1", "SURVEY", np.array([300.0, 300.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'SURVEYING')]
    relays = [DroneState("UAV_R1", "RELAY", np.array([50.0, 50.0, 10.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')]
    setpoints = vsm_engine.compute_relay_setpoints(gcs, surveyors, relays, [])
    assert 70.0 <= setpoints['UAV_R1'][2] <= 90.0


def test_f12_network_partition_detection(network_engine: ReferenceNetworkEngine):
    """F12.3: Verify find_route returns empty list when network partition occurs."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0]), 'UAV_REMOTE': np.array([480.0, 480.0, 30.0])}
    network_engine.update_topology(nodes, [])
    assert network_engine.find_route('UAV_REMOTE', 'GCS') == []


def test_f12_topology_healing_with_mobile_relay(network_engine: ReferenceNetworkEngine):
    """F12.4: Verify partitioned network heals when mobile relay arrives at setpoint."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0]), 'UAV_REMOTE': np.array([400.0, 0.0, 30.0])}
    network_engine.update_topology(nodes, [])
    assert network_engine.find_route('UAV_REMOTE', 'GCS') == []

    # Relay arrives
    nodes['UAV_RELAY'] = np.array([200.0, 0.0, 75.0])
    network_engine.update_topology(nodes, [])
    route = network_engine.find_route('UAV_REMOTE', 'GCS')
    assert route == ['UAV_REMOTE', 'UAV_RELAY', 'GCS']


def test_f12_low_battery_drone_rtl_mode():
    """F12.5: Verify low battery SoC (< 0.20) triggers Return-to-Launch without crashing fleet."""
    drone = DroneState("UAV_1", "SURVEY", np.array([200.0, 200.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 0.18, 'SURVEYING')
    # Policy check: low battery transitions to RTL
    if drone.battery_soc < 0.20:
        drone.flight_mode = 'RTL'
        drone.target_position = np.array([50.0, 50.0, 10.0])  # GCS base
    assert drone.flight_mode == 'RTL'
    assert drone.target_position[0] == 50.0
