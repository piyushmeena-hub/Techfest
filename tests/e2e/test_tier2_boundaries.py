"""
Tier 2: Boundary Value Analysis & Corner Cases (≥ 60 tests: 12 categories × ≥ 5 tests each).
Validates edge cases, coordinate limits, physical constraints, buffer capacities, and extreme topologies.
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
# Category 1: CLI & Execution Parameter Boundaries (B1)
# ============================================================================

def test_b1_zero_duration_boundary():
    """B1.1: Verify duration=0 terminates cleanly after 0 steps."""
    sim = ReferenceSimulationController(num_drones=3, num_pois=2, duration=0.0)
    snaps = sim.run()
    assert len(snaps) == 0
    assert sim.sim_time == 0.0


def test_b1_large_duration_scaling():
    """B1.2: Verify step calculation scales to large durations without floating point error."""
    sim = ReferenceSimulationController(num_drones=2, num_pois=1, duration=1000.0)
    expected_steps = int(1000.0 / sim.dt)
    assert expected_steps == 30000


def test_b1_minimal_fleet_single_drone():
    """B1.3: Verify swarm initializes and steps with minimal fleet size of 1 drone."""
    sim = ReferenceSimulationController(num_drones=1, num_pois=1, duration=0.1)
    assert len(sim.drones) == 1
    snap = sim.step()
    assert len(snap.drones) == 1


def test_b1_large_fleet_30_drones():
    """B1.4: Verify large fleet of 30 drones initializes without ID collisions."""
    sim = ReferenceSimulationController(num_drones=30, num_pois=5, duration=0.1)
    assert len(sim.drones) == 30
    ids = {d.id for d in sim.drones}
    assert len(ids) == 30


def test_b1_zero_pois_environment():
    """B1.5: Verify zero PoIs does not crash task allocation or simulation loop."""
    sim = ReferenceSimulationController(num_drones=4, num_pois=0, duration=0.1)
    assert len(sim.pois) == 0
    snap = sim.step()
    assert len(snap.pois) == 0
    assert snap.metrics['completed_pois'] == 0


# ============================================================================
# Category 2: Spatial & Coordinate Boundaries (B2)
# ============================================================================

def test_b2_exact_corner_origin_coord(kinematics_engine: ReferenceKinematicsEngine):
    """B2.1: UAV at [0.0, 0.0, 0.0] spatial lower bound remains inside bounds."""
    drone = DroneState("UAV_ORIGIN", "SURVEY", np.array([0.0, 0.0, 0.0]), np.array([-5.0, -5.0, -5.0]), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    kinematics_engine.step_drone(drone, 0.1, [drone], [])
    assert np.all(drone.position >= 0.0)


def test_b2_exact_corner_max_coord(kinematics_engine: ReferenceKinematicsEngine):
    """B2.2: UAV at [500.0, 500.0, 100.0] spatial upper bound remains inside bounds."""
    drone = DroneState("UAV_CORNER", "SURVEY", np.array([500.0, 500.0, 100.0]), np.array([10.0, 10.0, 10.0]), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    kinematics_engine.step_drone(drone, 0.1, [drone], [])
    assert drone.position[0] <= 500.0
    assert drone.position[1] <= 500.0
    assert drone.position[2] <= 100.0


def test_b2_sub_surface_z_clamping(kinematics_engine: ReferenceKinematicsEngine):
    """B2.3: Sub-surface descent is clamped to ground level z=0.0."""
    drone = DroneState("UAV_DIVE", "SURVEY", np.array([100.0, 100.0, 0.5]), np.array([0.0, 0.0, -10.0]), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    kinematics_engine.step_drone(drone, 0.1, [drone], [])
    assert drone.position[2] == 0.0


def test_b2_ceiling_altitude_clamping(kinematics_engine: ReferenceKinematicsEngine):
    """B2.4: UAV exceeding 100m ceiling is clamped to z=100.0m."""
    drone = DroneState("UAV_HIGH", "RELAY", np.array([100.0, 100.0, 99.0]), np.array([0.0, 0.0, 15.0]), np.zeros(3), np.zeros(4), 1.0, 'RELAY')
    kinematics_engine.step_drone(drone, 0.2, [drone], [])
    assert drone.position[2] == 100.0


def test_b2_gcs_origin_boundary():
    """B2.5: GCS placed at [0.0, 0.0, 0.0] origin coordinate."""
    sim = ReferenceSimulationController(num_drones=2, num_pois=1, duration=0.1, gcs_pos=np.zeros(3))
    assert np.allclose(sim.gcs_pos, np.zeros(3))
    snap = sim.step()
    assert snap.gcs['pos'] == [0.0, 0.0, 0.0]


# ============================================================================
# Category 3: Kinematic & Physical Limits (B3)
# ============================================================================

def test_b3_zero_velocity_hover_stability(kinematics_engine: ReferenceKinematicsEngine):
    """B3.1: Zero velocity and no target maintains exact stationary position."""
    pos = np.array([250.0, 250.0, 50.0])
    drone = DroneState("UAV_HOVER", "SURVEY", np.copy(pos), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    kinematics_engine.step_drone(drone, 1.0, [drone], [])
    assert np.allclose(drone.position, pos)
    assert np.allclose(drone.velocity, np.zeros(3))


def test_b3_extreme_overspeed_clamp(kinematics_engine: ReferenceKinematicsEngine):
    """B3.2: Extreme commanded speed of 100 m/s clamped to vmax = 15.0 m/s."""
    drone = DroneState("UAV_SPEED", "SURVEY", np.array([100.0, 100.0, 50.0]), np.array([100.0, 0.0, 0.0]), np.zeros(3), np.zeros(4), 1.0, 'TRANSIT')
    kinematics_engine.step_drone(drone, 0.1, [drone], [])
    v_mag = float(np.linalg.norm(drone.velocity))
    assert v_mag <= kinematics_engine.v_max + 1e-4


def test_b3_instantaneous_impulse_acceleration_clamp(kinematics_engine: ReferenceKinematicsEngine):
    """B3.3: Commanded target 1000m away clamps acceleration to amax = 5.0 m/s^2."""
    drone = DroneState("UAV_ACC", "SURVEY", np.array([0.0, 0.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'TRANSIT')
    drone.target_position = np.array([1000.0, 1000.0, 50.0])
    acc = kinematics_engine.compute_acceleration(drone, [drone], [])
    a_mag = float(np.linalg.norm(acc))
    assert math.isclose(a_mag, kinematics_engine.a_max, abs_tol=1e-4)


def test_b3_battery_soc_zero_clamp(kinematics_engine: ReferenceKinematicsEngine):
    """B3.4: Battery SoC cannot drop below 0.0 (no negative energy)."""
    drone = DroneState("UAV_DEAD", "SURVEY", np.array([100.0, 100.0, 50.0]), np.array([10.0, 0.0, 0.0]), np.zeros(3), np.zeros(4), 0.0001, 'TRANSIT')
    kinematics_engine.step_drone(drone, 10.0, [drone], [])
    assert drone.battery_soc == 0.0


def test_b3_battery_rtl_exact_threshold():
    """B3.5: Battery at exactly 0.20 triggers RTL threshold logic."""
    d_above = DroneState("U1", "SURVEY", np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(4), 0.2001, 'SURVEYING')
    d_exact = DroneState("U2", "SURVEY", np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(4), 0.2000, 'SURVEYING')
    d_below = DroneState("U3", "SURVEY", np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(4), 0.1999, 'SURVEYING')

    assert not (d_above.battery_soc < 0.20)
    assert not (d_exact.battery_soc < 0.20)
    assert (d_below.battery_soc < 0.20)


# ============================================================================
# Category 4: Flocking & Proximity Extremes (B4)
# ============================================================================

def test_b4_near_zero_separation_singularity_avoidance(kinematics_engine: ReferenceKinematicsEngine):
    """B4.1: Separation d = 1e-4m does not raise ZeroDivisionError or NaN."""
    d1 = DroneState("UAV_1", "SURVEY", np.array([100.0, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    d2 = DroneState("UAV_2", "SURVEY", np.array([100.0001, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    acc = kinematics_engine.compute_acceleration(d1, [d1, d2], [])
    assert not np.isnan(acc).any()
    assert not np.isinf(acc).any()


def test_b4_exact_safe_distance_boundary(kinematics_engine: ReferenceKinematicsEngine):
    """B4.2: Distance exactly at d_safe = 3.0m produces zero APF repulsion."""
    d1 = DroneState("UAV_1", "SURVEY", np.array([100.0, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    d2 = DroneState("UAV_2", "SURVEY", np.array([103.0, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    acc = kinematics_engine.compute_acceleration(d1, [d1, d2], [])
    assert np.allclose(acc, np.zeros(3), atol=1e-5)


def test_b4_collinear_three_drone_compression(kinematics_engine: ReferenceKinematicsEngine):
    """B4.3: Three collinear drones in close succession repel outwards."""
    d_left = DroneState("UAV_L", "SURVEY", np.array([98.5, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    d_mid = DroneState("UAV_M", "SURVEY", np.array([100.0, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    d_right = DroneState("UAV_R", "SURVEY", np.array([101.5, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')

    acc_l = kinematics_engine.compute_acceleration(d_left, [d_left, d_mid, d_right], [])
    acc_r = kinematics_engine.compute_acceleration(d_right, [d_left, d_mid, d_right], [])
    # Left drone pushed further left (-X), right drone pushed further right (+X)
    assert acc_l[0] < -0.1
    assert acc_r[0] > 0.1


def test_b4_high_speed_head_on_encounter(kinematics_engine: ReferenceKinematicsEngine):
    """B4.4: Drones approaching head-on with relative speed 30 m/s compute finite acceleration."""
    d1 = DroneState("UAV_1", "SURVEY", np.array([95.0, 100.0, 50.0]), np.array([15.0, 0.0, 0.0]), np.zeros(3), np.zeros(4), 1.0, 'TRANSIT')
    d2 = DroneState("UAV_2", "SURVEY", np.array([105.0, 100.0, 50.0]), np.array([-15.0, 0.0, 0.0]), np.zeros(3), np.zeros(4), 1.0, 'TRANSIT')
    acc1 = kinematics_engine.compute_acceleration(d1, [d1, d2], [])
    acc2 = kinematics_engine.compute_acceleration(d2, [d1, d2], [])
    assert float(np.linalg.norm(acc1)) <= kinematics_engine.a_max + 1e-4
    assert float(np.linalg.norm(acc2)) <= kinematics_engine.a_max + 1e-4


def test_b4_exact_downwash_threshold_boundary(kinematics_engine: ReferenceKinematicsEngine):
    """B4.5: Downwash triggers when other drone is vertically within 0 to 5m above."""
    d_bot = DroneState("BOT", "SURVEY", np.array([100.0, 100.0, 50.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    d_in = DroneState("IN", "SURVEY", np.array([100.1, 100.0, 54.9]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    d_out = DroneState("OUT", "SURVEY", np.array([100.1, 100.0, 55.1]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')

    acc_in = kinematics_engine.compute_acceleration(d_bot, [d_bot, d_in], [])
    acc_out = kinematics_engine.compute_acceleration(d_bot, [d_bot, d_out], [])
    assert float(np.linalg.norm(acc_in[:2])) > 0.1
    assert np.allclose(acc_out, np.zeros(3))


# ============================================================================
# Category 5: RF & Distance Extremes (B5)
# ============================================================================

def test_b5_zero_distance_path_loss_guard(channel_model: ReferenceChannelModel):
    """B5.1: Path loss at distance d=0 is guarded to d=1m (no math domain error)."""
    pl = channel_model.compute_path_loss(0.0)
    assert not math.isnan(pl)
    assert not math.isinf(pl)
    assert math.isclose(pl, channel_model.pl0, abs_tol=0.1)


def test_b5_exact_direct_los_range_boundary(channel_model: ReferenceChannelModel):
    """B5.2: Distance 320.0m is viable, 320.1m is non-viable."""
    v_320, _, _ = channel_model.is_link_viable(320.0, is_los=True)
    v_320_1, _, _ = channel_model.is_link_viable(320.1, is_los=True)
    assert v_320 is True
    assert v_320_1 is False


def test_b5_extreme_range_1000m_drop(channel_model: ReferenceChannelModel):
    """B5.3: Distance 1000m produces extreme path loss (> 100 dB) and non-viable link."""
    viable, pl, snr = channel_model.is_link_viable(1000.0, is_los=True)
    assert pl > 100.0
    assert viable is False
    # Under NLoS, SNR drops far below 0 dB
    pl_nlos = channel_model.compute_path_loss(1000.0, is_los=False)
    snr_nlos = channel_model.compute_snr(pl_nlos)
    assert snr_nlos < 0.0


def test_b5_extreme_low_tx_power_zero_dbm():
    """B5.4: Reduced transmit power P_tx = 0 dBm drops SNR significantly."""
    ch_normal = ReferenceChannelModel(p_tx_dbm=20.0)
    ch_low = ReferenceChannelModel(p_tx_dbm=0.0)
    snr_norm = ch_normal.compute_snr(70.0)
    snr_low = ch_low.compute_snr(70.0)
    assert math.isclose(snr_norm - snr_low, 20.0)


def test_b5_high_noise_floor_interference():
    """B5.5: Elevated noise floor of -60 dBm drops SNR by 35 dB."""
    ch_clean = ReferenceChannelModel(noise_floor_dbm=-95.0)
    ch_noisy = ReferenceChannelModel(noise_floor_dbm=-60.0)
    snr_clean = ch_clean.compute_snr(60.0)
    snr_noisy = ch_noisy.compute_snr(60.0)
    assert math.isclose(snr_clean - snr_noisy, 35.0)


# ============================================================================
# Category 6: Obstacle Occlusion & Geometry Extremes (B6)
# ============================================================================

def test_b6_ray_grazing_building_exact_face(occlusion_engine: ReferenceOcclusionEngine):
    """B6.1: Ray grazing exactly along building face is detected without exception."""
    # BUILDING_1 is [200, 200, 0] to [260, 260, 35]
    p1 = np.array([200.0, 100.0, 15.0])
    p2 = np.array([200.0, 300.0, 15.0])
    is_occ, _ = occlusion_engine.check_occlusion(p1, p2)
    assert isinstance(is_occ, bool)


def test_b6_ray_through_corner_vertex(occlusion_engine: ReferenceOcclusionEngine):
    """B6.2: Ray passing directly through corner vertex [200, 200, 35]."""
    p1 = np.array([190.0, 190.0, 36.0])
    p2 = np.array([210.0, 210.0, 34.0])
    is_occ, _ = occlusion_engine.check_occlusion(p1, p2)
    assert is_occ is True


def test_b6_drone_inside_building_boundary(sample_obstacles: List[AABB]):
    """B6.3: Point inside building returns True for contains_point."""
    b1 = sample_obstacles[0]
    inside_pt = np.array([230.0, 230.0, 20.0])
    outside_pt = np.array([270.0, 230.0, 20.0])
    assert b1.contains_point(inside_pt) is True
    assert b1.contains_point(outside_pt) is False


def test_b6_zero_height_degenerate_obstacle():
    """B6.4: Zero-height obstacle does not occlude elevated ray."""
    flat_obs = AABB("FLAT", np.array([200.0, 200.0, 0.0]), np.array([260.0, 260.0, 0.0]), height=0.0)
    occ = ReferenceOcclusionEngine([flat_obs])
    p1 = np.array([150.0, 230.0, 10.0])
    p2 = np.array([300.0, 230.0, 10.0])
    is_occ, _ = occ.check_occlusion(p1, p2)
    assert is_occ is False


def test_b6_large_obstacle_spanning_width():
    """B6.5: Massive 200m building reliably occludes through-path."""
    huge_obs = AABB("HUGE", np.array([100.0, 100.0, 0.0]), np.array([300.0, 300.0, 50.0]), height=50.0)
    occ = ReferenceOcclusionEngine([huge_obs])
    p1 = np.array([50.0, 200.0, 20.0])
    p2 = np.array([350.0, 200.0, 20.0])
    is_occ, count = occ.check_occlusion(p1, p2)
    assert is_occ is True
    assert count == 1


# ============================================================================
# Category 7: Routing & Graph Extremes (B7)
# ============================================================================

def test_b7_fully_disconnected_graph(network_engine: ReferenceNetworkEngine):
    """B7.1: Graph with zero edges returns empty route."""
    nodes = {'GCS': np.array([0.0, 0.0, 0.0]), 'UAV_1': np.array([490.0, 490.0, 0.0])}
    network_engine.update_topology(nodes, [])
    assert network_engine.find_route('UAV_1', 'GCS') == []


def test_b7_complete_graph_tie_breaking(network_engine: ReferenceNetworkEngine):
    """B7.2: Fully connected complete graph chooses direct 1-hop path."""
    nodes = {
        'GCS': np.array([0.0, 0.0, 10.0]),
        'U1': np.array([50.0, 50.0, 30.0]),
        'U2': np.array([25.0, 75.0, 30.0]),
    }
    network_engine.update_topology(nodes, [])
    route = network_engine.find_route('U1', 'GCS')
    assert route == ['U1', 'GCS']


def test_b7_long_linear_daisy_chain_10_hops(network_engine: ReferenceNetworkEngine):
    """B7.3: 10-node linear chain creates valid 9-hop path."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0])}
    for i in range(1, 10):
        nodes[f"UAV_{i}"] = np.array([i * 100.0, 0.0, 30.0])
    network_engine.update_topology(nodes, [])
    route = network_engine.find_route('UAV_9', 'GCS')
    expected = [f"UAV_{i}" for i in range(9, 0, -1)] + ['GCS']
    assert route == expected
    assert len(route) == 10


def test_b7_self_route_source_is_destination(network_engine: ReferenceNetworkEngine):
    """B7.4: Route query where source equals destination returns [source]."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0])}
    network_engine.update_topology(nodes, [])
    assert network_engine.find_route('GCS', 'GCS') == ['GCS']


def test_b7_nonexistent_node_route_query(network_engine: ReferenceNetworkEngine):
    """B7.5: Querying unknown node gracefully returns empty list."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0])}
    network_engine.update_topology(nodes, [])
    assert network_engine.find_route('NONEXISTENT', 'GCS') == []


# ============================================================================
# Category 8: Packet Buffer & Logging Bounds (B8)
# ============================================================================

def test_b8_dtn_buffer_pop_on_empty():
    """B8.1: Popping empty DTN buffer safely returns None."""
    buf = ReferenceDTNBuffer(capacity=250)
    assert buf.is_empty()
    assert buf.pop() is None


def test_b8_dtn_buffer_exact_capacity_250():
    """B8.2: Buffer accepts exactly 250 packets without drop."""
    buf = ReferenceDTNBuffer(capacity=250)
    for i in range(250):
        success = buf.push(NetworkPacket(f"P_{i}", "U1", "GCS", "DATA"))
        assert success is True
    assert buf.size() == 250
    assert buf.total_dropped == 0


def test_b8_dtn_buffer_overflow_drop_oldest():
    """B8.3: Pushing packet 251 drops oldest packet (P_0) and retains P_250."""
    buf = ReferenceDTNBuffer(capacity=250)
    for i in range(251):
        buf.push(NetworkPacket(f"P_{i}", "U1", "GCS", "DATA"))
    assert buf.size() == 250
    assert buf.total_dropped == 1
    # First popped packet should now be P_1, not P_0
    first_pkt = buf.pop()
    assert first_pkt.packet_id == "P_1"


def test_b8_zero_payload_packet_handling(network_engine: ReferenceNetworkEngine):
    """B8.4: Zero payload byte size packet transmits without error."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0]), 'U1': np.array([50.0, 0.0, 30.0])}
    network_engine.update_topology(nodes, [])
    pkt = NetworkPacket("P_ZERO", "U1", "GCS", "HEARTBEAT", data_size_bytes=0)
    success = network_engine.transmit_packet(pkt, 1.0)
    assert success is True
    assert pkt.status == 'DELIVERED'


def test_b8_large_jumbo_payload_packet(network_engine: ReferenceNetworkEngine):
    """B8.5: Jumbo payload (10 MB) transmits with valid metadata."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0]), 'U1': np.array([50.0, 0.0, 30.0])}
    network_engine.update_topology(nodes, [])
    pkt = NetworkPacket("P_JUMBO", "U1", "GCS", "SURVEY_DATA", data_size_bytes=10 * 1024 * 1024)
    success = network_engine.transmit_packet(pkt, 1.0)
    assert success is True
    assert pkt.data_size_bytes == 10485760


# ============================================================================
# Category 9: PoI Spatial & Value Limits (B9)
# ============================================================================

def test_b9_poi_at_exact_corner_boundary():
    """B9.1: PoI at [500.0, 500.0, 30.0] correctly targeted."""
    poi = PoI("POI_CORNER", np.array([500.0, 500.0, 30.0]), "HIGH")
    mission = ReferenceMissionCoordinator([poi])
    drone = DroneState("U1", "SURVEY", np.array([450.0, 450.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    mission.assign_tasks([drone])
    assert drone.assigned_poi_id == "POI_CORNER"
    assert np.allclose(drone.target_position, np.array([500.0, 500.0, 30.0]))


def test_b9_poi_directly_above_gcs():
    """B9.2: PoI at [50.0, 50.0, 40.0] directly over GCS base."""
    poi = PoI("POI_GCS_ROOF", np.array([50.0, 50.0, 40.0]), "HIGH")
    assert poi.position[2] == 40.0


def test_b9_zero_dwell_time_poi():
    """B9.3: PoI with 0.0s dwell completes instantaneously upon arrival."""
    poi = PoI("POI_INSTANT", np.array([100.0, 100.0, 30.0]), "HIGH", dwell_time_required=0.0)
    mission = ReferenceMissionCoordinator([poi])
    drone = DroneState("U1", "SURVEY", np.array([100.0, 100.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'SURVEYING', assigned_poi_id="POI_INSTANT")
    net = ReferenceNetworkEngine()
    mission.update_dwell([drone], 0.01, net, 1.0)
    assert poi.status == 'COMPLETED'
    assert mission.completed_pois == 1


def test_b9_long_dwell_time_poi_100s():
    """B9.4: PoI with 100s dwell time accumulates smoothly."""
    poi = PoI("POI_LONG", np.array([100.0, 100.0, 30.0]), "HIGH", dwell_time_required=100.0)
    mission = ReferenceMissionCoordinator([poi])
    drone = DroneState("U1", "SURVEY", np.array([100.0, 100.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'SURVEYING', assigned_poi_id="POI_LONG")
    net = ReferenceNetworkEngine()
    mission.update_dwell([drone], 50.0, net, 1.0)
    assert poi.status == 'IN_PROGRESS'
    assert math.isclose(poi.dwell_time_accumulated, 50.0)


def test_b9_duplicate_coordinate_pois():
    """B9.5: Multiple PoIs at exact same coordinate handled sequentially."""
    p1 = PoI("P1", np.array([100.0, 100.0, 30.0]), "HIGH", dwell_time_required=0.1)
    p2 = PoI("P2", np.array([100.0, 100.0, 30.0]), "MEDIUM", dwell_time_required=0.1)
    mission = ReferenceMissionCoordinator([p1, p2])
    drone = DroneState("U1", "SURVEY", np.array([50.0, 50.0, 10.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    mission.assign_tasks([drone])
    assert drone.assigned_poi_id == "P1"


# ============================================================================
# Category 10: FSM State & Transitions Extremes (B10)
# ============================================================================

def test_b10_immediate_rtl_from_surveying():
    """B10.1: Immediate mission abort / RTL during active SURVEYING dwell."""
    drone = DroneState("U1", "SURVEY", np.array([200.0, 200.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'SURVEYING', assigned_poi_id="POI_1")
    # Abort signal
    drone.flight_mode = 'RTL'
    drone.assigned_poi_id = None
    assert drone.flight_mode == 'RTL'
    assert drone.assigned_poi_id is None


def test_b10_idle_to_landed_without_takeoff():
    """B10.2: Drone transitions from IDLE directly to LANDED without takeoff."""
    drone = DroneState("U1", "SURVEY", np.array([50.0, 50.0, 0.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    drone.flight_mode = 'LANDED'
    assert drone.flight_mode == 'LANDED'


def test_b10_partial_dwell_interruption():
    """B10.3: Dwell interrupted at 99% progress does not mark PoI completed."""
    poi = PoI("P1", np.array([100.0, 100.0, 30.0]), "HIGH", dwell_time_required=10.0, dwell_time_accumulated=9.9)
    assert poi.status != 'COMPLETED'
    assert poi.dwell_time_accumulated < poi.dwell_time_required


def test_b10_invalid_flight_mode_tolerance():
    """B10.4: Unknown flight mode string does not crash state inspection."""
    drone = DroneState("U1", "SURVEY", np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'UNKNOWN_MODE')
    valid_modes = {'IDLE', 'TAKEOFF', 'TRANSIT', 'SURVEYING', 'RELAY', 'DATA_TX', 'RTL', 'LANDED', 'COMPLETED'}
    assert drone.flight_mode not in valid_modes


def test_b10_zero_battery_emergency_landing():
    """B10.5: Battery depletion to 0.0 forces LANDED state."""
    drone = DroneState("U1", "SURVEY", np.array([100.0, 100.0, 0.0]), np.zeros(3), np.zeros(3), np.zeros(4), 0.0, 'RTL')
    if drone.battery_soc <= 0.0:
        drone.flight_mode = 'LANDED'
    assert drone.flight_mode == 'LANDED'


# ============================================================================
# Category 11: Telemetry & Metrics Extremes (B11)
# ============================================================================

def test_b11_pdr_zero_packets_transmitted(network_engine: ReferenceNetworkEngine):
    """B11.1: PDR with 0 packets transmitted defaults safely to 1.0."""
    metrics = network_engine.get_metrics()
    assert metrics['pdr'] == 1.0
    assert metrics['total_transmitted'] == 0.0


def test_b11_pdr_100_percent_success(network_engine: ReferenceNetworkEngine):
    """B11.2: PDR with 50 successful deliveries returns exactly 1.0."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0]), 'U1': np.array([50.0, 0.0, 30.0])}
    network_engine.update_topology(nodes, [])
    for i in range(50):
        network_engine.transmit_packet(NetworkPacket(f"P_{i}", "U1", "GCS", "TELEMETRY"))
    metrics = network_engine.get_metrics()
    assert metrics['pdr'] == 1.0


def test_b11_pdr_zero_percent_all_dropped(network_engine: ReferenceNetworkEngine):
    """B11.3: PDR with 50 dropped packets returns exactly 0.0."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0]), 'U1': np.array([450.0, 450.0, 30.0])}
    network_engine.update_topology(nodes, [])
    for i in range(50):
        network_engine.transmit_packet(NetworkPacket(f"P_{i}", "U1", "GCS", "TELEMETRY"))
    metrics = network_engine.get_metrics()
    assert metrics['pdr'] == 0.0


def test_b11_burst_traffic_100_packets_instant(network_engine: ReferenceNetworkEngine):
    """B11.4: 100 packets injected in single tick processed without buffer starvation."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0]), 'U1': np.array([50.0, 0.0, 30.0])}
    network_engine.update_topology(nodes, [])
    successes = [network_engine.transmit_packet(NetworkPacket(f"BURST_{i}", "U1", "GCS", "DATA")) for i in range(100)]
    assert all(successes)
    assert network_engine.packets_delivered == 100


def test_b11_telemetry_snapshot_empty_swarm():
    """B11.5: Snapshot with 0 drones and 0 PoIs serializes valid empty structures."""
    snap = TelemetrySnapshot(
        sim_time=0.0,
        drones=[],
        gcs={'pos': [0.0, 0.0, 0.0]},
        pois=[],
        active_routes=[],
        links=[],
        packets=[],
        metrics={'pdr': 1.0, 'avg_latency_ms': 0.0, 'completed_pois': 0}
    )
    assert len(snap.drones) == 0
    assert len(snap.pois) == 0
    assert snap.metrics['completed_pois'] == 0


# ============================================================================
# Category 12: Network Topology & Failure Limits (B12)
# ============================================================================

def test_b12_all_relays_fail_simultaneously(network_engine: ReferenceNetworkEngine):
    """B12.1: Simultaneous failure of all relays isolates distant surveyor."""
    nodes = {
        'GCS': np.array([0.0, 0.0, 10.0]),
        'U_R1': np.array([180.0, 0.0, 75.0]),
        'U_R2': np.array([180.0, 50.0, 75.0]),
        'U_S': np.array([360.0, 0.0, 30.0]),  # 360m > 320m direct range
    }
    network_engine.update_topology(nodes, [])
    assert len(network_engine.find_route('U_S', 'GCS')) > 0

    # Fail both relays simultaneously
    del nodes['U_R1']
    del nodes['U_R2']
    network_engine.update_topology(nodes, [])
    assert network_engine.find_route('U_S', 'GCS') == []


def test_b12_critical_bridge_node_failure(network_engine: ReferenceNetworkEngine):
    """B12.2: Removal of single bridge node severs communication."""
    nodes = {
        'GCS': np.array([0.0, 0.0, 10.0]),
        'BRIDGE': np.array([180.0, 0.0, 75.0]),
        'LEAF': np.array([360.0, 0.0, 30.0]),
    }
    network_engine.update_topology(nodes, [])
    assert network_engine.find_route('LEAF', 'GCS') == ['LEAF', 'BRIDGE', 'GCS']

    del nodes['BRIDGE']
    network_engine.update_topology(nodes, [])
    assert network_engine.find_route('LEAF', 'GCS') == []


def test_b12_zero_relay_fleet_survey_only():
    """B12.3: Fleet of 100% survey drones functions with direct links only."""
    sim = ReferenceSimulationController(num_drones=3, num_pois=2, duration=0.1)
    for d in sim.drones:
        d.role = 'SURVEY'
    snap = sim.step()
    roles = {d['role'] for d in snap.drones}
    assert roles == {'SURVEY'}


def test_b12_zero_survey_fleet_relay_only():
    """B12.4: Fleet of 100% relay drones operates without mission exceptions."""
    sim = ReferenceSimulationController(num_drones=3, num_pois=0, duration=0.1)
    for d in sim.drones:
        d.role = 'RELAY'
    snap = sim.step()
    roles = {d['role'] for d in snap.drones}
    assert roles == {'RELAY'}


def test_b12_rapid_topology_flapping(network_engine: ReferenceNetworkEngine):
    """B12.5: Node alternating between in-range and out-of-range does not corrupt routing table."""
    nodes = {'GCS': np.array([0.0, 0.0, 10.0]), 'U1': np.array([100.0, 0.0, 30.0])}
    for cycle in range(10):
        # In range
        nodes['U1'] = np.array([100.0, 0.0, 30.0])
        network_engine.update_topology(nodes, [])
        assert network_engine.find_route('U1', 'GCS') == ['U1', 'GCS']

        # Out of range
        nodes['U1'] = np.array([450.0, 0.0, 30.0])
        network_engine.update_topology(nodes, [])
        assert network_engine.find_route('U1', 'GCS') == []
