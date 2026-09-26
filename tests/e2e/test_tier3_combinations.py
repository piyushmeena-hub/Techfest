"""
Tier 3: Pairwise Combinatorial Interactions (≥ 12 tests).
Validates cross-feature interactions between kinematics, obstacles, routing, buffering, dwell, and mission control.
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


def test_p1_kinematics_and_obstacle_avoidance(kinematics_engine: ReferenceKinematicsEngine):
    """P1: Kinematics (F3) + Obstacle Avoidance (F4/F6): Drone moving toward building stops/turns without penetration."""
    obs = [AABB("BUILDING", np.array([100.0, 100.0, 0.0]), np.array([140.0, 140.0, 35.0]), 35.0)]
    # Drone starts at [80, 120, 20] heading directly into building (+X)
    drone = DroneState("UAV_1", "TRANSIT", np.array([80.0, 120.0, 20.0]), np.array([10.0, 0.0, 0.0]), np.zeros(3), np.zeros(4), 1.0, 'TRANSIT')
    drone.target_position = np.array([160.0, 120.0, 20.0])

    dt = 0.05
    for _ in range(80):
        kinematics_engine.step_drone(drone, dt, [drone], obs)
        # Verify drone position never penetrates inside building volume
        assert not obs[0].contains_point(drone.position), f"Drone penetrated obstacle: {drone.position}"


def test_p2_dynamic_routing_and_store_and_forward_buffering(network_engine: ReferenceNetworkEngine):
    """P2: Dynamic Routing (F7) + DTN Buffering (F11): Obstacle occlusion buffers packets until relay clears route."""
    obs = [AABB("BUILDING", np.array([150.0, 150.0, 0.0]), np.array([210.0, 210.0, 40.0]))]
    nodes = {
        'GCS': np.array([50.0, 180.0, 10.0]),
        'UAV_Survey': np.array([300.0, 180.0, 20.0]),  # Occluded by building
    }
    network_engine.update_topology(nodes, obs)
    assert network_engine.find_route('UAV_Survey', 'GCS') == []

    # Send 3 survey packets during blackout -> should be buffered
    for i in range(3):
        pkt = NetworkPacket(f"SURVEY_{i}", "UAV_Survey", "GCS", "SURVEY_DATA")
        delivered = network_engine.transmit_packet(pkt, 1.0)
        assert delivered is False

    assert network_engine.dtn_buffers['UAV_Survey'].size() == 3

    # Deploy relay above building (z=75m) restoring route
    nodes['UAV_Relay'] = np.array([180.0, 180.0, 75.0])
    network_engine.update_topology(nodes, obs)
    assert len(network_engine.find_route('UAV_Survey', 'GCS')) == 3

    # Drain buffer
    buf = network_engine.dtn_buffers['UAV_Survey']
    drained_count = 0
    while not buf.is_empty():
        p = buf.pop()
        assert p is not None
        assert network_engine.transmit_packet(p, 2.0) is True
        drained_count += 1

    assert drained_count == 3


def test_p3_survey_dwell_and_battery_depletion_rtl():
    """P3: PoI Survey Dwell (F10) + Battery Depletion (F3/F12): Dwell depletes battery to RTL threshold."""
    poi = PoI("POI_SURV", np.array([200.0, 200.0, 30.0]), "HIGH", dwell_time_required=20.0)
    drone = DroneState("UAV_1", "SURVEY", np.array([200.0, 200.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 0.201, 'SURVEYING', assigned_poi_id=poi.id)
    ke = ReferenceKinematicsEngine()

    # Step drone until battery drops below 0.20
    dt = 1.0
    for _ in range(10):
        ke.step_drone(drone, dt, [drone], [])
        if drone.battery_soc < 0.20:
            drone.flight_mode = 'RTL'
            drone.target_position = np.array([50.0, 50.0, 10.0])  # Return to GCS
            drone.assigned_poi_id = None
            break

    assert drone.flight_mode == 'RTL'
    assert drone.battery_soc < 0.20
    assert drone.assigned_poi_id is None


def test_p4_flocking_separation_and_vsm_relay_positioning(vsm_engine: ReferenceVSMEngine, kinematics_engine: ReferenceKinematicsEngine):
    """P4: Flocking Separation (F4) + VSM Relay Positioning (F12): Relays converging to setpoints maintain clearance."""
    gcs = np.array([0.0, 0.0, 10.0])
    surveyors = [DroneState("UAV_S", "SURVEY", np.array([400.0, 400.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'SURVEYING')]
    relays = [
        DroneState("UAV_R1", "RELAY", np.array([195.0, 200.0, 75.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'RELAY'),
        DroneState("UAV_R2", "RELAY", np.array([205.0, 200.0, 75.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'RELAY'),
    ]
    setpoints = vsm_engine.compute_relay_setpoints(gcs, surveyors, relays, [])
    relays[0].target_position = setpoints['UAV_R1']
    relays[1].target_position = setpoints['UAV_R2']

    min_dist = float('inf')
    for _ in range(40):
        kinematics_engine.step_drone(relays[0], 0.05, relays, [])
        kinematics_engine.step_drone(relays[1], 0.05, relays, [])
        dist = float(np.linalg.norm(relays[0].position - relays[1].position))
        min_dist = min(min_dist, dist)

    assert min_dist >= 1.5


def test_p5_altitude_corridors_and_rf_path_loss(channel_model: ReferenceChannelModel, occlusion_engine: ReferenceOcclusionEngine):
    """P5: Altitude Corridors (F2) + RF Path Loss (F5): Elevating to relay corridor (80m) clears obstacle and boosts SNR."""
    obs = [AABB("BUILDING", np.array([150.0, 150.0, 0.0]), np.array([210.0, 210.0, 35.0]), 35.0)]
    occ = ReferenceOcclusionEngine(obs)

    p_gcs = np.array([50.0, 180.0, 10.0])
    p_transit = np.array([180.0, 180.0, 25.0])  # Behind building, occluded
    p_relay = np.array([180.0, 180.0, 80.0])    # In relay corridor [70, 90]m, clear

    occ_transit, cnt_transit = occ.check_occlusion(p_gcs, p_transit)
    occ_relay, cnt_relay = occ.check_occlusion(p_gcs, p_relay)

    assert occ_transit is True
    assert occ_relay is False

    pl_transit = channel_model.compute_path_loss(np.linalg.norm(p_transit - p_gcs), is_los=not occ_transit, num_occlusions=cnt_transit)
    pl_relay = channel_model.compute_path_loss(np.linalg.norm(p_relay - p_gcs), is_los=not occ_relay, num_occlusions=cnt_relay)

    # Relay altitude has significantly lower path loss despite slightly greater 3D distance
    assert pl_transit > pl_relay + 15.0


def test_p6_concurrent_multi_poi_inspection_and_hop_logging(network_engine: ReferenceNetworkEngine):
    """P6: Concurrent Multi-PoI (F9/F10) + Hop Logging (F8): Concurrent survey packets log distinct hop traces."""
    nodes = {
        'GCS': np.array([0.0, 0.0, 10.0]),
        'RELAY_NORTH': np.array([0.0, 180.0, 75.0]),
        'RELAY_EAST': np.array([180.0, 0.0, 75.0]),
        'SURVEY_NORTH': np.array([0.0, 360.0, 30.0]),
        'SURVEY_EAST': np.array([360.0, 0.0, 30.0]),
    }
    network_engine.update_topology(nodes, [])

    pkt_n = NetworkPacket("PKT_N", "SURVEY_NORTH", "GCS", "SURVEY_DATA")
    pkt_e = NetworkPacket("PKT_E", "SURVEY_EAST", "GCS", "SURVEY_DATA")

    assert network_engine.transmit_packet(pkt_n, 1.0) is True
    assert network_engine.transmit_packet(pkt_e, 1.0) is True

    assert pkt_n.hop_trace == ['SURVEY_NORTH', 'RELAY_NORTH', 'GCS']
    assert pkt_e.hop_trace == ['SURVEY_EAST', 'RELAY_EAST', 'GCS']


def test_p7_node_mobility_and_link_state_route_switching(network_engine: ReferenceNetworkEngine):
    """P7: Node Mobility (F3) + Dynamic Routing (F7): Fast-moving drone switches hops seamlessly as distance changes."""
    nodes = {
        'GCS': np.array([0.0, 0.0, 10.0]),
        'RELAY_1': np.array([200.0, 0.0, 75.0]),
        'RELAY_2': np.array([400.0, 0.0, 75.0]),
        'UAV_MOBILE': np.array([100.0, 0.0, 30.0]),
    }

    # Position 1: 100m from GCS (< 320m) -> direct 1-hop
    network_engine.update_topology(nodes, [])
    assert network_engine.find_route('UAV_MOBILE', 'GCS') == ['UAV_MOBILE', 'GCS']

    # Position 2: Moves to 380m (> 320m from GCS) -> 2-hop via RELAY_1
    nodes['UAV_MOBILE'] = np.array([380.0, 0.0, 30.0])
    network_engine.update_topology(nodes, [])
    route2 = network_engine.find_route('UAV_MOBILE', 'GCS')
    assert len(route2) == 3
    assert route2 == ['UAV_MOBILE', 'RELAY_1', 'GCS']

    # Position 3: Moves to 580m (> 320m from RELAY_1) -> 3-hop via RELAY_2 -> RELAY_1 -> GCS
    nodes['UAV_MOBILE'] = np.array([580.0, 0.0, 30.0])
    network_engine.update_topology(nodes, [])
    route3 = network_engine.find_route('UAV_MOBILE', 'GCS')
    assert len(route3) == 4
    assert route3 == ['UAV_MOBILE', 'RELAY_2', 'RELAY_1', 'GCS']


def test_p8_obstacle_shadowing_and_dynamic_relay_dispatch(vsm_engine: ReferenceVSMEngine, network_engine: ReferenceNetworkEngine):
    """P8: Obstacle Shadowing (F6) + Dynamic Relay Positioning (F12): VSM places relay to bridge shadow."""
    obs = [AABB("BUILDING", np.array([160.0, 160.0, 0.0]), np.array([220.0, 220.0, 35.0]), 35.0)]
    gcs = np.array([50.0, 50.0, 10.0])
    surveyor = DroneState("UAV_S", "SURVEY", np.array([350.0, 350.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'SURVEYING')
    relay = DroneState("UAV_R", "RELAY", np.array([50.0, 50.0, 10.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')

    # Without relay, direct link is blocked
    nodes = {'GCS': gcs, 'UAV_S': surveyor.position}
    network_engine.update_topology(nodes, obs)
    assert network_engine.find_route('UAV_S', 'GCS') == []

    # VSM sets relay setpoint elevated in corridor
    sp = vsm_engine.compute_relay_setpoints(gcs, [surveyor], [relay], obs)
    relay.position = sp['UAV_R']
    nodes['UAV_R'] = relay.position
    network_engine.update_topology(nodes, obs)

    # Route is established through relay
    route = network_engine.find_route('UAV_S', 'GCS')
    assert route == ['UAV_S', 'UAV_R', 'GCS']


def test_p9_packet_delivery_rate_and_hop_count_tradeoff(network_engine: ReferenceNetworkEngine):
    """P9: Telemetry Delivery (F11) + Hop Count (F7/F8): Latency grows monotonically with hop count."""
    # 1-hop test: distance 100m (< 320m)
    nodes1 = {'GCS': np.array([0.0, 0.0, 10.0]), 'N1': np.array([100.0, 0.0, 30.0])}
    network_engine.update_topology(nodes1, [])
    p1 = NetworkPacket("P1", "N1", "GCS", "DATA")
    assert network_engine.transmit_packet(p1, 1.0) is True
    assert len(p1.hop_trace) == 2  # 1 hop
    lat1 = network_engine.latencies_ms[-1]

    # 2-hop test: distance 380m (> 320m direct, 190m to R1)
    nodes2 = {
        'GCS': np.array([0.0, 0.0, 10.0]),
        'R1': np.array([190.0, 0.0, 75.0]),
        'N2': np.array([380.0, 0.0, 30.0])
    }
    network_engine.update_topology(nodes2, [])
    p2 = NetworkPacket("P2", "N2", "GCS", "DATA")
    assert network_engine.transmit_packet(p2, 1.0) is True
    assert len(p2.hop_trace) == 3  # 2 hops
    lat2 = network_engine.latencies_ms[-1]

    # 3-hop test: distance 560m (> 380m, requires 2 intermediate relays)
    nodes3 = {
        'GCS': np.array([0.0, 0.0, 10.0]),
        'R1': np.array([190.0, 0.0, 75.0]),
        'R2': np.array([380.0, 0.0, 75.0]),
        'N3': np.array([560.0, 0.0, 30.0])
    }
    network_engine.update_topology(nodes3, [])
    p3 = NetworkPacket("P3", "N3", "GCS", "DATA")
    assert network_engine.transmit_packet(p3, 1.0) is True
    assert len(p3.hop_trace) == 4  # 3 hops
    lat3 = network_engine.latencies_ms[-1]

    assert lat1 < lat2 < lat3


def test_p10_single_setup_cli_and_swarm_scaling():
    """P10: CLI Setup (F1) + Swarm Size Scaling (F2/F12): 8 drones provides higher link redundancy than 4."""
    sim4 = ReferenceSimulationController(num_drones=4, num_pois=2, duration=0.1)
    sim8 = ReferenceSimulationController(num_drones=8, num_pois=2, duration=0.1)
    snap4 = sim4.step()
    snap8 = sim8.step()

    # Denser network should have more active links
    assert len(snap8.links) > len(snap4.links)


def test_p11_composite_cost_routing_and_asymmetric_shadowing(network_engine: ReferenceNetworkEngine):
    """P11: Obstacle Attenuation (F6) + Dynamic Routing (F7): Clear 2-hop path chosen over occluded 1-hop path."""
    obs = [AABB("THICK_BUILDING", np.array([80.0, 90.0, 0.0]), np.array([140.0, 110.0, 45.0]))]
    nodes = {
        'GCS': np.array([0.0, 100.0, 10.0]),
        'SURVEY': np.array([200.0, 100.0, 20.0]),  # Direct line goes through thick building
        'RELAY_BYPASS': np.array([100.0, 25.0, 30.0]),  # Far to the south, clear LoS
    }
    network_engine.update_topology(nodes, obs)
    route = network_engine.find_route('SURVEY', 'GCS')
    assert route == ['SURVEY', 'RELAY_BYPASS', 'GCS']


def test_p12_poi_completion_and_autonomous_reassignment():
    """P12: PoI Dwell Completion (F10) + Autonomous Reassignment (F9): High PoI completion triggers next task."""
    p_high = PoI("P_HIGH", np.array([100.0, 100.0, 30.0]), "HIGH", dwell_time_required=0.2)
    p_med = PoI("P_MED", np.array([200.0, 200.0, 30.0]), "MEDIUM", dwell_time_required=0.5)
    mission = ReferenceMissionCoordinator([p_high, p_med])
    drone = DroneState("U1", "SURVEY", np.array([100.0, 100.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE')
    net = ReferenceNetworkEngine()

    mission.assign_tasks([drone])
    assert drone.assigned_poi_id == "P_HIGH"

    # Complete first survey
    mission.update_dwell([drone], 0.3, net, 1.0)
    assert p_high.status == 'COMPLETED'
    assert drone.assigned_poi_id is None

    # Next assignment cycle picks up P_MED
    mission.assign_tasks([drone])
    assert drone.assigned_poi_id == "P_MED"
    assert np.allclose(drone.target_position, p_med.position)
