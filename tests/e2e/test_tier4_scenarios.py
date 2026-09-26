"""
Tier 4: Real-World Disaster Workload Scenarios (≥ 6 scenarios).
Validates end-to-end mission workflows under disaster conditions as specified in TEST_INFRA.md.
"""

from __future__ import annotations
import math
import os
import subprocess
import sys
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
# Scenario 1: Urban Earthquake Collapse Survey
# ============================================================================

def test_scenario1_urban_earthquake_collapse_survey():
    """
    Scenario 1: Urban Earthquake Collapse Survey.
    Survey UAV behind 35m building maintains connectivity to GCS via 2-hop relay and transmits survey data.
    """
    obs = [AABB("COLLAPSED_TOWER", np.array([160.0, 180.0, 0.0]), np.array([220.0, 220.0, 35.0]), 35.0)]
    gcs_pos = np.array([50.0, 200.0, 10.0])
    poi_survivor = PoI("POI_SURVIVOR", np.array([320.0, 200.0, 30.0]), "HIGH", dwell_time_required=2.0)

    uav_survey = DroneState("UAV_SURVEY", "SURVEY", np.array([320.0, 200.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'SURVEYING', assigned_poi_id=poi_survivor.id)
    uav_relay = DroneState("UAV_RELAY", "RELAY", np.array([190.0, 200.0, 80.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'RELAY')

    net = ReferenceNetworkEngine(ReferenceChannelModel(), ReferenceOcclusionEngine(obs))
    nodes = {'GCS': gcs_pos, 'UAV_SURVEY': uav_survey.position, 'UAV_RELAY': uav_relay.position}
    net.update_topology(nodes, obs)

    # 1. Verify direct LoS is blocked by building
    is_occ, _ = net.occlusion.check_occlusion(gcs_pos, uav_survey.position)
    assert is_occ is True

    # 2. Verify 2-hop route via relay is established
    route = net.find_route('UAV_SURVEY', 'GCS')
    assert route == ['UAV_SURVEY', 'UAV_RELAY', 'GCS']

    # 3. Transmit survey payload and verify delivery
    pkt = NetworkPacket("SURVEY_PAYLOAD_1", "UAV_SURVEY", "GCS", "SURVEY_DATA", data_size_bytes=4096)
    delivered = net.transmit_packet(pkt, 1.0)
    assert delivered is True
    assert pkt.status == 'DELIVERED'
    assert pkt.hop_trace == ['UAV_SURVEY', 'UAV_RELAY', 'GCS']


# ============================================================================
# Scenario 2: Wide-Area Flash Flood Search & Rescue
# ============================================================================

def test_scenario2_wide_area_flash_flood_search_and_rescue():
    """
    Scenario 2: Wide-Area Flash Flood Search & Rescue.
    Survey UAVs at 450m range (beyond direct 320m LoS) route through intermediate relay drone with PDR >= 95%.
    """
    gcs_pos = np.array([0.0, 0.0, 10.0])
    # 450m range along X axis
    uav_survey = DroneState("UAV_FLOOD_SURV", "SURVEY", np.array([450.0, 0.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'SURVEYING')
    uav_relay = DroneState("UAV_FLOOD_RELAY", "RELAY", np.array([225.0, 0.0, 75.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'RELAY')

    net = ReferenceNetworkEngine()
    nodes = {'GCS': gcs_pos, 'UAV_FLOOD_SURV': uav_survey.position, 'UAV_FLOOD_RELAY': uav_relay.position}
    net.update_topology(nodes, [])

    # Transmit 20 packets from surveyor to GCS
    for i in range(20):
        pkt = NetworkPacket(f"FLOOD_PKT_{i}", "UAV_FLOOD_SURV", "GCS", "SURVEY_DATA")
        net.transmit_packet(pkt, float(i))

    metrics = net.get_metrics()
    assert metrics['pdr'] >= 0.95
    assert metrics['total_delivered'] == 20
    assert metrics['avg_latency_ms'] > 0.0


# ============================================================================
# Scenario 3: Sudden Relay Drone Battery Failure / Dropout
# ============================================================================

def test_scenario3_sudden_relay_battery_failure_and_route_repair():
    """
    Scenario 3: Sudden Relay Drone Battery Failure / Dropout.
    Active relay drone drops out; FANET dynamically discovers alternate 3-hop route within 2 simulation seconds.
    """
    gcs_pos = np.array([0.0, 0.0, 10.0])
    # Primary relay path: Survey (400m) -> Relay_Primary (200m) -> GCS
    # Backup relay chain: Survey (400m) -> Relay_B2 (300m, 100m Y) -> Relay_B1 (150m, 100m Y) -> GCS
    nodes = {
        'GCS': gcs_pos,
        'RELAY_PRIMARY': np.array([200.0, 0.0, 75.0]),
        'RELAY_B1': np.array([150.0, 120.0, 75.0]),
        'RELAY_B2': np.array([300.0, 120.0, 75.0]),
        'UAV_SURVEY': np.array([400.0, 0.0, 30.0]),
    }
    net = ReferenceNetworkEngine()
    net.update_topology(nodes, [])

    # Initial route uses primary relay
    initial_route = net.find_route('UAV_SURVEY', 'GCS')
    assert initial_route == ['UAV_SURVEY', 'RELAY_PRIMARY', 'GCS']

    # Sudden catastrophic battery depletion on RELAY_PRIMARY
    del nodes['RELAY_PRIMARY']

    # Route recomputation within 2 simulation seconds (at 30 Hz = 60 steps)
    repaired = False
    for step in range(60):
        net.update_topology(nodes, [])
        new_route = net.find_route('UAV_SURVEY', 'GCS')
        if new_route and 'RELAY_PRIMARY' not in new_route:
            repaired = True
            assert 'RELAY_B1' in new_route or 'RELAY_B2' in new_route
            # Verify packet delivery on repaired route
            repaired_pkt = NetworkPacket("REPAIRED_PKT", "UAV_SURVEY", "GCS", "SURVEY_DATA")
            assert net.transmit_packet(repaired_pkt, 1.0) is True
            break

    assert repaired is True


# ============================================================================
# Scenario 4: High-Density Multi-UAV Swarm Collision Stress
# ============================================================================

def test_scenario4_high_density_multi_uav_swarm_collision_stress():
    """
    Scenario 4: High-Density Multi-UAV Swarm Collision Stress.
    12 UAVs simultaneously converge towards central disaster hub without mid-air distance dropping below 1.5m.
    """
    center = np.array([250.0, 250.0, 50.0])
    radius = 50.0
    num_drones = 12
    drones = []
    ke = ReferenceKinematicsEngine(d_safe=6.0)
    ke.k_rep = 30.0

    for i in range(num_drones):
        angle = (2.0 * math.pi * i) / num_drones
        pos = center + np.array([radius * math.cos(angle), radius * math.sin(angle), 0.0])
        opposite_angle = angle + math.pi
        target = center + np.array([radius * math.cos(opposite_angle), radius * math.sin(opposite_angle), 0.0])

        drones.append(DroneState(
            id=f"UAV_{i+1}",
            role="SURVEY",
            position=pos,
            velocity=np.zeros(3),
            attitude=np.zeros(3),
            rotor_speeds=np.zeros(4),
            battery_soc=1.0,
            flight_mode="TRANSIT",
            target_position=target
        ))

    min_recorded_distance = float('inf')
    dt = 0.05
    # Simulate convergence over 80 steps
    for _ in range(80):
        for d in drones:
            ke.step_drone(d, dt, drones, [])

        # Pairwise distance check across all 66 pairs
        for i in range(num_drones):
            for j in range(i + 1, num_drones):
                dist = float(np.linalg.norm(drones[i].position - drones[j].position))
                min_recorded_distance = min(min_recorded_distance, dist)

    assert min_recorded_distance >= 1.5, f"Swarm collision detected: min distance {min_recorded_distance}m < 1.5m"


# ============================================================================
# Scenario 5: Heavy Multi-PoI Concurrent Inspection
# ============================================================================

def test_scenario5_heavy_multi_poi_concurrent_inspection():
    """
    Scenario 5: Heavy Multi-PoI Concurrent Inspection.
    Fleet completes 100% of prioritized PoIs, all survey datasets successfully delivered to GCS.
    """
    pois = [
        PoI("POI_1", np.array([200.0, 200.0, 30.0]), "HIGH", dwell_time_required=0.5),
        PoI("POI_2", np.array([250.0, 200.0, 30.0]), "HIGH", dwell_time_required=0.5),
        PoI("POI_3", np.array([200.0, 250.0, 30.0]), "MEDIUM", dwell_time_required=0.5),
        PoI("POI_4", np.array([250.0, 250.0, 30.0]), "MEDIUM", dwell_time_required=0.5),
    ]

    sim = ReferenceSimulationController(num_drones=8, num_pois=4, duration=5.0)
    sim.pois = pois
    sim.mission.pois = pois

    # Pre-position surveyors at their PoIs to simulate active inspection phase
    surveyors = [d for d in sim.drones if d.role == 'SURVEY']
    for idx, s in enumerate(surveyors[:4]):
        s.position = np.copy(pois[idx].position)

    # Step simulation until all PoIs completed
    for _ in range(30):
        sim.step()
        if sim.mission.completed_pois == len(pois):
            break

    assert sim.mission.completed_pois == len(pois)
    for p in pois:
        assert p.status == 'COMPLETED'
    assert sim.network.packets_delivered > 0


# ============================================================================
# Scenario 6: Full Headless End-to-End Pipeline Execution
# ============================================================================

def test_scenario6_full_headless_pipeline_execution():
    """
    Scenario 6: Full Headless End-to-End Pipeline Execution.
    Verifies headless simulation runs flawlessly, produces valid telemetry snapshot frames, and exits cleanly.
    """
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    run_sim_script = os.path.join(workspace_root, "run_simulation.py")

    if os.path.exists(run_sim_script):
        # When Milestone 5 creates run_simulation.py, run it via CLI
        cmd = [sys.executable, run_sim_script, "--headless", "--duration", "2", "--drones", "4", "--pois", "2"]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        assert result.returncode == 0
    else:
        # Before Milestone 5 lands, run controller directly to verify pipeline contract
        sim = ReferenceSimulationController(num_drones=4, num_pois=2, duration=2.0, headless=True)
        snaps = sim.run()
        assert len(snaps) == int(2.0 / sim.dt)
        assert snaps[-1].sim_time >= 1.9
        assert 'pdr' in snaps[-1].metrics
        assert snaps[-1].metrics['pdr'] >= 0.0
