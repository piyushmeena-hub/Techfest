"""
tests/unit/test_cbba_consensus.py: Unit tests for CBBA Task Allocation & Dynamic Relay Relief.
"""

import numpy as np
import pytest

from sim.drone import Drone
from sim.planning import CBBASolver, RelayReliefManager
from sim.types import DroneRole, FlightMode


def test_cbba_conflict_free_allocation():
    """Verify CBBA allocates PoIs to surveyors with zero duplicate assignments."""
    solver = CBBASolver(max_bundle_size=2)

    drones = {
        "UAV_1": Drone("UAV_1", role=DroneRole.SURVEY, initial_pos=[-50.0, 0.0, 30.0]),
        "UAV_2": Drone("UAV_2", role=DroneRole.SURVEY, initial_pos=[50.0, 0.0, 30.0]),
    }

    pois = {
        "POI_WEST": {"id": "POI_WEST", "position": np.array([-60.0, 80.0, 25.0]), "priority": "HIGH", "is_completed": False},
        "POI_EAST": {"id": "POI_EAST", "position": np.array([60.0, 80.0, 25.0]), "priority": "HIGH", "is_completed": False},
        "POI_MID": {"id": "POI_MID", "position": np.array([0.0, 80.0, 25.0]), "priority": "MEDIUM", "is_completed": False},
    }

    assignments = solver.solve(drones, pois)

    # 1. POI_WEST should go to UAV_1 (closer)
    assert assignments["POI_WEST"] == "UAV_1"
    # 2. POI_EAST should go to UAV_2 (closer)
    assert assignments["POI_EAST"] == "UAV_2"
    # 3. All assignments must be valid
    assigned_drones = [v for v in assignments.values() if v is not None]
    assert len(assigned_drones) == 3


def test_relay_relief_rotation():
    """Verify RelayReliefManager triggers handover when relay battery falls below threshold."""
    manager = RelayReliefManager(low_battery_threshold=0.35, takeover_min_battery=0.65)

    # Create relay with low battery (30%)
    relay = Drone("RELAY_1", role=DroneRole.RELAY, initial_pos=[0.0, 0.0, 75.0])
    relay.set_flight_mode(FlightMode.RELAY)
    relay.battery.soc = 0.30

    # Create surveyor with high battery (85%)
    surveyor = Drone("UAV_1", role=DroneRole.SURVEY, initial_pos=[0.0, 20.0, 30.0])
    surveyor.set_flight_mode(FlightMode.TRANSIT)
    surveyor.battery.soc = 0.85

    drones = {"RELAY_1": relay, "UAV_1": surveyor}

    # Evaluate relief
    handover = manager.evaluate_relief_rotation(drones, sim_time=120.0)
    assert handover is not None
    assert handover == ("RELAY_1", "UAV_1")

    # Execute handover
    manager.execute_handover(relay, surveyor)

    # Verify roles and flight modes updated
    assert surveyor.role == DroneRole.RELAY
    assert surveyor.flight_mode == FlightMode.RELAY
    assert relay.flight_mode == FlightMode.RTL
