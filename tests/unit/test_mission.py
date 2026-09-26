"""
tests/unit/test_mission.py: Unit tests for DisasterMissionManager and FSM control.
"""

import numpy as np
import pytest

from sim.core import SimulationConfig, SwarmSimulationCore
from sim.drone import Drone
from sim.mission import DisasterMissionManager
from sim.types import DroneRole, FlightMode


def test_mission_role_allocation_and_poi_assignment():
    mgr = DisasterMissionManager(gcs_position=[0.0, 0.0, 0.0])
    drones = {
        f"D{i}": Drone(f"D{i}", initial_pos=np.array([float(i * 5), 0.0, 0.0]))
        for i in range(4)
    }
    pois = {
        "POI_1": {
            "id": "POI_1",
            "position": np.array([100.0, 50.0, 30.0]),
            "priority": "HIGH",
            "required_dwell_time": 5.0,
            "current_dwell_time": 0.0,
            "is_completed": False,
            "assigned_drone_id": None,
        },
        "POI_2": {
            "id": "POI_2",
            "position": np.array([120.0, -80.0, 30.0]),
            "priority": "MEDIUM",
            "required_dwell_time": 5.0,
            "current_dwell_time": 0.0,
            "is_completed": False,
            "assigned_drone_id": None,
        }
    }

    mgr.update(drones, pois, dt=0.05)

    # Verify role split: at least 1 relay and remaining surveyors
    roles = [d.role for d in drones.values()]
    assert DroneRole.RELAY in roles
    assert DroneRole.SURVEY in roles

    # Verify high priority PoI assigned first
    assert pois["POI_1"]["assigned_drone_id"] is not None
