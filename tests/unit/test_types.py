"""
tests/unit/test_types.py: Unit tests for core data models, enums, and types.
"""

from __future__ import annotations

import json
import pytest
import numpy as np

from sim.types import (
    AIRSPACE_CORRIDORS,
    AltitudeCorridor,
    BatteryModel,
    DroneLimits,
    DroneRole,
    DroneState,
    FlightMode,
    PoIPriority,
    TelemetrySnapshot,
)


def test_drone_role_enum():
    """Verify DroneRole enum members and string values."""
    assert DroneRole.SURVEY.value == "SURVEY"
    assert DroneRole.RELAY.value == "RELAY"
    assert DroneRole.RESERVE.value == "RESERVE"
    assert str(DroneRole.SURVEY) == "SURVEY"


def test_flight_mode_enum():
    """Verify MAVSDK-compliant FlightMode enum members."""
    modes = {
        FlightMode.IDLE,
        FlightMode.TAKEOFF,
        FlightMode.TRANSIT,
        FlightMode.SURVEYING,
        FlightMode.RELAY,
        FlightMode.DATA_TX,
        FlightMode.RTL,
        FlightMode.LANDING,
        FlightMode.LANDED,
        FlightMode.EMERGENCY_LAND,
        FlightMode.COMPLETED,
    }
    assert len(modes) == 11
    assert FlightMode.IDLE.value == "IDLE"
    assert FlightMode.SURVEYING.value == "SURVEYING"


def test_poi_priority_enum():
    """Verify PoIPriority levels."""
    assert PoIPriority.HIGH.value == "HIGH"
    assert PoIPriority.CRITICAL.value == "CRITICAL"


def test_drone_limits_defaults():
    """Verify default physical and kinematic limits."""
    limits = DroneLimits()
    assert limits.max_speed_xy == 10.0
    assert limits.max_speed_z_up == 3.5
    assert limits.max_speed_z_down == 2.5
    assert limits.max_accel == 4.0
    assert limits.max_tilt_rad == pytest.approx(0.5236, abs=1e-4)
    assert limits.max_yaw_rate == pytest.approx(1.5708, abs=1e-4)
    assert limits.mass_kg == 1.20
    assert limits.hover_thrust_n == pytest.approx(11.768, abs=1e-3)
    assert limits.separation_radius == 6.0
    assert limits.collision_radius == 1.0


def test_battery_model_energy_calculation():
    """Verify total energy capacity calculation in Joules (4S 5000mAh)."""
    battery = BatteryModel(capacity_mah=5000.0, nominal_voltage=14.8)
    expected_joules = 5.0 * 14.8 * 3600.0  # 266,400 J
    assert battery.total_energy_joules == pytest.approx(expected_joules, rel=1e-6)


def test_battery_model_step_depletion():
    """Verify power consumption and SoC depletion during hover."""
    battery = BatteryModel()
    initial_soc = battery.soc
    assert initial_soc == 1.0

    # Hover for 10 seconds: speed=0, accel=0, not transmitting, not surveying
    power = battery.step(dt=10.0, speed=0.0, accel=0.0, is_transmitting=False, is_surveying=False)
    expected_power = 12.0 + 180.0 + 3.0  # 195 W
    assert power == pytest.approx(expected_power, rel=1e-5)

    expected_energy = expected_power * 10.0  # 1950 J
    expected_soc_drop = expected_energy / battery.total_energy_joules
    assert battery.soc == pytest.approx(1.0 - expected_soc_drop, rel=1e-5)


def test_battery_model_thresholds():
    """Verify RTL and Emergency landing threshold triggers."""
    battery = BatteryModel()
    battery.soc = 0.26
    assert not battery.is_low()
    assert not battery.is_critical()

    battery.soc = 0.25
    assert battery.is_low()
    assert not battery.is_critical()

    battery.soc = 0.10
    assert battery.is_low()
    assert battery.is_critical()

    battery.soc = 0.05
    assert battery.is_critical()


def test_battery_model_remaining_flight_time():
    """Verify endurance estimation."""
    battery = BatteryModel()
    endurance_s = battery.remaining_flight_time_s(average_power_w=195.0)
    expected_s = 266400.0 / 195.0
    assert endurance_s == pytest.approx(expected_s, rel=1e-4)


def test_battery_model_reset():
    """Verify battery recharge resets SoC to 1.0."""
    battery = BatteryModel()
    battery.step(dt=60.0, speed=5.0, accel=2.0)
    assert battery.soc < 1.0
    battery.reset()
    assert battery.soc == 1.0


def test_altitude_corridor_containment():
    """Verify 4-tier altitude corridor containment logic."""
    tier1 = AIRSPACE_CORRIDORS["TIER_1_LAUNCH"]
    assert tier1.contains(0.0)
    assert tier1.contains(10.0)
    assert tier1.contains(20.0)
    assert not tier1.contains(25.0)

    tier2 = AIRSPACE_CORRIDORS["TIER_2_SURVEY"]
    assert tier2.contains(35.0)
    assert not tier2.contains(50.0)

    tier4 = AIRSPACE_CORRIDORS["TIER_4_RELAY"]
    assert tier4.contains(80.0)
    assert not tier4.contains(65.0)


def test_drone_state_to_dict():
    """Verify DroneState serialization produces valid JSON-compliant dictionary."""
    state = DroneState(
        id="UAV_01",
        role=DroneRole.SURVEY,
        position=np.array([12.3456, -78.9012, 34.5678]),
        velocity=np.array([1.234, -2.345, 0.567]),
        attitude=np.array([0.05, -0.08, 1.57]),
        rotor_speeds=np.array([990.2, 995.1, 988.4, 992.0]),
        battery_soc=0.87654,
        flight_mode=FlightMode.SURVEYING,
        assigned_poi_id="POI_ALPHA",
        target_position=np.array([15.0, -80.0, 35.0]),
    )
    d = state.to_dict()
    assert d["id"] == "UAV_01"
    assert d["role"] == "SURVEY"
    assert d["flight_mode"] == "SURVEYING"
    assert d["position"] == [12.346, -78.901, 34.568]
    assert d["battery_soc"] == 0.8765
    assert d["battery_pct"] == 87.7
    assert d["assigned_poi_id"] == "POI_ALPHA"
    assert d["target_position"] == [15.0, -80.0, 35.0]

    # Verify JSON serializability
    json_str = json.dumps(d)
    assert "UAV_01" in json_str


def test_telemetry_snapshot_to_dict():
    """Verify TelemetrySnapshot serialization with visualizer aliases."""
    snapshot = TelemetrySnapshot(
        sim_time=12.3456,
        drones=[{"id": "UAV_1", "role": "SURVEY"}],
        gcs={"position": [0.0, 0.0, 0.0], "comm_radius": 80.0, "packets_received": 5},
        pois=[{"id": "POI_1", "progress": 50.0, "is_completed": False}],
        active_routes=[["UAV_1", "GCS"]],
        links=[{"source": "UAV_1", "target": "GCS", "snr": 25.0, "status": "ACTIVE"}],
        packets=[],
        metrics={"pdr": 1.0, "avg_latency_ms": 5.2, "completed_pois": 0},
    )
    d = snapshot.to_dict()
    assert d["sim_time"] == 12.346
    assert d["timestamp"] == 12.346
    assert d["swarm"] == d["drones"]
    assert d["routes"] == d["active_routes"]
    json_str = json.dumps(d)
    assert "UAV_1" in json_str
