"""
sim/mission.py: Disaster PoI Survey Mission Control & Dynamic Role Allocation.

Implements:
1. Multi-priority Disaster Points of Interest (PoI) management (Survivor Search, Structural Collapse, Hazard Zone).
2. Autonomous MAVSDK-compliant Finite State Machine (FSM) transitions:
   TAKEOFF -> TRANSIT -> SURVEYING -> RELAY -> RTL -> LANDED -> COMPLETED.
3. Dynamic Fleet Role Allocation:
   - Survey UAVs: Dispatched to inspect high-priority PoIs and stream reconnaissance telemetry.
   - Relay UAVs: Deployed as elevated aerial communication bridges linking surveyors to GCS.
4. PoI dwell time accumulation, sensor data payload generation, and mission completion tracking.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union
import numpy as np

from sim.types import DroneRole, FlightMode


class DisasterMissionManager:
    """
    High-Level Disaster Survey Mission Orchestrator.
    
    Coordinates the multi-UAV fleet:
    - Allocates roles between Surveyors and Relays.
    - Dispatches Survey UAVs to pending PoIs in order of priority.
    - Manages PoI dwell time, data gathering, and state transitions.
    - Coordinates Return-to-Launch (RTL) upon mission or battery exhaustion.
    """

    def __init__(
        self,
        gcs_position: Sequence[float] = (0.0, 0.0, 0.0),
        survey_dwell_radius: float = 12.0,
    ) -> None:
        self.gcs_position = np.array(gcs_position, dtype=np.float64)
        self.survey_dwell_radius = survey_dwell_radius
        self.total_mission_time: float = 0.0
        self.completed_pois_count: int = 0
        
        # Swarm intelligence modules
        from sim.planning import CBBASolver, RelayReliefManager
        self.cbba_solver = CBBASolver(max_bundle_size=2)
        self.relief_manager = RelayReliefManager(low_battery_threshold=0.35, takeover_min_battery=0.65)

    def update(
        self,
        drones: Dict[str, Any],
        pois: Dict[str, Dict[str, Any]],
        dt: float,
    ) -> None:
        """
        Advance mission state by dt seconds.
        Assigns pending PoIs, updates dwell times, and transitions drone FSM states.
        """
        self.total_mission_time += dt

        # 1. Ensure role allocation for fleet
        self._ensure_role_allocation(drones)

        # 2. Check dynamic relay battery relief rotation
        handover = self.relief_manager.evaluate_relief_rotation(drones, self.total_mission_time)
        if handover:
            r_id, s_id = handover
            self.relief_manager.execute_handover(drones[r_id], drones[s_id])

        # 3. Assign available survey drones to pending PoIs
        self._assign_pending_pois(drones, pois)

        # 4. Update drones and their active mission targets
        for drone_id, drone in drones.items():
            self._update_drone_fsm(drone, pois, dt)

        # Count completed PoIs
        self.completed_pois_count = sum(1 for p in pois.values() if p.get("is_completed", False))

    def _ensure_role_allocation(self, drones: Dict[str, Any]) -> None:
        """Ensure an optimal split of Survey and Relay drones in the fleet."""
        sorted_ids = sorted(drones.keys())
        n = len(sorted_ids)
        if n == 0:
            return

        # If roles already assigned, preserve them
        has_survey = any(d.role == DroneRole.SURVEY for d in drones.values())
        has_relay = any(d.role == DroneRole.RELAY for d in drones.values())
        if has_survey and (has_relay or n <= 2):
            return

        # Allocate approx 2/3 as Surveyors, 1/3 as Relays (minimum 1 relay if n >= 3)
        num_relays = max(1, n // 3) if n >= 3 else 0
        num_surveyors = n - num_relays

        for i, d_id in enumerate(sorted_ids):
            drone = drones[d_id]
            if i < num_surveyors:
                drone.role = DroneRole.SURVEY
            else:
                drone.role = DroneRole.RELAY

    def _assign_pending_pois(self, drones: Dict[str, Any], pois: Dict[str, Dict[str, Any]]) -> None:
        """Assign unassigned Survey drones to high-priority pending PoIs."""
        # Find unassigned pending PoIs, sorted by priority (HIGH first)
        pending_pois = [
            p for p in pois.values()
            if not p.get("is_completed", False) and p.get("assigned_drone_id") is None
        ]
        priority_order = {"CRITICAL": 0, "HIGH": 1, "MEDIUM": 2, "LOW": 3}
        pending_pois.sort(key=lambda p: priority_order.get(p.get("priority", "MEDIUM"), 2))

        # Find idle or unassigned survey drones
        available_surveyors = [
            d for d in drones.values()
            if d.role == DroneRole.SURVEY and d.assigned_poi_id is None
            and d.flight_mode not in (FlightMode.RTL, FlightMode.LANDED, FlightMode.COMPLETED)
        ]

        # 1. Consensus-Based Bundle Algorithm (CBBA) distributed auction
        if hasattr(self, "cbba_solver"):
            cbba_map = self.cbba_solver.solve(drones, pois)
            for p in pending_pois:
                p_id = p["id"]
                d_id = cbba_map.get(p_id)
                if d_id and d_id in drones and drones[d_id] in available_surveyors:
                    selected_drone = drones[d_id]
                    available_surveyors.remove(selected_drone)
                    p["assigned_drone_id"] = selected_drone.id
                    selected_drone.assigned_poi_id = p_id
                    selected_drone.set_target_waypoint(p["position"])

        # 2. Greedy fallback for any remaining unassigned
        for poi in pending_pois:
            if poi.get("assigned_drone_id") is not None or not available_surveyors:
                continue
            poi_pos = poi["position"]
            available_surveyors.sort(key=lambda d: float(np.linalg.norm(d.position - poi_pos)))
            selected_drone = available_surveyors.pop(0)
            poi["assigned_drone_id"] = selected_drone.id
            selected_drone.assigned_poi_id = poi["id"]
            selected_drone.set_target_waypoint(poi_pos)

    def _update_drone_fsm(self, drone: Any, pois: Dict[str, Dict[str, Any]], dt: float) -> None:
        """Handle individual UAV FSM updates based on mission progress."""
        # Emergency low battery check (RTL threshold)
        if hasattr(drone, "battery") and drone.battery.soc <= 0.15:
            if drone.flight_mode not in (FlightMode.RTL, FlightMode.LANDED, FlightMode.COMPLETED):
                drone.set_flight_mode(FlightMode.RTL)
                drone.set_target_waypoint(self.gcs_position + np.array([0.0, 0.0, 15.0]))
                return

        # Relay drone mission logic
        if drone.role == DroneRole.RELAY:
            # If all PoIs are completed, relay drones return to GCS
            all_done = (len(pois) > 0 and all(p.get("is_completed", False) for p in pois.values()))
            if all_done:
                if drone.flight_mode not in (FlightMode.RTL, FlightMode.LANDED, FlightMode.COMPLETED):
                    drone.set_flight_mode(FlightMode.RTL)
                    drone.set_target_waypoint(self.gcs_position + np.array([0.0, 0.0, 25.0]))
                elif drone.flight_mode == FlightMode.RTL:
                    dist_to_gcs = float(np.linalg.norm(drone.position[:2] - self.gcs_position[:2]))
                    if dist_to_gcs < 8.0:
                        drone.set_flight_mode(FlightMode.COMPLETED)
                        drone.set_target_waypoint(self.gcs_position)
                return

            if drone.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
                drone.set_flight_mode(FlightMode.TAKEOFF)
            elif drone.flight_mode == FlightMode.TAKEOFF and drone.position[2] >= 30.0:
                drone.set_flight_mode(FlightMode.RELAY)
            return

        # Survey drone mission logic
        if drone.role == DroneRole.SURVEY:
            # Check state transitions
            if drone.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
                drone.set_flight_mode(FlightMode.TAKEOFF)
                drone.set_target_waypoint(drone.position + np.array([0.0, 0.0, 30.0]))

            elif drone.flight_mode == FlightMode.TAKEOFF:
                if drone.position[2] >= 25.0:
                    drone.set_flight_mode(FlightMode.TRANSIT)
                    if drone.assigned_poi_id is not None and drone.assigned_poi_id in pois:
                        poi = pois[drone.assigned_poi_id]
                        drone.set_target_waypoint(poi["position"])

            elif drone.flight_mode == FlightMode.TRANSIT:
                if drone.assigned_poi_id is not None and drone.assigned_poi_id in pois:
                    poi = pois[drone.assigned_poi_id]
                    drone.set_target_waypoint(poi["position"])
                    dist_to_poi = float(np.linalg.norm(drone.position[:2] - poi["position"][:2]))
                    if dist_to_poi < self.survey_dwell_radius:
                        # Arrived at PoI: begin survey dwelling
                        drone.set_flight_mode(FlightMode.SURVEYING)
                        drone.is_transmitting = True
                else:
                    all_done = (len(pois) > 0 and all(p.get("is_completed", False) for p in pois.values()))
                    if all_done:
                        drone.set_flight_mode(FlightMode.RTL)
                        drone.set_target_waypoint(self.gcs_position + np.array([0.0, 0.0, 20.0]))

            elif drone.flight_mode == FlightMode.SURVEYING:
                if drone.assigned_poi_id is not None and drone.assigned_poi_id in pois:
                    poi = pois[drone.assigned_poi_id]
                    drone.set_target_waypoint(poi["position"])
                    poi["current_dwell_time"] += dt
                    drone.dwell_time = poi["current_dwell_time"]
                    drone.is_transmitting = True

                    # Check if inspection complete
                    if poi["current_dwell_time"] >= poi["required_dwell_time"]:
                        poi["is_completed"] = True
                        drone.assigned_poi_id = None
                        drone.dwell_time = 0.0
                        drone.is_transmitting = False

                        # Check if any more PoIs remain
                        has_more = any(not p["is_completed"] for p in pois.values())
                        if has_more:
                            drone.set_flight_mode(FlightMode.TRANSIT)
                        else:
                            # All PoIs surveyed: Return to GCS
                            drone.set_flight_mode(FlightMode.RTL)
                            drone.set_target_waypoint(self.gcs_position + np.array([0.0, 0.0, 20.0]))

            elif drone.flight_mode == FlightMode.RTL:
                dist_to_gcs = float(np.linalg.norm(drone.position[:2] - self.gcs_position[:2]))
                if dist_to_gcs < 8.0:
                    drone.set_flight_mode(FlightMode.COMPLETED)
                    drone.set_target_waypoint(self.gcs_position)
