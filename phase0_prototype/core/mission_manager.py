"""
UAV-X Phase 0: Mission Manager
Inspired by: MAVSDK mission orchestration + EchoRescue evidence patterns

Handles:
- PoI assignment to Scout UAVs (priority-aware)
- Relay chain construction and slot management
- Battery-aware RTL triggering
- Relay handoff when a relay UAV goes low-battery
- Fault detection and automatic re-tasking
"""
from __future__ import annotations
from typing import Dict, List, Optional
import time

from .models import (
    UAV, UAVRole, UAVStatus, PoIStatus, DataPriority,
    PointOfInterest, GroundControlStation, DataPacket, Position
)
from .comm_network import CommNetwork


class MissionManager:
    """
    Central orchestrator — analogous to the MAVSDK mission layer.
    Runs every simulation tick to maintain mission state.
    """

    def __init__(self, uavs: Dict[str, UAV], pois: List[PointOfInterest],
                 gcs: GroundControlStation, network: CommNetwork,
                 relay_altitude: float = 50.0, relay_spacing: float = 400.0):
        self.uavs = uavs
        self.pois = pois
        self.gcs = gcs
        self.network = network
        self.relay_altitude = relay_altitude
        self.relay_spacing = relay_spacing

        # Priority queue (list sorted by DataPriority value)
        self.data_queue: List[DataPacket] = []
        self._packet_counter = 0

        # Event log (EchoRescue-style)
        self.event_log: List[dict] = []

        # Relay slots: positions along GCS→scout chain
        self.relay_slots: List[Position] = []
        self._compute_relay_slots()

    # ────────────────────────────────────────────
    #  Relay Slot Geometry
    # ────────────────────────────────────────────

    def _compute_relay_slots(self):
        """
        Pre-compute evenly spaced relay positions between GCS and the
        furthest PoI. Each slot is a hover position for a relay UAV.
        """
        if not self.pois:
            return
        furthest = max(self.pois, key=lambda p: self.gcs.position.distance_2d(p.position))
        gcs_pos = self.gcs.position
        target = furthest.position
        total_dist = gcs_pos.distance_2d(target)
        n_slots = int(total_dist / self.relay_spacing)
        self.relay_slots = []
        for i in range(1, n_slots + 1):
            frac = i / (n_slots + 1)
            slot = Position(
                x=gcs_pos.x + frac * (target.x - gcs_pos.x),
                y=gcs_pos.y + frac * (target.y - gcs_pos.y),
                z=self.relay_altitude
            )
            self.relay_slots.append(slot)

    # ────────────────────────────────────────────
    #  Core Tick
    # ────────────────────────────────────────────

    def tick(self, tick_number: int):
        """Run one simulation tick of mission management."""
        self._detect_faults(tick_number)
        self._trigger_rtl(tick_number)
        self._assign_relays(tick_number)
        self._assign_scouts(tick_number)
        self._drain_batteries(tick_number)
        self._log_telemetry(tick_number)

    # ────────────────────────────────────────────
    #  Fault Detection
    # ────────────────────────────────────────────

    def _detect_faults(self, tick: int):
        for uav in self.uavs.values():
            if uav.battery <= 0 and uav.status != UAVStatus.FAILED:
                uav.status = UAVStatus.FAILED
                uav.role = UAVRole.IDLE
                self._log_event(tick, "FAULT", uav.uav_id, "UAV failed — battery depleted")
                # Free up assigned PoI
                if uav.assigned_poi:
                    poi = self._get_poi(uav.assigned_poi)
                    if poi:
                        poi.status = PoIStatus.PENDING
                        poi.assigned_uav = None
                    uav.assigned_poi = None

    # ────────────────────────────────────────────
    #  Battery-Aware RTL
    # ────────────────────────────────────────────

    def _trigger_rtl(self, tick: int):
        for uav in self.uavs.values():
            if uav.status == UAVStatus.ACTIVE and uav.is_low_battery and uav.role != UAVRole.RTL:
                uav.role = UAVRole.RTL
                uav.waypoints = [self.gcs.position]
                uav.current_waypoint_idx = 0
                self._log_event(tick, "RTL", uav.uav_id,
                                f"Battery at {uav.battery:.1f}% — returning to launch")
                # Release PoI assignment
                if uav.assigned_poi:
                    poi = self._get_poi(uav.assigned_poi)
                    if poi and poi.status == PoIStatus.ASSIGNED:
                        poi.status = PoIStatus.PENDING
                        poi.assigned_uav = None
                    uav.assigned_poi = None

    # ────────────────────────────────────────────
    #  Relay Assignment
    # ────────────────────────────────────────────

    def _assign_relays(self, tick: int):
        """Ensure every relay slot has an active relay UAV."""
        for slot_idx, slot_pos in enumerate(self.relay_slots):
            # Check if slot is already covered by an active relay
            slot_covered = any(
                u.role == UAVRole.RELAY and u.relay_slot == slot_idx and u.status == UAVStatus.ACTIVE
                for u in self.uavs.values()
            )
            if slot_covered:
                continue

            # Find best idle UAV to fill this slot
            candidate = self._find_idle_uav_for_relay(slot_pos)
            if candidate:
                candidate.role = UAVRole.RELAY
                candidate.relay_slot = slot_idx
                candidate.waypoints = [slot_pos]
                candidate.current_waypoint_idx = 0
                self._log_event(tick, "RELAY_ASSIGN", candidate.uav_id,
                                f"Assigned to relay slot {slot_idx} at {slot_pos}")

    def _find_idle_uav_for_relay(self, target: Position) -> Optional[UAV]:
        idle = [u for u in self.uavs.values()
                if u.role == UAVRole.IDLE
                and u.status == UAVStatus.ACTIVE
                and not u.is_low_battery]
        if not idle:
            return None
        return min(idle, key=lambda u: u.position.distance_to(target))

    # ────────────────────────────────────────────
    #  Scout Assignment
    # ────────────────────────────────────────────

    def _assign_scouts(self, tick: int):
        """Assign idle UAVs to pending PoIs, highest priority first."""
        pending_pois = sorted(
            [p for p in self.pois if p.status == PoIStatus.PENDING],
            key=lambda p: p.priority.value   # lower value = higher priority
        )
        idle_uavs = [u for u in self.uavs.values()
                     if u.role == UAVRole.IDLE
                     and u.status == UAVStatus.ACTIVE
                     and not u.is_low_battery]

        for poi in pending_pois:
            if not idle_uavs:
                break
            # Pick nearest idle UAV to this PoI
            scout = min(idle_uavs, key=lambda u: u.position.distance_to(poi.position))
            scout.role = UAVRole.SCOUT
            scout.assigned_poi = poi.poi_id
            scout.waypoints = [poi.position]
            scout.current_waypoint_idx = 0
            poi.status = PoIStatus.ASSIGNED
            poi.assigned_uav = scout.uav_id
            idle_uavs.remove(scout)
            self._log_event(tick, "SCOUT_ASSIGN", scout.uav_id,
                            f"Assigned to PoI {poi.poi_id} (priority={poi.priority.name})")

    # ────────────────────────────────────────────
    #  Movement & Survey
    # ────────────────────────────────────────────

    def move_uavs(self, tick: int, dt: float = 1.0):
        """Move each UAV toward its current waypoint."""
        for uav in self.uavs.values():
            if uav.status != UAVStatus.ACTIVE:
                continue
            if not uav.waypoints:
                continue
            target = uav.waypoints[uav.current_waypoint_idx]
            dist = uav.position.distance_to(target)

            if dist < uav.speed * dt:
                # Arrived at waypoint
                uav.position = Position(target.x, target.y, target.z)
                self._on_waypoint_reached(uav, tick)
            else:
                # Move toward target
                ratio = uav.speed * dt / dist
                uav.position = Position(
                    x=uav.position.x + ratio * (target.x - uav.position.x),
                    y=uav.position.y + ratio * (target.y - uav.position.y),
                    z=uav.position.z + ratio * (target.z - uav.position.z),
                )
            uav.update_dynamics(dt)

    def _on_waypoint_reached(self, uav: UAV, tick: int):
        """Called when a UAV arrives at its waypoint."""
        if uav.role == UAVRole.SCOUT:
            poi = self._get_poi(uav.assigned_poi)
            if poi and poi.status == PoIStatus.ASSIGNED:
                poi.status = PoIStatus.SURVEYED
                poi.surveyed_at = tick
                # Simulate survivor detection (20% chance at critical PoIs)
                import random
                survivor = poi.priority == DataPriority.CRITICAL and random.random() < 0.4
                poi.survivor_detected = survivor
                self._enqueue_data(uav, poi, tick)
                self._log_event(tick, "SURVEY_COMPLETE", uav.uav_id,
                                f"PoI {poi.poi_id} surveyed. Survivor={survivor}")
                uav.role = UAVRole.IDLE
                uav.assigned_poi = None
                uav.waypoints = []

        elif uav.role == UAVRole.RTL:
            uav.role = UAVRole.IDLE
            uav.status = UAVStatus.LANDED
            self._log_event(tick, "LANDED", uav.uav_id, "Safely landed at GCS")

    # ────────────────────────────────────────────
    #  Data Queue (Priority Queue)
    # ────────────────────────────────────────────

    def _enqueue_data(self, uav: UAV, poi: PointOfInterest, tick: int):
        """Create a data packet from surveyed PoI and push to priority queue."""
        import heapq
        import random
        priority = DataPriority.CRITICAL if poi.survivor_detected else poi.priority
        confidence = round(random.uniform(0.89, 0.97), 3) if poi.survivor_detected else round(random.uniform(0.05, 0.18), 3)
        temp_c = round(random.uniform(36.4, 37.6), 1) if poi.survivor_detected else round(random.uniform(16.5, 20.2), 1)
        self._packet_counter += 1
        packet = DataPacket(
            priority=priority,
            packet_id=f"PKT-{self._packet_counter:04d}",
            source_uav=uav.uav_id,
            poi_id=poi.poi_id,
            data={
                "survivor": poi.survivor_detected,
                "confidence": confidence,
                "temperature_c": temp_c,
                "gps": (round(47.397742 + poi.position.y * 0.0000089, 6),
                        round(8.545594 + poi.position.x * 0.0000089, 6)),
                "imagery_band": "LWIR 8-14μm + 4K RGB",
                "tick": tick
            },
            created_at=tick,
        )
        heapq.heappush(self.data_queue, packet)
        self._log_event(tick, "DATA_ENQUEUE", uav.uav_id,
                        f"Packet {packet.packet_id} queued (priority={priority.name}) [Thermal: {temp_c}°C, Conf: {int(confidence*100)}%]")

    def flush_data_queue(self, tick: int) -> dict:
        """Attempt to deliver all queued packets through the network."""
        import heapq
        delivered = 0
        failed = 0
        while self.data_queue:
            packet = heapq.heappop(self.data_queue)
            success, hops = self.network.deliver_packet(packet, tick)
            if success:
                delivered += 1
                self._log_event(tick, "DELIVERED", packet.source_uav,
                                f"Packet {packet.packet_id} delivered via {hops} hops")
            else:
                failed += 1
                # Re-queue for retry
                heapq.heappush(self.data_queue, packet)
                self._log_event(tick, "DELIVERY_FAIL", packet.source_uav,
                                f"Packet {packet.packet_id} delivery failed — will retry")
                break   # Don't retry all — wait for next tick
        return {"delivered": delivered, "failed_retry": failed}

    # ────────────────────────────────────────────
    #  Battery Drain
    # ────────────────────────────────────────────

    def _drain_batteries(self, tick: int):
        for uav in self.uavs.values():
            if uav.status == UAVStatus.ACTIVE:
                uav.drain_battery()

    def _log_telemetry(self, tick: int):
        for uav in self.uavs.values():
            uav.log_telemetry(tick)

    # ────────────────────────────────────────────
    #  Helpers
    # ────────────────────────────────────────────

    def _get_poi(self, poi_id: Optional[str]) -> Optional[PointOfInterest]:
        if poi_id is None:
            return None
        return next((p for p in self.pois if p.poi_id == poi_id), None)

    def _log_event(self, tick: int, event_type: str, actor: str, message: str):
        self.event_log.append({
            "tick": tick,
            "type": event_type,
            "actor": actor,
            "message": message,
        })

    # ────────────────────────────────────────────
    #  Mission Summary
    # ────────────────────────────────────────────

    def mission_summary(self) -> dict:
        surveyed = [p for p in self.pois if p.status == PoIStatus.SURVEYED]
        survivors = [p for p in surveyed if p.survivor_detected]
        failed_uavs = [u for u in self.uavs.values() if u.status == UAVStatus.FAILED]
        return {
            "total_pois": len(self.pois),
            "surveyed_pois": len(surveyed),
            "pending_pois": len([p for p in self.pois if p.status == PoIStatus.PENDING]),
            "survivors_detected": len(survivors),
            "survivor_pois": [p.poi_id for p in survivors],
            "packets_delivered": len(self.gcs.received_packets),
            "failed_uavs": [u.uav_id for u in failed_uavs],
            "total_events": len(self.event_log),
        }
