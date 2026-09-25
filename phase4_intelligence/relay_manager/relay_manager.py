"""
UAV-X Phase 4: Relay Manager
Handles dynamic relay chain maintenance, slot handoff, and redundancy.
Extracted from MissionManager for modularity in Phase 4 full-stack.
"""
from __future__ import annotations
from typing import Dict, List, Optional, Tuple
from core.models import UAV, UAVRole, UAVStatus, Position


class RelayManager:
    """
    Manages the relay chain between the GCS and the furthest PoIs.
    Ensures continuous comms coverage even as UAVs RTL or fail.
    Inspired by: UAV Swarm Network Simulator coverage algorithms.
    """

    def __init__(self, relay_slots: List[Position], uavs: Dict[str, UAV],
                 handoff_battery_threshold: float = 40.0):
        self.relay_slots = relay_slots
        self.uavs = uavs
        self.handoff_threshold = handoff_battery_threshold
        # slot_idx -> uav_id currently assigned
        self.slot_assignments: Dict[int, Optional[str]] = {
            i: None for i in range(len(relay_slots))
        }

    def get_slot_uav(self, slot_idx: int) -> Optional[UAV]:
        uid = self.slot_assignments.get(slot_idx)
        return self.uavs.get(uid) if uid else None

    def needs_handoff(self, slot_idx: int) -> bool:
        """Return True if the relay in this slot should be handed off."""
        uav = self.get_slot_uav(slot_idx)
        if uav is None:
            return True
        if uav.status != UAVStatus.ACTIVE:
            return True
        if uav.battery <= self.handoff_threshold:
            return True
        return False

    def assign_relay(self, slot_idx: int, uav: UAV):
        """Assign a UAV to a relay slot."""
        uav.role = UAVRole.RELAY
        uav.relay_slot = slot_idx
        uav.waypoints = [self.relay_slots[slot_idx]]
        uav.current_waypoint_idx = 0
        self.slot_assignments[slot_idx] = uav.uav_id

    def release_slot(self, slot_idx: int):
        """Release a relay slot (UAV is leaving)."""
        self.slot_assignments[slot_idx] = None

    def get_chain_status(self) -> List[dict]:
        """Return status of each relay slot in the chain."""
        result = []
        for i, pos in enumerate(self.relay_slots):
            uav = self.get_slot_uav(i)
            result.append({
                "slot": i,
                "position": (pos.x, pos.y, pos.z),
                "uav": uav.uav_id if uav else None,
                "battery": round(uav.battery, 1) if uav else None,
                "covered": uav is not None and uav.status == UAVStatus.ACTIVE,
            })
        return result

    def find_replacement(self, slot_idx: int) -> Optional[UAV]:
        """Find the best idle UAV to replace a relay in the given slot."""
        target = self.relay_slots[slot_idx]
        candidates = [
            u for u in self.uavs.values()
            if u.role == UAVRole.IDLE
            and u.status == UAVStatus.ACTIVE
            and not u.is_low_battery
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda u: u.position.distance_to(target))
