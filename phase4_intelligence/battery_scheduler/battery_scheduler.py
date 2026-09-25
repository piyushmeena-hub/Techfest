"""
UAV-X Phase 4: Battery Scheduler
Predicts when each UAV needs to RTL based on its current position,
mission waypoints, battery level, and drain rate.
Inspired by: EchoRescue battery-aware mission planning.
"""
from __future__ import annotations
import math
from typing import Dict, Optional
from core.models import UAV, UAVRole, Position


class BatteryScheduler:
    """
    Predicts whether a UAV has enough battery to:
    1. Complete its current mission segment
    2. Fly back to GCS
    And triggers RTL preemptively if not.
    """

    def __init__(self, gcs_position: Position, safety_margin: float = 0.15):
        self.gcs = gcs_position
        self.safety_margin = safety_margin  # 15% buffer above RTL threshold

    def ticks_to_return(self, uav: UAV) -> float:
        """Estimate ticks needed for UAV to fly back to GCS from current position."""
        dist = uav.position.distance_to(self.gcs)
        if uav.speed <= 0:
            return float('inf')
        return dist / uav.speed   # ticks (since speed is m/tick)

    def battery_to_return(self, uav: UAV) -> float:
        """Estimate battery % consumed flying back to GCS."""
        ticks = self.ticks_to_return(uav)
        return ticks * uav.battery_drain_rate

    def should_rtl_now(self, uav: UAV) -> bool:
        """
        Return True if UAV should start RTL immediately.
        Decision: battery_remaining - battery_to_return < (rtl_threshold + safety_margin)
        """
        if uav.role == UAVRole.RTL:
            return False   # Already returning
        needed = self.battery_to_return(uav)
        safe_threshold = uav.rtl_battery_threshold + (self.safety_margin * 100)
        return uav.battery - needed < safe_threshold

    def estimated_mission_ticks(self, uav: UAV) -> Optional[float]:
        """Estimate ticks remaining to reach current waypoint."""
        if not uav.waypoints:
            return 0.0
        target = uav.waypoints[uav.current_waypoint_idx]
        dist = uav.position.distance_to(target)
        return dist / uav.speed if uav.speed > 0 else float('inf')

    def get_fleet_schedule(self, uavs: Dict[str, UAV]) -> Dict[str, dict]:
        """Return RTL schedule prediction for all UAVs."""
        schedule = {}
        for uid, uav in uavs.items():
            schedule[uid] = {
                "battery": round(uav.battery, 1),
                "ticks_to_return": round(self.ticks_to_return(uav), 1),
                "battery_to_return": round(self.battery_to_return(uav), 1),
                "should_rtl": self.should_rtl_now(uav),
                "mission_ticks_left": round(self.estimated_mission_ticks(uav) or 0, 1),
            }
        return schedule
