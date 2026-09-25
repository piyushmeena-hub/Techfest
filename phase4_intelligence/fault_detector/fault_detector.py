"""
UAV-X Phase 4: Fault Detector
Real-time monitoring of UAV health, link quality, and mission progress.
Detects: battery failures, comms blackouts, stuck UAVs, relay chain gaps.
Inspired by: EchoRescue failure-injection testing patterns.
"""
from __future__ import annotations
from typing import Dict, List, Optional, Callable
from core.models import UAV, UAVStatus, UAVRole
from core.comm_network import CommNetwork, LinkStatus


class FaultType:
    BATTERY_CRITICAL  = "BATTERY_CRITICAL"
    UAV_UNRESPONSIVE  = "UAV_UNRESPONSIVE"
    LINK_DOWN         = "LINK_DOWN"
    RELAY_CHAIN_BREAK = "RELAY_CHAIN_BREAK"
    MISSION_STUCK     = "MISSION_STUCK"
    ISOLATION         = "ISOLATION"


class Fault:
    def __init__(self, fault_type: str, actor: str, tick: int, detail: str):
        self.fault_type = fault_type
        self.actor = actor
        self.tick = tick
        self.detail = detail
        self.resolved = False

    def __repr__(self):
        status = "RESOLVED" if self.resolved else "ACTIVE"
        return f"Fault({self.fault_type}, {self.actor}, tick={self.tick}, [{status}])"


class FaultDetector:
    """
    Monitors the fleet and network for faults every tick.
    Fires registered callbacks when faults are detected.
    """

    def __init__(self, uavs: Dict[str, UAV], network: CommNetwork,
                 stuck_threshold: int = 30):
        self.uavs = uavs
        self.network = network
        self.stuck_threshold = stuck_threshold  # ticks before declaring UAV stuck
        self.active_faults: List[Fault] = []
        self.fault_history: List[Fault] = []
        self._position_history: Dict[str, list] = {uid: [] for uid in uavs}
        self._callbacks: List[Callable] = []

    def register_callback(self, callback: Callable[[Fault], None]):
        """Register a function to be called when a fault is detected."""
        self._callbacks.append(callback)

    def _fire(self, fault: Fault):
        self.active_faults.append(fault)
        self.fault_history.append(fault)
        for cb in self._callbacks:
            cb(fault)

    def scan(self, tick: int):
        """Run all fault detection checks for this tick."""
        self._check_battery(tick)
        self._check_isolation(tick)
        self._check_relay_chain(tick)
        self._check_stuck(tick)
        self._resolve_cleared(tick)

    def _check_battery(self, tick: int):
        for uav in self.uavs.values():
            if uav.battery <= 5.0 and uav.status == UAVStatus.ACTIVE:
                existing = [f for f in self.active_faults
                            if f.fault_type == FaultType.BATTERY_CRITICAL
                            and f.actor == uav.uav_id and not f.resolved]
                if not existing:
                    self._fire(Fault(FaultType.BATTERY_CRITICAL, uav.uav_id, tick,
                                     f"Battery critically low: {uav.battery:.1f}%"))

    def _check_isolation(self, tick: int):
        report = self.network.get_connectivity_report()
        for isolated_id in report["isolated_nodes"]:
            existing = [f for f in self.active_faults
                        if f.fault_type == FaultType.ISOLATION
                        and f.actor == isolated_id and not f.resolved]
            if not existing:
                self._fire(Fault(FaultType.ISOLATION, isolated_id, tick,
                                 f"UAV {isolated_id} has no route to GCS"))

    def _check_relay_chain(self, tick: int):
        relay_uavs = [u for u in self.uavs.values() if u.role == UAVRole.RELAY]
        if not relay_uavs:
            return
        # Check for gaps between relay UAVs
        relay_positions = sorted(relay_uavs, key=lambda u: u.position.distance_to(
            next(iter(self.uavs.values())).position))
        for i in range(len(relay_positions) - 1):
            dist = relay_positions[i].position.distance_to(relay_positions[i+1].position)
            if dist > self.network.max_range:
                self._fire(Fault(FaultType.RELAY_CHAIN_BREAK,
                                 f"{relay_positions[i].uav_id}-{relay_positions[i+1].uav_id}",
                                 tick, f"Gap {dist:.0f}m > max range {self.network.max_range}m"))

    def _check_stuck(self, tick: int):
        for uid, uav in self.uavs.items():
            if uav.role in (UAVRole.IDLE, UAVRole.RTL):
                continue
            pos = (round(uav.position.x, 1), round(uav.position.y, 1))
            hist = self._position_history[uid]
            hist.append(pos)
            if len(hist) > self.stuck_threshold:
                hist.pop(0)
            if len(hist) == self.stuck_threshold and len(set(hist)) == 1:
                existing = [f for f in self.active_faults
                            if f.fault_type == FaultType.MISSION_STUCK
                            and f.actor == uid and not f.resolved]
                if not existing:
                    self._fire(Fault(FaultType.MISSION_STUCK, uid, tick,
                                     f"UAV hasn't moved in {self.stuck_threshold} ticks"))

    def _resolve_cleared(self, tick: int):
        report = self.network.get_connectivity_report()
        for fault in self.active_faults:
            if fault.resolved:
                continue
            if fault.fault_type == FaultType.ISOLATION:
                if fault.actor not in report["isolated_nodes"]:
                    fault.resolved = True
            elif fault.fault_type == FaultType.BATTERY_CRITICAL:
                uav = self.uavs.get(fault.actor)
                if uav and uav.battery > 10.0:
                    fault.resolved = True

    def get_summary(self) -> dict:
        active = [f for f in self.active_faults if not f.resolved]
        return {
            "active_faults": len(active),
            "total_faults": len(self.fault_history),
            "fault_types": list({f.fault_type for f in active}),
            "details": [f"{f.fault_type}:{f.actor}" for f in active],
        }
