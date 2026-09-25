"""
UAV-X Phase 0: Core Data Models
Defines all entities: UAV, PoI, GCS, Link, Mission
"""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple
import math
import time


# ─────────────────────────────────────────────
#  Enumerations
# ─────────────────────────────────────────────

class UAVRole(Enum):
    SCOUT = "Scout"       # Flies to PoIs and collects data
    RELAY = "Relay"       # Hovers to relay comms between nodes
    IDLE  = "Idle"        # On ground or awaiting assignment
    RTL   = "RTL"         # Returning to Launch (low battery)


class UAVStatus(Enum):
    ACTIVE   = "Active"
    LOW_BAT  = "LowBattery"
    FAILED   = "Failed"
    LANDED   = "Landed"


class PoIStatus(Enum):
    PENDING   = "Pending"
    ASSIGNED  = "Assigned"
    SURVEYED  = "Surveyed"


class DataPriority(Enum):
    CRITICAL  = 0   # Survivor detected — always first
    HIGH      = 1   # Structural collapse risk
    MEDIUM    = 2   # General situational data
    LOW       = 3   # Routine imagery


class LinkStatus(Enum):
    UP    = "Up"
    WEAK  = "Weak"
    DOWN  = "Down"


# ─────────────────────────────────────────────
#  Position
# ─────────────────────────────────────────────

@dataclass
class Position:
    x: float   # metres east
    y: float   # metres north
    z: float   # metres altitude

    def distance_to(self, other: "Position") -> float:
        return math.sqrt(
            (self.x - other.x) ** 2 +
            (self.y - other.y) ** 2 +
            (self.z - other.z) ** 2
        )

    def distance_2d(self, other: "Position") -> float:
        return math.sqrt(
            (self.x - other.x) ** 2 +
            (self.y - other.y) ** 2
        )

    def __repr__(self):
        return f"Pos({self.x:.1f}, {self.y:.1f}, alt={self.z:.1f})"


# ─────────────────────────────────────────────
#  Point of Interest
# ─────────────────────────────────────────────

@dataclass
class PointOfInterest:
    poi_id: str
    position: Position
    priority: DataPriority
    status: PoIStatus = PoIStatus.PENDING
    assigned_uav: Optional[str] = None
    surveyed_at: Optional[float] = None
    survivor_detected: bool = False

    def __repr__(self):
        return f"PoI({self.poi_id}, {self.priority.name}, {self.status.name})"


# ─────────────────────────────────────────────
#  Data Packet
# ─────────────────────────────────────────────

@dataclass(order=True)
class DataPacket:
    priority_value: int = field(init=False)
    priority: DataPriority = field(compare=False)
    packet_id: str = field(compare=False)
    source_uav: str = field(compare=False)
    poi_id: str = field(compare=False)
    data: dict = field(compare=False, default_factory=dict)
    created_at: float = field(compare=False, default_factory=time.time)
    delivered: bool = field(compare=False, default=False)

    def __post_init__(self):
        self.priority_value = self.priority.value   # lower = higher priority


# ─────────────────────────────────────────────
#  Communication Link
# ─────────────────────────────────────────────

@dataclass
class CommLink:
    node_a: str
    node_b: str
    max_range: float = 800.0   # metres
    status: LinkStatus = LinkStatus.UP
    signal_strength: float = 1.0   # 0.0 – 1.0
    packet_loss: float = 0.0       # 0.0 – 1.0
    rssi_dbm: float = -50.0        # dBm
    snr_db: float = 25.0           # dB
    throughput_mbps: float = 24.0  # Mbps

    def update_from_distance(self, distance: float):
        """Update link quality and RF physics metrics based on distance between nodes."""
        ratio = distance / self.max_range
        if ratio >= 1.0:
            self.status = LinkStatus.DOWN
            self.signal_strength = 0.0
            self.packet_loss = 1.0
        elif ratio >= 0.75:
            self.status = LinkStatus.WEAK
            self.signal_strength = 1.0 - ratio
            self.packet_loss = (ratio - 0.75) * 2.0
        else:
            self.status = LinkStatus.UP
            self.signal_strength = 1.0 - ratio * 0.5
            self.packet_loss = 0.0

        d = max(1.0, distance)
        pl = 40.0 + 26.0 * math.log10(d)
        self.rssi_dbm = round(20.0 - pl, 1)
        self.snr_db = round(self.rssi_dbm - (-95.0), 1)
        if self.status == LinkStatus.DOWN:
            self.throughput_mbps = 0.0
        else:
            snr_lin = max(0.01, 10.0 ** (self.snr_db / 10.0))
            self.throughput_mbps = round(min(54.0, max(0.0, 15.0 * math.log2(1.0 + snr_lin) * 0.15)), 1)


# ─────────────────────────────────────────────
#  UAV
# ─────────────────────────────────────────────

@dataclass
class UAV:
    uav_id: str
    position: Position
    role: UAVRole = UAVRole.IDLE
    status: UAVStatus = UAVStatus.ACTIVE
    battery: float = 100.0          # percentage
    battery_drain_rate: float = 0.5 # % per simulation tick at cruise
    hover_drain_rate: float = 0.2   # % per tick when hovering as relay
    speed: float = 10.0             # m/s
    max_range: float = 800.0        # comms range in metres
    rtl_battery_threshold: float = 20.0   # % — trigger RTL
    waypoints: List[Position] = field(default_factory=list)
    current_waypoint_idx: int = 0
    assigned_poi: Optional[str] = None
    relay_slot: Optional[int] = None
    telemetry_log: List[dict] = field(default_factory=list)
    roll: float = 0.0      # degrees tilt
    pitch: float = 0.0     # degrees tilt
    yaw: float = 0.0       # degrees heading
    speed_mps: float = 0.0 # m/s
    rpms: List[int] = field(default_factory=lambda: [4200, 4200, 4200, 4200])
    prev_position: Optional[Position] = None

    @property
    def is_low_battery(self) -> bool:
        return self.battery <= self.rtl_battery_threshold

    @property
    def is_failed(self) -> bool:
        return self.status == UAVStatus.FAILED

    def update_dynamics(self, dt: float = 1.0):
        """Update 6-DOF attitude and rotor RPMs based on displacement."""
        if self.prev_position is not None:
            vx = (self.position.x - self.prev_position.x) / max(dt, 1e-4)
            vy = (self.position.y - self.prev_position.y) / max(dt, 1e-4)
            vz = (self.position.z - self.prev_position.z) / max(dt, 1e-4)
            self.speed_mps = round(math.sqrt(vx * vx + vy * vy + vz * vz), 2)
            if self.speed_mps > 0.05:
                self.yaw = round(math.degrees(math.atan2(vy, vx)), 1)
            # Tilt into movement
            self.pitch = round(max(-25.0, min(25.0, vx * 1.5)), 1)
            self.roll = round(max(-25.0, min(25.0, -vy * 1.5)), 1)
            # Rotor RPMs
            base_rpm = 4200 if self.role in (UAVRole.SCOUT, UAVRole.RELAY) else (0 if self.status == UAVStatus.FAILED else 1200)
            diff = int(self.speed_mps * 40.0)
            self.rpms = [base_rpm + diff, base_rpm - diff, base_rpm - diff, base_rpm + diff]
        self.prev_position = Position(self.position.x, self.position.y, self.position.z)

    def drain_battery(self, ticks: int = 1):
        drain = self.hover_drain_rate if self.role == UAVRole.RELAY else self.battery_drain_rate
        self.battery = max(0.0, self.battery - drain * ticks)
        if self.battery <= 0:
            self.status = UAVStatus.FAILED

    def log_telemetry(self, tick: int):
        self.telemetry_log.append({
            "tick": tick,
            "pos": (round(self.position.x, 2), round(self.position.y, 2), round(self.position.z, 2)),
            "attitude": (self.roll, self.pitch, self.yaw),
            "speed": self.speed_mps,
            "rpms": self.rpms,
            "battery": round(self.battery, 2),
            "role": self.role.value,
            "status": self.status.value,
        })

    def __repr__(self):
        return (f"UAV({self.uav_id}, {self.role.name}, "
                f"bat={self.battery:.1f}%, pos={self.position})")


# ─────────────────────────────────────────────
#  Ground Control Station
# ─────────────────────────────────────────────

@dataclass
class GroundControlStation:
    position: Position
    received_packets: List[DataPacket] = field(default_factory=list)
    acceptance_log: List[dict] = field(default_factory=list)

    def receive_packet(self, packet: DataPacket, tick: int):
        packet.delivered = True
        self.received_packets.append(packet)
        self.acceptance_log.append({
            "tick": tick,
            "packet_id": packet.packet_id,
            "poi_id": packet.poi_id,
            "priority": packet.priority.name,
            "source": packet.source_uav,
            "latency_ticks": tick - int(packet.created_at),
        })

    def __repr__(self):
        return f"GCS(pos={self.position}, packets_received={len(self.received_packets)})"
