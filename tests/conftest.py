"""
Shared pytest fixtures and contract reference adapters for 3D UAV Swarm & Communication Network.
Strictly implements interface contracts defined in PROJECT.md and TEST_INFRA.md.
"""

from __future__ import annotations
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
import numpy as np
import pytest


# ============================================================================
# 1. Interface Dataclasses (PROJECT.md Interface Contracts)
# ============================================================================

@dataclass
class DroneState:
    """Quadcopter dynamic state contract."""
    id: str
    role: str  # 'SURVEY' | 'RELAY'
    position: np.ndarray  # shape (3,), [x, y, z] in meters
    velocity: np.ndarray  # shape (3,), [vx, vy, vz] in m/s
    attitude: np.ndarray  # shape (3,), [roll, pitch, yaw] in radians
    rotor_speeds: np.ndarray  # shape (4,), rad/s
    battery_soc: float  # [0.0, 1.0]
    flight_mode: str  # 'IDLE', 'TAKEOFF', 'TRANSIT', 'SURVEYING', 'RELAY', 'DATA_TX', 'RTL', 'LANDED', 'COMPLETED'
    assigned_poi_id: Optional[str] = None
    target_position: Optional[np.ndarray] = None


@dataclass
class AABB:
    """Axis-Aligned Bounding Box obstacle in disaster zone."""
    id: str
    min_pt: np.ndarray  # shape (3,), [x_min, y_min, z_min]
    max_pt: np.ndarray  # shape (3,), [x_max, y_max, z_max]
    height: float = 35.0

    def contains_point(self, pt: np.ndarray) -> bool:
        """Check if point is inside AABB."""
        return bool(np.all(pt >= self.min_pt - 1e-5) and np.all(pt <= self.max_pt + 1e-5))

    def ray_intersection(self, origin: np.ndarray, direction: np.ndarray, max_dist: float) -> bool:
        """
        Slab method for Ray-AABB intersection test.
        Origin: start point of ray (e.g. UAV position)
        Direction: unit direction vector
        max_dist: distance between origin and destination
        """
        dir_norm = np.linalg.norm(direction)
        if dir_norm < 1e-9:
            return False
        d = direction / dir_norm

        t_min = 0.0
        t_max = max_dist

        for i in range(3):
            if abs(d[i]) < 1e-9:
                if origin[i] < self.min_pt[i] or origin[i] > self.max_pt[i]:
                    return False
            else:
                inv_d = 1.0 / d[i]
                t1 = (self.min_pt[i] - origin[i]) * inv_d
                t2 = (self.max_pt[i] - origin[i]) * inv_d
                if t1 > t2:
                    t1, t2 = t2, t1
                t_min = max(t_min, t1)
                t_max = min(t_max, t2)
                if t_min > t_max:
                    return False

        return (t_max >= 0.0) and (t_min <= max_dist)


@dataclass
class NetworkPacket:
    """Packet contract with hop trace logging."""
    packet_id: str
    source_id: str
    destination_id: str
    payload_type: str  # 'TELEMETRY' | 'SURVEY_DATA' | 'HEARTBEAT'
    data_size_bytes: int = 1024
    timestamp_sent: float = 0.0
    timestamp_received: Optional[float] = None
    hop_trace: List[str] = field(default_factory=list)
    status: str = 'QUEUED'  # 'QUEUED', 'IN_FLIGHT', 'DELIVERED', 'DROPPED'


@dataclass
class PoI:
    """Disaster Point of Interest."""
    id: str
    position: np.ndarray  # shape (3,), [x, y, z]
    priority: str  # 'HIGH' | 'MEDIUM' | 'LOW'
    dwell_time_required: float = 5.0  # seconds
    dwell_time_accumulated: float = 0.0
    status: str = 'PENDING'  # 'PENDING' | 'ASSIGNED' | 'IN_PROGRESS' | 'COMPLETED'
    assigned_drone_id: Optional[str] = None
    data_payload_size: int = 2048


@dataclass
class TelemetrySnapshot:
    """Compact telemetry frame (< 1.5 KB @ 30 Hz)."""
    sim_time: float
    drones: List[Dict[str, Any]]
    gcs: Dict[str, Any]
    pois: List[Dict[str, Any]]
    active_routes: List[List[str]]  # e.g. [['UAV_3', 'UAV_1', 'GCS']]
    links: List[Dict[str, Any]]     # [{'source': 'UAV_3', 'target': 'UAV_1', 'snr': 18.2, 'status': 'ACTIVE'}]
    packets: List[Dict[str, Any]]   # in-flight packet traces
    metrics: Dict[str, float]       # {'pdr': 0.98, 'avg_latency_ms': 14.2, 'completed_pois': 3}


# ============================================================================
# 2. Reference Physics, RF, Routing & Mission Engines
# ============================================================================

class ReferenceChannelModel:
    """2.4 GHz Friis FSPL + Log-distance path loss + Building penetration model."""
    def __init__(self, freq_hz: float = 2.4e9, p_tx_dbm: float = 20.0, noise_floor_dbm: float = -95.0):
        self.freq_hz = freq_hz
        self.p_tx_dbm = p_tx_dbm
        self.noise_floor_dbm = noise_floor_dbm
        self.c = 3.0e8
        self.wavelength = self.c / self.freq_hz
        # PL0 = 20*log10(4*pi / lambda) approx 40.05 dB at d0=1m
        self.pl0 = 20.0 * math.log10(4.0 * math.pi / self.wavelength)
        self.eta_los = 2.05
        self.eta_nlos = 3.60
        self.building_penetration_loss_db = 22.0
        self.max_direct_los_range = 320.0  # meters
        self.min_snr_threshold_db = 0.0

    def compute_path_loss(self, distance: float, is_los: bool = True, num_occlusions: int = 0) -> float:
        d = max(1.0, float(distance))
        eta = self.eta_los if is_los else self.eta_nlos
        pl = self.pl0 + 10.0 * eta * math.log10(d)
        if not is_los or num_occlusions > 0:
            pl += max(1, num_occlusions) * self.building_penetration_loss_db
        return float(pl)

    def compute_snr(self, path_loss_db: float) -> float:
        p_rx_dbm = self.p_tx_dbm - path_loss_db
        return float(p_rx_dbm - self.noise_floor_dbm)

    def is_link_viable(self, distance: float, is_los: bool = True, num_occlusions: int = 0) -> Tuple[bool, float, float]:
        pl = self.compute_path_loss(distance, is_los, num_occlusions)
        snr = self.compute_snr(pl)
        viable = (snr >= self.min_snr_threshold_db) and (distance <= (self.max_direct_los_range if is_los else 160.0))
        return viable, pl, snr


class ReferenceOcclusionEngine:
    """3D Ray-AABB building occlusion detection."""
    def __init__(self, obstacles: Optional[List[AABB]] = None):
        self.obstacles: List[AABB] = obstacles or []

    def check_occlusion(self, p1: np.ndarray, p2: np.ndarray) -> Tuple[bool, int]:
        diff = p2 - p1
        dist = float(np.linalg.norm(diff))
        if dist < 1e-6:
            return False, 0
        direction = diff / dist

        occlusion_count = 0
        for obs in self.obstacles:
            if obs.ray_intersection(p1, direction, dist):
                occlusion_count += 1

        return (occlusion_count > 0), occlusion_count


class ReferenceDTNBuffer:
    """DTN Store-and-Forward ring buffer (capacity 250 packets)."""
    def __init__(self, capacity: int = 250):
        self.capacity = capacity
        self.buffer: List[NetworkPacket] = []
        self.total_dropped: int = 0

    def push(self, packet: NetworkPacket) -> bool:
        if len(self.buffer) >= self.capacity:
            # Drop oldest (FIFO overflow)
            self.buffer.pop(0)
            self.total_dropped += 1
            self.buffer.append(packet)
            return False
        self.buffer.append(packet)
        return True

    def pop(self) -> Optional[NetworkPacket]:
        if not self.buffer:
            return None
        return self.buffer.pop(0)

    def size(self) -> int:
        return len(self.buffer)

    def is_empty(self) -> bool:
        return len(self.buffer) == 0


class ReferenceNetworkEngine:
    """Dynamic Link-State Routing (FANET-DLS) with composite Dijkstra cost."""
    def __init__(self, channel_model: Optional[ReferenceChannelModel] = None, occlusion_engine: Optional[ReferenceOcclusionEngine] = None):
        self.channel = channel_model or ReferenceChannelModel()
        self.occlusion = occlusion_engine or ReferenceOcclusionEngine()
        self.node_positions: Dict[str, np.ndarray] = {}
        self.link_cache: Dict[Tuple[str, str], Dict[str, Any]] = {}
        self.dtn_buffers: Dict[str, ReferenceDTNBuffer] = {}
        self.packets_transmitted: int = 0
        self.packets_delivered: int = 0
        self.latencies_ms: List[float] = []
        self.hop_counts: List[int] = []

    def update_topology(self, node_positions: Dict[str, np.ndarray], obstacles: Optional[List[AABB]] = None) -> None:
        self.node_positions = {k: np.array(v, dtype=float) for k, v in node_positions.items()}
        if obstacles is not None:
            self.occlusion.obstacles = obstacles
        self.link_cache.clear()

        nodes = list(self.node_positions.keys())
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                n1, n2 = nodes[i], nodes[j]
                p1, p2 = self.node_positions[n1], self.node_positions[n2]
                dist = float(np.linalg.norm(p1 - p2))
                is_occ, occ_cnt = self.occlusion.check_occlusion(p1, p2)
                viable, pl, snr = self.channel.is_link_viable(dist, not is_occ, occ_cnt)

                cost = dist + max(0.0, 30.0 - snr) * 2.0 + (occ_cnt * 500.0)
                link_info = {
                    'distance': dist,
                    'is_los': not is_occ,
                    'occlusion_count': occ_cnt,
                    'path_loss_db': pl,
                    'snr': snr,
                    'viable': viable,
                    'cost': cost
                }
                self.link_cache[(n1, n2)] = link_info
                self.link_cache[(n2, n1)] = link_info

    def get_link_info(self, n1: str, n2: str) -> Optional[Dict[str, Any]]:
        return self.link_cache.get((n1, n2))

    def find_route(self, src_id: str, dst_id: str) -> List[str]:
        if src_id == dst_id:
            return [src_id]
        if src_id not in self.node_positions or dst_id not in self.node_positions:
            return []

        # Dijkstra algorithm
        nodes = list(self.node_positions.keys())
        distances = {n: float('inf') for n in nodes}
        previous = {n: None for n in nodes}
        distances[src_id] = 0.0
        unvisited = set(nodes)

        while unvisited:
            current = min(unvisited, key=lambda n: distances[n])
            if distances[current] == float('inf'):
                break
            if current == dst_id:
                break
            unvisited.remove(current)

            for neighbor in nodes:
                if neighbor in unvisited:
                    link = self.link_cache.get((current, neighbor))
                    if link and link['viable']:
                        alt = distances[current] + link['cost']
                        if alt < distances[neighbor]:
                            distances[neighbor] = alt
                            previous[neighbor] = current

        # Reconstruct path
        path = []
        curr = dst_id
        while curr is not None:
            path.append(curr)
            curr = previous[curr]
        path.reverse()

        return path if path and path[0] == src_id else []

    def transmit_packet(self, packet: NetworkPacket, current_sim_time: float = 0.0) -> bool:
        self.packets_transmitted += 1
        route = self.find_route(packet.source_id, packet.destination_id)

        if not route:
            # Route broken -> DTN store
            if packet.source_id not in self.dtn_buffers:
                self.dtn_buffers[packet.source_id] = ReferenceDTNBuffer()
            packet.status = 'QUEUED'
            self.dtn_buffers[packet.source_id].push(packet)
            return False

        # Hop through route
        packet.hop_trace = list(route)
        hop_count = len(route) - 1
        # Approx 5ms per hop
        latency_ms = hop_count * 5.0 + float(np.random.uniform(0.5, 2.0))
        packet.timestamp_received = current_sim_time + (latency_ms / 1000.0)
        packet.status = 'DELIVERED'

        self.packets_delivered += 1
        self.latencies_ms.append(latency_ms)
        self.hop_counts.append(hop_count)
        return True

    def get_metrics(self) -> Dict[str, float]:
        pdr = (self.packets_delivered / max(1, self.packets_transmitted)) if self.packets_transmitted > 0 else 1.0
        avg_lat = float(np.mean(self.latencies_ms)) if self.latencies_ms else 0.0
        return {
            'pdr': float(pdr),
            'avg_latency_ms': float(avg_lat),
            'total_transmitted': float(self.packets_transmitted),
            'total_delivered': float(self.packets_delivered)
        }


class ReferenceKinematicsEngine:
    """6-DOF Quadcopter Kinematics, 4-tier altitude corridors, APF and flocking."""
    ALTITUDE_CORRIDORS = {
        'LAUNCH': (0.0, 20.0),
        'SURVEY': (25.0, 45.0),
        'TRANSIT': (50.0, 65.0),
        'RELAY': (70.0, 90.0)
    }

    def __init__(self, v_max: float = 15.0, a_max: float = 5.0, d_safe: float = 3.0):
        self.v_max = v_max
        self.a_max = a_max
        self.d_safe = d_safe
        self.k_rep = 8.0
        self.k_downwash = 12.0

    def compute_acceleration(self, drone: DroneState, all_drones: List[DroneState], obstacles: List[AABB]) -> np.ndarray:
        if drone.target_position is not None:
            target_vec = drone.target_position - drone.position
            dist = np.linalg.norm(target_vec)
            if dist > 1e-4:
                desired_vel = (target_vec / dist) * min(self.v_max, dist * 1.5)
            else:
                desired_vel = np.zeros(3)
            a_att = (desired_vel - drone.velocity) * 1.5
        else:
            # Hover damping
            a_att = -drone.velocity * 1.5

        # Drone-Drone APF Repulsion
        a_rep = np.zeros(3)
        for other in all_drones:
            if other.id == drone.id:
                continue
            diff = drone.position - other.position
            d = np.linalg.norm(diff)
            if 1e-4 < d < self.d_safe:
                rep_mag = self.k_rep * 6.0 * (1.0 / d - 1.0 / self.d_safe) / (d ** 2)
                rep_dir = diff / d
                # Add lateral avoidance (cross with vertical unit vector) to avoid head-on deadlocks
                lateral = np.cross(rep_dir, np.array([0.0, 0.0, 1.0]))
                lat_norm = np.linalg.norm(lateral)
                if lat_norm > 1e-4:
                    lateral = lateral / lat_norm
                else:
                    lateral = np.array([0.0, 1.0, 0.0])
                a_rep += rep_dir * rep_mag + lateral * (rep_mag * 1.5)

            # Downwash repulsion if stacked vertically within 2.5m horizontally
            horiz_dist = np.linalg.norm(diff[:2])
            if horiz_dist < 2.5 and -5.0 < diff[2] < 0.0:
                # Drone is right below other drone: push horizontally away
                horiz_dir = diff[:2] / max(1e-4, horiz_dist) if horiz_dist > 1e-4 else np.array([1.0, 0.0])
                a_rep[:2] += horiz_dir * self.k_downwash

        # Obstacle APF Repulsion
        for obs in obstacles:
            # Distance to AABB
            clamped = np.clip(drone.position, obs.min_pt, obs.max_pt)
            diff = drone.position - clamped
            d = np.linalg.norm(diff)
            if 1e-4 < d < 20.0:
                rep_dir = diff / d
                closing_speed = -np.dot(drone.velocity, rep_dir)
                brake = rep_dir * (closing_speed * 3.0) if closing_speed > 0 else np.zeros(3)
                rep_mag = self.k_rep * 15.0 * (1.0 / d - 1.0 / 20.0) / d
                lat = np.cross(rep_dir, np.array([0.0, 0.0, 1.0]))
                lat_norm = np.linalg.norm(lat)
                lat_dir = lat / lat_norm if lat_norm > 1e-4 else np.array([0.0, 1.0, 0.0])
                a_rep += rep_dir * rep_mag + brake + lat_dir * (rep_mag * 1.5)

        total_a = a_att + a_rep
        # Acceleration clamp
        a_mag = np.linalg.norm(total_a)
        if a_mag > self.a_max:
            total_a = (total_a / a_mag) * self.a_max

        return total_a

    def step_drone(self, drone: DroneState, dt: float, all_drones: List[DroneState], obstacles: List[AABB]) -> None:
        a = self.compute_acceleration(drone, all_drones, obstacles)
        drone.velocity += a * dt

        # Velocity clamp
        v_mag = np.linalg.norm(drone.velocity)
        if v_mag > self.v_max:
            drone.velocity = (drone.velocity / v_mag) * self.v_max

        # Position update
        drone.position += drone.velocity * dt

        # Boundary containment (500x500m zone, 0-100m height)
        drone.position[0] = np.clip(drone.position[0], 0.0, 500.0)
        drone.position[1] = np.clip(drone.position[1], 0.0, 500.0)
        drone.position[2] = np.clip(drone.position[2], 0.0, 100.0)

        # Battery depletion: base 50W + prop power
        power = 50.0 + 15.0 * (v_mag ** 1.5)
        energy_used = power * dt
        # Assuming 300,000 Joule pack (~83 Wh)
        drone.battery_soc = max(0.0, drone.battery_soc - (energy_used / 300000.0))


class ReferenceVSMEngine:
    """Virtual Spring Mesh (VSM) relay positioning engine."""
    def compute_relay_setpoints(
        self,
        gcs_pos: np.ndarray,
        survey_drones: List[DroneState],
        relay_drones: List[DroneState],
        obstacles: List[AABB]
    ) -> Dict[str, np.ndarray]:
        setpoints: Dict[str, np.ndarray] = {}
        if not relay_drones:
            return setpoints

        if not survey_drones:
            # Default to midpoint loitering in relay corridor
            for r in relay_drones:
                setpoints[r.id] = np.array([250.0, 250.0, 75.0])
            return setpoints

        # Calculate geometric center of survey fleet
        survey_center = np.mean([s.position for s in survey_drones], axis=0)

        num_relays = len(relay_drones)
        for idx, relay in enumerate(relay_drones):
            # Line interpolation between GCS and Survey Center
            alpha = (idx + 1) / (num_relays + 1)
            target_xy = gcs_pos[:2] * (1.0 - alpha) + survey_center[:2] * alpha
            # Relay corridor altitude [70, 90]m to avoid 35m buildings
            target_z = 75.0 + (idx * 5.0)
            setpoints[relay.id] = np.array([target_xy[0], target_xy[1], target_z])

        return setpoints


class ReferenceMissionCoordinator:
    """Disaster survey mission and PoI task allocation."""
    def __init__(self, pois: Optional[List[PoI]] = None):
        self.pois: List[PoI] = pois or []
        self.completed_pois: int = 0

    def assign_tasks(self, drones: List[DroneState]) -> None:
        survey_drones = [d for d in drones if d.role == 'SURVEY' and d.flight_mode in ('IDLE', 'TRANSIT')]
        # Sort pending PoIs by priority
        priority_weights = {'HIGH': 3, 'MEDIUM': 2, 'LOW': 1}
        pending_pois = [p for p in self.pois if p.status == 'PENDING']
        pending_pois.sort(key=lambda p: priority_weights.get(p.priority, 0), reverse=True)

        for drone in survey_drones:
            if drone.assigned_poi_id is None and pending_pois:
                poi = pending_pois.pop(0)
                poi.status = 'ASSIGNED'
                poi.assigned_drone_id = drone.id
                drone.assigned_poi_id = poi.id
                drone.target_position = np.copy(poi.position)
                drone.flight_mode = 'TRANSIT'

    def update_dwell(self, drones: List[DroneState], dt: float, network: ReferenceNetworkEngine, sim_time: float) -> List[NetworkPacket]:
        generated_packets = []
        for drone in drones:
            if drone.assigned_poi_id:
                poi = next((p for p in self.pois if p.id == drone.assigned_poi_id), None)
                if poi:
                    dist = float(np.linalg.norm(drone.position - poi.position))
                    if dist <= 3.0:
                        drone.flight_mode = 'SURVEYING'
                        poi.status = 'IN_PROGRESS'
                        poi.dwell_time_accumulated += dt

                        # Generate data packet during survey
                        pkt = NetworkPacket(
                            packet_id=f"PKT_{poi.id}_{int(sim_time*10)}",
                            source_id=drone.id,
                            destination_id="GCS",
                            payload_type="SURVEY_DATA",
                            data_size_bytes=poi.data_payload_size,
                            timestamp_sent=sim_time
                        )
                        generated_packets.append(pkt)
                        network.transmit_packet(pkt, sim_time)

                        if poi.dwell_time_accumulated >= poi.dwell_time_required:
                            poi.status = 'COMPLETED'
                            drone.assigned_poi_id = None
                            drone.flight_mode = 'IDLE'
                            self.completed_pois += 1
        return generated_packets


class ReferenceSimulationController:
    """Master simulation coordinator & CLI controller for E2E tests."""
    def __init__(
        self,
        num_drones: int = 5,
        num_pois: int = 4,
        duration: float = 10.0,
        gcs_pos: Optional[np.ndarray] = None,
        headless: bool = True
    ):
        self.duration = duration
        self.headless = headless
        self.sim_time = 0.0
        self.dt = 1.0 / 30.0  # 30 Hz
        self.gcs_pos = gcs_pos if gcs_pos is not None else np.array([50.0, 50.0, 10.0])

        self.obstacles: List[AABB] = [
            AABB("OBS_CENTER", np.array([200.0, 200.0, 0.0]), np.array([260.0, 260.0, 35.0]), 35.0),
            AABB("OBS_NORTH", np.array([120.0, 350.0, 0.0]), np.array([180.0, 410.0, 30.0]), 30.0),
        ]

        # Initialize Drones: 3 Surveyors, 2 Relays
        self.drones: List[DroneState] = []
        for i in range(num_drones):
            role = 'RELAY' if i % 2 == 1 and i > 0 else 'SURVEY'
            pos = np.array([40.0 + i * 5.0, 40.0 + i * 5.0, 10.0 + i * 2.0])
            self.drones.append(DroneState(
                id=f"UAV_{i+1}",
                role=role,
                position=pos,
                velocity=np.zeros(3),
                attitude=np.zeros(3),
                rotor_speeds=np.array([400.0, 400.0, 400.0, 400.0]),
                battery_soc=1.0,
                flight_mode='IDLE'
            ))

        # Initialize PoIs
        self.pois: List[PoI] = []
        priorities = ['HIGH', 'MEDIUM', 'LOW', 'HIGH']
        for i in range(num_pois):
            self.pois.append(PoI(
                id=f"POI_{i+1}",
                position=np.array([350.0 + (i % 2) * 50.0, 350.0 + (i // 2) * 50.0, 30.0]),
                priority=priorities[i % len(priorities)],
                dwell_time_required=3.0
            ))

        self.channel = ReferenceChannelModel()
        self.occlusion = ReferenceOcclusionEngine(self.obstacles)
        self.network = ReferenceNetworkEngine(self.channel, self.occlusion)
        self.kinematics = ReferenceKinematicsEngine()
        self.vsm = ReferenceVSMEngine()
        self.mission = ReferenceMissionCoordinator(self.pois)
        self.snapshots: List[TelemetrySnapshot] = []

    def step(self) -> TelemetrySnapshot:
        # 1. Update task allocation
        self.mission.assign_tasks(self.drones)

        # 2. VSM relay setpoints
        survey_drones = [d for d in self.drones if d.role == 'SURVEY']
        relay_drones = [d for d in self.drones if d.role == 'RELAY']
        relay_targets = self.vsm.compute_relay_setpoints(self.gcs_pos, survey_drones, relay_drones, self.obstacles)
        for r in relay_drones:
            if r.id in relay_targets:
                r.target_position = relay_targets[r.id]
                r.flight_mode = 'RELAY'

        # 3. Kinematics step
        for drone in self.drones:
            self.kinematics.step_drone(drone, self.dt, self.drones, self.obstacles)

        # 4. Topology update
        node_pos = {d.id: d.position for d in self.drones}
        node_pos['GCS'] = self.gcs_pos
        self.network.update_topology(node_pos, self.obstacles)

        # 5. Dwell & Data collection
        pkts = self.mission.update_dwell(self.drones, self.dt, self.network, self.sim_time)

        # 6. Capture Telemetry Snapshot
        active_routes = []
        for d in self.drones:
            route = self.network.find_route(d.id, "GCS")
            if route:
                active_routes.append(route)

        links = []
        for (n1, n2), info in self.network.link_cache.items():
            if info['viable'] and n1 < n2:
                links.append({'source': n1, 'target': n2, 'snr': info['snr'], 'status': 'ACTIVE'})

        metrics = self.network.get_metrics()
        metrics['completed_pois'] = float(self.mission.completed_pois)

        snap = TelemetrySnapshot(
            sim_time=self.sim_time,
            drones=[{'id': d.id, 'pos': d.position.tolist(), 'role': d.role, 'mode': d.flight_mode} for d in self.drones],
            gcs={'pos': self.gcs_pos.tolist()},
            pois=[{'id': p.id, 'status': p.status, 'progress': p.dwell_time_accumulated / p.dwell_time_required if p.dwell_time_required > 0 else 1.0} for p in self.pois],
            active_routes=active_routes,
            links=links,
            packets=[{'id': p.packet_id, 'trace': p.hop_trace, 'status': p.status} for p in pkts],
            metrics=metrics
        )
        self.snapshots.append(snap)
        self.sim_time += self.dt
        return snap

    def run(self, max_seconds: Optional[float] = None) -> List[TelemetrySnapshot]:
        target = max_seconds if max_seconds is not None else self.duration
        steps = int(target / self.dt)
        for _ in range(steps):
            self.step()
        return self.snapshots


# ============================================================================
# 3. Pytest Shared Fixtures
# ============================================================================

@pytest.fixture
def channel_model() -> ReferenceChannelModel:
    return ReferenceChannelModel()


@pytest.fixture
def sample_obstacles() -> List[AABB]:
    return [
        AABB("BUILDING_1", np.array([200.0, 200.0, 0.0]), np.array([260.0, 260.0, 35.0]), 35.0),
        AABB("BUILDING_2", np.array([100.0, 350.0, 0.0]), np.array([160.0, 410.0, 40.0]), 40.0),
    ]


@pytest.fixture
def occlusion_engine(sample_obstacles: List[AABB]) -> ReferenceOcclusionEngine:
    return ReferenceOcclusionEngine(sample_obstacles)


@pytest.fixture
def network_engine(channel_model: ReferenceChannelModel, occlusion_engine: ReferenceOcclusionEngine) -> ReferenceNetworkEngine:
    return ReferenceNetworkEngine(channel_model, occlusion_engine)


@pytest.fixture
def kinematics_engine() -> ReferenceKinematicsEngine:
    return ReferenceKinematicsEngine()


@pytest.fixture
def sample_drones() -> List[DroneState]:
    return [
        DroneState("UAV_1", "SURVEY", np.array([450.0, 450.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE'),
        DroneState("UAV_2", "RELAY", np.array([250.0, 250.0, 75.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'RELAY'),
        DroneState("UAV_3", "SURVEY", np.array([100.0, 100.0, 30.0]), np.zeros(3), np.zeros(3), np.zeros(4), 1.0, 'IDLE'),
    ]


@pytest.fixture
def sample_pois() -> List[PoI]:
    return [
        PoI("POI_SURVIVOR", np.array([450.0, 450.0, 30.0]), "HIGH", 3.0),
        PoI("POI_COLLAPSE", np.array([300.0, 300.0, 25.0]), "MEDIUM", 4.0),
        PoI("POI_HAZARD", np.array([150.0, 150.0, 25.0]), "LOW", 2.0),
    ]


@pytest.fixture
def vsm_engine() -> ReferenceVSMEngine:
    return ReferenceVSMEngine()


@pytest.fixture
def mission_coordinator(sample_pois: List[PoI]) -> ReferenceMissionCoordinator:
    return ReferenceMissionCoordinator(sample_pois)


@pytest.fixture
def sim_controller() -> ReferenceSimulationController:
    return ReferenceSimulationController(num_drones=4, num_pois=3, duration=2.0)
