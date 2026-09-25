"""
UAV-X Phase 0: Communication Network Module
Simulates multi-hop NS-3-style link behavior in pure Python.
Inspired by: UAV Swarm Network Simulator + IoD_Sim
"""
from __future__ import annotations
from typing import Dict, List, Optional, Tuple
import heapq
import random
from .models import UAV, GroundControlStation, CommLink, LinkStatus, DataPacket, Position


class CommNetwork:
    """
    Models the aerial mesh communication network.
    Each node is either a UAV or the GCS.
    Links are updated each tick based on real distances.
    Multi-hop routing uses Dijkstra on the live link graph.
    """

    def __init__(self, uavs: Dict[str, UAV], gcs: GroundControlStation,
                 max_range: float = 800.0, noise_enabled: bool = True):
        self.uavs = uavs
        self.gcs = gcs
        self.max_range = max_range
        self.noise_enabled = noise_enabled
        self.links: Dict[Tuple[str, str], CommLink] = {}
        self.routing_table: Dict[str, List[str]] = {}  # node → path to GCS

    # ────────────────────────────────────────────
    #  Link Management
    # ────────────────────────────────────────────

    def _node_position(self, node_id: str) -> Optional[Position]:
        if node_id == "GCS":
            return self.gcs.position
        uav = self.uavs.get(node_id)
        return uav.position if uav else None

    def _link_key(self, a: str, b: str) -> Tuple[str, str]:
        return (min(a, b), max(a, b))

    def update_links(self):
        """Recalculate all links based on current positions."""
        all_nodes = list(self.uavs.keys()) + ["GCS"]
        self.links.clear()

        for i, a in enumerate(all_nodes):
            for b in all_nodes[i + 1:]:
                pos_a = self._node_position(a)
                pos_b = self._node_position(b)
                if pos_a is None or pos_b is None:
                    continue

                dist = pos_a.distance_to(pos_b)
                if dist <= self.max_range * 1.1:   # include weak links
                    link = CommLink(node_a=a, node_b=b, max_range=self.max_range)
                    link.update_from_distance(dist)

                    # Add environmental noise
                    if self.noise_enabled and link.status != LinkStatus.DOWN:
                        link.packet_loss = min(1.0, link.packet_loss + random.uniform(0, 0.05))

                    key = self._link_key(a, b)
                    self.links[key] = link

    # ────────────────────────────────────────────
    #  Multi-Hop Routing (Dijkstra)
    # ────────────────────────────────────────────

    def _build_graph(self) -> Dict[str, Dict[str, float]]:
        """Build adjacency graph weighted by packet loss (lower = better)."""
        graph: Dict[str, Dict[str, float]] = {}
        for (a, b), link in self.links.items():
            if link.status == LinkStatus.DOWN:
                continue
            cost = 1.0 + link.packet_loss * 10.0   # penalise lossy links
            graph.setdefault(a, {})[b] = cost
            graph.setdefault(b, {})[a] = cost
        return graph

    def compute_routing_table(self):
        """Run Dijkstra from GCS to find best path for all nodes."""
        graph = self._build_graph()
        dist: Dict[str, float] = {"GCS": 0.0}
        prev: Dict[str, Optional[str]] = {"GCS": None}
        heap = [(0.0, "GCS")]

        while heap:
            cost, node = heapq.heappop(heap)
            if cost > dist.get(node, float("inf")):
                continue
            for neighbor, edge_cost in graph.get(node, {}).items():
                new_cost = cost + edge_cost
                if new_cost < dist.get(neighbor, float("inf")):
                    dist[neighbor] = new_cost
                    prev[neighbor] = node
                    heapq.heappush(heap, (new_cost, neighbor))

        # Build routing table: each node → ordered hop list to GCS
        self.routing_table = {}
        for node in self.uavs:
            if node not in prev:
                self.routing_table[node] = []   # unreachable
                continue
            path = []
            cur = node
            while cur is not None:
                path.append(cur)
                cur = prev.get(cur)
            self.routing_table[node] = path   # [node, hop1, hop2, ..., GCS]

    # ────────────────────────────────────────────
    #  Packet Delivery
    # ────────────────────────────────────────────

    def deliver_packet(self, packet: DataPacket, tick: int) -> Tuple[bool, int]:
        """
        Attempt to deliver a packet from its source UAV to the GCS.
        Returns (delivered: bool, hops: int).
        Simulates packet loss at each hop.
        """
        source = packet.source_uav
        path = self.routing_table.get(source, [])

        if not path or path[-1] != "GCS":
            return False, 0   # No route to GCS

        hops = len(path) - 1
        # Simulate packet loss along the path
        for i in range(len(path) - 1):
            key = self._link_key(path[i], path[i + 1])
            link = self.links.get(key)
            if link is None or link.status == LinkStatus.DOWN:
                return False, hops
            if random.random() < link.packet_loss:
                return False, hops   # Dropped at this hop

        self.gcs.receive_packet(packet, tick)
        return True, hops

    # ────────────────────────────────────────────
    #  Diagnostics
    # ────────────────────────────────────────────

    def get_connectivity_report(self) -> dict:
        """Return a snapshot of network connectivity."""
        reachable = [n for n, path in self.routing_table.items() if path and path[-1] == "GCS"]
        isolated = [n for n, path in self.routing_table.items() if not path or path[-1] != "GCS"]
        return {
            "total_nodes": len(self.uavs),
            "reachable_to_gcs": len(reachable),
            "isolated_nodes": isolated,
            "active_links": sum(1 for l in self.links.values() if l.status == LinkStatus.UP),
            "weak_links": sum(1 for l in self.links.values() if l.status == LinkStatus.WEAK),
            "down_links": sum(1 for l in self.links.values() if l.status == LinkStatus.DOWN),
        }
