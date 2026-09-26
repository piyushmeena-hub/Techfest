"""
tests/unit/test_network.py: Unit tests for FANET Network Engine & RF Channel.
"""

import numpy as np
import pytest

from sim.network import (
    DTNBuffer,
    FANETNetworkEngine,
    NetworkPacket,
    RFChannelModel,
)
from sim.obstacles import ObstacleAABB


def test_channel_friis_and_path_loss():
    channel = RFChannelModel()
    # At 1.0 m, PL should be approx 40.05 dB
    pl_1m = channel.compute_path_loss(1.0, is_los=True)
    assert abs(pl_1m - 40.05) < 0.5

    # At 100 m LoS vs NLoS
    pl_los = channel.compute_path_loss(100.0, is_los=True)
    pl_nlos = channel.compute_path_loss(100.0, is_los=False)
    assert pl_nlos > pl_los + 20.0  # NLoS has higher exponent + 22dB building penetration


def test_dtn_buffer_capacity_and_fifo():
    buf = DTNBuffer(capacity=3)
    p1 = NetworkPacket("P1", "U1", "GCS")
    p2 = NetworkPacket("P2", "U1", "GCS")
    p3 = NetworkPacket("P3", "U1", "GCS")
    p4 = NetworkPacket("P4", "U1", "GCS")

    assert buf.push(p1) is True
    assert buf.push(p2) is True
    assert buf.push(p3) is True
    # 4th packet overflows capacity 3: drops P1
    assert buf.push(p4) is False
    assert buf.total_dropped == 1
    assert buf.size() == 3

    popped = buf.pop()
    assert popped is not None and popped.packet_id == "P2"


def test_multi_hop_dijkstra_routing_over_obstacle():
    """
    Scenario:
    - GCS at (0, 0, 0)
    - Building obstacle at (20, -20, 0) to (80, 20, 50) blocking direct LoS
    - UAV_Survey at (100, 0, 30) (behind building)
    - UAV_Relay at (50, 60, 75) (elevated clear of building)
    
    Direct link GCS <-> UAV_Survey is occluded by building.
    Path GCS <-> UAV_Relay and UAV_Relay <-> UAV_Survey are clear LoS.
    Dijkstra must route UAV_Survey -> UAV_Relay -> GCS (multi-hop).
    """
    net = FANETNetworkEngine()
    building = ObstacleAABB("BLD1", "HighRise", np.array([20.0, -20.0, 0.0]), np.array([80.0, 20.0, 50.0]))
    
    positions = {
        "GCS": np.array([0.0, 0.0, 0.0]),
        "UAV_Relay": np.array([50.0, 60.0, 75.0]),
        "UAV_Survey": np.array([100.0, 0.0, 30.0]),
    }
    net.update_topology(positions, [building])

    routes = net.active_routes
    assert "UAV_Survey" in routes
    path = routes["UAV_Survey"]
    # Path must be multi-hop: UAV_Survey -> UAV_Relay -> GCS
    assert path == ["UAV_Survey", "UAV_Relay", "GCS"], f"Expected 2-hop route, got {path}"
