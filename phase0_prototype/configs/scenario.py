"""
UAV-X Phase 0: Scenario Configuration
Defines the disaster environment, UAV fleet, and PoI layout.
"""
from __future__ import annotations
from typing import Dict, List
from core.models import UAV, PointOfInterest, GroundControlStation, Position, DataPriority


def create_disaster_scenario() -> tuple:
    """
    Creates a realistic post-earthquake scenario:
    - GCS stationed outside affected zone (origin)
    - 10 UAVs deployed
    - 7 PoIs spread across disaster zone (0-2km radius)
    """

    # ── GCS ──────────────────────────────────────────────
    gcs = GroundControlStation(position=Position(x=0, y=0, z=0))

    # ── UAVs ─────────────────────────────────────────────
    # All start at GCS, battery full, idle
    uav_configs = [
        ("UAV-01", Position(0, 0, 0), 100.0),
        ("UAV-02", Position(0, 0, 0), 100.0),
        ("UAV-03", Position(0, 0, 0), 100.0),
        ("UAV-04", Position(0, 0, 0), 95.0),    # slightly used
        ("UAV-05", Position(0, 0, 0), 100.0),
        ("UAV-06", Position(0, 0, 0), 100.0),
        ("UAV-07", Position(0, 0, 0), 100.0),
        ("UAV-08", Position(0, 0, 0), 100.0),
        ("UAV-09", Position(0, 0, 0), 40.0),    # lower battery — will RTL early
        ("UAV-10", Position(0, 0, 0), 100.0),
    ]
    uavs: Dict[str, UAV] = {}
    for uid, pos, bat in uav_configs:
        uavs[uid] = UAV(
            uav_id=uid,
            position=pos,
            battery=bat,
            battery_drain_rate=0.2,   # slower drain — realistic for 300 tick mission
            hover_drain_rate=0.08,    # relay hover is more efficient
            speed=12.0,
            max_range=800.0,
            rtl_battery_threshold=25.0,  # trigger RTL earlier for safe return
        )

    # ── Points of Interest ────────────────────────────────
    # Spread across the disaster zone (up to 2km from GCS)
    pois: List[PointOfInterest] = [
        PointOfInterest(
            poi_id="POI-A",
            position=Position(x=500,  y=300,  z=0),
            priority=DataPriority.CRITICAL,    # school building — possible survivors
        ),
        PointOfInterest(
            poi_id="POI-B",
            position=Position(x=900,  y=100,  z=0),
            priority=DataPriority.HIGH,        # hospital wing — structural risk
        ),
        PointOfInterest(
            poi_id="POI-C",
            position=Position(x=1200, y=600,  z=0),
            priority=DataPriority.CRITICAL,    # residential block — trapped civilians
        ),
        PointOfInterest(
            poi_id="POI-D",
            position=Position(x=700,  y=900,  z=0),
            priority=DataPriority.MEDIUM,      # road bridge — infrastructure assessment
        ),
        PointOfInterest(
            poi_id="POI-E",
            position=Position(x=1500, y=200,  z=0),
            priority=DataPriority.HIGH,        # industrial zone — hazmat risk
        ),
        PointOfInterest(
            poi_id="POI-F",
            position=Position(x=1800, y=800,  z=0),
            priority=DataPriority.LOW,         # open field — staging area scouting
        ),
        PointOfInterest(
            poi_id="POI-G",
            position=Position(x=300,  y=1100, z=0),
            priority=DataPriority.MEDIUM,      # community center — relief hub assessment
        ),
    ]

    return gcs, uavs, pois
