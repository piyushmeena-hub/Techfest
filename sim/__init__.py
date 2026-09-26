"""
sim: Pure-Python 3D Resilient Multi-Hop Aerial UAV Swarm Simulation Subsystem.

Exports public classes, configurations, data models, and geometry objects.
"""

from sim.core import SimulationConfig, SwarmSimulationCore
from sim.drone import Drone
from sim.environment import AltitudeCorridor, DisasterEnvironment, EnvironmentConfig
from sim.obstacles import (
    Obstacle,
    ObstacleAABB,
    ObstacleManager,
    RayIntersectionResult,
    create_default_disaster_obstacles,
)
from sim.types import (
    AIRSPACE_CORRIDORS,
    BatteryModel,
    DroneLimits,
    DroneRole,
    DroneState,
    FlightMode,
    PoIPriority,
    TelemetrySnapshot,
)

from sim.network import (
    DTNBuffer,
    FANETNetworkEngine,
    NetworkPacket,
    RFChannelModel,
)
from sim.mission import DisasterMissionManager

__version__ = "0.1.0"

__all__ = [
    "SwarmSimulationCore",
    "SimulationConfig",
    "Drone",
    "DisasterEnvironment",
    "EnvironmentConfig",
    "Obstacle",
    "ObstacleAABB",
    "ObstacleManager",
    "RayIntersectionResult",
    "create_default_disaster_obstacles",
    "DroneState",
    "DroneLimits",
    "BatteryModel",
    "DroneRole",
    "FlightMode",
    "PoIPriority",
    "AltitudeCorridor",
    "AIRSPACE_CORRIDORS",
    "TelemetrySnapshot",
    "FANETNetworkEngine",
    "RFChannelModel",
    "DTNBuffer",
    "NetworkPacket",
    "DisasterMissionManager",
]
