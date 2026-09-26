# Project: 3D Resilient Multi-Hop Aerial UAV Communication Network Simulation

## Architecture
A decoupled, high-performance simulation architecture for simulating and visualizing a resilient Flying Ad-Hoc Network (FANET) formed by autonomous UAVs conducting post-disaster survey and data relay to a Ground Control Station (GCS).

```
+-----------------------------------------------------------------------------------+
|                                 SIMULATION CORE (sim/)                            |
|                                                                                   |
|  +-------------------------+   +-------------------------+   +------------------+ |
|  |   UAV Fleet Kinematics  |   |   Disaster Environment  |   |   Mission FSM &  | |
|  | 6-DOF Newton-Euler, APF |   | 500x500m zone, AABB obs |   | PoI Survey Logic | |
|  | Flocking, 4-tier layer  |   | multi-priority PoIs     |   | VSM Relay Mesh   | |
|  +-------------------------+   +-------------------------+   +------------------+ |
|               |                             |                          |          |
|               +-----------------------------+--------------------------+          |
|                                             |                                     |
|                                             v                                     |
|                                +-------------------------+                        |
|                                |   FANET Comm & Routing  |                        |
|                                | Friis / Log-distance PL |                        |
|                                | 3D Ray-AABB Occlusion   |                        |
|                                | Dynamic Link-State DLS  |                        |
|                                | DTN Store-and-Forward   |                        |
|                                +-------------------------+                        |
+---------------------------------------------|-------------------------------------+
                                              | Serialized State Frames (30 Hz)
                                              v
+-----------------------------------------------------------------------------------+
|                             VISUALIZATION ENGINE (vis/)                           |
|                                                                                   |
|  +--------------------------------+       +------------------------------------+  |
|  |     FastAPI / WebSocket Server | ----> |   Three.js WebGL 3D Cockpit        |  |
|  |     (vis/server.py)            |       |   - 3D UAVs with animated rotors   |  |
|  |     Broadcasts telemetry JSON  |       |   - Glowing multi-hop link tubes   |  |
|  |     Handles HUD commands       |       |   - Spline packet pulses           |  |
|  |                                |       |   - Glassmorphic Telemetry HUD     |  |
|  +--------------------------------+       +------------------------------------+  |
+-----------------------------------------------------------------------------------+
                                              ^
                                              |
+-----------------------------------------------------------------------------------+
|                        ENTRYPOINT & ORCHESTRATION LAYER                           |
|                                                                                   |
|  +-----------------------------------------------------------------------------+  |
|  |  run_simulation.py                                                          |  |
|  |  - Auto-checks/installs requirements                                        |  |
|  |  - Spawns simulation loop + optional web server                            |  |
|  |  - CLI options: --headless, --drones, --duration, --pois, --port            |  |
|  +-----------------------------------------------------------------------------+  |
+-----------------------------------------------------------------------------------+
```

## Feature Inventory
| # | Feature | Description | Milestone | Source |
|---|---------|-------------|-----------|--------|
| 1 | 3D 6-DOF Quadcopter Kinematics | Newton-Euler rigid body dynamics, position/velocity integration, attitude quaternions | M1 | Survey |
| 2 | Flocking & Collision Avoidance | Khatib Artificial Potential Fields + Reynolds boids separation & downwash repulsion | M1 | Survey |
| 3 | 4-Tier Altitude Corridor Layering | Altitudes partitioned: launch [0,20]m, PoI survey [25,45]m, transit [50,65]m, relay [70,90]m | M1 | Survey |
| 4 | Disaster Environment & Obstacles | 500m x 500m disaster area, 3D AABB building obstacles, GCS base coordinates | M1 | Survey |
| 5 | Battery & Energy Depletion Model | Electro-mechanical power draw ($P_{base} + P_{prop} + P_{sen} + P_{rf}$) with RTL low-battery threshold | M1 | Survey |
| 6 | RF Propagation & Path Loss Model | 2.4 GHz Friis FSPL ($PL_0 = 40.05$ dB) + Log-distance model ($\eta_{LoS}=2.05, \eta_{NLoS}=3.60$) | M2 | Survey |
| 7 | 3D Ray-AABB Occlusion Engine | Ray-slab intersection test for LOS occlusion; $+22$ dB building penetration loss forcing multi-hop | M2 | Survey |
| 8 | Dynamic Link-State Routing (FANET-DLS) | Sub-millisecond Dijkstra shortest-path routing with composite cost (SNR, distance, obstacle penalty) | M2 | Survey |
| 9 | DTN Store-and-Forward Buffering | 250-packet FIFO ring buffer caching survey packets during transient link interruptions | M2 | Survey |
| 10 | Packet Trace & Hop Event Logging | Structured logging proving multi-hop paths (e.g., `UAV_3 -> UAV_1 -> GCS`, timestamps, latency) | M2 | Survey |
| 11 | Network Telemetry Metrics | Continuous calculation of Packet Delivery Ratio (PDR), End-to-End latency, and hop count distribution | M2 | Survey |
| 12 | Disaster PoI Generation & Priority | Multi-priority PoIs (Survivor Search, Structural Collapse, Hazard Zone) with data collection requirements | M3 | Survey |
| 13 | 10-State MAVSDK-Compliant FSM | Autonomous flight states: IDLE, TAKEOFF, TRANSIT, SURVEYING, RELAY, DATA_TX, RTL, LANDING, COMPLETED | M3 | Survey |
| 14 | Dynamic Role Allocation | Fleet dynamically assigned into Survey UAVs (PoI inspection) and Relay UAVs (communication bridging) | M3 | Survey |
| 15 | Virtual Spring Mesh (VSM) Relays | Autonomous positioning of Relay UAVs connected via spring-damper forces to GCS and Survey UAVs | M3 | Survey |
| 16 | Survey Dwell & Sensor Simulation | Dwell timer at PoIs, simulated high-resolution sensor payload generation, transmission triggers | M3 | Survey |
| 17 | Three.js 3D WebGL Cockpit | Hardware-accelerated 3D visualization, orbiting camera, grid terrain, GCS tower, PoI beacons | M4 | Survey |
| 18 | Dynamic Multi-Hop Link Tube Render | Glowing 3D links between UAVs/GCS color-coded by SNR/hop count, updating dynamically at 30 Hz | M4 | Survey |
| 19 | Catmull-Rom Animated Packet Pulses | Visual glowing photon packets traveling along multi-hop relay splines from surveying UAVs to GCS | M4 | Survey |
| 20 | Glassmorphic HUD & Telemetry Dashboard | Real-time overlay showing swarm status, active routing table, PDR, latency, and PoI survey progress | M4 | Survey |
| 21 | WebSocket Telemetry Streaming Server | FastAPI / WebSocket server broadcasting compact JSON frames (< 1.5 KB @ 30 Hz) to connected clients | M4 | Survey |
| 22 | Automated Setup & Single Run Script | `run_simulation.py` checking dependencies, launching web server, and opening browser automatically | M5 | Survey |
| 23 | CLI Parameter Control & Headless Mode | CLI flags (`--headless`, `--duration`, `--drones`, `--pois`, `--port`, `--speed`) for tests and benchmarks | M5 | Survey |
| 24 | E2E 4-Tier Test Suite | Requirement-driven opaque-box test suite covering Tiers 1-4 with comprehensive assertions | E2E | Survey |
| 25 | 3D LiDAR Perception & Voxel Grid Mapping | Multi-beam rotating LiDAR scanner, range noise, OctoMap log-odds 3D voxel reconstruction | M7 | Survey |
| 26 | Dual-Viewport Split-Screen Cockpit | Side-by-side synchronized view: External Theater Reality vs Autonomous Drone SLAM Perception | M7 | Survey |
| 27 | Real-Time Scientific Analytical Dashboard | Chart.js telemetry: 9-state EKF convergence, PDR & throughput, RF SNR vs distance, battery curves | M7 | Survey |
| 28 | Swarm Scaling & Heterogeneous Fleet | 8-UAV fleet (Surveyors, High-Altitude Relays, Pathfinders) & 5 disaster PoIs with CBBA consensus | M7 | Survey |
| 29 | Investor Pitch & Executive Presentation Mode | One-click 60s choreographed cinematic tour with commercial KPI callout banners | M7 | Survey |

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| 1 | Core Drone Kinematics, Dynamics & Environment Engine | `sim/core.py`, `sim/drone.py`, `sim/environment.py`, `sim/obstacles.py`, `sim/types.py` | none | COMPLETED |
| 2 | Resilient FANET Networking & Dynamic Multi-Hop Routing | `sim/network.py`, `sim/routing.py`, `sim/channel.py`, `sim/packets.py` | M1 | COMPLETED |
| 3 | PoI Mission Control, Dynamic Role Allocation & VSM Relay | `sim/mission.py`, `sim/poi.py`, `sim/fsm.py`, `sim/vsm.py` | M1, M2 | COMPLETED |
| 4 | Interactive 3D WebGL Cockpit, Telemetry Streamer & HUD | `vis/server.py`, `vis/static/index.html`, `vis/static/js/cockpit.js`, `vis/static/css/style.css`, Three.js vendor bundle | M1, M2, M3 | COMPLETED |
| 5 | Unified Entrypoint Script, Automated Setup & CLI Runner | `run_simulation.py`, `requirements.txt`, headless validation | M1, M2, M3, M4 | COMPLETED |
| E2E | Opaque-Box E2E Test Suite (Tiers 1-4) | `tests/e2e/test_runner.py`, `tests/e2e/test_tier1_features.py`, `tests/e2e/test_tier2_boundaries.py`, `tests/e2e/test_tier3_combinations.py`, `tests/e2e/test_tier4_scenarios.py` | none | COMPLETED |
| 6 | Final E2E Test Pass (100%) & Tier 5 Adversarial Hardening | Verification of 100% pass rate on Tiers 1-4 + Tier 5 Challenger stress tests & Forensic Audit | M5, E2E | COMPLETED |
| 7 | Investor & Defense-Grade Autonomy Upgrade | 3D LiDAR/Voxel SLAM, Dual-Viewport Split Screen, Chart.js Analytics, 8-UAV Fleet, Investor Demo | M1-M6 | COMPLETED |

## Interface Contracts

### 1. Kinematics (`sim/drone.py`) <-> Environment (`sim/environment.py`)
```python
@dataclass
class DroneState:
    id: str
    role: str  # 'SURVEY' | 'RELAY'
    position: np.ndarray  # shape (3,), [x, y, z] in meters
    velocity: np.ndarray  # shape (3,), [vx, vy, vz] in m/s
    attitude: np.ndarray  # shape (3,), [roll, pitch, yaw] in radians
    rotor_speeds: np.ndarray  # shape (4,), rad/s
    battery_soc: float  # [0.0, 1.0]
    flight_mode: str  # 'IDLE', 'TAKEOFF', 'TRANSIT', 'SURVEYING', 'RELAY', 'RTL', 'LANDED'
    assigned_poi_id: Optional[str] = None
    target_position: Optional[np.ndarray] = None
```

### 2. Kinematics (`sim/drone.py`) <-> Network Subsystem (`sim/network.py`)
```python
class NetworkEngine:
    def update_topology(self, node_positions: Dict[str, np.ndarray], obstacles: List[AABB]) -> None:
        """Computes pairwise SNR, applies 3D ray-AABB occlusion penalties, and rebuilds routing graph."""
        ...
    def find_route(self, src_id: str, dst_id: str) -> List[str]:
        """Returns ordered list of node IDs forming optimal multi-hop path, e.g. ['UAV_3', 'UAV_1', 'GCS']."""
        ...
    def transmit_packet(self, packet: NetworkPacket) -> bool:
        """Forwards packet along computed route, updating hop trace and telemetry."""
        ...
```

### 3. Mission Logic (`sim/mission.py`) <-> VSM Relay (`sim/vsm.py`)
```python
class RelayPositioningEngine:
    def compute_relay_setpoints(
        self,
        gcs_pos: np.ndarray,
        survey_drones: List[DroneState],
        relay_drones: List[DroneState],
        obstacles: List[AABB]
    ) -> Dict[str, np.ndarray]:
        """Computes 3D target coordinates for relay drones using Virtual Spring Mesh and elevation gradient."""
        ...
```

### 4. Simulation Engine (`sim/core.py`) <-> Visualization Server (`vis/server.py`)
```python
@dataclass
class TelemetrySnapshot:
    sim_time: float
    drones: List[Dict[str, Any]]
    gcs: Dict[str, Any]
    pois: List[Dict[str, Any]]
    active_routes: List[List[str]]  # e.g. [['UAV_3', 'UAV_1', 'GCS']]
    links: List[Dict[str, Any]]     # [{'source': 'UAV_3', 'target': 'UAV_1', 'snr': 18.2, 'status': 'ACTIVE'}]
    packets: List[Dict[str, Any]]   # In-flight packets with progress [0.0, 1.0] along route
    metrics: Dict[str, float]       # {'pdr': 0.98, 'avg_latency_ms': 14.2, 'completed_pois': 3}
```

## Code Layout
```
d:/drone model/IIT Bombay/
├── run_simulation.py               # Unified execution & setup script (Milestone 5)
├── requirements.txt                # Python package dependencies
├── README.md                       # Comprehensive operational & architecture guide
├── sim/                            # Pure Python simulation core
│   ├── __init__.py
│   ├── core.py                     # Master simulation loop and coordinator (Milestone 1)
│   ├── drone.py                    # 6-DOF Quadcopter dynamics & kinematics (Milestone 1)
│   ├── environment.py              # 500x500m disaster area, coordinate limits (Milestone 1)
│   ├── obstacles.py                # 3D AABB buildings & ray-slab occlusion (Milestone 1)
│   ├── types.py                    # Dataclasses & shared data models (Milestone 1)
│   ├── channel.py                  # RF propagation, Friis, log-distance path loss (Milestone 2)
│   ├── routing.py                  # Dynamic Link-State Dijkstra (FANET-DLS) (Milestone 2)
│   ├── packets.py                  # Packet dataclass, DTN ring buffer, hop logging (Milestone 2)
│   ├── network.py                  # Master FANET communication manager (Milestone 2)
│   ├── poi.py                      # Point of Interest generator and tracking (Milestone 3)
│   ├── fsm.py                      # 10-state MAVSDK-compliant drone state machine (Milestone 3)
│   ├── vsm.py                      # Virtual Spring Mesh cooperative relay positioning (Milestone 3)
│   └── mission.py                  # Disaster survey mission coordinator & task allocation (Milestone 3)
├── vis/                            # Visualization engine
│   ├── __init__.py
│   ├── server.py                   # FastAPI / WebSocket server (Milestone 4)
│   └── static/                     # Three.js 3D cockpit web application (Milestone 4)
│       ├── index.html              # Cockpit view & glassmorphic HUD
│       ├── css/
│       │   └── style.css           # Modern sci-fi tactical command HUD styling
│       └── js/
│           ├── cockpit.js          # Three.js WebGL rendering, camera, animation loop
│           └── vendor/             # Vendored Three.js, OrbitControls, UnrealBloom
└── tests/                          # E2E and unit test suites
    ├── __init__.py
    ├── conftest.py                 # Shared pytest fixtures
    ├── e2e/                        # Opaque-box E2E testing track
    │   ├── test_runner.py          # Unified test orchestrator & report generator
    │   ├── test_tier1_features.py  # Tier 1: Feature isolation (>= 5 tests per feature)
    │   ├── test_tier2_boundaries.py# Tier 2: Boundary & corner cases
    │   ├── test_tier3_combinations.py # Tier 3: Pairwise feature interactions
    │   └── test_tier4_scenarios.py # Tier 4: Real-world post-disaster scenarios
    └── unit/                       # Subsystem unit tests
