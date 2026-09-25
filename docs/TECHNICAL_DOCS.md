# UAV-X: Technical Documentation

## Project Overview
**UAV-X** is a post-disaster aerial swarm system that combines the world's best UAV repositories into a unified 5-layer architecture to solve resilient multi-hop aerial communication and PoI surveying.

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  LAYER 5: GCS Dashboard (Flask Web UI, real-time updates)   │
├─────────────────────────────────────────────────────────────┤
│  LAYER 4: Mission Intelligence                              │
│  MissionManager │ FaultDetector │ BatteryScheduler          │
│  RelayManager   │ PriorityQueue │ AcceptanceReporter         │
├─────────────────────────────────────────────────────────────┤
│  LAYER 3: Planning & Autonomy                               │
│  EGO-Swarm Bridge (ROS2) │ Fast-Planner Bridge (ESDF)       │
├─────────────────────────────────────────────────────────────┤
│  LAYER 2: Flight & Communications                           │
│  XTDrone + PX4 SITL │ MAVSDK │ NS-3 via IoD_Sim             │
├─────────────────────────────────────────────────────────────┤
│  LAYER 1: Simulation World                                  │
│  Gazebo (Phase 1-4) │ gym-pybullet (Phase 0) │ AirSim (P5) │
└─────────────────────────────────────────────────────────────┘
```

## Module Reference

### Phase 0: Core Algorithm (`phase0_prototype/`)
| Module | File | Purpose |
|--------|------|---------|
| Data Models | `core/models.py` | UAV, PoI, GCS, CommLink, DataPacket entities |
| Comm Network | `core/comm_network.py` | NS-3-style multi-hop routing (Dijkstra) |
| Mission Manager | `core/mission_manager.py` | MAVSDK-style orchestration brain |
| Acceptance Reporter | `core/acceptance_reporter.py` | EchoRescue-style evidence tests |
| Terminal Viz | `visualizer/terminal_viz.py` | ASCII real-time mission map |
| Matplotlib Viz | `visualizer/matplotlib_viz.py` | Path traces, battery, network charts |
| Scenario | `configs/scenario.py` | Disaster zone, UAV fleet, PoI config |
| Main | `main.py` | Entry point with fault injection |

### Phase 1: SITL (`phase1_sitl/`)
| Module | File | Purpose |
|--------|------|---------|
| MAVSDK Controller | `mavsdk_scripts/mission_controller.py` | Async PX4 mission orchestration |
| Fleet Launcher | `launch/spawn_fleet.sh` | Spawn 10 PX4 SITL instances |
| Gazebo World | `worlds/disaster_zone.sdf` | Post-earthquake simulation environment |

### Phase 2: Communications (`phase2_comms/`)
| Module | File | Purpose |
|--------|------|---------|
| NS-3 Bridge | `ns3_models/swarm_network.py` | NS-3 RF simulation + pure-Python fallback |

### Phase 3: Planning (`phase3_planning/`)
| Module | File | Purpose |
|--------|------|---------|
| EGO-Swarm Bridge | `ego_swarm_bridge/planner_bridge.py` | ROS2 decentralized planning + potential field fallback |
| Fast-Planner Bridge | `fast_planner_bridge/fast_planner_bridge.py` | ESDF + B-spline trajectory planning |

### Phase 4: Intelligence (`phase4_intelligence/`)
| Module | File | Purpose |
|--------|------|---------|
| Relay Manager | `relay_manager/relay_manager.py` | Dynamic relay chain and slot handoff |
| Battery Scheduler | `battery_scheduler/battery_scheduler.py` | Predictive RTL scheduling |
| Fault Detector | `fault_detector/fault_detector.py` | Real-time health monitoring |
| Priority Queue | `priority_queue/priority_queue_manager.py` | CRITICAL-first data delivery |

### GCS Dashboard (`dashboard/`)
| Module | File | Purpose |
|--------|------|---------|
| Flask Server | `app.py` | REST API + background simulation runner |
| Web UI | `templates/index.html` | Real-time tactical dashboard |

## Running the Project

### Phase 0 (Works Now — No Dependencies)
```bash
cd ~/Desktop/techfest/uav_x/phase0_prototype

# Standard run with live ASCII visualization
python3 main.py --ticks 300 --fault-tick 80

# Fast headless run
python3 main.py --no-viz --ticks 300

# Generate matplotlib charts
python3 visualizer/matplotlib_viz.py

# Run tests
python3 -m pytest tests/ -v
```

### GCS Dashboard
```bash
cd ~/Desktop/techfest/uav_x
pip install flask
cd dashboard && python3 app.py
# Open http://localhost:5000
```

### Full Stack (Docker)
```bash
cd ~/Desktop/techfest/uav_x
docker compose up gcs-dashboard simulation
# For SITL (Linux + PX4 required):
docker compose --profile sitl up
```

### Phase 1 SITL (Linux + ROS2 + PX4 required)
```bash
# Install PX4, ROS2 Humble, Gazebo first
cd ~/Desktop/techfest/uav_x
./setup.sh   # clones all vendor repos
cd phase1_sitl/launch && ./spawn_fleet.sh
# In another terminal:
cd phase1_sitl/mavsdk_scripts && python3 mission_controller.py
```

## Key Design Decisions

### Why NS-3 for Communications?
NS-3 models real RF propagation including:
- Free-space path loss
- Interference and collision
- Packet-level simulation
Unlike simplified "always-connected" models, this proves the system works under real comms constraints.

### Why EGO-Swarm + Fast-Planner?
- **EGO-Swarm**: Each UAV plans independently — no single point of failure
- **Fast-Planner**: ESDF distance fields ensure smooth, obstacle-free trajectories
- Both have demonstrated real-hardware multi-UAV flights

### Why MAVSDK over Raw MAVLink?
MAVSDK provides a stable Python async API that abstracts vehicle-specific differences, supports multiple simultaneous connections, and handles reconnection automatically.

### Why Layered Architecture?
Each layer is independently testable and replaceable:
- Swap Gazebo for AirSim without changing planning layer
- Swap NS-3 for real radio hardware without changing mission layer
- Run Phase 0 pure-Python prototype without ANY external dependencies

## Acceptance Criteria (MVP)

| Behavior | Test | Evidence Source |
|----------|------|-----------------|
| PoI Surveying | ≥85% PoIs surveyed | `acceptance_reporter.py` |
| GCS Delivery | All surveyed data reaches GCS | `gcs.acceptance_log` |
| Relay Reassignment | New relay assigned after RTL | `event_log` type=RELAY_ASSIGN |
| Comm Recovery | Failed deliveries retried and succeed | `event_log` type=DELIVERY_FAIL → DELIVERED |
| Priority Handling | CRITICAL packets delivered before LOW | delivery tick comparison |
| Battery-Aware RTL | 0 battery-crash failures | `uav.status != FAILED` |
| Safety | 0 unplanned crashes | injected faults excluded |
| Reproducibility | Full event + telemetry logs | JSON + Markdown reports |

## Repository Credits

| Repo | Used For | License |
|------|----------|---------|
| [XTDrone](https://github.com/robin-shaun/XTDrone) | Multi-UAV PX4/ROS2/Gazebo scaffold | MIT |
| [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) | Flight stack, SITL | BSD-3 |
| [MAVSDK-Python](https://github.com/mavlink/MAVSDK-Python) | Mission orchestration API | Apache-2.0 |
| [EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2) | Decentralized trajectory planning | GPL-3.0 |
| [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) | ESDF + B-spline planning | LGPL-3.0 |
| [UAV Swarm Network Simulator](https://github.com/nathanlct/uav-swarm-network-simulator) | NS-3 comms model inspiration | MIT |
| [IoD_Sim](https://github.com/telematics-lab/IoD_Sim) | NS-3 drone networking | GPL-3.0 |
| [U2UData](https://github.com/ucla-mobility/U2UData) | Cooperative perception reference | MIT |
| [EchoRescue](https://echorescue.io) | Evidence/testing patterns | Proprietary — study only |
| [Microsoft AirSim](https://github.com/microsoft/AirSim) | Phase 5 visual simulation | MIT |
| [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) | Phase 0 lightweight sim inspiration | MIT |

> **Licensing Note:** Always check each repository's license before redistributing derived code. GPL-licensed repositories impose source-distribution obligations on covered derivatives.
