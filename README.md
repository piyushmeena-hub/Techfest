# UAV-X: Post-Disaster Aerial Swarm System

> A unified system combining the world's best UAV repositories to solve resilient multi-hop aerial communication and PoI surveying after major disasters.

## Challenge
A major earthquake/landslide disrupts terrestrial comms. A GCS deploys a fleet of UAVs to:
- Survey designated Points of Interest (PoIs)
- Collect imagery and situational data
- Relay information back via a resilient multi-hop aerial network

## Architecture (5 Layers)
```
Layer 5 → GCS Dashboard (Flask/Streamlit)
Layer 4 → Mission Intelligence (MAVSDK + Custom)
Layer 3 → Planning & Autonomy (EGO-Swarm + Fast-Planner)
Layer 2 → Flight + Comms (XTDrone + PX4 + NS-3)
Layer 1 → Simulation World (Gazebo → AirSim)
```

## Build Phases
| Phase | Description | Status |
|-------|-------------|--------|
| Phase 0 | Algorithm prototyping (pure Python) | 🔨 In Progress |
| Phase 1 | Core SITL (XTDrone + PX4 + MAVSDK) | ⏳ Pending |
| Phase 2 | Communication layer (NS-3) | ⏳ Pending |
| Phase 3 | Planning & Autonomy (EGO-Swarm) | ⏳ Pending |
| Phase 4 | Intelligence & Resilience | ⏳ Pending |
| Phase 5 | Visual Validation (AirSim) | ⏳ Pending |

## Quick Start (Phase 0)
```bash
cd phase0_prototype
pip install -r requirements.txt
python main.py
```

## Key Capabilities (MVP)
- [x] PoI surveying with dynamic assignment
- [x] Multi-hop relay chain formation
- [x] Battery-aware return-to-base
- [x] Relay handoff on low battery
- [x] Link failure detection and rerouting
- [x] Priority data queue (survivor > routine)
- [x] Collision-free decentralized planning
- [x] EchoRescue-style acceptance logs

## Repo Credits
- **XTDrone** — Multi-UAV PX4/ROS2/Gazebo scaffold
- **PX4 Autopilot** — Flight stack
- **MAVSDK** — Mission orchestration
- **EGO-Swarm + Fast-Planner** — Decentralized planning
- **UAV Swarm Network Simulator + IoD_Sim** — NS-3 comms
- **U2UData** — Cooperative perception
- **EchoRescue** — Evidence patterns (study only)
- **AirSim** — Visual simulation
