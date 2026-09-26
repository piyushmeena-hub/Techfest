# Original User Request

## 2026-09-25T14:23:45Z

<USER_REQUEST>
A 3D simulation of a resilient multi-hop aerial communication network using a fleet of UAVs for post-disaster survey and data relay to a Ground Control Station (GCS). The system extracts and combines concepts from repositories like UAV-Swarm-Sim, MAVSDK, and gym-pybullet-drones to create an impressive visual and functional demonstration of swarm networking.

Use a very large team of agents.

Working directory: d:/drone model/IIT Bombay
Integrity mode: development

## Requirements

### R1. 3D Swarm Simulation Environment
A 3D graphical simulation must be created to visualize the UAV fleet. It should visually demonstrate the UAVs flying in a coordinated manner over a simulated environment.

### R2. Multi-hop Communication Modeling
The system must model the communication network, actively determining and visualizing the multi-hop routing paths from any surveying UAV back to the stationary GCS node, especially when direct line-of-sight is unavailable.

### R3. PoI Surveying Logic
UAVs must be assigned specific Points of Interest (PoIs) to survey. They should navigate to these PoIs, hold position to simulate data collection, and transmit the data through the network.

## Acceptance Criteria

### Automated Execution
- [ ] A single setup/run script exists (e.g., `run_simulation.py` or `launch.sh`) that installs any python dependencies and starts the 3D visualization.

### Functional Demonstration
- [ ] The 3D visualization clearly shows multiple UAVs, the GCS, and the PoIs.
- [ ] The system visually or programmatically logs the communication links, proving that multi-hop routing is occurring (e.g., UAV A -> UAV B -> GCS).
- [ ] UAVs successfully reach their assigned PoIs and the network maintains connectivity.
</USER_REQUEST>
