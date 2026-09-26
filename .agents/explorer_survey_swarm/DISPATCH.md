# Dispatch: Explorer 2 — UAV Swarm & Kinematics Specialist

## Assignment
Investigate and design the UAV fleet dynamics, kinematics, swarm coordination, and disaster PoI survey mission logic.

## Inputs
- ORIGINAL_REQUEST: `d:\drone model\IIT Bombay\.agents\ORIGINAL_REQUEST.md`

## Objectives
1. Synthesize concepts from reference repositories (`gym-pybullet-drones`, `UAV-Swarm-Sim`, `MAVSDK`):
   - 3D quadcopter kinematics and state updates (position, velocity, acceleration, orientation/attitude, yaw control).
   - Swarm navigation and coordination: waypoint following, velocity obstacle/artificial potential field collision avoidance between drones and obstacles.
2. Formulate PoI Survey Mission Logic:
   - Dynamic disaster environment representation (disaster zone bounds, high-priority emergency PoIs, obstacle/hazard zones).
   - UAV role assignment: Surveying drones (assigned to fly to PoIs, hold/orbit position, gather data) vs Relay drones (maneuver to maintain communication line-of-sight and network bridge).
   - Mission lifecycle states: IDLE -> TAKEOFF -> TRANSIT -> SURVEYING (data collection) -> RELAY -> RETURN_TO_BASE.
3. Write your comprehensive report to `d:\drone model\IIT Bombay\.agents\explorer_survey_swarm\survey_swarm_report.md` and deliver `handoff.md`.

## 2026-09-25T14:26:00Z
Received user request:
- Role: Explorer 2: UAV Swarm & Kinematics Specialist
- Objective: Authoritative technical survey on UAV fleet dynamics, kinematics, swarm coordination, and disaster survey mission logic.
- Extract concepts from gym-pybullet-drones, UAV-Swarm-Sim, and MAVSDK:
  1. 3D Quadcopter kinematics & physical state model (position [x, y, z], velocity, acceleration, attitude [roll, pitch, yaw], motor/flight limits, battery model).
  2. Swarm navigation & collision avoidance: Reynolds flocking / artificial potential fields / velocity obstacle methods.
  3. PoI Survey Mission Logic:
     - Disaster zone definition with multiple priority PoIs (e.g. damaged structures, survivor zones).
     - Role allocation: Survey UAVs and Relay UAVs.
     - Mission state machine: IDLE -> TAKEOFF -> TRANSIT -> SURVEYING -> DATA_RELAY -> RETURN_TO_BASE.
  4. Concrete API contracts, class designs, and step-by-step update tick routines.
- Outputs: `survey_swarm_report.md` and `handoff.md`.

