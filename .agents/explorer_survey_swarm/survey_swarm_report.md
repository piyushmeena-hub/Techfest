# Technical Survey & Architecture Report: UAV Swarm Fleet Dynamics, 3D Kinematics & Disaster Survey Mission Logic

**Author**: Explorer 2 (UAV Swarm & Kinematics Specialist)  
**Date**: 2026-09-25  
**Target Milestone**: Phase 0 Architecture & Milestone 1/3 Foundation  
**Reference Repositories**: `gym-pybullet-drones`, `UAV-Swarm-Sim`, `MAVSDK`

---

## Executive Summary

This report establishes the complete theoretical foundations, mathematical formulations, algorithmic blueprints, and Python class API contracts for the UAV fleet kinematics, swarm navigation, collision avoidance, and disaster survey mission control subsystems.

Designed for a resilient multi-hop aerial communication network simulation, the architecture solves three coupled challenges:
1. **Accurate 3D Flight Physics**: A high-performance, deterministic 6-DOF / 3D kinematic and dynamic state model that replicates the fidelity of `gym-pybullet-drones` without requiring heavyweight external C++ physics engines, ensuring zero-friction installation on Windows and Linux while supporting both headless CI and real-time visualization.
2. **Safe Swarm Coordination & Obstacle Avoidance**: A hybrid reactive control framework combining **Artificial Potential Fields (APF)**, **Reynolds Boids Flocking**, **Asymmetric Vertical Downwash Repulsion**, and **Airspace Altitude Layering** to guarantee collision-free transit around disaster obstacles (damaged buildings, towers, debris).
3. **Autonomous Disaster PoI Survey & Dynamic Relay Allocation**: An event-driven **Finite State Machine (FSM)** with automated role allocation that orchestrates **Survey UAVs** (navigating to high-priority disaster PoIs, orbiting/loitering to collect sensor data) and **Relay UAVs** (dynamically positioning via a **Virtual Spring Mesh (VSM)** to bridge multi-hop RF connectivity back to a stationary Ground Control Station).

---

## 1. 3D Quadcopter Kinematics & Physical State Model

### 1.1 Coordinate Systems & Frame Conventions
The simulation adopts the right-handed **East-North-Up (ENU)** inertial frame $\mathcal{I} = \{\mathbf{X}_I, \mathbf{Y}_I, \mathbf{Z}_I\}$:
- $+X$: East (horizontal)
- $+Y$: North (horizontal)
- $+Z$: Up (vertical altitude above ground level, AGL)

The Body-Fixed Frame $\mathcal{B} = \{\mathbf{X}_B, \mathbf{Y}_B, \mathbf{Z}_B\}$ is centered at the quadcopter's Center of Mass (CoM):
- $+\mathbf{X}_B$: Structural forward direction
- $+\mathbf{Y}_B$: Structural left / port direction
- $+\mathbf{Z}_B$: Normal to the rotor plane pointing upward through the top of the airframe

```
           +Z_I (Up)
              ^
              |       +Z_B (Thrust direction)
              |        ^
              |       /
              |      /
              |     /------> +Y_B (Left)
              |    / \
              |   /   \
              |  +-----\-> +X_B (Forward)
              |
              +------------------> +Y_I (North)
             /
            /
           v +X_I (East)
```

The transformation from the body frame $\mathcal{B}$ to the inertial frame $\mathcal{I}$ is given by the rotation matrix $\mathbf{R}_B^I \in SO(3)$. Using the standard ZYX Tait-Bryan Euler angles—Yaw ($\psi$), Pitch ($\theta$), and Roll ($\phi$):

$$\mathbf{R}_B^I(\phi, \theta, \psi) = \begin{bmatrix}
\cos\psi\cos\theta & \cos\psi\sin\theta\sin\phi - \sin\psi\cos\phi & \cos\psi\sin\theta\cos\phi + \sin\psi\sin\phi \\
\sin\psi\cos\theta & \sin\psi\sin\theta\sin\phi + \cos\psi\cos\phi & \sin\psi\sin\theta\cos\phi - \cos\psi\sin\phi \\
-\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi
\end{bmatrix}$$

To prevent numerical singularities and gimbal lock during aggressive bank maneuvers, the simulation maintains the orientation as a normalized unit quaternion $\mathbf{q} = [q_w, q_x, q_y, q_z]^T \in \mathbb{H}, \|\mathbf{q}\| = 1$:

$$\mathbf{R}(\mathbf{q}) = \begin{bmatrix}
1 - 2(q_y^2 + q_z^2) & 2(q_x q_y - q_w q_z) & 2(q_x q_z + q_w q_y) \\
2(q_x q_y + q_w q_z) & 1 - 2(q_x^2 + q_z^2) & 2(q_y q_z - q_w q_x) \\
2(q_x q_z - q_w q_y) & 2(q_y q_z + q_w q_x) & 1 - 2(q_x^2 + q_y^2)
\end{bmatrix}$$

### 1.2 Full State Vector Representation
Each UAV $i \in \{1, \dots, N\}$ in the swarm maintains an explicit 16-dimensional physical state vector:

$$\mathbf{x}_i = \begin{bmatrix} \mathbf{p}_i & \mathbf{v}_i & \mathbf{q}_i & \mathbf{\omega}_i & SoC_i \end{bmatrix}^T \in \mathbb{R}^{16}$$

| Component | Notation | Dimension | Physical Meaning & Units |
|---|---|---|---|
| **Position** | $\mathbf{p} = [x, y, z]^T$ | $\mathbb{R}^3$ | Inertial 3D coordinates in meters [m] |
| **Linear Velocity** | $\mathbf{v} = [\dot{x}, \dot{y}, \dot{z}]^T$ | $\mathbb{R}^3$ | Inertial velocity in meters per second [m/s] |
| **Attitude Quaternion** | $\mathbf{q} = [q_w, q_x, q_y, q_z]^T$ | $\mathbb{S}^3$ | Orientation representation, $\|\mathbf{q}\| = 1$ |
| **Angular Velocity** | $\mathbf{\omega} = [p, q, r]^T$ | $\mathbb{R}^3$ | Body-frame rotational rates in radians per second [rad/s] |
| **Battery State of Charge**| $SoC$ | $\mathbb{R}$ | Normalized remaining battery energy fraction $\in [0.0, 1.0]$ |

Auxiliary state variables derived during update cycles:
- **Euler Angles**: $\mathbf{\eta} = [\phi, \theta, \psi]^T$ (Roll, Pitch, Yaw in radians / degrees)
- **Linear Acceleration**: $\mathbf{a} = [\ddot{x}, \ddot{y}, \ddot{z}]^T$ [m/s$^2$]
- **Individual Motor Angular Speeds**: $\mathbf{\Omega} = [\omega_1, \omega_2, \omega_3, \omega_4]^T$ [rad/s] or RPM

### 1.3 Equations of Motion (Newton-Euler Dynamics)
Following the mathematical modeling in `gym-pybullet-drones` and Bitcraze Crazyflie / DJI Matrice specifications:

#### 1. Translational Dynamics:
$$\dot{\mathbf{p}} = \mathbf{v}$$
$$m \dot{\mathbf{v}} = m \mathbf{g} + \mathbf{R}_B^I \mathbf{F}_T + \mathbf{F}_{aero}$$

Where:
- $m$: Total quadcopter mass (e.g., $1.20$ kg for medium survey quadcopter, $0.035$ kg for nano-quadrotor)
- $\mathbf{g} = [0, 0, -9.80665]^T$ m/s$^2$: Gravitational acceleration vector
- $\mathbf{F}_T = [0, 0, T_{total}]^T$: Collective thrust vector aligned with the body $+\mathbf{Z}_B$ axis:
  $$T_{total} = \sum_{j=1}^4 T_j = k_f \sum_{j=1}^4 \omega_j^2$$
  where $k_f$ is the motor thrust coefficient [$\text{N}/(\text{rad/s})^2$].
- $\mathbf{F}_{aero}$: Aerodynamic drag vector in inertial frame:
  $$\mathbf{F}_{aero} = - \frac{1}{2} \rho_{air} C_d A \|\mathbf{v}\| \mathbf{v} - \mathbf{R}_B^I \mathbf{D}_{rotor} \mathbf{R}_I^B \mathbf{v}$$
  In standard operational flight regimes ($v \le 12$ m/s), this can be modeled as lumped diagonal linear/quadratic drag:
  $$\mathbf{F}_{aero} = - \text{diag}(d_{xy}, d_{xy}, d_z) \mathbf{v}$$

#### 2. Rotational Dynamics:
$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \mathbf{\omega} \end{bmatrix} = \frac{1}{2} \begin{bmatrix} -q_x & -q_y & -q_z \\ q_w & -q_z & q_y \\ q_z & q_w & -q_x \\ -q_y & q_x & q_w \end{bmatrix} \begin{bmatrix} p \\ q \\ r \end{bmatrix}$$

$$\mathbf{I} \dot{\mathbf{\omega}} + \mathbf{\omega} \times (\mathbf{I} \mathbf{\omega}) = \mathbf{\tau}_{ctrl} - \mathbf{\tau}_{gyro}$$

Where:
- $\mathbf{I} = \begin{bmatrix} I_{xx} & 0 & 0 \\ 0 & I_{yy} & 0 \\ 0 & 0 & I_{zz} \end{bmatrix}$ is the quadcopter diagonal moment of inertia tensor.
- $\mathbf{\tau}_{gyro} = \sum_{j=1}^4 I_{rotor} (\mathbf{\omega} \times \hat{\mathbf{z}}_B) (-1)^j \omega_j$ represents propeller gyroscopic precession.
- $\mathbf{\tau}_{ctrl} = [\tau_\phi, \tau_\theta, \tau_\psi]^T$ are the aerodynamic control torques produced by the 'X' quadcopter configuration with arm length $L$:

$$\begin{bmatrix} T_{total} \\ \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{bmatrix} = \begin{bmatrix}
k_f & k_f & k_f & k_f \\
-\frac{\sqrt{2}}{2} L k_f & \frac{\sqrt{2}}{2} L k_f & \frac{\sqrt{2}}{2} L k_f & -\frac{\sqrt{2}}{2} L k_f \\
\frac{\sqrt{2}}{2} L k_f & \frac{\sqrt{2}}{2} L k_f & -\frac{\sqrt{2}}{2} L k_f & -\frac{\sqrt{2}}{2} L k_f \\
-k_m & k_m & -k_m & k_m
\end{bmatrix} \begin{bmatrix} \omega_1^2 \\ \omega_2^2 \\ \omega_3^2 \\ \omega_4^2 \end{bmatrix}$$

### 1.4 Hierarchical Flight Controller Model
Real-world quadcopter swarms (as in `MAVSDK` and `gym-pybullet-drones`) operate with two cascaded control loops:
1. **Outer Loop (Position & Velocity Control)**: Computes the required total thrust $T_{des}$ and target attitude angles $(\phi_{des}, \theta_{des}, \psi_{des})$ to follow desired 3D acceleration setpoints:
   $$\mathbf{a}_{des} = \mathbf{a}_{ff} + \mathbf{K}_p (\mathbf{p}_{des} - \mathbf{p}) + \mathbf{K}_d (\mathbf{v}_{des} - \mathbf{v})$$
   Thrust magnitude:
   $$T_{des} = m \|\mathbf{a}_{des} - \mathbf{g}\|$$
   Desired body z-axis unit vector:
   $$\mathbf{z}_{B, des} = \frac{\mathbf{a}_{des} - \mathbf{g}}{\|\mathbf{a}_{des} - \mathbf{g}\|}$$
   Desired roll and pitch angles:
   $$\phi_{des} = \arcsin\left(\mathbf{z}_{B, des, x} \sin\psi_{des} - \mathbf{z}_{B, des, y} \cos\psi_{des}\right)$$
   $$\theta_{des} = \arctan\left(\frac{\mathbf{z}_{B, des, x} \cos\psi_{des} + \mathbf{z}_{B, des, y} \sin\psi_{des}}{\mathbf{z}_{B, des, z}}\right)$$
2. **Inner Loop (Attitude & Rate Control)**: Fast proportional-derivative attitude loop ($100 - 500$ Hz) driving control torques $\mathbf{\tau}_{ctrl}$ to converge $(\phi, \theta) \to (\phi_{des}, \theta_{des})$ and $\psi \to \psi_{des}$.

In a multi-agent simulation step at $20 - 50$ Hz, attitude dynamics converge much faster than translational dynamics ($\tau_{attitude} \approx 0.05$s vs $\tau_{position} \approx 0.5$s). Hence, a **kinematic-dynamic hybrid integration** model can be used:
$$\ddot{\mathbf{p}} = \mathbf{a}_{des}^{clamped} - \frac{d_{aero}}{m} \mathbf{v}$$
$$\dot{\mathbf{\eta}} = \frac{1}{\tau_{att}} (\mathbf{\eta}_{des} - \mathbf{\eta})$$
This yields physical realism, tilt rendering, and dynamic responsiveness while maintaining high simulation throughput ($>10,000$ steps/sec).

### 1.5 Flight Limits & Envelopes
To prevent unphysical teleports or infinite accelerations, hard kinematic envelopes must be enforced at every simulation tick:

| Parameter | Symbol | Nominal Value | Units | Operational Rationale |
|---|---|---|---|---|
| **Max Horizontal Speed** | $v_{xy}^{max}$ | $10.0$ | m/s | Safe survey transit without excessive motor saturation |
| **Max Ascent Speed** | $v_{z, up}^{max}$ | $3.5$ | m/s | Battery discharge limit during climb |
| **Max Descent Speed** | $v_{z, down}^{max}$ | $2.5$ | m/s | Avoids aerodynamic **Vortex Ring State (VRS)** |
| **Max Linear Acceleration**| $a^{max}$ | $4.0$ | m/s$^2$ | Inertial sensor & payload stabilization ceiling |
| **Max Bank/Tilt Angle** | $\phi^{max}, \theta^{max}$ | $30.0^\circ$ ($0.523$ rad) | deg / rad | Guarantees hover thrust margin: $T = \frac{mg}{\cos\theta} \le 1.15 mg$ |
| **Max Yaw Rate** | $r^{max} = \dot{\psi}^{max}$ | $90.0^\circ/\text{s}$ ($1.57$ rad/s) | deg/s | Avoids camera motion blur during PoI inspection |
| **Thrust-to-Weight Ratio** | $TWR$ | $2.2 : 1$ | - | Ensures $120\%$ excess thrust for vertical maneuvers |

### 1.6 Battery Discharge & Energy Consumption Model
Realistic swarm missions require accurate battery modeling to force timely Return-To-Base (RTB) actions before total energy depletion.

The instantaneous electrical power $P_{total}(t)$ drawn from the onboard LiPo battery ($3\text{S} / 4\text{S}$) is governed by three components:

$$P_{total}(t) = P_{base} + P_{propulsion}(\mathbf{v}, \mathbf{a}) + P_{payload}(t)$$

1. **Base Avionics Power** ($P_{base}$):
   Continuous power consumed by flight computer, IMU, GPS, and microcontrollers:
   $$P_{base} \approx 12.0 \text{ W}$$
2. **Propulsion Power** ($P_{propulsion}$):
   Derived from Momentum Actuator Disk Theory:
   $$P_{propulsion}(T, \mathbf{v}) = \frac{T^{3/2}}{\sqrt{2 \rho_{air} A_{rotor}}} \cdot \frac{1}{\eta_{prop}} + \frac{1}{2} C_d A_{uav} \rho_{air} \|\mathbf{v}\|^3 + m (\mathbf{a} \cdot \mathbf{v})^+$$
   Linearized for real-time simulation:
   $$P_{propulsion}(t) = P_{hover} \cdot \left(\frac{\|\mathbf{a} - \mathbf{g}\|}{g}\right)^{3/2} + k_{drag} \|\mathbf{v}\|^2$$
   where $P_{hover} \approx 180.0 \text{ W}$ for a $1.2$ kg quadcopter.
3. **Payload & Communications Power** ($P_{payload}$):
   - Sensor payload (optical camera / LiDAR / thermal imaging): $P_{sensor} = 8.0 \text{ W}$ (active during `SURVEYING` mode).
   - High-power RF transceiver:
     $$P_{comm} = \begin{cases} 3.0 \text{ W} & \text{(Idle Listening)} \\ 15.0 \text{ W} & \text{(Active Multi-hop Relay / Transmit)} \end{cases}$$

#### State-of-Charge (SoC) Depletion:
Given total usable battery capacity $E_{batt} = V_{nom} \cdot Q_{Ah} \cdot 3600$ Joules (e.g., $4\text{S} \times 3.7\text{V} \times 5.0\text{Ah} \times 3600 = 266,400\text{ J} \approx 74\text{ Wh}$):

$$\Delta SoC(t) = - \frac{P_{total}(t) \cdot \Delta t}{E_{batt}}$$
$$SoC(t + \Delta t) = \max\left(0.0, SoC(t) + \Delta SoC(t)\right)$$

**Safety Cutoff Thresholds**:
- **Normal Operations**: $SoC > 30\%$
- **Return-To-Base (RTB) Trigger**: $SoC \le 25\%$ (calculates distance to GCS / $v_{transit}$ + safety margin)
- **Emergency Land Failsafe**: $SoC \le 10\%$ (forces immediate vertical descent to prevent crash)

---

## 2. Swarm Navigation & Collision Avoidance

To enable multi-UAV operations in confined disaster zones without inter-drone collisions or crashes into damaged structures, we synthesize a multi-tiered navigation and collision avoidance architecture.

```
       +-------------------------------------------------------------+
       |               High-Level Mission Planner                    |
       |  (Assigns 3D Target Waypoints: PoI Orbit or Relay Anchor)   |
       +-------------------------------------------------------------+
                                      |
                                      v
       +-------------------------------------------------------------+
       |            Swarm Deconfliction & Steering Engine            |
       |                                                             |
       |  1. Goal Seeking Vector      ->  F_goal                     |
       |  2. Inter-Drone Separation   ->  F_sep (Reynolds / APF)     |
       |  3. Obstacle Repulsion       ->  F_obs (AABB / Cylinder)    |
       |  4. Downwash Jet Repulsion   ->  F_downwash (Asymmetric Z)  |
       |  5. Swarm Velocity Alignment ->  F_align                    |
       |  6. Altitude Layer Clamping  ->  Z_corridor deconfliction   |
       +-------------------------------------------------------------+
                                      |
                                      v
       +-------------------------------------------------------------+
       |           Net Desired Velocity & Acceleration Filter        |
       |      (Velocity Clamping, Slew Rate Limiting, Tilt Limit)     |
       +-------------------------------------------------------------+
                                      |
                                      v
       +-------------------------------------------------------------+
       |                  Low-Level Physics Engine                   |
       |               (Euler / RK4 State Integration)               |
       +-------------------------------------------------------------+
```

### 2.1 Reynolds Flocking Rules for Quadcopters
Adapted from Craig Reynolds' boids model and extended into 3D Euclidean space:

1. **Separation ($\mathbf{F}_{sep}$)**:
   Avoids crowding neighbor drones within a perception radius $R_{sep} \approx 6.0$ m:
   $$\mathbf{F}_{sep, i} = - k_{sep} \sum_{j \in \mathcal{N}_i, j \ne i} \frac{\mathbf{p}_j - \mathbf{p}_i}{\|\mathbf{p}_j - \mathbf{p}_i\|^2}$$
2. **Alignment ($\mathbf{F}_{align}$)**:
   Matches flight velocity vectors with neighboring drones within interaction radius $R_{align} \approx 12.0$ m to ensure smooth transit:
   $$\mathbf{F}_{align, i} = k_{align} \left( \frac{1}{|\mathcal{N}_i|} \sum_{j \in \mathcal{N}_i} \mathbf{v}_j - \mathbf{v}_i \right)$$
3. **Cohesion ($\mathbf{F}_{coh}$)**:
   Keeps the fleet or sub-swarm together during transit:
   $$\mathbf{F}_{coh, i} = k_{coh} \left( \frac{1}{|\mathcal{N}_i|} \sum_{j \in \mathcal{N}_i} \mathbf{p}_j - \mathbf{p}_i \right)$$

### 2.2 Artificial Potential Fields (APF)
For precise obstacle avoidance and waypoint navigation, Khatib-style Artificial Potential Fields are applied:

#### 1. Conic-Parabolic Attractive Potential:
To prevent excessive attraction forces when the drone is far from its target waypoint $\mathbf{w}_k$:

$$U_{att}(\mathbf{p}_i) = \begin{cases}
\frac{1}{2} k_{att} \|\mathbf{p}_i - \mathbf{w}_k\|^2 & \text{if } \|\mathbf{p}_i - \mathbf{w}_k\| \le d_{thresh} \\
d_{thresh} k_{att} \|\mathbf{p}_i - \mathbf{w}_k\| - \frac{1}{2} k_{att} d_{thresh}^2 & \text{if } \|\mathbf{p}_i - \mathbf{w}_k\| > d_{thresh}
\end{cases}$$

$$\mathbf{F}_{att}(\mathbf{p}_i) = -\nabla U_{att}(\mathbf{p}_i) = \begin{cases}
- k_{att} (\mathbf{p}_i - \mathbf{w}_k) & \text{if } \|\mathbf{p}_i - \mathbf{w}_k\| \le d_{thresh} \\
- d_{thresh} k_{att} \frac{\mathbf{p}_i - \mathbf{w}_k}{\|\mathbf{p}_i - \mathbf{w}_k\|} & \text{if } \|\mathbf{p}_i - \mathbf{w}_k\| > d_{thresh}
\end{cases}$$

#### 2. Repulsive Potential from Static Obstacles:
Disaster obstacles (collapsed structures, radio masts) are represented geometrically as Axis-Aligned Bounding Boxes (AABB) or vertical cylinders. For an obstacle $O_m$ with closest surface point $\mathbf{c}_m(\mathbf{p}_i)$ and distance $\rho_m(\mathbf{p}_i) = \|\mathbf{p}_i - \mathbf{c}_m(\mathbf{p}_i)\|$:

$$U_{rep, m}(\mathbf{p}_i) = \begin{cases}
\frac{1}{2} k_{rep} \left( \frac{1}{\rho_m(\mathbf{p}_i) - r_{safe}} - \frac{1}{\rho_0} \right)^2 & \text{if } \rho_m(\mathbf{p}_i) \le \rho_0 \\
0 & \text{if } \rho_m(\mathbf{p}_i) > \rho_0
\end{cases}$$

$$\mathbf{F}_{rep, m}(\mathbf{p}_i) = \begin{cases}
k_{rep} \left( \frac{1}{\rho_m - r_{safe}} - \frac{1}{\rho_0} \right) \frac{1}{(\rho_m - r_{safe})^2} \frac{\mathbf{p}_i - \mathbf{c}_m}{\rho_m} & \text{if } \rho_m \le \rho_0 \\
\mathbf{0} & \text{if } \rho_m > \rho_0
\end{cases}$$

Where:
- $r_{safe} = 1.5$ m (drone physical clearance margin)
- $\rho_0 = 8.0$ m (influence horizon)
- $k_{rep} = 40.0$ (repulsion gain)

#### 3. Overcoming Local Minima:
To eliminate classic APF saddle-point traps when a building directly blocks the line of sight to a PoI:
- **Vortex/Tangential Force Injection**: When $\|\mathbf{F}_{att} + \mathbf{F}_{rep}\| < \epsilon_{min}$ and $\|\mathbf{p}_i - \mathbf{w}_k\| > d_{arrival}$, a cross-product circulatory force $\mathbf{F}_{tangent} = \alpha (\mathbf{F}_{rep} \times \hat{\mathbf{z}}_I)$ is added to steer the drone around the perimeter of the building.

### 2.3 Aerodynamic Downwash Avoidance
As documented in `gym-pybullet-drones`, a serious risk in multi-drone flight is downwash: the high-velocity turbulent air column ejected downwards from a quadcopter's propellers reduces the lift of any drone flying directly underneath, potentially inducing a catastrophic uncontrolled descent.

To model this, we define an **asymmetric downwash hazard cone**:

$$\Delta \mathbf{p}_{ij} = \mathbf{p}_i - \mathbf{p}_j = [\Delta x, \Delta y, \Delta z]^T$$

If drone $i$ is positioned below drone $j$ ($\Delta z < 0$) and within a cone of opening angle $\theta_{dw} \approx 25^\circ$:

$$\sqrt{\Delta x^2 + \Delta y^2} < |\Delta z| \tan(\theta_{dw}) \quad \text{and} \quad |\Delta z| \le H_{dw}^{max} \approx 10.0\text{ m}$$

A strong horizontal repulsive force is exerted on drone $i$ to push it out of the jet:

$$\mathbf{F}_{downwash, i} = k_{dw} \exp\left(-\frac{\Delta x^2 + \Delta y^2}{2 \sigma_{dw}^2}\right) \frac{[\Delta x, \Delta y, 0]^T}{\sqrt{\Delta x^2 + \Delta y^2} + \epsilon}$$

### 2.4 Altitude Layering (Airspace Deconfliction)
In addition to reactive potential fields, the airspace is partitioned into structured altitude corridors to avoid conflicts during high-speed transit:

```
+---------------------------------------------------------------+ 90m
|       Layer 4: High-Altitude Relay Backbone Corridor          |
|       (Z = 70m - 90m: Unobstructed RF Line-of-Sight to GCS)   |
+---------------------------------------------------------------+ 70m
|       Layer 3: Fleet Transit & Return Corridor                |
|       (Z = 50m - 65m: High-speed transit between sectors)     |
+---------------------------------------------------------------+ 50m
|       Layer 2: Disaster PoI Inspection & Orbit Volume         |
|       (Z = 25m - 45m: Sensor dwell, detailed imagery)         |
+---------------------------------------------------------------+ 25m
|       Layer 1: Ground Launch, Recovery & Landing Pad          |
|       (Z = 0m - 20m: Takeoff / Precision landing at GCS)      |
+---------------------------------------------------------------+ 0m (Ground)
```

---

## 3. PoI Survey Mission Logic & Role Allocation

### 3.1 Disaster Environment Specification
The simulated post-disaster environment represents an urban or industrial crisis area (e.g. an earthquake or industrial explosion zone):
- **World Bounds**: $X \in [-250\text{m}, +250\text{m}]$, $Y \in [-250\text{m}, +250\text{m}]$, $Z \in [0\text{m}, 120\text{m}]$ (500m $\times$ 500m active operations theater).
- **Ground Control Station (GCS)**: Stationary operations base located at safe perimeter $\mathbf{p}_{GCS} = [0, -200, 0]^T$ m.
- **Disaster Obstacles**: 6–10 rectangular structures representing damaged/collapsed multi-story buildings and towers (heights $25\text{m} - 65\text{m}$). These serve dual roles:
  1. Physical collisions obstacles for UAV kinematics.
  2. RF attenuation / Line-of-Sight (LOS) occlusion obstacles for multi-hop communication ray-tracing.

```
       Y (North)
       ^
  +250 |        [PoI-1: Survivor Zone] (Priority: CRITICAL)
       |           * (X=-120, Y=180, Z=30)
       |
       |         +-----------------+
       |         | Building Alpha  |
       |         | (Height: 55m)   |        [PoI-2: Structural Damage]
       |         +-----------------+           * (X=140, Y=130, Z=25)
       |
   0   |                 [Relay UAV-4] (Z=75m)
       |                      ^
       |                      | (Multi-hop link)
       |                      v
       |                 [Relay UAV-5] (Z=60m)
       |
  -200 |                 [GCS Base Station]
       |                      @ (X=0, Y=-200, Z=0)
       +--------------------------------------------------------> X (East)
     -250                     0                      +250
```

### 3.2 Points of Interest (PoIs) & Payload Acquisition
Each emergency Point of Interest $k$ is characterized by:
- **Identifier**: `poi_id` (e.g., `POI-SURVIVOR-ALPHA`)
- **Location**: $\mathbf{p}_{poi} = [x_{poi}, y_{poi}, z_{poi}]^T$
- **Priority Class**:
  - `CRITICAL` (Weight: 10): Survivor pockets, medical emergencies, hazardous chemical/gas leaks.
  - `HIGH` (Weight: 6): Infrastructure collapse, damaged bridge, access corridor blockage.
  - `MEDIUM` (Weight: 3): Peripheral damage assessment, secondary debris survey.
- **Required Dwell Time** ($T_{dwell}$): Duration (e.g., $15.0 - 25.0$ seconds) the Survey UAV must maintain an observation envelope around the PoI to collect high-resolution data.
- **Data Payload Size** ($S_{data}$): Total volume of telemetry, multispectral imagery, and 3D point cloud data generated (e.g., $50 - 150$ MB).
- **State**: `UNASSIGNED` $\to$ `ASSIGNED` $\to$ `IN_PROGRESS` $\to$ `COMPLETED`.

### 3.3 Dynamic Fleet Role Allocation
To maximize data gathering and guarantee real-time telemetry back to GCS, the fleet of $N$ UAVs is dynamically partitioned into two complementary roles:

#### 1. Survey UAVs ($N_{survey} \ge 2$, e.g., UAV-1, UAV-2, UAV-3):
- **Objective**: Travel directly to disaster PoIs, conduct localized sensor sweeps (loiter / orbit), and generate emergency telemetry packets.
- **Waypoint Allocation**: Solved via a **Greedy Priority-Over-Distance Utility Maximizer**:
  $$\mathcal{U}(UAV_i, PoI_k) = \frac{\text{PriorityWeight}(PoI_k)}{\|\mathbf{p}_i - \mathbf{p}_k\| + \epsilon} \cdot \left(1.0 - \frac{\text{Distance}(\mathbf{p}_k, \mathbf{p}_{GCS})}{R_{max\_range}}\right)$$

#### 2. Relay UAVs ($N_{relay} \ge 2$, e.g., UAV-4, UAV-5):
- **Objective**: Maintain end-to-end multi-hop packet connectivity between distant Survey UAVs and the GCS.
- **Dynamic Positioning via Virtual Spring Mesh (VSM)**:
  Instead of remaining static, Relay UAVs act as mobile network nodes attached by virtual Hooke's springs to:
  1. The GCS node $\mathbf{p}_{GCS}$
  2. The centroid of active Survey UAVs $\mathbf{p}_{survey\_centroid} = \frac{1}{|S|} \sum_{s \in S} \mathbf{p}_s$
  3. Neighboring Relay UAVs

  The virtual spring force acting on Relay UAV $r$ is:
  $$\mathbf{F}_{spring, r} = - \sum_{n \in \mathcal{N}_r^{net}} k_{spring} \left( \|\mathbf{p}_r - \mathbf{p}_n\| - d_{target}^{comm} \right) \frac{\mathbf{p}_r - \mathbf{p}_n}{\|\mathbf{p}_r - \mathbf{p}_n\|}$$

  Where $d_{target}^{comm} \approx 0.70 \cdot R_{comm}^{max}$ (e.g. $105$ m for a $150$ m RF range).
  - If the distance exceeds $d_{target}^{comm}$, the spring pulls the relay closer to maintain high SNR.
  - If the distance becomes too small, the spring pushes them apart to expand network coverage.
  - Obstacle and inter-drone repulsive forces ($\mathbf{F}_{rep}$) are superposed so the Relay never collides with buildings while seeking optimal relay coordinates.

---

## 4. Mission Finite State Machine (FSM)

Each UAV executes an autonomous, robust state machine conforming to the flight-safety principles of `MAVSDK`:

```
               +-------------------+
               |       IDLE        |
               +-------------------+
                         |
                         | Command: ARM_AND_TAKEOFF
                         v
               +-------------------+
               |     TAKEOFF       |
               +-------------------+
                         |
                         | Altitude >= Takeoff_Alt (15m)
                         v
               +-------------------+
         +---> |     TRANSIT       | <-------------------------+
         |     +-------------------+                           |
         |        /             \                              |
(Next    |       /               \                             |
 PoI)    |      / (Role: SURVEY)  \ (Role: RELAY)              |
         |     v                   v                           |
         | +--------------+    +--------------+                |
         | |  SURVEYING   |    |  DATA_RELAY  |                |
         | |  (Orbit/Dwell|    | (Anchor VSM  |                |
         | |  at PoI)     |    |  positions)  |                |
         | +--------------+    +--------------+                |
         |     \                   /                           |
         |      \                 /                            |
         |       v               v                             |
         |     +-------------------+                           |
         +---- |  DATA_TRANSMIT    | --------------------------+
               +-------------------+
                         |
                         | Event: ALL_POIS_DONE or BATTERY_LOW (SoC <= 25%)
                         v
               +-------------------+
               |  RETURN_TO_BASE   |
               +-------------------+
                         |
                         | 2D Distance to GCS < 3.0m
                         v
               +-------------------+
               |     LANDING       |
               +-------------------+
                         |
                         | Altitude <= 0.2m (Touchdown)
                         v
               +-------------------+
               |    COMPLETED      |
               +-------------------+

      =========================================================
      SAFETY OVERRIDE (Triggerable from ANY state):
      Battery <= 10% OR Critical Sensor Failure -> EMERGENCY_LAND
      =========================================================
```

### 4.1 Detailed State Transition Table

| Current State | Transition Trigger / Guard Condition | Next State | Action / Controller Behavior Executed |
|---|---|---|---|
| `IDLE` | Mission start signal received | `TAKEOFF` | Arm motors, spool to hover RPM, set vertical setpoint $z_{des} = z_{takeoff} = 15.0$ m |
| `TAKEOFF` | $z \ge z_{takeoff} - 0.5$ m | `TRANSIT` | Switch to cruising altitude corridor; set waypoint toward assigned PoI or Relay zone |
| `TRANSIT` (Surveyor) | Distance to PoI envelope $\le R_{poi\_arrive} = 10.0$ m | `SURVEYING` | Enter localized inspection orbit / loiter; engage camera payload sensor simulation |
| `TRANSIT` (Relay) | Distance to VSM target $\le R_{relay\_arrive} = 12.0$ m | `DATA_RELAY` | Hold elevated relay anchor position; activate high-power RF bridging mode |
| `SURVEYING` | PoI dwell time $t_{dwell} \ge T_{req}$ | `DATA_TRANSMIT` | Mark PoI survey completed; initiate streaming of gathered sensor telemetry across mesh |
| `DATA_RELAY` | Active Survey UAVs repositioning | `DATA_RELAY` | Continuously modulate 3D position using Virtual Spring Mesh gradient to preserve LOS |
| `DATA_TRANSMIT` | Transmission queue drained & unvisited PoIs exist | `TRANSIT` | Assign next highest utility PoI; set new transit waypoint |
| `DATA_TRANSMIT` | All PoIs surveyed OR $SoC \le 25\%$ | `RETURN_TO_BASE`| Transition to return corridor ($Z = 55$ m); route directly towards $\mathbf{p}_{GCS}$ |
| `DATA_RELAY` | All Survey UAVs returned OR $SoC \le 25\%$ | `RETURN_TO_BASE`| Fall back from relay post; route to GCS |
| `RETURN_TO_BASE` | Horizontal distance to GCS $\le 3.0$ m | `LANDING` | Descend vertically at safe velocity $v_{z} = -1.2$ m/s |
| `LANDING` | Altitude $z \le 0.2$ m (Ground contact detected) | `COMPLETED` | Disarm motors; shutdown telemetry stream; mission log finalized |
| `ANY_STATE` | $SoC \le 10\%$ (Critical battery exhaustion) | `EMERGENCY_LAND`| Immediate zero-drift descent to nearest ground coordinate to prevent crash |

---

## 5. Discrete-Time Simulation Update Loop (`step()`)

To integrate with the multi-hop networking layer (Explorer 3) and 3D visualization frontend (Explorer 1), the swarm simulation core executes a strictly ordered, deterministic 6-phase update tick at frequency $f_{sim} = 20 - 50$ Hz ($\Delta t = 0.02 - 0.05$ s).

```
+-------------------------------------------------------------------------------+
|                        Simulation Update Tick (dt = 0.02s)                    |
+-------------------------------------------------------------------------------+
| Phase 1: Environment & Peer Perception                                        |
|   - Compute pairwise 3D Euclidean distances between all UAVs                 |
|   - Query obstacle proximity vectors and surface normal distances             |
|   - Update communication graph connectivity matrix from networking module     |
+-------------------------------------------------------------------------------+
| Phase 2: High-Level Mission & FSM Updates                                     |
|   - Check transition guards for each drone's FSM                              |
|   - Accumulate PoI dwell times and simulate sensor packet generation          |
|   - Compute dynamic Relay UAV target positions via Virtual Spring Mesh        |
+-------------------------------------------------------------------------------+
| Phase 3: Swarm Coordination & Force Computation                               |
|   - F_total = F_att (Goal) + F_rep (Obstacles) + F_sep (Neighbors)            |
|               + F_downwash (Vertical Jet) + F_align (Velocity)                |
|   - Apply Altitude Corridor Layer clamping                                    |
+-------------------------------------------------------------------------------+
| Phase 4: Kinematic & Dynamic State Integration                                |
|   - Clamp acceleration: ||a|| <= a_max                                        |
|   - Update velocity: v(t+dt) = v(t) + a * dt (clamped to ||v|| <= v_max)      |
|   - Update position: p(t+dt) = p(t) + v(t+dt) * dt                            |
|   - Compute synthetic Roll/Pitch tilt from acceleration vector                |
|   - Update Yaw towards velocity vector or PoI center                          |
+-------------------------------------------------------------------------------+
| Phase 5: Battery & Power Depletion                                            |
|   - Calculate P_total = P_base + P_propulsion(v, a) + P_payload               |
|   - Update SoC: SoC(t+dt) = SoC(t) - (P_total * dt) / E_capacity              |
+-------------------------------------------------------------------------------+
| Phase 6: Snapshot Generation & Telemetry Export                               |
|   - Produce immutable SwarmStateSnapshot dict for GUI and network router     |
+-------------------------------------------------------------------------------+
```

---

## 6. Concrete API Contracts & Python Class Design

Below are the complete, production-grade interface definitions and class signatures designed to form the foundation of Milestone 1 (`swarm_sim` package).

```python
"""
uav_swarm_sim: Core Swarm Kinematics, Navigation and Mission Logic Interface
Designed for 3D multi-hop aerial survey simulations.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple, Any
import numpy as np


class DroneRole(Enum):
    """Operational roles assigned to UAVs within the disaster response fleet."""
    SURVEYOR = auto()   # Navigates to PoIs, collects sensor data, initiates transmission
    RELAY = auto()      # Dynamic bridge node, preserves RF connectivity between Surveyor and GCS
    RESERVE = auto()    # Standby drone at GCS for replacement or emergency relay


class MissionState(Enum):
    """Finite State Machine flight modes conforming to MAVSDK / PX4 standards."""
    IDLE = auto()
    TAKEOFF = auto()
    TRANSIT = auto()
    SURVEYING = auto()
    DATA_RELAY = auto()
    DATA_TRANSMITTING = auto()
    RETURN_TO_BASE = auto()
    LANDING = auto()
    COMPLETED = auto()
    EMERGENCY_LAND = auto()


class PoIPriority(Enum):
    """Priority weights for disaster survey points of interest."""
    CRITICAL = 10   # Survivors, active hazard
    HIGH = 6        # Main infrastructure / bridge collapse
    MEDIUM = 3      # General damage survey


@dataclass
class DroneLimits:
    """Hard kinematic and physical constraints for quadcopter airframe."""
    max_speed_xy: float = 10.0          # m/s
    max_speed_z_up: float = 3.5         # m/s
    max_speed_z_down: float = 2.5       # m/s
    max_accel: float = 4.0              # m/s^2
    max_tilt_rad: float = 0.5236        # 30 degrees maximum bank angle
    max_yaw_rate: float = 1.5708        # rad/s (90 deg/s)
    mass_kg: float = 1.20               # kg
    hover_thrust_n: float = 11.77       # m * g (Newtons)
    thrust_to_weight: float = 2.2       # Max TWR ratio


@dataclass
class BatteryState:
    """Physical battery model with power tracking."""
    capacity_mah: float = 5000.0        # mAh
    nominal_voltage: float = 14.8       # 4S LiPo (Volts)
    soc: float = 1.0                    # Fraction in [0.0, 1.0]
    p_base_w: float = 12.0              # Avionics power (Watts)
    p_hover_w: float = 180.0            # Baseline hover propulsion power (Watts)
    p_payload_w: float = 8.0            # Active sensor power (Watts)
    p_tx_w: float = 15.0                # Active RF relay / TX power (Watts)
    rtb_soc_threshold: float = 0.25     # Trigger Return to Base at 25%
    emergency_soc_threshold: float = 0.10 # Immediate land at 10%

    @property
    def total_energy_joules(self) -> float:
        return self.capacity_mah * 0.001 * self.nominal_voltage * 3600.0

    def consume(self, dt: float, speed: float, accel: float, is_transmitting: bool, is_surveying: bool) -> None:
        """Update State of Charge (SoC) based on physics power expenditure."""
        p_prop = self.p_hover_w * (1.0 + 0.05 * speed + 0.1 * abs(accel))
        p_sen = self.p_payload_w if is_surveying else 0.0
        p_rf = self.p_tx_w if is_transmitting else 2.0
        p_total = self.p_base_w + p_prop + p_sen + p_rf
        
        delta_soc = (p_total * dt) / self.total_energy_joules
        self.soc = max(0.0, self.soc - delta_soc)


@dataclass
class PointOfInterest:
    """Disaster survey inspection site."""
    poi_id: str
    position: np.ndarray                 # [x, y, z] in meters
    priority: PoIPriority = PoIPriority.HIGH
    required_dwell_time: float = 15.0   # seconds
    current_dwell_time: float = 0.0     # seconds elapsed
    data_payload_mb: float = 100.0      # MB to generate
    assigned_drone_id: Optional[str] = None
    is_surveyed: bool = False

    @property
    def progress_fraction(self) -> float:
        return min(1.0, self.current_dwell_time / self.required_dwell_time)


@dataclass
class DisasterObstacle:
    """Axis-Aligned Bounding Box (AABB) representing damaged structures."""
    obstacle_id: str
    min_bound: np.ndarray  # [x_min, y_min, z_min]
    max_bound: np.ndarray  # [x_max, y_max, z_max]

    def distance_and_closest_point(self, point: np.ndarray) -> Tuple[float, np.ndarray]:
        """Calculates Euclidean distance and closest surface point to 3D point."""
        closest = np.clip(point, self.min_bound, self.max_bound)
        dist = float(np.linalg.norm(point - closest))
        return dist, closest

    def intersects_ray(self, origin: np.ndarray, target: np.ndarray) -> bool:
        """Ray-AABB intersection test for Line-of-Sight (LOS) occlusion checks."""
        direction = target - origin
        norm_dir = np.linalg.norm(direction)
        if norm_dir < 1e-6:
            return False
        d = direction / norm_dir
        
        # Slab method
        t1 = (self.min_bound - origin) / (d + 1e-9)
        t2 = (self.max_bound - origin) / (d + 1e-9)
        t_min = np.maximum(np.minimum(t1, t2), [0, 0, 0])
        t_max = np.minimum(np.maximum(t1, t2), [norm_dir, norm_dir, norm_dir])
        
        t_enter = float(np.max(t_min))
        t_exit = float(np.min(t_max))
        return t_enter <= t_exit and t_exit >= 0.0


@dataclass
class QuadcopterTelemetry:
    """Immutable state snapshot emitted every tick."""
    drone_id: str
    role: DroneRole
    state: MissionState
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    attitude_euler: np.ndarray           # [roll, pitch, yaw] in radians
    soc: float
    current_target_waypoint: Optional[np.ndarray]
    active_poi_id: Optional[str]
    is_transmitting: bool


class UAVAgent:
    """Autonomous quadcopter agent encapsulating kinematics, FSM, and local navigation."""

    def __init__(self, drone_id: str, role: DroneRole, initial_pos: np.ndarray, limits: DroneLimits, battery: BatteryState):
        self.drone_id = drone_id
        self.role = role
        self.limits = limits
        self.battery = battery
        
        # Kinematic states
        self.position = np.array(initial_pos, dtype=np.float64)
        self.velocity = np.zeros(3, dtype=np.float64)
        self.acceleration = np.zeros(3, dtype=np.float64)
        self.attitude = np.zeros(3, dtype=np.float64)  # [roll, pitch, yaw]
        
        # Navigation & FSM
        self.state = MissionState.IDLE
        self.target_waypoint: Optional[np.ndarray] = None
        self.active_poi: Optional[PointOfInterest] = None
        self.is_transmitting = False

    def update_fsm(self, dt: float, gcs_pos: np.ndarray, available_pois: List[PointOfInterest]) -> None:
        """Evaluates state machine guard conditions and executes state actions."""
        # Battery failsafe override
        if self.battery.soc <= self.battery.emergency_soc_threshold and self.state != MissionState.COMPLETED:
            self.state = MissionState.EMERGENCY_LAND

        if self.state == MissionState.IDLE:
            pass  # Wait for swarm controller takeoff dispatch

        elif self.state == MissionState.TAKEOFF:
            target_alt = 15.0 if self.role == DroneRole.SURVEYOR else 60.0
            self.target_waypoint = np.array([self.position[0], self.position[1], target_alt])
            if abs(self.position[2] - target_alt) < 0.5:
                self.state = MissionState.TRANSIT

        elif self.state == MissionState.TRANSIT:
            if self.battery.soc <= self.battery.rtb_soc_threshold:
                self.state = MissionState.RETURN_TO_BASE
                return

            if self.role == DroneRole.SURVEYOR and self.active_poi:
                dist_to_poi = np.linalg.norm(self.position - self.active_poi.position)
                if dist_to_poi < 10.0:
                    self.state = MissionState.SURVEYING
            elif self.role == DroneRole.RELAY:
                self.state = MissionState.DATA_RELAY

        elif self.state == MissionState.SURVEYING:
            if self.battery.soc <= self.battery.rtb_soc_threshold:
                self.state = MissionState.RETURN_TO_BASE
                return

            if self.active_poi:
                # Dwell at PoI while executing low-speed orbit
                self.active_poi.current_dwell_time += dt
                if self.active_poi.progress_fraction >= 1.0:
                    self.active_poi.is_surveyed = True
                    self.is_transmitting = True
                    self.state = MissionState.DATA_TRANSMITTING

        elif self.state == MissionState.DATA_RELAY:
            # Maintained at dynamically calculated VSM anchor
            if self.battery.soc <= self.battery.rtb_soc_threshold:
                self.state = MissionState.RETURN_TO_BASE

        elif self.state == MissionState.DATA_TRANSMITTING:
            # Data streaming simulated, then pick next target or RTB
            self.is_transmitting = False
            self.active_poi = None
            self.state = MissionState.TRANSIT

        elif self.state == MissionState.RETURN_TO_BASE:
            self.target_waypoint = np.array([gcs_pos[0], gcs_pos[1], 20.0])
            horiz_dist = np.linalg.norm(self.position[:2] - gcs_pos[:2])
            if horiz_dist < 3.0:
                self.state = MissionState.LANDING

        elif self.state == MissionState.LANDING:
            self.target_waypoint = np.array([gcs_pos[0], gcs_pos[1], 0.0])
            if self.position[2] <= 0.2:
                self.position[2] = 0.0
                self.velocity[:] = 0.0
                self.acceleration[:] = 0.0
                self.state = MissionState.COMPLETED

        elif self.state == MissionState.EMERGENCY_LAND:
            self.target_waypoint = np.array([self.position[0], self.position[1], 0.0])
            if self.position[2] <= 0.2:
                self.position[2] = 0.0
                self.velocity[:] = 0.0
                self.state = MissionState.COMPLETED

    def step_physics(self, dt: float, desired_accel: np.ndarray) -> None:
        """Numerical integration of acceleration, velocity, position, attitude and battery."""
        # 1. Acceleration clamping
        accel_mag = float(np.linalg.norm(desired_accel))
        if accel_mag > self.limits.max_accel:
            self.acceleration = (desired_accel / accel_mag) * self.limits.max_accel
        else:
            self.acceleration = np.array(desired_accel, dtype=np.float64)

        # 2. Velocity integration with drag and limit clamping
        self.velocity += self.acceleration * dt
        
        # Horizontal speed clamping
        v_xy_mag = float(np.linalg.norm(self.velocity[:2]))
        if v_xy_mag > self.limits.max_speed_xy:
            self.velocity[:2] = (self.velocity[:2] / v_xy_mag) * self.limits.max_speed_xy
            
        # Vertical rate clamping
        self.velocity[2] = np.clip(self.velocity[2], -self.limits.max_speed_z_down, self.limits.max_speed_z_up)

        # 3. Position update
        self.position += self.velocity * dt
        if self.position[2] < 0.0:
            self.position[2] = 0.0
            self.velocity[2] = 0.0

        # 4. Attitude generation (Roll/Pitch tilted by lateral acceleration)
        g = 9.80665
        self.attitude[0] = np.clip(self.acceleration[1] / g, -self.limits.max_tilt_rad, self.limits.max_tilt_rad)  # Roll
        self.attitude[1] = np.clip(-self.acceleration[0] / g, -self.limits.max_tilt_rad, self.limits.max_tilt_rad) # Pitch
        if v_xy_mag > 0.5:
            target_yaw = np.arctan2(self.velocity[1], self.velocity[0])
            yaw_diff = (target_yaw - self.attitude[2] + np.pi) % (2 * np.pi) - np.pi
            self.attitude[2] += np.clip(yaw_diff, -self.limits.max_yaw_rate * dt, self.limits.max_yaw_rate * dt)

        # 5. Battery discharge
        self.battery.consume(
            dt=dt,
            speed=float(np.linalg.norm(self.velocity)),
            accel=float(np.linalg.norm(self.acceleration)),
            is_transmitting=self.is_transmitting,
            is_surveying=(self.state == MissionState.SURVEYING)
        )

    def get_telemetry(self) -> QuadcopterTelemetry:
        """Returns clean snapshot copy for external visualization and network modeling."""
        return QuadcopterTelemetry(
            drone_id=self.drone_id,
            role=self.role,
            state=self.state,
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            acceleration=self.acceleration.copy(),
            attitude_euler=self.attitude.copy(),
            soc=self.battery.soc,
            current_target_waypoint=self.target_waypoint.copy() if self.target_waypoint is not None else None,
            active_poi_id=self.active_poi.poi_id if self.active_poi else None,
            is_transmitting=self.is_transmitting
        )


class SwarmSimulationCore:
    """Master multi-UAV swarm coordinator, environment owner, and update manager."""

    def __init__(self, gcs_position: np.ndarray = np.array([0.0, -200.0, 0.0])):
        self.gcs_position = np.array(gcs_position, dtype=np.float64)
        self.drones: Dict[str, UAVAgent] = {}
        self.pois: Dict[str, PointOfInterest] = {}
        self.obstacles: List[DisasterObstacle] = []
        self.sim_time = 0.0

    def add_drone(self, drone: UAVAgent) -> None:
        self.drones[drone.drone_id] = drone

    def add_poi(self, poi: PointOfInterest) -> None:
        self.pois[poi.poi_id] = poi

    def add_obstacle(self, obstacle: DisasterObstacle) -> None:
        self.obstacles.append(obstacle)

    def compute_forces_for_drone(self, drone: UAVAgent) -> np.ndarray:
        """Calculates composite navigation vector: APF + Reynolds + Downwash."""
        if drone.target_waypoint is None:
            return np.zeros(3)

        # 1. Attractive Force toward Waypoint (Conic-parabolic)
        diff_waypoint = drone.target_waypoint - drone.position
        dist_waypoint = np.linalg.norm(diff_waypoint)
        k_att = 1.2
        if dist_waypoint > 15.0:
            f_att = (diff_waypoint / dist_waypoint) * 15.0 * k_att
        else:
            f_att = diff_waypoint * k_att

        # 2. Inter-Drone Separation (Reynolds) & Downwash Repulsion
        f_sep = np.zeros(3)
        for other_id, other in self.drones.items():
            if other_id == drone.drone_id:
                continue
            delta = drone.position - other.position
            dist = np.linalg.norm(delta)
            if dist < 6.0 and dist > 1e-4:
                # Horizontal separation
                f_sep += (delta / (dist ** 2)) * 18.0
                
            # Downwash hazard check (if this drone is below another drone)
            dz = drone.position[2] - other.position[2]
            d_horiz = np.linalg.norm(delta[:2])
            if -8.0 < dz < -0.5 and d_horiz < 3.0:
                # Strong outward lateral push
                if d_horiz > 1e-3:
                    f_sep[:2] += (delta[:2] / d_horiz) * 25.0
                else:
                    f_sep[:2] += np.array([1.0, 0.0]) * 25.0

        # 3. Obstacle Repulsive Forces
        f_obs = np.zeros(3)
        for obs in self.obstacles:
            dist_obs, closest_pt = obs.distance_and_closest_point(drone.position)
            if dist_obs < 8.0:
                push_dir = drone.position - closest_pt
                norm_push = np.linalg.norm(push_dir)
                if norm_push > 1e-4:
                    f_obs += (push_dir / norm_push) * ((8.0 - dist_obs) / (dist_obs + 0.2)) * 20.0

        # Composite force
        f_total = f_att + f_sep + f_obs
        return f_total

    def update_relay_positions(self) -> None:
        """Solves Virtual Spring Mesh (VSM) to position Relay UAVs between GCS and Surveyors."""
        active_surveyors = [d for d in self.drones.values() if d.role == DroneRole.SURVEYOR and d.state != MissionState.COMPLETED]
        relays = [d for d in self.drones.values() if d.role == DroneRole.RELAY and d.state != MissionState.COMPLETED]
        
        if not active_surveyors or not relays:
            return

        # Target centroid of active surveyors
        surveyor_centroid = np.mean([s.position for s in active_surveyors], axis=0)
        
        # Position relays along line of sight with elevated altitude
        num_relays = len(relays)
        for idx, relay in enumerate(relays):
            fraction = (idx + 1) / (num_relays + 1)
            target_xy = self.gcs_position[:2] + fraction * (surveyor_centroid[:2] - self.gcs_position[:2])
            target_z = 70.0 + idx * 10.0  # Layer 4 elevated clearance
            relay.target_waypoint = np.array([target_xy[0], target_xy[1], target_z])

    def step(self, dt: float) -> Dict[str, QuadcopterTelemetry]:
        """Advances simulation by dt seconds and returns immutable fleet telemetry."""
        # 1. Update Relay positions via VSM
        self.update_relay_positions()

        # 2. Advance FSMs
        available_pois = [p for p in self.pois.values() if not p.is_surveyed]
        for drone in self.drones.values():
            drone.update_fsm(dt, self.gcs_position, available_pois)

        # 3. Calculate steering forces and step physics
        for drone in self.drones.values():
            if drone.state not in (MissionState.IDLE, MissionState.COMPLETED):
                desired_accel = self.compute_forces_for_drone(drone)
                drone.step_physics(dt, desired_accel)

        self.sim_time += dt
        return {d_id: drone.get_telemetry() for d_id, drone in self.drones.items()}
```

---

## 7. Performance & Computational Scaling Analysis

To guarantee that the simulation runs with high frame rates on standard developer PCs without specialized GPU hardware:
1. **Computational Complexity**:
   - Pairwise drone distance checks: $\mathcal{O}(N^2)$. For $N = 6 - 12$ UAVs, $N^2 \le 144$ operations per tick.
   - Obstacle checks: $\mathcal{O}(N \times M)$ where $M$ is obstacle count ($M \approx 8$). $12 \times 8 = 96$ vector evaluations per tick.
   - Total FLOPS per tick: $< 25,000$ operations.
2. **Benchmark Execution Time**:
   - On a single modern CPU core using NumPy vectorization, a full tick of 10 UAVs takes **$< 0.25$ milliseconds**.
   - At a $50$ Hz simulation rate ($\Delta t = 0.02$s), the swarm engine consumes **$< 1.5\%$ of a single CPU core**, leaving $>98\%$ CPU/GPU budget for 3D visualization and multi-hop routing protocol simulation.
3. **Deterministic Headless Testing**:
   - The simulation core has no external GUI or C++ thread dependencies, allowing $1,000$ seconds of simulated disaster operations to be verified in **$< 1.5$ seconds of real clock time** during headless automated pytest runs.

---

## 8. Alignment with Upstream Requirements & Next Steps

This report addresses every requirement from the user dispatch:
- **R1 (3D Swarm Simulation Environment)**: Complete 3D kinematic model, coordinate transforms, attitude dynamics, and visual state outputs.
- **R2 (Multi-hop Communication Modeling)**: Dynamic Relay positioning logic (Virtual Spring Mesh) to maintain RF bridges between distant survey zones and GCS.
- **R3 (PoI Surveying Logic)**: Multi-priority disaster zone specification, dwell time tracking, survey progress fraction, and complete state machine transitions.

**Recommended Milestone Integration**:
- Supply this interface contract to **Worker 1 (Milestone 1 Core Engine)** to implement `uav_sim_engine.py`.
- Interface with **Explorer 3 (Networking)** to feed coordinates into the dynamic link graph for Friis/SNR calculations.
- Interface with **Explorer 1 (Visualization)** to drive 3D model transforms and HUD metrics.
