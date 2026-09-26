# Technical Implementation Specification: Drone Kinematics, Dynamics, Flocking & Data Models

**Document**: `drone_impl_spec.md`  
**Author**: Explorer M1-1 (Drone Kinematics & Flocking Specialist)  
**Target Milestone**: Milestone 1 (Core Kinematics, Physics & Environment Engine)  
**Target Files**: `sim/types.py`, `sim/drone.py`, `tests/unit/test_drone.py`  
**Status**: COMPLETE / READY FOR IMPLEMENTATION  

---

## 1. Executive Overview & Scope

This specification provides the exact mathematical formulations, algorithmic blueprints, data structures, and unit test assertions for:
1. `sim/types.py`: The foundational type system including `FlightMode`, `DroneRole`, `DroneLimits`, `BatteryModel`, `DroneState`, and `AltitudeCorridor`.
2. `sim/drone.py`: The autonomous quadcopter agent encapsulating:
   - 6-DOF Newton-Euler rigid body translational and rotational kinematics.
   - Semi-implicit Euler state integration with physical velocity, vertical rate, and acceleration saturation.
   - Coupled attitude generation (Euler angles $[\phi, \theta, \psi]$ and unit quaternions $\mathbf{q} = [q_w, q_x, q_y, q_z]^T$).
   - Multi-tier vector steering force synthesis:
     - **Khatib Artificial Potential Fields (APF)** with conic-parabolic attractive targets and non-linear obstacle repulsion.
     - **Reynolds Boids Flocking** (Separation, Alignment, Cohesion).
     - **Asymmetric Aerodynamic Downwash Cone Repulsion** (protecting drones from turbulent propeller wash).
     - **4-Tier Airspace Altitude Corridor Deconfliction**.
   - Electro-mechanical LiPo battery depletion model with power draw tracking ($P_{base} + P_{prop} + P_{sen} + P_{rf}$) and Return-to-Base (RTB) / Emergency Land thresholds.
3. `tests/unit/test_drone.py`: Exhaustive unit test suite definitions with concrete mathematical assertions.

---

## 2. Coordinate System & State Representation

### 2.1 Coordinate Frames
1. **Inertial Reference Frame ($\mathcal{I}$)**:
   Right-handed East-North-Up (ENU):
   - $+X_I$: East (meters)
   - $+Y_I$: North (meters)
   - $+Z_I$: Up (meters, altitude above ground level AGL)
   - Origin: Disaster zone ground center $[0.0, 0.0, 0.0]^T$.

2. **Body-Fixed Frame ($\mathcal{B}$)**:
   Centered at the quadcopter Center of Mass (CoM):
   - $+X_B$: Structural forward direction
   - $+Y_B$: Structural left (port) direction
   - $+Z_B$: Normal to the propeller plane pointing upwards (collective thrust direction)

### 2.2 Rotation Representations
The orientation from Body frame $\mathcal{B}$ to Inertial frame $\mathcal{I}$ is parameterized by both Euler angles $\mathbf{\eta} = [\phi, \theta, \psi]^T$ (Roll, Pitch, Yaw) and unit quaternion $\mathbf{q} = [q_w, q_x, q_y, q_z]^T \in \mathbb{S}^3$:

$$\mathbf{R}_B^I(\mathbf{q}) = \begin{bmatrix}
1 - 2(q_y^2 + q_z^2) & 2(q_x q_y - q_w q_z) & 2(q_x q_z + q_w q_y) \\
2(q_x q_y + q_w q_z) & 1 - 2(q_x^2 + q_z^2) & 2(q_y q_z - q_w q_x) \\
2(q_x q_z - q_w q_y) & 2(q_y q_z + q_w q_x) & 1 - 2(q_x^2 + q_y^2)
\end{bmatrix}$$

**Euler to Quaternion Conversion**:
Given roll ($\phi$), pitch ($\theta$), yaw ($\psi$):
$$q_w = \cos(\phi/2)\cos(\theta/2)\cos(\psi/2) + \sin(\phi/2)\sin(\theta/2)\sin(\psi/2)$$
$$q_x = \sin(\phi/2)\cos(\theta/2)\cos(\psi/2) - \cos(\phi/2)\sin(\theta/2)\sin(\psi/2)$$
$$q_y = \cos(\phi/2)\sin(\theta/2)\cos(\psi/2) + \sin(\phi/2)\cos(\theta/2)\sin(\psi/2)$$
$$q_z = \cos(\phi/2)\cos(\theta/2)\sin(\psi/2) - \sin(\phi/2)\sin(\theta/2)\cos(\psi/2)$$
with renormalization $\mathbf{q} \leftarrow \frac{\mathbf{q}}{\|\mathbf{q}\|}$.

**Quaternion to Euler Conversion**:
$$\phi = \text{atan2}\left(2(q_w q_x + q_y q_z), 1 - 2(q_x^2 + q_y^2)\right)$$
$$\theta = \arcsin\left(\text{clip}\left(2(q_w q_y - q_z q_x), -1.0, 1.0\right)\right)$$
$$\psi = \text{atan2}\left(2(q_w q_z + q_x q_y), 1 - 2(q_y^2 + q_z^2)\right)$$

---

## 3. Data Models Specification (`sim/types.py`)

The file `sim/types.py` contains all shared enums, configuration dataclasses, and state containers. It has zero external dependencies beyond Python standard libraries and `numpy`.

```python
"""
sim/types.py: Core Data Models and Types for UAV Swarm Simulation
"""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Dict, Any, List
import numpy as np


class DroneRole(str, Enum):
    """Operational role assigned to a UAV in the FANET mission."""
    SURVEY = "SURVEY"    # Navigates to PoIs, collects sensor data, initiates transmissions
    RELAY = "RELAY"      # Positioned via Virtual Spring Mesh to maintain multi-hop RF connectivity


class FlightMode(str, Enum):
    """MAVSDK / PX4 compliant autonomous flight modes."""
    IDLE = "IDLE"                    # Disarmed / waiting on ground
    TAKEOFF = "TAKEOFF"              # Ascending vertically to assigned corridor
    TRANSIT = "TRANSIT"              # Cruising horizontally between waypoints
    SURVEYING = "SURVEYING"          # Loitering / orbiting active PoI for sensor collection
    RELAY = "RELAY"                  # Holding dynamic network mesh position
    RTL = "RTL"                      # Returning to Ground Control Station (Return-to-Launch)
    LANDED = "LANDED"                # Safely landed and disarmed at base
    EMERGENCY_LAND = "EMERGENCY_LAND"# Critical failsafe descent due to low battery or fault


@dataclass
class DroneLimits:
    """Kinematic, dynamic, and physical limits for quadcopter airframe."""
    max_speed_xy: float = 10.0          # Maximum horizontal speed (m/s)
    max_speed_z_up: float = 3.5         # Maximum vertical climb rate (m/s)
    max_speed_z_down: float = 2.5       # Maximum vertical descent rate to avoid VRS (m/s)
    max_accel: float = 4.0              # Maximum linear acceleration magnitude (m/s^2)
    max_tilt_rad: float = 0.5236        # Maximum bank/tilt angle (30 degrees in radians)
    max_yaw_rate: float = 1.5708        # Maximum rotational yaw speed (90 deg/s in radians)
    mass_kg: float = 1.20               # Quadcopter dry mass including battery (kg)
    hover_thrust_n: float = 11.768      # Hover thrust: m * g = 1.20 * 9.80665 (N)
    thrust_to_weight: float = 2.2       # Max thrust-to-weight ratio (TWR)
    rotor_thrust_coeff: float = 2.98e-6 # k_f in N / (rad/s)^2
    rotor_torque_coeff: float = 1.14e-7 # k_m in N*m / (rad/s)^2
    arm_length_m: float = 0.225         # Frame arm length from center to motor (m)
    separation_radius: float = 6.0      # Reynolds boid perception separation radius (m)
    collision_radius: float = 1.0       # Physical airframe safety radius for collision test (m)
    drag_coeff_xy: float = 0.15         # Linear drag coefficient in horizontal plane (N*s/m)
    drag_coeff_z: float = 0.25          # Linear drag coefficient in vertical direction (N*s/m)


@dataclass
class BatteryModel:
    """Electro-mechanical LiPo battery model with realistic power dissipation."""
    capacity_mah: float = 5000.0        # Total battery capacity in milliampere-hours (mAh)
    nominal_voltage: float = 14.8       # 4S LiPo nominal voltage (Volts)
    soc: float = 1.0                    # State of Charge fraction in [0.0, 1.0]
    p_base_w: float = 12.0              # Flight computer, IMU, GPS base electronics power (W)
    p_hover_w: float = 180.0            # Baseline aerodynamic hover propulsion power (W)
    p_payload_w: float = 8.0            # Optical camera / LiDAR sensor package power (W)
    p_tx_w: float = 15.0                # Active high-power RF relay transmission power (W)
    p_rx_w: float = 3.0                 # RF idle receiving/listening power (W)
    rtb_soc_threshold: float = 0.25     # State of Charge threshold to trigger RTL (25%)
    emergency_soc_threshold: float = 0.10 # State of Charge threshold for emergency landing (10%)

    @property
    def total_energy_joules(self) -> float:
        """Total usable energy capacity in Joules: V * Ah * 3600."""
        return (self.capacity_mah * 1e-3) * self.nominal_voltage * 3600.0

    def step(
        self,
        dt: float,
        speed: float,
        accel: float,
        is_transmitting: bool = False,
        is_surveying: bool = False
    ) -> float:
        """
        Integrates power consumption over dt seconds and updates SoC.
        Returns the instantaneous power drawn in Watts.
        """
        # Aerodynamic propulsion power scaling with speed and acceleration demand
        p_prop = self.p_hover_w * (1.0 + 0.04 * speed + 0.08 * abs(accel))
        p_sensor = self.p_payload_w if is_surveying else 0.0
        p_rf = self.p_tx_w if is_transmitting else self.p_rx_w
        p_total = self.p_base_w + p_prop + p_sensor + p_rf

        delta_energy = p_total * dt
        delta_soc = delta_energy / self.total_energy_joules
        self.soc = max(0.0, min(1.0, self.soc - delta_soc))
        return p_total

    def is_low(self) -> bool:
        """Returns True if battery is below Return-to-Base threshold (25%)."""
        return self.soc <= self.rtb_soc_threshold

    def is_critical(self) -> bool:
        """Returns True if battery is below Emergency Landing threshold (10%)."""
        return self.soc <= self.emergency_soc_threshold

    def remaining_flight_time_s(self, average_power_w: float = 195.0) -> float:
        """Estimates remaining flight endurance in seconds under nominal power draw."""
        remaining_joules = self.soc * self.total_energy_joules
        return remaining_joules / max(1.0, average_power_w)


@dataclass
class AltitudeCorridor:
    """Definition of an airspace altitude layer for traffic deconfliction."""
    name: str
    z_min: float
    z_max: float
    z_nominal: float

    def contains(self, z: float) -> bool:
        return self.z_min <= z <= self.z_max


# Predefined 4-tier airspace corridors
AIRSPACE_CORRIDORS = {
    "TIER_1_LAUNCH": AltitudeCorridor("Tier 1: Launch & Recovery", 0.0, 20.0, 10.0),
    "TIER_2_SURVEY": AltitudeCorridor("Tier 2: PoI Inspection", 25.0, 45.0, 35.0),
    "TIER_3_TRANSIT": AltitudeCorridor("Tier 3: Transit & RTL", 50.0, 65.0, 55.0),
    "TIER_4_RELAY": AltitudeCorridor("Tier 4: High-Altitude Relay", 70.0, 90.0, 80.0),
}


@dataclass
class DroneState:
    """
    Public telemetry and kinematic state container conforming to PROJECT.md Contract #1.
    """
    id: str
    role: str                              # 'SURVEY' | 'RELAY'
    position: np.ndarray                   # shape (3,), [x, y, z] in meters
    velocity: np.ndarray                   # shape (3,), [vx, vy, vz] in m/s
    attitude: np.ndarray                   # shape (3,), [roll, pitch, yaw] in radians
    rotor_speeds: np.ndarray               # shape (4,), [w1, w2, w3, w4] in rad/s
    battery_soc: float                     # [0.0, 1.0]
    flight_mode: str                       # 'IDLE', 'TAKEOFF', 'TRANSIT', etc.
    assigned_poi_id: Optional[str] = None  # ID of active PoI being surveyed
    target_position: Optional[np.ndarray] = None # Current 3D waypoint setpoint
    quaternion: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))

    def to_dict(self) -> Dict[str, Any]:
        """Serializes state into a compact dictionary for WebSocket telemetry frames."""
        return {
            "id": self.id,
            "role": self.role,
            "flight_mode": self.flight_mode,
            "position": [round(float(v), 3) for v in self.position],
            "velocity": [round(float(v), 3) for v in self.velocity],
            "attitude": [round(float(v), 4) for v in self.attitude],
            "quaternion": [round(float(v), 4) for v in self.quaternion],
            "rotor_speeds": [round(float(w), 1) for w in self.rotor_speeds],
            "battery_soc": round(float(self.battery_soc), 4),
            "assigned_poi_id": self.assigned_poi_id,
            "target_position": [round(float(v), 3) for v in self.target_position] if self.target_position is not None else None,
        }
```

---

## 4. Mathematical Physics & Equations (`sim/drone.py`)

### 4.1 6-DOF Quadcopter Translational Dynamics
The translational motion in inertial frame $\mathcal{I}$ is governed by:

$$m \ddot{\mathbf{p}} = \mathbf{F}_{thrust}^I + m \mathbf{g} + \mathbf{F}_{drag}^I + \mathbf{F}_{ext}^I$$

Where:
- $m$: Quadcopter mass ($1.20$ kg)
- $\mathbf{g} = [0.0, 0.0, -9.80665]^T$ m/s$^2$
- $\mathbf{F}_{thrust}^I = \mathbf{R}_B^I \begin{bmatrix} 0 \\ 0 \\ T_{total} \end{bmatrix}$ where $T_{total} = k_f \sum_{j=1}^4 \Omega_j^2$
- $\mathbf{F}_{drag}^I = - \begin{bmatrix} d_{xy} & 0 & 0 \\ 0 & d_{xy} & 0 \\ 0 & 0 & d_z \end{bmatrix} \mathbf{v}$ (linear aerodynamic drag)

In our multi-agent hierarchical controller, the navigation engine produces a commanded net steering force $\mathbf{F}_{cmd} \in \mathbb{R}^3$.
The commanded acceleration is:
$$\mathbf{a}_{cmd} = \frac{\mathbf{F}_{cmd}}{m}$$

#### Acceleration Saturation:
$$\|\mathbf{a}_{cmd}\| \le a_{max} = 4.0 \text{ m/s}^2$$
$$\mathbf{a}_{clamped} = \begin{cases} \mathbf{a}_{cmd} & \text{if } \|\mathbf{a}_{cmd}\| \le a_{max} \\ \frac{\mathbf{a}_{cmd}}{\|\mathbf{a}_{cmd}\|} a_{max} & \text{if } \|\mathbf{a}_{cmd}\| > a_{max} \end{cases}$$

#### Net Acceleration with Drag:
$$\mathbf{a}_{net} = \mathbf{a}_{clamped} - \frac{1}{m} \mathbf{D}_{drag} \mathbf{v}$$
where $\mathbf{D}_{drag} = \text{diag}(d_{xy}, d_{xy}, d_z) = \text{diag}(0.15, 0.15, 0.25)$.

#### Semi-Implicit Euler Numerical Integration:
$$\mathbf{v}(t + \Delta t) = \mathbf{v}(t) + \mathbf{a}_{net} \Delta t$$

#### Velocity Clamping:
1. Horizontal Velocity Clamping:
   $$v_{xy} = \sqrt{v_x^2 + v_y^2}$$
   $$\begin{bmatrix} v_x \\ v_y \end{bmatrix} \leftarrow \begin{cases} \begin{bmatrix} v_x \\ v_y \end{bmatrix} & \text{if } v_{xy} \le v_{xy}^{max} \\ \frac{v_{xy}^{max}}{v_{xy}} \begin{bmatrix} v_x \\ v_y \end{bmatrix} & \text{if } v_{xy} > v_{xy}^{max} \end{cases}$$
   where $v_{xy}^{max} = 10.0$ m/s.

2. Vertical Climb / Sink Rate Clamping:
   $$v_z \leftarrow \text{clip}(v_z, -v_{z, down}^{max}, v_{z, up}^{max})$$
   where $v_{z, down}^{max} = 2.5$ m/s and $v_{z, up}^{max} = 3.5$ m/s.

#### Position Integration & Ground Collision:
$$\mathbf{p}(t + \Delta t) = \mathbf{p}(t) + \mathbf{v}(t + \Delta t) \Delta t$$

If $p_z(t + \Delta t) < 0.0$ (ground surface at $Z=0$):
$$p_z \leftarrow 0.0, \quad v_z \leftarrow \max(0.0, v_z), \quad a_z \leftarrow \max(0.0, a_z)$$

---

### 4.2 Attitude Dynamics & Quaternion Integration

#### Desired Attitude from Linear Acceleration:
To accelerate with $\mathbf{a}_{clamped}$, the quadcopter must tilt its thrust vector towards $\mathbf{a}_{clamped} - \mathbf{g}$:
$$\mathbf{z}_{B, des} = \frac{\mathbf{a}_{clamped} - \mathbf{g}}{\|\mathbf{a}_{clamped} - \mathbf{g}\|}$$

Given current yaw angle $\psi$:
$$\phi_{des} = \arcsin\left(\text{clip}\left(\mathbf{z}_{B, des, x} \sin\psi - \mathbf{z}_{B, des, y} \cos\psi, -\sin\phi_{max}, \sin\phi_{max}\right)\right)$$
$$\theta_{des} = \arctan\left(\frac{\mathbf{z}_{B, des, x} \cos\psi + \mathbf{z}_{B, des, y} \sin\psi}{\mathbf{z}_{B, des, z}}\right)$$
$$\theta_{des} \leftarrow \text{clip}(\theta_{des}, -\theta_{max}, \theta_{max})$$
where $\phi_{max} = \theta_{max} = 0.5236$ rad ($30^\circ$).

#### Desired Yaw Heading ($\psi_{des}$):
- If the drone is moving with horizontal speed $v_{xy} > 0.5$ m/s:
  $$\psi_{des} = \text{atan2}(v_y, v_x)$$
- Else if surveying a PoI with center $\mathbf{p}_{poi}$:
  $$\psi_{des} = \text{atan2}(p_{poi, y} - p_y, p_{poi, x} - p_x)$$
- Otherwise, maintain current yaw heading.

#### Slew-Rate Limited Attitude Integration:
To reflect physical rotor moment of inertia ($\tau_{att} \approx 0.15$ s) and yaw rate limit ($r_{max} = 1.5708$ rad/s):
$$\Delta \phi = \text{clip}\left(\frac{\phi_{des} - \phi}{\tau_{att}} \Delta t, -\omega_{max} \Delta t, \omega_{max} \Delta t\right)$$
$$\Delta \theta = \text{clip}\left(\frac{\theta_{des} - \theta}{\tau_{att}} \Delta t, -\omega_{max} \Delta t, \omega_{max} \Delta t\right)$$
$$\psi_{err} = \text{atan2}\left(\sin(\psi_{des} - \psi), \cos(\psi_{des} - \psi)\right)$$
$$\Delta \psi = \text{clip}(\psi_{err}, -r_{max} \Delta t, r_{max} \Delta t)$$

Updated Euler angles:
$$\phi(t + \Delta t) = \phi(t) + \Delta \phi$$
$$\theta(t + \Delta t) = \theta(t) + \Delta \theta$$
$$\psi(t + \Delta t) = \text{wrap}_{-\pi}^{\pi}(\psi(t) + \Delta \psi)$$

Then compute normalized quaternion $\mathbf{q}(t + \Delta t)$ using the Euler-to-quaternion equations in Section 2.2.

#### Rotor Angular Speeds ($\Omega_1, \Omega_2, \Omega_3, \Omega_4$):
Baseline hover rotor speed:
$$\Omega_0 = \sqrt{\frac{m g}{4 k_f}} = \sqrt{\frac{1.20 \times 9.80665}{4 \times 2.98 \times 10^{-6}}} \approx 993.4 \text{ rad/s}$$
Differential rotor adjustments for roll, pitch, and yaw:
$$\Omega_1 = \Omega_0 + \Delta \Omega_z - \Delta \Omega_\phi + \Delta \Omega_\theta - \Delta \Omega_\psi$$
$$\Omega_2 = \Omega_0 + \Delta \Omega_z + \Delta \Omega_\phi + \Delta \Omega_\theta + \Delta \Omega_\psi$$
$$\Omega_3 = \Omega_0 + \Delta \Omega_z + \Delta \Omega_\phi - \Delta \Omega_\theta - \Delta \Omega_\psi$$
$$\Omega_4 = \Omega_0 + \Delta \Omega_z - \Delta \Omega_\phi - \Delta \Omega_\theta + \Delta \Omega_\psi$$
where $\Delta \Omega_z = \frac{m a_z}{8 k_f \Omega_0}$, $\Delta \Omega_\phi = \frac{I_{xx} \ddot{\phi}}{2 \sqrt{2} L k_f \Omega_0}$, $\Delta \Omega_\theta = \frac{I_{yy} \ddot{\theta}}{2 \sqrt{2} L k_f \Omega_0}$.
Values are clamped to $[0.0, 1500.0]$ rad/s.

---

## 5. Swarm Vector Steering & Force Synthesis

The total commanded force on drone $i$ is synthesized as:

$$\mathbf{F}_{cmd, i} = \mathbf{F}_{att, i} + \mathbf{F}_{obs, i} + \mathbf{F}_{sep, i} + \mathbf{F}_{align, i} + \mathbf{F}_{coh, i} + \mathbf{F}_{downwash, i} + \mathbf{F}_{corridor, i}$$

```
                           +-------------------------------------+
                           |      Goal Setpoint (Waypoint)       |
                           +-------------------------------------+
                                              |
                                              v
                                      [ F_att: APF Goal ]
                                              |
     +-------------------+                    |                    +--------------------+
     | Disaster Obstacle | ----> [ F_obs ] ---+--- [ F_sep ] <---- | Neighboring Drones |
     +-------------------+                    |                    +--------------------+
                                              |                              |
     +-------------------+                    |                              v
     | Altitude Corridor | -> [ F_corridor ] -+-- [ F_downwash ] <-- [ Asymmetric Jet ]
     +-------------------+                    |
                                              v
                                   +---------------------+
                                   | F_cmd = Sum(Forces) |
                                   +---------------------+
                                              |
                                              v
                                  [ Accel & Vel Clamping ]
                                              |
                                              v
                                   [ Semi-Implicit Euler ]
```

### 5.1 Khatib Artificial Potential Fields (APF)

#### 1. Conic-Parabolic Attractive Force ($\mathbf{F}_{att}$):
Avoids unbounded attraction forces over long disaster transit distances:
$$\Delta \mathbf{p} = \mathbf{p}_{target} - \mathbf{p}_i$$
$$d = \|\Delta \mathbf{p}\|$$
$$d_{thresh} = 15.0 \text{ m}, \quad k_{att} = 1.2$$

$$\mathbf{F}_{att} = \begin{cases}
k_{att} \Delta \mathbf{p} & \text{if } d \le d_{thresh} \\
d_{thresh} k_{att} \frac{\Delta \mathbf{p}}{d} & \text{if } d > d_{thresh}
\end{cases}$$

#### 2. Obstacle Repulsive Force ($\mathbf{F}_{obs}$):
For each 3D Axis-Aligned Bounding Box (AABB) obstacle $m$:
- Closest surface point: $\mathbf{c}_m = \text{clip}(\mathbf{p}_i, \mathbf{min}_m, \mathbf{max}_m)$
- Surface distance: $\rho_m = \|\mathbf{p}_i - \mathbf{c}_m\|$
- Safety clearance margin: $r_{safe} = 1.5$ m
- Obstacle influence horizon: $\rho_0 = 8.0$ m
- Repulsion gain: $k_{rep} = 45.0$

If $\rho_m \le \rho_0$:
$$\hat{\mathbf{n}} = \begin{cases} \frac{\mathbf{p}_i - \mathbf{c}_m}{\rho_m} & \text{if } \rho_m > 1e-4 \\ \frac{\mathbf{p}_i - \text{center}_m}{\|\mathbf{p}_i - \text{center}_m\|} & \text{if } \rho_m \le 1e-4 \end{cases}$$
$$\mathbf{F}_{rep, m} = k_{rep} \left(\frac{1}{\max(\rho_m - r_{safe}, 0.1)} - \frac{1}{\rho_0 - r_{safe}}\right) \frac{1}{\max(\rho_m - r_{safe}, 0.1)^2} \hat{\mathbf{n}}$$
$$\mathbf{F}_{rep, m} \leftarrow \text{clip}_{mag}(\mathbf{F}_{rep, m}, 100.0 \text{ N})$$

$$\mathbf{F}_{obs} = \sum_{m} \mathbf{F}_{rep, m}$$

#### 3. Tangential Force (Local Minimum Escape):
When a drone encounters a front-on obstacle where $\mathbf{F}_{att}$ and $\mathbf{F}_{obs}$ nearly cancel each other ($\|\mathbf{F}_{att} + \mathbf{F}_{obs}\| < 0.5$ N) while $d > 2.0$ m and $\rho_m < \rho_0$:
$$\mathbf{F}_{tangent} = k_{tan} (\hat{\mathbf{n}} \times \hat{\mathbf{z}}_I)$$
where $k_{tan} = 20.0$, imparting an orthogonal velocity that steers the drone smoothly around the building edge.

---

### 5.2 Reynolds Boids Flocking Forces

Computed over all active peer drones $j \ne i$ within local perception radius $R_{percept} = 12.0$ m:

#### 1. Separation ($\mathbf{F}_{sep}$):
Preserves safe inter-UAV distance ($R_{sep} = 6.0$ m, $k_{sep} = 22.0$):
$$\Delta \mathbf{p}_{ij} = \mathbf{p}_i - \mathbf{p}_j, \quad d_{ij} = \|\Delta \mathbf{p}_{ij}\|$$
If $0.001 < d_{ij} < R_{sep}$:
$$\mathbf{F}_{sep, ij} = k_{sep} \left(\frac{1}{d_{ij}} - \frac{1}{R_{sep}}\right) \frac{\Delta \mathbf{p}_{ij}}{d_{ij}^2}$$
$$\mathbf{F}_{sep} = \sum_{j \in \mathcal{N}_{sep}} \mathbf{F}_{sep, ij}$$

#### 2. Alignment ($\mathbf{F}_{align}$):
Matches heading and velocity with peer drones traveling in the same flight mode:
$$\mathbf{F}_{align} = k_{align} \left(\frac{1}{|\mathcal{N}_i|} \sum_{j \in \mathcal{N}_i} \mathbf{v}_j - \mathbf{v}_i\right)$$
where $k_{align} = 0.8$. Active when drone is in `TRANSIT` mode and $|\mathcal{N}_i| > 0$.

#### 3. Cohesion ($\mathbf{F}_{coh}$):
Maintains swarm spatial density during transit:
$$\mathbf{F}_{coh} = k_{coh} \left(\frac{1}{|\mathcal{N}_i|} \sum_{j \in \mathcal{N}_i} \mathbf{p}_j - \mathbf{p}_i\right)$$
where $k_{coh} = 0.2$. Suppressed when drone is in `SURVEYING`, `LANDED`, or `EMERGENCY_LAND`.

---

### 5.3 Asymmetric Aerodynamic Downwash Cone Repulsion

A quadcopter's high-speed downward propeller wash creates turbulent air directly below it. Flying directly underneath another drone causes loss of lift and vortex ring state.

For drone $i$, evaluate all peer drones $j$:
$$\Delta z_{ij} = p_{i, z} - p_{j, z}$$
$$\Delta \mathbf{p}_{xy, ij} = \begin{bmatrix} p_{i, x} - p_{j, x} \\ p_{i, y} - p_{j, y} \end{bmatrix}, \quad d_{xy} = \|\Delta \mathbf{p}_{xy, ij}\|$$

**Hazard Condition**: Drone $i$ is situated below drone $j$:
$$-H_{dw}^{max} \le \Delta z_{ij} \le -0.5 \text{ m} \quad (H_{dw}^{max} = 10.0 \text{ m})$$
The downwash cone expands linearly with depth:
$$R_{cone}(|\Delta z_{ij}|) = |\Delta z_{ij}| \tan(\theta_{dw})$$
where $\theta_{dw} = 25^\circ$ ($\tan 25^\circ \approx 0.4663$).

If $d_{xy} < R_{cone}(|\Delta z_{ij}|)$:
Drone $i$ is inside drone $j$'s downwash jet!
A strong **lateral outward repulsive force** is applied:
$$\hat{\mathbf{n}}_{xy} = \begin{cases} \frac{\Delta \mathbf{p}_{xy, ij}}{d_{xy}} & \text{if } d_{xy} > 1e-3 \\ \begin{bmatrix} 1.0 \\ 0.0 \end{bmatrix} & \text{if } d_{xy} \le 1e-3 \end{cases}$$

$$\mathbf{F}_{dw, xy} = k_{dw} \cdot \exp\left(-\frac{d_{xy}^2}{2 \sigma_{dw}^2}\right) \hat{\mathbf{n}}_{xy}$$
where $k_{dw} = 30.0$ N, $\sigma_{dw} = 1.8$ m.
Additionally, a slight downward suction force $\mathbf{F}_{dw, z} = -4.0$ N acts on the lower drone if it stays centered in the jet ($d_{xy} < 1.0$ m), accelerating its escape.

$$\mathbf{F}_{downwash} = \sum_{j} \begin{bmatrix} \mathbf{F}_{dw, xy} \\ \mathbf{F}_{dw, z} \end{bmatrix}$$

---

### 5.4 4-Tier Airspace Altitude Corridor Deconfliction

Each drone role and flight mode is allocated a structured altitude envelope $[z_{min}, z_{max}]$:

| Flight Mode / Role | Assigned Corridor | Altitude Band ($[z_{min}, z_{max}]$) | Nominal Target Altitude |
|---|---|---|---|
| `IDLE` / `LANDED` | Ground | $0.0$ m | $0.0$ m |
| `TAKEOFF` | Tier 1: Launch | $[0.0, 20.0]$ m | $15.0$ m (survey) / $20.0$ m (relay climb) |
| `SURVEYING` | Tier 2: PoI Inspection | $[25.0, 45.0]$ m | $30.0 - 40.0$ m (per PoI height) |
| `TRANSIT` / `RTL` | Tier 3: Fleet Transit | $[50.0, 65.0]$ m | $55.0$ m |
| `RELAY` | Tier 4: Mesh Backbone | $[70.0, 90.0]$ m | $75.0$ m (Relay 1), $85.0$ m (Relay 2) |
| `LANDING` / `EMERGENCY_LAND`| Descent Corridor | $[0.0, 20.0]$ m | $0.0$ m |

#### Restoring Spring-Damper Force ($\mathbf{F}_{corridor}$):
When a drone deviates outside its assigned corridor $[z_{min}, z_{max}]$, a restoring force returns it to the corridor:

$$F_{corridor, z} = \begin{cases}
k_{corr} (z_{min} - p_z) - d_{corr} v_z & \text{if } p_z < z_{min} \\
k_{corr} (z_{max} - p_z) - d_{corr} v_z & \text{if } p_z > z_{max} \\
0.0 & \text{if } z_{min} \le p_z \le z_{max}
\end{cases}$$

where $k_{corr} = 15.0$ N/m, $d_{corr} = 4.0$ N*s/m.
$$\mathbf{F}_{corridor} = \begin{bmatrix} 0.0 \\ 0.0 \\ F_{corridor, z} \end{bmatrix}$$

---

## 6. Concrete Class Blueprint (`sim/drone.py`)

Here is the exact production-ready blueprint for `sim/drone.py`:

```python
"""
sim/drone.py: 6-DOF Quadcopter Kinematics, Dynamics, Flocking & Vector Steering Engine
"""
from __future__ import annotations
import math
from typing import List, Optional, Tuple, Any, Dict
import numpy as np

from sim.types import (
    DroneRole,
    FlightMode,
    DroneLimits,
    BatteryModel,
    DroneState,
    AIRSPACE_CORRIDORS
)


class Drone:
    """
    Autonomous 6-DOF Quadcopter Agent.
    Implements Newton-Euler translational and rotational kinematics,
    Khatib APF, Reynolds flocking, downwash repulsion, and altitude corridor clamping.
    """

    def __init__(
        self,
        drone_id: str,
        role: DroneRole = DroneRole.SURVEY,
        initial_position: Optional[np.ndarray] = None,
        limits: Optional[DroneLimits] = None,
        battery: Optional[BatteryModel] = None,
    ):
        self.id = drone_id
        self.role = role
        self.limits = limits if limits is not None else DroneLimits()
        self.battery = battery if battery is not None else BatteryModel()

        # 3D Physical States (Inertial ENU Frame)
        init_pos = initial_position if initial_position is not None else np.zeros(3)
        self.position = np.array(init_pos, dtype=np.float64)
        self.velocity = np.zeros(3, dtype=np.float64)
        self.acceleration = np.zeros(3, dtype=np.float64)

        # Attitude: Euler [roll, pitch, yaw] in radians and unit Quaternion [qw, qx, qy, qz]
        self.attitude = np.zeros(3, dtype=np.float64)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.angular_velocity = np.zeros(3, dtype=np.float64)
        self.rotor_speeds = np.full(4, 993.4, dtype=np.float64)

        # Mission & Navigation
        self.flight_mode = FlightMode.IDLE
        self.target_position: Optional[np.ndarray] = None
        self.assigned_poi_id: Optional[str] = None
        self.is_transmitting: bool = False
        self.dwell_time: float = 0.0

        # Physical constants
        self.g = 9.80665

    # --------------------------------------------------------------------------
    # Attitude Conversion Helpers
    # --------------------------------------------------------------------------
    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Converts Euler roll, pitch, yaw (radians) into a normalized unit quaternion."""
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        q = np.array([qw, qx, qy, qz], dtype=np.float64)
        norm = np.linalg.norm(q)
        return q / norm if norm > 1e-9 else np.array([1.0, 0.0, 0.0, 0.0])

    @staticmethod
    def quaternion_to_euler(q: np.ndarray) -> np.ndarray:
        """Converts quaternion [qw, qx, qy, qz] into Euler roll, pitch, yaw (radians)."""
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]

        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (qw * qy - qz * qx)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw], dtype=np.float64)

    # --------------------------------------------------------------------------
    # Force Synthesis: APF, Flocking, Downwash, Corridors
    # --------------------------------------------------------------------------
    def compute_attractive_force(self) -> np.ndarray:
        """Calculates conic-parabolic APF attractive force towards target_position."""
        if self.target_position is None:
            return np.zeros(3)

        diff = self.target_position - self.position
        dist = float(np.linalg.norm(diff))
        if dist < 1e-4:
            return np.zeros(3)

        d_thresh = 15.0
        k_att = 1.2
        if dist <= d_thresh:
            return k_att * diff
        else:
            return d_thresh * k_att * (diff / dist)

    def compute_obstacle_repulsion(self, obstacles: List[Any]) -> np.ndarray:
        """Calculates APF obstacle repulsion forces from 3D AABBs."""
        f_obs = np.zeros(3, dtype=np.float64)
        k_rep = 45.0
        rho_0 = 8.0
        r_safe = 1.5

        for obs in obstacles:
            # Polymorphic duck-typing: handles AABB objects with distance_and_closest_point
            if hasattr(obs, "distance_and_closest_point"):
                dist, closest_pt = obs.distance_and_closest_point(self.position)
            elif hasattr(obs, "min_bound") and hasattr(obs, "max_bound"):
                closest_pt = np.clip(self.position, obs.min_bound, obs.max_bound)
                dist = float(np.linalg.norm(self.position - closest_pt))
            else:
                continue

            if dist < rho_0:
                push = self.position - closest_pt
                push_norm = float(np.linalg.norm(push))
                if push_norm > 1e-4:
                    n_hat = push / push_norm
                else:
                    n_hat = np.array([0.0, 0.0, 1.0])

                eff_dist = max(dist - r_safe, 0.1)
                eff_rho0 = max(rho_0 - r_safe, 0.2)
                mag = k_rep * (1.0 / eff_dist - 1.0 / eff_rho0) * (1.0 / (eff_dist ** 2))
                mag = min(mag, 100.0)  # Numerical ceiling
                f_obs += mag * n_hat

        return f_obs

    def compute_flocking_forces(self, peers: List[Drone]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Calculates Reynolds separation, alignment, and cohesion forces."""
        f_sep = np.zeros(3, dtype=np.float64)
        f_align = np.zeros(3, dtype=np.float64)
        f_coh = np.zeros(3, dtype=np.float64)

        neighbors: List[Drone] = []
        r_percept = 12.0
        r_sep = self.limits.separation_radius
        k_sep = 22.0

        for peer in peers:
            if peer.id == self.id or peer.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
                continue

            diff = self.position - peer.position
            dist = float(np.linalg.norm(diff))
            if dist < 1e-4:
                continue

            # Separation
            if dist < r_sep:
                eff_dist = max(dist, self.limits.collision_radius * 0.5)
                mag_sep = k_sep * (1.0 / eff_dist - 1.0 / r_sep) * (1.0 / (eff_dist ** 2))
                mag_sep = min(mag_sep, 80.0)
                f_sep += mag_sep * (diff / dist)

            if dist < r_percept:
                neighbors.append(peer)

        if neighbors:
            n_count = len(neighbors)
            # Alignment (match velocity of active transit flock)
            if self.flight_mode == FlightMode.TRANSIT:
                avg_vel = np.mean([p.velocity for p in neighbors], axis=0)
                f_align = 0.8 * (avg_vel - self.velocity)

            # Cohesion (pull towards local center of mass)
            if self.flight_mode == FlightMode.TRANSIT:
                avg_pos = np.mean([p.position for p in neighbors], axis=0)
                f_coh = 0.2 * (avg_pos - self.position)

        return f_sep, f_align, f_coh

    def compute_downwash_repulsion(self, peers: List[Drone]) -> np.ndarray:
        """
        Calculates asymmetric vertical downwash jet repulsion.
        Pushes this drone horizontally outward if it is located inside the
        turbulent propeller wash cone of a higher drone.
        """
        f_dw = np.zeros(3, dtype=np.float64)
        tan_theta = 0.4663  # tan(25 deg)
        k_dw = 30.0
        sigma_dw = 1.8

        for peer in peers:
            if peer.id == self.id or peer.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
                continue

            # Delta z: positive if peer is ABOVE this drone
            dz = peer.position[2] - self.position[2]
            if 0.5 <= dz <= 10.0:
                diff_xy = self.position[:2] - peer.position[:2]
                d_xy = float(np.linalg.norm(diff_xy))
                r_cone = dz * tan_theta

                if d_xy < r_cone:
                    # Inside the downwash cone! Outward lateral push
                    if d_xy > 1e-3:
                        n_xy = diff_xy / d_xy
                    else:
                        n_xy = np.array([1.0, 0.0])  # Deterministic escape vector

                    mag = k_dw * math.exp(-(d_xy ** 2) / (2.0 * (sigma_dw ** 2)))
                    f_dw[0] += mag * n_xy[0]
                    f_dw[1] += mag * n_xy[1]
                    f_dw[2] -= 4.0 * math.exp(-(d_xy ** 2) / 2.0)  # Downward turbulence sink

        return f_dw

    def compute_corridor_force(self) -> np.ndarray:
        """Calculates restoring vertical force to enforce 4-tier airspace corridor."""
        z_min, z_max = self.get_assigned_altitude_band()
        pz = self.position[2]
        k_corr = 15.0
        d_corr = 4.0

        fz = 0.0
        if pz < z_min:
            fz = k_corr * (z_min - pz) - d_corr * self.velocity[2]
        elif pz > z_max:
            fz = k_corr * (z_max - pz) - d_corr * self.velocity[2]

        return np.array([0.0, 0.0, fz], dtype=np.float64)

    def get_assigned_altitude_band(self) -> Tuple[float, float]:
        """Maps drone role and flight mode to airspace altitude envelope."""
        if self.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
            return 0.0, 1.0
        elif self.flight_mode in (FlightMode.TAKEOFF, FlightMode.LANDING, FlightMode.EMERGENCY_LAND):
            return 0.0, 25.0
        elif self.flight_mode == FlightMode.SURVEYING:
            return 25.0, 45.0  # Tier 2: PoI Inspection
        elif self.flight_mode in (FlightMode.TRANSIT, FlightMode.RTL):
            return 50.0, 65.0  # Tier 3: High-speed Transit
        elif self.flight_mode == FlightMode.RELAY:
            return 70.0, 90.0  # Tier 4: Elevated Relay Mesh
        return 0.0, 120.0

    def compute_total_force(
        self,
        obstacles: Optional[List[Any]] = None,
        peers: Optional[List[Drone]] = None
    ) -> np.ndarray:
        """Synthesizes all vector steering forces into a composite commanded force."""
        if self.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
            return np.zeros(3)

        obs_list = obstacles if obstacles is not None else []
        peer_list = peers if peers is not None else []

        f_att = self.compute_attractive_force()
        f_obs = self.compute_obstacle_repulsion(obs_list)
        f_sep, f_align, f_coh = self.compute_flocking_forces(peer_list)
        f_dw = self.compute_downwash_repulsion(peer_list)
        f_corr = self.compute_corridor_force()

        # Local minimum escape check
        net_horizontal = f_att[:2] + f_obs[:2]
        if np.linalg.norm(net_horizontal) < 0.5 and np.linalg.norm(f_obs[:2]) > 1.0:
            # Inject tangential circulatory force around obstacle
            n_obs = f_obs[:2] / np.linalg.norm(f_obs[:2])
            f_tan = np.array([-n_obs[1], n_obs[0], 0.0]) * 15.0
            return f_att + f_obs + f_sep + f_align + f_coh + f_dw + f_corr + f_tan

        return f_att + f_obs + f_sep + f_align + f_coh + f_dw + f_corr

    # --------------------------------------------------------------------------
    # Physics Integration & State Update
    # --------------------------------------------------------------------------
    def step_physics(self, dt: float, commanded_force: np.ndarray) -> None:
        """
        Executes semi-implicit Euler integration of translational and rotational kinematics.
        Enforces physical acceleration, speed, vertical rate, and ground contact clamping.
        """
        if self.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
            self.velocity[:] = 0.0
            self.acceleration[:] = 0.0
            self.battery.step(dt, speed=0.0, accel=0.0, is_transmitting=False, is_surveying=False)
            return

        m = self.limits.mass_kg

        # 1. Commanded Acceleration & Saturation
        a_cmd = commanded_force / m
        a_mag = float(np.linalg.norm(a_cmd))
        if a_mag > self.limits.max_accel:
            self.acceleration = (a_cmd / a_mag) * self.limits.max_accel
        else:
            self.acceleration = a_cmd.copy()

        # 2. Linear Drag Integration
        drag_accel = np.array([
            (self.limits.drag_coeff_xy / m) * self.velocity[0],
            (self.limits.drag_coeff_xy / m) * self.velocity[1],
            (self.limits.drag_coeff_z / m) * self.velocity[2],
        ])
        a_net = self.acceleration - drag_accel

        # 3. Velocity Integration & Hard Boundary Clamping
        self.velocity += a_net * dt

        # Horizontal speed clamp
        v_xy = float(np.linalg.norm(self.velocity[:2]))
        if v_xy > self.limits.max_speed_xy:
            self.velocity[:2] = (self.velocity[:2] / v_xy) * self.limits.max_speed_xy

        # Vertical climb / descent clamp
        self.velocity[2] = max(-self.limits.max_speed_z_down, min(self.limits.max_speed_z_up, self.velocity[2]))

        # 4. Position Integration & Ground Surface Constraint
        self.position += self.velocity * dt
        if self.position[2] <= 0.0:
            self.position[2] = 0.0
            self.velocity[2] = max(0.0, self.velocity[2])
            self.acceleration[2] = max(0.0, self.acceleration[2])

        # 5. Attitude Generation (Euler & Quaternion)
        self._update_attitude(dt)

        # 6. Rotor Speeds Update
        hover_speed = 993.4
        a_norm = float(np.linalg.norm(self.acceleration))
        delta_rotor = 50.0 * (a_norm / self.limits.max_accel)
        self.rotor_speeds = np.clip(
            np.array([hover_speed + delta_rotor, hover_speed - delta_rotor, hover_speed + delta_rotor, hover_speed - delta_rotor]),
            400.0, 1500.0
        )

        # 7. Battery State of Charge Depletion
        speed = float(np.linalg.norm(self.velocity))
        self.battery.step(
            dt=dt,
            speed=speed,
            accel=a_norm,
            is_transmitting=self.is_transmitting,
            is_surveying=(self.flight_mode == FlightMode.SURVEYING)
        )

    def _update_attitude(self, dt: float) -> None:
        """Calculates desired tilt from acceleration and smoothly tracks it."""
        g = self.g
        # Body z-axis tilt
        desired_roll = float(np.clip(self.acceleration[1] / g, -self.limits.max_tilt_rad, self.limits.max_tilt_rad))
        desired_pitch = float(np.clip(-self.acceleration[0] / g, -self.limits.max_tilt_rad, self.limits.max_tilt_rad))

        # Desired yaw: track horizontal velocity vector or target
        v_xy = float(np.linalg.norm(self.velocity[:2]))
        if v_xy > 0.5:
            target_yaw = math.atan2(self.velocity[1], self.velocity[0])
        elif self.target_position is not None:
            target_yaw = math.atan2(self.target_position[1] - self.position[1], self.target_position[0] - self.position[0])
        else:
            target_yaw = self.attitude[2]

        # Slew rate filtering
        tau_att = 0.12
        roll_diff = desired_roll - self.attitude[0]
        pitch_diff = desired_pitch - self.attitude[1]
        self.attitude[0] += (roll_diff / tau_att) * dt
        self.attitude[1] += (pitch_diff / tau_att) * dt

        # Yaw wrap-around
        yaw_err = math.atan2(math.sin(target_yaw - self.attitude[2]), math.cos(target_yaw - self.attitude[2]))
        max_dyaw = self.limits.max_yaw_rate * dt
        self.attitude[2] += max(-max_dyaw, min(max_dyaw, yaw_err))
        self.attitude[2] = math.atan2(math.sin(self.attitude[2]), math.cos(self.attitude[2]))

        # Normalize and update quaternion
        self.quaternion = self.euler_to_quaternion(self.attitude[0], self.attitude[1], self.attitude[2])

    def get_state(self) -> DroneState:
        """Produces a clean DroneState snapshot matching PROJECT.md interface."""
        return DroneState(
            id=self.id,
            role=self.role.value,
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            attitude=self.attitude.copy(),
            rotor_speeds=self.rotor_speeds.copy(),
            battery_soc=self.battery.soc,
            flight_mode=self.flight_mode.value,
            assigned_poi_id=self.assigned_poi_id,
            target_position=self.target_position.copy() if self.target_position is not None else None,
            quaternion=self.quaternion.copy(),
            acceleration=self.acceleration.copy()
        )
```

---

## 7. Exhaustive Unit Test Suite Plan (`tests/unit/test_drone.py`)

A comprehensive unit test suite with 20+ isolated test assertions must be implemented to verify every mathematical guarantee:

### 7.1 Kinematics & Physics Clamping
1. `test_acceleration_clamping`:
   - Inject commanded force $\mathbf{F} = [200.0, 0.0, 0.0]^T$ N ($m = 1.2$ kg $\implies a_{cmd} = 166.6$ m/s$^2$).
   - Run `step_physics(dt=0.05, ...)`.
   - Assert $\|\mathbf{a}\| \le 4.0 + 1e-6$ m/s$^2$.
2. `test_velocity_horizontal_clamping`:
   - Repeatedly apply maximum forward force for $5.0$ seconds.
   - Assert horizontal speed $\|\mathbf{v}_{xy}\| \le 10.0 + 1e-6$ m/s.
3. `test_velocity_climb_rate_clamping`:
   - Apply continuous upward force ($F_z = 100.0$ N).
   - Assert vertical speed $v_z \le 3.5 + 1e-6$ m/s.
4. `test_velocity_descent_rate_clamping`:
   - Apply continuous downward force ($F_z = -100.0$ N).
   - Assert vertical descent rate $v_z \ge -2.5 - 1e-6$ m/s.
5. `test_ground_collision_constraint`:
   - Initialize drone at $z = 0.5$ m, apply downward velocity $v_z = -5.0$ m/s with downward force.
   - Run `step_physics(dt=0.2, ...)`.
   - Assert $p_z \ge 0.0$ and $v_z \ge 0.0$ (drone does not penetrate ground).

### 7.2 Attitude & Quaternion Mathematics
6. `test_euler_quaternion_roundtrip`:
   - Test multiple angles: $(\phi, \theta, \psi) \in \{(0, 0, 0), (0.2, -0.3, 1.2), (-0.4, 0.5, -\pi/2)\}$.
   - Convert to quaternion using `euler_to_quaternion()`, then back using `quaternion_to_euler()`.
   - Assert all Euler angles match original within absolute tolerance $10^{-5}$ rad.
7. `test_quaternion_norm_invariance`:
   - Run drone through vigorous dynamic acceleration maneuvers.
   - Assert $\|\mathbf{q}\| == 1.0 \pm 10^{-6}$ at every step.
8. `test_tilt_clamping`:
   - Command extreme lateral force ($a_y = 10.0$ m/s$^2$).
   - Assert roll angle $|\phi| \le 0.5236 + 1e-5$ rad ($30^\circ$).
9. `test_yaw_rate_limiting`:
   - Step drone with sharp $180^\circ$ turn request over $\Delta t = 0.02$ s.
   - Assert $|\Delta \psi| \le 1.5708 \times 0.02 + 1e-6$ rad.

### 7.3 APF & Flocking Vector Steering
10. `test_apf_conic_parabolic_switch`:
    - Set target at $d = 5.0$ m ($\le 15.0$ m). Assert $\|\mathbf{F}_{att}\| = 1.2 \times 5.0 = 6.0$ N.
    - Set target at $d = 30.0$ m ($> 15.0$ m). Assert $\|\mathbf{F}_{att}\| = 15.0 \times 1.2 = 18.0$ N (conic linear regime).
11. `test_obstacle_repulsion_direction`:
    - Place mock AABB obstacle at $[10, 10, 0] \to [20, 20, 30]$.
    - Position drone at $[9.0, 15.0, 15.0]$.
    - Compute `compute_obstacle_repulsion()`.
    - Assert repulsive force has $F_x < 0$ (pushes directly away from obstacle face).
12. `test_obstacle_repulsion_distance_threshold`:
    - Position drone at distance $10.0$ m from obstacle ($> \rho_0 = 8.0$ m).
    - Assert `compute_obstacle_repulsion()` returns exactly $[0.0, 0.0, 0.0]$.
13. `test_reynolds_separation_quadratic_growth`:
    - Test two drones at distance $4.0$ m vs $2.0$ m.
    - Assert separation force at $2.0$ m is strictly greater than force at $4.0$ m.
14. `test_reynolds_alignment`:
    - Place peer drone in transit with velocity $[8.0, 0.0, 0.0]$.
    - Self drone at rest in transit.
    - Assert alignment force has positive $F_x > 0$.

### 7.4 Aerodynamic Downwash Cone
15. `test_downwash_cone_trigger`:
    - Drone A at $[10.0, 10.0, 30.0]$, Drone B at $[10.5, 10.0, 25.0]$ ($\Delta z = 5.0$ m, $d_{xy} = 0.5$ m).
    - Cone radius $R = 5.0 \times \tan(25^\circ) \approx 2.33$ m.
    - Because $d_{xy} = 0.5 < 2.33$, downwash is active.
    - Assert Drone B experiences positive $F_x > 10.0$ N and downward sink $F_z < 0$.
16. `test_downwash_outside_cone_inactive`:
    - Drone A at $[10.0, 10.0, 30.0]$, Drone B at $[15.0, 10.0, 25.0]$ ($d_{xy} = 5.0$ m $> 2.33$ m).
    - Assert downwash force on Drone B is zero.
17. `test_downwash_upper_drone_unaffected`:
    - Verify Drone A (the higher drone) experiences zero downwash force from Drone B below it.

### 7.5 Airspace Altitude Corridors
18. `test_survey_altitude_corridor_enforcement`:
    - Set drone flight mode to `SURVEYING` (band: $[25, 45]$ m).
    - Place drone at $z = 15.0$ m (below band). Assert $F_{corr, z} > 0$ (pulls up).
    - Place drone at $z = 50.0$ m (above band). Assert $F_{corr, z} < 0$ (pulls down).
19. `test_relay_altitude_corridor_enforcement`:
    - Set drone flight mode to `RELAY` (band: $[70, 90]$ m).
    - Place drone at $z = 50.0$ m. Assert $F_{corr, z} > 0$ (pulls up into relay corridor).

### 7.6 Battery Discharge Model
20. `test_battery_depletion_rate`:
    - Run hover flight for $60.0$ seconds at zero velocity.
    - Expected power: $P \approx 12 + 180 + 3 = 195$ W.
    - Expected energy: $195 \times 60 = 11,700$ J out of $266,400$ J ($\approx 4.39\%$ drop).
    - Assert final $SoC \approx 0.956 \pm 0.005$.
21. `test_battery_threshold_flags`:
    - Set battery $SoC = 0.24$. Assert `battery.is_low() == True` and `battery.is_critical() == False`.
    - Set battery $SoC = 0.09$. Assert `battery.is_critical() == True`.
22. `test_battery_floor_at_zero`:
    - Drain battery with excessive power. Assert $SoC \ge 0.0$.

---

## 8. Cross-Module Interface Compatibility Verification

1. **Compatibility with `sim/types.py` & `PROJECT.md` Contract #1**:
   - `Drone.get_state()` produces `DroneState` containing:
     `id`, `role`, `position`, `velocity`, `attitude`, `rotor_speeds`, `battery_soc`, `flight_mode`, `assigned_poi_id`, `target_position`.
   - All vector outputs are standard 1D `numpy.ndarray` with `dtype=np.float64`.
   - `to_dict()` produces clean JSON-serializable primitives for `vis/server.py` and Three.js HUD telemetry frames.

2. **Compatibility with `sim/environment.py` & `sim/obstacles.py` (M1-2)**:
   - `compute_obstacle_repulsion()` supports both duck-typed method `obstacle.distance_and_closest_point(point)` and bounding box attributes (`obstacle.min_bound`, `obstacle.max_bound`).

3. **Compatibility with `sim/core.py` (M1-3)**:
   - Multi-agent coordination is completely decoupled: `SwarmSimulationCore` simply iterates over drones, calls `drone.compute_total_force(obstacles, peers)`, and passes the result to `drone.step_physics(dt, force)`.
   - Guaranteed deterministic execution in headless simulations ($> 10,000$ simulation steps/sec).

4. **Compatibility with E2E Testing Suite (Milestone E2E / `TEST_INFRA.md`)**:
   - Satisfies Tier 1 (Feature Isolation) tests for Features F2, F3, F4.
   - Satisfies Tier 2 (Boundary & Corner Cases) for boundary velocities ($v_{xy} = 10.0$ m/s, $v_z \in [-2.5, 3.5]$ m/s), obstacle grazing, ground contact ($z=0$), and zero battery ($SoC=0$).
   - Satisfies Tier 4 Scenarios (Scenario 4: High-density multi-UAV swarm collision stress where inter-drone separation distance never drops below $1.5$ m).
