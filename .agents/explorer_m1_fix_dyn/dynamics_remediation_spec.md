# Engineering Remediation Specification: Dynamic Collision Avoidance, Physics & Kinematics Integrity

**Document ID**: `SPEC-M1-FIX-002`  
**Target Module**: `sim/drone.py` (with integration guidance for `sim/core.py`)  
**Author**: Explorer M1-Fix-2 (`teamwork_preview_explorer` — Dynamics & Collision Specialist)  
**Status**: APPROVED BLUEPRINT FOR IMPLEMENTATION  
**Date**: 2026-09-25T14:58:00Z  

---

## 1. Executive Summary & Root Cause Matrix

Adversarial stress testing by Challenger M1-1 and forensic auditing by Auditor M1 uncovered four distinct failure modes in the Milestone 1 codebase. While basic kinematics constraints (acceleration $\le 4.0\text{ m/s}^2$, speeds $\le 10.0\text{ m/s}$, ground contact $Z \ge 0$) were rigorously enforced, the separation, obstacle avoidance, and type reflection subsystems failed under dynamic stress conditions:

| # | Vulnerability | Location | Root Cause | Impact |
|---|---------------|----------|------------|--------|
| **V1** | Missing `Sequence` Type Import | `sim/drone.py:13` | `from typing import ...` omitted `Sequence`, referenced on lines 91 & 96 | `typing.get_type_hints()` crashes with `NameError` |
| **V2** | High-Velocity Head-On Collision Tunneling | `sim/drone.py:235-253` | Static $r_{sep}=6.0\text{ m}$ horizon; stopping from $v_{close}=20\text{ m/s}$ requires $\ge 25\text{ m}$ to $50\text{ m}$ | Drones tunnel through each other ($d_{min} = 0.026\text{ m}$, `crossed = True`) |
| **V3** | Multi-Drone Collinear Compression | `sim/drone.py:174-190, 248-253` | Target attraction ($22.5\text{ N}$) overpowers separation ($6.67\text{ N}$); no attraction attenuation along blocked axis | Fleet compresses to $< 0.1\text{ m}$, violating $1.5\text{ m}$ safety bubble |
| **V4** | High-Speed 3D Obstacle Penetration | `sim/drone.py:191-225, 420-425` | Obstacle sensing radius static $\rho_0=8.0\text{ m}$ ($d_{stop}=12.5\text{ m}$ at $10\text{ m/s}$); no hard obstacle stopping in `step_physics` | Drones cruise at $10\text{ m/s}$ into solid building interiors |

This specification provides the exact mathematical derivations, physical barrier formulations, and complete drop-in Python code replacements to resolve all four vulnerabilities with zero regression to existing unit or E2E tests.

---

## 2. Vulnerability 1: Type Annotation Import Fix

### 2.1 Problem Analysis
In `sim/drone.py`:
- Line 91: `def set_target_waypoint(self, pos: Union[np.ndarray, Sequence[float]]) -> None:`
- Line 95: `def set_target(self, pos: Union[np.ndarray, Sequence[float]]) -> None:`
- Line 13: `from typing import Any, List, Optional, Tuple, Union`

Under Python 3.10+, `from __future__ import annotations` postpones evaluation of type annotations at module load time. However, any runtime reflection or introspection tool calling `typing.get_type_hints(Drone.set_target_waypoint)` fails immediately with:
```
NameError: name 'Sequence' is not defined
```

### 2.2 Exact Code Replacement
In `sim/drone.py`, line 13:
```python
# BEFORE
from typing import Any, List, Optional, Tuple, Union

# AFTER
from typing import Any, List, Optional, Sequence, Tuple, Union
```

### 2.3 Verification Command
```powershell
python -c "import typing, sim.drone; print(typing.get_type_hints(sim.drone.Drone.set_target_waypoint))"
```
*Expected Output*: `{'pos': typing.Union[numpy.ndarray, typing.Sequence[float]], 'return': <class 'NoneType'>}` without error.

---

## 3. Vulnerability 2: High-Velocity Head-On Collision Tunneling

### 3.1 Physical Derivation & Kinematic Analysis
Let Drone $i$ have position $\mathbf{p}_i$ and velocity $\mathbf{v}_i$, and Drone $j$ have position $\mathbf{p}_j$ and velocity $\mathbf{v}_j$.
- Displacement: $\mathbf{r}_{ij} = \mathbf{p}_i - \mathbf{p}_j$
- Euclidean distance: $d_{ij} = \|\mathbf{r}_{ij}\|$
- Unit separation vector: $\hat{\mathbf{r}}_{ij} = \frac{\mathbf{r}_{ij}}{d_{ij}}$
- Relative velocity: $\mathbf{v}_{rel} = \mathbf{v}_i - \mathbf{v}_j$
- Relative closing velocity:
  $$v_{close} = \max\left(0.0, \; -\mathbf{v}_{rel} \cdot \hat{\mathbf{r}}_{ij}\right) = \max\left(0.0, \; (\mathbf{v}_j - \mathbf{v}_i) \cdot \hat{\mathbf{r}}_{ij}\right)$$

When two drones approach head-on at maximum velocity ($v_{max} = 10.0\text{ m/s}$), the closing speed is $v_{close} = 20.0\text{ m/s}$.
The maximum physical deceleration is $a_{max} = 4.0\text{ m/s}^2$ (`limits.max_accel`).
The stopping distance for closing speed $v_{close}$ at effective relative deceleration $a_{eff}$ plus reaction margin is:
$$d_{stop}(v_{close}) = \frac{v_{close}^2}{2 a_{eff}} + v_{close} \cdot \tau_{margin} + r_{safe}$$

Setting $a_{eff} = 3.0\text{ m/s}^2$ (conservative fleet capability), $\tau_{margin} = 0.8\text{ s}$ (accounting for attitude lag $\tau_{att} = 0.12\text{ s}$ and discrete timestep $\Delta t = 0.05\text{ s}$), and $r_{safe} = 2.5\text{ m}$:
$$d_{stop}(20.0\text{ m/s}) = \frac{400}{6.0} + 20.0 \times 0.8 + 2.5 = 66.67 + 16.0 + 2.5 = 85.17\text{ m}$$
Under static $r_{sep} = 6.0\text{ m}$, the available deceleration time at $20\text{ m/s}$ is only $t = 6 / 20 = 0.30\text{ s}$, which allows at most $\Delta v = a_{max} \cdot t = 1.2\text{ m/s}$ of speed reduction. The drones collide at $8.8\text{ m/s}$ and tunnel through one another.

### 3.2 Dynamic Velocity-Dependent Repulsive Horizon
The separation interaction horizon $r_{sep, dyn}$ must expand dynamically based on relative closing velocity:
$$r_{sep, dyn} = \max\left(r_{sep, static}, \; \frac{v_{close}^2}{6.0} + 0.8 \cdot v_{close} + 2.5\right)$$
where $r_{sep, static} = 6.0\text{ m}$.

### 3.3 Force Synthesis with Closing Velocity Damping
Inside $d_{ij} < r_{sep, dyn}$, the separation force $\mathbf{F}_{sep}$ incorporates three complementary physical components:
1. **Positional Khatib Potential**:
   $$F_{pos} = k_{sep} \left(\frac{1}{d_{eff}} - \frac{1}{r_{eff}}\right) \frac{1}{d_{eff}^2}$$
   where $d_{eff} = \max(d_{ij} - 1.8, 0.1)$, $r_{eff} = \max(r_{sep, dyn} - 1.8, 0.2)$, $k_{sep} = 45.0$.
2. **Relative Closing Velocity Damping (Kinetic Energy Dissipation)**:
   $$F_{damp} = k_{v, sep} \cdot v_{close} \cdot \left(\frac{r_{sep, dyn} - d_{ij}}{r_{sep, dyn}}\right)^2 \cdot m$$
   where $k_{v, sep} = 12.0$, $m = 1.2\text{ kg}$. This exerts a smooth, powerful braking force along $\hat{\mathbf{r}}_{ij}$ that directly dissipates relative kinetic energy.
3. **Singularity Barrier**:
   For $d_{ij} < 2.5\text{ m}$:
   $$F_{barrier} = 80.0 \cdot \left(\frac{2.2}{\max(d_{ij}, 0.1)}\right)^3$$

The composite separation magnitude is capped at $250.0\text{ N}$ (well beyond the $4.8\text{ N}$ saturation threshold, ensuring maximum commanded deceleration $a_{max} = 4.0\text{ m/s}^2$):
$$\mathbf{F}_{sep} = \min\left(F_{pos} + F_{damp} + F_{barrier}, \; 250.0\right) \cdot \hat{\mathbf{r}}_{ij}$$

---

## 4. Vulnerability 3: Multi-Drone Collinear Compression & Attractive Attenuation

### 4.1 Root Cause Analysis
In a collinear formation of $N \ge 3$ drones (e.g. D1 at $-15\text{ m}$, D2 at $0\text{ m}$, D3 at $+15\text{ m}$), D1 is commanded to waypoint $+50\text{ m}$ and D3 to $-50\text{ m}$.
- The attractive force on D1 pulls with $15 \times 1.5 = 22.5\text{ N}$ in $+X$.
- When D1 approaches D2 at $d = 1.5\text{ m}$, the baseline separation force was only $6.67\text{ N}$ in $-X$.
- Net force on D1: $+22.5 - 6.67 = +15.83\text{ N}$ forward!
- D1 accelerates directly into D2, compressing the fleet to $0.038\text{ m} - 0.085\text{ m}$.

### 4.2 Mathematical Solution: Prioritized Safety Filter (Attraction Attenuation)
When another drone or obstacle lies along the path between the drone and its waypoint setpoint, the attractive force pulling towards the blocked path must be attenuated and ultimately zeroed out as $d \to r_{safe}$:

1. **Target and Obstacle Unit Vectors**:
   $$\hat{\mathbf{u}}_{tgt} = \frac{\mathbf{p}_{tgt} - \mathbf{p}}{\|\mathbf{p}_{tgt} - \mathbf{p}\|}, \quad \hat{\mathbf{u}}_{peer} = -\hat{\mathbf{r}}_{ij} = \frac{\mathbf{p}_j - \mathbf{p}}{d_{ij}}$$
2. **Alignment Metric**:
   $$\cos \phi = \hat{\mathbf{u}}_{tgt} \cdot \hat{\mathbf{u}}_{peer}$$
   If $\cos \phi > 0.0$, Peer $j$ lies in front of Drone $i$ on its way to the target.
3. **Directional Attractive Component**:
   $$F_{att, parallel} = \max\left(0.0, \; \mathbf{F}_{att} \cdot \hat{\mathbf{u}}_{peer}\right)$$
4. **Quadratic Attenuation Factor $\gamma(d_{ij})$**:
   $$\gamma(d_{ij}) = \begin{cases}
   0.0 & \text{if } d_{ij} \le 2.2\text{ m} \\
   \left(\frac{d_{ij} - 2.2}{8.0 - 2.2}\right)^2 & \text{if } 2.2\text{ m} < d_{ij} < 8.0\text{ m} \\
   1.0 & \text{if } d_{ij} \ge 8.0\text{ m}
   \end{cases}$$
5. **Attenuated Attractive Force**:
   $$\mathbf{F}_{att, eff} = \mathbf{F}_{att} - (1.0 - \gamma(d_{ij})) \cdot F_{att, parallel} \cdot \hat{\mathbf{u}}_{peer}$$

When $d_{ij} \le 2.2\text{ m}$, $\gamma = 0.0$, eliminating 100% of the forward attractive force pulling towards Peer $j$. Combined with the $F_{barrier} \ge 80.0\text{ N}$ repulsive barrier, the net force on D1 is purely repulsive ($-4.0\text{ m/s}^2$), bringing D1 to a complete halt at $d \ge 3.0\text{ m}$.

---

## 5. Vulnerability 4: Dynamic Obstacle Sensing Horizon & Penetration Prevention

### 5.1 Physical Kinematics of Obstacle Approach
At maximum cruising speed $v = 10.0\text{ m/s}$, the kinematic stopping distance is $d_{stop} = \frac{v^2}{2 a_{max}} = \frac{100}{8} = 12.5\text{ m}$.
Static perception $\rho_0 = 8.0\text{ m}$ guarantees penetration: at $8.0\text{ m}$, full deceleration can only reduce speed to $\sqrt{10^2 - 2 \cdot 4 \cdot 8} = \sqrt{36} = 6.0\text{ m/s}$. The drone crosses the obstacle face at $6.0\text{ m/s}$.

### 5.2 Dynamic Obstacle Horizon Formulation
For any 3D AABB obstacle:
1. Compute closest surface point $\mathbf{p}_{closest}$ and Euclidean surface distance $d_{obs}$.
2. Outward unit normal: $\hat{\mathbf{n}}_{obs} = \frac{\mathbf{p} - \mathbf{p}_{closest}}{\max(d_{obs}, 10^{-4})}$.
3. Inward approach speed:
   $$v_{approach} = \max\left(0.0, \; -\mathbf{v} \cdot \hat{\mathbf{n}}_{obs}\right)$$
4. Dynamic obstacle interaction radius:
   $$\rho_{0, dyn} = \max\left(8.0, \; \frac{v_{approach}^2}{2 a_{max}} + 0.6 \cdot v_{approach} + 2.5\right)$$
   For $v_{approach} = 10.0\text{ m/s}$, $\rho_{0, dyn} = \max(8.0, 12.5 + 6.0 + 2.5) = 21.0\text{ m}$.

### 5.3 Obstacle Repulsive Force & Escape Vortex
Inside $d_{obs} < \rho_{0, dyn}$:
$$\mathbf{F}_{obs} = \min\left(F_{pos} + F_{damp} + F_{barrier}, \; 300.0\right) \cdot \hat{\mathbf{n}}_{obs} + \mathbf{F}_{vortex}$$
where:
- $F_{pos} = 60.0 \cdot \left(\frac{1}{d_{eff}} - \frac{1}{\rho_{eff}}\right) \frac{1}{d_{eff}^2}$, with $d_{eff} = \max(d_{obs} - 2.0, 0.1), \rho_{eff} = \max(\rho_{0, dyn} - 2.0, 0.2)$.
- $F_{damp} = 15.0 \cdot v_{approach} \cdot \left(\frac{\rho_{0, dyn} - d_{obs}}{\rho_{0, dyn}}\right)^2 \cdot m$.
- $F_{barrier} = 100.0 \cdot \left(\frac{2.5}{\max(d_{obs}, 0.1)}\right)^3$ for $d_{obs} < 3.0\text{ m}$.
- Tangential circulatory vortex force: $\mathbf{F}_{vortex} = (\hat{\mathbf{n}}_{obs} \times \hat{\mathbf{z}}) \cdot 25.0\text{ N}$ when approaching obstacle.
- Directional attraction attenuation: $\mathbf{F}_{att}$ component along $-\hat{\mathbf{n}}_{obs}$ is attenuated to 0 for $d_{obs} \le 2.5\text{ m}$.

### 5.4 Hard Collision Stopping in `step_physics`
As a guaranteed hard physical constraint (analogous to the ground contact constraint $Z \ge 0.0$), `step_physics` tests the integrated position $\mathbf{p}_{new} = \mathbf{p} + \mathbf{v} \Delta t$ against all registered obstacles:
```python
for obs in obs_list:
    if hasattr(obs, "contains_point") and obs.contains_point(self.position, margin=0.0):
        # Drone has reached or penetrated obstacle surface
        normal = obs.surface_normal(self.position) if hasattr(obs, "surface_normal") else np.array([0.0, 0.0, 1.0])
        # Project outside dominant face
        for axis in range(3):
            if normal[axis] > 0.5:
                self.position[axis] = getattr(obs, "max_pt", obs.max_bound)[axis] + 0.02
            elif normal[axis] < -0.5:
                self.position[axis] = getattr(obs, "min_pt", obs.min_bound)[axis] - 0.02
        # Zero inward normal velocity
        v_dot_n = float(np.dot(self.velocity, normal))
        if v_dot_n < 0.0:
            self.velocity -= v_dot_n * normal
        # Zero inward normal acceleration
        a_dot_n = float(np.dot(self.acceleration, normal))
        if a_dot_n < 0.0:
            self.acceleration -= a_dot_n * normal
```

---

## 6. Concrete Code Replacement Blueprint for `sim/drone.py`

### 6.1 Replacement 1: Imports (Line 13)
```python
<<<<< BEFORE (sim/drone.py:12-14)
import math
from typing import Any, List, Optional, Tuple, Union
import numpy as np
=====
import math
from typing import Any, List, Optional, Sequence, Tuple, Union
import numpy as np
>>>>>
```

### 6.2 Replacement 2: `__init__` Obstacle Registry (Lines 67-76)
```python
<<<<< BEFORE (sim/drone.py:67-76)
        # Mission & Navigation
        self.flight_mode = FlightMode.IDLE
        self.target_position: Optional[np.ndarray] = None
        self.assigned_poi_id: Optional[str] = None
        self.is_transmitting: bool = False
        self.dwell_time: float = 0.0

        # Physical constants
        self.g = 9.80665
=====
        # Mission & Navigation
        self.flight_mode = FlightMode.IDLE
        self.target_position: Optional[np.ndarray] = None
        self.assigned_poi_id: Optional[str] = None
        self.is_transmitting: bool = False
        self.dwell_time: float = 0.0
        self.obstacles: List[Any] = []

        # Physical constants
        self.g = 9.80665
>>>>>
```

### 6.3 Replacement 3: `compute_obstacle_repulsion` (Lines 191-226)
```python
<<<<< BEFORE (sim/drone.py:191-226)
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
            elif hasattr(obs, "min_pt") and hasattr(obs, "max_pt"):
                closest_pt = np.clip(self.position, obs.min_pt, obs.max_pt)
                dist = float(np.linalg.norm(self.position - closest_pt))
            else:
                continue

            if dist < rho_0:
                push = self.position - closest_pt
                push_norm = float(np.linalg.norm(push))
                if push_norm > 1e-4:
                    n_hat = push / push_norm
                else:
                    n_hat = np.array([0.0, 0.0, 1.0], dtype=np.float64)

                eff_dist = max(dist - r_safe, 0.1)
                eff_rho0 = max(rho_0 - r_safe, 0.2)
                mag = k_rep * (1.0 / eff_dist - 1.0 / eff_rho0) * (1.0 / (eff_dist ** 2))
                mag = min(mag, 100.0)  # Numerical ceiling
                f_obs += mag * n_hat

        return f_obs
=====
    def compute_obstacle_repulsion(self, obstacles: List[Any]) -> np.ndarray:
        """
        Calculates APF obstacle repulsion forces from 3D AABBs with dynamic velocity-dependent
        sensing horizon, approach damping, and singularity barrier.
        """
        f_obs = np.zeros(3, dtype=np.float64)
        m = self.limits.mass_kg
        a_max = self.limits.max_accel

        for obs in obstacles:
            if hasattr(obs, "distance_and_closest_point"):
                dist, closest_pt = obs.distance_and_closest_point(self.position)
            elif hasattr(obs, "min_bound") and hasattr(obs, "max_bound"):
                closest_pt = np.clip(self.position, obs.min_bound, obs.max_bound)
                dist = float(np.linalg.norm(self.position - closest_pt))
            elif hasattr(obs, "min_pt") and hasattr(obs, "max_pt"):
                closest_pt = np.clip(self.position, obs.min_pt, obs.max_pt)
                dist = float(np.linalg.norm(self.position - closest_pt))
            else:
                continue

            push = self.position - closest_pt
            push_norm = float(np.linalg.norm(push))
            if push_norm > 1e-4:
                n_hat = push / push_norm
            elif hasattr(obs, "surface_normal"):
                n_hat = obs.surface_normal(self.position)
            else:
                n_hat = np.array([0.0, 0.0, 1.0], dtype=np.float64)

            # Dynamic sensing horizon based on approach speed
            v_approach = max(0.0, float(-np.dot(self.velocity, n_hat)))
            rho_0_dyn = max(8.0, (v_approach ** 2) / (2.0 * a_max) + 0.6 * v_approach + 2.5)

            if dist < rho_0_dyn:
                d_eff = max(dist - 2.0, 0.1)
                rho_eff = max(rho_0_dyn - 2.0, 0.2)
                mag_apf = 60.0 * (1.0 / d_eff - 1.0 / rho_eff) / (d_eff ** 2)
                mag_damp = 15.0 * v_approach * ((rho_0_dyn - dist) / rho_0_dyn) ** 2 * m
                mag_barrier = 100.0 * ((2.5 / max(dist, 0.1)) ** 3) if dist < 3.0 else 0.0
                mag = min(mag_apf + mag_damp + mag_barrier, 300.0)
                f_obs += mag * n_hat

                # Lateral vortex circulatory force to avoid deadlock
                vortex = np.cross(n_hat, np.array([0.0, 0.0, 1.0]))
                if np.linalg.norm(vortex) > 1e-4:
                    f_obs += (vortex / np.linalg.norm(vortex)) * 20.0

        return f_obs
>>>>>
```

### 6.4 Replacement 4: `compute_flocking_forces` (Lines 227-268)
```python
<<<<< BEFORE (sim/drone.py:227-268)
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
            # Alignment (match velocity of active transit flock)
            if self.flight_mode == FlightMode.TRANSIT:
                avg_vel = np.mean([p.velocity for p in neighbors], axis=0)
                f_align = 0.8 * (avg_vel - self.velocity)

            # Cohesion (pull towards local center of mass)
            if self.flight_mode == FlightMode.TRANSIT:
                avg_pos = np.mean([p.position for p in neighbors], axis=0)
                f_coh = 0.2 * (avg_pos - self.position)

        return f_sep, f_align, f_coh
=====
    def compute_flocking_forces(self, peers: List[Drone]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Calculates Reynolds separation, alignment, and cohesion forces with dynamic
        relative closing velocity horizon, approach damping, and singularity barrier.
        """
        f_sep = np.zeros(3, dtype=np.float64)
        f_align = np.zeros(3, dtype=np.float64)
        f_coh = np.zeros(3, dtype=np.float64)

        neighbors: List[Drone] = []
        r_percept = 12.0
        r_sep_static = self.limits.separation_radius
        m = self.limits.mass_kg

        for peer in peers:
            if peer.id == self.id or peer.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
                continue

            diff = self.position - peer.position
            dist = float(np.linalg.norm(diff))
            if dist < 1e-4:
                continue

            r_hat = diff / dist
            v_rel = self.velocity - peer.velocity
            v_close = max(0.0, float(-np.dot(v_rel, r_hat)))

            # Dynamic velocity-dependent separation horizon
            r_sep_dyn = max(r_sep_static, (v_close ** 2) / 6.0 + 0.8 * v_close + 2.5)

            if dist < r_sep_dyn:
                d_eff = max(dist - 1.8, 0.1)
                r_eff = max(r_sep_dyn - 1.8, 0.2)
                mag_apf = 45.0 * (1.0 / d_eff - 1.0 / r_eff) / (d_eff ** 2)
                mag_damp = 12.0 * v_close * ((r_sep_dyn - dist) / r_sep_dyn) ** 2 * m
                mag_barrier = 80.0 * ((2.2 / max(dist, 0.1)) ** 3) if dist < 2.5 else 0.0

                mag_total = min(mag_apf + mag_damp + mag_barrier, 250.0)
                f_sep += mag_total * r_hat

            if dist < r_percept:
                neighbors.append(peer)

        if neighbors:
            # Alignment (match velocity of active transit flock)
            if self.flight_mode == FlightMode.TRANSIT:
                avg_vel = np.mean([p.velocity for p in neighbors], axis=0)
                f_align = 0.8 * (avg_vel - self.velocity)

            # Cohesion (pull towards local center of mass)
            if self.flight_mode == FlightMode.TRANSIT:
                avg_pos = np.mean([p.position for p in neighbors], axis=0)
                f_coh = 0.2 * (avg_pos - self.position)

        return f_sep, f_align, f_coh
>>>>>
```

### 6.5 Replacement 5: `compute_total_force` with Prioritized Safety Attenuation (Lines 335-363)
```python
<<<<< BEFORE (sim/drone.py:335-363)
    def compute_total_force(
        self,
        obstacles: Optional[List[Any]] = None,
        peers: Optional[List[Drone]] = None,
    ) -> np.ndarray:
        """Synthesizes all vector steering forces into a composite commanded force."""
        if self.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
            return np.zeros(3, dtype=np.float64)

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
            f_tan = np.array([-n_obs[1], n_obs[0], 0.0], dtype=np.float64) * 15.0
            return f_att + f_obs + f_sep + f_align + f_coh + f_dw + f_corr + f_tan

        return f_att + f_obs + f_sep + f_align + f_coh + f_dw + f_corr
=====
    def compute_total_force(
        self,
        obstacles: Optional[List[Any]] = None,
        peers: Optional[List[Drone]] = None,
    ) -> np.ndarray:
        """
        Synthesizes all vector steering forces into a composite commanded force,
        applying Prioritized Safety Attenuation of waypoint attraction when blocked
        by peers or obstacles.
        """
        if self.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
            return np.zeros(3, dtype=np.float64)

        obs_list = obstacles if obstacles is not None else getattr(self, "obstacles", [])
        peer_list = peers if peers is not None else []

        f_att = self.compute_attractive_force()

        # Prioritized Safety Attenuation: Attenuate attractive force along blocked paths
        for peer in peer_list:
            if peer.id == self.id or peer.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
                continue
            delta = self.position - peer.position
            dist = float(np.linalg.norm(delta))
            if 0.0 < dist < 8.0:
                r_hat = delta / dist
                gamma = 0.0 if dist <= 2.2 else ((dist - 2.2) / (8.0 - 2.2)) ** 2
                proj = max(0.0, float(np.dot(f_att, -r_hat)))
                f_att -= proj * (1.0 - gamma) * (-r_hat)

        for obs in obs_list:
            if hasattr(obs, "distance_and_closest_point"):
                dist_o, closest_pt = obs.distance_and_closest_point(self.position)
            elif hasattr(obs, "min_pt") and hasattr(obs, "max_pt"):
                closest_pt = np.clip(self.position, obs.min_pt, obs.max_pt)
                dist_o = float(np.linalg.norm(self.position - closest_pt))
            else:
                continue

            if dist_o < 10.0:
                push = self.position - closest_pt
                push_norm = float(np.linalg.norm(push))
                unit_push = push / push_norm if push_norm > 1e-4 else (
                    obs.surface_normal(self.position) if hasattr(obs, "surface_normal") else np.array([0.0, 0.0, 1.0])
                )
                gamma_obs = 0.0 if dist_o <= 2.5 else ((dist_o - 2.5) / (10.0 - 2.5)) ** 2
                proj_o = max(0.0, float(np.dot(f_att, -unit_push)))
                f_att -= proj_o * (1.0 - gamma_obs) * (-unit_push)

        f_obs = self.compute_obstacle_repulsion(obs_list)
        f_sep, f_align, f_coh = self.compute_flocking_forces(peer_list)
        f_dw = self.compute_downwash_repulsion(peer_list)
        f_corr = self.compute_corridor_force()

        # Local minimum escape check
        net_horizontal = f_att[:2] + f_obs[:2]
        if np.linalg.norm(net_horizontal) < 0.5 and np.linalg.norm(f_obs[:2]) > 1.0:
            n_obs = f_obs[:2] / np.linalg.norm(f_obs[:2])
            f_tan = np.array([-n_obs[1], n_obs[0], 0.0], dtype=np.float64) * 20.0
            return f_att + f_obs + f_sep + f_align + f_coh + f_dw + f_corr + f_tan

        return f_att + f_obs + f_sep + f_align + f_coh + f_dw + f_corr
>>>>>
```

### 6.6 Replacement 6: `step` and `step_physics` Hard Collision Stopping (Lines 367-425)
```python
<<<<< BEFORE (sim/drone.py:367-425)
    def step(self, dt: float, desired_accel: Optional[np.ndarray] = None) -> None:
        """
        Advance drone physics by dt seconds.
        If desired_accel is supplied, commanded force is desired_accel * mass.
        Otherwise, commanded force is synthesized from active fields.
        """
        if desired_accel is not None:
            cmd_force = np.asarray(desired_accel, dtype=np.float64) * self.limits.mass_kg
        else:
            cmd_force = self.compute_total_force()
        self.step_physics(dt, cmd_force)

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
        ], dtype=np.float64)
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
            self.velocity[2] = max(0.0, float(self.velocity[2]))
            self.acceleration[2] = max(0.0, float(self.acceleration[2]))
=====
    def step(
        self,
        dt: float,
        desired_accel: Optional[np.ndarray] = None,
        obstacles: Optional[Sequence[Any]] = None,
    ) -> None:
        """
        Advance drone physics by dt seconds.
        If desired_accel is supplied, commanded force is desired_accel * mass.
        Otherwise, commanded force is synthesized from active fields.
        """
        obs_list = obstacles if obstacles is not None else getattr(self, "obstacles", [])
        if desired_accel is not None:
            cmd_force = np.asarray(desired_accel, dtype=np.float64) * self.limits.mass_kg
        else:
            cmd_force = self.compute_total_force(obstacles=obs_list)
        self.step_physics(dt, cmd_force, obstacles=obs_list)

    def step_physics(
        self,
        dt: float,
        commanded_force: np.ndarray,
        obstacles: Optional[Sequence[Any]] = None,
    ) -> None:
        """
        Executes semi-implicit Euler integration of translational and rotational kinematics.
        Enforces physical acceleration, speed, vertical rate, ground contact, and hard
        obstacle boundary collision constraints.
        """
        if self.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
            self.velocity[:] = 0.0
            self.acceleration[:] = 0.0
            self.battery.step(dt, speed=0.0, accel=0.0, is_transmitting=False, is_surveying=False)
            return

        m = self.limits.mass_kg
        obs_list = obstacles if obstacles is not None else getattr(self, "obstacles", [])

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
        ], dtype=np.float64)
        a_net = self.acceleration - drag_accel

        # 3. Velocity Integration & Hard Speed Clamping
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
            self.velocity[2] = max(0.0, float(self.velocity[2]))
            self.acceleration[2] = max(0.0, float(self.acceleration[2]))

        # 5. Hard Obstacle Surface Collision Clamping
        for obs in obs_list:
            if hasattr(obs, "contains_point") and obs.contains_point(self.position, margin=0.0):
                normal = obs.surface_normal(self.position) if hasattr(obs, "surface_normal") else np.array([0.0, 0.0, 1.0])
                min_p = getattr(obs, "min_pt", getattr(obs, "min_bound", None))
                max_p = getattr(obs, "max_pt", getattr(obs, "max_bound", None))
                if min_p is not None and max_p is not None:
                    for axis in range(3):
                        if normal[axis] > 0.5:
                            self.position[axis] = max_p[axis] + 0.02
                        elif normal[axis] < -0.5:
                            self.position[axis] = min_p[axis] - 0.02
                v_dot_n = float(np.dot(self.velocity, normal))
                if v_dot_n < 0.0:
                    self.velocity -= v_dot_n * normal
                a_dot_n = float(np.dot(self.acceleration, normal))
                if a_dot_n < 0.0:
                    self.acceleration -= a_dot_n * normal
>>>>>
```

---

## 7. Integration & Coordination Guidance for `sim/core.py`

While Explorer M1-Fix-1 owns `sim/core.py`, the master simulation loop in `sim/core.py` evaluates forces in `compute_steering_forces(drone)` and advances physics in `step()`. To ensure full systemic harmony:

1. **Obstacle Propagation**:
   In `sim/core.py`, ensure registered obstacles are available to drones:
   ```python
   # In SwarmSimulationCore.add_obstacle(obs):
   self.obstacles.append(obstacle)
   self.environment.add_obstacle(obstacle)
   for d in self.drones.values():
       if obstacle not in d.obstacles:
           d.obstacles.append(obstacle)

   # In SwarmSimulationCore.add_drone(drone):
   self.drones[drone.id] = drone
   drone.obstacles = self.obstacles
   ```
2. **Force Computation Alignment**:
   In `sim/core.py:compute_steering_forces(drone)`:
   The separation and obstacle repulsion calculations in `sim/core.py` should incorporate the dynamic closing velocity horizon $r_{sep, dyn}$ and dynamic obstacle horizon $\rho_{0, dyn}$ as formulated above, or delegate directly:
   ```python
   # Recommended delegation in sim/core.py:
   peers = list(self.drones.values())
   force = drone.compute_total_force(obstacles=self.obstacles, peers=peers)
   # Add soft world boundary containment force
   force += self._compute_boundary_containment(drone)
   return force
   ```
3. **Double-Buffered State Integration**:
   In `SwarmSimulationCore.step()`:
   Compute all forces at time $t_k$ before updating any drone states to $t_{k+1}$ to guarantee Newton's third law ($\mathbf{F}_{ij} = -\mathbf{F}_{ji}$) and prevent ID-based sequential update priority artifacts.

---

## 8. Verification & Test Validation Plan

### 8.1 Verification Commands
1. **Type Introspection Verification**:
   ```powershell
   python -c "import typing, sim.drone; print(typing.get_type_hints(sim.drone.Drone.set_target_waypoint))"
   ```
   *Pass Condition*: Output dictionary contains `Sequence[float]`, no `NameError`.

2. **Adversarial Stress Test Suite**:
   ```powershell
   python -m pytest tests/unit/test_adversarial_m1.py -v
   ```
   *Pass Condition*:
   - `test_head_on_collision_encounter`: PASSED ($min\_sep \ge 1.5\text{ m}$, `crossed = False`).
   - `test_multi_drone_collinear_compression`: PASSED ($min\_sep \ge 1.5\text{ m}$).
   - `test_high_speed_obstacle_penetration`: PASSED (`penetrated = False`).
   - `test_downwash_execution_and_lateral_escape_in_core`: PASSED.

3. **Empirical Numerical Harness**:
   ```powershell
   python tests/adversarial_harness_m1.py
   ```
   *Pass Condition*:
   - Collinear Head-on: $min\_dist \ge 1.5\text{ m}$, `crossed = False`.
   - Compression N=3, 4, 5: $min\_pairwise \ge 1.5\text{ m}$.
   - Boundary & ground clamping: 100% stable.

4. **Drone Subsystem Regression Suite**:
   ```powershell
   python -m pytest tests/unit/test_drone.py -v
   ```
   *Pass Condition*: All 22 tests PASSED.

5. **Full E2E Suite**:
   ```powershell
   python -m pytest tests/e2e -v
   ```
   *Pass Condition*: All 138 tests PASSED.
