# Remediation Specification: Simulation Core & Aerodynamic Downwash Engine

**Module**: `sim/core.py`  
**Test Suite**: `tests/unit/test_sim_core.py`  
**Author**: Explorer M1-Fix-1 (Core Simulation & Downwash Remediation Specialist)  
**Date**: 2026-09-25  
**Target Milestone**: M1 Remediation  

---

## 1. Executive Summary & Root Cause Analysis

During Milestone 1 audit and adversarial verification, two critical defects were uncovered in `sim/core.py`, along with a test masking vulnerability in `tests/unit/test_sim_core.py`:

1. **Fatal NameError (`sim/core.py:183, 186`)**:
   `SwarmSimulationCore.compute_steering_forces()` invokes `math.exp()` when computing aerodynamic downwash cone avoidance forces. However, `import math` is completely omitted from the module imports of `sim/core.py`. Any multi-drone simulation where one drone is positioned below another in the downwash cone triggers `NameError: name 'math' is not defined`.
2. **Zero-Offset Lateral Escape Vanishing Degeneracy (`sim/core.py:182`)**:
   When drone $i$ is situated directly beneath another drone $j$ ($d_{xy} < 10^{-3}\text{ m}$), the current code calculates:
   ```python
   lateral_dir = delta[:2] / max(d_xy, 1e-3)
   ```
   Because `delta[:2] = [0.0, 0.0]`, dividing by `1e-3` yields `[0.0, 0.0]`. The lateral escape force vector vanishes identically (`f_downwash[:2] == [0.0, 0.0]`), while downward suction is applied (`f_downwash[2] -= 4.0`). Consequently, the lower drone experiences zero horizontal force to escape the column and is pinned underneath the upper drone indefinitely.
3. **Test Masking in Baseline Suite (`tests/unit/test_sim_core.py:160`)**:
   The baseline unit test `test_separation_and_downwash()` placed both UAVs at identical altitude $z = 30.0\text{ m}$ ($dz = 0.0$). The downwash cone condition `if -8.0 <= dz <= -0.5` was never entered, leaving lines 181–186 completely unexecuted during baseline test passes.

---

## 2. Mathematical Physics & Downwash Model

### 2.1 Aerodynamic Downwash Cone Geometry
Quadcopter rotor downwash generates a turbulent, high-velocity slipstream cone beneath the aircraft. In `sim/core.py` and `sim/drone.py`:
- **Opening Half-Angle**: $\theta = 25^\circ \implies \tan(25^\circ) \approx 0.466307658$.
- **Effective Vertical Zone**: $-8.0\text{ m} \le \Delta z \le -0.5\text{ m}$ (where $\Delta z = z_i - z_j$, meaning drone $i$ is below drone $j$).
- **Cone Radius Boundary**:
  $$r_{cone}(\Delta z) = |\Delta z| \tan(25^\circ) + r_0 = |\Delta z| \times 0.4663 + 1.0\text{ m}$$
  where $r_0 = 1.0\text{ m}$ represents the initial rotor wake radius at the disk.

### 2.2 Force Field Formulation
When drone $i$ is within the cone ($d_{xy} \le r_{cone}$ and $-8.0 \le \Delta z \le -0.5$):
- **Lateral Repulsion Magnitude**:
  $$F_{xy}^{dw}(d_{xy}) = F_{max}^{dw} \exp\left(-\frac{d_{xy}^2}{2 \sigma_{dw}^2}\right)$$
  In `sim/core.py`, $F_{max}^{dw} = 35.0\text{ N}$ and $2 \sigma_{dw}^2 = 8.0 \implies \sigma_{dw} = 2.0\text{ m}$.
- **Downward Suction/Sink Force**:
  $$F_z^{dw}(d_{xy}) = -4.0 \exp\left(-\frac{d_{xy}^2}{2.0}\right)\text{ N}$$
- **Lateral Unit Direction Vector**:
  $$\hat{\mathbf{n}}_{xy} = \begin{cases} \frac{\boldsymbol{\delta}_{xy}}{\|\boldsymbol{\delta}_{xy}\|}, & \text{if } d_{xy} > 10^{-3}\text{ m} \\ [1.0, 0.0]^T, & \text{if } d_{xy} \le 10^{-3}\text{ m} \text{ (deterministic lateral nudge)} \end{cases}$$

Setting $\hat{\mathbf{n}}_{xy} = [1.0, 0.0]^T$ when $d_{xy} \le 10^{-3}\text{ m}$ breaks the rotational symmetry deterministically, imparting a $35.0\text{ N}$ force along the $+X$ axis. Within a single simulation step ($dt = 0.05\text{ s}$, $m = 1.2\text{ kg}$), the lower drone accelerates at $29.17\text{ m/s}^2$, displacing by $\approx 3.6\text{ cm}$, instantly breaking the singularity and establishing a well-conditioned radial escape vector for subsequent time steps.

---

## 3. Concrete Code Changes in `sim/core.py`

### 3.1 Change 1: Add `import math` to Top-Level Imports
**File**: `sim/core.py`  
**Location**: Lines 9–16  

#### Before:
```python
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union
import numpy as np
```

#### After:
```python
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union
import numpy as np
```

### 3.2 Change 2: Deterministic Lateral Nudge on Concentric Alignment
**File**: `sim/core.py`  
**Location**: Lines 176–187  

#### Before:
```python
            # Aerodynamic downwash cone avoidance
            if self.config.enable_downwash:
                dz = pos_i[2] - other.position[2]
                d_xy = float(np.linalg.norm(delta[:2]))
                # If drone_i is below other drone within 25-degree opening cone
                if -8.0 <= dz <= -0.5 and d_xy <= (abs(dz) * 0.4663 + 1.0):
                    lateral_dir = delta[:2] / max(d_xy, 1e-3)
                    mag_dw = 35.0 * math.exp(-(d_xy ** 2) / 8.0)
                    f_downwash[0] += lateral_dir[0] * mag_dw
                    f_downwash[1] += lateral_dir[1] * mag_dw
                    f_downwash[2] -= 4.0 * math.exp(-(d_xy ** 2) / 2.0)
```

#### After:
```python
            # Aerodynamic downwash cone avoidance
            if self.config.enable_downwash:
                dz = pos_i[2] - other.position[2]
                d_xy = float(np.linalg.norm(delta[:2]))
                # If drone_i is below other drone within 25-degree opening cone
                if -8.0 <= dz <= -0.5 and d_xy <= (abs(dz) * 0.4663 + 1.0):
                    if d_xy > 1e-3:
                        lateral_dir = delta[:2] / d_xy
                    else:
                        # Deterministic non-zero lateral nudge to break concentric column lock
                        lateral_dir = np.array([1.0, 0.0], dtype=np.float64)
                    mag_dw = 35.0 * math.exp(-(d_xy ** 2) / 8.0)
                    f_downwash[0] += lateral_dir[0] * mag_dw
                    f_downwash[1] += lateral_dir[1] * mag_dw
                    f_downwash[2] -= 4.0 * math.exp(-(d_xy ** 2) / 2.0)
```

---

## 4. Unit Test Remediation Blueprint (`tests/unit/test_sim_core.py`)

### 4.1 Defect in Existing Test Suite
`test_separation_and_downwash()` in `tests/unit/test_sim_core.py` was titled with "downwash", yet only placed drones at $z=30.0$:
```python
d1 = Drone("UAV_1", initial_position=np.array([0.0, 0.0, 30.0]))
d2 = Drone("UAV_2", initial_position=np.array([1.5, 0.0, 30.0]))
```
Because $dz = 0.0$, the downwash block was completely unexercised.

### 4.2 Proposed New Unit Tests
Add the following comprehensive tests to `tests/unit/test_sim_core.py`:

```python
def test_downwash_vertical_separation_concentric():
    """
    Verify downwash cone behavior in SwarmSimulationCore when lower drone is
    directly beneath upper drone (dz = -4.0m, d_xy = 0.0m).
    
    Validates:
    1. Execution does not throw NameError (verifies import math).
    2. Lower drone receives non-zero deterministic lateral escape force (> 10.0 N) along +X.
    3. Lower drone receives downward turbulence sink force (fz < 0.0 N).
    4. Upper drone experiences 0.0 downwash force (asymmetric aerodynamic wash).
    """
    core = SwarmSimulationCore(SimulationConfig(enable_downwash=True))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    d_bot = Drone("UAV_BOT", initial_position=np.array([0.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    # Compute steering forces
    f_bot = core.compute_steering_forces(d_bot)
    f_top = core.compute_steering_forces(d_top)

    # Lower drone: lateral escape along +X must be non-zero and substantial
    assert f_bot[0] > 10.0, f"Lower drone did not receive sufficient lateral escape force: {f_bot}"
    # Lower drone: downward sink force must be negative
    assert f_bot[2] < 0.0, f"Lower drone did not receive downward downwash sink: {f_bot}"

    # Upper drone: must feel zero downwash (only separation force upwards if within 6m)
    # At 4m distance, separation force on upper drone is along +Z (pos_top - pos_bot = +4m)
    # Downwash component on upper drone must be 0
    assert f_top[0] == pytest.approx(0.0, abs=1e-5)
    assert f_top[1] == pytest.approx(0.0, abs=1e-5)


def test_downwash_with_lateral_offset_in_core():
    """
    Verify downwash radial push on lower drone when laterally offset inside cone
    (dz = -4.0m, dx = 1.0m, cone radius = 4 * 0.4663 + 1.0 = 2.865m).
    """
    core = SwarmSimulationCore(SimulationConfig(enable_downwash=True))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    d_bot = Drone("UAV_BOT", initial_position=np.array([1.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    f_bot = core.compute_steering_forces(d_bot)
    # Lower drone must be pushed outward along +X away from (0, 0)
    assert f_bot[0] > 0.0
    assert f_bot[2] < 0.0


def test_downwash_outside_cone_in_core():
    """
    Verify that when lower drone is outside the 25-degree downwash cone
    (dz = -4.0m, dx = 5.0m > r_cone = 2.865m), downwash force is 0.
    """
    core = SwarmSimulationCore(SimulationConfig(enable_downwash=True))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    # Place drone at 7m distance (outside 6m separation zone and outside downwash cone)
    d_bot = Drone("UAV_BOT", initial_position=np.array([7.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    f_bot = core.compute_steering_forces(d_bot)
    assert np.allclose(f_bot, 0.0)


def test_downwash_disabled_configuration():
    """Verify that when enable_downwash=False, no downwash force is computed."""
    core = SwarmSimulationCore(SimulationConfig(enable_downwash=False))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    d_bot = Drone("UAV_BOT", initial_position=np.array([0.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    f_bot = core.compute_steering_forces(d_bot)
    # Only vertical separation force exists (along -Z), zero lateral force
    assert f_bot[0] == pytest.approx(0.0, abs=1e-6)
    assert f_bot[1] == pytest.approx(0.0, abs=1e-6)


def test_downwash_dynamic_lateral_escape_step():
    """
    Verify full dynamic simulation step: lower drone escapes the concentric
    downwash column across successive simulation steps.
    """
    core = SwarmSimulationCore(SimulationConfig(dt=0.05, enable_downwash=True))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    d_bot = Drone("UAV_BOT", initial_position=np.array([0.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    initial_x = d_bot.position[0]
    for _ in range(20):  # 1.0 second simulation
        core.step(0.05)

    # Lower drone must have moved significantly along +X (> 1.0 meter)
    assert d_bot.position[0] - initial_x > 1.0, f"Lower drone failed to dynamically escape downwash cone: {d_bot.position}"
```

---

## 5. Verification & Invalidation Protocol

### 5.1 Static Verification: Bytecode Disassembly
Verify that zero undefined global names exist in `SwarmSimulationCore.compute_steering_forces`:
```powershell
python -c "import dis, sim.core; missing = [i.argval for i in dis.get_instructions(sim.core.SwarmSimulationCore.compute_steering_forces) if i.opname == 'LOAD_GLOBAL' and i.argval not in sim.core.__dict__ and i.argval not in __builtins__.__dict__]; print('Missing:', missing); assert len(missing) == 0"
```
*Expected*: `Missing: []`.

### 5.2 Dynamic Unit Test Execution
Execute the full unit test suite:
```powershell
python -m pytest tests/unit/test_sim_core.py tests/unit/test_adversarial_m1.py::TestAdversarialDownwash -v
```
*Expected*: All tests pass with 0 failures, 0 errors, 0 warnings.

### 5.3 Stress Test Execution
Run the Challenger M1-2 stress test:
```powershell
python -c "from sim.core import SwarmSimulationCore, SimulationConfig; from sim.drone import Drone, FlightMode; core = SwarmSimulationCore(SimulationConfig(enable_downwash=True)); d1 = Drone('D1', initial_pos=[0,0,10]); d1.flight_mode = FlightMode.TRANSIT; d2 = Drone('D2', initial_pos=[0,0,15]); d2.flight_mode = FlightMode.TRANSIT; core.add_drone(d1); core.add_drone(d2); core.step(0.05); print('Downwash step passed without error')"
```
*Expected*: Prints `Downwash step passed without error`.

---

## 6. Implementer Checklist

- [ ] Apply `import math` to `sim/core.py:13`.
- [ ] Update downwash `lateral_dir` branching in `sim/core.py:182-187`.
- [ ] Add the 5 new downwash unit tests to `tests/unit/test_sim_core.py`.
- [ ] Run `python -m pytest tests/unit/test_sim_core.py -v`.
- [ ] Run `python -m pytest tests/unit/test_adversarial_m1.py -k downwash -v`.
- [ ] Confirm no regression across other modules.
