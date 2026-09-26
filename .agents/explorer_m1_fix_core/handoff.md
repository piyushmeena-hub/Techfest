# Handoff Report: Explorer M1-Fix-1 (Core Simulation & Downwash Remediation)

**Role**: Explorer M1-Fix-1 (Core Simulation & Downwash Remediation Specialist)  
**Working Directory**: `d:\drone model\IIT Bombay\.agents\explorer_m1_fix_core`  
**Recipient**: Parent Orchestrator (`4ad727ae-0330-41e2-8017-656bde75909d`)  
**Status**: Task Complete (Hard Handoff)  

---

## 1. Observation

### 1.1 Unresolved Global `math` in `sim/core.py:183, 186`
- **File**: `sim/core.py` lines 181–187:
  ```python
  181:                 if -8.0 <= dz <= -0.5 and d_xy <= (abs(dz) * 0.4663 + 1.0):
  182:                     lateral_dir = delta[:2] / max(d_xy, 1e-3)
  183:                     mag_dw = 35.0 * math.exp(-(d_xy ** 2) / 8.0)
  184:                     f_downwash[0] += lateral_dir[0] * mag_dw
  185:                     f_downwash[1] += lateral_dir[1] * mag_dw
  186:                     f_downwash[2] -= 4.0 * math.exp(-(d_xy ** 2) / 2.0)
  ```
- **Bytecode Disassembly Observation**:
  ```powershell
  python -c "import dis, sim.core; missing = [i.argval for i in dis.get_instructions(sim.core.SwarmSimulationCore.compute_steering_forces) if i.opname == 'LOAD_GLOBAL' and i.argval not in sim.core.__dict__ and i.argval not in __builtins__.__dict__]; print(missing)"
  ```
  Result: `['math', 'math']`.
- **Top-Level Module Imports in `sim/core.py:1-24`**:
  Omitted `import math` completely (only imported `deque`, `dataclass`, `json`, `typing`, `np`, `Drone`, `DisasterEnvironment`, `types`).
- **Verbatim Error**:
  Executing `pytest tests/unit/test_adversarial_m1.py::TestAdversarialDownwash::test_downwash_execution_and_lateral_escape_in_core`:
  ```
  sim\core.py:183: in compute_steering_forces
      mag_dw = 35.0 * math.exp(-(d_xy ** 2) / 8.0)
  E   NameError: name 'math' is not defined. Did you forget to import 'math'?
  ```

### 1.2 Zero-Offset Lateral Escape Vanishing Degeneracy
- **Code**: `lateral_dir = delta[:2] / max(d_xy, 1e-3)` at `sim/core.py:182`.
- **Observation**:
  When drone $i$ is directly underneath drone $j$ ($d_{xy} = 0.0$ m, e.g. drone $j$ at `[0, 0, 35]`, drone $i$ at `[0, 0, 30]`):
  - `delta[:2] = [0.0, 0.0]`.
  - `lateral_dir = [0.0, 0.0] / 1e-3 = [0.0, 0.0]`.
  - `f_downwash[:2] = [0.0, 0.0] * mag_dw = [0.0, 0.0]`.
  - `f_downwash[2] = -4.0 * math.exp(0) = -4.0 N`.
- **Empirical Execution**:
  With `math` injected, running concentric drones:
  ```python
  forces_bot = core.compute_steering_forces(d_bot)
  # forces_bot yields [0.0, 0.0, -4.04]
  ```
  `np.linalg.norm(forces_bot[:2]) == 0.0`.
  Assertion `assert lat_force_mag > 0.1` failed:
  ```
  AssertionError: Zero lateral escape force on lower drone: [ 0.    0.   -4.04]
  ```
  The lower drone is sucked downward by $-4.0\text{ N}$ with zero lateral force to escape the column.

### 1.3 Test Coverage Masking in `tests/unit/test_sim_core.py:160`
- **Observation**:
  `test_separation_and_downwash()` in `tests/unit/test_sim_core.py:160`:
  ```python
  d1 = Drone("UAV_1", initial_position=np.array([0.0, 0.0, 30.0]))
  d2 = Drone("UAV_2", initial_position=np.array([1.5, 0.0, 30.0]))
  ```
  Both drones were coplanar ($z = 30.0\text{ m} \implies dz = 0.0\text{ m}$).
  The conditional branch `if -8.0 <= dz <= -0.5` on line 181 was never reached. The test only validated horizontal separation, leaving downwash completely unexecuted and concealing the unimported `math` module during normal test suite execution.

---

## 2. Logic Chain

1. **Step 1 (Root Cause of NameError)**:
   In `sim/core.py:183, 186`, `math.exp` is invoked. Because `import math` was omitted from module imports (Obs 1.1), any execution path where $-8.0 \le dz \le -0.5$ and $d_{xy} \le |dz| \times 0.4663 + 1.0$ crashes with `NameError`.
2. **Step 2 (Root Cause of Zero Lateral Escape)**:
   The author used `max(d_xy, 1e-3)` as a divisor for `delta[:2]`. However, when $d_{xy} \to 0$, $\|\delta_{xy}\| \to 0$ simultaneously. Thus, for concentric drones ($d_{xy} = 0$), the direction vector is `[0.0, 0.0] / 1e-3 = [0.0, 0.0]`, completely eliminating horizontal repulsion (Obs 1.2).
3. **Step 3 (Symmetry Breaking Solution)**:
   Per `sim/drone.py:297` (`compute_downwash_repulsion`), when $d_{xy} \le 10^{-3}\text{ m}$, rotational symmetry must be broken by setting `lateral_dir = np.array([1.0, 0.0], dtype=np.float64)`. This guarantees an immediate $+35.0\text{ N}$ lateral nudge along $+X$. Within 1 timestep ($0.05\text{ s}$), the lower drone moves $\approx 3.6\text{ cm}$ horizontally, after which standard radial unit normalization `delta[:2] / d_xy` guides it out of the downwash cylinder.
4. **Step 4 (Test Remediation Necessity)**:
   Because `tests/unit/test_sim_core.py` masked lines 181–186 due to $dz = 0.0$ (Obs 1.3), unit tests explicitly exercising concentric vertical separation ($dz = -4.0\text{ m}$), offset downwash ($dz = -4.0\text{ m}, dx = 1.0\text{ m}$), outside-cone immunity ($dx = 7.0\text{ m}$), and configuration disabling (`enable_downwash=False`) are necessary to permanently prevent regression.

---

## 3. Caveats

- **Scope Boundary**: This investigation focuses strictly on `sim/core.py` downwash mechanics and `tests/unit/test_sim_core.py`. Other issues flagged by Auditor M1 (such as `sim/drone.py` missing `Sequence` typing import and high-speed obstacle tunneling) are handled by peer specialists (Explorer M1-Fix-Dyn and Explorer M1-Fix-Geom).
- **Escape Vector Convention**: Setting `lateral_dir = [1.0, 0.0]` deterministically breaks symmetry along $+X$. This convention is identical to `sim/drone.py:297`, ensuring consistent behavior across both swarm-level and drone-level physics engines.

---

## 4. Conclusion

- **Assessment**: The defect is completely understood, mathematically characterized, and verified via independent prototype execution.
- **Deliverables Produced**:
  1. `core_remediation_spec.md`: Full architectural and mathematical specification.
  2. `proposed_core_remediation.patch`: Unified diff patch ready for implementation in `sim/core.py`.
  3. `proposed_test_sim_core_addition.py`: 5 standalone pytest test cases covering downwash concentric separation, offset radial push, cone boundary limits, configuration disabling, and dynamic escape.

---

## 5. Verification Method

### 5.1 Verification Commands
Once the implementer applies the changes:
1. **Pytest Unit Test Suite**:
   ```powershell
   python -m pytest tests/unit/test_sim_core.py tests/unit/test_adversarial_m1.py::TestAdversarialDownwash -v
   ```
   *Expected Result*: All tests pass with 0 failures.
2. **Bytecode Global Load Audit**:
   ```powershell
   python -c "import dis, sim.core; missing = [i.argval for i in dis.get_instructions(sim.core.SwarmSimulationCore.compute_steering_forces) if i.opname == 'LOAD_GLOBAL' and i.argval not in sim.core.__dict__ and i.argval not in __builtins__.__dict__]; print('Missing:', missing); assert len(missing) == 0"
   ```
   *Expected Result*: `Missing: []`.
3. **Dynamic Lateral Escape Test**:
   ```powershell
   python -c "from sim.core import SwarmSimulationCore, SimulationConfig; from sim.drone import Drone, FlightMode; import numpy as np; core = SwarmSimulationCore(SimulationConfig(enable_downwash=True)); d1 = Drone('TOP', initial_pos=[0,0,35]); d2 = Drone('BOT', initial_pos=[0,0,31]); d1.set_flight_mode(FlightMode.SURVEYING); d2.set_flight_mode(FlightMode.SURVEYING); core.add_drone(d1); core.add_drone(d2); [core.step(0.05) for _ in range(20)]; assert d2.position[0] > 1.0; print('Dynamic escape verified! Final x:', d2.position[0])"
   ```
   *Expected Result*: `Dynamic escape verified! Final x: 2.007...`.

### 5.2 Invalidation Conditions
- If `SwarmSimulationCore.compute_steering_forces()` produces `[0.0, 0.0, ...]` lateral force when two drones have identical $(x, y)$ coordinates and $-8.0 \le dz \le -0.5$.
- If `LOAD_GLOBAL` on `sim.core.SwarmSimulationCore.compute_steering_forces` references `math` without `math` existing in `sim.core.__dict__`.
