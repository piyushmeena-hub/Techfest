# Handoff Report: Challenger M1-2 — Geometry, Scale & Determinism Stress

## 1. Observation

### Obs 1: Fatal `NameError` in `sim/core.py` During Aerodynamic Downwash Calculations
- **File**: `sim/core.py` lines 182–186:
  ```python
  182:                     lateral_dir = delta[:2] / max(d_xy, 1e-3)
  183:                     mag_dw = 35.0 * math.exp(-(d_xy ** 2) / 8.0)
  184:                     f_downwash[0] += lateral_dir[0] * mag_dw
  185:                     f_downwash[1] += lateral_dir[1] * mag_dw
  186:                     f_downwash[2] -= 4.0 * math.exp(-(d_xy ** 2) / 2.0)
  ```
- **Error**: `NameError: name 'math' is not defined. Did you forget to import 'math'?`
- **Trigger**: Occurs in default configuration `SimulationConfig(enable_downwash=True)` whenever drone $i$ is situated underneath drone $j$ within the downwash cone ($-8.0 \le \Delta z \le -0.5\text{ m}$ and $d_{xy} \le |\Delta z| \times 0.4663 + 1.0\text{ m}$).
- **Empirical Execution**:
  Command: `python -c "from sim.core import SwarmSimulationCore, SimulationConfig; from sim.drone import Drone, FlightMode; core = SwarmSimulationCore(SimulationConfig(enable_downwash=True)); d1 = Drone('D1', initial_pos=[0,0,10]); d1.flight_mode = FlightMode.TRANSIT; d2 = Drone('D2', initial_pos=[0,0,15]); d2.flight_mode = FlightMode.TRANSIT; core.add_drone(d1); core.add_drone(d2); core.step(0.05)"`
  Result:
  ```
  Traceback (most recent call last):
    File "D:\drone model\IIT Bombay\sim\core.py", line 295, in step
      force = self.compute_steering_forces(drone)
    File "D:\drone model\IIT Bombay\sim\core.py", line 183, in compute_steering_forces
      mag_dw = 35.0 * math.exp(-(d_xy ** 2) / 8.0)
                      ^^^^
  NameError: name 'math' is not defined. Did you forget to import 'math'?
  ```
  Existing test `tests/unit/test_adversarial_m1.py::TestAdversarialDownwash::test_downwash_zero_lateral_offset_in_core` failed with this exact error.
  In large swarm scaling test (25 drones flying transit trajectories), simulation crashed at tick 79 with this exact traceback.

### Obs 2: Degenerate 3D Ray-AABB Geometry Anomalies in `sim/obstacles.py`
Tested on obstacle box $X, Y, Z \in [0, 10]\text{ m}$:
1. **Vertex Grazing (0-volume contact)**:
   - Ray: $[11, 9, 10] \to [9, 11, 10]$ passes through outer vertex $(10, 10, 10)$ at $t=0.5$.
   - Output: `hit=True, t_enter=0.5000, t_exit=0.5000, penetration=0.0000m, attenuation=22.00dB`.
   - `los_clear=False`.
   - The ray never enters the interior volume, yet is marked as occluded and penalized with full 22.0 dB base attenuation.
2. **Coplanar Face Grazing Discontinuity**:
   - Ray: $[-5, 5, 10] \to [15, 5, 10]$ skimming outer rooftop face at $z=10.0\text{ m}$.
   - Output: `hit=True, penetration=10.0000m, attenuation=37.00dB`, `los_clear=False`.
   - Perturbed by $+10^{-10}\text{ m}$ (0.1 nanometer outside plane): `hit=False, penetration=0.0000m, attenuation=0.00dB`, `los_clear=True`.
   - Step discontinuity of 37.0 dB across an infinitesimal distance.
3. **Surface Origin Pointing Outward**:
   - Ray starting at $x=0.0$ on exterior boundary pointing outward into open space $[-10, 5, 5]$:
   - Output: `hit=True, penetration=0.0000m, attenuation=22.00dB`, `los_clear=False`.
   - Ray starting on outer wall pointing away from building is marked occluded.
4. **Zero-Length Rays**:
   - Inside box $[5, 5, 5]$: `hit=True, los_clear=False, pen=0.0m, att=22.0dB`.
   - Outside box $[20, 20, 20]$: `hit=False, los_clear=True, pen=0.0m, att=0.0dB`.
   - On boundary $[10, 5, 5]$: `hit=True, los_clear=False, pen=0.0m, att=22.0dB`.

### Obs 3: Vectorized vs Scalar Numerical Consistency Across 10,000 Adversarial Rays
Tested on `ObstacleManager` with 8 disaster obstacles using 10,000 rays (7,000 theater-wide random, 2,000 obstacle-targeted hits, 500 axis-aligned grazing, 500 near-zero length):
- **Boolean LoS Match**: 10,000 / 10,000 (100.000%).
- **Boolean Mismatches**: 0.
- **Penetration Distance Max Difference**: $1.448664 \times 10^{-10}\text{ m}$.
- **Penetration Distance Mean Difference**: $3.345638 \times 10^{-13}\text{ m}$.
- **Attenuation Max Difference**: $2.476455 \times 10^{-10}\text{ dB}$.
- **Attenuation Mean Difference**: $5.125187 \times 10^{-13}\text{ dB}$.
- **Scalar Runtime**: $0.3177\text{ s}$ ($31,478.9\text{ rays/sec}$).
- **Vectorized Runtime**: $0.0135\text{ s}$ ($739,426.2\text{ rays/sec}$).
- **Speedup Factor**: $23.49\times$.

### Obs 4: Large Swarm Scaling Benchmark (25 and 50 Drones, 500 Ticks)
Benchmark executed with `sim.core.math` made available:
- **25 Drones (500 ticks, 25.0s sim duration)**:
  - Total Wall Time: $2.418\text{ s}$
  - Mean Step Time: $4.83\text{ ms}$ (P95: $5.37\text{ ms}$, Max: $11.10\text{ ms}$)
  - Effective Simulation Rate: $206.8\text{ Hz}$ ($10.34\times$ faster than 20 Hz real-time)
  - Numerical Stability: PASS (All Finite, 0 NaNs, 0 Infs)
  - Max Speed Observed: $10.31\text{ m/s}$ (clamped near $10.0\text{ m/s}$ limit)
  - Max Accel Observed: $4.00\text{ m/s}^2$ (strictly clamped at $4.00\text{ m/s}^2$)
- **50 Drones (500 ticks, 25.0s sim duration)**:
  - Total Wall Time: $7.300\text{ s}$
  - Mean Step Time: $14.60\text{ ms}$ (P95: $16.83\text{ ms}$, Max: $49.70\text{ ms}$)
  - Effective Simulation Rate: $68.5\text{ Hz}$ ($3.42\times$ faster than 20 Hz real-time)
  - Scaling Ratio ($50\text{ vs }25$ drones): $3.02\times$ increase in runtime for $4.08\times$ increase in pairwise comparisons ($O(N^2)$)
  - Numerical Stability: PASS (All Finite)
  - Max Accel Observed: $4.00\text{ m/s}^2$

### Obs 5: Multi-Process Determinism
Multi-agent simulation (10 drones, 8 obstacles, VSM relays, APF steering, 200 ticks) executed in two completely independent Python child processes via `subprocess.run`:
- State snapshots evaluated at ticks 0, 50, 100, 199 across all 10 drones:
  - Max coordinate diff ($X, Y, Z$): $0.0000000000\text{e}+00$.
  - Max velocity diff ($Vx, Vy, Vz$): $0.0000000000\text{e}+00$.
  - Bit-for-bit JSON equality: `True`.

---

## 2. Logic Chain

1. **Premise 1**: A core simulation module cannot have missing standard library imports in its active execution branches.
   - *Supported by Obs 1*: `sim/core.py:183,186` references `math.exp()` without `import math`. When `enable_downwash=True` (default), multi-drone swarms crash with fatal `NameError`.
   - *Inference*: SwarmSimulationCore cannot reliably run swarms in its default configuration without throwing unhandled exceptions. This blocks milestone sign-off until patched.

2. **Premise 2**: 3D Ray-AABB intersection for RF line-of-sight must distinguish between passing through solid material and grazing outer boundaries in free air.
   - *Supported by Obs 2*: In `sim/obstacles.py:176`, `hit = (t_enter <= t_exit) and (t_exit >= 0.0) and (t_enter <= 1.0)` allows equality `t_enter == t_exit`, classifying zero-depth vertex grazes as hits. Furthermore, parallel face grazing computes `(t_out - t_in) * length`, treating exterior face traversal as solid concrete penetration.
   - *Inference*: While safe for conservative collision avoidance, for RF path loss this causes phantom 22–37 dB attenuations and broken line-of-sight when drones fly along building rooftops or past building corners.

3. **Premise 3**: Vectorized batch operations must be numerically identical to scalar reference implementations.
   - *Supported by Obs 3*: Across 10,000 rays, boolean matches are 100%, and maximum coordinate differences are below $1.5 \times 10^{-10}\text{ m}$.
   - *Inference*: The vectorized ray-slab implementation in `ObstacleManager.check_los_batch` is mathematically sound, robust, and provides a $23.5\times$ acceleration with zero numerical drift.

4. **Premise 4**: Scalability and determinism requirements dictate real-time execution (> 20 Hz) at large fleet scale and bit-level cross-process reproducibility.
   - *Supported by Obs 4 & Obs 5*: 25 drones run at 206.8 Hz, 50 drones run at 68.5 Hz (> 3x real-time), and multi-process runs produce zero floating-point divergence.
   - *Inference*: The simulation kinematics, Reynolds flocking, and VSM algorithms scale gracefully and maintain strict determinism.

---

## 3. Caveats

- **Obstacle Representation**: Tested exclusively against 3D Axis-Aligned Bounding Boxes (AABBs) as defined in M1-2 specifications. Oriented Bounding Boxes (OBB) or polygonal meshes were not evaluated.
- **Dynamic Obstacles**: All obstacles tested were static structures per disaster theater layout requirements.
- **Downwash Scaling Bypass**: During the 500-tick scaling benchmarks, `sim.core.math = math` was supplied in-memory to evaluate true physics throughput without the unhandled `NameError`.

---

## 4. Conclusion & Verdict

**Final Verdict: REJECT**

### Blocking Defect
- **CRITICAL**: Missing `import math` in `sim/core.py`.
  - Line 183: `mag_dw = 35.0 * math.exp(-(d_xy ** 2) / 8.0)`
  - Line 186: `f_downwash[2] -= 4.0 * math.exp(-(d_xy ** 2) / 2.0)`
  - **Resolution**: Add `import math` to the top-level imports of `sim/core.py`.

### Non-Blocking Recommendations (Geometry Engine)
- **MEDIUM**: Degenerate ray grazing handling in `sim/obstacles.py`:
  - When `t_exit <= t_enter + 1e-6` (tangential graze) or ray points outward from boundary surface (`dot(d, normal) > 0`), set `hit = False` or clamp attenuation to 0.0 dB to prevent phantom 22–37 dB attenuation spikes for signals in open air.

---

## 5. Verification Method

### 1. Reproduce the Downwash NameError Crash
Execute:
```bash
python -c "from sim.core import SwarmSimulationCore, SimulationConfig; from sim.drone import Drone, FlightMode; core = SwarmSimulationCore(SimulationConfig(enable_downwash=True)); d1 = Drone('D1', initial_pos=[0,0,10]); d1.flight_mode = FlightMode.TRANSIT; d2 = Drone('D2', initial_pos=[0,0,15]); d2.flight_mode = FlightMode.TRANSIT; core.add_drone(d1); core.add_drone(d2); core.step(0.05)"
```
Expected output: `NameError: name 'math' is not defined`.

### 2. Run the Full Challenger Stress Harness
Execute:
```bash
python tests/stress_geometry_scale_determinism.py
```
Verifies degenerate geometry, 10,000-ray vector vs scalar comparison (100% match), 25/50 drone benchmarks, and cross-process determinism.

### 3. Run Pytest Suite
Execute:
```bash
pytest tests/unit/test_challenger_m1_2.py
```
Expected output: 8 passed.

---

## Adversarial Challenge Report

### Challenge Summary
- **Overall Risk Assessment**: **CRITICAL** (due to fatal NameError crash on default downwash settings).

### Challenge Matrix
| Level | Challenge | Challenged Item | Finding | Mitigation |
|---|---|---|---|---|
| **CRITICAL** | Aerodynamic downwash code path | `sim/core.py:183` | Missing `import math` crashes any multi-drone simulation when one drone flies below another. | Add `import math` at top of `sim/core.py`. |
| **MEDIUM** | Vertex & edge grazing | `sim/obstacles.py:176` | Rays grazing corners in open air are penalized with 22.0 dB loss. | Require `t_exit > t_enter + 1e-6` for hit classification. |
| **MEDIUM** | Coplanar face grazing | `sim/obstacles.py:183` | Rays along exterior rooftop face assessed 10m solid penetration (37 dB). | Check ray alignment with surface normal or require interior penetration. |
| **MEDIUM** | Outward-directed surface rays | `sim/obstacles.py:176` | Transmitters on building surfaces pointing into open air are marked occluded. | Check ray direction relative to surface normal. |
| **LOW** | Vectorized vs scalar divergence | `sim/obstacles.py:336` | 10,000 rays tested across random, targeted, and axis-aligned configurations. | 100% boolean match, max diff $< 1.5 \times 10^{-10}\text{ m}$. Robust. |
| **LOW** | Swarm scalability | `sim/core.py:274` | 25 and 50 drones running 500 ticks benchmarked. | Exceeds real-time target: 206.8 Hz (25 drones), 68.5 Hz (50 drones). |
| **LOW** | Determinism | `sim/core.py:291` | Cross-process multi-agent runs tested. | Identical floating-point state trajectories ($\Delta = 0.0$). |
