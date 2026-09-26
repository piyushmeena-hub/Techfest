# Handoff Report: Explorer M1-Fix-3 — Ray-AABB Geometry & Grazing Remediation

**Agent**: Explorer M1-Fix-3 (`teamwork_preview_explorer` / Specialist)  
**Working Directory**: `d:\drone model\IIT Bombay\.agents\explorer_m1_fix_geom`  
**Date**: 2026-09-25T15:00:00Z  
**Type**: Hard Handoff (Investigation & Specification Complete)  
**Recipient**: Parent Orchestrator (`4ad727ae-0330-41e2-8017-656bde75909d`)  

---

## 1. Observation

### 1.1 Direct Observation of Flawed Code Paths in `sim/obstacles.py`

1. **Vertex and Edge Grazing (Zero Penetration False Hits)**:
   - **File**: `sim/obstacles.py`, lines 176 and 316.
   - **Code**:
     ```python
     176: hit = (t_enter <= t_exit) and (t_exit >= 0.0) and (t_enter <= 1.0)
     ```
   - **Observed**: For ray $[11.0, 9.0, 10.0] \to [9.0, 11.0, 10.0]$ passing through outer corner vertex $(10.0, 10.0, 10.0)$ of box $[0, 10]^3$:
     - $t_{enter} = 0.5000$, $t_{exit} = 0.5000$.
     - $t_{enter} \le t_{exit}$ evaluates to `True`.
     - $d_{pen} = (0.5 - 0.5) \times \sqrt{8} = 0.0\text{ m}$.
     - Output: `hit=True`, `penetration=0.0000m`, `attenuation=22.00dB`, `los_clear=False`.
     - Direct verbatim test command:
       ```powershell
       python -c "from sim.obstacles import ObstacleAABB; import numpy as np; box = ObstacleAABB('B', 'Box', np.zeros(3), np.full(3, 10.0)); res = box.intersect_ray_segment(np.array([11., 9., 10.]), np.array([9., 11., 10.])); print(res)"
       ```
     - Output: `RayIntersectionResult(hit=True, t_enter=0.5, t_exit=0.5, penetration_distance=0.0, ..., attenuation_db=22.0)`

2. **Coplanar Outer Face Grazing Step Discontinuity**:
   - **File**: `sim/obstacles.py`, lines 160–164, 303–305, 369.
   - **Code**:
     ```python
     161: if src[i] < self.min_pt[i] or src[i] > self.max_pt[i]:
     162:     return RayIntersectionResult(hit=False, ...)
     ```
   - **Observed**: For ray $[-5.0, 5.0, 10.0] \to [15.0, 5.0, 10.0]$ skimming roof face $z=10.0\text{ m}$:
     - $d_z = 0.0$.
     - $src_z = 10.0$ is NOT $< 0$ and NOT $> 10$, so parallel miss is not triggered.
     - $t_{\text{near}, z} = -\infty, t_{\text{far}, z} = +\infty$.
     - Computed $t_{enter} = 0.25, t_{exit} = 0.75$, $d_{pen} = 10.0\text{ m}$, `attenuation=37.00dB`.
     - However, perturbed ray with $z = 10.0 + 10^{-10}\text{ m}$:
       $src_z > 10.0$ is `True`, returns `hit=False`, `attenuation=0.00dB`.
     - Verbatim step discontinuity: $37.00\text{ dB}$ across $10^{-10}\text{ m}$.

3. **Exterior Surface Origin Directed Outward**:
   - **File**: `sim/obstacles.py`, line 176.
   - **Observed**: Ray starting at $[0.0, 5.0, 5.0]$ on $x=0$ wall pointing away into open air $[-10.0, 5.0, 5.0]$:
     - $t_{enter} = -1.0$, $t_{exit} = -0.0$.
     - In IEEE 754: `-0.0 >= 0.0` evaluates to `True`.
     - Output: `hit=True`, `los_clear=False`, `penetration=-0.0000m`, `attenuation=22.00dB`.

### 1.2 Direct Empirical Verification of the Remediated Implementation

Tested the remediated blueprint (requiring volumetric penetration $t_{exit} - t_{enter} > 10^{-5}$, $t_{out} - t_{in} > 10^{-5}$, $d_{pen} > 10^{-4}\text{ m}$, and parallel face clearance $|src_i - \text{face}| > 10^{-5}\text{ m}$):
- **Vertex Graze**: `hit=False`, `pen=0.0m`, `att=0.0dB`, `los_clear=True` (both scalar and batch).
- **Coplanar Roof Face Graze**: `hit=False`, `pen=0.0m`, `att=0.0dB`, `los_clear=True` (both scalar and batch).
- **Perturbed Roof Face**: `hit=False`, `pen=0.0m`, `att=0.0dB` (zero discontinuity).
- **Surface Outward Ray**: `hit=False`, `pen=0.0m`, `att=0.0dB`, `los_clear=True` (both scalar and batch).
- **Surface Inward Ray**: `hit=True`, `pen=10.0m`, `att=37.0dB`, `los_clear=False` (both scalar and batch).
- **10,000 Adversarial Rays**:
  - Boolean match: **10,000 / 10,000 (100.000%)**
  - Boolean mismatches: **0**
  - Max penetration difference: $4.26 \times 10^{-14}\text{ m}$
  - Max attenuation difference: $5.68 \times 10^{-14}\text{ dB}$
  - Vectorized speedup: **22.74x** ($666,675\text{ rays/sec}$)
- **Existing Unit Tests (`tests/unit/test_obstacles.py`)**: 15 / 15 passed (100%).
- **Existing E2E Tests (`tests/e2e/`)**: 138 / 138 passed (100%).

---

## 2. Logic Chain

1. **Premise 1**: In RF propagation through an urban obstacle field, an obstacle introduces non-line-of-sight (NLoS) attenuation only when electromagnetic energy must penetrate through the solid material volume of the building.
2. **Premise 2**: A ray that only grazes a 0-dimensional vertex or 1-dimensional edge of an obstacle in open air intersects a set of Lebesgue measure zero in $\mathbb{R}^3$. Its penetration distance through building material is identically zero.
   - *Supported by Obs 1.1 item 1*: The current slab implementation treats $t_{enter} == t_{exit}$ as `hit=True` and applies $22\text{ dB}$ attenuation, corrupting topological link state.
3. **Premise 3**: A ray coplanar with an exterior boundary face (such as a horizontal rooftop or vertical exterior wall) travels in free air outside or on the exterior boundary plane. If it does not enter the interior $B^\circ$, its physical traversal through building concrete is zero.
   - *Supported by Obs 1.1 item 2*: The current parallel slab handler sets $t_{\text{near}} = -\infty$ and $t_{\text{far}} = +\infty$ when $src_i \in [min_i, max_i]$, falsely accumulating the full building length as solid penetration.
4. **Premise 4**: An RF transmitter on an outer wall transmitting outward into open space has line-of-sight in the half-space away from the building.
   - *Supported by Obs 1.1 item 3*: `t_exit = -0.0` passing `t_exit >= 0.0` falsely occludes outward transmissions.
5. **Inference**: By requiring strict volumetric penetration ($t_{exit} - t_{enter} > 10^{-5}$, $t_{out} - t_{in} > 10^{-5}$, and $d_{pen} > 10^{-4}\text{ m}$) and checking that parallel rays are strictly interior ($src_i \in (min_i + 10^{-5}, max_i - 10^{-5})$), all three failure modes are completely eliminated while preserving genuine interior hits and sub-surface containment.
6. **Inference**: Applying identical mathematical criteria across `ObstacleAABB.intersect_ray_segment`, `ObstacleManager.check_los`, and `ObstacleManager.check_los_batch` preserves 100% numerical and boolean parity between scalar and vectorized implementations.

---

## 3. Caveats

- **Obstacle Geometry**: Analysis applies strictly to 3D Axis-Aligned Bounding Boxes (AABBs).
- **Tolerance Scale**: The selected tolerances ($\epsilon_t = 10^{-5}$, $\epsilon_d = 10^{-4}\text{ m}$, $\epsilon_{\text{plane}} = 10^{-5}\text{ m}$) are chosen based on the simulation scale ($500\text{m} \times 500\text{m}$ area, typical UAV link lengths $5\text{m} - 320\text{m}$). If ray lengths exceed $100\text{ km}$, $\epsilon_t \times L$ would scale proportionally.
- **Zero-Length Rays**: Zero-length rays ($L < 10^{-9}\text{ m}$) maintain point containment via `contains_point`, preserving compatibility with node containment tests.

---

## 4. Conclusion

The exact mathematical remediation for `sim/obstacles.py` has been formulated, empirically validated across 10,000 rays, and fully specified in `d:\drone model\IIT Bombay\.agents\explorer_m1_fix_geom\geom_remediation_spec.md`.

The implementation requires 3 synchronized edits in `sim/obstacles.py`:
1. **In `ObstacleAABB.intersect_ray_segment` (lines 158–198)**:
   - Update parallel slab condition: `if src[i] <= self.min_pt[i] + 1e-5 or src[i] >= self.max_pt[i] - 1e-5: return RayIntersectionResult(hit=False, ...)`
   - Update hit condition to require volumetric penetration:
     `hit = (t_enter <= t_exit) and (t_exit - t_enter > 1e-5) and (t_out - t_in > 1e-5) and (pen_dist > 1e-4)`
2. **In `ObstacleManager.check_los` (lines 302–325)**:
   - Update parallel miss: `miss = (src[i] <= self._cached_min[:, i] + 1e-5) | (src[i] >= self._cached_max[:, i] - 1e-5)`
   - Update hits mask: `hits = (t_enter <= t_exit) & (~parallel_miss) & (t_exit - t_enter > 1e-5) & (t_out - t_in > 1e-5) & (pen_dists > 1e-4)`
3. **In `ObstacleManager.check_los_batch` (lines 369–391)**:
   - Update parallel miss: `parallel_miss = is_parallel & ((orig_exp <= bmin_exp + 1e-5) | (orig_exp >= bmax_exp - 1e-5))`
   - Update hits mask: `hits = (t_enter <= t_exit) & (~any_parallel_miss) & (t_exit - t_enter > 1e-5) & (t_out - t_in > 1e-5) & (pen_dists > 1e-4)`

This eliminates all phantom 22 dB attenuation on vertex/edge grazes, eliminates the 37 dB roof grazing step discontinuity, and fixes outward surface rays, while ensuring 100% scalar/vectorized parity and passing all 15 unit tests and 138 E2E tests.

---

## 5. Verification Method

### 5.1 Direct Code Inspection
Inspect the full mathematical blueprint and line-by-line replacement chunks in:
`d:\drone model\IIT Bombay\.agents\explorer_m1_fix_geom\geom_remediation_spec.md`

### 5.2 Verification Commands
After Worker applies the patch to `sim/obstacles.py`:

1. **Verify Degenerate Geometry Fixes**:
   ```powershell
   python -c "
   import numpy as np
   from sim.obstacles import ObstacleAABB, ObstacleManager

   box = ObstacleAABB('B', 'Box', np.zeros(3), np.full(3, 10.0))
   mgr = ObstacleManager([box])

   # 1. Vertex Graze
   assert not box.intersect_ray_segment(np.array([11., 9., 10.]), np.array([9., 11., 10.])).hit
   assert mgr.check_los(np.array([11., 9., 10.]), np.array([9., 11., 10.]))[0] is True

   # 2. Coplanar Roof Face
   assert not box.intersect_ray_segment(np.array([-5., 5., 10.]), np.array([15., 5., 10.])).hit
   assert mgr.check_los(np.array([-5., 5., 10.]), np.array([15., 5., 10.]))[0] is True

   # 3. Outward Ray from Surface
   assert not box.intersect_ray_segment(np.array([0., 5., 5.]), np.array([-10., 5., 5.])).hit
   assert mgr.check_los(np.array([0., 5., 5.]), np.array([-10., 5., 5.]))[0] is True

   # 4. Inward Ray from Surface
   assert box.intersect_ray_segment(np.array([0., 5., 5.]), np.array([10., 5., 5.])).hit is True
   assert mgr.check_los(np.array([0., 5., 5.]), np.array([10., 5., 5.]))[0] is False

   print('ALL GEOMETRY REMEDIATION ASSERTIONS PASSED.')
   "
   ```
   *Expected outcome*: Exits cleanly with `ALL GEOMETRY REMEDIATION ASSERTIONS PASSED.`

2. **Verify 10,000-Ray Equivalence**:
   ```powershell
   python tests/stress_geometry_scale_determinism.py
   ```
   *Expected outcome*: 100% Boolean match, 0 mismatches between scalar and batch implementations across 10,000 adversarial rays.

3. **Run Unit and E2E Tests**:
   ```powershell
   pytest tests/unit/test_obstacles.py -v
   pytest tests/e2e/ -v
   ```
   *Expected outcome*: 100% tests passing (15/15 unit, 138/138 E2E).
