# Dispatch: Worker M1-Fix — Milestone 1 Iteration 2 Remediation Implementation

## Role
Worker (`teamwork_preview_worker`)

## Working Directory
`d:\drone model\IIT Bombay\.agents\worker_m1_fix`

## Mandatory Integrity Warning
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

## Inputs
- `d:\drone model\IIT Bombay\.agents\ORIGINAL_REQUEST.md`
- `d:\drone model\IIT Bombay\PROJECT.md`
- Full Auditor Evidence: `d:\drone model\IIT Bombay\.agents\auditor_m1\handoff.md`
- Challenger M1-1 Report: `d:\drone model\IIT Bombay\.agents\challenger_m1_1\handoff.md`
- Challenger M1-2 Report: `d:\drone model\IIT Bombay\.agents\challenger_m1_2\handoff.md`
- Reviewer M1-2 Report: `d:\drone model\IIT Bombay\.agents\reviewer_m1_2\handoff.md`
- Core Remediation Spec: `d:\drone model\IIT Bombay\.agents\explorer_m1_fix_core\core_remediation_spec.md`
- Dynamics Remediation Spec: `d:\drone model\IIT Bombay\.agents\explorer_m1_fix_dyn\dynamics_remediation_spec.md`
- Geometry Remediation Spec: `d:\drone model\IIT Bombay\.agents\explorer_m1_fix_geom\geom_remediation_spec.md`

## Write Ownership (Exclusive)
- `sim/core.py`
- `sim/drone.py`
- `sim/obstacles.py`
- `tests/unit/test_sim_core.py`
- `tests/unit/test_adversarial_m1.py`
- `tests/unit/test_challenger_m1_2.py`
- `tests/unit/test_drone.py`
- `tests/unit/test_obstacles.py`

## Remediation Tasks to Apply
1. **`sim/core.py`**:
   - Add `import math` to top-level imports.
   - Fix zero-offset downwash lateral escape when $d_{xy} < 10^{-3}$ m (apply deterministic $+35.0$ N lateral nudge `np.array([1.0, 0.0])`).
2. **`sim/drone.py`**:
   - Add `Sequence` to `from typing import ...` on line 13.
   - Implement dynamic closing velocity repulsive horizon and kinetic damping for head-on collisions ($v_{close} \cdot m \cdot \hat{\mathbf{r}}$).
   - Implement prioritized safety filter attenuating attractive force when $d < r_{safety} = 1.5$ m plus cubic singularity barrier to prevent collinear multi-drone compression.
   - Implement dynamic obstacle sensing horizon $\rho_{0, dyn}$ and hard surface collision clamping in `step_physics`.
3. **`sim/obstacles.py`**:
   - Require strict volumetric intersection ($t_{exit} - t_{enter} > 10^{-5}$ and $d_{pen} > 10^{-4}$ m) to eliminate vertex/edge grazing false hits and coplanar face grazing step discontinuities.
   - Fix outward-facing surface rays in both scalar `check_los` and vectorized `check_los_batch`.
4. **Unit Tests**:
   - Add vertical separation downwash tests in `tests/unit/test_sim_core.py`.
   - Update `tests/unit/test_adversarial_m1.py` and `tests/unit/test_challenger_m1_2.py` to assert correct behavior and eliminate warnings.
5. **Validation**:
   - Run `pytest tests/unit/ -v` and `pytest tests/e2e/ -v`.
   - Ensure 100% pass rate.
   - Deliver `handoff.md`.
