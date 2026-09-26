# Dispatch: Explorer M1-Fix-3 — Ray-AABB Geometry & Grazing Remediation

## Role
Geometry & Ray-Tracing Remediation Explorer (`teamwork_preview_explorer`)

## Working Directory
`d:\drone model\IIT Bombay\.agents\explorer_m1_fix_geom`

## Inputs & Unfiltered Audit & Challenger Evidence
You MUST read:
1. `d:\drone model\IIT Bombay\.agents\ORIGINAL_REQUEST.md`
2. `d:\drone model\IIT Bombay\PROJECT.md`
3. Full Challenger M1-2 Report:
   `d:\drone model\IIT Bombay\.agents\challenger_m1_2\handoff.md`
4. Full Reviewer M1-2 Report:
   `d:\drone model\IIT Bombay\.agents\reviewer_m1_2\handoff.md`

## Key Geometry Vulnerabilities to Remediate
1. **Vertex Grazing (Zero Penetration False Hits)**:
   - Rays touching a vertex or edge have $t_{enter} == t_{exit}$, meaning zero volume is intersected. Currently reports `hit=True` with +22 dB attenuation.
   - Remediation: Enforce strict volumetric intersection ($t_{exit} - t_{enter} > 10^{-5}$ and penetration distance $d_{pen} > 10^{-4}$ m) for a solid hit.
2. **Coplanar Face Grazing Discontinuity**:
   - Skimming a flat roof face at $z=10.0$ reports 37 dB attenuation, while $z=10.0000000001$ reports 0 dB.
   - Remediation: Rays that are coplanar with an exterior face and do not enter the interior must be treated as clear line-of-sight in free air.
3. **Exterior Surface Origin Pointing Outward**:
   - In `sim/obstacles.py:176`, `t_exit >= 0.0` causes rays starting on an outer wall directed away from the building into open space to falsely report `hit=True`.
   - Remediation: Require $t_{enter} < 1.0$ and $t_{exit} > 10^{-5}$, and verify that the ray actually points into the AABB interior.

## Your Task
1. Formulate exact mathematical geometry algorithms and code changes for `sim/obstacles.py` (both scalar `check_los` and vectorized `check_los_batch`).
2. Write report to `d:\drone model\IIT Bombay\.agents\explorer_m1_fix_geom\geom_remediation_spec.md` and deliver `handoff.md`.

## 2026-09-25T14:53:48Z
You are Explorer M1-Fix-3: Ray-AABB Geometry & Grazing Remediation Specialist.
Your working directory is: d:\drone model\IIT Bombay\.agents\explorer_m1_fix_geom

You MUST read:
1. ORIGINAL_REQUEST: d:\drone model\IIT Bombay\.agents\ORIGINAL_REQUEST.md
2. Global Architecture: d:\drone model\IIT Bombay\PROJECT.md
3. Challenger M1-2 Report: d:\drone model\IIT Bombay\.agents\challenger_m1_2\handoff.md
4. Reviewer M1-2 Report: d:\drone model\IIT Bombay\.agents\reviewer_m1_2\handoff.md
5. Your dispatch instructions: d:\drone model\IIT Bombay\.agents\explorer_m1_fix_geom\DISPATCH.md

Your task is to formulate the exact remediation blueprint for sim/obstacles.py:
- Eliminate phantom 22 dB attenuation on non-penetrating vertex/edge grazes (require volumetric penetration).
- Eliminate step discontinuity on coplanar outer face grazing.
- Fix rays starting on outer surface pointing outward falsely marked as occluded.
Ensure both scalar check_los and vectorized check_los_batch remain identical.
Write your analysis to geom_remediation_spec.md, deliver handoff.md, and notify parent orchestrator with send_message.
