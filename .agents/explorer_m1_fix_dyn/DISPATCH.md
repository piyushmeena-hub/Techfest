# Dispatch: Explorer M1-Fix-2 — Dynamic Collision Avoidance & Physics Remediation

## Role
Dynamics & Flocking Remediation Explorer (`teamwork_preview_explorer`)

## Working Directory
`d:\drone model\IIT Bombay\.agents\explorer_m1_fix_dyn`

## Inputs & Unfiltered Audit & Challenger Evidence
You MUST read:
1. `d:\drone model\IIT Bombay\.agents\ORIGINAL_REQUEST.md`
2. `d:\drone model\IIT Bombay\PROJECT.md`
3. Full Forensic Auditor Report (INTEGRITY VIOLATION):
   `d:\drone model\IIT Bombay\.agents\auditor_m1\handoff.md`
4. Full Challenger M1-1 Report (REJECT):
   `d:\drone model\IIT Bombay\.agents\challenger_m1_1\handoff.md`
5. Full Reviewer M1-1 Report:
   `d:\drone model\IIT Bombay\.agents\reviewer_m1_1\handoff.md`

## Key Vulnerabilities to Remediate
1. **Type Annotation Bug**: In `sim/drone.py:13`, add `Sequence` to `from typing import ...` to fix `typing.get_type_hints` failure on lines 91 and 96.
2. **High-Velocity Head-On Collision Tunneling**: Drones closing at 20 m/s collide and pass through each other ($min\_sep = 0.026$ m). Static 6m separation radius is inadequate for stopping distance $d_{stop} = v_{rel}^2 / (2 a_{max}) = 20^2 / 8 = 50$ m.
   - Remediation: Implement velocity-adaptive repulsive horizon and force scaling based on relative closing velocity $\mathbf{v}_{rel} \cdot \hat{\mathbf{r}}_{ij}$.
3. **Collinear Multi-Drone Compression**: When multiple drones converge on a line, waypoint attraction ($22.5$ N) overpowers separation ($6.67$ N), compressing fleet to $< 0.1$ m.
   - Remediation: Nonlinear singularity barrier or prioritized safety repulsion capping/attenuating attractive force when $d < r_{safety} = 1.5$ m.
4. **High-Speed Obstacle Penetration**: Drones traveling at 10 m/s penetrate solid AABB buildings because repulsion is limited to 8m (stopping distance is 12.5m) and hard boundary clamping does not prevent obstacle entry.
   - Remediation: Dynamic obstacle sensing horizon $d_{obs} = \max(8.0, v^2 / (2 a_{max}) + r_{safe})$ plus hard collision stopping in `step_physics`.

## Your Task
1. Formulate the exact mathematical equations, algorithms, and code changes for `sim/drone.py`.
2. Write report to `d:\drone model\IIT Bombay\.agents\explorer_m1_fix_dyn\dynamics_remediation_spec.md` and deliver `handoff.md`.

## 2026-09-25T14:53:48Z
You are Explorer M1-Fix-2: Dynamic Collision Avoidance & Physics Remediation Specialist.
Your working directory is: d:\drone model\IIT Bombay\.agents\explorer_m1_fix_dyn

You MUST read:
1. ORIGINAL_REQUEST: d:\drone model\IIT Bombay\.agents\ORIGINAL_REQUEST.md
2. Global Architecture: d:\drone model\IIT Bombay\PROJECT.md
3. Full Forensic Auditor Report (INTEGRITY VIOLATION): d:\drone model\IIT Bombay\.agents\auditor_m1\handoff.md
4. Challenger M1-1 Report: d:\drone model\IIT Bombay\.agents\challenger_m1_1\handoff.md
5. Reviewer M1-1 Report: d:\drone model\IIT Bombay\.agents\reviewer_m1_1\handoff.md
6. Your dispatch instructions: d:\drone model\IIT Bombay\.agents\explorer_m1_fix_dyn\DISPATCH.md

Your task is to formulate the exact remediation blueprint for sim/drone.py:
- Add Sequence to typing imports on line 13.
- Relative closing velocity-dependent repulsive horizon to prevent high-speed head-on collision tunneling.
- Singularity barrier / attractive force attenuation to prevent multi-drone collinear compression.
- Dynamic obstacle sensing horizon and penetration prevention.
Write your analysis to dynamics_remediation_spec.md, deliver handoff.md, and notify parent orchestrator with send_message.
