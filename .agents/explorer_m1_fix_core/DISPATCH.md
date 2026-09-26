# Dispatch: Explorer M1-Fix-1 — Core Simulation & Downwash Remediation

## Role
Core Simulation Remediation Explorer (`teamwork_preview_explorer`)

## Working Directory
`d:\drone model\IIT Bombay\.agents\explorer_m1_fix_core`

## Inputs & Unfiltered Audit Evidence
You MUST read:
1. `d:\drone model\IIT Bombay\.agents\ORIGINAL_REQUEST.md`
2. `d:\drone model\IIT Bombay\PROJECT.md`
3. Full Forensic Auditor Report (INTEGRITY VIOLATION):
   `d:\drone model\IIT Bombay\.agents\auditor_m1\handoff.md`
4. Full Challenger M1-2 Report:
   `d:\drone model\IIT Bombay\.agents\challenger_m1_2\handoff.md`

## Verbatim Auditor Evidence
```
FAILURES:
______ TestAdversarialDownwash.test_downwash_zero_lateral_offset_in_core ______
sim\core.py:183: NameError: name 'math' is not defined. Did you forget to import 'math'?
```
Bytecode analysis:
- `sim/core.py:183`: missing global load 'math'
- `sim/core.py:186`: missing global load 'math'
Test masking:
- `tests/unit/test_sim_core.py:160` (`test_separation_and_downwash`) only tested coplanar drones ($dz=0.0$), leaving lines 181-186 unexecuted.
Degeneracy:
- When $d_{xy}=0$, lateral escape direction is zero, pinning drone down.

## Your Task
1. Formulate exact remediation strategy for `sim/core.py`:
   - Add `import math` (or use `np.exp`).
   - Fix zero-offset lateral escape: when $d_{xy} < 10^{-3}$, add a deterministic non-zero lateral nudge so the lower drone escapes horizontally instead of being pinned.
   - Formulate new unit test in `tests/unit/test_sim_core.py` explicitly testing downwash with vertical separation (e.g. $dz = -4.0$ m).
2. Write report to `d:\drone model\IIT Bombay\.agents\explorer_m1_fix_core\core_remediation_spec.md` and deliver `handoff.md`.

## 2026-09-25T14:54:00Z
You are Explorer M1-Fix-1: Core Simulation & Downwash Remediation Specialist.
Your working directory is: d:\drone model\IIT Bombay\.agents\explorer_m1_fix_core

You MUST read:
1. ORIGINAL_REQUEST: d:\drone model\IIT Bombay\.agents\ORIGINAL_REQUEST.md
2. Global Architecture: d:\drone model\IIT Bombay\PROJECT.md
3. Full Forensic Auditor Report (INTEGRITY VIOLATION): d:\drone model\IIT Bombay\.agents\auditor_m1\handoff.md
4. Challenger M1-2 Report: d:\drone model\IIT Bombay\.agents\challenger_m1_2\handoff.md
5. Your dispatch instructions: d:\drone model\IIT Bombay\.agents\explorer_m1_fix_core\DISPATCH.md

Your task is to formulate the exact remediation blueprint for sim/core.py:
- Missing 'import math' in sim/core.py:183, 186.
- Zero-offset lateral escape when d_xy < 1e-3.
- Unit test in tests/unit/test_sim_core.py testing downwash with vertical separation.
Write your analysis to core_remediation_spec.md, deliver handoff.md, and notify parent orchestrator with send_message.
