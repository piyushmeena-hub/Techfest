# Dispatch: Forensic Integrity Auditor — Milestone 1

## Role
Forensic Auditor (`teamwork_preview_auditor`)

## Working Directory
`d:\drone model\IIT Bombay\.agents\auditor_m1`

## Inputs
- `d:\drone model\IIT Bombay\.agents\ORIGINAL_REQUEST.md`
- `d:\drone model\IIT Bombay\PROJECT.md`
- Codebase: `sim/`, `tests/`

## Instructions
1. Perform an uncompromised, forensic integrity audit of all code and tests produced for Milestone 1.
2. Verify against integrity violations:
   - Check whether any test results, physics values, or collision/telemetry outputs are hardcoded in source code or tests.
   - Check whether any dummy or facade classes exist that mimic real physics or calculations without performing actual computation.
   - Inspect AST / code structure to verify that Newton-Euler equations, Williams ray-slab algorithms, vector forces, and battery discharge models actually execute genuine mathematical calculations.
   - Verify that all 71 unit tests in `tests/unit/` genuinely execute the real implementation and perform meaningful assertions.
3. Deliver a binary verdict: `CLEAN` or `INTEGRITY VIOLATION`.
   Document all forensic checks, files audited, and AST inspection details in `d:\drone model\IIT Bombay\.agents\auditor_m1\handoff.md`.

## 2026-09-25T14:46:22Z
<USER_REQUEST>
You are the Forensic Integrity Auditor for Milestone 1.
Your working directory is: d:\drone model\IIT Bombay\.agents\auditor_m1
You MUST read:
1. ORIGINAL_REQUEST: d:\drone model\IIT Bombay\.agents\ORIGINAL_REQUEST.md
2. Global Architecture: d:\drone model\IIT Bombay\PROJECT.md
3. Your dispatch instructions: d:\drone model\IIT Bombay\.agents\auditor_m1\DISPATCH.md

Perform a strict forensic audit of the entire Milestone 1 codebase (sim/, tests/):
- Search for hardcoded test outputs or expected return values.
- Verify that classes and methods perform genuine calculations (AST and source inspection).
- Check that unit tests actually execute real logic and do not use mocking to bypass computation.
- Verify genuine implementation of Newton-Euler equations, Ray-AABB slab intersection, APF forces, and battery models.
Deliver a binary verdict: CLEAN or INTEGRITY VIOLATION.
Document all evidence in handoff.md and notify parent orchestrator with send_message.
</USER_REQUEST>
