# Dispatch: Test Writer — E2E Testing Track

## Role
E2E Test Writer (`teamwork_preview_test_writer`)

## Working Directory
`d:\drone model\IIT Bombay\.agents\test_writer_e2e`

## Instructions
Read `ORIGINAL_REQUEST.md`, `PROJECT.md`, and `TEST_INFRA.md`:
- `d:\drone model\IIT Bombay\.agents\ORIGINAL_REQUEST.md`
- `d:\drone model\IIT Bombay\PROJECT.md`
- `d:\drone model\IIT Bombay\TEST_INFRA.md`

## Objectives
Implement the complete, requirement-driven, opaque-box E2E test suite in `tests/e2e/`:
1. `tests/conftest.py` & `tests/e2e/__init__.py`: Fixtures and environment setup.
2. `tests/e2e/test_tier1_features.py`: Feature Isolation Coverage (≥ 60 tests: ≥ 5 tests per feature for each of the 12 features in `TEST_INFRA.md`).
3. `tests/e2e/test_tier2_boundaries.py`: Boundary Value Analysis & Corner Cases (≥ 60 tests: limits, extreme distances, zero velocities, obstacle bounds, max buffer capacities, elevation grazing).
4. `tests/e2e/test_tier3_combinations.py`: Pairwise Combinatorial Interactions (≥ 12 tests: kinematics + obstacles, routing + buffering, survey dwell + transmission, etc.).
5. `tests/e2e/test_tier4_scenarios.py`: Real-World Disaster Workload Scenarios (≥ 6 realistic disaster scenarios defined in `TEST_INFRA.md`).
6. `tests/e2e/test_runner.py`: Unified test orchestrator running all 4 tiers, reporting scorecard, and returning exit code 0 on 100% pass.
7. Upon successful creation and verification of test infrastructure, create `d:\drone model\IIT Bombay\TEST_READY.md`.

Deliver `handoff.md` and report back when complete.

## 2026-09-25T14:33:00Z
<USER_REQUEST>
You are the E2E Test Writer for the 3D UAV Swarm & Communication Network project.
Your working directory is: d:\drone model\IIT Bombay\.agents\test_writer_e2e
You MUST read the following specifications before writing any code:
1. ORIGINAL_REQUEST: d:\drone model\IIT Bombay\.agents\ORIGINAL_REQUEST.md
2. Global Architecture: d:\drone model\IIT Bombay\PROJECT.md
3. Test Methodology & Thresholds: d:\drone model\IIT Bombay\TEST_INFRA.md
4. Your dispatch instructions: d:\drone model\IIT Bombay\.agents\test_writer_e2e\DISPATCH.md

Your task is to build the complete, requirement-driven, opaque-box E2E test suite in tests/e2e/:
- tests/conftest.py & tests/e2e/__init__.py
- tests/e2e/test_tier1_features.py (≥ 60 tests covering 12 features, ≥ 5 tests each)
- tests/e2e/test_tier2_boundaries.py (≥ 60 boundary and edge-case tests, ≥ 5 each)
- tests/e2e/test_tier3_combinations.py (≥ 12 pairwise interaction tests)
- tests/e2e/test_tier4_scenarios.py (≥ 6 realistic disaster mission scenarios)
- tests/e2e/test_runner.py (unified test orchestrator with clean terminal scorecard and exit code 0)

Ensure the tests are cleanly organized, use mocks or simulated dummy data where backend modules are not yet written, or verify interfaces as defined in PROJECT.md and TEST_INFRA.md.
When the test suite structure and cases are ready and verified, create:
d:\drone model\IIT Bombay\TEST_READY.md
Document commands and results in handoff.md and notify the parent orchestrator with send_message.
</USER_REQUEST>
