# Dispatch: Worker M1 — Core Simulation Engine & Dynamics Implementation

## Role
Worker (`teamwork_preview_worker`)

## Working Directory
`d:\drone model\IIT Bombay\.agents\worker_m1`

## Inputs
- `d:\drone model\IIT Bombay\.agents\ORIGINAL_REQUEST.md`
- `d:\drone model\IIT Bombay\PROJECT.md`
- Explorer M1-1 Specification: `d:\drone model\IIT Bombay\.agents\explorer_m1_drone\drone_impl_spec.md`
- Explorer M1-2 Specification: `d:\drone model\IIT Bombay\.agents\explorer_m1_env\env_impl_spec.md`
- Explorer M1-3 Specification: `d:\drone model\IIT Bombay\.agents\explorer_m1_core\core_impl_spec.md`

## Write Ownership (Exclusive)
You have exclusive write ownership over:
- `sim/__init__.py`
- `sim/types.py`
- `sim/drone.py`
- `sim/environment.py`
- `sim/obstacles.py`
- `sim/core.py`
- `tests/unit/test_types.py`
- `tests/unit/test_drone.py`
- `tests/unit/test_environment.py`
- `tests/unit/test_obstacles.py`
- `tests/unit/test_sim_core.py`

## Instructions
1. Implement the complete, genuine, production-grade simulation core based directly on the specifications provided by the 3 explorers.
2. Implement:
   - `sim/types.py`: `DroneRole`, `FlightMode`, `DroneLimits`, `BatteryModel`, `AltitudeCorridor`, `DroneState` (with `.to_dict()`).
   - `sim/drone.py`: 6-DOF Quadcopter physics, state vector integration, velocity/acceleration clamping, attitude Euler/quaternion updates, Khatib APF + Reynolds flocking forces, aerodynamic downwash cone repulsion, and 4-tier altitude corridor deconfliction.
   - `sim/environment.py`: 500x500x120m disaster zone, GCS base station placement at `[0, 0, 0]`, coordinate validation, and boundary clamping.
   - `sim/obstacles.py`: 3D AABB obstacle model, vectorized Ray-AABB slab intersection algorithm (Williams et al.), penetration distance calculation, NLoS RF attenuation, and default disaster obstacles preset.
   - `sim/core.py`: `SwarmSimulationCore` class managing the deterministic multi-agent step tick (`step(dt)`), physics integration, obstacle collision checks, fleet coordination (VSM relay positioning), and telemetry snapshot serialization (`to_dict()` / JSON export).
   - `sim/__init__.py`: Clean public API export.
3. Write comprehensive unit test suites in `tests/unit/`:
   - `tests/unit/test_types.py`
   - `tests/unit/test_drone.py`
   - `tests/unit/test_environment.py`
   - `tests/unit/test_obstacles.py`
   - `tests/unit/test_sim_core.py`
4. Run all unit tests using `pytest tests/unit/ -v` and ensure 100% pass rate.
5. Provide detailed test outputs, command logs, and code verification in your `handoff.md`.

## Mandatory Integrity Warning
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

## 2026-09-25T14:38:16Z
Worker M1 activated for Core Simulation Engine & Dynamics Implementation.

