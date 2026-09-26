"""
Unified Test Runner & Scorecard Generator for 3D Resilient Multi-Hop FANET Simulation.
Executes all 4 E2E testing tiers and prints a comprehensive terminal scorecard.
Returns exit code 0 on 100% pass, non-zero on failure.
"""

from __future__ import annotations
import sys
import time
import pytest


def run_tier(tier_name: str, file_path: str, min_expected: int) -> dict:
    """Run a specific tier file and collect execution statistics."""
    print(f"\n>> Executing {tier_name} ({file_path})...")
    start_time = time.perf_counter()

    # Capture pytest results
    # Use quiet mode to capture concise pass/fail
    class CollectorPlugin:
        def __init__(self):
            self.passed = 0
            self.failed = 0
            self.skipped = 0
            self.errors = 0

        def pytest_runtest_logreport(self, report):
            if report.when == "call":
                if report.passed:
                    self.passed += 1
                elif report.failed:
                    self.failed += 1
                elif report.skipped:
                    self.skipped += 1
            elif report.when in ("setup", "teardown") and report.failed:
                self.errors += 1

    collector = CollectorPlugin()
    exit_code = pytest.main(["-q", file_path], plugins=[collector])
    elapsed = time.perf_counter() - start_time

    total = collector.passed + collector.failed + collector.skipped
    pass_rate = (collector.passed / max(1, total)) * 100.0 if total > 0 else 0.0

    return {
        "tier": tier_name,
        "file": file_path,
        "total": total,
        "passed": collector.passed,
        "failed": collector.failed,
        "skipped": collector.skipped,
        "errors": collector.errors,
        "pass_rate": pass_rate,
        "duration": elapsed,
        "min_expected": min_expected,
        "status": "PASS" if (collector.failed == 0 and collector.errors == 0 and total >= min_expected) else "FAIL"
    }


def main() -> int:
    header = "=" * 80
    print(header)
    print("   3D RESILIENT MULTI-HOP AERIAL UAV NETWORK -- E2E TEST RUNNER")
    print("   Requirement-Driven Opaque-Box Test Suite (Tiers 1-4)")
    print(header)

    tiers = [
        ("Tier 1: Feature Isolation", "tests/e2e/test_tier1_features.py", 60),
        ("Tier 2: Boundary & Corner Cases", "tests/e2e/test_tier2_boundaries.py", 60),
        ("Tier 3: Pairwise Combinations", "tests/e2e/test_tier3_combinations.py", 12),
        ("Tier 4: Real-World Scenarios", "tests/e2e/test_tier4_scenarios.py", 6),
    ]

    suite_start = time.perf_counter()
    results = []

    for name, path, min_exp in tiers:
        res = run_tier(name, path, min_exp)
        results.append(res)

    suite_elapsed = time.perf_counter() - suite_start

    # Render Scorecard
    print("\n" + header)
    print("                      E2E TEST SCORECARD SUMMARY")
    print(header)
    print(f"{'Tier Name':<34} | {'Tests':<7} | {'Passed':<7} | {'Failed':<7} | {'Pass %':<7} | {'Status':<6}")
    print("-" * 80)

    total_tests = 0
    total_passed = 0
    total_failed = 0
    all_passed = True

    for r in results:
        total_tests += r["total"]
        total_passed += r["passed"]
        total_failed += r["failed"]
        if r["status"] != "PASS":
            all_passed = False

        status_str = f"[\033[92mPASS\033[0m]" if r["status"] == "PASS" else f"[\033[91mFAIL\033[0m]"
        print(f"{r['tier']:<34} | {r['total']:<7} | {r['passed']:<7} | {r['failed']:<7} | {r['pass_rate']:>6.1f}% | {status_str}")

    print("-" * 80)
    overall_rate = (total_passed / max(1, total_tests)) * 100.0
    final_status = "[\033[92mALL PASSED\033[0m]" if (all_passed and total_tests >= 138) else "[\033[91mFAILED\033[0m]"
    print(f"{'TOTAL E2E SUITE':<34} | {total_tests:<7} | {total_passed:<7} | {total_failed:<7} | {overall_rate:>6.1f}% | {final_status}")
    print(header)
    print(f"Total Execution Time: {suite_elapsed:.2f} seconds")
    print(f"Threshold Verification: {total_tests}/138 minimum tests required (Satisfied: {'YES' if total_tests >= 138 else 'NO'})")
    print(header + "\n")

    return 0 if (all_passed and total_tests >= 138) else 1


if __name__ == "__main__":
    sys.exit(main())
