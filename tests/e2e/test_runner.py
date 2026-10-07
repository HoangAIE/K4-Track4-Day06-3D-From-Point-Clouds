"""Standalone E2E Test Suite Runner for Topic C: LiDAR-Camera Sensor Degradation.

Executes all 4 test tiers (Tiers 1-4) or selected tiers, records detailed statistics,
computes pass rates, formats a summary table, and exits with 0 on pass or 1 on failure.

Usage:
    .venv/Scripts/python.exe -m tests.e2e.test_runner
    .venv/Scripts/python.exe -m tests.e2e.test_runner --tier 1
    .venv/Scripts/python.exe -m tests.e2e.test_runner --verbose
    .venv/Scripts/python.exe -m tests.e2e.test_runner --strict
"""
from __future__ import annotations

import argparse
import io
import sys
import time
import unittest
from dataclasses import dataclass
from typing import Dict, List, Optional


@dataclass
class TierResult:
    tier_name: str
    total: int = 0
    passed: int = 0
    failed: int = 0
    errors: int = 0
    skipped: int = 0
    duration_s: float = 0.0

    @property
    def pass_rate(self) -> float:
        # Effective pass rate among executed non-skipped tests
        executed = self.total - self.skipped
        if executed == 0:
            return 100.0
        return (self.passed / executed) * 100.0

    @property
    def is_success(self) -> bool:
        return self.failed == 0 and self.errors == 0


class CustomTestResult(unittest.TestResult):
    """Custom TestResult collecting detailed per-test execution information."""

    def __init__(self, stream=None, descriptions=None, verbosity=1):
        super().__init__(stream, descriptions, verbosity)
        self.passed_tests: List[unittest.TestCase] = []

    def addSuccess(self, test):
        super().addSuccess(test)
        self.passed_tests.append(test)


def load_tier_suite(tier: int) -> unittest.TestSuite:
    loader = unittest.defaultTestLoader
    if tier == 1:
        from tests.e2e import test_tier1_features
        return loader.loadTestsFromModule(test_tier1_features)
    elif tier == 2:
        from tests.e2e import test_tier2_boundaries
        return loader.loadTestsFromModule(test_tier2_boundaries)
    elif tier == 3:
        from tests.e2e import test_tier3_combinations
        return loader.loadTestsFromModule(test_tier3_combinations)
    elif tier == 4:
        from tests.e2e import test_tier4_applications
        return loader.loadTestsFromModule(test_tier4_applications)
    else:
        raise ValueError(f"Unknown tier: {tier}")


def run_tier(tier: int, verbose: bool = False) -> TierResult:
    tier_titles = {
        1: "Tier 1: Feature Coverage (F1..F14)",
        2: "Tier 2: Boundary & Corner Cases",
        3: "Tier 3: Pairwise Cross-Feature Combinations",
        4: "Tier 4: Real-World Workload Scenarios",
    }
    tier_name = tier_titles.get(tier, f"Tier {tier}")
    suite = load_tier_suite(tier)
    result = CustomTestResult(verbosity=2 if verbose else 1)

    t0 = time.perf_counter()
    suite.run(result)
    t1 = time.perf_counter()

    tier_res = TierResult(
        tier_name=tier_name,
        total=result.testsRun,
        passed=len(result.passed_tests),
        failed=len(result.failures),
        errors=len(result.errors),
        skipped=len(result.skipped),
        duration_s=t1 - t0,
    )
    return tier_res, result


def print_summary_table(tier_results: List[TierResult], strict: bool = False) -> None:
    print("\n" + "=" * 90)
    print("                    E2E TEST SUITE EXECUTION SUMMARY REPORT")
    print("=" * 90)
    headers = f"{'Tier Name':<42} | {'Total':>5} | {'Pass':>5} | {'Fail':>5} | {'Err':>4} | {'Skip':>5} | {'Rate %':>7} | {'Time (s)':>8}"
    print(headers)
    print("-" * 90)

    total_all = 0
    pass_all = 0
    fail_all = 0
    err_all = 0
    skip_all = 0
    duration_all = 0.0

    for tr in tier_results:
        total_all += tr.total
        pass_all += tr.passed
        fail_all += tr.failed
        err_all += tr.errors
        skip_all += tr.skipped
        duration_all += tr.duration_s

        rate_str = f"{tr.pass_rate:6.1f}%"
        print(f"{tr.tier_name:<42} | {tr.total:>5} | {tr.passed:>5} | {tr.failed:>5} | {tr.errors:>4} | {tr.skipped:>5} | {rate_str:>7} | {tr.duration_s:>8.3f}")

    print("-" * 90)
    executed_all = total_all - skip_all
    overall_rate = (pass_all / executed_all * 100.0) if executed_all > 0 else 100.0
    overall_str = f"{overall_rate:6.1f}%"
    print(f"{'OVERALL TOTALS':<42} | {total_all:>5} | {pass_all:>5} | {fail_all:>5} | {err_all:>4} | {skip_all:>5} | {overall_str:>7} | {duration_all:>8.3f}")
    print("=" * 90)

    all_passed = (fail_all == 0 and err_all == 0)
    if strict and skip_all > 0:
        all_passed = False
        print(f"[STATUS] STRICT AUDIT FAILED: {skip_all} tests were skipped awaiting future milestone features.")
    elif all_passed:
        print(f"[STATUS] PASS: All {pass_all} executed tests passed cleanly! ({skip_all} pending future milestones)")
    else:
        print(f"[STATUS] FAIL: Detected {fail_all} failure(s) and {err_all} error(s).")
    print("=" * 90 + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run E2E Test Suite for Topic C LiDAR-Camera Project")
    parser.add_argument("--tier", choices=["1", "2", "3", "4", "all"], default="all",
                        help="Select tier to execute (1, 2, 3, 4, or all)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Print verbose test information")
    parser.add_argument("--strict", action="store_true",
                        help="Strict mode: require 0 skipped tests (for final milestone verification)")
    args = parser.parse_args()

    # Reconfigure UTF-8 for Windows console
    if sys.platform == "win32":
        try:
            sys.stdout.reconfigure(encoding="utf-8")
            sys.stderr.reconfigure(encoding="utf-8")
        except AttributeError:
            pass

    tiers_to_run = [1, 2, 3, 4] if args.tier == "all" else [int(args.tier)]
    results: List[TierResult] = []
    has_failure = False

    print(f"\nStarting E2E Test Runner [Tiers: {tiers_to_run}, Strict: {args.strict}]...")

    for tier in tiers_to_run:
        print(f"\n>> Executing Tier {tier}...")
        tier_res, test_res = run_tier(tier, verbose=args.verbose)
        results.append(tier_res)
        if not tier_res.is_success:
            has_failure = True
            for test, err in test_res.failures:
                print(f"   [FAIL] {test}: {err.splitlines()[-1] if err else ''}")
            for test, err in test_res.errors:
                print(f"   [ERROR] {test}: {err.splitlines()[-1] if err else ''}")

    print_summary_table(results, strict=args.strict)

    if has_failure:
        return 1
    if args.strict and any(r.skipped > 0 for r in results):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
