# E2E Test Infra: Topic C — LiDAR-Camera Sensor Degradation

## Test Philosophy
- Opaque-box, requirement-driven derived from `ORIGINAL_REQUEST.md` and `PROJECT.md`.
- No reliance on internal module internals; tests exercise public interfaces, CLI commands, scripts, and output files.
- Methodology: Category-Partition + Boundary Value Analysis (BVA) + Pairwise Combinations + Real-World Workload Scenarios.

## Feature Inventory & Test Mapping
| # | Feature | Requirement | Tier 1 | Tier 2 | Tier 3 |
|---|---------|-------------|:------:|:------:|:------:|
| 1 | Velo to Cam Transformation | ORIGINAL_REQUEST §R1 | 5 | 5 | ✓ |
| 2 | Cam to Image Projection | ORIGINAL_REQUEST §R1 | 5 | 5 | ✓ |
| 3 | Multi-dataset Projection | ORIGINAL_REQUEST §R1 | 5 | 5 | ✓ |
| 4 | LiDAR Point Degradations | ORIGINAL_REQUEST §R2 | 5 | 5 | ✓ |
| 5 | Non-DL Object Metrics | ORIGINAL_REQUEST §R2 | 5 | 5 | ✓ |
| 6 | FOV & Health Score Metrics | ORIGINAL_REQUEST §R2 | 5 | 5 | ✓ |
| 7 | Benchmark CSV & Figures | ORIGINAL_REQUEST §R2 | 5 | 5 | ✓ |
| 8 | Failure Case Reproduction | ORIGINAL_REQUEST §R3 | 5 | 5 | ✓ |
| 9 | Debug Layer Classification | ORIGINAL_REQUEST §R3 | 5 | 5 | ✓ |
| 10 | Interactive CPU Demo App | ORIGINAL_REQUEST §R4 | 5 | 5 | ✓ |
| 11 | Headless Export for Demo | ORIGINAL_REQUEST §R4 | 5 | 5 | ✓ |
| 12 | Complete REPORT.md | ORIGINAL_REQUEST §R5 | 5 | 5 | ✓ |
| 13 | Student Info Verification | ORIGINAL_REQUEST §R5 | 5 | 5 | ✓ |
| 14 | 100% Submission Gate Pass | ORIGINAL_REQUEST §R5 | 5 | 5 | ✓ |

## Test Architecture
- Test runner: `tests/e2e/test_runner.py`
- Invocation: `.venv/Scripts/python.exe -m tests.e2e.test_runner`
- Pass/Fail semantics: All tests must assert expected outputs, file generation, and numeric thresholds with exit code 0.
- Directory layout:
  - `tests/e2e/test_runner.py`
  - `tests/e2e/test_tier1_features.py`
  - `tests/e2e/test_tier2_boundaries.py`
  - `tests/e2e/test_tier3_combinations.py`
  - `tests/e2e/test_tier4_applications.py`

## Real-World Application Scenarios (Tier 4)
| # | Scenario | Features Exercised | Complexity |
|---|----------|--------------------|------------|
| 1 | Adverse Weather (Fog/Rain): Range & Beam Attenuation on Highway | F1, F2, F3, F4, F5, F6, F7 | High |
| 2 | Mechanical Vibration / Mount Shock: Extrinsic Drift vs VRU Safety | F1, F2, F4, F5, F8, F9 | High |
| 3 | Multi-dataset Benchmark: Synthetic + KITTI + nuScenes consistency | F1, F2, F3, F4, F6, F7 | High |
| 4 | End-to-End Pipeline & Automated Submission Gate Audit | F1-F14 | High |
| 5 | Interactive Demo Headless Run & Live Snapshot Export | F10, F11, F6, F7 | Medium |

## Coverage Thresholds
- Tier 1: ≥5 per feature
- Tier 2: ≥5 boundary & corner cases per feature
- Tier 3: pairwise coverage of major feature combinations
- Tier 4: ≥5 realistic end-to-end application scenarios
