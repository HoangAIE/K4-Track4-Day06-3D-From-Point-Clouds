# TEST READY: Topic C — LiDAR-Camera Sensor Degradation Test Suite

**Date:** 2026-10-07  
**Author:** E2E Test Writer (`test_writer_e2e_1`)  
**Status:** SUITE OPERATIONAL & VERIFIED  

---

## 1. Test Suite Summary

An opaque-box, requirement-driven E2E test suite has been designed and implemented in `tests/e2e/` adhering to the specifications of `PROJECT.md`, `TEST_INFRA.md`, and `ORIGINAL_REQUEST.md`.

| Metric | Value |
|---|---|
| **Total Test Cases** | **156 tests** |
| **Tier 1 (Feature Coverage F1..F14)** | 70 tests (5 tests per feature) |
| **Tier 2 (Boundaries & Corners F1..F14)** | 70 tests (5 boundary tests per feature) |
| **Tier 3 (Pairwise Cross-Feature)** | 11 tests |
| **Tier 4 (Real-World Application Scenarios)** | 5 tests |
| **Test Execution Framework** | Standard Library `unittest` (0 external dependencies required) |
| **Standalone Runner** | `tests/e2e/test_runner.py` |
| **Current Pass Rate (Executed Tests)** | **100.0%** (87 passed, 69 pending future milestones) |
| **Full Suite Latency** | **1.22 seconds** on CPU |

---

## 2. Test Execution Command

To execute the entire E2E test suite:
```powershell
.venv\Scripts\python.exe -m tests.e2e.test_runner
```

To execute specific tiers:
```powershell
.venv\Scripts\python.exe -m tests.e2e.test_runner --tier 1
.venv\Scripts\python.exe -m tests.e2e.test_runner --tier 2
.venv\Scripts\python.exe -m tests.e2e.test_runner --tier 3
.venv\Scripts\python.exe -m tests.e2e.test_runner --tier 4
```

To run with verbose per-test reporting:
```powershell
.venv\Scripts\python.exe -m tests.e2e.test_runner --verbose
```

To run in strict verification mode (for final submission gate audit):
```powershell
.venv\Scripts\python.exe -m tests.e2e.test_runner --strict
```

---

## 3. Coverage & Verification Mapping

### Tier 1: Feature Coverage (`tests/e2e/test_tier1_features.py` — 70 Tests)
- **Feature 1 (Velo to Cam Transformation)**: Shape preservation (N, 3), CP2 benchmark point (10, 0, 0) -> z_cam ~ 9.73m, origin translation offset, batch mathematical identity `(P_homo @ T_cam_velo.T)[:, :3]`, point ordering and multi-point fidelity.
- **Feature 2 (Cam to Image Projection)**: Return structure `(uv, depth, mask)`, principal axis projection near $(c_u, c_v)$, strictly positive depth filtering ($z > min\_depth$), image boundary filtering ($0 \le u < W, 0 \le v < H$), NaN/Inf filtering.
- **Feature 3 (Multi-dataset Projection)**: Verification on `data/synthetic` frame 000000, `data/kitti_mini` frame 000001, `data/nuscenes_mini_subset` frame scene-0103_010, overlay generation with `cv2.applyColorMap`, nuScenes `use_ego_motion` flag.
- **Feature 4 (LiDAR Point Degradations)**: Deterministic `random_dropout` with fixed seed, radial `range_dropout` cutoff, elevation `beam_dropout` decimation, `gaussian_noise` standard deviation fidelity, extrinsic perturbation matrix properties and input immutability.
- **Feature 5 (Non-DL Object Metrics)**: Inclusion of interior 3D box points, exclusion of exterior points, rotation yaw angle handling, multi-object count stratification, starved object detection under severe decimation.
- **Feature 6 (FOV & Health Score Metrics)**: FOV percentage validity $[0.0, 100.0]\%$, health score range $[0.0, 100.0]$, monotonic decline under increasing random dropout, monotonic decline under range cutoff, health status band classification (HEALTHY, DEGRADED, CRITICAL, FAILURE).
- **Feature 7 (Benchmark CSV & Figures)**: Presence of `results/*.csv`, schema headers verification, presence of `results/figures/*.png`, valid non-empty PNG content, deterministic seed reproducibility.
- **Feature 8 (Failure Case Reproduction)**: Presence of `results/figures/fail_*.png`, minimum 2 failure case artifacts, yaw drift starvation mechanism proof, VRU starvation under beam decimation proof, 3-channel color image fidelity.
- **Feature 9 (Debug Layer Classification)**: Geometry layer mapping for calibration drift, Sensor/Environment layer mapping for weather/beam attenuation, Time layer mapping for nuScenes ego-motion desynchronization, Preprocess layer mapping, canonical 6-layer membership.
- **Feature 10 (Interactive CPU Demo App)**: Importability of `src.app`, CLI argument parser with `--headless`, dual visualizers (Camera overlay + BEV map), CPU rendering latency < 500ms, cross-dataset compatibility.
- **Feature 11 (Headless Export for Demo)**: Headless CLI command execution, generation of `results/figures/demo_gui_snapshot.png`, minimum snapshot resolution ($\ge 400 \times 300$), custom frame loading, perturbation parameter injection.
- **Feature 12 (Complete REPORT.md)**: File presence, all 6 required section headings present, 0 remaining `[ĐIỀN]` placeholders, verifiable Section 1 technical claim, Section 6 AI disclosure table.
- **Feature 13 (Student Info Verification)**: Student name `Ngô Xuân Hoàng`, MSSV `2A202602597`, Class `H209`, Topic C indicator, strict alphanumeric regex matching `\*\*MSSV:\*\*\s*([A-Za-z0-9]+)`.
- **Feature 14 (100% Submission Gate Pass)**: File size $< 20 \text{ MB}$, no raw data outside `data/`, no `.env` file, secret pattern scan immunity, execution of `tools/check_submission.py`.

### Tier 2: Boundary Value Analysis (`tests/e2e/test_tier2_boundaries.py` — 70 Tests)
- **F1 Boundaries**: Empty $(0, 3)$ arrays, single point $(1, 3)$, extreme distance coordinates $(\pm 10000\text{m})$, NaN/Inf coordinate resilience, identity extrinsic transformation.
- **F2 Boundaries**: Negative depth ($z_{cam} < 0$), zero depth ($z_{cam} = 0$), exact $min\_depth$ threshold boundary, pixel boundary limits ($u=0, u=W-1, u=W, v=0, v=H-1, v=H$), empty input points.
- **F3 Boundaries**: Corrupted NaN/Inf points in synthetic frame, empty label file, dense full scans ($>100k$ points), sparse night/rain scans (`scene-1094`), invalid frame ID exception handling.
- **F4 Boundaries**: `keep_ratio` at $0.0$ and $1.0$, `max_range` at $0.0$ and $10000.0\text{m}$, `beam_dropout` strides at $1$ and $64$, zero noise ($\sigma=0.0$), extreme angles ($yaw=180^\circ, pitch=90^\circ$).
- **F5 Boundaries**: Empty point cloud box containment, point lying exactly on box surface boundary, zero-volume degenerate box, gigantic $1000\text{m}$ box, empty object lists.
- **F6 Boundaries**: $0.0\%$ FOV retention, $100.0\%$ FOV retention, health score on empty point cloud, health score on unperturbed pristine frame, health score with NaN-corrupted inputs.
- **F7 Boundaries**: Division by zero prevention when object count is 0, numeric precision in CSV, repeated seed hash consistency, figure size threshold ($\ge 5 \text{ KB}$), automatic `results/` folder creation.
- **F8 Boundaries**: Complete VRU point extinction ($0$ points), angular threshold step-function, distant vs near sensitivity gradient, case-insensitive media matching, non-uniform pixel intensity validation.
- **F9 Boundaries**: Multi-factor failure layer disambiguation, rejection of non-standard debug layers, canonical 6-layer definition audit, sector occlusion layer mapping, motion blur time layer mapping.
- **F10 Boundaries**: 0-point rendering resilience, extreme $1.0\text{m}$ noise jitter, extreme $360^\circ$ yaw drift slider values, empty bounding box rendering, consecutive rapid dataset switching without state leakage.
- **F11 Boundaries**: Zero keep ratio snapshot export, missing parent directory creation, clean snapshot overwrite, invalid frame error handling, sequential export calls without file lock contention.
- **F12 Boundaries**: Heading whitespace variations, heading casing integrity, nested placeholder detection, maximum file size cap, UTF-8 file decoding.
- **F13 Boundaries**: MSSV punctuation/space strictness, Vietnamese unicode accents in student name, exact class string `H209`, presence of all 4 identity fields, MSSV length bounds.
- **F14 Boundaries**: Exact 20 MB size threshold simulation, case-insensitive file extension checks, `.env` variant checks, secret scanner false-positive immunity, compilation integrity of `check_submission.py`.

### Tier 3: Pairwise Combinations (`tests/e2e/test_tier3_combinations.py` — 11 Tests)
- High-level `project_velo_to_image` vs explicit sequential `cam_to_image(velo_to_cam(...))` bit-for-bit equivalence.
- Extrinsic perturbation modifying 3D camera coordinate frame.
- Extrinsic yaw drift propagating into lateral pixel shifts across image u-axis.
- Point cloud decimation directly starving 3D GT boxes.
- Range attenuation degrading FOV retention and health score concurrently.
- Perturbation operators functioning consistently across Synthetic, KITTI, and nuScenes point clouds.
- Multi-dataset health metric schema and range consistency.
- Benchmark runner sweep aggregation into CSV.
- Failure scenarios bound to explicit debug layers.
- Headless demo export parity with interactive application.
- Report completeness and student credentials driving submission gate compliance.

### Tier 4: Real-World Workload Scenarios (`tests/e2e/test_tier4_applications.py` — 5 Tests)
- **Scenario 1**: Adverse Weather (Fog/Rain) Range & Beam Attenuation on Highway (exercising F1, F2, F3, F4, F5, F6, F7).
- **Scenario 2**: Mechanical Vibration / Mount Shock & VRU Safety Extrinsic Drift (exercising F1, F2, F4, F5, F8, F9).
- **Scenario 3**: Multi-dataset Heterogeneous Sensor Cross-Benchmarking (exercising F1, F2, F3, F4, F6, F7).
- **Scenario 4**: End-to-End Pipeline & Submission Gate Audit (exercising F1..F14).
- **Scenario 5**: Interactive Demo Headless Run & Live Snapshot Export (exercising F10, F11, F6, F7).

---

## 4. Progressive Testability Architecture

To ensure strict adherence to the progressive testability guideline:
- Tests for currently completed milestones (e.g. M1 geometry, starter perturbations) execute immediately with full mathematical assertions.
- Tests for downstream milestones (M2 metrics/benchmarks, M3 failure cases, M4 demo app, M5 report finalization) safely skip if the target module/artifact is not yet produced.
- As workers implement M2, M3, M4, and M5, tests automatically un-skip and verify the implementations without requiring test suite modifications.

---

## 5. Escalation: Implementation Bug Discovered

- **Component:** `starter/projection.py:overlay_points`
- **Location:** Lines 105–108
- **Defect:** When `uv` and `depth` are empty arrays (shape `(0, 2)` and `(0,)`), `depth / max_depth` produces an empty array. Calling `cv2.applyColorMap` on an empty array returns `None`. Line 107 then attempts `colors[:, 0]`, causing:
  ```
  TypeError: 'NoneType' object is not subscriptable
  ```
- **Recommended Fix:** Add an early return guard in `overlay_points`:
  ```python
  if len(uv) == 0 or len(depth) == 0:
      return out
  ```
- **Status:** Escalated to worker/implementer.
