# Handoff Report: Milestone 1 — Adversarial & Empirical Verification

**Agent:** `challenger_m1_1` (Milestone 1 Empirical Challenger & Critic)  
**Date:** 2026-10-07  
**Working Directory:** `d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/challenger_m1_1`  
**Target Codebase:** `starter/projection.py`  
**Test Suite Created:** `tests/test_challenger_m1_oracle.py`  
**Verdict:** **APPROVE**

---

## 1. Observation

### 1.1 Direct Test Invocations and Results
1. **Challenger Oracle Test Suite (`tests/test_challenger_m1_oracle.py`)**:
   - Command: `.venv\Scripts\python.exe -m unittest tests.test_challenger_m1_oracle -v`
   - Outcome: Exit code 0, 14/14 tests passed in 0.414s:
     - `test_adversarial_nan_and_inf_robustness`: PASS
     - `test_adversarial_shapes_and_dtypes`: PASS
     - `test_boundary_depth_filtering`: PASS
     - `test_boundary_pixel_coordinates`: PASS
     - `test_cp2_benchmark_kitti_mini`: PASS
     - `test_cp2_benchmark_nuscenes`: PASS
     - `test_cp2_benchmark_synthetic`: PASS
     - `test_full_pipeline_kitti_mini_dataset`: PASS
     - `test_full_pipeline_nuscenes_dataset`: PASS
     - `test_full_pipeline_synthetic_dataset`: PASS
     - `test_oracle_cam_to_image_analytical_pinhole`: PASS
     - `test_oracle_velo_to_cam_analytical_equivalence`: PASS
     - `test_oracle_velo_to_cam_inverse_reversibility`: PASS
     - `test_stress_one_million_points_throughput`: PASS (1M points in 174.3ms on CPU).

2. **Existing E2E Test Suite Tiers 1–3**:
   - Tier 1: `.venv\Scripts\python.exe -m tests.e2e.test_runner --tier 1` -> 35 passed, 0 failed, 35 skipped (M2–M5 features). Total time: 0.649s.
   - Tier 2: `.venv\Scripts\python.exe -m tests.e2e.test_runner --tier 2` -> 45 passed, 0 failed, 25 skipped. Total time: 0.486s.
   - Tier 3: `.venv\Scripts\python.exe -m tests.e2e.test_runner --tier 3` -> 4 passed, 0 failed, 7 skipped. Total time: 0.204s.

3. **CP2 Benchmark Evaluation**:
   - Point: $p_{velo} = [10.0, 0.0, 0.0]$ with `data/synthetic/training/calib/000000.txt`.
   - `velo_to_cam`: $p_{cam} = [-0.00045, 0.02939, 9.72732]$ ($z_{cam} = 9.7273 > 0$).
   - `cam_to_image`: $(u, v) = (613.96, 175.01)$, $depth = 9.7273$, $mask = True$.
   - Point: $p_{velo} = [10.0, 0.0, 0.0]$ with `data/kitti_mini/training/calib/000001.txt`.
   - `velo_to_cam`: $z_{cam} = 9.7231 > 0$, $(u, v) = (607.74, 187.97)$, $mask = True$.
   - nuScenes Forward Point: $p_{lidar} = [0.0, 10.0, 0.0]$ with `data/nuscenes_mini_subset` `scene-0103_010`.
   - `velo_to_cam`: $z_{cam} = 9.5584 > 0$, $(u, v) = (840.05, 493.33)$, $mask = True$.

4. **Multi-Dataset CLI Execution**:
   - Synthetic frame 000000: `points=23953 inside_image=3910 (16.3%) -> results\figures\overlay_000000_r0.0_p0.0_y0.0_t0.0_0.0_0.0.png`.
   - KITTI frame 000011: `points=108004 inside_image=19946 (18.5%) -> results\figures\overlay_000011_r0.0_p0.0_y0.0_t0.0_0.0_0.0.png`.
   - nuScenes frame scene-0103_010: `points=34720 inside_image=3120 (9.0%) -> results\figures\overlay_scene-0103_010_r0.0_p0.0_y0.0_t0.0_0.0_0.0.png`.

5. **Adversarial Edge Findings**:
   - `velo_to_cam` at line 44 emits `RuntimeWarning: invalid value encountered in matmul` when passed raw NaN values because `pts_homo @ calib.T_cam_velo.T` is computed before filtering. Downstream `cam_to_image` safely isolates and discards NaNs via `finite_mask`.
   - `project_velo_to_image` at line 98 contains `points[:, :3]`, which expects `ndim >= 2`. Calling `project_velo_to_image(np.array([10.0, 0.0, 0.0]), ...)` raises `IndexError: too many indices for array`, whereas calling `velo_to_cam` directly supports 1D inputs.
   - `test_scenario_1_adverse_weather_highway_attenuation` in `tests/e2e/test_tier4_applications.py` failed due to an over-restrictive assertion threshold (asserted surviving FOV points $< 40\%$, but actual surviving returns were $43.3\%$ on frame 000001). This test belongs to Milestone 4/Final scope.

---

## 2. Logic Chain

1. **Analytical Oracle Conformance**:
   - Observation 1.1 confirms that for 10,000 randomized points, `velo_to_cam` matches independent matrix multiplication $P_{velo} \cdot R^T + t$ within float64 precision ($< 10^{-6}$ error).
   - Round-trip reconstruction $(P_{cam} - t) \cdot R$ restores original coordinates within tolerance ($10^{-5}$), proving that `calib.T_cam_velo` preserves rigid $SE(3)$ invariants.
   - For 20,000 points spanning full 3D space, `cam_to_image` matches the closed-form pinhole model $u = (f_x x + c_u z + P_{03})/s$ and $v = (f_y y + c_v z + P_{13})/s$ bit-for-bit.
2. **CP2 Geometric Benchmark**:
   - Observation 1.3 shows that Velodyne point $(10, 0, 0)$ maps to $z_{cam} = 9.7273 > 0$, satisfying the physical constraint that objects in front of the vehicle have positive camera depth.
   - Projection into synthetic camera produces $(u, v) = (613.96, 175.01)$, matching the expected values $(\approx 614, 175)$ within $0.05$ pixels.
   - In nuScenes, where the sensor frame aligns $y$ forward and $x$ right (Observation 1.3), the forward point $(0, 10, 0)$ projects to $z_{cam} = 9.558 > 0$ and $(u, v) = (840.05, 493.33)$ near the principal point of the $900 \times 1600$ CAM_FRONT.
3. **Frustum & Boundary Enforcement**:
   - Observation 1.1 (`test_boundary_depth_filtering`) demonstrates that points with $z \le 0$ (behind camera), $z = 0$ (focal plane), and $z \le min\_depth$ are strictly masked out.
   - Observation 1.1 (`test_boundary_pixel_coordinates`) demonstrates that boundary conditions are strictly half-open $[0, W) \times [0, H)$: pixels at $u=W$ or $v=H$ evaluate to `False`, preventing downstream buffer out-of-bounds errors.
4. **Adversarial Resilience & High-Throughput CPU Performance**:
   - Observation 1.1 (`test_adversarial_nan_and_inf_robustness`) confirms that point clouds with combinations of NaN, $\pm\infty$, and corrupt numbers produce zero NaN/Inf values in the output `(uv, depth)`.
   - Observation 1.1 (`test_stress_one_million_points_throughput`) verifies that 1,000,000 points project in 174.3 ms on CPU without memory issues, well below the 1.5s real-time threshold.

---

## 3. Adversarial Review & Challenges

### Challenge Summary
- **Overall risk assessment**: **LOW** (Core projection implementation is robust, correct, and verified).

### Challenges Identified

#### [Low] Challenge 1: Matrix Multiplication RuntimeWarning on Raw Non-Finite Inputs
- **Assumption challenged**: Raw LiDAR points are cleaned before any matrix multiplication.
- **Attack scenario**: Passing uncleaned point clouds with NaNs (such as `data/synthetic/training/velodyne/000000.bin`) into `velo_to_cam` triggers `RuntimeWarning: invalid value encountered in matmul` at line 44.
- **Blast radius**: Cosmetic console warning. Downstream output is unaffected because `cam_to_image` applies `finite_mask` to discard NaNs before perspective division.
- **Mitigation**: Optionally add defensive finite filtering in `velo_to_cam` or ignore warning in CLI.

#### [Low] Challenge 2: 1D Input Inconsistency in Helper Function
- **Assumption challenged**: `project_velo_to_image` accepts all inputs accepted by `velo_to_cam`.
- **Attack scenario**: A caller passing a single 1D point `np.array([10.0, 0.0, 0.0])` into `project_velo_to_image` causes `IndexError: too many indices for array` at line 98 due to `points[:, :3]`.
- **Blast radius**: Limited to callers passing 1D points to `project_velo_to_image`. `velo_to_cam` directly supports 1D inputs.
- **Mitigation**: Recommend Milestone 2 workers ensure point clouds passed to `project_velo_to_image` have shape `(N, 3)` or `(N, 4)`.

---

## 4. Caveats

1. **nuScenes Forward Axis**:
   - In KITTI / synthetic data, the LiDAR forward axis is $+x$.
   - In nuScenes data, the LiDAR forward axis is $+y$ ($+x$ is lateral right).
   - When benchmarking nuScenes forward points, tests must use $(0, 10, 0)$ rather than $(10, 0, 0)$.
2. **Tier 4 Application Test**:
   - `test_scenario_1_adverse_weather_highway_attenuation` in `tests/e2e/test_tier4_applications.py` fails on frame 000001 due to a hardcoded $< 40\%$ retention assertion. This test is part of Milestone 4/Final and does not reflect an issue in `starter/projection.py`.

---

## 5. Conclusion & Empirical Verdict

- **Verdict**: **APPROVE**
- `starter/projection.py` satisfies all requirements of Milestone 1, Section §R1 of `ORIGINAL_REQUEST.md`, and Checkpoint CP2.
- The implementation is mathematically accurate, handles all tested boundary and corrupted point conditions safely, and executes smoothly across Synthetic, KITTI mini, and nuScenes mini datasets.
- Milestone 1 is verified ready for Milestone 2 (`src/stress_test.py` and `src/metrics.py`).

---

## 6. Verification Method

To independently verify this evaluation, execute:

1. **Run the Challenger Oracle Test Suite**:
   ```powershell
   .venv\Scripts\python.exe -m unittest tests.test_challenger_m1_oracle -v
   ```
   *Expected outcome:* All 14 tests pass with exit code 0 in $< 1.0$s.

2. **Run E2E Tiers 1–3**:
   ```powershell
   .venv\Scripts\python.exe -m tests.e2e.test_runner --tier 1
   .venv\Scripts\python.exe -m tests.e2e.test_runner --tier 2
   .venv\Scripts\python.exe -m tests.e2e.test_runner --tier 3
   ```
   *Expected outcome:* All executed tests pass with exit code 0.

3. **Verify CP2 Benchmark Assertions**:
   ```powershell
   .venv\Scripts\python.exe -c "import numpy as np; from starter.kitti_io import load_calib; from starter.projection import velo_to_cam, cam_to_image; calib = load_calib('data/synthetic/training/calib/000000.txt'); pt = velo_to_cam(np.array([[10.0, 0.0, 0.0]]), calib); uv, d, m = cam_to_image(pt, calib.P2, (375, 1242)); assert np.isclose(pt[0, 2], 9.7273, atol=0.01); assert np.isclose(uv[0, 0], 613.96, atol=0.5); assert np.isclose(uv[0, 1], 175.01, atol=0.5); assert m[0]; print('PASS CP2')"
   ```
   *Expected outcome:* Prints `PASS CP2` and exits 0.

4. **Invalidation Conditions**:
   - If `test_challenger_m1_oracle` fails any assertion.
   - If point $(10, 0, 0)$ maps to negative $z_{cam}$ or invalid $(u, v)$ coordinates.
   - If non-finite values result in unhandled exceptions or NaN leakage into `(uv, depth)`.
