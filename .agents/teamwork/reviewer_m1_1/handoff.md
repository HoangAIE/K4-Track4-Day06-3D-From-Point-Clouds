# Milestone 1 Review & Adversarial Challenge Report: Core Projection Geometry

**Reviewer / Critic Agent:** `reviewer_m1_1`  
**Review Target:** `starter/projection.py` (Worker: `worker_m1_1`)  
**Verdict:** **APPROVE**  
**Integrity Status:** **VERIFIED (NO VIOLATIONS DETECTED)**  
**Date:** 2026-10-07  

---

## 1. Observation

### 1.1 Source Code and Implementation
- Inspected `starter/projection.py` lines 27–45 (`velo_to_cam`):
  ```python
  def velo_to_cam(points_xyz: np.ndarray, calib: KittiCalib) -> np.ndarray:
      pts = np.asarray(points_xyz)
      if pts.size == 0 or len(pts) == 0:
          return np.empty((0, 3), dtype=float)
      was_1d = pts.ndim == 1
      if was_1d:
          pts = pts.reshape(1, -1)
      pts_3d = pts[:, :3]
      pts_homo = np.hstack([pts_3d, np.ones((len(pts_3d), 1), dtype=pts_3d.dtype)])
      pts_cam = (pts_homo @ calib.T_cam_velo.T)[:, :3]
      return pts_cam[0] if was_1d else pts_cam
  ```
- Inspected `starter/projection.py` lines 48–93 (`cam_to_image`):
  ```python
  def cam_to_image(points_cam: np.ndarray, P2: np.ndarray, image_shape: tuple[int, ...],
                   min_depth: float = 0.1) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
      pts = np.asarray(points_cam)
      if pts.size == 0 or len(pts) == 0:
          return np.empty((0, 2), dtype=float), np.empty((0,), dtype=float), np.zeros((0,), dtype=bool)
      if pts.ndim == 1:
          pts = pts.reshape(1, -1)

      N = len(pts)
      H, W = int(image_shape[0]), int(image_shape[1])
      pts_3d = pts[:, :3]
      finite_mask = np.isfinite(pts_3d).all(axis=1)
      pts_clean = np.where(finite_mask[:, None], pts_3d, 0.0)
      pts_homo = np.hstack([pts_clean, np.ones((N, 1), dtype=pts_clean.dtype)])

      proj = pts_homo @ P2.T
      s = proj[:, 2]
      z_cam = pts_3d[:, 2]

      valid_pre = finite_mask & (z_cam > min_depth) & (s > 1e-4)
      u = np.zeros(N, dtype=float)
      v = np.zeros(N, dtype=float)
      u[valid_pre] = proj[valid_pre, 0] / s[valid_pre]
      v[valid_pre] = proj[valid_pre, 1] / s[valid_pre]

      in_bounds = (u >= 0.0) & (u < W) & (v >= 0.0) & (v < H)
      mask = valid_pre & in_bounds

      uv = np.column_stack([u[mask], v[mask]])
      depth = z_cam[mask]
      return uv, depth, mask
  ```
- Checked lines 121–124 and 194–197 for Windows CLI stdout/stderr reconfiguration (`sys.stdout.reconfigure(encoding="utf-8")`), resolving the Windows `cp1252` encoding issue when printing argparse help.
- Checked `git diff starter/projection.py`: Only `velo_to_cam`, `cam_to_image`, and the UTF-8 stream configuration were touched. No changes outside ownership.

### 1.2 Independent Command Executions
1. **Synthetic Dataset (`data/synthetic`, frame `000000`)**:
   - Command: `.venv\Scripts\python.exe -m starter.projection --data-root data/synthetic --frame 000000 --out-dir results/figures`
   - Output: `points=23953 inside_image=3910 (16.3%) -> results\figures\overlay_000000_r0.0_p0.0_y0.0_t0.0_0.0_0.0.png`
   - Exit code: 0.

2. **KITTI Mini Dataset (`data/kitti_mini`, frame `000011`)**:
   - Command: `.venv\Scripts\python.exe -m starter.projection --data-root data/kitti_mini --frame 000011 --out-dir results/figures`
   - Output: `points=108004 inside_image=19946 (18.5%) -> results\figures\overlay_000011_r0.0_p0.0_y0.0_t0.0_0.0_0.0.png`
   - Exit code: 0.

3. **nuScenes Mini Subset (`data/nuscenes_mini_subset`, frame `scene-0103_010`)**:
   - Command: `.venv\Scripts\python.exe -m starter.projection --data-root data/nuscenes_mini_subset --frame scene-0103_010 --out-dir results/figures`
   - Output: `points=34720 inside_image=3120 (9.0%) -> results\figures\overlay_scene-0103_010_r0.0_p0.0_y0.0_t0.0_0.0_0.0.png`
   - Exit code: 0.

4. **Synthetic with Extrinsic Yaw Drift (`--yaw-deg 1.0`)**:
   - Command: `.venv\Scripts\python.exe -m starter.projection --data-root data/synthetic --frame 000000 --yaw-deg 1.0 --out-dir results/figures`
   - Output: `points=23953 inside_image=3935 (16.4%) -> results\figures\overlay_000000_r0.0_p0.0_y1.0_t0.0_0.0_0.0.png`
   - Exit code: 0.

5. **Milestone 1 Unit & Boundary Tests**:
   - Executed 13 Tier 1 tests: `Ran 13 tests in 0.202s -> OK`.
   - Executed 14 Tier 2 tests: `Ran 14 tests in 0.185s -> OK`.

### 1.3 Visual Overlay Inspection (`results/figures/`)
- `overlay_000000_r0.0_p0.0_y0.0_t0.0_0.0_0.0.png`: Point cloud precisely covers the 3D Car and Pedestrian targets. Depth colormap gradients match physical 3D scene depth smoothly (red foreground to dark blue background).
- `overlay_000011_r0.0_p0.0_y0.0_t0.0_0.0_0.0.png`: High-density KITTI scan perfectly overlays the vehicles, pedestrians, building facades, and roadway with exact geometric alignment.
- `overlay_scene-0103_010_r0.0_p0.0_y0.0_t0.0_0.0_0.0.png`: 32-beam nuScenes rings align with road surface and vehicles; bounding boxes encapsulate corresponding LiDAR clusters.
- `overlay_000000_r0.0_p0.0_y1.0_t0.0_0.0_0.0.png`: Demonstrates expected leftward pixel shift from a +1.0 deg yaw perturbation, validating calibration sensitivity.

---

## 2. Logic Chain

1. **Integrity Verification**:
   - Scanned implementation for hardcoded inputs, test case pattern matches, or facade stubs.
   - Both `velo_to_cam` and `cam_to_image` perform generalized NumPy linear algebra and projective calculations for arbitrary inputs $N \ge 0$.
   - No mock dictionaries or bypassed mathematical operations were found.

2. **Mathematical Correctness**:
   - **Extrinsic Transformation**: LiDAR point $p_{velo} = [X, Y, Z]^T$ is converted to homogeneous coordinates $[X, Y, Z, 1]^T$ and multiplied by $T_{cam\_velo} = R_{0,rect} \cdot Tr_{velo\_to\_cam}$ ($4 \times 4$). Since data points are stored as row vectors in NumPy, $([P, \mathbf{1}] \cdot T_{cam\_velo}^T)_{:, :3}$ accurately performs $T_{cam\_velo} \cdot p$.
   - **Benchmark Test**: For $p_{velo} = [10.0, 0.0, 0.0]$ on synthetic calib, $p_{cam} = [-0.00045, 0.02939, 9.72732]$. The forward depth $z_{cam} = 9.7273$ is strictly positive and matches the physical geometry of Velodyne x-forward mapped to camera z-forward.
   - **Intrinsic Projection & Perspective Division**: Projective coordinates $[s \cdot u, s \cdot v, s]^T = P_2 \cdot [p_{cam}, 1]^T$ are normalized by $s$. Division is strictly gated by `valid_pre = finite_mask & (z_cam > min_depth) & (s > 1e-4)`. Points behind the camera ($z \le min\_depth$) or with near-zero focal plane denominators ($s \le 10^{-4}$) are masked prior to division, preventing invalid float exceptions and erroneous negative-depth mirroring into the image plane.
   - **Boundary Enforcement**: Points are strictly constrained to $0 \le u < W$ and $0 \le v < H$.

3. **Data Robustness & Sanitization**:
   - Synthetic frame 000000 contains 23 non-finite points (NaN/Inf). `cam_to_image` safely isolates them using `finite_mask = np.isfinite(pts_3d).all(axis=1)` and replaces non-finite entries with 0.0 prior to matrix operations, preventing division warnings and ensuring that no NaN coordinates enter the output pixel array `uv`.
   - Empty input clouds (`shape == (0, 3)` or `(0, 4)`) return appropriate empty structures with correct dimensions: `uv` is `(0, 2)`, `depth` is `(0,)`, `mask` is `(0,) bool`.
   - Single-point inputs (`shape == (3,)`) are handled with shape preservation for both functions.

---

## 3. Adversarial Stress-Test Findings & Challenges

### Challenge 1: Extreme Coordinates & Massive Scale
- **Attack Scenario**: Evaluated performance with extreme coordinates ($10^{15}$) and a massive 1,000,000-point synthetic point cloud.
- **Result**:
  - Out-of-bounds coordinates were cleanly rejected by boundary checks without overflow.
  - 1,000,000 points were processed in 159 ms total (`velo_to_cam`: 33 ms, `cam_to_image`: 126 ms) on CPU. Fully vectorized and performant for real-time ADAS / demo usage.

### Challenge 2: Non-Contiguous & Fortran-Ordered Memory
- **Attack Scenario**: Evaluated inputs with sliced striding (`pts[::2]`) and Fortran-contiguous layouts (`np.asfortranarray(...)`).
- **Result**: Handled flawlessly by `np.asarray()` and `np.hstack()`.

### Challenge 3: Raw NaN Propagation in `velo_to_cam`
- **Observation**: If raw NaNs are passed directly to `velo_to_cam`, NumPy emits a `RuntimeWarning: invalid value encountered in matmul` at line 44 (`pts_cam = (pts_homo @ calib.T_cam_velo.T)[:, :3]`).
- **Impact Assessment**: Minor / Non-blocking. The resulting NaNs in `pts_cam` are cleanly intercepted and sanitized by `cam_to_image` without crashing or corrupting projections.
- **Suggestion for Future Refinement**: In downstream preprocessing, non-finite points can be filtered upfront at point-cloud loading time.

---

## 4. Caveats

- Milestone 1 strictly covers the core projection geometry in `starter/projection.py`.
- Submission gate evaluation: `tools/check_submission.py` currently reports `[FAIL]` on downstream requirements (stress test CSVs, failure case images, and report student placeholders) which belong to Milestones 2, 3, 4, and 5. This is normal and expected at Milestone 1 completion.
- No other caveats.

---

## 5. Conclusion

- **Verdict:** **APPROVE**
- **Integrity:** 100% genuine implementation. Zero cheating, zero hardcoding, zero facade methods.
- **Mathematical Conformance:** Extrinsic matrix multiplications, homogeneous coordinates, depth thresholding ($z > min\_depth$), and pinhole image bounds ($0 \le u < W, 0 \le v < H$) are mathematically rigorous.
- **Multi-Dataset Verification:** Fully verified on `data/synthetic`, `data/kitti_mini`, and `data/nuscenes_mini_subset`.
- **Recommendation:** Proceed immediately to Milestone 2 (Stress Test Pipeline & Sensor Health Metrics).

---

## 6. Verification Method

To independently reproduce this verification:

1. **Verify All Three Datasets**:
   ```powershell
   .venv\Scripts\python.exe -m starter.projection --data-root data/synthetic --frame 000000 --out-dir results/figures
   .venv\Scripts\python.exe -m starter.projection --data-root data/kitti_mini --frame 000011 --out-dir results/figures
   .venv\Scripts\python.exe -m starter.projection --data-root data/nuscenes_mini_subset --frame scene-0103_010 --out-dir results/figures
   ```
   *Expected outcome:* Exit code 0, outputs valid projection point counts (synthetic: 16.3%, kitti: 18.5%, nuscenes: 9.0%), and creates PNG images in `results/figures/`.

2. **Verify Mathematical Precision & Edge Cases**:
   ```powershell
   .venv\Scripts\python.exe -c "import numpy as np; from starter.kitti_io import load_calib; from starter.projection import velo_to_cam, cam_to_image; c = load_calib('data/synthetic/training/calib/000000.txt'); pt = velo_to_cam(np.array([[10.0, 0.0, 0.0]]), c); uv, d, m = cam_to_image(pt, c.P2, (375, 1242)); assert np.isclose(pt[0, 2], 9.7273, atol=0.01); assert np.isclose(uv[0, 0], 613.96, atol=0.5); assert np.isclose(uv[0, 1], 175.01, atol=0.5); assert m[0]; print('MATH VERIFIED')"
   ```
   *Expected outcome:* Prints `MATH VERIFIED`.

3. **Verify Milestone 1 E2E Test Suite**:
   ```powershell
   .venv\Scripts\python.exe -m unittest tests.e2e.test_tier1_features.TestTier1Features.test_f1_01_velo_to_cam_shape_and_dtype tests.e2e.test_tier1_features.TestTier1Features.test_f1_02_velo_to_cam_cp2_benchmark_point tests.e2e.test_tier1_features.TestTier1Features.test_f2_01_cam_to_image_return_structure tests.e2e.test_tier1_features.TestTier1Features.test_f3_01_multi_dataset_synthetic
   ```
   *Expected outcome:* All tests pass with code 0.
