# Task Assignment: Milestone 1 — Core Projection Geometry Implementation
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/ORIGINAL_REQUEST.md
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/PROJECT.md
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/explorer_survey_1/handoff.md
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/spec_miner_survey_1/handoff.md
- Exclusively own and edit: starter/projection.py
- Implement:
  1. velo_to_cam(points_xyz, calib) -> np.ndarray:
     * Homogeneous coordinates (N, 4)
     * Matrix multiplication with calib.T_cam_velo: (points_homo @ calib.T_cam_velo.T)[:, :3]
  2. cam_to_image(points_cam, P2, image_shape, min_depth=0.1) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
     * Defensive handling of NaN/Inf (np.isfinite)
     * Homogeneous projection via P2
     * Filter points with depth > min_depth and perspective division s > 1e-4
     * Image boundary filtering 0 <= u < W and 0 <= v < H
     * Return (uv, depth, mask)
  3. Ensure Windows console encoding compatibility (e.g. sys.stdout.reconfigure(encoding='utf-8') at top of main block) so python -m starter.projection --help and commands run cleanly.
- Verify with commands:
  * python -m starter.projection --data-root data/synthetic --frame 000000 --out-dir results/figures
  * Test on data/kitti_mini and data/nuscenes_mini_subset
- Produce handoff report at .agents/teamwork/worker_m1_1/handoff.md

## 2026-10-07T09:50:48Z
You are the Milestone 1 Implementation Worker for Topic C.
Your working directory: d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/worker_m1_1
Your task assignment: d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/worker_m1_1/DISPATCH.md
MANDATORY: First read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/ORIGINAL_REQUEST.md.
Also read d:/K4-Track4-Day06-3D-From-Point-Clouds/PROJECT.md, d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/explorer_survey_1/handoff.md, and d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/spec_miner_survey_1/handoff.md.

Write ownership:
You EXCLUSIVELY own starter/projection.py. DO NOT edit files outside your ownership.

MANDATORY INTEGRITY WARNING:
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

Detailed Tasks:
1. Complete `velo_to_cam` in starter/projection.py:
   - Takes `(N, 3)` points and `calib: KittiCalib`.
   - Forms homogeneous coordinates `(N, 4)`.
   - Computes `(points_homo @ calib.T_cam_velo.T)[:, :3]`.
   - Handles empty arrays gracefully.
2. Complete `cam_to_image` in starter/projection.py:
   - Takes `(N, 3)` points, `P2: np.ndarray`, `image_shape: tuple`, `min_depth: float = 0.1`.
   - Finite filtering with `np.isfinite` (defensive against NaNs/Infs present in synthetic data).
   - Projects via P2: `proj = points_cam_homo @ P2.T`.
   - Filters `z_cam > min_depth` and perspective divisor `s > 1e-4`.
   - Divides `u = proj[:, 0] / s`, `v = proj[:, 1] / s`.
   - Filters boundaries `0 <= u < W` and `0 <= v < H` where `(H, W) = image_shape[:2]`.
   - Returns `(uv, depth, mask)` where `uv` is `(M, 2)`, `depth` is `(M,)`, `mask` is `(N,)` bool.
3. Windows UTF-8 stdout fix:
   - Ensure `sys.stdout.reconfigure(encoding="utf-8")` is included so CLI `python -m starter.projection --help` and printing Vietnamese characters does not crash on Windows `cp1252`.
4. Testing & Verification:
   - Run: `.venv/Scripts/python.exe -m starter.projection --data-root data/synthetic --frame 000000 --out-dir results/figures`
   - Test on `data/kitti_mini` and `data/nuscenes_mini_subset`.
   - Verify CP2 test point: `(10, 0, 0)` velodyne yields `z_cam ~ 9.73 > 0` and `(u, v) ~ (614, 175)`.
5. Document all commands, tests, outputs, and diffs in d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/worker_m1_1/handoff.md.
When finished, send a message to your parent.
