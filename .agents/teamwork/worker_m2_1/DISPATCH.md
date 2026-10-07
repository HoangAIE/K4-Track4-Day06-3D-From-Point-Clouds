## 2026-10-07T10:13:27Z
# Task Assignment: Milestone 2 — Stress Test Pipeline & Sensor Health Metrics
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/ORIGINAL_REQUEST.md
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/PROJECT.md
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/explorer_survey_2/handoff.md
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/explorer_survey_1/handoff.md

Write Ownership:
- `src/metrics.py` (New file)
- `src/stress_test.py` (New file)
- `starter/projection.py` (Only for adding guard in overlay_points when uv/depth is empty: if len(uv) == 0 or len(depth) == 0: return out.copy())

Tasks:
1. Create `src/metrics.py`:
   - `points_in_box3d(points_cam, obj: KittiObject) -> np.ndarray`:
     Vectorized testing of points inside 3D oriented bounding box via canonical rotation $R_y(\theta)^T$.
     Canonical boundaries: length $[-l/2, l/2]$, height $[-h, 0]$, width $[-w/2, w/2]$.
   - `compute_fov_ratio(points, calib, image_shape)`:
     Fraction / percentage of valid LiDAR points inside camera image FOV.
   - `compute_sensor_health_score(points, baseline_points, points_in_fov, baseline_fov, ...)`:
     Composite Sensor Health Score $\in [0, 100]$ combining point count retention, distance percentiles (p95), azimuth coverage (binning), and FOV retention.
   - `compute_frame_metrics(points, calib, labels, image_shape)`:
     Returns comprehensive metrics dict (n_points, pts_in_fov, pts_in_fov_pct, pts_car, pts_ped, pts_cyc, health_score).
2. Create `src/stress_test.py`:
   - Support 5 degradation modes:
     * Random Dropout (keep_ratio: 1.0, 0.8, 0.6, 0.4, 0.2)
     * Range Dropout (max_range_m: 80, 50, 30, 20, 10)
     * Beam Dropout (keep_every: 1, 2, 4, 8, 16)
     * Gaussian Noise (sigma_m: 0.0, 0.02, 0.05, 0.10, 0.20)
     * Calibration Drift (yaw_deg: 0.0, 0.5, 1.0, 2.0, 3.0, 5.0)
   - Support deterministic random seed (`seed=42`) for 100% reproducibility across runs.
   - Benchmark across datasets: `data/synthetic` (frames 000000-000004), `data/kitti_mini` (sample frames), and `data/nuscenes_mini_subset`.
   - Export benchmark CSV: `results/sensor_degradation_benchmark.csv`.
   - Generate informative, high-resolution plots in `results/figures/`:
     * `results/figures/degradation_curves.png`: Multi-panel degradation curves (Points on Car, Pedestrian, Cyclist, FOV %, and Health Score).
     * `results/figures/health_score_vs_intensity.png`: Comparison of health score drop across all 5 degradation modes.
     * `results/figures/vru_starvation_analysis.png`: Vulnerable Road User point loss comparison vs vehicle.
3. Fix `starter/projection.py:overlay_points` guard:
   - If `len(uv) == 0 or len(depth) == 0`: return `out.copy()`.
4. Run verification:
   - Run `python -m src.stress_test --seed 42 --out-csv results/sensor_degradation_benchmark.csv --out-dir results/figures`
   - Run `.venv/Scripts/python.exe -m tests.e2e.test_runner --tier 1`
   - Run `.venv/Scripts/python.exe -m tests.e2e.test_runner --tier 2`
   - Run `.venv/Scripts/python.exe -m tests.e2e.test_runner --tier 3`
5. Write handoff report to `.agents/teamwork/worker_m2_1/handoff.md`.
