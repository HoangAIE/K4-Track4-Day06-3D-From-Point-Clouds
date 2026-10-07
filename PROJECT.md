# Project: Topic C — Sensor Degradation Stress Test on LiDAR-Camera

## Architecture
- **Input Data**: 3 Datasets (`data/synthetic`, `data/kitti_mini`, `data/nuscenes_mini_subset`).
- **Core Geometry Layer (`starter/projection.py`)**:
  - `velo_to_cam`: Homogeneous transformation from Velodyne coordinate frame to rectified camera frame using `calib.T_cam_velo`.
  - `cam_to_image`: Perspective projection using camera matrix $P_2$, finite filtering, $z > min\_depth$ depth filtering, perspective division, and image bounds check.
- **Stress Testing & Health Analysis Engine (`src/`)**:
  - `src/stress_test.py`: Multi-level degradation runner (Random Dropout, Range Dropout, Beam Dropout, Gaussian Noise, Calibration Drift) with deterministic fixed seed (`seed=42`).
  - `src/metrics.py`: Model-free quantitative metrics (LiDAR points in 3D GT bounding boxes for Car, Pedestrian, Cyclist; camera FOV ratio; composite Sensor Health Score).
  - `src/failure_analysis.py`: Systematic failure case reproduction and debug layer classification (Geometry, Sensor/Environment, Preprocess, Time).
  - `src/app.py`: CPU-smooth interactive desktop UI (Tkinter + Pillow) with real-time Camera depth projection, BEV visualizer, and telemetry panel.
- **Verification & Submission Gate (`tools/`, `report/`)**:
  - `tools/check_submission.py`: 10 strict compliance gates.
  - `report/REPORT.md`: Comprehensive technical report with 6 required sections, zero placeholders, student info `Ngô Xuân Hoàng`, MSSV `2A202602597`, Class `H209`.

## Feature Inventory
| # | Feature | Description | Milestone | Source |
|---|---------|-------------|-----------|--------|
| 1 | Velo to Cam Transformation | Homogeneous transform `P_cam = (P_homo @ T_cam_velo.T)[:, :3]` | M1 | ORIGINAL_REQUEST §R1 |
| 2 | Cam to Image Projection | Pinhole projection, NaN/Inf filter, $z > min\_depth$ filter, FOV bounds | M1 | ORIGINAL_REQUEST §R1 |
| 3 | Multi-dataset Projection | Projection verified on synthetic, kitti_mini, nuscenes_mini_subset | M1 | ORIGINAL_REQUEST §R1 |
| 4 | LiDAR Point Degradations | 5 degradation types × 5 severity levels with deterministic seed 42 | M2 | ORIGINAL_REQUEST §R2 |
| 5 | Non-DL Object Metrics | Exact point counts inside 3D GT boxes (Car, Ped, Cyclist) | M2 | ORIGINAL_REQUEST §R2 |
| 6 | FOV & Health Score Metrics | Camera FOV retention ratio and composite Sensor Health Score | M2 | ORIGINAL_REQUEST §R2 |
| 7 | Benchmark CSV & Figures | Export `results/*.csv` and `results/figures/*.png` degradation curves | M2 | ORIGINAL_REQUEST §R2 |
| 8 | Failure Case Reproduction | Reproduce >=2 distinct safety failure scenarios with visual proofs | M3 | ORIGINAL_REQUEST §R3 |
| 9 | Debug Layer Classification | Classify failures into Geometry, Preprocess, Sensor/Environment layers | M3 | ORIGINAL_REQUEST §R3 |
| 10 | Interactive CPU Demo App | Tkinter app with dataset/frame selection, sliders, Camera + BEV | M4 | ORIGINAL_REQUEST §R4 |
| 11 | Headless Export for Demo | CLI mode (`--headless`, `--export-demo`) for automated testing | M4 | ORIGINAL_REQUEST §R4 |
| 12 | Complete REPORT.md | Fill all 6 sections with zero `[ĐIỀN]` placeholders | M5 | ORIGINAL_REQUEST §R5 |
| 13 | Student Info Verification | Student: Ngô Xuân Hoàng, MSSV: 2A202602597, Class: H209 | M5 | ORIGINAL_REQUEST §R5 |
| 14 | 100% Submission Gate Pass | `python tools/check_submission.py` exits 0 with `KẾT QUẢ: SẴN SÀNG NỘP` | M5 | ORIGINAL_REQUEST §R5 |

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| M1 | Core Projection Geometry | Implement `velo_to_cam` & `cam_to_image` in `starter/projection.py` | None | DONE |
| M2 | Stress Test Pipeline & Metrics | Create `src/stress_test.py` and `src/metrics.py`, produce CSV & figures | M1 | IN_PROGRESS |
| M3 | Failure Cases & Debug Layers | Create `src/failure_analysis.py`, save `results/figures/fail_*.png` | M1, M2 | PLANNED |
| M4 | Interactive CPU Demo App | Create `src/app.py` (Tkinter + Pillow) with dual Camera+BEV | M1, M2 | PLANNED |
| M5 | Report & Submission Gate | Finalize `report/REPORT.md`, pass `tools/check_submission.py` 100% | M1, M2, M3, M4 | PLANNED |
| M_FINAL | E2E Testing & Hardening | Full E2E test suite pass across all tiers (Tiers 1-4) & adversarial test | M1-M5 | IN_PROGRESS |

## Interface Contracts
### `starter.projection` ↔ `src.stress_test` & `src.app`
- `velo_to_cam(points_xyz: np.ndarray, calib: KittiCalib) -> np.ndarray`: Shape `(N, 3)` -> `(N, 3)`.
- `cam_to_image(points_cam: np.ndarray, P2: np.ndarray, image_shape: tuple[int, ...], min_depth: float = 0.1) -> tuple[np.ndarray, np.ndarray, np.ndarray]`: Returns `(uv, depth, mask)` where `uv` is `(M, 2)`, `depth` is `(M,)`, `mask` is `(N,)` bool.
- `project_velo_to_image(points: np.ndarray, calib: KittiCalib, image_shape: tuple[int, ...]) -> tuple[np.ndarray, np.ndarray, np.ndarray]`.

### `src.metrics`
- `points_in_box3d(points_cam: np.ndarray, obj: KittiObject) -> np.ndarray`: Boolean mask `(N,)` indicating points strictly inside the 3D oriented bounding box.
- `compute_frame_metrics(points: np.ndarray, calib: KittiCalib, labels: list[KittiObject], image_shape: tuple[int, ...]) -> dict`: Returns dictionary with keys: `n_points`, `pts_in_fov`, `pts_in_fov_pct`, `pts_car`, `pts_ped`, `pts_cyc`, `health_score`.

## Code Layout
- `starter/projection.py`: Core projection functions (`velo_to_cam`, `cam_to_image`).
- `src/metrics.py`: 3D GT box containment, FOV percentage, composite health score calculations.
- `src/stress_test.py`: Benchmark runner iterating degradations, levels, and frames; generates `results/sensor_degradation_benchmark.csv` and degradation plots.
- `src/failure_analysis.py`: Targeted generator for failure case scenarios producing `results/figures/fail_*.png`.
- `src/app.py`: Interactive Tkinter GUI application with camera overlay, BEV map, and real-time controls.
- `tests/e2e/`: Requirement-driven E2E test runner and test cases.
- `results/`: Benchmark CSV files and generated figures.
- `report/REPORT.md`: Student report and submission documentation.
