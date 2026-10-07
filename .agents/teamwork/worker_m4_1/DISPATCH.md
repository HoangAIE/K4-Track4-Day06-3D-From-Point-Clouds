# Task Assignment: Milestone 4 — Interactive CPU Demo Application
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/ORIGINAL_REQUEST.md
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/PROJECT.md
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/explorer_survey_2/handoff.md
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/worker_m2_1/handoff.md

Write Ownership:
- `src/app.py` (New file)
- `results/figures/demo_gui_snapshot.png`

Tasks:
1. Implement interactive desktop application `src/app.py` using `tkinter` and `PIL` (Pillow), strictly runnable on CPU in `.venv`:
   - Controls (Left Panel):
     * Dataset dropdown: `synthetic`, `kitti_mini`, `nuscenes_mini_subset`
     * Frame selector: dropdown or spinbox with valid frames
     * Degradation sliders:
       - Random Dropout (0% to 80%)
       - Range Cutoff (10m to 80m)
       - Beam Count (64, 32, 16, 8, 4)
       - Gaussian Noise (0.0 to 0.20m)
       - Calibration Yaw Drift (-5.0 to +5.0 deg)
       - Calibration Pitch & Roll Drift (-2.0 to +2.0 deg)
     * Reset button (restores clean baseline)
   - Dual Visualizers (Right / Center Panel):
     * Camera View: Projected LiDAR points color-coded by depth, with 2D/3D GT bounding boxes overlaid.
     * BEV (Bird's-Eye-View) Map: Top-down projection of point cloud with vehicle origin and bounding boxes.
   - Live Telemetry (Bottom Panel):
     * Total LiDAR points remaining
     * % points inside Camera FOV
     * Points on Car, Pedestrian, Cyclist
     * Composite Sensor Health Score ($S_{health} \in [0, 100]$) and status banner (HEALTHY / DEGRADED / CRITICAL / FAILURE)
     * Processing Latency / FPS
2. Headless & Snapshot Support:
   - Provide `--headless` and `--export-snapshot <path>` CLI options so that automated test runners can invoke `python -m src.app --headless --export-snapshot results/figures/demo_gui_snapshot.png` to verify the UI engine, rendering pipeline, and metrics calculation without requiring an interactive display window.
3. Test execution:
   - Verify `python -m src.app --headless --export-snapshot results/figures/demo_gui_snapshot.png` runs smoothly and exits 0.
   - Run E2E test runner to ensure Tier 1-4 tests covering Feature 10 and Feature 11 pass.
4. Write handoff report to `.agents/teamwork/worker_m4_1/handoff.md`.
