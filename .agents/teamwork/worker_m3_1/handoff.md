# Handoff Report: Milestone 3 — Failure Cases & Critical Sensor Limits Analysis

**Author:** Milestone 3 Implementation Worker (`worker_m3_1`)  
**Date:** 2026-10-07  
**Working Directory:** `d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/worker_m3_1`  
**Target Audience:** Orchestrator, M4 Demo App Worker, M5 Report Worker, Teamwork Auditor  

---

## 1. Observation

### 1.1 Implementation of `src/failure_analysis.py`
- **File created:** `d:/K4-Track4-Day06-3D-From-Point-Clouds/src/failure_analysis.py` (530 lines).
- **Canonical Debug Layers Defined:**
  `CANONICAL_DEBUG_LAYERS = {"I/O", "Geometry", "Time", "Preprocess", "Model", "Metric", "Sensor/Environment", "Sensor"}`
- **Registered Failure Scenarios (`FAILURE_SCENARIOS`):**
  1. `fail_01_extrinsic_drift`:
     - **Title:** "LiDAR-Camera Extrinsic Calibration Drift (Yaw Misalignment)"
     - **Layer:** `Geometry`
     - **Dataset:** `data/kitti_mini`, frame `000001`
     - **Output:** `results/figures/fail_01_extrinsic_drift.png`
     - **Quantitative Observation:**
       Cyclist is located at distance $d = 46.1\text{ m}$. At baseline ($0.0^\circ$ drift), $18\text{ points}$ lie inside the 3D bounding box.
       At $+1.0^\circ$ yaw drift, transverse linear displacement $e \approx d \cdot \sin(1.0^\circ) = 0.805\text{ m}$ exceeds cyclist bounding box half-width ($0.3\text{ m}$), causing points inside 3D box to drop from **18 to 0 points (100% loss / total extinction)**.
       Mean pixel shift of projected points: $[-12.78\text{ px}, +0.14\text{ px}]$ shifts all laser returns off the cyclist chassis onto pavement.
  2. `fail_02_beam_starvation`:
     - **Title:** "Distant VRU Starvation under Beam Decimation & Range Attenuation"
     - **Layer:** `Sensor/Environment` (with Preprocess considerations)
     - **Dataset:** `data/kitti_mini`, frame `000011`
     - **Output:** `results/figures/fail_02_beam_starvation.png`
     - **Quantitative Observation:**
       Distant pedestrian ($d = 34.2\text{ m}$) drops from $40\text{ points}$ (64-beam baseline) to $5\text{ points}$ ($8\text{-beam}$, starvation) and $0\text{ points}$ ($4\text{-beam}$, 100% extinction).
       Under heavy rain/fog range attenuation ($R_{max} \le 20\text{ m}$), distant pedestrian loses all returns (**0 points, 100% extinction**).
       In sharp contrast, the nearby vehicle ($d = 6.6\text{ m}$) retains $412\text{ points}$ at $8\text{-beam}$ and $3251\text{ points}$ at $R_{max} = 20\text{ m}$ ($100\%$ retention), showing the asymmetric vulnerability of VRUs compared to vehicles.
  3. `fail_03_time_desync_nuScenes`:
     - **Title:** "Ego-motion Temporal Desynchronization between LiDAR and Camera"
     - **Layer:** `Time`
     - **Dataset:** `data/nuscenes_mini_subset`, frame `scene-0103_010`
     - **Output:** `results/figures/fail_03_time_desync_nuScenes.png`
     - **Quantitative Observation:**
       Camera shutter and LiDAR sweep timestamps differ by $\Delta t = -35.62\text{ ms}$.
       At ego speed $v \approx 8.65\text{ m/s}$ ($31\text{ km/h}$), omitting ego motion compensation (`use_ego_motion=False`) induces a spatial translation error of $\bar{\Delta x} = 0.308\text{ m}$ (max $0.360\text{ m}$).
       Vehicle returns inside 3D bounding boxes plummet:
       - Car 1: $38\text{ points} \to 9\text{ points}$ (**76% loss**)
       - Car 3: $24\text{ points} \to 6\text{ points}$ (**75% loss**)
       - Car 9: $25\text{ points} \to 7\text{ points}$ (**72% loss**)

### 1.2 Generated Visual Media Artifacts
- **CLI Command Executed:** `python -m src.failure_analysis --out-dir results/figures`
  - `results/figures/fail_01_extrinsic_drift.png`: $1800 \times 2700 \times 3$, size: $2552.0\text{ KB}$, std: $72.04$.
  - `results/figures/fail_02_beam_starvation.png`: $1800 \times 2700 \times 3$, size: $3123.7\text{ KB}$, std: $65.85$.
  - `results/figures/fail_03_time_desync_nuScenes.png`: $1800 \times 2700 \times 3$, size: $2954.2\text{ KB}$, std: $63.36$.
- All 3 figures feature:
  - 2x2 multi-panel layout at 150 DPI with high-contrast technical dark theme (`#16181D`).
  - 3D wireframe bounding box projections and depth-colored LiDAR overlays.
  - High-magnification zoom callout insets highlighting critical point shifts.
  - Explicit debug layer badges (`[DEBUG LAYER: GEOMETRY]`, `[DEBUG LAYER: SENSOR/ENVIRONMENT]`, `[DEBUG LAYER: TIME]`).
  - Quantitative degradation plots (curve and bar charts) with kinematic and geometric telemetry text boxes.

### 1.3 Submission Gate Verification
- **Command:** `.venv/Scripts/python.exe tools/check_submission.py`
- **Result for Check 6:**
  `[PASS] results/ có ảnh failure case (tên chứa 'fail')` (previously `[FAIL]`).
- Only remaining non-passing checks are REPORT.md placeholder items assigned to Milestone 5.

### 1.4 Test Suite Verification Results
- **Tier 1 (`tests.e2e.test_runner --tier 1`):**
  `70 total | 63 pass | 0 fail | 0 err | 7 skip | Rate: 100.0%`
  (All 10 tests for F8 Failure Reproduction and F9 Debug Layer Classification passed cleanly).
- **Tier 2 (`tests.e2e.test_runner --tier 2`):**
  `70 total | 65 pass | 0 fail | 0 err | 5 skip | Rate: 100.0%`
  (All boundary tests for failure cases B8 and debug layers B9 passed cleanly).
- **Tier 3 (`tests.e2e.test_runner --tier 3`):**
  `11 total | 10 pass | 0 fail | 0 err | 1 skip | Rate: 100.0%`
  (`test_combo_f8_f9_failure_cases_bound_to_debug_layers` passed cleanly).
- **Tier 4 (`tests.e2e.test_runner --tier 4`):**
  `5 total | 4 pass | 0 fail | 0 err | 1 skip | Rate: 100.0%`
- **Dedicated Unit Test Suite (`tests/test_failure_analysis.py`):**
  `7 total | 7 pass | 0 fail | 0 err | Rate: 100.0%`
- **Challenger Oracle (`tests.test_challenger_m1_oracle`):**
  `14 total | 14 pass | 0 fail | 0 err | Rate: 100.0%`

---

## 2. Logic Chain

1. **Distance Amplification of Rotational Drift (Geometry Layer)**:
   In perspective projection, rotational errors in sensor extrinsics do not produce constant pixel or spatial offsets; rather, transverse displacement scales linearly with radial distance:
   $$e \approx d \cdot \sin(\Delta \psi)$$
   For a cyclist at $d = 46.1\text{ m}$, a minor yaw drift of $\Delta \psi = 1.0^\circ$ ($0.01745\text{ rad}$) generates $e = 0.805\text{ m}$ of lateral shift.
   Because a cyclist or pedestrian has an effective cross-sectional width of only $\approx 0.6\text{ m}$ (half-width $0.3\text{ m}$), an offset of $0.805\text{ m}$ displaces every single laser return outside the 3D ground truth bounding box.
   This explains why small objects suffer 100% point loss at drift levels where large vehicles ($w \ge 1.8\text{ m}$) still retain overlapping returns.

2. **Geometric Cross-Section Asymmetry under Decimation (Sensor/Environment Layer)**:
   LiDAR laser pulses diverge at fixed angular increments $\Delta \theta$.
   The spatial beam separation at range $r$ expands as $\Delta s \approx r \cdot \Delta \theta$.
   When beam count is decimated from 64 to 8 beams ($8\times$ sparser elevation resolution), the vertical gap between scan lines exceeds the height of a distant pedestrian ($h \approx 1.7\text{ m}$ at $r = 34.2\text{ m}$), causing rays to overshoot or undershoot the pedestrian entirely.
   Conversely, a passenger vehicle ($h \approx 1.5\text{ m}, w \approx 1.6\text{ m}, l \approx 4.0\text{ m}$) presents a large physical target that continues to intercept multiple beams.
   In atmospheric attenuation (modeled via range cutoff $R \le 20\text{ m}$), all photons scattered by aerosols drop below the receiver SNR threshold beyond $20\text{ m}$, completely extinguishing distant VRU detection while proximal vehicle tracking remains unaffected.

3. **Temporal Kinematics & Desynchronization (Time Layer)**:
   In modern autonomous driving sensor suites, camera rolling/global shutters and rotating LiDAR sweeps operate on unsynchronized clocks or with intentional phase offsets.
   On nuScenes `scene-0103_010`, the camera frame timestamp and LiDAR sweep timestamp exhibit a temporal discrepancy of $\Delta t = -35.62\text{ ms}$.
   When the ego vehicle travels at $v = 8.65\text{ m/s}$ ($31.1\text{ km/h}$), the vehicle translates by:
   $$\Delta x = v \cdot |\Delta t| = 8.65 \times 0.03562 \approx 0.308\text{ m}$$
   Omitting continuous-time ego-motion transformation (`use_ego_motion=False`) projects LiDAR returns from their position at $t_{lidar}$ directly using the camera pose at $t_{cam}$.
   This $0.31\text{ m}$ translational mismatch causes point clouds on vehicle bodies to shear off the chassis onto background pavement, discarding up to $76\%$ of valid bounding box points.

---

## 3. Caveats

1. **Synthetic Dataset Scope**:
   Failure scenarios 1 and 2 are built on `data/kitti_mini` (providing real Velodyne 64-beam data with complex 3D annotations including Cyclist, Pedestrian, Car, and Truck), and scenario 3 is built on `data/nuscenes_mini_subset` (providing real asynchronous sensor timestamps and calibrated ego poses). `data/synthetic` was not used for failure analysis because its 24k points and box geometries are static synthetic models without sensor timestamps or high-beam resolution.
2. **Deterministic Outputs**:
   `src/failure_analysis.py` contains 0 non-deterministic random calls; running `python -m src.failure_analysis` always produces bit-for-bit identical figures.
3. **No Caveats within Scope**:
   All Milestone 3 deliverables are 100% completed, verified, and integrated.

---

## 4. Conclusion

1. `src/failure_analysis.py` is implemented, fully tested, and cleanly integrated.
2. Three real, non-DL failure scenarios were systematically reproduced and classified into canonical debug layers:
   - Scenario 1 (`Geometry`): Extrinsic Calibration Drift ($\Delta \psi = 1.0^\circ, 2.0^\circ$).
   - Scenario 2 (`Sensor/Environment`): Distant VRU Starvation under Beam Decimation (8-beam) & Range Attenuation ($R_{max} \le 20\text{m}$).
   - Scenario 3 (`Time`): nuScenes Ego-motion Desynchronization ($\Delta t = -35.6\text{ms}, \Delta x = 0.31\text{m}$).
3. All three publication-grade figures (`fail_01_extrinsic_drift.png`, `fail_02_beam_starvation.png`, `fail_03_time_desync_nuScenes.png`) are generated in `results/figures/` with high resolution ($1800 \times 2700$), high contrast, 3D bounding boxes, zoom insets, and quantitative diagnostics.
4. `tools/check_submission.py` Gate 6 evaluates to `[PASS]`.
5. All E2E test runner tiers pass with 100.0% pass rate. Dedicated unit test suite passes 100%.

---

## 5. Verification Method

To independently reproduce and verify this work:

1. **Run Failure Analysis CLI**:
   ```powershell
   .venv\Scripts\python.exe -m src.failure_analysis --out-dir results/figures
   ```
   *Expected output:* Exit code 0, confirms generation of all 3 figures (`fail_01_extrinsic_drift.png`, `fail_02_beam_starvation.png`, `fail_03_time_desync_nuScenes.png`).

2. **Verify Submission Gate 6**:
   ```powershell
   .venv\Scripts\python.exe tools/check_submission.py
   ```
   *Expected output:* Line `[PASS] results/ có ảnh failure case (tên chứa 'fail')` is printed.

3. **Run E2E Test Suite Tiers 1 through 4**:
   ```powershell
   .venv\Scripts\python.exe -m tests.e2e.test_runner --tier 1
   .venv\Scripts\python.exe -m tests.e2e.test_runner --tier 2
   .venv\Scripts\python.exe -m tests.e2e.test_runner --tier 3
   .venv\Scripts\python.exe -m tests.e2e.test_runner --tier 4
   ```
   *Expected output:* 100.0% pass rate across all tiers with 0 failures and 0 errors.

4. **Run Dedicated Failure Analysis Unit Tests**:
   ```powershell
   .venv\Scripts\python.exe -m unittest tests/test_failure_analysis.py
   ```
   *Expected output:* 7 tests ran, OK.
