# Task Assignment: Milestone 3 — Failure Cases & Critical Sensor Limits Analysis
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/ORIGINAL_REQUEST.md
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/PROJECT.md
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/explorer_survey_2/handoff.md
- Read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/worker_m2_1/handoff.md

Write Ownership:
- `src/failure_analysis.py` (New file)
- `results/figures/fail_*.png` (Generated failure figures)

Tasks:
1. Create `src/failure_analysis.py` to systematically reproduce and analyze at least 2 (ideally 3) distinct failure scenarios:
   - Scenario 1 (Geometry Layer): Extrinsic Calibration Drift (Yaw drift 1.0 deg and 2.0 deg).
     * Demonstrate how small rotational drift causes catastrophic displacement at distance ($e \approx d \cdot \sin(\Delta \psi)$).
     * Show points shifting off small objects (Cyclist/Pedestrian) onto road/background, dropping points in 3D GT box from 18+ to 0 (100% loss).
     * Save `results/figures/fail_01_extrinsic_drift.png`.
   - Scenario 2 (Sensor/Environment & Preprocess Layers): Distant VRU Starvation under Beam Dropout & Range Attenuation.
     * Demonstrate that small objects (pedestrians at >20m) suffer complete data starvation (<3 points) when beams drop from 64 to 16/8 or range is limited, while large vehicles still retain returns.
     * Save `results/figures/fail_02_beam_starvation.png`.
   - Scenario 3 (Time Layer - Bonus): nuScenes Ego-motion Desynchronization.
     * Demonstrate timestamp offset between LiDAR and camera when ignore_ego_motion=True on scene-0103_010.
     * Save `results/figures/fail_03_time_desync_nuScenes.png`.
2. Ensure failure images have clean side-by-side or multi-panel layouts with clear labels, bounding box overlays, and textual debug annotations indicating the debug layer.
3. Test running `python -m src.failure_analysis --out-dir results/figures`.
4. Run check_submission.py to verify failure image gate passes (`results/ có ảnh failure case (tên chứa 'fail')`).
5. Write handoff report to `.agents/teamwork/worker_m3_1/handoff.md`.


## 2026-10-07T10:25:40Z
You are the Milestone 3 Implementation Worker for Topic C.
Your working directory: d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/worker_m3_1
Your task assignment: d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/worker_m3_1/DISPATCH.md
MANDATORY: First read d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/ORIGINAL_REQUEST.md.
Also read d:/K4-Track4-Day06-3D-From-Point-Clouds/PROJECT.md, d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/explorer_survey_2/handoff.md, and d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/worker_m2_1/handoff.md.

Write Ownership:
- src/failure_analysis.py (New file)
- results/figures/fail_*.png

MANDATORY INTEGRITY WARNING:
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

Key Tasks:
1. Implement `src/failure_analysis.py`:
   - Systematically reproduce at least 2 (ideally 3) real failure cases:
     * Fail 1 (Geometry Layer): Extrinsic Calibration Drift (Yaw drift 1.0 deg and 2.0 deg). Show points shifting off small objects (Cyclist/Pedestrian) onto road, causing 100% loss of points in 3D GT box. Output: results/figures/fail_01_extrinsic_drift.png.
     * Fail 2 (Sensor/Environment & Preprocess Layers): Distant VRU Starvation under Beam Dropout & Range Attenuation. Show pedestrians at distance losing all returns under beam decimation (64 -> 8) or range limits (<20m) while vehicles retain points. Output: results/figures/fail_02_beam_starvation.png.
     * Fail 3 (Time Layer - Bonus): nuScenes Ego-motion Desynchronization. Show displacement when camera shutter and LiDAR sweep timestamps differ without ego motion compensation. Output: results/figures/fail_03_time_desync_nuScenes.png.
2. Ensure failure images have clean side-by-side or multi-panel layouts with clear labels, bounding box overlays, and textual debug annotations indicating the debug layer.
3. Test running `python -m src.failure_analysis --out-dir results/figures`.
4. Verify with `.venv/Scripts/python.exe tools/check_submission.py` that check 6 (`results/ có ảnh failure case (tên chứa 'fail')`) now evaluates to `[PASS]`.
5. Run E2E tests: `.venv/Scripts/python.exe -m tests.e2e.test_runner --tier 1` and `--tier 2`.
6. Write handoff report to d:/K4-Track4-Day06-3D-From-Point-Clouds/.agents/teamwork/worker_m3_1/handoff.md.
When finished, send a message to your parent.
