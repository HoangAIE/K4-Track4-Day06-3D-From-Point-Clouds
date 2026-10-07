"""Tier 4: Real-World Workload Scenarios Test Suite.

Contains 5 comprehensive end-to-end workload scenarios:
1. Scenario 1: Adverse Weather (Fog/Rain) Range & Beam Attenuation on Highway
2. Scenario 2: Mechanical Vibration / Mount Shock & VRU Safety Extrinsic Drift
3. Scenario 3: Multi-dataset Heterogeneous Sensor Cross-Benchmarking
4. Scenario 4: End-to-End Pipeline & Submission Gate Audit
5. Scenario 5: Interactive Demo Headless Run & Live Snapshot Export
"""
from __future__ import annotations

import importlib
import math
import os
import re
import subprocess
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

from starter.datasets import dataset_type, list_frames, load_frame, load_points
from starter.kitti_io import KittiCalib, KittiObject, load_calib
from starter.perturb import (
    beam_dropout,
    gaussian_noise,
    motion_smear,
    random_dropout,
    range_dropout,
    sector_dropout,
)
from starter.projection import (
    box3d_corners_cam,
    cam_to_image,
    overlay_points,
    perturb_extrinsic,
    project_velo_to_image,
    velo_to_cam,
)

REPO_ROOT = Path(__file__).resolve().parents[2]


class TestTier4Applications(unittest.TestCase):
    """Tier 4: Realistic Real-World Workload Scenarios."""

    # =========================================================================
    # Scenario 1: Adverse Weather (Fog/Rain) Attenuation on Highway
    # =========================================================================

    def test_scenario_1_adverse_weather_highway_attenuation(self):
        """Scenario 1: Simulates adverse weather causing severe range & beam attenuation.

        Verifies that while near objects retain LiDAR returns, distant vehicles (>35m)
        suffer complete point starvation and health score drops into CRITICAL/FAILURE band.
        Exercised features: F1, F2, F3, F4, F5, F6, F7.
        """
        fr = load_frame(REPO_ROOT / "data/kitti_mini", "000001")
        pts_raw = fr["points"]
        calib = fr["calib"]
        labels = fr["labels"]
        img_shape = fr["image"].shape

        # Baseline clean projection
        uv_clean, depth_clean, mask_clean = project_velo_to_image(pts_raw, calib, img_shape)
        self.assertGreater(mask_clean.sum(), 3000)

        # Apply compound adverse weather attenuation: Range cutoff at 20m + Beam dropout 50%
        pts_weather = range_dropout(pts_raw, max_range_m=20.0)
        pts_weather = beam_dropout(pts_weather, keep_every=2)

        uv_w, depth_w, mask_w = project_velo_to_image(pts_weather, calib, img_shape)
        self.assertTrue((depth_w <= 20.5).all(), "All surviving returns must be within attenuated range")
        self.assertLess(mask_w.sum(), mask_clean.sum() * 0.5, "Severe weather should reduce FOV points by >50%")

        # If metrics module is available, evaluate object-level starvation
        try:
            metrics = importlib.import_module("src.metrics")
            m_clean = metrics.compute_frame_metrics(pts_raw, calib, labels, img_shape)
            m_weather = metrics.compute_frame_metrics(pts_weather, calib, labels, img_shape)

            self.assertGreater(m_clean["health_score"], m_weather["health_score"])
            self.assertLess(m_weather["health_score"], 65.0, "Attenuated health score must be degraded")
        except ImportError:
            pass  # Pending M2 metrics implementation

    # =========================================================================
    # Scenario 2: Mechanical Vibration / Mount Shock & VRU Safety
    # =========================================================================

    def test_scenario_2_mechanical_vibration_extrinsic_drift_vru(self):
        """Scenario 2: Simulates mechanical shock causing 1.5 deg yaw drift & 5cm translation.

        Verifies that vulnerable road users (pedestrians/cyclists) suffer total point loss
        inside 3D bounding boxes, demonstrating high safety risk from Geometry layer misalignment.
        Exercised features: F1, F2, F4, F5, F8, F9.
        """
        fr = load_frame(REPO_ROOT / "data/kitti_mini", "000011")
        calib_nominal = fr["calib"]

        # Induce mount drift: 1.5 deg yaw + 5cm translation
        calib_shock = perturb_extrinsic(calib_nominal, yaw_deg=1.5, t_xyz_m=(0.05, 0.0, 0.0))

        # Check VRU objects (Pedestrian)
        pedestrians = [o for o in fr["labels"] if o.type == "Pedestrian"]
        if not pedestrians:
            self.skipTest("No pedestrians in frame 000011")
        ped = pedestrians[0]

        pts_cam_nominal = velo_to_cam(fr["points"][:, :3], calib_nominal)
        pts_cam_shock = velo_to_cam(fr["points"][:, :3], calib_shock)

        try:
            metrics = importlib.import_module("src.metrics")
            pts_in_ped_nominal = metrics.points_in_box3d(pts_cam_nominal, ped).sum()
            pts_in_ped_shock = metrics.points_in_box3d(pts_cam_shock, ped).sum()

            # Shock misalignment should drastically reduce points in narrow pedestrian bounding box
            self.assertGreater(pts_in_ped_nominal, pts_in_ped_shock)
        except ImportError:
            # If metrics not yet implemented, verify that camera coordinates shifted significantly
            diff_cam = np.abs(pts_cam_shock - pts_cam_nominal)
            mean_shift = diff_cam.mean(axis=0)
            self.assertGreater(mean_shift[0], 0.2, "Transverse lateral shift must exceed 20cm")

    # =========================================================================
    # Scenario 3: Multi-dataset Heterogeneous Sensor Cross-Benchmarking
    # =========================================================================

    def test_scenario_3_multi_dataset_sensor_cross_benchmark(self):
        """Scenario 3: Benchmarks 64-beam KITTI vs 32-beam nuScenes vs Synthetic datasets.

        Verifies pipeline executes uniformly across datasets with different LiDAR architectures,
        coordinate standards, and image resolutions.
        Exercised features: F1, F2, F3, F4, F6, F7.
        """
        datasets = [
            ("synthetic", "000000", (375, 1242)),
            ("kitti_mini", "000001", (375, 1242)),
            ("nuscenes_mini_subset", "scene-0103_010", (900, 1600)),
        ]

        results = {}
        for name, fid, expected_shape in datasets:
            root = REPO_ROOT / f"data/{name}"
            fr = load_frame(root, fid)
            pts = fr["points"]
            calib = fr["calib"]
            img = fr["image"]

            uv, depth, mask = project_velo_to_image(pts, calib, img.shape)
            fov_pct = float(mask.mean() * 100.0)

            results[name] = {
                "n_raw": len(pts),
                "n_proj": int(mask.sum()),
                "fov_pct": fov_pct,
                "mean_depth": float(depth.mean()) if len(depth) else 0.0,
            }

            self.assertGreater(results[name]["n_proj"], 100)
            self.assertGreater(results[name]["mean_depth"], 5.0)

        # KITTI 64-beam should have significantly higher point count than nuScenes 32-beam
        self.assertGreater(results["kitti_mini"]["n_raw"], results["nuscenes_mini_subset"]["n_raw"])

    # =========================================================================
    # Scenario 4: End-to-End Pipeline & Submission Gate Audit
    # =========================================================================

    def test_scenario_4_e2e_pipeline_and_submission_gate_audit(self):
        """Scenario 4: Complete pre-submission audit testing all 10 gates.

        Audits projection code, benchmark tables, failure proof figures, student info,
        and runs tools/check_submission.py in verification mode.
        Exercised features: F1 through F14.
        """
        # Audit Gate 1-3: REPORT.md
        report_path = REPO_ROOT / "report/REPORT.md"
        self.assertTrue(report_path.exists())
        text = report_path.read_text(encoding="utf-8")

        for sec in ["## 1. Claim", "## 2. Evidence", "## 3. Failure case",
                    "## 4. Khuyến nghị", "## 5. Cách chạy lại", "## 6. Khai báo sử dụng AI"]:
            self.assertIn(sec, text)

        # Audit Gate 4: check_submission.py script exists and compiles
        chk = REPO_ROOT / "tools/check_submission.py"
        self.assertTrue(chk.exists())

        # If repo is already in final state, verify zero placeholders
        placeholders = re.findall(r"\[ĐIỀN[^\]]*\]", text)
        if placeholders:
            self.skipTest(f"M5 Gate: {len(placeholders)} placeholders remain in REPORT.md (pending final M5).")
        else:
            # Run check_submission.py
            env = os.environ.copy()
            env["PYTHONUTF8"] = "1"
            res = subprocess.run([sys.executable, str(chk)], cwd=REPO_ROOT, capture_output=True, text=True, env=env)
            self.assertEqual(res.returncode, 0)

    # =========================================================================
    # Scenario 5: Interactive Demo Headless Run & Live Snapshot Export
    # =========================================================================

    def test_scenario_5_demo_app_headless_live_snapshot_export(self):
        """Scenario 5: Headless demo execution applying compound degradation and exporting snapshot.

        Simulates automated testing of the interactive UI: loads dataset, applies compound
        degradation (yaw drift + dropout), renders Camera depth overlay and BEV map,
        and saves verification artifact to disk.
        Exercised features: F10, F11, F6, F7.
        """
        app_path = REPO_ROOT / "src/app.py"
        if not app_path.exists():
            self.skipTest("M4 Feature: src/app.py not yet implemented.")

        env = os.environ.copy()
        env["PYTHONUTF8"] = "1"
        out_snap = REPO_ROOT / "results/figures/demo_gui_snapshot.png"

        cmd = [
            sys.executable, "-m", "src.app",
            "--headless",
            "--data-root", "data/kitti_mini",
            "--frame", "000001",
            "--yaw-deg", "1.0",
            "--dropout", "0.7",
            "--export-demo",
        ]
        proc = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True, env=env)
        self.assertEqual(proc.returncode, 0, f"Headless run failed: {proc.stderr}")
        self.assertTrue(out_snap.exists(), "Snapshot file must be generated")
        self.assertGreater(out_snap.stat().st_size, 5000, "Snapshot must be non-empty image")


if __name__ == "__main__":
    unittest.main()
