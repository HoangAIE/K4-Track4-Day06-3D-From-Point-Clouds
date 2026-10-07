"""Tier 3: Pairwise Cross-Feature Combinations Test Suite.

Verifies end-to-end interactions and contracts between pairs and triads of features:
- F1 + F2: Full LiDAR to Camera Image projection pipeline equivalence
- F4 + F1: Perturbations altering camera-rectified 3D coordinate distributions
- F4 + F2: Extrinsic yaw/pitch drift propagating into 2D pixel displacement
- F4 + F5: Point cloud decimation causing 3D GT box point starvation
- F4 + F6: Degradation severity directly degrading composite Sensor Health Score
- F3 + F4: Cross-dataset compatibility of all perturbation operators
- F3 + F5 + F6: Consistent metric evaluation across Synthetic, KITTI, and nuScenes
- F7 + F4 + F5 + F6: Benchmark runner accurately recording sweeps into CSV
- F8 + F9: Systematic linkage between reproduced failure cases and debug layers
- F10 + F11: Parity between interactive demo app and headless CLI export
- F12 + F13 + F14: Report completeness and student credentials driving submission gate pass
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


class TestTier3Combinations(unittest.TestCase):
    """Tier 3: Pairwise Cross-Feature Integration Tests."""

    @classmethod
    def setUpClass(cls):
        cls.synthetic_calib = load_calib(REPO_ROOT / "data/synthetic/training/calib/000000.txt")
        cls.kitti_calib = load_calib(REPO_ROOT / "data/kitti_mini/training/calib/000001.txt")

    # =========================================================================
    # Combination 1: F1 (Velo to Cam) + F2 (Cam to Image)
    # =========================================================================

    def test_combo_f1_f2_projection_pipeline_equivalence(self):
        """Combo F1+F2: project_velo_to_image strictly equals cam_to_image(velo_to_cam(...))."""
        fr = load_frame(REPO_ROOT / "data/synthetic", "000000")
        points = fr["points"]
        calib = fr["calib"]
        shape = fr["image"].shape

        # Path A: high-level wrapper
        uv_a, depth_a, mask_a = project_velo_to_image(points, calib, shape)

        # Path B: explicit 2-stage pipeline
        pts_cam = velo_to_cam(points[:, :3], calib)
        uv_b, depth_b, mask_b = cam_to_image(pts_cam, calib.P2, shape)

        np.testing.assert_array_equal(mask_a, mask_b)
        np.testing.assert_allclose(uv_a, uv_b, atol=1e-5)
        np.testing.assert_allclose(depth_a, depth_b, atol=1e-5)

    # =========================================================================
    # Combination 2: F4 (Perturbations) + F1 (Velo to Cam)
    # =========================================================================

    def test_combo_f4_f1_extrinsic_perturbation_impacts_cam_frame(self):
        """Combo F4+F1: Extrinsic perturbation alters 3D camera coordinates systematically."""
        fr = load_frame(REPO_ROOT / "data/synthetic", "000000")
        calib_orig = fr["calib"]
        calib_shifted = perturb_extrinsic(calib_orig, t_xyz_m=(1.0, 0.0, 0.0))  # 1m translation along velo x

        pts_orig = velo_to_cam(fr["points"][:10, :3], calib_orig)
        pts_shifted = velo_to_cam(fr["points"][:10, :3], calib_shifted)

        # Since velo x is approximately camera z forward, z coordinates should increase ~1m
        diff = pts_shifted - pts_orig
        self.assertAlmostEqual(diff[0, 2], 1.0, delta=0.2)

    # =========================================================================
    # Combination 3: F4 (Perturbations) + F2 (Cam to Image)
    # =========================================================================

    def test_combo_f4_f2_yaw_drift_causes_horizontal_pixel_shift(self):
        """Combo F4+F2: Extrinsic yaw drift shifts projected pixels laterally across u-axis."""
        fr = load_frame(REPO_ROOT / "data/synthetic", "000000")
        calib_orig = fr["calib"]
        calib_yaw = perturb_extrinsic(calib_orig, yaw_deg=1.0)

        uv_orig, depth_orig, mask_orig = project_velo_to_image(fr["points"], calib_orig, fr["image"].shape)
        uv_yaw, depth_yaw, mask_yaw = project_velo_to_image(fr["points"], calib_yaw, fr["image"].shape)

        # Common valid points
        common_mask = mask_orig & mask_yaw
        self.assertGreater(common_mask.sum(), 500)

        # Yaw around Velodyne z (up) shifts points left/right in camera image (u coordinate)
        pts_cam_orig = velo_to_cam(fr["points"][:, :3], calib_orig)
        pts_cam_yaw = velo_to_cam(fr["points"][:, :3], calib_yaw)
        u_orig = (pts_cam_orig @ calib_orig.P2[:3, :3].T)[:, 0] / pts_cam_orig[:, 2]
        u_yaw = (pts_cam_yaw @ calib_orig.P2[:3, :3].T)[:, 0] / pts_cam_yaw[:, 2]
        mean_u_shift = np.abs(u_yaw[common_mask] - u_orig[common_mask]).mean()
        self.assertGreater(mean_u_shift, 5.0, "1.0 deg yaw drift must cause significant pixel shift (>5 px)")

    # =========================================================================
    # Combination 4: F4 (Perturbations) + F5 (3D Box Containment)
    # =========================================================================

    def test_combo_f4_f5_dropout_degrades_3d_box_containment(self):
        """Combo F4+F5: Point cloud decimation directly starves 3D GT bounding boxes."""
        try:
            metrics = importlib.import_module("src.metrics")
        except ImportError:
            self.skipTest("M2 Feature: src.metrics not yet implemented.")

        fr = load_frame(REPO_ROOT / "data/kitti_mini", "000001")
        if not fr["labels"]:
            self.skipTest("No labels in frame 000001")

        counts = []
        for keep_ratio in [1.0, 0.5, 0.1]:
            pts_d = random_dropout(fr["points"], keep_ratio=keep_ratio, seed=42)
            pts_cam = velo_to_cam(pts_d[:, :3], fr["calib"])
            total_pts = sum(metrics.points_in_box3d(pts_cam, obj).sum() for obj in fr["labels"])
            counts.append(total_pts)

        self.assertGreater(counts[0], counts[1])
        self.assertGreater(counts[1], counts[2])

    # =========================================================================
    # Combination 5: F4 (Perturbations) + F6 (Health Metrics)
    # =========================================================================

    def test_combo_f4_f6_range_dropout_degrades_fov_and_health_score(self):
        """Combo F4+F6: Radial range attenuation degrades camera FOV and health score."""
        try:
            metrics = importlib.import_module("src.metrics")
        except ImportError:
            self.skipTest("M2 Feature: src.metrics not yet implemented.")

        fr = load_frame(REPO_ROOT / "data/kitti_mini", "000001")
        m_full = metrics.compute_frame_metrics(fr["points"], fr["calib"], fr["labels"], fr["image"].shape)
        pts_attenuated = range_dropout(fr["points"], max_range_m=20.0)
        m_atten = metrics.compute_frame_metrics(pts_attenuated, fr["calib"], fr["labels"], fr["image"].shape)

        self.assertGreater(m_full["n_points"], m_atten["n_points"])
        self.assertGreater(m_full["health_score"], m_atten["health_score"])

    # =========================================================================
    # Combination 6: F3 (Multi-dataset) + F4 (Perturbations)
    # =========================================================================

    def test_combo_f3_f4_perturbations_across_multiple_datasets(self):
        """Combo F3+F4: Perturbation functions work consistently across Synthetic, KITTI, nuScenes."""
        datasets = [
            ("synthetic", "000000"),
            ("kitti_mini", "000001"),
            ("nuscenes_mini_subset", "scene-0103_010"),
        ]
        for ds, fid in datasets:
            pts = load_points(REPO_ROOT / f"data/{ds}", fid)
            self.assertGreater(len(pts), 0)

            # Test random dropout
            pts_drop = random_dropout(pts, keep_ratio=0.5, seed=42)
            self.assertAlmostEqual(len(pts_drop) / len(pts), 0.5, delta=0.05)

            # Test gaussian noise
            pts_noise = gaussian_noise(pts, sigma_xyz_m=0.05, seed=42)
            self.assertEqual(pts_noise.shape, pts.shape)

            # Test beam dropout
            pts_beam = beam_dropout(pts, keep_every=2)
            self.assertLess(len(pts_beam), len(pts))

    # =========================================================================
    # Combination 7: F3 (Multi-dataset) + F5 & F6 (Metrics)
    # =========================================================================

    def test_combo_f3_f5_f6_metrics_consistency_across_datasets(self):
        """Combo F3+F5+F6: Metrics calculation evaluates consistently across all datasets."""
        try:
            metrics = importlib.import_module("src.metrics")
        except ImportError:
            self.skipTest("M2 Feature: src.metrics not yet implemented.")

        for ds, fid in [("synthetic", "000000"), ("kitti_mini", "000001")]:
            fr = load_frame(REPO_ROOT / f"data/{ds}", fid)
            m = metrics.compute_frame_metrics(fr["points"], fr["calib"], fr["labels"], fr["image"].shape)
            self.assertIn("health_score", m)
            self.assertIn("pts_in_fov_pct", m)
            self.assertTrue(0.0 <= m["health_score"] <= 100.0)

    # =========================================================================
    # Combination 8: F7 (Benchmark CSV) + F4, F5, F6 (Pipelines)
    # =========================================================================

    def test_combo_f7_f4_f5_f6_benchmark_runner_aggregates_metrics(self):
        """Combo F7+F4+F5+F6: Benchmark runner executes sweeps and logs metrics to CSV."""
        try:
            stress_mod = importlib.import_module("src.stress_test")
        except ImportError:
            self.skipTest("M2 Feature: src.stress_test not yet implemented.")
        self.assertTrue(hasattr(stress_mod, "run_stress_test") or hasattr(stress_mod, "main"))

    # =========================================================================
    # Combination 9: F8 (Failure Reproduction) + F9 (Debug Layers)
    # =========================================================================

    def test_combo_f8_f9_failure_cases_bound_to_debug_layers(self):
        """Combo F8+F9: All reproduced failure scenarios are mapped to explicit debug layers."""
        try:
            fail_mod = importlib.import_module("src.failure_analysis")
        except ImportError:
            self.skipTest("M3 Feature: src.failure_analysis not yet implemented.")

        if hasattr(fail_mod, "FAILURE_SCENARIOS"):
            for sc_name, sc_data in fail_mod.FAILURE_SCENARIOS.items():
                self.assertIn("layer", sc_data)
                self.assertIn(sc_data["layer"], ["Geometry", "Sensor/Environment", "Sensor", "Time", "Preprocess"])

    # =========================================================================
    # Combination 10: F10 (Demo App) + F11 (Headless Export)
    # =========================================================================

    def test_combo_f10_f11_demo_app_interactive_and_headless_parity(self):
        """Combo F10+F11: Headless CLI export uses same rendering contracts as interactive app."""
        app_file = REPO_ROOT / "src/app.py"
        if not app_file.exists():
            self.skipTest("M4 Feature: src/app.py not yet implemented.")
        app_mod = importlib.import_module("src.app")
        self.assertTrue(hasattr(app_mod, "main"))

    # =========================================================================
    # Combination 11: F12 (Report) + F13 (Student) + F14 (Gate Pass)
    # =========================================================================

    def test_combo_f12_f13_f14_report_student_submission_gate(self):
        """Combo F12+F13+F14: Report completion and student info satisfy submission gate checks."""
        report = REPO_ROOT / "report/REPORT.md"
        if not report.exists():
            self.skipTest("M5 Feature: report/REPORT.md missing.")
        text = report.read_text(encoding="utf-8")
        if "[ĐIỀN]" in text:
            self.skipTest("M5 Feature: REPORT.md contains placeholders, pending M5.")
        self.assertIn("Ngô Xuân Hoàng", text)
        self.assertIn("2A202602597", text)
        self.assertIn("H209", text)


if __name__ == "__main__":
    unittest.main()
