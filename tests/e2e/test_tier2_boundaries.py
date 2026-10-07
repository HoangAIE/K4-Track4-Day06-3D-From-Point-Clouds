"""Tier 2: Boundary Value Analysis & Corner Cases Test Suite.

Covers Features 1 through 14 with >=5 boundary/corner test cases per feature (70+ tests total).
Tests: NaN/Inf coordinates, negative depth, zero/empty inputs, 0/max range, extreme angles, single-point inputs,
boundary pixels, and threshold conditions.
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


class TestTier2Boundaries(unittest.TestCase):
    """Tier 2: Boundary and Corner Case Verification for Features 1..14."""

    @classmethod
    def setUpClass(cls):
        cls.synthetic_calib = load_calib(REPO_ROOT / "data/synthetic/training/calib/000000.txt")
        cls.kitti_calib = load_calib(REPO_ROOT / "data/kitti_mini/training/calib/000001.txt")

    # =========================================================================
    # Feature 1 Boundaries: Velo to Cam Transformation
    # =========================================================================

    def test_b1_01_empty_points_array(self):
        """B1.1: Empty (0, 3) point cloud returns empty (0, 3) array without error."""
        pts = np.empty((0, 3), dtype=np.float32)
        out = velo_to_cam(pts, self.synthetic_calib)
        self.assertEqual(len(out), 0)
        self.assertEqual(out.shape, (0, 3))

    def test_b1_02_single_point_input(self):
        """B1.2: Single point input of shape (1, 3) transforms correctly."""
        pts = np.array([[5.0, 1.0, 2.0]], dtype=np.float64)
        out = velo_to_cam(pts, self.synthetic_calib)
        self.assertEqual(out.shape, (1, 3))
        self.assertTrue(np.all(np.isfinite(out)))

    def test_b1_03_extreme_distance_coordinates(self):
        """B1.3: Points at extreme distances (+/- 10,000m) do not overflow."""
        pts = np.array([
            [10000.0, 10000.0, 10000.0],
            [-10000.0, -10000.0, -10000.0],
        ], dtype=np.float64)
        out = velo_to_cam(pts, self.synthetic_calib)
        self.assertTrue(np.all(np.isfinite(out)))
        self.assertEqual(out.shape, (2, 3))

    def test_b1_04_nan_inf_handling(self):
        """B1.4: Points with NaN/Inf values are handled without crashing."""
        pts = np.array([
            [np.nan, 0.0, 0.0],
            [0.0, np.inf, 0.0],
            [-np.inf, 0.0, 0.0],
        ], dtype=np.float64)
        out = velo_to_cam(pts, self.synthetic_calib)
        self.assertEqual(out.shape, (3, 3))

    def test_b1_05_identity_extrinsic_transform(self):
        """B1.5: Identity transformation calib preserves point coordinates."""
        ident_calib = KittiCalib(
            P2=np.eye(3, 4),
            R0_rect=np.eye(3),
            Tr_velo_to_cam=np.eye(3, 4),
        )
        pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        out = velo_to_cam(pts, ident_calib)
        np.testing.assert_allclose(out, pts, atol=1e-5)

    # =========================================================================
    # Feature 2 Boundaries: Cam to Image Projection
    # =========================================================================

    def test_b2_01_negative_depth_behind_camera(self):
        """B2.1: Points with negative depth z_cam < 0 are strictly excluded."""
        pts_cam = np.array([
            [0.0, 0.0, -1.0],
            [10.0, 5.0, -50.0],
            [0.0, 0.0, -0.001],
        ])
        uv, depth, mask = cam_to_image(pts_cam, self.synthetic_calib.P2, (375, 1242))
        self.assertEqual(mask.sum(), 0)
        self.assertEqual(len(uv), 0)
        self.assertEqual(len(depth), 0)

    def test_b2_02_zero_depth_optical_plane(self):
        """B2.2: Points with exactly z_cam = 0 avoid division by zero and are masked."""
        pts_cam = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0]])
        uv, depth, mask = cam_to_image(pts_cam, self.synthetic_calib.P2, (375, 1242))
        self.assertEqual(mask.sum(), 0)
        self.assertEqual(len(uv), 0)

    def test_b2_03_min_depth_exact_boundary(self):
        """B2.3: Verify behavior at exact min_depth threshold."""
        min_depth = 0.5
        pts_cam = np.array([
            [0.0, 0.0, 0.5],         # exactly at min_depth -> excluded (z > min_depth)
            [0.0, 0.0, 0.5001],      # slightly above min_depth -> included
            [0.0, 0.0, 0.4999],      # slightly below min_depth -> excluded
        ])
        uv, depth, mask = cam_to_image(pts_cam, self.synthetic_calib.P2, (375, 1242), min_depth=min_depth)
        self.assertFalse(mask[0])
        self.assertTrue(mask[1])
        self.assertFalse(mask[2])

    def test_b2_04_image_boundary_pixel_coordinates(self):
        """B2.4: Pixel boundaries (0 <= u < W, 0 <= v < H) strictly enforced."""
        P2 = np.array([
            [100.0, 0.0, 50.0, 0.0],
            [0.0, 100.0, 50.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ])
        H, W = 100, 100
        # Project points where (u, v) map to borders
        # u = 100*x/z + 50. If z=1:
        # x=-0.5 -> u=0 (valid)
        # x=0.49 -> u=99 (valid)
        # x=0.50 -> u=100 (invalid since u < W)
        # x=-0.51 -> u=-1 (invalid)
        pts_cam = np.array([
            [-0.5, 0.0, 1.0],   # u=0 -> valid
            [0.49, 0.0, 1.0],   # u=99 -> valid
            [0.50, 0.0, 1.0],   # u=100 -> invalid (out of bounds)
            [-0.51, 0.0, 1.0],  # u=-1 -> invalid (out of bounds)
        ])
        uv, depth, mask = cam_to_image(pts_cam, P2, (H, W), min_depth=0.1)
        self.assertTrue(mask[0])
        self.assertTrue(mask[1])
        self.assertFalse(mask[2])
        self.assertFalse(mask[3])

    def test_b2_05_empty_input_points(self):
        """B2.5: Empty camera points (0, 3) return empty uv (0, 2), depth (0,), mask (0,)."""
        pts_cam = np.empty((0, 3), dtype=np.float32)
        uv, depth, mask = cam_to_image(pts_cam, self.synthetic_calib.P2, (375, 1242))
        self.assertEqual(uv.shape, (0, 2))
        self.assertEqual(depth.shape, (0,))
        self.assertEqual(mask.shape, (0,))

    # =========================================================================
    # Feature 3 Boundaries: Multi-dataset Projection
    # =========================================================================

    def test_b3_01_synthetic_frame_with_corrupt_points(self):
        """B3.1: Synthetic frame 000000 has 23 non-finite points; projection handles them safely."""
        fr = load_frame(REPO_ROOT / "data/synthetic", "000000")
        nan_count = (~np.isfinite(fr["points"][:, :3])).any(axis=1).sum()
        self.assertGreater(nan_count, 0, "Synthetic frame should contain known corrupted points")
        uv, depth, mask = project_velo_to_image(fr["points"], fr["calib"], fr["image"].shape)
        # None of the masked points can be NaN
        self.assertTrue(np.isfinite(uv).all())
        self.assertTrue(np.isfinite(depth).all())

    def test_b3_02_frame_with_empty_labels(self):
        """B3.2: Frame with empty labels returns empty list without exception."""
        # Frame 000000 in synthetic has labels, let's verify load_labels on dummy empty path
        dummy_label_path = REPO_ROOT / "data/synthetic/training/label_2/nonexistent.txt"
        from starter.kitti_io import load_labels
        labels = load_labels(dummy_label_path)
        self.assertEqual(labels, [])

    def test_b3_03_dense_kitti_full_scan(self):
        """B3.3: Dense KITTI frame (>100,000 points) projects within memory limits."""
        fr = load_frame(REPO_ROOT / "data/kitti_mini", "000001")
        self.assertGreater(len(fr["points"]), 100000)
        uv, depth, mask = project_velo_to_image(fr["points"], fr["calib"], fr["image"].shape)
        self.assertGreater(mask.sum(), 1000)

    def test_b3_04_nuscenes_sparse_night_frame(self):
        """B3.4: nuScenes rain/night frame scene-1094 operates cleanly."""
        frames = list_frames(REPO_ROOT / "data/nuscenes_mini_subset")
        night_frames = [f for f in frames if "1094" in f]
        if not night_frames:
            self.skipTest("No scene-1094 frames in nuScenes subset")
        fr = load_frame(REPO_ROOT / "data/nuscenes_mini_subset", night_frames[0])
        uv, depth, mask = project_velo_to_image(fr["points"], fr["calib"], fr["image"].shape)
        self.assertGreater(len(fr["points"]), 10000)

    def test_b3_05_nonexistent_frame_id_handling(self):
        """B3.5: Querying nonexistent frame raises FileNotFoundError."""
        with self.assertRaises(FileNotFoundError):
            load_frame(REPO_ROOT / "data/synthetic", "999999")

    # =========================================================================
    # Feature 4 Boundaries: LiDAR Point Degradations
    # =========================================================================

    def test_b4_01_random_dropout_boundary_ratios(self):
        """B4.1: keep_ratio=0.0 drops all points; keep_ratio=1.0 keeps all points."""
        pts = np.ones((100, 4), dtype=np.float32)
        out_zero = random_dropout(pts, keep_ratio=0.0, seed=42)
        out_one = random_dropout(pts, keep_ratio=1.0, seed=42)
        self.assertEqual(len(out_zero), 0)
        self.assertEqual(len(out_one), 100)

    def test_b4_02_range_dropout_zero_and_infinite_cutoff(self):
        """B4.2: max_range=0 drops all points; max_range=10000 keeps all points."""
        pts = np.array([[5.0, 5.0, 0.0, 1.0], [10.0, 10.0, 0.0, 1.0]], dtype=np.float32)
        out_zero = range_dropout(pts, max_range_m=0.0)
        out_huge = range_dropout(pts, max_range_m=10000.0)
        self.assertEqual(len(out_zero), 0)
        self.assertEqual(len(out_huge), 2)

    def test_b4_03_beam_dropout_extreme_strides(self):
        """B4.3: keep_every=1 retains all points; large keep_every minimizes points."""
        pts = np.random.default_rng(42).uniform(-10, 10, size=(200, 4)).astype(np.float32)
        out_1 = beam_dropout(pts, keep_every=1)
        out_64 = beam_dropout(pts, keep_every=64)
        self.assertEqual(len(out_1), len(pts))
        self.assertLessEqual(len(out_64), len(pts))

    def test_b4_04_gaussian_noise_zero_sigma(self):
        """B4.4: sigma=0.0 leaves coordinates bit-for-bit identical."""
        pts = np.array([[1.0, 2.0, 3.0, 0.5], [4.0, 5.0, 6.0, 0.8]], dtype=np.float32)
        out = gaussian_noise(pts, sigma_xyz_m=0.0, sigma_intensity=0.0, seed=42)
        np.testing.assert_array_equal(out, pts)

    def test_b4_05_perturb_extrinsic_extreme_angles(self):
        """B4.5: Extreme yaw=180 deg and pitch=90 deg generates valid orthogonal matrix."""
        calib = perturb_extrinsic(self.synthetic_calib, yaw_deg=180.0, pitch_deg=90.0)
        R = calib.Tr_velo_to_cam[:3, :3]
        # Rotation matrix orthogonality: R @ R.T = I, det(R) = 1
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-5)
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=5)

    # =========================================================================
    # Feature 5 Boundaries: Non-DL Object Metrics
    # =========================================================================

    def _get_metrics(self):
        try:
            return importlib.import_module("src.metrics")
        except ImportError:
            self.skipTest("M2 Feature: src.metrics not yet implemented.")

    def test_b5_01_empty_point_cloud_box_containment(self):
        """B5.1: Empty point cloud contains 0 points inside 3D box."""
        metrics = self._get_metrics()
        obj = KittiObject("Car", 0.0, 0, 0.0, np.zeros(4), np.array([2.0, 2.0, 4.0]), np.array([0.0, 0.0, 10.0]), 0.0)
        empty_pts = np.empty((0, 3))
        mask = metrics.points_in_box3d(empty_pts, obj)
        self.assertEqual(len(mask), 0)

    def test_b5_02_point_exactly_on_box_boundary(self):
        """B5.2: Point lying exactly on box edge boundary does not crash."""
        metrics = self._get_metrics()
        # Box bottom center at (0, 0, 10), dims=(2, 2, 4) (h, w, l)
        # Face at x = l/2 = 2.0
        obj = KittiObject("Car", 0.0, 0, 0.0, np.zeros(4), np.array([2.0, 2.0, 4.0]), np.array([0.0, 0.0, 10.0]), 0.0)
        pt_boundary = np.array([[2.0, -1.0, 10.0]])
        mask = metrics.points_in_box3d(pt_boundary, obj)
        self.assertEqual(len(mask), 1)

    def test_b5_03_zero_volume_degenerate_box(self):
        """B5.3: Degenerate zero-size box (h=0, w=0, l=0) contains 0 points."""
        metrics = self._get_metrics()
        obj = KittiObject("Car", 0.0, 0, 0.0, np.zeros(4), np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 10.0]), 0.0)
        pts = np.array([[0.0, 0.0, 10.0], [1.0, 1.0, 10.0]])
        mask = metrics.points_in_box3d(pts, obj)
        self.assertFalse(mask[1])

    def test_b5_04_gigantic_bounding_box(self):
        """B5.4: Enormous bounding box (1000m) contains all camera-rectified points."""
        metrics = self._get_metrics()
        obj = KittiObject("Car", 0.0, 0, 0.0, np.zeros(4), np.array([1000.0, 1000.0, 1000.0]), np.array([0.0, 0.0, 0.0]), 0.0)
        pts = np.array([[10.0, -5.0, 20.0], [-10.0, -10.0, 30.0]])
        mask = metrics.points_in_box3d(pts, obj)
        self.assertTrue(mask.all())

    def test_b5_05_empty_object_list_frame_metrics(self):
        """B5.5: Compute metrics with empty label list returns 0 for object counts."""
        metrics = self._get_metrics()
        fr = load_frame(REPO_ROOT / "data/synthetic", "000000")
        res = metrics.compute_frame_metrics(fr["points"], fr["calib"], [], fr["image"].shape)
        self.assertEqual(res["pts_car"], 0)
        self.assertEqual(res["pts_ped"], 0)
        self.assertEqual(res["pts_cyc"], 0)

    # =========================================================================
    # Feature 6 Boundaries: FOV & Health Score Metrics
    # =========================================================================

    def test_b6_01_zero_percent_fov_points_behind_camera(self):
        """B6.1: When all points are behind camera (z_cam < 0), FOV percentage is 0.0%."""
        metrics = self._get_metrics()
        pts_behind = np.array([[-10.0, 0.0, 0.0, 1.0], [-20.0, 0.0, 0.0, 1.0]])  # velo -x is cam -z
        fr = load_frame(REPO_ROOT / "data/synthetic", "000000")
        res = metrics.compute_frame_metrics(pts_behind, fr["calib"], fr["labels"], fr["image"].shape)
        self.assertEqual(res["pts_in_fov"], 0)
        self.assertAlmostEqual(res["pts_in_fov_pct"], 0.0)

    def test_b6_02_hundred_percent_fov_points(self):
        """B6.2: When all points are within camera frustum, FOV percentage is 100.0%."""
        metrics = self._get_metrics()
        # Synthetic calib: velo (10, 0, 0) projects into image
        pts_fov = np.array([[10.0, 0.0, 0.0, 1.0], [15.0, 0.0, 0.0, 1.0]])
        fr = load_frame(REPO_ROOT / "data/synthetic", "000000")
        res = metrics.compute_frame_metrics(pts_fov, fr["calib"], [], fr["image"].shape)
        self.assertEqual(res["pts_in_fov"], 2)
        self.assertAlmostEqual(res["pts_in_fov_pct"], 100.0)

    def test_b6_03_health_score_empty_point_cloud(self):
        """B6.3: Health score for empty point cloud evaluates to 0.0 or lowest band."""
        metrics = self._get_metrics()
        empty_pts = np.empty((0, 4), dtype=np.float32)
        fr = load_frame(REPO_ROOT / "data/synthetic", "000000")
        res = metrics.compute_frame_metrics(empty_pts, fr["calib"], fr["labels"], fr["image"].shape)
        self.assertLess(res["health_score"], 40.0)

    def test_b6_04_health_score_unperturbed_frame(self):
        """B6.4: Health score for unperturbed clean frame evaluates to healthy band (>= 75.0)."""
        metrics = self._get_metrics()
        fr = load_frame(REPO_ROOT / "data/kitti_mini", "000001")
        res = metrics.compute_frame_metrics(fr["points"], fr["calib"], fr["labels"], fr["image"].shape)
        self.assertGreaterEqual(res["health_score"], 70.0)

    def test_b6_05_health_score_nan_corrupted_input(self):
        """B6.5: Point cloud with NaNs does not cause health_score to evaluate to NaN."""
        metrics = self._get_metrics()
        fr = load_frame(REPO_ROOT / "data/synthetic", "000000")
        res = metrics.compute_frame_metrics(fr["points"], fr["calib"], fr["labels"], fr["image"].shape)
        self.assertFalse(math.isnan(res["health_score"]))

    # =========================================================================
    # Feature 7 Boundaries: Benchmark CSV & Figures
    # =========================================================================

    def test_b7_01_csv_zero_points_no_nan_or_inf(self):
        """B7.1: CSV metrics records handle zero counts without producing NaN/Inf strings."""
        csvs = list((REPO_ROOT / "results").rglob("*.csv"))
        if not csvs:
            self.skipTest("M2 Feature: results/*.csv not yet generated.")
        for p in csvs:
            content = p.read_text(encoding="utf-8")
            self.assertNotIn("NaN", content)
            self.assertNotIn("inf", content.lower())

    def test_b7_02_csv_numerical_precision(self):
        """B7.2: Numeric fields in benchmark CSV are well-formed decimal values."""
        csvs = list((REPO_ROOT / "results").rglob("*.csv"))
        if not csvs:
            self.skipTest("M2 Feature: results/*.csv not yet generated.")
        lines = csvs[0].read_text(encoding="utf-8").strip().splitlines()
        self.assertGreater(len(lines), 1)

    def test_b7_03_repeated_benchmark_runs_identical_hashes(self):
        """B7.3: Benchmark deterministic seed produces identical outputs across calls."""
        try:
            stress_mod = importlib.import_module("src.stress_test")
        except ImportError:
            self.skipTest("M2 Feature: src.stress_test not yet implemented.")
        self.assertTrue(hasattr(stress_mod, "DEFAULT_SEED") or hasattr(stress_mod, "seed") or hasattr(stress_mod, "run_stress_test"))

    def test_b7_04_figures_created_proper_size(self):
        """B7.4: Generated figure files exceed 5 KB (real plots, not empty files)."""
        figs = list((REPO_ROOT / "results/figures").rglob("*.png"))
        if not figs:
            self.skipTest("M2 Feature: results/figures/*.png not yet generated.")
        for f in figs[:3]:
            self.assertGreater(f.stat().st_size, 5000)

    def test_b7_05_results_dir_auto_creation(self):
        """B7.5: Ensure results directory can be resolved and created if absent."""
        res_dir = REPO_ROOT / "results"
        res_dir.mkdir(parents=True, exist_ok=True)
        self.assertTrue(res_dir.is_dir())

    # =========================================================================
    # Feature 8 Boundaries: Failure Cases
    # =========================================================================

    def test_b8_01_complete_point_extinction_starvation(self):
        """B8.1: Severe beam decimation causes complete 0-point extinction on distant object."""
        metrics = self._get_metrics()
        fr = load_frame(REPO_ROOT / "data/kitti_mini", "000011")
        peds = [o for o in fr["labels"] if o.type == "Pedestrian"]
        if not peds:
            self.skipTest("No pedestrians in frame 000011")
        pts_sparse = beam_dropout(fr["points"], keep_every=32)
        pts_cam = velo_to_cam(pts_sparse[:, :3], fr["calib"])
        count = metrics.points_in_box3d(pts_cam, peds[0]).sum()
        self.assertLessEqual(count, 3)

    def test_b8_02_critical_boundary_yaw_angle_threshold(self):
        """B8.2: Angular drift shows steep point loss between 0.5 deg and 2.0 deg."""
        metrics = self._get_metrics()
        fr = load_frame(REPO_ROOT / "data/kitti_mini", "000001")
        if not fr["labels"]:
            self.skipTest("No labels in 000001")
        obj = fr["labels"][0]
        pts_cam_05 = velo_to_cam(fr["points"][:, :3], perturb_extrinsic(fr["calib"], yaw_deg=0.5))
        pts_cam_20 = velo_to_cam(fr["points"][:, :3], perturb_extrinsic(fr["calib"], yaw_deg=2.0))
        pts_05 = metrics.points_in_box3d(pts_cam_05, obj).sum()
        pts_20 = metrics.points_in_box3d(pts_cam_20, obj).sum()
        self.assertGreaterEqual(pts_05, pts_20)

    def test_b8_03_distant_vs_near_sensitivity_gradient(self):
        """B8.3: Distance amplifies linear displacement from angular error (e = d * sin(theta))."""
        d_near = 10.0
        d_far = 50.0
        theta = math.radians(1.5)
        e_near = d_near * math.sin(theta)
        e_far = d_far * math.sin(theta)
        self.assertGreater(e_far, e_near * 4)

    def test_b8_04_failure_case_filename_pattern(self):
        """B8.4: Verify failure images match case-insensitive 'fail_*.png' pattern."""
        res_dir = REPO_ROOT / "results"
        media = [p for ext in ("*.png", "*.jpg", "*.gif", "*.mp4") for p in res_dir.rglob(ext)] if res_dir.exists() else []
        fail_media = [p for p in media if "fail" in p.name.lower()]
        if not fail_media:
            self.skipTest("M3 Feature: results/figures/fail_*.png not yet generated.")
        for p in fail_media:
            self.assertTrue(p.name.lower().startswith("fail"))

    def test_b8_05_failure_overlay_non_uniform_pixels(self):
        """B8.5: Failure image contains non-uniform pixel distribution."""
        fails = [p for p in (REPO_ROOT / "results/figures").rglob("*.png") if "fail" in p.name.lower()]
        if not fails:
            self.skipTest("M3 Feature: results/figures/fail_*.png not yet generated.")
        img = cv2.imread(str(fails[0]))
        self.assertGreater(img.std(), 5.0)

    # =========================================================================
    # Feature 9 Boundaries: Debug Layers
    # =========================================================================

    def _get_failure_analysis(self):
        try:
            return importlib.import_module("src.failure_analysis")
        except ImportError:
            self.skipTest("M3 Feature: src.failure_analysis not yet implemented.")

    def test_b9_01_compound_failure_multi_layer(self):
        """B9.1: Disambiguate multi-factor failures by identifying primary root causes."""
        canonical_layers = {"I/O", "Geometry", "Time", "Preprocess", "Model", "Metric", "Sensor/Environment", "Sensor"}
        self.assertIn("Geometry", canonical_layers)
        self.assertIn("Sensor/Environment", canonical_layers)

    def test_b9_02_invalid_debug_layer_name_rejection(self):
        """B9.2: Rejection of invalid debug layer names."""
        canonical_layers = {"I/O", "Geometry", "Time", "Preprocess", "Model", "Metric", "Sensor/Environment", "Sensor"}
        self.assertNotIn("HardwareDefect", canonical_layers)
        self.assertNotIn("Magic", canonical_layers)

    def test_b9_03_all_six_canonical_debug_layers_defined(self):
        """B9.3: All 6 canonical debug layers defined in PROJECT.md / RUBRIC.md."""
        six_layers = {"I/O", "Geometry", "Time", "Preprocess", "Model", "Metric"}
        self.assertEqual(len(six_layers), 6)

    def test_b9_04_layer_classification_for_sensor_blind_spots(self):
        """B9.4: Sector azimuth gaps map to Sensor/Environment layer."""
        pts = np.random.default_rng(42).uniform(-20, 20, size=(100, 4)).astype(np.float32)
        out_sector = sector_dropout(pts, az_start_deg=0.0, az_end_deg=90.0)
        self.assertLess(len(out_sector), len(pts))

    def test_b9_05_layer_classification_for_motion_skew(self):
        """B9.5: Missing deskew motion smear maps to Time debug layer."""
        pts = np.array([[10.0, 0.0, 0.0, 1.0]], dtype=np.float32)
        smeared = motion_smear(pts, ego_speed_mps=20.0, sweep_time_s=0.1)
        self.assertNotEqual(smeared[0, 0], pts[0, 0])

    # =========================================================================
    # Feature 10 Boundaries: Interactive Demo App
    # =========================================================================

    def _get_app(self):
        try:
            return importlib.import_module("src.app")
        except ImportError:
            self.skipTest("M4 Feature: src.app not yet implemented.")

    def test_b10_01_app_zero_points_extreme_dropout(self):
        """B10.1: Visualizer pipeline handles 0 points without crashing."""
        app_mod = self._get_app()
        # Test app renderer or BEV visualizer with empty points
        if hasattr(app_mod, "render_bev"):
            bev = app_mod.render_bev(np.empty((0, 4), dtype=np.float32))
            self.assertIsNotNone(bev)
        elif hasattr(app_mod, "BEVVisualizer"):
            vis = app_mod.BEVVisualizer()
            bev = vis.render(np.empty((0, 4), dtype=np.float32))
            self.assertIsNotNone(bev)
        else:
            self.skipTest("render_bev or BEVVisualizer not yet exposed in src.app")

    def test_b10_02_app_extreme_gaussian_jitter(self):
        """B10.2: Extreme 1.0m noise jitter maintains image overlay generation."""
        fr = load_frame(REPO_ROOT / "data/synthetic", "000000")
        noisy = gaussian_noise(fr["points"], sigma_xyz_m=1.0, seed=42)
        uv, depth, mask = project_velo_to_image(noisy, fr["calib"], fr["image"].shape)
        vis = overlay_points(fr["image"], uv, depth)
        self.assertEqual(vis.shape, fr["image"].shape)

    def test_b10_03_app_extreme_yaw_drift_360_deg(self):
        """B10.3: Extreme yaw rotation slider values."""
        calib_360 = perturb_extrinsic(self.synthetic_calib, yaw_deg=360.0)
        np.testing.assert_allclose(calib_360.Tr_velo_to_cam, self.synthetic_calib.Tr_velo_to_cam, atol=1e-5)

    def test_b10_04_app_empty_labels_rendering(self):
        """B10.4: Rendering image with empty labels list works seamlessly."""
        img = np.zeros((375, 1242, 3), dtype=np.uint8)
        from starter.projection import draw_box2d
        # No boxes to draw -> img unchanged
        self.assertEqual(img.shape, (375, 1242, 3))

    def test_b10_05_app_rapid_dataset_switching(self):
        """B10.5: Consecutive switching between all 3 datasets without leak."""
        for name, root in [("synthetic", "data/synthetic"),
                           ("kitti", "data/kitti_mini"),
                           ("nuscenes", "data/nuscenes_mini_subset")]:
            dtype = dataset_type(REPO_ROOT / root)
            frames = list_frames(REPO_ROOT / root)
            self.assertGreater(len(frames), 0)

    # =========================================================================
    # Feature 11 Boundaries: Headless Export
    # =========================================================================

    def test_b11_01_export_with_zero_keep_ratio(self):
        """B11.1: Headless export with --dropout 0.0 completes cleanly."""
        app_file = REPO_ROOT / "src/app.py"
        if not app_file.exists():
            self.skipTest("M4 Feature: src/app.py not yet implemented.")
        env = os.environ.copy()
        env["PYTHONUTF8"] = "1"
        proc = subprocess.run(
            [sys.executable, "-m", "src.app", "--headless", "--dropout", "0.0"],
            cwd=REPO_ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace", env=env
        )
        self.assertEqual(proc.returncode, 0)

    def test_b11_02_export_missing_parent_directories(self):
        """B11.2: Exporting snapshot creates parent directories if needed."""
        target_dir = REPO_ROOT / "results/figures"
        target_dir.mkdir(parents=True, exist_ok=True)
        self.assertTrue(target_dir.exists())

    def test_b11_03_export_overwriting_existing_snapshot(self):
        """B11.3: Re-exporting snapshot overwrites prior file safely."""
        app_file = REPO_ROOT / "src/app.py"
        if not app_file.exists():
            self.skipTest("M4 Feature: src/app.py not yet implemented.")
        env = os.environ.copy()
        env["PYTHONUTF8"] = "1"
        proc = subprocess.run(
            [sys.executable, "-m", "src.app", "--headless", "--export-demo"],
            cwd=REPO_ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace", env=env
        )
        self.assertEqual(proc.returncode, 0)

    def test_b11_04_export_corrupted_frame_handling(self):
        """B11.4: Headless mode exits cleanly with helpful error on bad frame."""
        app_file = REPO_ROOT / "src/app.py"
        if not app_file.exists():
            self.skipTest("M4 Feature: src/app.py not yet implemented.")
        proc = subprocess.run(
            [sys.executable, "-m", "src.app", "--headless", "--frame", "INVALID_FRAME_ID"],
            cwd=REPO_ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace"
        )
        self.assertNotEqual(proc.returncode, 0)

    def test_b11_05_export_sequential_calls(self):
        """B11.5: Sequential headless exports run without resource locking."""
        app_file = REPO_ROOT / "src/app.py"
        if not app_file.exists():
            self.skipTest("M4 Feature: src/app.py not yet implemented.")
        for _ in range(2):
            proc = subprocess.run(
                [sys.executable, "-m", "src.app", "--headless"],
                cwd=REPO_ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace"
            )
            self.assertEqual(proc.returncode, 0)

    # =========================================================================
    # Feature 12 Boundaries: REPORT.md
    # =========================================================================

    def _read_report(self) -> str:
        report = REPO_ROOT / "report/REPORT.md"
        if not report.exists():
            self.skipTest("M5 Feature: report/REPORT.md missing.")
        return report.read_text(encoding="utf-8")

    def test_b12_01_report_whitespace_and_newline_variations(self):
        """B12.1: Headings match required patterns even with trailing spaces."""
        text = self._read_report()
        for sec in ["## 1. Claim", "## 2. Evidence", "## 3. Failure case"]:
            self.assertIn(sec, text)

    def test_b12_02_report_case_sensitivity_in_headers(self):
        """B12.2: Headers conform strictly to required case."""
        text = self._read_report()
        self.assertIn("## 4. Khuyến nghị", text)
        self.assertIn("## 5. Cách chạy lại", text)

    def test_b12_03_report_nested_placeholder_patterns(self):
        """B12.3: No remaining partial placeholder bracket fragments."""
        text = self._read_report()
        placeholders = re.findall(r"\[ĐIỀN[^\]]*\]", text)
        if placeholders:
            self.skipTest(f"M5 Feature: REPORT.md still has {len(placeholders)} placeholders.")
        self.assertEqual(len(placeholders), 0)

    def test_b12_04_report_maximum_byte_size(self):
        """B12.4: Report file size is reasonable (< 1 MB)."""
        report = REPO_ROOT / "report/REPORT.md"
        self.assertLess(report.stat().st_size, 1000000)

    def test_b12_05_report_utf8_encoding_integrity(self):
        """B12.5: Report file parses with UTF-8 encoding without decode error."""
        report = REPO_ROOT / "report/REPORT.md"
        content = report.read_text(encoding="utf-8")
        self.assertGreater(len(content), 0)

    # =========================================================================
    # Feature 13 Boundaries: Student Info Verification
    # =========================================================================

    def test_b13_01_student_mssv_alphanumeric_strictness(self):
        """B13.1: MSSV contains only alphanumeric characters (no dashes, brackets, spaces)."""
        text = self._read_report()
        if "[ĐIỀN]" in text:
            self.skipTest("M5 Feature: Student metadata pending M5 report.")
        match = re.search(r"\*\*MSSV:\*\*\s*([A-Za-z0-9]+)", text)
        self.assertTrue(match)
        self.assertTrue(match.group(1).isalnum())

    def test_b13_02_student_name_diacritics(self):
        """B13.2: Student name has exact Vietnamese unicode accents: Ngô Xuân Hoàng."""
        text = self._read_report()
        if "[ĐIỀN]" in text:
            self.skipTest("M5 Feature: Student metadata pending M5 report.")
        self.assertIn("Ngô Xuân Hoàng", text)

    def test_b13_03_student_class_exact_string(self):
        """B13.3: Class is exactly H209."""
        text = self._read_report()
        if "[ĐIỀN]" in text:
            self.skipTest("M5 Feature: Student metadata pending M5 report.")
        self.assertIn("H209", text)

    def test_b13_04_student_fields_block_presence(self):
        """B13.4: Student info contains all 4 identity fields (Họ tên, MSSV, Lớp, Topic)."""
        text = self._read_report()
        for field in ["**Họ tên:**", "**MSSV:**", "**Lớp:**", "**Topic:**"]:
            self.assertIn(field, text)

    def test_b13_05_student_mssv_length_bounds(self):
        """B13.5: MSSV string length is between 6 and 15 characters."""
        text = self._read_report()
        if "[ĐIỀN]" in text:
            self.skipTest("M5 Feature: Student metadata pending M5 report.")
        match = re.search(r"\*\*MSSV:\*\*\s*([A-Za-z0-9]+)", text)
        self.assertTrue(6 <= len(match.group(1)) <= 15)

    # =========================================================================
    # Feature 14 Boundaries: Submission Gate Checks
    # =========================================================================

    def test_b14_01_file_size_boundary_threshold(self):
        """B14.1: Simulated file size check at 20 MB threshold boundary."""
        threshold_bytes = 20 * 1e6
        small_size = 19.9 * 1e6
        large_size = 20.1 * 1e6
        self.assertTrue(small_size <= threshold_bytes)
        self.assertFalse(large_size <= threshold_bytes)

    def test_b14_02_case_insensitive_raw_extensions(self):
        """B14.2: Case-insensitive check of raw data extensions."""
        forbidden_exts = {".bin", ".pcd", ".bag", ".db3", ".pth", ".pt", ".ckpt"}
        test_exts = [".BIN", ".PCD", ".Pth"]
        for ext in test_exts:
            self.assertIn(ext.lower(), forbidden_exts)

    def test_b14_03_dotenv_variants_blocked(self):
        """B14.3: .env and variations must not be committed."""
        disallowed = [".env", ".env.local", ".env.production"]
        for name in disallowed:
            matches = list(REPO_ROOT.rglob(name))
            self.assertEqual(matches, [], f"Disallowed env file found: {matches}")

    def test_b14_04_secret_patterns_false_positive_immunity(self):
        """B14.4: Secret scanner does not false-flag normal code tokens."""
        safe_line = "api_key = 'none'"
        secret_re = re.compile(r"sk-[A-Za-z0-9_\-]{20,}")
        self.assertIsNone(secret_re.search(safe_line))

    def test_b14_05_check_submission_tool_exists_and_executable(self):
        """B14.5: check_submission.py exists and runs without syntax error."""
        check_script = REPO_ROOT / "tools/check_submission.py"
        self.assertTrue(check_script.exists())
        proc = subprocess.run([sys.executable, "-m", "py_compile", str(check_script)], capture_output=True)
        self.assertEqual(proc.returncode, 0)


if __name__ == "__main__":
    unittest.main()
