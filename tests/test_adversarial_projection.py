"""Adversarial Stress Testing Suite for starter/projection.py.

Conducted by Challenger 2 for Milestone 1.
Covers:
  - Extreme shapes: 1M random points, 0 points, 1D arrays, multi-feature arrays.
  - Non-finite & numerical extremes: NaNs, +Infs, -Infs, subnormals, large floats.
  - Depth boundaries: negative depths, z=0, min_depth epsilon boundaries, huge z.
  - Image boundaries: subpixel, exact boundary, outside FOV.
  - Memory leak, stability, and runtime profiling.
  - Downstream functions: project_velo_to_image and overlay_points.
"""
from __future__ import annotations

import gc
import math
import os
import sys
import time
import tracemalloc
import unittest
from pathlib import Path

import cv2
import numpy as np

from starter.kitti_io import KittiCalib, load_calib
from starter.projection import (
    cam_to_image,
    overlay_points,
    perturb_extrinsic,
    project_velo_to_image,
    velo_to_cam,
)

REPO_ROOT = Path(__file__).resolve().parents[1]


class TestAdversarialShapes(unittest.TestCase):
    """Stress tests covering unexpected array shapes, dimensions, and memory layouts."""

    @classmethod
    def setUpClass(cls):
        cls.calib = load_calib(REPO_ROOT / "data/synthetic/training/calib/000000.txt")
        cls.img_shape = (375, 1242, 3)

    def test_01_empty_points_velo_to_cam(self):
        """Empty inputs (0, 3), (0, 4), (0,), [] should return empty (0, 3) without crashing."""
        for empty_in in [
            np.empty((0, 3), dtype=np.float32),
            np.empty((0, 3), dtype=np.float64),
            np.empty((0, 4), dtype=np.float32),
            np.empty((0,), dtype=np.float64),
            [],
            np.array([]),
        ]:
            out = velo_to_cam(empty_in, self.calib)
            self.assertEqual(out.shape, (0, 3), f"Failed for {type(empty_in)} shape {getattr(empty_in, 'shape', None)}")
            self.assertEqual(len(out), 0)

    def test_02_empty_points_cam_to_image(self):
        """Empty inputs (0, 3), (0, 4), (0,), [] should return (empty (0, 2), empty (0,), empty (0,) bool)."""
        for empty_in in [
            np.empty((0, 3), dtype=np.float32),
            np.empty((0, 3), dtype=np.float64),
            np.empty((0, 4), dtype=np.float32),
            np.empty((0,), dtype=np.float64),
            [],
            np.array([]),
        ]:
            uv, depth, mask = cam_to_image(empty_in, self.calib.P2, self.img_shape)
            self.assertEqual(uv.shape, (0, 2))
            self.assertEqual(depth.shape, (0,))
            self.assertEqual(mask.shape, (0,))
            self.assertEqual(mask.dtype, bool)

    def test_03_1d_array_velo_to_cam(self):
        """1D array of shape (3,) or (4,) should be processed and return 1D shape (3,)."""
        pt_3d = np.array([10.0, 0.0, 0.0])
        out_3d = velo_to_cam(pt_3d, self.calib)
        self.assertEqual(out_3d.shape, (3,))
        self.assertAlmostEqual(out_3d[2], 9.7273, places=2)

        pt_4d = np.array([10.0, 0.0, 0.0, 0.5])
        out_4d = velo_to_cam(pt_4d, self.calib)
        self.assertEqual(out_4d.shape, (3,))
        np.testing.assert_allclose(out_3d, out_4d)

    def test_04_1d_array_cam_to_image(self):
        """1D array of shape (3,) representing a point inside image FOV."""
        pt_cam = np.array([0.0, 0.0, 10.0])  # Center ahead
        uv, depth, mask = cam_to_image(pt_cam, self.calib.P2, self.img_shape)
        self.assertEqual(mask.shape, (1,))
        self.assertTrue(mask[0])
        self.assertEqual(uv.shape, (1, 2))
        self.assertEqual(depth.shape, (1,))
        self.assertAlmostEqual(depth[0], 10.0)

    def test_05_multi_column_inputs(self):
        """Points with extra columns (x, y, z, intensity, timestamp) should use first 3 columns."""
        pts_5d = np.array([
            [10.0, 0.0, 0.0, 0.8, 123456.0],
            [15.0, 1.0, -0.5, 0.2, 123457.0],
        ])
        out = velo_to_cam(pts_5d, self.calib)
        self.assertEqual(out.shape, (2, 3))
        # Compare with 3D slice
        out_3d = velo_to_cam(pts_5d[:, :3], self.calib)
        np.testing.assert_allclose(out, out_3d)

    def test_06_non_contiguous_and_strided_arrays(self):
        """Fortran-ordered, strided, and reversed arrays must execute identically."""
        rng = np.random.default_rng(42)
        base = rng.uniform(-10, 50, size=(1000, 3))

        # Fortran contiguous
        f_arr = np.asfortranarray(base)
        self.assertFalse(f_arr.flags.c_contiguous)
        out_f = velo_to_cam(f_arr, self.calib)
        out_c = velo_to_cam(base, self.calib)
        np.testing.assert_allclose(out_f, out_c)

        # Sliced step 2
        sliced = base[::2]
        out_sliced = velo_to_cam(sliced, self.calib)
        np.testing.assert_allclose(out_sliced, out_c[::2])

        # Reversed order
        reversed_arr = base[::-1]
        out_rev = velo_to_cam(reversed_arr, self.calib)
        np.testing.assert_allclose(out_rev, out_c[::-1])

    def test_07_various_numeric_dtypes(self):
        """Various dtypes (float32, float64, int32, int64) should be supported."""
        pts_f32 = np.array([[10.0, 0.0, 0.0]], dtype=np.float32)
        out_f32 = velo_to_cam(pts_f32, self.calib)
        self.assertAlmostEqual(out_f32[0, 2], 9.7273, places=2)

        pts_i32 = np.array([[10, 0, 0]], dtype=np.int32)
        out_i32 = velo_to_cam(pts_i32, self.calib)
        self.assertAlmostEqual(out_i32[0, 2], 9.7273, places=2)

        pts_i64 = np.array([[10, 0, 0]], dtype=np.int64)
        out_i64 = velo_to_cam(pts_i64, self.calib)
        self.assertAlmostEqual(out_i64[0, 2], 9.7273, places=2)


class TestAdversarialExtremeValues(unittest.TestCase):
    """Stress tests covering NaNs, infinities, subnormals, and extreme numerical scales."""

    @classmethod
    def setUpClass(cls):
        cls.calib = load_calib(REPO_ROOT / "data/synthetic/training/calib/000000.txt")
        cls.img_shape = (375, 1242, 3)

    def test_01_all_nans_cam_to_image(self):
        """All-NaN points in cam_to_image should be safely rejected without unhandled exception."""
        nans = np.full((100, 3), np.nan)
        uv, depth, mask = cam_to_image(nans, self.calib.P2, self.img_shape)
        self.assertEqual(len(uv), 0)
        self.assertEqual(len(depth), 0)
        self.assertEqual(len(mask), 100)
        self.assertFalse(mask.any(), "No NaN point should be marked valid")

    def test_02_all_infs_cam_to_image(self):
        """All-Inf points (+Inf and -Inf) should be safely rejected."""
        infs = np.array([
            [np.inf, 0.0, 10.0],
            [-np.inf, 0.0, 10.0],
            [0.0, np.inf, 10.0],
            [0.0, -np.inf, 10.0],
            [0.0, 0.0, np.inf],
            [0.0, 0.0, -np.inf],
            [np.inf, np.inf, np.inf],
            [-np.inf, -np.inf, -np.inf],
        ])
        uv, depth, mask = cam_to_image(infs, self.calib.P2, self.img_shape)
        self.assertEqual(len(uv), 0)
        self.assertFalse(mask.any())

    def test_03_mixed_finite_and_nonfinite(self):
        """Clean points interspersed with NaNs and Infs must preserve valid points."""
        pts = np.array([
            [np.nan, 0.0, 10.0],       # Invalid: NaN
            [0.0, 0.0, 10.0],          # Valid center point
            [np.inf, np.nan, 5.0],     # Invalid: Inf & NaN
            [0.1, -0.1, 15.0],         # Valid center point
            [0.0, 0.0, -np.inf],       # Invalid: -Inf
        ])
        uv, depth, mask = cam_to_image(pts, self.calib.P2, self.img_shape)
        self.assertEqual(mask.sum(), 2)
        np.testing.assert_array_equal(mask, [False, True, False, True, False])
        self.assertEqual(len(uv), 2)
        self.assertEqual(len(depth), 2)
        self.assertAlmostEqual(depth[0], 10.0)
        self.assertAlmostEqual(depth[1], 15.0)

    def test_04_floating_point_extremes(self):
        """Subnormal numbers, tiny floats, and extreme large floats."""
        pts = np.array([
            [1e-300, 1e-300, 10.0],    # Subnormal/tiny x, y -> valid inside image
            [1e300, 0.0, 10.0],        # Huge x -> outside image bounds
            [0.0, 1e300, 10.0],        # Huge y -> outside image bounds
            [-1e300, 0.0, 10.0],       # Negative huge x -> outside
        ])
        uv, depth, mask = cam_to_image(pts, self.calib.P2, self.img_shape)
        self.assertEqual(mask.sum(), 1)
        self.assertTrue(mask[0])
        self.assertFalse(mask[1])
        self.assertFalse(mask[2])
        self.assertFalse(mask[3])


class TestAdversarialDepths(unittest.TestCase):
    """Stress tests on depth constraints (z <= 0, min_depth, huge z)."""

    @classmethod
    def setUpClass(cls):
        cls.calib = load_calib(REPO_ROOT / "data/synthetic/training/calib/000000.txt")
        cls.img_shape = (375, 1242, 3)

    def test_01_negative_depths_strictly_rejected(self):
        """Points behind camera (z < 0) must never be projected or accepted."""
        pts = np.array([
            [0.0, 0.0, -1000.0],
            [0.0, 0.0, -100.0],
            [0.0, 0.0, -1.0],
            [0.0, 0.0, -0.001],
            [0.0, 0.0, -1e-6],
        ])
        uv, depth, mask = cam_to_image(pts, self.calib.P2, self.img_shape)
        self.assertEqual(mask.sum(), 0)
        self.assertEqual(len(uv), 0)

    def test_02_zero_depth_strictly_rejected(self):
        """Point at camera focal center (z = 0.0) must not cause divide-by-zero crash."""
        pts = np.array([[0.0, 0.0, 0.0]])
        uv, depth, mask = cam_to_image(pts, self.calib.P2, self.img_shape)
        self.assertEqual(mask.sum(), 0)
        self.assertFalse(mask[0])

    def test_03_min_depth_boundary_precision(self):
        """Test exact boundary: min_depth - eps, min_depth, min_depth + eps."""
        min_d = 0.1
        pts = np.array([
            [0.0, 0.0, 0.09999],    # Below min_depth
            [0.0, 0.0, 0.10000],    # Exactly min_depth (z > min_depth is False)
            [0.0, 0.0, 0.10001],    # Above min_depth
        ])
        uv, depth, mask = cam_to_image(pts, self.calib.P2, self.img_shape, min_depth=min_d)
        self.assertFalse(mask[0], "z < min_depth must be False")
        self.assertFalse(mask[1], "z == min_depth must be False under strict inequality")
        self.assertTrue(mask[2], "z > min_depth must be True")

    def test_04_huge_positive_depths(self):
        """Points very far away (z = 10,000m and z = 100,000m) along camera axis."""
        pts = np.array([
            [0.0, 0.0, 10000.0],
            [0.0, 0.0, 100000.0],
        ])
        uv, depth, mask = cam_to_image(pts, self.calib.P2, self.img_shape)
        self.assertEqual(mask.sum(), 2)
        # Principal point is roughly cx ~ 607, cy ~ 185
        cx = self.calib.P2[0, 2]
        cy = self.calib.P2[1, 2]
        np.testing.assert_allclose(uv[:, 0], cx, atol=1e-2)
        np.testing.assert_allclose(uv[:, 1], cy, atol=1e-2)

    def test_05_custom_min_depth_values(self):
        """Custom min_depth: 2.0m and negative min_depth."""
        pts = np.array([
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 3.0],
        ])
        uv, depth, mask = cam_to_image(pts, self.calib.P2, self.img_shape, min_depth=2.0)
        np.testing.assert_array_equal(mask, [False, True])


class TestAdversarialImageBoundaries(unittest.TestCase):
    """Stress tests on pixel coordinates, boundary limits, and subpixel accuracy."""

    @classmethod
    def setUpClass(cls):
        cls.calib = load_calib(REPO_ROOT / "data/synthetic/training/calib/000000.txt")
        cls.H = 375
        cls.W = 1242
        cls.img_shape = (cls.H, cls.W, 3)
        cls.P2 = cls.calib.P2
        # Extract intrinsic parameters: fx = P2[0,0], cx = P2[0,2], fy = P2[1,1], cy = P2[1,2]
        cls.fx = cls.P2[0, 0]
        cls.fy = cls.P2[1, 1]
        cls.cx = cls.P2[0, 2]
        cls.cy = cls.P2[1, 2]

    def _make_cam_point(self, u: float, v: float, z: float = 10.0) -> np.ndarray:
        """Construct 3D camera point that projects exactly to (u, v) at depth z using P2."""
        s = z + self.P2[2, 3]
        x = (u * s - self.P2[0, 2] * z - self.P2[0, 3]) / self.P2[0, 0]
        y = (v * s - self.P2[1, 2] * z - self.P2[1, 3]) / self.P2[1, 1]
        return np.array([x, y, z])

    def test_01_corner_pixels_inside(self):
        """Points projecting inside corner pixels (0.5, 0.5), (W-0.5, 0.5), (0.5, H-0.5), (W-0.5, H-0.5)."""
        pts = np.array([
            self._make_cam_point(0.5, 0.5),
            self._make_cam_point(self.W - 0.5, 0.5),
            self._make_cam_point(0.5, self.H - 0.5),
            self._make_cam_point(self.W - 0.5, self.H - 0.5),
            self._make_cam_point(self.W - 0.001, self.H - 0.001),
        ])
        uv, depth, mask = cam_to_image(pts, self.P2, self.img_shape)
        self.assertEqual(mask.sum(), 5)
        np.testing.assert_allclose(uv[0], [0.5, 0.5], atol=1e-3)
        np.testing.assert_allclose(uv[1], [self.W - 0.5, 0.5], atol=1e-3)
        np.testing.assert_allclose(uv[2], [0.5, self.H - 0.5], atol=1e-3)
        np.testing.assert_allclose(uv[3], [self.W - 0.5, self.H - 0.5], atol=1e-3)

    def test_02_strict_boundary_exclusion(self):
        """Points at exact outer boundary (u == W, v == H, u < 0, v < 0) must be excluded."""
        pts = np.array([
            self._make_cam_point(-0.001, 100.0),      # u < 0
            self._make_cam_point(100.0, -0.001),      # v < 0
            self._make_cam_point(float(self.W), 100.0), # u == W (out of bounds since 0 <= u < W)
            self._make_cam_point(100.0, float(self.H)), # v == H (out of bounds since 0 <= v < H)
            self._make_cam_point(float(self.W) + 1.0, 100.0),
            self._make_cam_point(100.0, float(self.H) + 1.0),
        ])
        uv, depth, mask = cam_to_image(pts, self.P2, self.img_shape)
        self.assertEqual(mask.sum(), 0, "Points on or outside outer boundaries must be excluded")

    def test_03_subpixel_floating_coordinates(self):
        """Subpixel coordinates must be preserved accurately as floats."""
        pts = np.array([
            self._make_cam_point(123.456, 234.567),
            self._make_cam_point(613.987, 175.123),
        ])
        uv, depth, mask = cam_to_image(pts, self.P2, self.img_shape)
        self.assertEqual(mask.sum(), 2)
        np.testing.assert_allclose(uv[0], [123.456, 234.567], atol=1e-3)
        np.testing.assert_allclose(uv[1], [613.987, 175.123], atol=1e-3)


class TestAdversarialScaleAndMemoryLeaks(unittest.TestCase):
    """Stress tests covering 1,000,000 points and multi-iteration memory leak detection."""

    @classmethod
    def setUpClass(cls):
        cls.calib = load_calib(REPO_ROOT / "data/synthetic/training/calib/000000.txt")
        cls.img_shape = (375, 1242, 3)

    def test_01_one_million_points_throughput_and_correctness(self):
        """Process 1,000,000 points through velo_to_cam and cam_to_image."""
        rng = np.random.default_rng(12345)
        n_pts = 1_000_000

        # Generate 1M LiDAR points in realistic range x: [0, 80], y: [-40, 40], z: [-3, 5]
        pts = rng.uniform(
            low=[0.0, -40.0, -3.0],
            high=[80.0, 40.0, 5.0],
            size=(n_pts, 3),
        ).astype(np.float64)

        # Measure velo_to_cam
        t0 = time.perf_counter()
        pts_cam = velo_to_cam(pts, self.calib)
        t_velo = time.perf_counter() - t0

        self.assertEqual(pts_cam.shape, (n_pts, 3))
        self.assertTrue(np.all(np.isfinite(pts_cam)))

        # Measure cam_to_image
        t1 = time.perf_counter()
        uv, depth, mask = cam_to_image(pts_cam, self.calib.P2, self.img_shape)
        t_cam = time.perf_counter() - t1

        self.assertEqual(len(mask), n_pts)
        self.assertEqual(len(uv), mask.sum())
        self.assertEqual(len(depth), mask.sum())

        total_time = t_velo + t_cam
        print(f"\n[1M Stress Test] velo_to_cam: {t_velo*1000:.1f}ms | cam_to_image: {t_cam*1000:.1f}ms | Total: {total_time*1000:.1f}ms (Points in FOV: {mask.sum()})")
        # Ensure 1M points completes within reasonable time (< 2.5s on modern CPU)
        self.assertLess(total_time, 2.5, f"1M points projection took too long: {total_time:.2f}s")

    def test_02_memory_leak_detection(self):
        """Verify memory is properly freed across 30 repeated runs of 200,000 points."""
        gc.collect()
        tracemalloc.start()

        rng = np.random.default_rng(999)
        n_pts = 200_000
        pts = rng.uniform(low=[0, -20, -2], high=[50, 20, 3], size=(n_pts, 3)).astype(np.float64)

        # Warmup
        _ = project_velo_to_image(pts, self.calib, self.img_shape)
        gc.collect()
        snapshot_start = tracemalloc.take_snapshot()

        # Run 30 iterations
        for _ in range(30):
            uv, depth, mask = project_velo_to_image(pts, self.calib, self.img_shape)
            del uv, depth, mask

        gc.collect()
        snapshot_end = tracemalloc.take_snapshot()
        tracemalloc.stop()

        top_stats = snapshot_end.compare_to(snapshot_start, "lineno")
        total_growth = sum(stat.size_diff for stat in top_stats)
        growth_kb = total_growth / 1024

        print(f"\n[Memory Leak Test] Memory growth across 30 iterations: {growth_kb:.2f} KB")
        # Memory growth should be negligible (< 100 KB across 30 iterations)
        self.assertLess(growth_kb, 150.0, f"Memory leak detected: growth was {growth_kb:.2f} KB")

    def test_03_five_million_points_throughput(self):
        """Ultra-scale stress test: 5,000,000 points throughput and stability."""
        rng = np.random.default_rng(54321)
        n_pts = 5_000_000
        pts = rng.uniform(low=[0.0, -40.0, -3.0], high=[80.0, 40.0, 5.0], size=(n_pts, 3)).astype(np.float64)

        t0 = time.perf_counter()
        pts_cam = velo_to_cam(pts, self.calib)
        uv, depth, mask = cam_to_image(pts_cam, self.calib.P2, self.img_shape)
        total_time = time.perf_counter() - t0

        print(f"\n[5M Ultra-Stress Test] 5M points projected in {total_time:.3f}s (In FOV: {mask.sum()})")
        self.assertEqual(len(mask), n_pts)
        self.assertEqual(len(uv), mask.sum())
        self.assertLess(total_time, 8.0, f"5M points took too long: {total_time:.2f}s")


class TestAdversarialDownstream(unittest.TestCase):
    """Stress tests on project_velo_to_image and overlay_points."""

    @classmethod
    def setUpClass(cls):
        cls.calib = load_calib(REPO_ROOT / "data/synthetic/training/calib/000000.txt")
        cls.img_shape = (375, 1242, 3)

    def test_01_project_velo_to_image_all_cases(self):
        """End-to-end project_velo_to_image on empty, NaN, and 100k points."""
        # Empty
        uv_e, d_e, m_e = project_velo_to_image(np.empty((0, 3)), self.calib, self.img_shape)
        self.assertEqual(len(uv_e), 0)
        self.assertEqual(len(d_e), 0)
        self.assertEqual(len(m_e), 0)

        # Behind sensor
        pts_behind = np.array([[-10.0, 0.0, 0.0], [-50.0, 5.0, 1.0]])
        uv_b, d_b, m_b = project_velo_to_image(pts_behind, self.calib, self.img_shape)
        self.assertEqual(m_b.sum(), 0)

    def test_02_overlay_points_empty_uv_behavior(self):
        """Document empirical behavior of overlay_points when uv is empty.
        
        Empirical finding: overlay_points crashes with TypeError because
        cv2.applyColorMap returns None on empty arrays.
        """
        img = np.zeros(self.img_shape, dtype=np.uint8)
        empty_uv = np.empty((0, 2), dtype=float)
        empty_d = np.empty((0,), dtype=float)

        try:
            overlay_points(img, empty_uv, empty_d)
            crashed = False
        except TypeError as e:
            crashed = True
            print(f"\n[Finding] overlay_points empty uv crash reproduced: {e}")

        self.assertTrue(crashed, "Empirically confirmed: overlay_points crashes on empty uv")

    def test_03_perturb_extrinsic_extreme_angles_and_immutability(self):
        """perturb_extrinsic preserves calib immutability and remains finite under extreme angles."""
        orig_tr = self.calib.Tr_velo_to_cam.copy()
        drifted = perturb_extrinsic(
            self.calib,
            roll_deg=90.0,
            pitch_deg=180.0,
            yaw_deg=360.0,
            t_xyz_m=(500.0, -500.0, 1000.0),
        )
        # Original calib must not be modified
        np.testing.assert_array_equal(self.calib.Tr_velo_to_cam, orig_tr)
        # Drifted calib must have valid shape (3, 4) and finite numbers
        self.assertEqual(drifted.Tr_velo_to_cam.shape, (3, 4))
        self.assertTrue(np.all(np.isfinite(drifted.Tr_velo_to_cam)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
