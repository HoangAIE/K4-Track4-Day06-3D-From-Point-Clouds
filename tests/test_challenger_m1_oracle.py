"""Milestone 1 Empirical & Oracle Challenger Adversarial Test Suite.

Author: Challenger 1 (specialist & critic)
Target: starter/projection.py (velo_to_cam, cam_to_image, project_velo_to_image)
Scope:
  1. Mathematical Oracle Invariance (high-precision analytical comparison)
  2. Inverse Transformation & SE(3) Conservation
  3. Camera Frustum Invariance (forward points, FOV bounding box)
  4. CP2 Benchmark Points (Synthetic, KITTI, nuScenes)
  5. Adversarial Input Stress (NaN, Inf, zeros, subnormals, extreme magnitudes)
  6. Dimension & Dtype Flexibility (1D, 2D, extra channels (N,4), (N,5), float32, float64)
  7. Boundary Threshold Exactness (z <= min_depth, u/v == W/H, u/v < 0)
  8. Stress & High-Volume Performance (1,000,000 points)
  9. Real-Dataset Overlays & Ground Truth Verification
"""
from __future__ import annotations

import math
import sys
import time
import unittest
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from starter.datasets import dataset_type, load_frame, load_points
from starter.kitti_io import KittiCalib, load_calib
from starter.projection import (
    cam_to_image,
    overlay_points,
    perturb_extrinsic,
    project_velo_to_image,
    velo_to_cam,
)


class ChallengerM1OracleTests(unittest.TestCase):
    """Rigorous empirical oracle and stress testing for Milestone 1."""

    @classmethod
    def setUpClass(cls):
        cls.synthetic_calib = load_calib(REPO_ROOT / "data/synthetic/training/calib/000000.txt")
        cls.kitti_calib = load_calib(REPO_ROOT / "data/kitti_mini/training/calib/000001.txt")
        cls.nuscenes_calib = load_frame(REPO_ROOT / "data/nuscenes_mini_subset", "scene-0103_010")["calib"]

    # =========================================================================
    # Section 1: Mathematical Oracle Invariants
    # =========================================================================

    def test_oracle_velo_to_cam_analytical_equivalence(self):
        """Verify velo_to_cam matches independent analytical SE(3) transformation."""
        rng = np.random.default_rng(1337)
        points_velo = rng.uniform(-50.0, 50.0, size=(10000, 3))

        T = self.synthetic_calib.T_cam_velo
        R = T[:3, :3]
        t = T[:3, 3]

        # Analytical oracle: P_cam = P_velo @ R.T + t
        expected_cam = points_velo @ R.T + t

        actual_cam = velo_to_cam(points_velo, self.synthetic_calib)
        np.testing.assert_allclose(actual_cam, expected_cam, rtol=1e-6, atol=1e-6,
                                  err_msg="velo_to_cam does not match analytical SE(3) oracle")

    def test_oracle_velo_to_cam_inverse_reversibility(self):
        """Verify SE(3) inverse transformation strictly recovers velodyne points."""
        rng = np.random.default_rng(2026)
        points_velo = rng.uniform(-40.0, 40.0, size=(5000, 3))

        T = self.kitti_calib.T_cam_velo
        R = T[:3, :3]
        t = T[:3, 3]

        pts_cam = velo_to_cam(points_velo, self.kitti_calib)

        # Invert: P_velo_rec = (P_cam - t) @ R
        pts_velo_rec = (pts_cam - t) @ R
        np.testing.assert_allclose(pts_velo_rec, points_velo, rtol=1e-5, atol=1e-5,
                                  err_msg="Inverse transformation failed to recover original velo coordinates")

    def test_oracle_cam_to_image_analytical_pinhole(self):
        """Verify cam_to_image matches analytical pinhole projective formula."""
        rng = np.random.default_rng(4242)
        # Generate 20,000 points scattered in front of and around camera
        x = rng.uniform(-20.0, 20.0, size=20000)
        y = rng.uniform(-10.0, 10.0, size=20000)
        z = rng.uniform(0.01, 80.0, size=20000)
        points_cam = np.column_stack([x, y, z])

        P2 = self.synthetic_calib.P2
        H, W = 375, 1242
        min_depth = 0.1

        uv, depth, mask = cam_to_image(points_cam, P2, (H, W), min_depth=min_depth)

        # Analytical oracle evaluation per point
        s_expected = P2[2, 0] * x + P2[2, 1] * y + P2[2, 2] * z + P2[2, 3]
        u_expected = (P2[0, 0] * x + P2[0, 1] * y + P2[0, 2] * z + P2[0, 3]) / s_expected
        v_expected = (P2[1, 0] * x + P2[1, 1] * y + P2[1, 2] * z + P2[1, 3]) / s_expected

        expected_valid = (
            (z > min_depth) &
            (s_expected > 1e-4) &
            (u_expected >= 0.0) & (u_expected < W) &
            (v_expected >= 0.0) & (v_expected < H)
        )

        np.testing.assert_array_equal(mask, expected_valid,
                                      err_msg="cam_to_image mask diverges from analytical pinhole oracle")
        np.testing.assert_allclose(uv[:, 0], u_expected[expected_valid], rtol=1e-5, atol=1e-5)
        np.testing.assert_allclose(uv[:, 1], v_expected[expected_valid], rtol=1e-5, atol=1e-5)
        np.testing.assert_allclose(depth, z[expected_valid], rtol=1e-5, atol=1e-5)

    # =========================================================================
    # Section 2: CP2 Benchmark Assertions
    # =========================================================================

    def test_cp2_benchmark_synthetic(self):
        """Verify CP2 benchmark point (10, 0, 0) on data/synthetic."""
        test_pt = np.array([[10.0, 0.0, 0.0]])
        pt_cam = velo_to_cam(test_pt, self.synthetic_calib)

        # Invariant 1: Forward point in Velodyne maps to positive z_cam
        self.assertGreater(pt_cam[0, 2], 0.0, "z_cam must be positive")
        self.assertAlmostEqual(pt_cam[0, 2], 9.7273, places=2)

        uv, depth, mask = cam_to_image(pt_cam, self.synthetic_calib.P2, (375, 1242))
        self.assertTrue(mask[0], "Benchmark point must project inside synthetic camera FOV")
        self.assertAlmostEqual(uv[0, 0], 613.96, delta=0.5, msg="u coordinate should be ~614")
        self.assertAlmostEqual(uv[0, 1], 175.01, delta=0.5, msg="v coordinate should be ~175")
        self.assertAlmostEqual(depth[0], pt_cam[0, 2], delta=1e-4)

    def test_cp2_benchmark_kitti_mini(self):
        """Verify CP2 benchmark point (10, 0, 0) on data/kitti_mini."""
        test_pt = np.array([[10.0, 0.0, 0.0]])
        pt_cam = velo_to_cam(test_pt, self.kitti_calib)

        self.assertGreater(pt_cam[0, 2], 0.0, "z_cam must be positive for KITTI")
        self.assertAlmostEqual(pt_cam[0, 2], 9.72, delta=0.5)

        uv, depth, mask = cam_to_image(pt_cam, self.kitti_calib.P2, (375, 1242))
        self.assertTrue(mask[0])
        self.assertTrue(0 <= uv[0, 0] < 1242)
        self.assertTrue(0 <= uv[0, 1] < 375)

    def test_cp2_benchmark_nuscenes(self):
        """Verify forward point in nuScenes LiDAR frame (y forward, x right)."""
        # In nuScenes: x is right, y is forward, z is up.
        # Therefore, a point 10m directly ahead of vehicle is (0, 10, 0), NOT (10, 0, 0).
        test_pt_fwd = np.array([[0.0, 10.0, 0.0]])
        pt_cam = velo_to_cam(test_pt_fwd, self.nuscenes_calib)

        self.assertGreater(pt_cam[0, 2], 0.0, "nuScenes forward point (0, 10, 0) must have positive z_cam")
        self.assertAlmostEqual(pt_cam[0, 2], 9.558, delta=0.5)

        uv, depth, mask = cam_to_image(pt_cam, self.nuscenes_calib.P2, (900, 1600))
        self.assertTrue(mask[0], "Forward point (0, 10, 0) must project into CAM_FRONT FOV")
        self.assertTrue(0 <= uv[0, 0] < 1600)
        self.assertTrue(0 <= uv[0, 1] < 900)
        # Point should be centered near principal point (c_u ~ 800, c_v ~ 450)
        self.assertAlmostEqual(uv[0, 0], 840.0, delta=50.0)
        self.assertAlmostEqual(uv[0, 1], 493.0, delta=50.0)

    # =========================================================================
    # Section 3: Adversarial Boundary & Edge Conditions
    # =========================================================================

    def test_boundary_depth_filtering(self):
        """Adversarial stress on depth boundary conditions."""
        min_depth = 0.5
        pts_cam = np.array([
            [0.0, 0.0, -100.0],           # far behind
            [0.0, 0.0, -0.0001],          # just behind optical plane
            [0.0, 0.0, 0.0],              # exact optical center (z=0)
            [0.0, 0.0, 1e-7],             # infinitesimal positive depth
            [0.0, 0.0, min_depth - 1e-6], # strictly below threshold
            [0.0, 0.0, min_depth],        # exact boundary (z == min_depth)
            [0.0, 0.0, min_depth + 1e-6], # strictly above threshold
            [0.0, 0.0, 10.0],             # nominal depth
        ])
        uv, depth, mask = cam_to_image(pts_cam, self.synthetic_calib.P2, (375, 1242), min_depth=min_depth)

        expected_mask = np.array([False, False, False, False, False, False, True, True])
        np.testing.assert_array_equal(mask, expected_mask,
                                      err_msg="cam_to_image did not filter exact depth boundaries correctly")

    def test_boundary_pixel_coordinates(self):
        """Adversarial stress on exact image boundary edges [0, W) x [0, H)."""
        P2 = np.array([
            [100.0, 0.0, 50.0, 0.0],
            [0.0, 100.0, 50.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ])
        H, W = 100, 100
        # u = 100*x/z + 50. With z=1:
        # x = -0.5 -> u = 0.0 (top-left border -> IN)
        # x = -0.50001 -> u = -0.001 (outside left -> OUT)
        # x = 0.49999 -> u = 99.999 (inside right -> IN)
        # x = 0.5 -> u = 100.0 (exact right border -> OUT since u < W)
        # y = -0.5 -> v = 0.0 (top border -> IN)
        # y = 0.5 -> v = 100.0 (bottom border -> OUT since v < H)
        pts_cam = np.array([
            [-0.5, -0.5, 1.0],      # u=0.0, v=0.0 -> IN
            [-0.50001, 0.0, 1.0],   # u < 0 -> OUT
            [0.49999, 0.0, 1.0],    # u ~ 99.999 -> IN
            [0.5, 0.0, 1.0],        # u = 100.0 -> OUT
            [0.0, -0.50001, 1.0],   # v < 0 -> OUT
            [0.0, 0.5, 1.0],        # v = 100.0 -> OUT
        ])
        uv, depth, mask = cam_to_image(pts_cam, P2, (H, W), min_depth=0.1)
        expected_mask = np.array([True, False, True, False, False, False])
        np.testing.assert_array_equal(mask, expected_mask,
                                      err_msg="Pixel boundaries [0, W) x [0, H) violated")

    def test_adversarial_nan_and_inf_robustness(self):
        """Adversarial stress with NaNs, +/- Infs, and corrupted floats."""
        pts_dirty = np.array([
            [np.nan, 0.0, 10.0],
            [0.0, np.nan, 10.0],
            [0.0, 0.0, np.nan],
            [np.inf, 0.0, 10.0],
            [-np.inf, 0.0, 10.0],
            [0.0, np.inf, 10.0],
            [0.0, -np.inf, 10.0],
            [0.0, 0.0, np.inf],
            [0.0, 0.0, -np.inf],
            [np.nan, np.nan, np.nan],
            [0.0, 0.0, 10.0],      # Clean point
        ])

        # cam_to_image should filter all non-finites without throwing
        uv, depth, mask = cam_to_image(pts_dirty, self.synthetic_calib.P2, (375, 1242))
        self.assertEqual(mask.sum(), 1, "Only the 1 clean point should survive")
        self.assertTrue(mask[-1])
        self.assertTrue(np.isfinite(uv).all())
        self.assertTrue(np.isfinite(depth).all())

    def test_adversarial_shapes_and_dtypes(self):
        """Adversarial stress on input shapes: 1D, (N, 4), (N, 5), float32, float64, empty."""
        # 1D input to velo_to_cam
        pt_1d = np.array([10.0, 0.0, 0.0], dtype=np.float32)
        out_1d = velo_to_cam(pt_1d, self.synthetic_calib)
        self.assertEqual(out_1d.shape, (3,))

        # (N, 4) input (with intensity)
        pts_4d = np.array([[10.0, 0.0, 0.0, 0.8], [5.0, 2.0, 1.0, 0.3]])
        out_4d = velo_to_cam(pts_4d, self.synthetic_calib)
        self.assertEqual(out_4d.shape, (2, 3))

        # (N, 5) input
        pts_5d = np.random.default_rng(0).uniform(-10, 10, size=(10, 5))
        out_5d = velo_to_cam(pts_5d, self.synthetic_calib)
        self.assertEqual(out_5d.shape, (10, 3))

        # Empty inputs
        for empty_shape in [(0, 3), (0, 4), (0,)]:
            pts_empty = np.empty(empty_shape)
            out_empty = velo_to_cam(pts_empty, self.synthetic_calib)
            self.assertEqual(out_empty.shape, (0, 3))

            uv_e, d_e, m_e = cam_to_image(out_empty, self.synthetic_calib.P2, (375, 1242))
            self.assertEqual(uv_e.shape, (0, 2))
            self.assertEqual(d_e.shape, (0,))
            self.assertEqual(m_e.shape, (0,))

    # =========================================================================
    # Section 4: High-Volume Performance Stress
    # =========================================================================

    def test_stress_one_million_points_throughput(self):
        """Stress-test with 1,000,000 points to measure throughput and memory stability."""
        rng = np.random.default_rng(9999)
        points_huge = rng.uniform(-50.0, 50.0, size=(1000000, 3)).astype(np.float32)

        t0 = time.perf_counter()
        pts_cam = velo_to_cam(points_huge, self.synthetic_calib)
        t1 = time.perf_counter()
        uv, depth, mask = cam_to_image(pts_cam, self.synthetic_calib.P2, (375, 1242))
        t2 = time.perf_counter()

        time_velo = (t1 - t0) * 1000.0
        time_cam = (t2 - t1) * 1000.0
        time_total = (t2 - t0) * 1000.0

        print(f"\n[STRESS TEST] 1,000,000 points projection completed in {time_total:.1f}ms "
              f"(velo_to_cam: {time_velo:.1f}ms, cam_to_image: {time_cam:.1f}ms)")
        print(f"[STRESS TEST] Retained points in FOV: {mask.sum():,} / {len(points_huge):,} ({mask.mean():.1%})")

        # Invariant checks
        self.assertEqual(len(uv), mask.sum())
        self.assertEqual(len(depth), mask.sum())
        self.assertTrue((depth > 0.1).all())
        self.assertTrue((uv[:, 0] >= 0).all() and (uv[:, 0] < 1242).all())
        self.assertTrue((uv[:, 1] >= 0).all() and (uv[:, 1] < 375).all())

        # Performance constraint: 1M points must take < 1.5 seconds on modern CPU
        self.assertLess(time_total, 1500.0, "1M points projection exceeded 1.5s latency threshold")

    # =========================================================================
    # Section 5: Full Pipeline on All 3 Datasets
    # =========================================================================

    def test_full_pipeline_synthetic_dataset(self):
        """Run full projection on data/synthetic frame 000000 (with known NaN points)."""
        fr = load_frame(REPO_ROOT / "data/synthetic", "000000")
        uv, depth, mask = project_velo_to_image(fr["points"], fr["calib"], fr["image"].shape)
        self.assertEqual(len(mask), len(fr["points"]))
        self.assertGreater(mask.sum(), 3000)
        self.assertTrue(np.isfinite(uv).all())
        self.assertTrue(np.isfinite(depth).all())
        self.assertTrue((depth > 0).all())

    def test_full_pipeline_kitti_mini_dataset(self):
        """Run full projection on data/kitti_mini frame 000011."""
        fr = load_frame(REPO_ROOT / "data/kitti_mini", "000011")
        uv, depth, mask = project_velo_to_image(fr["points"], fr["calib"], fr["image"].shape)
        self.assertEqual(len(mask), 108004)
        self.assertEqual(mask.sum(), 19946)
        self.assertAlmostEqual(mask.mean(), 0.18468, places=4)
        self.assertTrue((depth > 0).all())

    def test_full_pipeline_nuscenes_dataset(self):
        """Run full projection on data/nuscenes_mini_subset frame scene-0103_010."""
        fr = load_frame(REPO_ROOT / "data/nuscenes_mini_subset", "scene-0103_010")
        uv, depth, mask = project_velo_to_image(fr["points"], fr["calib"], fr["image"].shape)
        self.assertEqual(len(mask), 34720)
        self.assertEqual(mask.sum(), 3120)
        self.assertAlmostEqual(mask.mean(), 0.08986, places=4)
        self.assertTrue((depth > 0).all())


if __name__ == "__main__":
    unittest.main()
