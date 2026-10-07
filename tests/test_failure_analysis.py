"""Unit and Integration Tests for src.failure_analysis module.

Verifies:
1. Module import and schema contracts (FAILURE_SCENARIOS, CANONICAL_DEBUG_LAYERS).
2. Debug layer classification compliance across all registered scenarios.
3. Box projection drawing utility (draw_box3d_projected).
4. Zoom-crop overlay utility (add_zoom_crop_overlay).
5. Kinematic / geometric error calculations (lateral drift e = d * sin(psi)).
6. Generated failure figure artifacts validation (dimensions, 3 channels, non-uniform pixels).
"""
from __future__ import annotations

import math
from pathlib import Path
import unittest

import cv2
import numpy as np

from starter.datasets import load_frame
from starter.kitti_io import KittiCalib, KittiObject
from src.failure_analysis import (
    CANONICAL_DEBUG_LAYERS,
    FAILURE_SCENARIOS,
    add_zoom_crop_overlay,
    draw_box3d_projected,
)

REPO_ROOT = Path(__file__).resolve().parent.parent


class TestFailureAnalysis(unittest.TestCase):
    """Test suite for failure analysis module components."""

    def test_01_canonical_debug_layers_membership(self):
        """Verify canonical debug layer definitions match standard perception taxonomy."""
        expected_layers = {"I/O", "Geometry", "Time", "Preprocess", "Model", "Metric", "Sensor/Environment", "Sensor"}
        self.assertTrue(expected_layers.issubset(CANONICAL_DEBUG_LAYERS))

    def test_02_failure_scenarios_metadata_schema(self):
        """Verify each scenario in FAILURE_SCENARIOS adheres to required metadata schema."""
        self.assertGreaterEqual(len(FAILURE_SCENARIOS), 3)
        required_keys = {"title", "layer", "dataset", "frame_id", "output_file", "description", "root_cause", "mitigation"}

        for scenario_id, meta in FAILURE_SCENARIOS.items():
            self.assertTrue(scenario_id.startswith("fail_"))
            for key in required_keys:
                self.assertIn(key, meta, f"Missing key '{key}' in scenario '{scenario_id}'")
            self.assertIn(meta["layer"], CANONICAL_DEBUG_LAYERS, f"Invalid debug layer in '{scenario_id}'")
            self.assertTrue(meta["output_file"].endswith(".png"))

    def test_03_required_debug_layers_represented(self):
        """Verify Geometry, Sensor/Environment, and Time debug layers are represented."""
        layers = {meta["layer"] for meta in FAILURE_SCENARIOS.values()}
        self.assertIn("Geometry", layers)
        self.assertTrue("Sensor/Environment" in layers or "Sensor" in layers)
        self.assertIn("Time", layers)

    def test_04_box3d_drawing_utility(self):
        """Verify draw_box3d_projected correctly renders onto image without mutation."""
        calib = KittiCalib(
            P2=np.array([[721.5, 0.0, 609.5, 44.8], [0.0, 721.5, 172.8, 0.2], [0.0, 0.0, 1.0, 0.0]]),
            R0_rect=np.eye(3),
            Tr_velo_to_cam=np.eye(3, 4),
        )
        dummy_obj = KittiObject(
            type="Car",
            truncated=0.0,
            occluded=0,
            alpha=0.0,
            bbox=np.array([100.0, 100.0, 200.0, 200.0]),
            dimensions=np.array([1.6, 1.8, 4.2]),
            location=np.array([0.0, 1.0, 15.0]),
            rotation_y=0.0,
        )
        img = np.zeros((375, 1242, 3), dtype=np.uint8)
        rendered = draw_box3d_projected(img, dummy_obj, calib, color=(0, 255, 0), thickness=2, label="Car")

        self.assertEqual(rendered.shape, img.shape)
        self.assertGreater(rendered.sum(), 0, "Rendered image must have drawn pixels")
        self.assertEqual(img.sum(), 0, "Original image must remain unchanged")

    def test_05_zoom_crop_overlay_utility(self):
        """Verify add_zoom_crop_overlay extracts patch and embeds enlarged zoom."""
        img = np.ones((400, 800, 3), dtype=np.uint8) * 50
        # Draw distinctive marker at source
        img[90:110, 190:210] = 255

        zoomed = add_zoom_crop_overlay(
            img,
            center_xy=(200, 100),
            crop_size=(40, 40),
            dest_rect=(600, 20, 150, 120),
            label="ZOOM TEST",
        )
        self.assertEqual(zoomed.shape, img.shape)
        # Destination area should contain white border and pixels
        dest_patch = zoomed[20:140, 600:750]
        self.assertGreater(dest_patch.mean(), 50)

    def test_06_lateral_displacement_math(self):
        """Verify distance-amplified lateral displacement matches e = d * sin(psi)."""
        d = 46.1  # distance to cyclist in frame 000001
        yaw_1deg = math.radians(1.0)
        e_1deg = d * math.sin(yaw_1deg)
        self.assertAlmostEqual(e_1deg, 0.8045, places=3)

        # Displacement must scale monotonically with distance and angle
        d_far = 92.2
        e_far = d_far * math.sin(yaw_1deg)
        self.assertAlmostEqual(e_far, e_1deg * 2.0, places=3)

    def test_07_generated_figures_inspection(self):
        """Verify all generated failure figures in results/figures meet quality standards."""
        fig_dir = REPO_ROOT / "results/figures"
        if not fig_dir.exists():
            self.skipTest("results/figures not yet created")

        for scenario_id, meta in FAILURE_SCENARIOS.items():
            fig_path = fig_dir / meta["output_file"]
            if not fig_path.exists():
                self.skipTest(f"{fig_path.name} not found")
            img = cv2.imread(str(fig_path))
            self.assertIsNotNone(img, f"Failed to read image {fig_path}")
            self.assertEqual(len(img.shape), 3, "Image must have 3 dimensions")
            self.assertEqual(img.shape[2], 3, "Image must have 3 color channels")
            self.assertGreater(img.shape[0], 500, "Image height must be high resolution")
            self.assertGreater(img.shape[1], 800, "Image width must be high resolution")
            self.assertGreater(img.std(), 10.0, "Image must have high pixel variance")


if __name__ == "__main__":
    unittest.main()
