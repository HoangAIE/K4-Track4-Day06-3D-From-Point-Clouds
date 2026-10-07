"""Interactive Desktop Demo Application & Headless Snapshot Engine.

Topic C: LiDAR-Camera Sensor Degradation Stress Test & Health Monitor.
Provides:
1. Real-time CPU-smooth Tkinter desktop GUI with side-by-side Camera Depth
   Projection and Bird's-Eye-View (BEV) Visualizers, Degradation Sliders,
   and Live Telemetry Cards.
2. Headless batch mode (--headless) with automated composite dashboard export
   (--export-snapshot / --export-demo).
"""
from __future__ import annotations

import argparse
import copy
import os
from pathlib import Path
import sys
import time
from typing import Any

import cv2
import numpy as np

# Core geometry and perturbation modules
from starter.datasets import dataset_type, list_frames, load_frame
from starter.kitti_io import KittiCalib, KittiObject
from starter.perturb import (
    beam_dropout,
    gaussian_noise,
    random_dropout,
    range_dropout,
)
from starter.projection import (
    box3d_corners_cam,
    cam_to_image,
    overlay_points,
    perturb_extrinsic,
    project_velo_to_image,
    velo_to_cam,
)
from src.metrics import (
    classify_health_score,
    compute_frame_metrics,
    compute_sensor_health_score,
)

# Optional Tkinter / PIL imports for desktop GUI mode
try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
    from PIL import Image, ImageTk
    HAS_TKINTER = True
except Exception:
    HAS_TKINTER = False

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SNAPSHOT_PATH = REPO_ROOT / "results/figures/demo_gui_snapshot.png"

DATASET_OPTIONS: dict[str, str] = {
    "kitti_mini": "data/kitti_mini",
    "synthetic": "data/synthetic",
    "nuscenes_mini_subset": "data/nuscenes_mini_subset",
}

CLASS_COLORS: dict[str, tuple[int, int, int]] = {
    "Car": (100, 255, 0),        # BGR: Lime Green
    "Van": (100, 255, 0),
    "Truck": (50, 200, 255),     # BGR: Amber
    "Pedestrian": (0, 230, 255), # BGR: Yellow-Cyan
    "Cyclist": (0, 140, 255),    # BGR: Orange
    "Bicycle": (0, 140, 255),
    "Default": (180, 180, 180),  # Gray
}


def apply_degradations(
    points: np.ndarray,
    calib: KittiCalib,
    dropout: float = 0.0,
    range_cutoff: float = 80.0,
    beam_count: int = 64,
    noise_sigma: float = 0.0,
    yaw_deg: float = 0.0,
    pitch_deg: float = 0.0,
    roll_deg: float = 0.0,
    seed: int = 42,
) -> tuple[np.ndarray, KittiCalib]:
    """Apply sensor degradation pipeline in deterministic sequence.

    Pipeline Order:
    1. Extrinsic calibration drift (Yaw, Pitch, Roll)
    2. Simulated beam decimation (Beam Dropout)
    3. Maximum range attenuation (Range Dropout)
    4. Random point loss (Random Dropout)
    5. Gaussian coordinate jitter (Gaussian Noise)
    """
    pts = points.copy()

    # 1. Extrinsic calibration drift
    if abs(yaw_deg) > 1e-4 or abs(pitch_deg) > 1e-4 or abs(roll_deg) > 1e-4:
        calib_out = perturb_extrinsic(
            calib,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            yaw_deg=yaw_deg,
        )
    else:
        calib_out = calib

    if len(pts) == 0:
        return pts, calib_out

    # 2. Simulated beam decimation
    if beam_count < 64:
        keep_every = max(1, 64 // max(1, beam_count))
        if keep_every > 1:
            pts = beam_dropout(pts, keep_every=keep_every, n_beams=64)

    if len(pts) == 0:
        return pts, calib_out

    # 3. Maximum range cutoff
    if range_cutoff < 79.5:
        pts = range_dropout(pts, max_range_m=float(range_cutoff))

    if len(pts) == 0:
        return pts, calib_out

    # 4. Random point dropout
    if dropout > 1e-4:
        keep_ratio = max(0.0, min(1.0, 1.0 - float(dropout)))
        pts = random_dropout(pts, keep_ratio=keep_ratio, seed=seed)

    if len(pts) == 0:
        return pts, calib_out

    # 5. Gaussian coordinate jitter
    if noise_sigma > 1e-4:
        pts = gaussian_noise(pts, sigma_xyz_m=float(noise_sigma), seed=seed)

    return pts, calib_out


def render_camera_view(
    image: np.ndarray,
    points: np.ndarray,
    calib: KittiCalib,
    labels: list[KittiObject] | None = None,
    max_depth: float = 50.0,
    radius: int = 2,
) -> np.ndarray:
    """Render camera frame with projected LiDAR points and 3D GT bounding boxes."""
    vis = image.copy()
    img_h, img_w = vis.shape[:2]

    # Project LiDAR points
    if len(points) > 0:
        uv, depth, mask = project_velo_to_image(points, calib, vis.shape)
        if len(uv) > 0:
            vis = overlay_points(vis, uv, depth, max_depth=max_depth, radius=radius)

    # Overlaid 3D ground truth bounding boxes
    if labels:
        wireframe_edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom face
            (4, 5), (5, 6), (6, 7), (7, 4),  # Top face
            (0, 4), (1, 5), (2, 6), (3, 7),  # Vertical pillars
        ]
        for obj in labels:
            color = CLASS_COLORS.get(obj.type, CLASS_COLORS["Default"])
            try:
                corners_cam = box3d_corners_cam(obj)
                uv_box, d_box, mask_box = cam_to_image(corners_cam, calib.P2, (img_h, img_w), min_depth=0.1)
                if len(uv_box) == 8 and (d_box > 0.1).sum() >= 4:
                    # Draw 12 box edges
                    for i, j in wireframe_edges:
                        if d_box[i] > 0.1 and d_box[j] > 0.1:
                            pt1 = (int(round(uv_box[i, 0])), int(round(uv_box[i, 1])))
                            pt2 = (int(round(uv_box[j, 0])), int(round(uv_box[j, 1])))
                            cv2.line(vis, pt1, pt2, color=color, thickness=2, lineType=cv2.LINE_AA)

                    # Front face diagonal indicator (front corners are 0, 1, 5, 4)
                    if d_box[0] > 0.1 and d_box[5] > 0.1:
                        pt_a = (int(round(uv_box[0, 0])), int(round(uv_box[0, 1])))
                        pt_b = (int(round(uv_box[5, 0])), int(round(uv_box[5, 1])))
                        cv2.line(vis, pt_a, pt_b, color=color, thickness=1, lineType=cv2.LINE_AA)

                    # Text label at highest corner
                    min_x = int(np.clip(np.min(uv_box[:, 0]), 2, img_w - 100))
                    min_y = int(np.clip(np.min(uv_box[:, 1]) - 4, 15, img_h - 5))
                    label_str = f"{obj.type} {obj.location[2]:.1f}m"
                    cv2.putText(vis, label_str, (min_x, min_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            except Exception:
                continue

    return vis


def render_bev(
    points: np.ndarray,
    calib: KittiCalib | None = None,
    labels: list[KittiObject] | None = None,
    width: int = 600,
    height: int = 600,
    range_x: tuple[float, float] = (-30.0, 30.0),
    range_z: tuple[float, float] = (0.0, 60.0),
    max_range: float = 60.0,
) -> np.ndarray:
    """Render top-down Bird's-Eye-View (BEV) representation of point cloud and 3D boxes.

    Camera frame coordinates:
    - z: forward distance (depth) [0.0, 60.0] meters (upwards on screen).
    - x: lateral distance [-30.0, +30.0] meters (left/right on screen).
    - Vehicle origin at (x=0, z=0) at bottom-center of map.
    """
    bev = np.full((height, width, 3), (22, 25, 28), dtype=np.uint8)

    x_min, x_max = range_x
    z_min, z_max = range_z
    u0 = int((0.0 - x_min) / (x_max - x_min) * (width - 1))
    v0 = int((1.0 - (0.0 - z_min) / (z_max - z_min)) * (height - 1))

    # 1. Lateral grid lines
    for x_val in [-20.0, -10.0, 10.0, 20.0]:
        u_grid = int((x_val - x_min) / (x_max - x_min) * (width - 1))
        cv2.line(bev, (u_grid, 0), (u_grid, height - 1), (38, 43, 50), 1)
    # Centerline
    cv2.line(bev, (u0, 0), (u0, height - 1), (55, 65, 75), 1)

    # 2. Concentric range rings & distance text
    for r in [10, 20, 30, 40, 50, 60]:
        r_px = int(r / (z_max - z_min) * (height - 1))
        cv2.circle(bev, (u0, v0), r_px, (48, 54, 62), 1, cv2.LINE_AA)
        cv2.putText(
            bev, f"{r}m", (u0 + 6, max(14, v0 - r_px - 4)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (110, 120, 135), 1, cv2.LINE_AA
        )

    # 3. Camera FOV cone boundaries (~80 deg horizontal FOV)
    fov_half_rad = np.deg2rad(41.0)
    ray_len = 60.0
    for sign in [-1.0, 1.0]:
        x_ray = sign * ray_len * np.sin(fov_half_rad)
        z_ray = ray_len * np.cos(fov_half_rad)
        u_ray = int((x_ray - x_min) / (x_max - x_min) * (width - 1))
        v_ray = int((1.0 - (z_ray - z_min) / (z_max - z_min)) * (height - 1))
        cv2.line(bev, (u0, v0), (u_ray, v_ray), (70, 65, 45), 1, cv2.LINE_AA)

    # 4. Render LiDAR points
    if len(points) > 0:
        pts = np.asarray(points)
        if pts.ndim == 1:
            pts = pts.reshape(1, -1)

        # Coordinate transformation
        if calib is not None:
            pts_cam = velo_to_cam(pts[:, :3], calib)
            x_pts = pts_cam[:, 0]
            z_pts = pts_cam[:, 2]
        else:
            # Fallback for raw velo points when calib is omitted (kitti velo: x forward, y left)
            x_pts = -pts[:, 1]
            z_pts = pts[:, 0]

        finite = np.isfinite(x_pts) & np.isfinite(z_pts)
        in_roi = finite & (x_pts >= x_min) & (x_pts <= x_max) & (z_pts >= z_min) & (z_pts <= z_max)

        x_roi, z_roi = x_pts[in_roi], z_pts[in_roi]
        if len(x_roi) > 0:
            u_coords = ((x_roi - x_min) / (x_max - x_min) * (width - 1)).astype(np.int32)
            v_coords = ((1.0 - (z_roi - z_min) / (z_max - z_min)) * (height - 1)).astype(np.int32)

            dists = np.sqrt(x_roi**2 + z_roi**2)
            d_norm = np.clip(dists / 50.0, 0.0, 1.0)
            colors = cv2.applyColorMap(
                (255 * (1.0 - d_norm)).astype(np.uint8).reshape(-1, 1),
                cv2.COLORMAP_JET
            )[:, 0, :]

            bev[v_coords, u_coords] = colors
            # Gentle dilation for enhanced point visibility
            bev = cv2.dilate(bev, np.ones((2, 2), np.uint8))

    # 5. Render 3D GT Bounding Boxes in BEV plane
    if labels:
        for obj in labels:
            color = CLASS_COLORS.get(obj.type, CLASS_COLORS["Default"])
            try:
                corners_cam = box3d_corners_cam(obj)
                # Bottom 4 corners (x, z)
                poly_x = corners_cam[0:4, 0]
                poly_z = corners_cam[0:4, 2]

                u_poly = ((poly_x - x_min) / (x_max - x_min) * (width - 1)).astype(np.int32)
                v_poly = ((1.0 - (poly_z - z_min) / (z_max - z_min)) * (height - 1)).astype(np.int32)
                poly_pts = np.column_stack([u_poly, v_poly])

                # Draw polygon footprint
                cv2.polylines(bev, [poly_pts], isClosed=True, color=color, thickness=2, lineType=cv2.LINE_AA)

                # Heading indicator arrow (front midpoint is between corners 0 and 1)
                front_mid = ((poly_pts[0] + poly_pts[1]) / 2.0).astype(np.int32)
                center_pt = np.mean(poly_pts, axis=0).astype(np.int32)
                cv2.arrowedLine(
                    bev, tuple(center_pt), tuple(front_mid),
                    color=(255, 255, 255), thickness=1, tipLength=0.3, line_type=cv2.LINE_AA
                )

                # Class name
                label_u = int(np.clip(np.min(u_poly), 2, width - 60))
                label_v = int(np.clip(np.min(v_poly) - 3, 12, height - 4))
                cv2.putText(
                    bev, obj.type[:3], (label_u, label_v),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA
                )
            except Exception:
                continue

    # 6. Ego Vehicle Marker at Origin (0, 0)
    car_w_px = int(1.8 / (x_max - x_min) * (width - 1))
    car_l_px = int(4.2 / (z_max - z_min) * (height - 1))
    c_x1, c_x2 = u0 - car_w_px // 2, u0 + car_w_px // 2
    c_y1, c_y2 = v0 - car_l_px, v0
    cv2.rectangle(bev, (c_x1, c_y1), (c_x2, c_y2), (0, 165, 255), -1)
    cv2.putText(bev, "EGO", (u0 - 10, v0 - car_l_px // 3), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)

    # 7. Corner Watermark
    cv2.putText(bev, "BEV MAP [TOP-DOWN]", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    cv2.putText(bev, "Range: 60m x 60m", (10, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 130, 140), 1)

    return bev


# Module-level aliases to satisfy interface contracts
generate_bev_map = render_bev


class BEVVisualizer:
    """Wrapper class for BEV map generation fulfilling E2E test contracts."""

    def __init__(
        self,
        width: int = 600,
        height: int = 600,
        range_x: tuple[float, float] = (-30.0, 30.0),
        range_z: tuple[float, float] = (0.0, 60.0),
    ) -> None:
        self.width = width
        self.height = height
        self.range_x = range_x
        self.range_z = range_z

    def render(
        self,
        points: np.ndarray,
        calib: KittiCalib | None = None,
        labels: list[KittiObject] | None = None,
        **kwargs: Any,
    ) -> np.ndarray:
        return render_bev(
            points=points,
            calib=calib,
            labels=labels,
            width=self.width,
            height=self.height,
            range_x=self.range_x,
            range_z=self.range_z,
            **kwargs,
        )


def _fit_image_in_box(image: np.ndarray, target_w: int, target_h: int) -> np.ndarray:
    """Resize image to fit box with letterboxing preserving aspect ratio."""
    h, w = image.shape[:2]
    scale = min(target_w / max(1, w), target_h / max(1, h))
    new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)

    canvas = np.full((target_h, target_w, 3), (18, 20, 24), dtype=np.uint8)
    offset_x = (target_w - new_w) // 2
    offset_y = (target_h - new_h) // 2
    canvas[offset_y:offset_y + new_h, offset_x:offset_x + new_w] = resized
    return canvas


def render_composite_dashboard(
    camera_img: np.ndarray,
    bev_img: np.ndarray,
    metrics: dict[str, Any],
    meta: dict[str, Any],
    width: int = 1280,
    height: int = 760,
) -> np.ndarray:
    """Composite publication-grade UI dashboard with visualizers and telemetry."""
    dashboard = np.full((height, width, 3), (20, 22, 26), dtype=np.uint8)

    # 1. Header Bar (Y: 0..55)
    cv2.rectangle(dashboard, (0, 0), (width, 55), (30, 34, 40), -1)
    cv2.line(dashboard, (0, 55), (width, 55), (50, 58, 68), 1)

    title = "TOPIC C: SENSOR DEGRADATION STRESS TEST & HEALTH MONITOR"
    cv2.putText(dashboard, title, (20, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)

    subtitle = (
        f"Dataset: {meta.get('dataset', 'kitti_mini')} | "
        f"Frame: {meta.get('frame', '000001')} | "
        f"Dropout: {meta.get('dropout_pct', 0.0):.0f}% | "
        f"Range: {meta.get('range_m', 80.0):.0f}m | "
        f"Beams: {meta.get('beam_count', 64)} | "
        f"Noise: {meta.get('noise_m', 0.0):.2f}m | "
        f"Drift: Yaw {meta.get('yaw_deg', 0.0):+.1f}°, Pitch {meta.get('pitch_deg', 0.0):+.1f}°"
    )
    cv2.putText(dashboard, subtitle, (20, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (140, 200, 255), 1, cv2.LINE_AA)

    # 2. Dual Visualizers Viewport (Y: 65..550, Height: 485)
    vp_y = 65
    vp_h = 485
    vp_w = 615

    # Left: Camera View
    left_x = 15
    fitted_cam = _fit_image_in_box(camera_img, vp_w, vp_h - 25)
    dashboard[vp_y + 25:vp_y + vp_h, left_x:left_x + vp_w] = fitted_cam
    cv2.rectangle(dashboard, (left_x, vp_y + 25), (left_x + vp_w, vp_y + vp_h), (55, 62, 72), 1)
    cv2.putText(
        dashboard, "CAMERA VIEW (LIDAR DEPTH PROJECTION + 3D GT BOXES)",
        (left_x, vp_y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 220, 240), 1, cv2.LINE_AA
    )

    # Right: BEV Map
    right_x = 650
    fitted_bev = _fit_image_in_box(bev_img, vp_w, vp_h - 25)
    dashboard[vp_y + 25:vp_y + vp_h, right_x:right_x + vp_w] = fitted_bev
    cv2.rectangle(dashboard, (right_x, vp_y + 25), (right_x + vp_w, vp_y + vp_h), (55, 62, 72), 1)
    cv2.putText(
        dashboard, "BIRD'S-EYE-VIEW (BEV) MAP [TOP-DOWN GROUND PLANE]",
        (right_x, vp_y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 220, 240), 1, cv2.LINE_AA
    )

    # 3. Bottom Telemetry Cards (Y: 565..745, Height: 180)
    card_y = 565
    card_h = 180
    cv2.rectangle(dashboard, (15, card_y), (width - 15, card_y + card_h), (25, 29, 35), -1)
    cv2.rectangle(dashboard, (15, card_y), (width - 15, card_y + card_h), (48, 55, 65), 1)

    # Card 1: Point Cloud Density (X: 30..330)
    c1_x = 35
    cv2.putText(dashboard, "POINT CLOUD DENSITY", (c1_x, card_y + 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 180, 255), 1, cv2.LINE_AA)
    n_pts = metrics.get("n_points", 0)
    n_base = meta.get("baseline_points", n_pts)
    retention_pct = (n_pts / max(1, n_base)) * 100.0
    cv2.putText(dashboard, f"Total Points: {n_pts:,} / {n_base:,}", (c1_x, card_y + 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (240, 240, 240), 1, cv2.LINE_AA)
    cv2.putText(dashboard, f"Point Retention: {retention_pct:.1f}%", (c1_x, card_y + 86),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    pts_fov = metrics.get("pts_in_fov", 0)
    fov_pct = metrics.get("pts_in_fov_pct", 0.0)
    cv2.putText(dashboard, f"In Camera FOV: {pts_fov:,} ({fov_pct:.1f}%)", (c1_x, card_y + 114),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

    # Card 2: 3D Ground Truth Returns (X: 350..660)
    c2_x = 350
    cv2.putText(dashboard, "3D GT OBJECT RETURNS", (c2_x, card_y + 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 180, 255), 1, cv2.LINE_AA)
    pts_car = metrics.get("pts_car", 0)
    pts_ped = metrics.get("pts_ped", 0)
    pts_cyc = metrics.get("pts_cyc", 0)
    starved = metrics.get("starved_objects", 0)
    cv2.putText(dashboard, f"Car Points: {pts_car:,}", (c2_x, card_y + 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1, cv2.LINE_AA)
    cv2.putText(dashboard, f"Pedestrian: {pts_ped:,}  |  Cyclist: {pts_cyc:,}", (c2_x, card_y + 86),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1, cv2.LINE_AA)
    cv2.putText(dashboard, f"Starved Objects (<=3 pts): {starved}", (c2_x, card_y + 114),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 120, 120) if starved > 0 else (180, 180, 180), 1, cv2.LINE_AA)

    # Card 3: Composite Sensor Health (X: 680..980)
    c3_x = 680
    cv2.putText(dashboard, "SENSOR HEALTH SCORE", (c3_x, card_y + 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 180, 255), 1, cv2.LINE_AA)
    score = float(metrics.get("health_score", 0.0))
    status = str(metrics.get("health_status", classify_health_score(score)))

    # Status color mapping (BGR)
    status_bg = {
        "HEALTHY": (40, 167, 69),      # Green
        "DEGRADED": (7, 193, 255),     # Amber
        "CRITICAL": (20, 126, 253),    # Dark Orange
        "FAILURE": (69, 53, 220),      # Crimson Red
    }.get(status, (100, 100, 100))

    cv2.putText(dashboard, f"S_health: {score:.1f} / 100.0", (c3_x, card_y + 64),
                cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255, 255, 255), 2, cv2.LINE_AA)

    # Status badge box
    badge_w = 160
    badge_h = 32
    cv2.rectangle(dashboard, (c3_x, card_y + 84), (c3_x + badge_w, card_y + 84 + badge_h), status_bg, -1)
    cv2.putText(dashboard, status, (c3_x + 18, card_y + 106),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)

    # Card 4: Telemetry & Execution Runtime (X: 990..1260)
    c4_x = 990
    cv2.putText(dashboard, "SYSTEM RUNTIME", (c4_x, card_y + 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 180, 255), 1, cv2.LINE_AA)
    latency_ms = meta.get("latency_ms", 25.0)
    fps = 1000.0 / max(1e-3, latency_ms)
    cv2.putText(dashboard, f"Pipeline Latency: {latency_ms:.1f} ms", (c4_x, card_y + 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (240, 240, 240), 1, cv2.LINE_AA)
    cv2.putText(dashboard, f"Frame Rate: {fps:.1f} FPS (CPU)", (c4_x, card_y + 86),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 255, 200), 1, cv2.LINE_AA)
    cv2.putText(dashboard, "Hardware: Pure CPU (.venv)", (c4_x, card_y + 114),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (150, 160, 175), 1, cv2.LINE_AA)

    return dashboard


if HAS_TKINTER:
    class DemoApp:
        """Tkinter Interactive GUI Desktop Application for LiDAR Degradation Analysis."""

        def __init__(self, root: tk.Tk, initial_dataset: str = "kitti_mini", initial_frame: str | None = None) -> None:
            self.root = root
            self.root.title("Topic C: LiDAR-Camera Sensor Degradation Stress Test & Health Monitor")
            self.root.geometry("1420x880")
            self.root.minsize(1100, 750)
            self.root.configure(bg="#1E2126")

            self.dataset_name = initial_dataset
            self.frame_id = initial_frame
            self.current_frame_data: dict[str, Any] | None = None
            self.baseline_point_count: int = 0
            self._update_job: str | None = None

            # Control Variables
            self.var_dataset = tk.StringVar(value=self.dataset_name)
            self.var_frame = tk.StringVar()
            self.var_dropout = tk.DoubleVar(value=0.0)
            self.var_range = tk.DoubleVar(value=80.0)
            self.var_beams = tk.IntVar(value=64)
            self.var_noise = tk.DoubleVar(value=0.00)
            self.var_yaw = tk.DoubleVar(value=0.0)
            self.var_pitch = tk.DoubleVar(value=0.0)
            self.var_roll = tk.DoubleVar(value=0.0)

            self._setup_styles()
            self._build_ui()
            self._populate_frames()
            self._load_current_frame()

        def _setup_styles(self) -> None:
            style = ttk.Style()
            style.theme_use("clam")
            style.configure(".", background="#1E2126", foreground="#E0E0E0", font=("Segoe UI", 9))
            style.configure("TFrame", background="#1E2126")
            style.configure("TLabel", background="#1E2126", foreground="#D0D0D0", font=("Segoe UI", 9))
            style.configure("Header.TLabel", font=("Segoe UI", 10, "bold"), foreground="#64B5F6")
            style.configure("TCombobox", fieldbackground="#2A2F38", background="#2A2F38", foreground="#FFFFFF")
            style.configure("TButton", background="#2979FF", foreground="#FFFFFF", font=("Segoe UI", 9, "bold"))
            style.map("TButton", background=[("active", "#1E88E5"), ("pressed", "#1565C0")])
            style.configure("Reset.TButton", background="#546E7A", foreground="#FFFFFF")
            style.map("Reset.TButton", background=[("active", "#607D8B"), ("pressed", "#455A64")])

        def _build_ui(self) -> None:
            # Main container with two columns
            self.main_box = ttk.Frame(self.root)
            self.main_box.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

            # Left Sidebar Controls (Width: ~320px)
            self.sidebar = ttk.Frame(self.main_box, width=320)
            self.sidebar.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

            # Right Viewports & Telemetry Container
            self.content_area = ttk.Frame(self.main_box)
            self.content_area.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

            self._build_sidebar_controls()
            self._build_viewports()
            self._build_telemetry_bar()

        def _build_sidebar_controls(self) -> None:
            # 1. Dataset & Frame Selector
            grp_data = ttk.LabelFrame(self.sidebar, text="Dataset & Frame", padding=8)
            grp_data.pack(fill=tk.X, pady=(0, 10))

            ttk.Label(grp_data, text="Dataset:").grid(row=0, column=0, sticky=tk.W, pady=3)
            self.combo_dataset = ttk.Combobox(
                grp_data, textvariable=self.var_dataset,
                values=list(DATASET_OPTIONS.keys()), state="readonly", width=18
            )
            self.combo_dataset.grid(row=0, column=1, sticky=tk.EW, pady=3)
            self.combo_dataset.bind("<<ComboboxSelected>>", self._on_dataset_change)

            ttk.Label(grp_data, text="Frame ID:").grid(row=1, column=0, sticky=tk.W, pady=3)
            self.combo_frame = ttk.Combobox(
                grp_data, textvariable=self.var_frame, state="readonly", width=18
            )
            self.combo_frame.grid(row=1, column=1, sticky=tk.EW, pady=3)
            self.combo_frame.bind("<<ComboboxSelected>>", self._on_frame_change)

            btn_box = ttk.Frame(grp_data)
            btn_box.grid(row=2, column=0, columnspan=2, pady=5)
            ttk.Button(btn_box, text="< Prev", width=8, command=self._prev_frame).pack(side=tk.LEFT, padx=3)
            ttk.Button(btn_box, text="Next >", width=8, command=self._next_frame).pack(side=tk.LEFT, padx=3)

            # 2. Degradation Sliders
            grp_deg = ttk.LabelFrame(self.sidebar, text="Degradation Controls", padding=8)
            grp_deg.pack(fill=tk.X, pady=(0, 10))

            self.lbl_dropout = ttk.Label(grp_deg, text="Random Dropout: 0%")
            self.lbl_dropout.pack(anchor=tk.W, pady=(2, 0))
            self.scale_dropout = tk.Scale(
                grp_deg, from_=0, to=80, orient=tk.HORIZONTAL, variable=self.var_dropout,
                command=self._on_slider_moved, bg="#2A2F38", fg="#E0E0E0", highlightthickness=0
            )
            self.scale_dropout.pack(fill=tk.X, pady=(0, 6))

            self.lbl_range = ttk.Label(grp_deg, text="Range Cutoff: 80m")
            self.lbl_range.pack(anchor=tk.W, pady=(2, 0))
            self.scale_range = tk.Scale(
                grp_deg, from_=10, to=80, orient=tk.HORIZONTAL, variable=self.var_range,
                command=self._on_slider_moved, bg="#2A2F38", fg="#E0E0E0", highlightthickness=0
            )
            self.scale_range.pack(fill=tk.X, pady=(0, 6))

            self.lbl_beams = ttk.Label(grp_deg, text="LiDAR Beams: 64")
            self.lbl_beams.pack(anchor=tk.W, pady=(2, 0))
            combo_beams = ttk.Combobox(
                grp_deg, textvariable=self.var_beams,
                values=[64, 32, 16, 8, 4], state="readonly", width=12
            )
            combo_beams.pack(anchor=tk.W, pady=(0, 6))
            combo_beams.bind("<<ComboboxSelected>>", lambda e: self._on_slider_moved(None))

            self.lbl_noise = ttk.Label(grp_deg, text="Gaussian Noise: 0.00m")
            self.lbl_noise.pack(anchor=tk.W, pady=(2, 0))
            self.scale_noise = tk.Scale(
                grp_deg, from_=0.0, to=0.20, resolution=0.01, orient=tk.HORIZONTAL, variable=self.var_noise,
                command=self._on_slider_moved, bg="#2A2F38", fg="#E0E0E0", highlightthickness=0
            )
            self.scale_noise.pack(fill=tk.X, pady=(0, 6))

            self.lbl_yaw = ttk.Label(grp_deg, text="Yaw Drift: +0.0°")
            self.lbl_yaw.pack(anchor=tk.W, pady=(2, 0))
            self.scale_yaw = tk.Scale(
                grp_deg, from_=-5.0, to=5.0, resolution=0.1, orient=tk.HORIZONTAL, variable=self.var_yaw,
                command=self._on_slider_moved, bg="#2A2F38", fg="#E0E0E0", highlightthickness=0
            )
            self.scale_yaw.pack(fill=tk.X, pady=(0, 6))

            self.lbl_pitch = ttk.Label(grp_deg, text="Pitch Drift: +0.0°")
            self.lbl_pitch.pack(anchor=tk.W, pady=(2, 0))
            self.scale_pitch = tk.Scale(
                grp_deg, from_=-2.0, to=2.0, resolution=0.1, orient=tk.HORIZONTAL, variable=self.var_pitch,
                command=self._on_slider_moved, bg="#2A2F38", fg="#E0E0E0", highlightthickness=0
            )
            self.scale_pitch.pack(fill=tk.X, pady=(0, 6))

            self.lbl_roll = ttk.Label(grp_deg, text="Roll Drift: +0.0°")
            self.lbl_roll.pack(anchor=tk.W, pady=(2, 0))
            self.scale_roll = tk.Scale(
                grp_deg, from_=-2.0, to=2.0, resolution=0.1, orient=tk.HORIZONTAL, variable=self.var_roll,
                command=self._on_slider_moved, bg="#2A2F38", fg="#E0E0E0", highlightthickness=0
            )
            self.scale_roll.pack(fill=tk.X, pady=(0, 10))

            # 3. Actions
            ttk.Button(grp_deg, text="Reset Baseline", style="Reset.TButton", command=self._reset_degradations).pack(fill=tk.X, pady=3)
            ttk.Button(grp_deg, text="Export Snapshot", command=self._export_snapshot_dialog).pack(fill=tk.X, pady=3)

        def _build_viewports(self) -> None:
            # Dual viewports side-by-side
            self.viewport_box = ttk.Frame(self.content_area)
            self.viewport_box.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

            # Left Viewport: Camera Projection
            self.frame_cam_view = ttk.LabelFrame(self.viewport_box, text="Camera Projection & 3D GT Boxes", padding=4)
            self.frame_cam_view.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
            self.lbl_cam_image = ttk.Label(self.frame_cam_view)
            self.lbl_cam_image.pack(fill=tk.BOTH, expand=True)

            # Right Viewport: BEV Map
            self.frame_bev_view = ttk.LabelFrame(self.viewport_box, text="Bird's-Eye-View (BEV) Map", padding=4)
            self.frame_bev_view.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
            self.lbl_bev_image = ttk.Label(self.frame_bev_view)
            self.lbl_bev_image.pack(fill=tk.BOTH, expand=True)

        def _build_telemetry_bar(self) -> None:
            self.telemetry_box = ttk.LabelFrame(self.content_area, text="Live Sensor Health & Telemetry", padding=8)
            self.telemetry_box.pack(fill=tk.X)

            # 4 columns of metrics
            self.lbl_tel_points = ttk.Label(self.telemetry_box, text="Total Points: 0\nIn FOV: 0 (0.0%)", font=("Segoe UI", 9, "bold"))
            self.lbl_tel_points.grid(row=0, column=0, sticky=tk.W, padx=15, pady=4)

            self.lbl_tel_objects = ttk.Label(self.telemetry_box, text="Car: 0 pts\nPed: 0  |  Cyc: 0", font=("Segoe UI", 9))
            self.lbl_tel_objects.grid(row=0, column=1, sticky=tk.W, padx=15, pady=4)

            self.lbl_tel_health = ttk.Label(self.telemetry_box, text="Health Score: 100.0\nStatus: HEALTHY", font=("Segoe UI", 10, "bold"), foreground="#4CAF50")
            self.lbl_tel_health.grid(row=0, column=2, sticky=tk.W, padx=15, pady=4)

            self.lbl_tel_perf = ttk.Label(self.telemetry_box, text="Latency: 0.0 ms\nFPS: 0.0 (CPU)", font=("Segoe UI", 9))
            self.lbl_tel_perf.grid(row=0, column=3, sticky=tk.W, padx=15, pady=4)

        def _populate_frames(self) -> None:
            root_path = REPO_ROOT / DATASET_OPTIONS[self.dataset_name]
            frames = list_frames(root_path)
            if not frames:
                frames = ["000000"]
            self.combo_frame["values"] = frames
            if self.frame_id in frames:
                self.var_frame.set(self.frame_id)
            else:
                self.var_frame.set(frames[0])
                self.frame_id = frames[0]

        def _load_current_frame(self) -> None:
            root_path = REPO_ROOT / DATASET_OPTIONS[self.dataset_name]
            fid = self.var_frame.get()
            self.frame_id = fid
            kwargs = {"use_ego_motion": True} if dataset_type(root_path) == "nuscenes" else {}
            self.current_frame_data = load_frame(root_path, fid, **kwargs)
            self.baseline_point_count = len(self.current_frame_data["points"])
            self._render_pipeline()

        def _on_dataset_change(self, event: Any = None) -> None:
            self.dataset_name = self.var_dataset.get()
            self._populate_frames()
            self._load_current_frame()

        def _on_frame_change(self, event: Any = None) -> None:
            self._load_current_frame()

        def _prev_frame(self) -> None:
            values = list(self.combo_frame["values"])
            curr = self.var_frame.get()
            if curr in values:
                idx = values.index(curr)
                if idx > 0:
                    self.var_frame.set(values[idx - 1])
                    self._load_current_frame()

        def _next_frame(self) -> None:
            values = list(self.combo_frame["values"])
            curr = self.var_frame.get()
            if curr in values:
                idx = values.index(curr)
                if idx < len(values) - 1:
                    self.var_frame.set(values[idx + 1])
                    self._load_current_frame()

        def _on_slider_moved(self, val: Any) -> None:
            # Update labels
            self.lbl_dropout.config(text=f"Random Dropout: {self.var_dropout.get():.0f}%")
            self.lbl_range.config(text=f"Range Cutoff: {self.var_range.get():.0f}m")
            self.lbl_beams.config(text=f"LiDAR Beams: {self.var_beams.get()}")
            self.lbl_noise.config(text=f"Gaussian Noise: {self.var_noise.get():.2f}m")
            self.lbl_yaw.config(text=f"Yaw Drift: {self.var_yaw.get():+.1f}°")
            self.lbl_pitch.config(text=f"Pitch Drift: {self.var_pitch.get():+.1f}°")
            self.lbl_roll.config(text=f"Roll Drift: {self.var_roll.get():+.1f}°")

            # Debounce rendering
            if self._update_job:
                self.root.after_cancel(self._update_job)
            self._update_job = self.root.after(25, self._render_pipeline)

        def _reset_degradations(self) -> None:
            self.var_dropout.set(0.0)
            self.var_range.set(80.0)
            self.var_beams.set(64)
            self.var_noise.set(0.0)
            self.var_yaw.set(0.0)
            self.var_pitch.set(0.0)
            self.var_roll.set(0.0)
            self._on_slider_moved(None)

        def _render_pipeline(self) -> None:
            self._update_job = None
            if self.current_frame_data is None:
                return

            t0 = time.perf_counter()

            pts_raw = self.current_frame_data["points"]
            calib_raw = self.current_frame_data["calib"]
            image_raw = self.current_frame_data["image"]
            labels = self.current_frame_data["labels"]

            # Degradation application
            pts_deg, calib_deg = apply_degradations(
                pts_raw, calib_raw,
                dropout=self.var_dropout.get() / 100.0,
                range_cutoff=self.var_range.get(),
                beam_count=self.var_beams.get(),
                noise_sigma=self.var_noise.get(),
                yaw_deg=self.var_yaw.get(),
                pitch_deg=self.var_pitch.get(),
                roll_deg=self.var_roll.get(),
                seed=42,
            )

            # Metrics
            metrics = compute_frame_metrics(
                pts_deg, calib_deg, labels, image_raw.shape,
                baseline_points=pts_raw
            )

            # Renderings
            cam_vis = render_camera_view(image_raw, pts_deg, calib_deg, labels)
            bev_vis = render_bev(pts_deg, calib_deg, labels, width=540, height=540)

            t1 = time.perf_counter()
            latency_ms = (t1 - t0) * 1000.0
            fps = 1000.0 / max(1e-3, latency_ms)

            # Display on Tkinter Labels
            cam_fit = _fit_image_in_box(cam_vis, 560, 420)
            bev_fit = _fit_image_in_box(bev_vis, 500, 420)

            cam_pil = Image.fromarray(cv2.cvtColor(cam_fit, cv2.COLOR_BGR2RGB))
            bev_pil = Image.fromarray(cv2.cvtColor(bev_fit, cv2.COLOR_BGR2RGB))

            self.tk_cam_img = ImageTk.PhotoImage(cam_pil)
            self.tk_bev_img = ImageTk.PhotoImage(bev_pil)

            self.lbl_cam_image.configure(image=self.tk_cam_img)
            self.lbl_bev_image.configure(image=self.tk_bev_img)

            # Update Telemetry Labels
            n_rem = metrics["n_points"]
            ret_pct = (n_rem / max(1, self.baseline_point_count)) * 100.0
            self.lbl_tel_points.config(
                text=f"Total Points: {n_rem:,} / {self.baseline_point_count:,} ({ret_pct:.1f}%)\n"
                     f"In FOV: {metrics['pts_in_fov']:,} ({metrics['pts_in_fov_pct']:.1f}%)"
            )
            self.lbl_tel_objects.config(
                text=f"Car: {metrics['pts_car']:,} pts  |  Starved: {metrics['starved_objects']}\n"
                     f"Pedestrian: {metrics['pts_ped']:,}  |  Cyclist: {metrics['pts_cyc']:,}"
            )
            score = metrics["health_score"]
            status = metrics["health_status"]
            status_colors = {
                "HEALTHY": "#4CAF50",
                "DEGRADED": "#FFB300",
                "CRITICAL": "#FB8C00",
                "FAILURE": "#E53935",
            }
            self.lbl_tel_health.config(
                text=f"Health Score: {score:.1f} / 100.0\nStatus: {status}",
                foreground=status_colors.get(status, "#FFFFFF")
            )
            self.lbl_tel_perf.config(
                text=f"Latency: {latency_ms:.1f} ms\nThroughput: {fps:.1f} FPS (CPU)"
            )

        def _export_snapshot_dialog(self) -> None:
            target = filedialog.asksaveasfilename(
                title="Save UI Snapshot",
                defaultextension=".png",
                filetypes=[("PNG Image", "*.png"), ("All Files", "*.*")],
                initialdir=str(DEFAULT_SNAPSHOT_PATH.parent),
                initialfile="demo_gui_snapshot.png",
            )
            if target:
                self._save_snapshot_to(Path(target))
                messagebox.showinfo("Export Successful", f"Snapshot exported successfully to:\n{target}")

        def _save_snapshot_to(self, out_path: Path) -> None:
            if self.current_frame_data is None:
                return
            pts_raw = self.current_frame_data["points"]
            calib_raw = self.current_frame_data["calib"]
            image_raw = self.current_frame_data["image"]
            labels = self.current_frame_data["labels"]

            pts_deg, calib_deg = apply_degradations(
                pts_raw, calib_raw,
                dropout=self.var_dropout.get() / 100.0,
                range_cutoff=self.var_range.get(),
                beam_count=self.var_beams.get(),
                noise_sigma=self.var_noise.get(),
                yaw_deg=self.var_yaw.get(),
                pitch_deg=self.var_pitch.get(),
                roll_deg=self.var_roll.get(),
            )
            metrics = compute_frame_metrics(pts_deg, calib_deg, labels, image_raw.shape, baseline_points=pts_raw)
            cam_vis = render_camera_view(image_raw, pts_deg, calib_deg, labels)
            bev_vis = render_bev(pts_deg, calib_deg, labels)

            meta = {
                "dataset": self.dataset_name,
                "frame": self.frame_id,
                "dropout_pct": self.var_dropout.get(),
                "range_m": self.var_range.get(),
                "beam_count": self.var_beams.get(),
                "noise_m": self.var_noise.get(),
                "yaw_deg": self.var_yaw.get(),
                "pitch_deg": self.var_pitch.get(),
                "baseline_points": self.baseline_point_count,
                "latency_ms": 22.0,
            }
            dashboard = render_composite_dashboard(cam_vis, bev_vis, metrics, meta)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out_path), dashboard)


def build_parser() -> argparse.ArgumentParser:
    """Build CLI argument parser for interactive and headless operation."""
    parser = argparse.ArgumentParser(
        description="Interactive CPU Demo & Headless Snapshot Engine for LiDAR Stress Testing",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--headless", action="store_true",
                        help="Run in headless execution mode without launching GUI window")
    parser.add_argument("--export-demo", action="store_true",
                        help="Export snapshot to default artifact results/figures/demo_gui_snapshot.png")
    parser.add_argument("--export-snapshot", type=str, default=None,
                        help="Explicit destination path for composite GUI snapshot PNG")
    parser.add_argument("--data-root", type=str, default="data/kitti_mini",
                        help="Root directory for dataset")
    parser.add_argument("--dataset", type=str, default=None,
                        choices=["synthetic", "kitti_mini", "nuscenes_mini_subset"],
                        help="Convenience alias for dataset name")
    parser.add_argument("--frame", type=str, default=None,
                        help="Frame ID to load (defaults to first available frame in dataset)")
    parser.add_argument("--dropout", type=float, default=0.0,
                        help="Random dropout ratio [0.0, 1.0]")
    parser.add_argument("--range-cutoff", type=float, default=80.0,
                        help="Radial range cutoff in meters [10.0, 80.0]")
    parser.add_argument("--beam-count", type=int, default=64,
                        choices=[64, 32, 16, 8, 4], help="Simulated beam count")
    parser.add_argument("--noise-sigma", type=float, default=0.0,
                        help="Gaussian coordinate noise std dev in meters")
    parser.add_argument("--yaw-deg", type=float, default=0.0,
                        help="Extrinsic calibration yaw drift in degrees")
    parser.add_argument("--pitch-deg", type=float, default=0.0,
                        help="Extrinsic calibration pitch drift in degrees")
    parser.add_argument("--roll-deg", type=float, default=0.0,
                        help="Extrinsic calibration roll drift in degrees")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed for perturbations")
    return parser


def run_headless(args: argparse.Namespace) -> int:
    """Execute headless frame processing and export composite snapshot artifact."""
    t_start = time.perf_counter()

    # Determine dataset root path
    if args.dataset:
        data_root_str = DATASET_OPTIONS.get(args.dataset, f"data/{args.dataset}")
    else:
        data_root_str = args.data_root

    data_root = Path(data_root_str)
    if not data_root.is_absolute():
        data_root = REPO_ROOT / data_root

    if not data_root.exists():
        print(f"Error: Dataset path does not exist: {data_root}", file=sys.stderr)
        return 1

    # Determine frame ID
    valid_frames = list_frames(data_root)
    if not valid_frames:
        print(f"Error: No valid frames found in dataset: {data_root}", file=sys.stderr)
        return 1

    if args.frame:
        frame_id = args.frame
        if frame_id not in valid_frames:
            print(f"Error: Frame '{frame_id}' not found in dataset '{data_root}'", file=sys.stderr)
            return 1
    else:
        frame_id = valid_frames[0]

    # Load frame
    kwargs = {"use_ego_motion": True} if dataset_type(data_root) == "nuscenes" else {}
    try:
        frame_data = load_frame(data_root, frame_id, **kwargs)
    except Exception as exc:
        print(f"Error loading frame '{frame_id}': {exc}", file=sys.stderr)
        return 1

    pts_raw = frame_data["points"]
    calib_raw = frame_data["calib"]
    image_raw = frame_data["image"]
    labels = frame_data["labels"]
    n_baseline = len(pts_raw)

    # Apply degradations
    pts_deg, calib_deg = apply_degradations(
        points=pts_raw,
        calib=calib_raw,
        dropout=args.dropout,
        range_cutoff=args.range_cutoff,
        beam_count=args.beam_count,
        noise_sigma=args.noise_sigma,
        yaw_deg=args.yaw_deg,
        pitch_deg=args.pitch_deg,
        roll_deg=args.roll_deg,
        seed=args.seed,
    )

    # Compute non-DL metrics
    metrics = compute_frame_metrics(
        points=pts_deg,
        calib=calib_deg,
        labels=labels,
        image_shape=image_raw.shape,
        baseline_points=pts_raw,
    )

    # Render Visualizers
    cam_vis = render_camera_view(image_raw, pts_deg, calib_deg, labels)
    bev_vis = render_bev(pts_deg, calib_deg, labels)

    t_end = time.perf_counter()
    latency_ms = (t_end - t_start) * 1000.0

    # Composite Dashboard Snapshot
    meta = {
        "dataset": data_root.name,
        "frame": frame_id,
        "dropout_pct": args.dropout * 100.0,
        "range_m": args.range_cutoff,
        "beam_count": args.beam_count,
        "noise_m": args.noise_sigma,
        "yaw_deg": args.yaw_deg,
        "pitch_deg": args.pitch_deg,
        "baseline_points": n_baseline,
        "latency_ms": latency_ms,
    }
    dashboard = render_composite_dashboard(cam_vis, bev_vis, metrics, meta)

    # Resolve destination path
    if args.export_snapshot:
        export_path = Path(args.export_snapshot)
    else:
        export_path = DEFAULT_SNAPSHOT_PATH

    if not export_path.is_absolute():
        export_path = REPO_ROOT / export_path

    export_path.parent.mkdir(parents=True, exist_ok=True)
    success = cv2.imwrite(str(export_path), dashboard)
    if not success:
        print(f"Error: Failed to write snapshot to {export_path}", file=sys.stderr)
        return 1

    print(f"[OK] Headless export completed: {export_path} ({dashboard.shape[1]}x{dashboard.shape[0]}, {export_path.stat().st_size:,} bytes)")
    print(f"     Metrics: Points {metrics['n_points']:,}/{n_baseline:,} | "
          f"FOV {metrics['pts_in_fov_pct']:.1f}% | "
          f"Health {metrics['health_score']:.1f} ({metrics['health_status']}) | "
          f"Latency {latency_ms:.1f}ms")
    return 0


def main() -> None:
    """CLI entry point for Interactive UI and Headless Execution."""
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")

    parser = build_parser()
    args = parser.parse_args()

    # Headless Mode
    if args.headless:
        code = run_headless(args)
        sys.exit(code)

    # Interactive Desktop Mode
    if not HAS_TKINTER:
        print("Error: Tkinter/Pillow is not available in current environment. Use --headless for snapshot export.", file=sys.stderr)
        sys.exit(1)

    initial_dataset = args.dataset or ("kitti_mini" if "kitti_mini" in args.data_root else "synthetic")
    root = tk.Tk()
    app = DemoApp(root, initial_dataset=initial_dataset, initial_frame=args.frame)
    root.mainloop()


if __name__ == "__main__":
    main()
