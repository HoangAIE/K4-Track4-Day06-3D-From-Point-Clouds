"""Model-free Quantitative Metrics for LiDAR Sensor Degradation Analysis.

Topic C: Non-DL metrics evaluating LiDAR point cloud health, 3D ground truth box
containment (Car, Pedestrian, Cyclist), camera FOV retention, and composite
Sensor Health Score.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from starter.kitti_io import KittiCalib, KittiObject
from starter.projection import project_velo_to_image, velo_to_cam


def points_in_box3d(points_cam: np.ndarray, obj: KittiObject) -> np.ndarray:
    """Vectorized testing of points inside 3D oriented bounding box in camera frame.

    Args:
        points_cam: (N, 3) or (N, >=3) coordinates in rectified camera frame
                    (x: right, y: down, z: forward).
        obj: KittiObject containing dimensions (h, w, l), location (x, y, z), and rotation_y.

    Returns:
        Boolean mask array of shape (N,) indicating whether each point is inside the box.
    """
    pts = np.asarray(points_cam)
    if pts.size == 0 or len(pts) == 0:
        return np.zeros(0, dtype=bool)

    was_1d = pts.ndim == 1
    if was_1d:
        pts = pts.reshape(1, -1)

    pts_3d = pts[:, :3]
    finite = np.isfinite(pts_3d).all(axis=1)

    h, w, l = float(obj.dimensions[0]), float(obj.dimensions[1]), float(obj.dimensions[2])
    c = float(np.cos(obj.rotation_y))
    s = float(np.sin(obj.rotation_y))
    R = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)

    # Relative to box bottom-center location, masked to avoid NaN arithmetic
    rel = np.where(finite[:, None], pts_3d - obj.location, 0.0)
    # Canonical box coordinate frame: P_canon = (P_cam - loc) @ R
    p_canon = rel @ R

    in_box = (
        finite
        & (p_canon[:, 0] >= -l / 2.0)
        & (p_canon[:, 0] <= l / 2.0)
        & (p_canon[:, 1] >= -h)
        & (p_canon[:, 1] <= 0.0)
        & (p_canon[:, 2] >= -w / 2.0)
        & (p_canon[:, 2] <= w / 2.0)
    )
    return in_box[0] if was_1d else in_box


def compute_fov_ratio(
    points: np.ndarray,
    calib: KittiCalib,
    image_shape: tuple[int, ...],
    as_percentage: bool = True,
) -> float:
    """Compute ratio/percentage of valid LiDAR points inside camera image FOV.

    Args:
        points: (N, >=3) LiDAR points in Velodyne frame.
        calib: Calibration object.
        image_shape: Image dimensions (H, W, ...).
        as_percentage: If True, return in [0.0, 100.0], else [0.0, 1.0].

    Returns:
        Float value representing FOV coverage.
    """
    if len(points) == 0:
        return 0.0
    _, _, mask = project_velo_to_image(points, calib, image_shape)
    ratio = float(mask.sum() / len(points))
    return ratio * 100.0 if as_percentage else ratio


def classify_health_score(score: float) -> str:
    """Classify composite Sensor Health Score into operational safety status bands.

    - HEALTHY: >= 75.0 (Optimal sensor performance, full object visibility)
    - DEGRADED: 60.0 <= score < 75.0 (Moderate attenuation, warning state)
    - CRITICAL: 40.0 <= score < 60.0 (Severe degradation, high risk of VRU starvation)
    - FAILURE: < 40.0 (Sensor blackout / blindness, failsafe required)
    """
    if score >= 75.0:
        return "HEALTHY"
    elif score >= 60.0:
        return "DEGRADED"
    elif score >= 40.0:
        return "CRITICAL"
    else:
        return "FAILURE"


def compute_sensor_health_score(
    points: np.ndarray,
    baseline_points: np.ndarray | None = None,
    points_in_fov: int | None = None,
    baseline_fov: int | None = None,
    calib: KittiCalib | None = None,
    image_shape: tuple[int, ...] | None = None,
    n_azimuth_bins: int = 36,
) -> float:
    """Compute composite Sensor Health Score S in [0.0, 100.0].

    Combines:
    1. Point count retention (35%): N / N_0
    2. Radial range 95th percentile (25%): R_p95 / R_0,p95
    3. Azimuth coverage continuity (20%): 1 - N_empty_az / 36
    4. Camera FOV retention (20%): N_fov / N_fov,0

    Args:
        points: (N, >=3) LiDAR points in current frame.
        baseline_points: Optional unperturbed clean reference points.
        points_in_fov: Precomputed number of points in FOV (optional).
        baseline_fov: Baseline number of points in FOV (optional).
        calib: Calibration object for on-the-fly FOV calculation (optional).
        image_shape: Image shape for FOV calculation (optional).
        n_azimuth_bins: Number of azimuth histogram bins (default 36).

    Returns:
        Composite health score in range [0.0, 100.0].
    """
    if len(points) == 0:
        return 0.0

    finite = np.isfinite(points[:, :3]).all(axis=1)
    p_clean = points[finite]
    if len(p_clean) == 0:
        return 0.0

    n_valid = len(p_clean)
    rng = np.linalg.norm(p_clean[:, :2], axis=1)
    rp95 = float(np.percentile(rng, 95)) if len(rng) > 0 else 0.0

    az = np.degrees(np.arctan2(p_clean[:, 1], p_clean[:, 0]))
    az_hist, _ = np.histogram(az, bins=n_azimuth_bins, range=(-180.0, 180.0))
    n_empty_az = int((az_hist == 0).sum())

    # Calculate points in FOV if calib is provided and points_in_fov not given
    n_fov = points_in_fov
    if n_fov is None and calib is not None and image_shape is not None:
        _, _, mask = project_velo_to_image(p_clean, calib, image_shape)
        n_fov = int(mask.sum())

    # Establish reference benchmarks
    if baseline_points is not None and len(baseline_points) > 0:
        base_finite = np.isfinite(baseline_points[:, :3]).all(axis=1)
        base_clean = baseline_points[base_finite]
        n_ref = max(1.0, float(len(base_clean)))
        base_rng = np.linalg.norm(base_clean[:, :2], axis=1)
        rp95_ref = max(1.0, float(np.percentile(base_rng, 95))) if len(base_rng) > 0 else 50.0
        fov_ref = float(baseline_fov) if baseline_fov else max(1.0, n_ref * 0.25)
    else:
        n_ref = 125000.0  # Reference count for HDL-64E / full LiDAR sweep
        rp95_ref = 50.0   # Reference 95th percentile range in meters
        fov_ref = 25000.0 # Reference FOV point count for front camera

    s_cnt = min(1.0, n_valid / n_ref)
    s_rng = min(1.0, rp95 / rp95_ref)
    s_az = max(0.0, 1.0 - (n_empty_az / float(n_azimuth_bins)))
    s_fov = min(1.0, (n_fov / fov_ref)) if n_fov is not None else s_cnt

    score = 100.0 * (0.35 * s_cnt + 0.25 * s_rng + 0.20 * s_az + 0.20 * s_fov)
    return float(np.clip(score, 0.0, 100.0))


def compute_frame_metrics(
    points: np.ndarray,
    calib: KittiCalib,
    labels: list[KittiObject],
    image_shape: tuple[int, ...],
    baseline_points: np.ndarray | None = None,
) -> dict[str, Any]:
    """Compute comprehensive non-DL quantitative metrics for a frame.

    Args:
        points: (N, >=3) LiDAR points.
        calib: Calibration parameters.
        labels: List of annotated 3D objects.
        image_shape: Image shape (H, W, ...).
        baseline_points: Optional baseline point cloud for relative comparison.

    Returns:
        Dictionary containing:
        - n_points: Total points
        - pts_in_fov: Count of points inside camera frustum
        - pts_in_fov_pct: Percentage of points inside camera frustum [0.0, 100.0]
        - pts_car: Count of points inside Car/Vehicle 3D GT boxes
        - pts_ped: Count of points inside Pedestrian 3D GT boxes
        - pts_cyc: Count of points inside Cyclist/Bicycle 3D GT boxes
        - starved_objects: Count of objects with <= 3 points
        - health_score: Composite Sensor Health Score [0.0, 100.0]
        - health_status: Categorical status (HEALTHY, DEGRADED, CRITICAL, FAILURE)
        - invalid_ratio: Percentage of non-finite points
    """
    n_total = int(len(points))
    if n_total == 0:
        return {
            "n_points": 0,
            "pts_in_fov": 0,
            "pts_in_fov_pct": 0.0,
            "pts_car": 0,
            "pts_ped": 0,
            "pts_cyc": 0,
            "starved_objects": len(labels),
            "health_score": 0.0,
            "health_status": "FAILURE",
            "invalid_ratio": 1.0,
        }

    finite_mask = np.isfinite(points[:, :3]).all(axis=1)
    invalid_ratio = float(1.0 - finite_mask.mean())

    # Project to image
    uv, depth, fov_mask = project_velo_to_image(points, calib, image_shape)
    pts_in_fov = int(fov_mask.sum())
    pts_in_fov_pct = float(pts_in_fov / n_total * 100.0) if n_total > 0 else 0.0

    # Camera frame coordinates for 3D box containment
    pts_cam = velo_to_cam(points[:, :3], calib)

    pts_car = 0
    pts_ped = 0
    pts_cyc = 0
    starved_objects = 0

    for obj in labels:
        obj_type = obj.type.lower()
        in_box = points_in_box3d(pts_cam, obj)
        pts_count = int(in_box.sum())

        if pts_count <= 3:
            starved_objects += 1

        if obj_type in ("car", "van", "truck") or "car" in obj_type:
            pts_car += pts_count
        elif "pedestrian" in obj_type or obj_type == "person":
            pts_ped += pts_count
        elif "cyclist" in obj_type or "cycl" in obj_type or "bicycle" in obj_type or "motorcycle" in obj_type:
            pts_cyc += pts_count

    health_score = compute_sensor_health_score(
        points=points,
        baseline_points=baseline_points,
        points_in_fov=pts_in_fov,
        calib=calib,
        image_shape=image_shape,
    )
    health_status = classify_health_score(health_score)

    return {
        "n_points": n_total,
        "pts_in_fov": pts_in_fov,
        "pts_in_fov_pct": pts_in_fov_pct,
        "pts_car": pts_car,
        "pts_ped": pts_ped,
        "pts_cyc": pts_cyc,
        "starved_objects": starved_objects,
        "health_score": health_score,
        "health_status": health_status,
        "invalid_ratio": invalid_ratio,
    }
