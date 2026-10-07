"""LiDAR Sensor Degradation Stress Testing & Benchmark Pipeline.

Topic C: Non-DL quantitative stress testing across 5 degradation modes:
1. Random Dropout (keep_ratio: 1.0, 0.8, 0.6, 0.4, 0.2)
2. Range Dropout (max_range_m: 80, 50, 30, 20, 10)
3. Beam Dropout (keep_every: 1, 2, 4, 8, 16)
4. Gaussian Noise (sigma_m: 0.0, 0.02, 0.05, 0.10, 0.20)
5. Calibration Drift (yaw_deg: 0.0, 0.5, 1.0, 2.0, 3.0)

Generates:
- results/sensor_degradation_benchmark.csv
- results/figures/degradation_curves.png
- results/figures/health_score_vs_intensity.png
- results/figures/vru_starvation_analysis.png
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time
from typing import Any, Callable

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from starter.datasets import dataset_type, list_frames, load_frame
from starter.kitti_io import KittiCalib, KittiObject
from starter.perturb import (
    beam_dropout,
    gaussian_noise,
    random_dropout,
    range_dropout,
)
from starter.projection import perturb_extrinsic
from src.metrics import (
    classify_health_score,
    compute_frame_metrics,
    points_in_box3d,
)

DEFAULT_SEED = 42
seed = DEFAULT_SEED

REPO_ROOT = Path(__file__).resolve().parents[1]

DEGRADATION_CONFIGS: dict[str, dict[str, Any]] = {
    "Random Dropout": {
        "unit": "keep_ratio",
        "values": [1.0, 0.8, 0.6, 0.4, 0.2],
        "apply": lambda pts, calib, val, s: (random_dropout(pts, keep_ratio=float(val), seed=s), calib),
    },
    "Range Dropout": {
        "unit": "max_range_m",
        "values": [80.0, 50.0, 30.0, 20.0, 10.0],
        "apply": lambda pts, calib, val, s: (range_dropout(pts, max_range_m=float(val)), calib),
    },
    "Beam Dropout": {
        "unit": "keep_every",
        "values": [1, 2, 4, 8, 16],
        "apply": lambda pts, calib, val, s: (beam_dropout(pts, keep_every=int(val)), calib),
    },
    "Gaussian Noise": {
        "unit": "sigma_m",
        "values": [0.0, 0.02, 0.05, 0.10, 0.20],
        "apply": lambda pts, calib, val, s: (gaussian_noise(pts, sigma_xyz_m=float(val), seed=s), calib),
    },
    "Calibration Drift": {
        "unit": "yaw_deg",
        "values": [0.0, 0.5, 1.0, 2.0, 3.0],
        "apply": lambda pts, calib, val, s: (pts.copy(), perturb_extrinsic(calib, yaw_deg=float(val))),
    },
}


def apply_degradation(
    points: np.ndarray,
    calib: KittiCalib,
    degradation_type: str,
    parameter_value: float,
    seed: int = DEFAULT_SEED,
) -> tuple[np.ndarray, KittiCalib]:
    """Apply specified degradation mode and parameter value to point cloud and calib."""
    if degradation_type not in DEGRADATION_CONFIGS:
        raise ValueError(f"Unknown degradation type: {degradation_type}")
    apply_fn = DEGRADATION_CONFIGS[degradation_type]["apply"]
    return apply_fn(points, calib, parameter_value, seed)


def run_stress_test(
    datasets: list[tuple[str, list[str]]] | None = None,
    seed: int = DEFAULT_SEED,
    out_csv: str | Path | None = None,
    out_dir: str | Path | None = None,
) -> pd.DataFrame:
    """Run full stress test benchmark suite across datasets and degradation levels.

    Args:
        datasets: List of tuples (dataset_name, [frame_ids]).
        seed: Random seed for deterministic reproducibility.
        out_csv: Optional destination CSV path.
        out_dir: Optional destination figures directory.

    Returns:
        Pandas DataFrame containing benchmark results.
    """
    if datasets is None:
        datasets = [
            ("synthetic", ["000000", "000001", "000002", "000003", "000004"]),
            ("kitti_mini", ["000001", "000011", "000021", "000031", "000041"]),
            ("nuscenes_mini_subset", ["scene-0103_010", "scene-0103_020", "scene-1094_010", "scene-1094_020"]),
        ]

    records: list[dict[str, Any]] = []

    for ds_name, frame_list in datasets:
        ds_path = REPO_ROOT / f"data/{ds_name}"
        if not ds_path.exists():
            continue

        available_frames = list_frames(ds_path)
        valid_frames = [f for f in frame_list if f in available_frames]
        if not valid_frames and available_frames:
            valid_frames = available_frames[:5]

        for fid in valid_frames:
            try:
                fr = load_frame(ds_path, fid)
            except Exception as e:
                print(f"[WARN] Could not load frame {fid} from {ds_name}: {e}")
                continue

            pts_raw = fr["points"]
            calib_raw = fr["calib"]
            labels = fr["labels"]
            img_shape = fr["image"].shape

            for deg_name, deg_meta in DEGRADATION_CONFIGS.items():
                unit = deg_meta["unit"]
                values = deg_meta["values"]

                for level_idx, val in enumerate(values, start=1):
                    t0 = time.perf_counter()
                    pts_deg, calib_deg = apply_degradation(pts_raw, calib_raw, deg_name, val, seed=seed)
                    m = compute_frame_metrics(
                        pts_deg, calib_deg, labels, img_shape, baseline_points=pts_raw
                    )
                    t1 = time.perf_counter()
                    latency_ms = (t1 - t0) * 1000.0

                    # Clean numeric values to ensure zero NaNs/Infs
                    health_sc = float(m["health_score"])
                    if math.isnan(health_sc) or math.isinf(health_sc):
                        health_sc = 0.0

                    fov_pct = float(m["pts_in_fov_pct"])
                    if math.isnan(fov_pct) or math.isinf(fov_pct):
                        fov_pct = 0.0

                    records.append({
                        "dataset": ds_name,
                        "frame_id": fid,
                        "degradation_type": deg_name,
                        "intensity_level": level_idx,
                        "parameter_value": float(val),
                        "parameter_unit": unit,
                        "n_points": int(m["n_points"]),
                        "pts_in_fov": int(m["pts_in_fov"]),
                        "pts_in_fov_pct": round(fov_pct, 2),
                        "pts_car": int(m["pts_car"]),
                        "pts_ped": int(m["pts_ped"]),
                        "pts_cyc": int(m["pts_cyc"]),
                        "starved_objects": int(m["starved_objects"]),
                        "health_score": round(health_sc, 2),
                        "health_status": str(m["health_status"]),
                        "latency_ms": round(latency_ms, 2),
                    })

    df = pd.DataFrame(records)

    if out_csv is not None:
        csv_path = Path(out_csv)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        # Avoid writing NaN or inf strings
        df.to_csv(csv_path, index=False, float_format="%.2f", encoding="utf-8")
        print(f"[OK] Exported benchmark CSV with {len(df)} rows to: {csv_path}")

    if out_dir is not None:
        generate_plots(df, out_dir)

    return df


def generate_plots(df: pd.DataFrame, out_dir: str | Path) -> list[Path]:
    """Generate high-resolution benchmark visualization figures.

    Produces:
    1. results/figures/degradation_curves.png
    2. results/figures/health_score_vs_intensity.png
    3. results/figures/vru_starvation_analysis.png
    """
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    generated_files: list[Path] = []

    if df.empty:
        print("[WARN] Empty DataFrame provided to generate_plots.")
        return generated_files

    # Color palette & marker setup
    deg_types = list(DEGRADATION_CONFIGS.keys())
    colors = {
        "Random Dropout": "#1f77b4",     # Blue
        "Range Dropout": "#ff7f0e",      # Orange
        "Beam Dropout": "#2ca02c",       # Green
        "Gaussian Noise": "#9467bd",     # Purple
        "Calibration Drift": "#d62728",  # Red
    }
    markers = {
        "Random Dropout": "o",
        "Range Dropout": "s",
        "Beam Dropout": "^",
        "Gaussian Noise": "D",
        "Calibration Drift": "v",
    }

    # =========================================================================
    # Plot 1: degradation_curves.png (Multi-panel curves)
    # =========================================================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10), dpi=180)
    fig.suptitle("LiDAR Sensor Degradation Curves across Stress Dimensions", fontsize=15, fontweight="bold")

    # Panel (0, 0): Sensor Health Score vs Intensity Level
    ax = axes[0, 0]
    for dt in deg_types:
        sub = df[df["degradation_type"] == dt]
        if not sub.empty:
            mean_vals = sub.groupby("intensity_level")["health_score"].mean()
            ax.plot(mean_vals.index, mean_vals.values, marker=markers[dt], color=colors[dt],
                    label=dt, linewidth=2.2, markersize=7)
    ax.set_title("(a) Composite Sensor Health Score (0-100)", fontsize=12, fontweight="bold")
    ax.set_xlabel("Degradation Severity Level (1=Clean → 5=Extreme)", fontsize=10)
    ax.set_ylabel("Health Score", fontsize=10)
    ax.set_ylim(-2, 102)
    ax.axhline(75.0, color="#2ca02c", linestyle="--", alpha=0.6, label="Healthy Threshold (75)")
    ax.axhline(40.0, color="#d62728", linestyle="--", alpha=0.6, label="Failure Threshold (40)")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(fontsize=8, loc="lower left")

    # Panel (0, 1): Points in Camera FOV (%) vs Intensity Level
    ax = axes[0, 1]
    for dt in deg_types:
        sub = df[df["degradation_type"] == dt]
        if not sub.empty:
            mean_vals = sub.groupby("intensity_level")["pts_in_fov_pct"].mean()
            ax.plot(mean_vals.index, mean_vals.values, marker=markers[dt], color=colors[dt],
                    label=dt, linewidth=2.2, markersize=7)
    ax.set_title("(b) Points Retained Inside Camera FOV (%)", fontsize=12, fontweight="bold")
    ax.set_xlabel("Degradation Severity Level", fontsize=10)
    ax.set_ylabel("LiDAR Points in FOV (%)", fontsize=10)
    ax.set_ylim(-2, 102)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(fontsize=8, loc="upper right")

    # Panel (1, 0): Object Point Returns (Car vs VRUs) under Beam Dropout
    ax = axes[1, 0]
    beam_sub = df[df["degradation_type"] == "Beam Dropout"]
    if not beam_sub.empty:
        car_mean = beam_sub.groupby("intensity_level")["pts_car"].mean()
        ped_mean = beam_sub.groupby("intensity_level")["pts_ped"].mean()
        cyc_mean = beam_sub.groupby("intensity_level")["pts_cyc"].mean()
        ax.plot(car_mean.index, car_mean.values, marker="o", color="#1f77b4", label="Car Returns", linewidth=2.0)
        ax.plot(ped_mean.index, ped_mean.values, marker="s", color="#ff7f0e", label="Pedestrian Returns", linewidth=2.0)
        ax.plot(cyc_mean.index, cyc_mean.values, marker="^", color="#2ca02c", label="Cyclist Returns", linewidth=2.0)
    ax.set_title("(c) Object-Level Returns under Beam Decimation", fontsize=12, fontweight="bold")
    ax.set_xlabel("Beam Decimation Level (1=64-beam → 5=4-beam)", fontsize=10)
    ax.set_ylabel("Mean Points inside 3D GT Box", fontsize=10)
    ax.set_yscale("log")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(fontsize=9, loc="upper right")

    # Panel (1, 1): Starved Objects Count vs Intensity Level
    ax = axes[1, 1]
    for dt in deg_types:
        sub = df[df["degradation_type"] == dt]
        if not sub.empty:
            starved_mean = sub.groupby("intensity_level")["starved_objects"].mean()
            ax.plot(starved_mean.index, starved_mean.values, marker=markers[dt], color=colors[dt],
                    label=dt, linewidth=2.2, markersize=7)
    ax.set_title("(d) Object Starvation Count (<= 3 pts/box)", fontsize=12, fontweight="bold")
    ax.set_xlabel("Degradation Severity Level", fontsize=10)
    ax.set_ylabel("Starved Objects Count", fontsize=10)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(fontsize=8, loc="upper left")

    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    p1 = out_path / "degradation_curves.png"
    plt.savefig(p1, bbox_inches="tight")
    plt.close(fig)
    generated_files.append(p1)
    print(f"[OK] Generated figure: {p1}")

    # =========================================================================
    # Plot 2: health_score_vs_intensity.png (Health Score & Critical Thresholds)
    # =========================================================================
    fig2, ax2 = plt.subplots(figsize=(10, 6.5), dpi=180)

    # Shaded health status zones
    ax2.axhspan(75, 100, color="#d4edda", alpha=0.7, label="HEALTHY Zone (>= 75)")
    ax2.axhspan(60, 75, color="#fff3cd", alpha=0.7, label="DEGRADED Zone (60 - 75)")
    ax2.axhspan(40, 60, color="#ffeeba", alpha=0.7, label="CRITICAL Zone (40 - 60)")
    ax2.axhspan(0, 40, color="#f8d7da", alpha=0.7, label="FAILURE Zone (< 40)")

    for dt in deg_types:
        sub = df[df["degradation_type"] == dt]
        if not sub.empty:
            mean_vals = sub.groupby("intensity_level")["health_score"].mean()
            std_vals = sub.groupby("intensity_level")["health_score"].std().fillna(0.0)
            ax2.plot(mean_vals.index, mean_vals.values, marker=markers[dt], color=colors[dt],
                     label=f"{dt}", linewidth=2.5, markersize=8)
            ax2.fill_between(mean_vals.index, mean_vals.values - std_vals, mean_vals.values + std_vals,
                             color=colors[dt], alpha=0.15)

    ax2.set_title("Composite Sensor Health Score vs Degradation Intensity", fontsize=13, fontweight="bold")
    ax2.set_xlabel("Intensity Level (1: Clean/Nominal → 5: Maximum Degradation)", fontsize=11)
    ax2.set_ylabel("Sensor Health Score [0 - 100]", fontsize=11)
    ax2.set_xticks([1, 2, 3, 4, 5])
    ax2.set_xticklabels(["Level 1\n(Nominal)", "Level 2\n(Mild)", "Level 3\n(Moderate)", "Level 4\n(Severe)", "Level 5\n(Critical)"])
    ax2.set_ylim(-2, 102)
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend(loc="lower left", fontsize=9, framealpha=0.9)

    plt.tight_layout()
    p2 = out_path / "health_score_vs_intensity.png"
    plt.savefig(p2, bbox_inches="tight")
    plt.close(fig2)
    generated_files.append(p2)
    print(f"[OK] Generated figure: {p2}")

    # =========================================================================
    # Plot 3: vru_starvation_analysis.png (VRU vs Vehicle Vulnerability)
    # =========================================================================
    fig3, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(14, 5.5), dpi=180)
    fig3.suptitle("Vulnerable Road User (VRU) Starvation Analysis under Sensor Attenuation",
                  fontsize=13, fontweight="bold")

    # Left: Retention percentage under Beam Dropout
    beam_data = df[df["degradation_type"] == "Beam Dropout"]
    if not beam_data.empty:
        car_b1 = max(1.0, float(beam_data[beam_data["intensity_level"] == 1]["pts_car"].mean()))
        ped_b1 = max(1.0, float(beam_data[beam_data["intensity_level"] == 1]["pts_ped"].mean()))
        cyc_b1 = max(1.0, float(beam_data[beam_data["intensity_level"] == 1]["pts_cyc"].mean()))

        levels = [1, 2, 3, 4, 5]
        car_ret = [float(beam_data[beam_data["intensity_level"] == l]["pts_car"].mean()) / car_b1 * 100.0 for l in levels]
        ped_ret = [float(beam_data[beam_data["intensity_level"] == l]["pts_ped"].mean()) / ped_b1 * 100.0 for l in levels]
        cyc_ret = [float(beam_data[beam_data["intensity_level"] == l]["pts_cyc"].mean()) / cyc_b1 * 100.0 for l in levels]

        beam_labels = ["64-beam", "32-beam", "16-beam", "8-beam", "4-beam"]
        ax_left.plot(beam_labels, car_ret, marker="o", color="#1f77b4", linewidth=2.4, label="Vehicle (Car)")
        ax_left.plot(beam_labels, ped_ret, marker="s", color="#d62728", linewidth=2.4, label="Pedestrian (VRU)")
        ax_left.plot(beam_labels, cyc_ret, marker="^", color="#ff7f0e", linewidth=2.4, label="Cyclist (VRU)")
        ax_left.axhline(10.0, color="#d62728", linestyle=":", alpha=0.7, label="Starvation Threshold (<10%)")
        ax_left.set_title("Point Retention % vs LiDAR Beam Count", fontsize=11, fontweight="bold")
        ax_left.set_xlabel("LiDAR Configuration / Beam Density", fontsize=10)
        ax_left.set_ylabel("Point Retention (% of 64-beam baseline)", fontsize=10)
        ax_left.set_ylim(-2, 105)
        ax_left.grid(True, linestyle=":", alpha=0.6)
        ax_left.legend(fontsize=9, loc="upper right")

    # Right: Extrinsic Yaw Drift Point Starvation
    drift_data = df[df["degradation_type"] == "Calibration Drift"]
    if not drift_data.empty:
        car_d1 = max(1.0, float(drift_data[drift_data["intensity_level"] == 1]["pts_car"].mean()))
        ped_d1 = max(1.0, float(drift_data[drift_data["intensity_level"] == 1]["pts_ped"].mean()))
        cyc_d1 = max(1.0, float(drift_data[drift_data["intensity_level"] == 1]["pts_cyc"].mean()))

        levels = [1, 2, 3, 4, 5]
        car_drift_ret = [float(drift_data[drift_data["intensity_level"] == l]["pts_car"].mean()) / car_d1 * 100.0 for l in levels]
        ped_drift_ret = [float(drift_data[drift_data["intensity_level"] == l]["pts_ped"].mean()) / ped_d1 * 100.0 for l in levels]
        cyc_drift_ret = [float(drift_data[drift_data["intensity_level"] == l]["pts_cyc"].mean()) / cyc_d1 * 100.0 for l in levels]

        drift_labels = ["0.0°", "0.5°", "1.0°", "2.0°", "3.0°"]
        ax_right.plot(drift_labels, car_drift_ret, marker="o", color="#1f77b4", linewidth=2.4, label="Vehicle (Car)")
        ax_right.plot(drift_labels, ped_drift_ret, marker="s", color="#d62728", linewidth=2.4, label="Pedestrian (VRU)")
        ax_right.plot(drift_labels, cyc_drift_ret, marker="^", color="#ff7f0e", linewidth=2.4, label="Cyclist (VRU)")
        ax_right.set_title("GT 3D Box Point Loss under Extrinsic Yaw Drift", fontsize=11, fontweight="bold")
        ax_right.set_xlabel("LiDAR-Camera Yaw Drift (degrees)", fontsize=10)
        ax_right.set_ylabel("Point Retention (% of aligned baseline)", fontsize=10)
        ax_right.set_ylim(-2, 105)
        ax_right.grid(True, linestyle=":", alpha=0.6)
        ax_right.legend(fontsize=9, loc="upper right")

    plt.tight_layout(rect=[0, 0.03, 1, 0.96])
    p3 = out_path / "vru_starvation_analysis.png"
    plt.savefig(p3, bbox_inches="tight")
    plt.close(fig3)
    generated_files.append(p3)
    print(f"[OK] Generated figure: {p3}")

    return generated_files


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")

    parser = argparse.ArgumentParser(description="LiDAR Sensor Degradation Stress Test Benchmark")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED, help="Deterministic random seed (default: 42)")
    parser.add_argument("--out-csv", default="results/sensor_degradation_benchmark.csv", help="Output benchmark CSV path")
    parser.add_argument("--out-dir", default="results/figures", help="Output directory for generated figures")
    parser.add_argument("--datasets", default="synthetic,kitti_mini,nuscenes_mini_subset",
                        help="Comma-separated dataset names to benchmark")
    args = parser.parse_args()

    ds_tokens = [d.strip() for d in args.datasets.split(",") if d.strip()]
    ds_plan: list[tuple[str, list[str]]] = []
    for d in ds_tokens:
        if d == "synthetic":
            ds_plan.append((d, ["000000", "000001", "000002", "000003", "000004"]))
        elif d == "kitti_mini":
            ds_plan.append((d, ["000001", "000011", "000021", "000031", "000041"]))
        elif d == "nuscenes_mini_subset":
            ds_plan.append((d, ["scene-0103_010", "scene-0103_020", "scene-1094_010", "scene-1094_020"]))
        else:
            ds_plan.append((d, []))

    print(f"Starting Sensor Degradation Stress Test Benchmark (seed={args.seed})...")
    df = run_stress_test(
        datasets=ds_plan,
        seed=args.seed,
        out_csv=args.out_csv,
        out_dir=args.out_dir,
    )
    print(f"Stress test finished successfully! Total benchmark runs: {len(df)}")


if __name__ == "__main__":
    main()
