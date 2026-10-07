import sys
import time
from pathlib import Path

# Add repo root to sys.path
REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import numpy as np

from starter.datasets import load_frame
from starter.kitti_io import load_calib, KittiCalib
from starter.projection import velo_to_cam, cam_to_image, project_velo_to_image

def run_all_tests():
    calib = load_calib("data/synthetic/training/calib/000000.txt")
    img_shape = (375, 1242)
    H, W = img_shape
    failures = []
    passes = 0

    def test(name, condition, msg=""):
        nonlocal passes
        if condition:
            passes += 1
            print(f"[PASS] {name}")
        else:
            failures.append((name, msg))
            print(f"[FAIL] {name}: {msg}")

    print("=== Section 1: velo_to_cam Edge Cases ===")
    
    # 1.1 Empty array variations
    t_empty = velo_to_cam(np.empty((0, 3)), calib)
    test("velo_to_cam empty (0,3)", t_empty.shape == (0, 3))
    
    t_empty4 = velo_to_cam(np.empty((0, 4)), calib)
    test("velo_to_cam empty (0,4)", t_empty4.shape == (0, 3))

    t_empty_1d = velo_to_cam(np.array([]), calib)
    test("velo_to_cam empty 1D", t_empty_1d.shape == (0, 3))

    # 1.2 1D array input
    t_1d_3 = velo_to_cam(np.array([10.0, 0.0, 0.0]), calib)
    test("velo_to_cam 1D 3-elem shape", t_1d_3.shape == (3,))
    test("velo_to_cam 1D 3-elem positive z", t_1d_3[2] > 9.0)

    t_1d_4 = velo_to_cam(np.array([10.0, 0.0, 0.0, 0.5]), calib)
    test("velo_to_cam 1D 4-elem shape", t_1d_4.shape == (3,))

    # 1.3 Python list input
    t_list = velo_to_cam([[10.0, 0.0, 0.0], [5.0, 1.0, 2.0]], calib)
    test("velo_to_cam python list", isinstance(t_list, np.ndarray) and t_list.shape == (2, 3))

    # 1.4 Single point 2D
    t_single = velo_to_cam(np.array([[10.0, 0.0, 0.0]]), calib)
    test("velo_to_cam single point 2D", t_single.shape == (1, 3))

    # 1.5 Dtypes (int, float32, float64)
    t_int = velo_to_cam(np.array([[10, 0, 0]], dtype=int), calib)
    test("velo_to_cam int dtype", t_int.shape == (1, 3) and np.isclose(t_int[0, 2], 9.727, atol=0.1))

    t_f32 = velo_to_cam(np.array([[10.0, 0.0, 0.0]], dtype=np.float32), calib)
    test("velo_to_cam float32 dtype", t_f32.shape == (1, 3) and np.isclose(t_f32[0, 2], 9.727, atol=0.1))

    # 1.6 Large point cloud performance & memory
    pts_large = np.random.randn(200000, 3)
    t0 = time.perf_counter()
    t_large = velo_to_cam(pts_large, calib)
    dt = time.perf_counter() - t0
    test(f"velo_to_cam 200k points latency ({dt*1000:.1f}ms)", dt < 0.1 and t_large.shape == (200000, 3))

    print("\n=== Section 2: cam_to_image Edge Cases & Non-Finite Handling ===")

    # 2.1 Empty array
    uv, depth, mask = cam_to_image(np.empty((0, 3)), calib.P2, img_shape)
    test("cam_to_image empty (0,3)", uv.shape == (0, 2) and depth.shape == (0,) and mask.shape == (0,))

    # 2.2 All non-finite (NaN, Inf, -Inf)
    bad_pts = np.array([
        [np.nan, 0.0, 10.0],
        [0.0, np.inf, 10.0],
        [0.0, 0.0, np.nan],
        [np.inf, np.inf, np.inf],
        [-np.inf, -np.inf, -np.inf],
        [np.nan, np.nan, np.nan],
    ])
    uv_bad, depth_bad, mask_bad = cam_to_image(bad_pts, calib.P2, img_shape)
    test("cam_to_image all non-finite mask sum is 0", mask_bad.sum() == 0 and len(uv_bad) == 0 and len(depth_bad) == 0)
    test("cam_to_image mask length matches input", len(mask_bad) == len(bad_pts))

    # 2.3 Mixed valid and non-finite
    mixed_pts = np.array([
        [0.0, 0.0, 10.0],       # valid center
        [np.nan, 0.0, 10.0],    # nan x
        [0.0, -np.inf, 10.0],   # inf y
        [0.0, 0.0, 20.0],       # valid center
        [0.0, 0.0, -np.nan],    # nan z
    ])
    uv_m, d_m, mask_m = cam_to_image(mixed_pts, calib.P2, img_shape)
    test("cam_to_image mixed non-finite mask", list(mask_m) == [True, False, False, True, False])
    test("cam_to_image mixed uv length", len(uv_m) == 2)
    test("cam_to_image mixed depth values", np.allclose(d_m, [10.0, 20.0]))

    # 2.4 Division-by-zero guards: z <= 0 and s <= 1e-4
    div_pts = np.array([
        [0.0, 0.0, 0.0],        # exact 0 depth
        [0.0, 0.0, -10.0],      # negative depth
        [0.0, 0.0, 0.05],       # below min_depth=0.1
        [0.0, 0.0, 0.0999],     # just below min_depth
        [0.0, 0.0, 0.1001],     # just above min_depth
    ])
    uv_div, d_div, mask_div = cam_to_image(div_pts, calib.P2, img_shape, min_depth=0.1)
    test("cam_to_image depth division guards", list(mask_div) == [False, False, False, False, True])

    # 2.5 Degenerate P2 matrix where s <= 0 or s <= 1e-4 despite z > min_depth
    # Construct synthetic P2 where row 2 produces small or negative s
    P2_degen = np.array([
        [700.0, 0.0, 600.0, 0.0],
        [0.0, 700.0, 180.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],   # s = 0 always!
    ])
    pts_valid_z = np.array([[0.0, 0.0, 10.0]])
    uv_deg, d_deg, mask_deg = cam_to_image(pts_valid_z, P2_degen, img_shape)
    test("cam_to_image s=0 guard prevents division by zero", mask_deg.sum() == 0 and len(uv_deg) == 0)

    # 2.6 Boundary conditions on pixel coordinates (0 <= u < W, 0 <= v < H)
    # Use canonical P2 where s = Z = 1.0
    P2_canon = np.array([
        [100.0, 0.0, 50.0, 0.0],
        [0.0, 100.0, 50.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
    ])
    H_c, W_c = 100, 100
    b_pts = np.array([
        [-0.50, 0.0, 1.0],    # u = 0.0 (in bounds)
        [0.49, 0.0, 1.0],     # u = 99.0 (in bounds)
        [0.50, 0.0, 1.0],     # u = 100.0 (out of bounds: u == W)
        [-0.51, 0.0, 1.0],    # u = -1.0 (out of bounds: u < 0)
        [0.0, -0.50, 1.0],    # v = 0.0 (in bounds)
        [0.0, 0.49, 1.0],     # v = 99.0 (in bounds)
        [0.0, 0.50, 1.0],     # v = 100.0 (out of bounds: v == H)
        [0.0, -0.51, 1.0],    # v = -1.0 (out of bounds: v < 0)
    ])
    uv_b, d_b, mask_b = cam_to_image(b_pts, P2_canon, (H_c, W_c))
    expected_b = [True, True, False, False, True, True, False, False]
    test("cam_to_image exact boundary pixels (u in [0, W), v in [0, H))", list(mask_b) == expected_b)

    # 2.7 1D input array to cam_to_image
    uv_1d, d_1d, mask_1d = cam_to_image(np.array([0.0, 0.0, 10.0]), calib.P2, img_shape)
    test("cam_to_image 1D input shape", uv_1d.shape == (1, 2) and d_1d.shape == (1,) and len(mask_1d) == 1)

    # 2.8 cam_to_image 200k points performance
    pts_cam_large = np.random.uniform(-10, 10, size=(200000, 3))
    pts_cam_large[:, 2] = np.random.uniform(-5, 50, size=200000)
    # inject some NaNs
    pts_cam_large[::100, 0] = np.nan
    t0 = time.perf_counter()
    uv_l, d_l, mask_l = cam_to_image(pts_cam_large, calib.P2, img_shape)
    dt = time.perf_counter() - t0
    test(f"cam_to_image 200k points latency ({dt*1000:.1f}ms)", dt < 0.1 and len(mask_l) == 200000)

    print("\n=== Section 3: Multi-Dataset Full Pipeline Verification ===")

    # 3.1 Synthetic
    fr_syn = load_frame("data/synthetic", "000000")
    uv_s, d_s, m_s = project_velo_to_image(fr_syn["points"], fr_syn["calib"], fr_syn["image"].shape)
    test("synthetic frame 000000 projection count", m_s.sum() == 3910)
    test("synthetic frame all depths > 0", (d_s > 0).all())
    test("synthetic uv in bounds", (uv_s[:, 0] >= 0).all() and (uv_s[:, 0] < fr_syn["image"].shape[1]).all() and (uv_s[:, 1] >= 0).all() and (uv_s[:, 1] < fr_syn["image"].shape[0]).all())

    # 3.2 KITTI Mini
    fr_kit = load_frame("data/kitti_mini", "000011")
    uv_k, d_k, m_k = project_velo_to_image(fr_kit["points"], fr_kit["calib"], fr_kit["image"].shape)
    test("kitti frame 000011 projection count", m_k.sum() == 19946)
    test("kitti all depths > 0", (d_k > 0).all())

    # 3.3 nuScenes Mini Subset
    fr_nus = load_frame("data/nuscenes_mini_subset", "scene-0103_010")
    uv_n, d_n, m_n = project_velo_to_image(fr_nus["points"], fr_nus["calib"], fr_nus["image"].shape)
    test("nuscenes frame projection count", m_n.sum() == 3120)
    test("nuscenes all depths > 0", (d_n > 0).all())

    print(f"\n==========================================")
    print(f"RESULTS: {passes} passed, {len(failures)} failed.")
    if failures:
        for f, m in failures:
            print(f"FAILED: {f} -> {m}")
        sys.exit(1)
    else:
        print("ALL ADVERSARIAL STRESS TESTS PASSED PERFECTLY!")
        sys.exit(0)

if __name__ == "__main__":
    run_all_tests()
