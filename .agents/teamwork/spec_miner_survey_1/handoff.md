# Handoff Report: Specification & Submission Criteria Survey (Topic C)

## Observation

### 1. `tools/check_submission.py` Implementation & Checks
Investigation of `tools/check_submission.py` (lines 15–96) revealed exactly 10 gates checked before submission is considered valid:

- **Root directory:** `ROOT = Path(__file__).resolve().parents[1]` (`d:\K4-Track4-Day06-3D-From-Point-Clouds`).
- **Regular Expressions:**
  - `PLACEHOLDER_RE = re.compile(r"\[ĐIỀN[^\]]*\]")` (line 16): Matches any bracketed string beginning with `[ĐIỀN` up to `]`.
  - `MSSV_RE = re.compile(r"\*\*MSSV:\*\*\s*([A-Za-z0-9]+)")` (line 17): Extracts student ID immediately following `**MSSV:**`.
- **Secret Detection Patterns (`SECRET_PATTERNS` lines 20–27):**
  - OpenAI / generic key: `re.compile(r"sk-[A-Za-z0-9_\-]{20,}")`
  - AWS Access Key: `re.compile(r"AKIA[0-9A-Z]{16}")`
  - HuggingFace token: `re.compile(r"hf_[A-Za-z0-9]{30,}")`
  - GitHub PAT: `re.compile(r"ghp_[A-Za-z0-9]{30,}")`
  - Google API key: `re.compile(r"AIza[0-9A-Za-z_\-]{35}")`
  - Generic token assignment: `re.compile(r"(?i)(api[_-]?key|secret|token)\s*[:=]\s*['\"][^'\"\s]{12,}['\"]")`
- **File constraints:**
  - `MAX_FILE_MB = 20` (line 28): Size checked as `stat().st_size > MAX_FILE_MB * 1e6` (20,000,000 bytes).
  - Forbidden raw / checkpoint extensions outside `data/`: `{".bin", ".pcd", ".bag", ".db3", ".pth", ".pt", ".ckpt"}` (lines 72–75).
  - Forbidden config: `.env` file (line 77).
- **The 10 Gates & Evaluated Conditions:**
  1. `report/REPORT.md đủ 6 mục`: Checks `report.exists() and not missing` where `missing = [s for s in REPORT_SECTIONS if s not in text]`. Required headings (verbatim):
     - `## 1. Claim`
     - `## 2. Evidence`
     - `## 3. Failure case`
     - `## 4. Khuyến nghị`
     - `## 5. Cách chạy lại`
     - `## 6. Khai báo sử dụng AI`
  2. `report/REPORT.md đã điền hết`: Checks `bool(text) and not left` where `left = PLACEHOLDER_RE.findall(text)`.
  3. `report/REPORT.md có dòng **MSSV:** hợp lệ (chỉ chữ và số)`: Checks `mssv is not None` where `mssv_match = MSSV_RE.search(text)`.
  4. `results/ có >= 1 bảng số liệu (.csv)`: Checks `len(csvs) >= 1` via `(ROOT / "results").rglob("*.csv")`.
  5. `results/ có >= 1 ảnh/video demo`: Checks `len(media) >= 1` where media matches `*.png`, `*.jpg`, `*.gif`, `*.mp4` in `results/`.
  6. `results/ có ảnh failure case (tên chứa 'fail')`: Checks `len(fail_media) >= 1` where `fail_media = [p for p in media if "fail" in p.name.lower()]`.
  7. `Không có file > 20 MB`: Checks `not big` across files tracked by `git ls-files` (or `ROOT.rglob("*")` if git fails).
  8. `Không commit dữ liệu thô / checkpoint ngoài thư mục data/`: No tracked file with disallowed extension outside `data/`.
  9. `Không commit .env`: No tracked file named `.env`.
  10. `Không lộ API key / token`: Text files in `{".py", ".md", ".txt", ".yaml", ".yml", ".json", ".ipynb", ".cfg", ".toml", ".sh", ".ps1"}` (excluding `check_submission.py`) do not match any secret pattern.
- **Exit Code:** Returns 0 if all 10 checks succeed and prints `KẾT QUẢ: SẴN SÀNG NỘP`. Returns 1 if any check fails and prints `KẾT QUẢ: CHƯA ĐỦ ĐIỀU KIỆN NỘP`.

### 2. Baseline Test Run
Running `.venv\Scripts\python.exe tools/check_submission.py` on the fresh workspace produced:
```
[PASS] report/REPORT.md đủ 6 mục
[FAIL] report/REPORT.md đã điền hết  -> còn 17 chỗ chưa điền, ví dụ ['[ĐIỀN tên đề tài ngắn]', '[ĐIỀN]', '[ĐIỀN]']
[FAIL] report/REPORT.md có dòng **MSSV:** hợp lệ (chỉ chữ và số)
[FAIL] results/ có >= 1 bảng số liệu (.csv)  -> 0 file
[FAIL] results/ có >= 1 ảnh/video demo  -> 0 file
[FAIL] results/ có ảnh failure case (tên chứa 'fail')  -> đặt tên ví dụ fail_01_yaw_drift.png
[PASS] Không có file > 20 MB
[PASS] Không commit dữ liệu thô / checkpoint ngoài thư mục data/
[PASS] Không commit .env
[PASS] Không lộ API key / token

KẾT QUẢ: CHƯA ĐỦ ĐIỀU KIỆN NỘP
```
Exit code was `1`.

### 3. `report/REPORT.md` Template Analysis
Inspection of `report/REPORT.md` (lines 1–60) revealed exactly 17 placeholder targets matching `PLACEHOLDER_RE`:
1. Line 1: `# Báo cáo Day 6: [ĐIỀN tên đề tài ngắn]`
2. Line 5: `- **Họ tên:** [ĐIỀN]` -> Expected: `Ngô Xuân Hoàng`
3. Line 6: `- **MSSV:** [ĐIỀN] (phải trùng với MSSV trong tên repo <HoVaTen>-<MSSV>-Track4-Day21)` -> Must match `\*\*MSSV:\*\*\s*([A-Za-z0-9]+)`, e.g. `- **MSSV:** 2A202602597`
4. Line 7: `- **Lớp:** [ĐIỀN]` -> Expected: `H209`
5. Line 8: `- **Link repo:** [ĐIỀN]` -> Fork URL / repo path
6. Line 9: `- **Topic:** [ĐIỀN một chữ cái A/B/C/D/E/F] — [ĐIỀN tên topic]` (contains 2 placeholders!) -> Expected: `- **Topic:** C — Sensor degradation stress test`
7. Line 10: `- **Dataset:** [ĐIỀN một hoặc nhiều trong: data/synthetic, data/kitti_mini, data/nuscenes_mini_subset, log riêng]`
8. Line 11: `- **Các frame đã dùng:** [ĐIỀN danh sách frame id, ví dụ 000011, 000049 hoặc scene-0103_010]`
9. Line 19: Under `## 1. Claim`: single verifiable technical hypothesis with measurable metric, conditions, and threshold.
10. Line 27: Under `## 2. Evidence`: Markdown table row `| [ĐIỀN] | | | |`
11. Line 29: Under `## 2. Evidence`: Image markdown `![demo](../results/figures/[ĐIỀN].png)`
12. Line 35: Under `## 3. Failure case`: Image markdown `![failure](../results/figures/fail_[ĐIỀN].png)`
13. Line 37: Under `## 3. Failure case`: Text explanation associating the failure with one of the 6 debug layers (`I/O`, `Geometry`, `Time`, `Preprocess`, `Model`, `Metric`).
14. Line 43: Under `## 4. Khuyến nghị`: Deployment advice for ADAS/robotics, engineering trade-offs, telemetry logging.
15. Line 50: Under `## 5. Cách chạy lại`: Inside code fence: clean repo reproduction instructions.
16. Line 59: Under `## 6. Khai báo sử dụng AI`: Markdown table row `| [ĐIỀN] | | |` for AI disclosure (tool, purpose, verification method).

### 4. Repository Rules, Rubrics, and Acceptance Criteria
- **RULES.md & RUBRIC.md:**
  - AI declaration is mandatory: Missing Section 6 = −10 points penalty.
  - Fabricating data or failure cases = 0 points.
  - Modifying or deleting original data in `data/` = −5 points penalty.
  - Committing raw data/checkpoints outside `data/` or files > 20 MB = −5 points penalty.
  - Code edits restricted to 2 TODO functions in `starter/projection.py`. All custom code must live in `src/`.
- **TOPICS.md & ORIGINAL_REQUEST.md (Topic C Specifications):**
  - Model-free stress testing on point clouds.
  - At least 4 perturbation types required (Random Dropout, Range Dropout, Beam Dropout, Gaussian Noise, Calibration Drift, Motion Smear) across 3–5 severity levels.
  - Metrics required:
    1. Points inside 3D GT bounding boxes (`Car`, `Pedestrian`, `Cyclist`).
    2. Ratio of points inside camera FOV (`mask.mean()`).
    3. Custom Sensor Health Score / Density metric.
  - Seed must be fixed for reproducibility across benchmark runs.
  - Interactive Demo GUI/Web app in `src/` (CPU-compatible) with real-time controls and dual camera projection + BEV visualizer.

## Logic Chain

1. From `tools/check_submission.py` line 90-91: The gatekeeper script returns exit code 0 if and only if all 10 checks evaluate to True.
2. From the baseline run observation: 5 out of 10 checks currently fail:
   - Placeholder check fails because 17 placeholders `[ĐIỀN...]` remain in `report/REPORT.md`.
   - MSSV check fails because line 6 of `report/REPORT.md` contains `[ĐIỀN]` instead of an alphanumeric MSSV string.
   - Results CSV check fails because `results/` contains 0 `.csv` files.
   - Results media check fails because `results/` contains 0 image/video files.
   - Failure case media check fails because `results/` contains 0 files with `'fail'` in their filename.
3. From `SUBMISSION.md` and `RUBRIC.md`:
   - Results artifacts must follow:
     - `results/*.csv` (e.g. `results/sensor_degradation_benchmark.csv`)
     - `results/figures/<name>.png` (e.g. `results/figures/degradation_curves.png`, `results/figures/overlay_demo.png`)
     - `results/figures/fail_<num>_<desc>.png` (e.g. `results/figures/fail_01_range_dropout_far_car.png`, `results/figures/fail_02_beam_dropout_pedestrian.png`, `results/figures/fail_03_yaw_drift_misalignment.png`)
4. From `ORIGINAL_REQUEST.md`:
   - Student info must match:
     - Name: `Ngô Xuân Hoàng`
     - MSSV: `2A202602597`
     - Class: `H209`
   - Projection implementation in `starter/projection.py` must fulfill 2 functions:
     - `velo_to_cam`: Homogeneous transformation `T_cam_velo = R0_rect @ Tr_velo_to_cam`, slicing first 3 columns.
     - `cam_to_image`: Filter NaN/Inf, project via `P2`, filter `depth > min_depth`, divide by `s`, bounds-check `0 <= u < W` and `0 <= v < H`.
   - Stress test pipeline in `src/` must compute points in GT 3D boxes without deep learning models, outputting fixed-seed CSV and degradation plots.
   - Demo application must run on CPU in `.venv`.

## Caveats

- `tools/check_submission.py` only validates formal compliance (syntax, presence of files, patterns, file size). Full grading depends on `RUBRIC.md` (reproducibility, depth of failure case analysis, metric rigor, code quality).
- Git tracking status: `tools/check_submission.py` uses `git ls-files` to determine tracked files. Any untracked scratch files in `.agents/` are ignored by git checks, but must not contain committed API keys or exceed 20 MB if staged.
- The environment Python is located at `.venv/Scripts/python.exe` on Windows; OpenCV, NumPy, Matplotlib, and Pandas are present in `.venv`.

## Conclusion

The submission specification and check pipeline are fully decoded:
1. `tools/check_submission.py` imposes 10 strict binary gates, exit code 0 required.
2. Baseline run currently fails 5 gates (`PLACEHOLDER_RE`, `MSSV_RE`, `.csv` count, media count, failure media count).
3. Student credentials are confirmed: `Ngô Xuân Hoàng`, MSSV `2A202602597`, Class `H209`.
4. Required output files:
   - `starter/projection.py` with `velo_to_cam` and `cam_to_image` implemented.
   - `src/` modules implementing degradation stress tests, metrics, and demo app.
   - `results/*.csv` with fixed-seed benchmark sweeps.
   - `results/figures/*.png` including demo visual overlays and degradation plots.
   - `results/figures/fail_*.png` with at least 2 distinct failure case figures mapped to debug layers.
   - `report/REPORT.md` with all 6 sections completely filled, 0 `[ĐIỀN]` placeholders remaining.

## Features Discovered

| # | Category | Feature | Description | Inputs | Outputs | Error Behavior | Discovered Via |
|---|----------|---------|-------------|--------|---------|----------------|----------------|
| 1 | Submission Gate | Section Count Check | Verifies all 6 required sections exist in `report/REPORT.md` | `report/REPORT.md` | `[PASS]` / `[FAIL]` | Lists missing sections; fails if any missing | `tools/check_submission.py:49-50` |
| 2 | Submission Gate | Placeholder Verification | Regex scan for remaining `[ĐIỀN...]` templates | `report/REPORT.md` | `[PASS]` / `[FAIL]` | Reports count and first 3 offending tokens | `tools/check_submission.py:51-53` |
| 3 | Submission Gate | Alphanumeric MSSV Gate | Validates `**MSSV:**` line format | `report/REPORT.md` | `[PASS]` / `[FAIL]` | Fails if missing or non-alphanumeric | `tools/check_submission.py:54-56` |
| 4 | Submission Gate | CSV Metric Artifact Gate | Confirms presence of at least 1 benchmark CSV file | `results/**/*.csv` | `[PASS]` / `[FAIL]` | Fails if 0 CSV files exist | `tools/check_submission.py:59,61` |
| 5 | Submission Gate | Demo Media Gate | Confirms presence of at least 1 image/video demo | `results/**/*.{png,jpg,gif,mp4}` | `[PASS]` / `[FAIL]` | Fails if 0 media files exist | `tools/check_submission.py:60,62` |
| 6 | Submission Gate | Failure Case Media Gate | Checks for media files containing 'fail' in filename | `results/**/fail*.{png,jpg,gif,mp4}` | `[PASS]` / `[FAIL]` | Fails if no file matches case-insensitive 'fail' | `tools/check_submission.py:63-64` |
| 7 | Submission Gate | File Size Cap | Ensures no tracked file exceeds 20 MB | Tracked git files | `[PASS]` / `[FAIL]` | Reports file names and sizes > 20 MB | `tools/check_submission.py:66-69` |
| 8 | Submission Gate | Forbidden Data/Weights Gate | Blocks committing raw `.bin`, `.pcd`, `.pth`, etc. outside `data/` | Tracked git files | `[PASS]` / `[FAIL]` | Lists offending paths outside `data/` | `tools/check_submission.py:72-75` |
| 9 | Submission Gate | Dotenv Leak Gate | Forbids committing `.env` configuration files | Tracked git files | `[PASS]` / `[FAIL]` | Fails if `.env` is tracked | `tools/check_submission.py:77` |
| 10 | Submission Gate | API Key / Token Leak Scan | Regex scanning of text files for exposed credentials | Tracked text files | `[PASS]` / `[FAIL]` | Identifies leaked credential paths | `tools/check_submission.py:78-85` |
| 11 | Data QA | Data Verification Utility | Validates dataset integrity against `MANIFEST.json` sha256 checksums | `--data-root <path>` | `[PASS] Dữ liệu đầy đủ` / `[FAIL]` | Prints missing/corrupted file lists, exit code 1 | `tools/verify_data.py:38-63` |
| 12 | Starter / Geometry | Velo to Camera Transformation | Converts LiDAR coordinates to rectified camera frame | `(N, 3)` points, `KittiCalib` | `(N, 3)` points_cam | Raises `NotImplementedError` currently | `starter/projection.py:26-36` |
| 13 | Starter / Geometry | Camera to Image Projection | Projects 3D camera points to 2D image coordinates with depth masking | `(N, 3)` points, `P2`, `image_shape` | `(M, 2)` uv, `(M,)` depth, `(N,)` mask | Raises `NotImplementedError` currently | `starter/projection.py:38-56` |
| 14 | Starter / Geometry | Extrinsic Perturbation | Simulates calibration drift (yaw, pitch, roll, translation) | `KittiCalib`, angles (deg), translation (m) | Perturbed `KittiCalib` | Valid return value | `starter/projection.py:82-94` |
| 15 | Starter / Perturb | Point Cloud Degradations | Implements 6 degradation methods: random dropout, range dropout, sector dropout, beam dropout, gaussian noise, motion smear | `(N, 4)` points, parameters, seed | `(M, 4)` degraded points | Preserves input immutability | `starter/perturb.py:11-52` |
| 16 | Starter / Data Health | Point Cloud Health Statistics | Computes point count, invalid ratio, distance percentiles, azimuth coverage | `(N, 4)` points | Summary dictionary | Returns NaN for empty arrays | `starter/data_health.py:23-42` |

## Edge Cases

| # | Feature | Input | Observed Behavior |
|---|---------|-------|-------------------|
| 1 | `tools/check_submission.py` MSSV check | `- **MSSV:** [2A202602597]` (bracketed) | Will FAIL because `[` is not in `[A-Za-z0-9]+` and triggers placeholder regex if `ĐIỀN` is inside |
| 2 | `tools/check_submission.py` Placeholder check | Multiple placeholders in same line (e.g. line 9: `[ĐIỀN...][ĐIỀN...]`) | `PLACEHOLDER_RE.findall` matches both instances; each must be cleanly removed |
| 3 | `tools/check_submission.py` Failure image check | Media filename with uppercase `FAIL_01.png` | Passes because check uses `p.name.lower()` |
| 4 | `tools/check_submission.py` Git tracking fallback | Git command failure or `.git` unavailable | Falls back to `ROOT.rglob("*")` excluding `.git`, checking untracked files as well |
| 5 | `starter/projection.py` Projection invalid values | Points with NaN, Inf, or $z \le min\_depth$ (e.g. points behind camera) | Must be filtered before or during division to prevent `ZeroDivisionError` or invalid pixel mappings |
| 6 | `tools/verify_data.py` on Synthetic | `python tools/verify_data.py --data-root data/synthetic` | Fails with `Không thấy data\synthetic\MANIFEST.json` because only `kitti_mini` and `nuscenes_mini_subset` have manifests |

## Verification Method

The findings and submission state can be independently verified with the following commands:
1. **Submission Gate Status:**
   ```powershell
   .venv\Scripts\python.exe tools/check_submission.py
   ```
   *Expected:* Exit code 1; reports 17 remaining placeholders in `report/REPORT.md`, invalid MSSV, 0 CSVs, 0 media, and 0 failure media files.
2. **Dataset Integrity Verification:**
   ```powershell
   .venv\Scripts\python.exe tools/verify_data.py --data-root data/kitti_mini
   .venv\Scripts\python.exe tools/verify_data.py --data-root data/nuscenes_mini_subset
   ```
   *Expected:* Both print `[PASS] Dữ liệu đầy đủ, dùng được.` and exit code 0.
3. **Data Health Script Execution:**
   ```powershell
   .venv\Scripts\python.exe -m starter.data_health --data-root data/synthetic
   ```
   *Expected:* Successfully evaluates 5 frames (`000000` to `000004`) and writes `results/data_health.csv`.
