# Original User Request

## Initial Request — 2026-10-07T09:27:13Z

Thực hiện toàn diện Topic C: Sensor degradation stress test (không dùng deep learning model) trên dữ liệu LiDAR-Camera KITTI / nuScenes / Synthetic, hoàn thành 2 hàm TODO projection, xây dựng pipeline stress test, tạo giao diện demo trực quan và hoàn thiện báo cáo khoa học nộp bài.

Working directory: d:/K4-Track4-Day06-3D-From-Point-Clouds
Integrity mode: development

## Requirements

### R1. Hoàn thiện các hàm TODO trong starter/projection.py
Cài đặt chính xác 2 hàm `velo_to_cam` và `cam_to_image` theo chuẩn hình học KITTI/nuScenes (toạ độ đồng nhất, ma trận biến đổi extrinsic $T_{cam\_velo} = R_0 \cdot Tr_{velo\_to\_cam}$, ma trận chiếu $P_2$, khử điểm NaN/Inf, lọc độ sâu $z > min\_depth$, chuẩn hoá toạ độ pixel $u = s \cdot u / s, v = s \cdot v / s$ và kiểm tra giới hạn trong ảnh). Bảo đảm `python -m starter.projection` chạy thành công trên cả 3 tập dữ liệu (`data/synthetic`, `data/kitti_mini`, `data/nuscenes_mini_subset`).

### R2. Phát triển Pipeline Stress Test & Sensor Health Metrics (src/)
Tạo các module trong `src/` để thực hiện stress test suy giảm dữ liệu LiDAR bằng ít nhất 4 loại suy giảm (Random Dropout, Range Dropout, Beam Dropout, Gaussian Noise, Calibration Drift) qua ít nhất 3-5 mức cường độ. Tính toán định lượng các chỉ số không cần model:
1. Số điểm LiDAR nằm trong 3D Ground Truth bounding box (Car, Pedestrian, Cyclist)
2. Tỷ lệ điểm nằm trong FOV camera
3. Sensor Health Score / Density metric phát hiện sớm suy giảm chất lượng dữ liệu trước khi hệ thống downstream sụp đổ.
Kết quả phải được lưu tự động thành các file `results/*.csv` và đồ thị trực quan `results/figures/*.png` với seed cố định bảo đảm tính tái lập.

### R3. Phân tích Failure Cases & Điểm tới hạn của Cảm biến
Xác định ít nhất 2 kịch bản failure case đặc thù khi dữ liệu bị suy giảm (ví dụ: mất hoàn toàn điểm phản xạ trên vật thể xa/nhỏ khi beam dropout hoặc range dropout, hiện tượng trôi box do calibration drift hoặc jitter do gaussian noise). Phân loại chính xác vào các lớp debug (Geometry, Preprocess, Sensor/Environment) và lưu hình ảnh minh chứng theo định dạng `results/figures/fail_*.png`.

### R4. Xây dựng Ứng dụng Demo Trực quan (Demo Interface)
Xây dựng một giao diện demo tương tác (Interactive Web/GUI Demo) trong `src/` cho phép:
- Chọn dataset và frame (KITTI, nuScenes, Synthetic)
- Thanh trượt điều chỉnh mức độ suy giảm (Dropout, Noise, Beam count, Calibration Yaw/Pitch/Roll)
- Khung hiển thị trực quan song song: Ảnh Camera với LiDAR point projection màu theo độ sâu, hình chiếu Bird's-Eye-View (BEV), và bảng đo chỉ số suy giảm/số điểm trong GT box real-time.
- Chạy mượt mà trên môi trường CPU sẵn có của `.venv`.

### R5. Hoàn thiện Báo cáo Khoa học REPORT.md & Đạt Chuẩn Submission
Điền đầy đủ và chuẩn xác toàn bộ 6 phần trong `report/REPORT.md`:
- Thông tin học viên: Ngô Xuân Hoàng, MSV: 2A202602597, Lớp: H209
- Claim kỹ thuật rõ ràng, có thể kiểm chứng độc lập
- Bảng số liệu Evidence & link biểu đồ / demo
- Phân tích Failure Cases chi tiết
- Khuyến nghị triển khai ADAS / Robot thực tế
- Hướng dẫn chạy lại từ repo sạch
- Khai báo trung thực việc sử dụng trợ lý AI theo `RULES.md`
- Đảm bảo lệnh `python tools/check_submission.py` trả về PASS 100%.

## Acceptance Criteria

### Hình học & Core Projection
- [ ] Chạy `python -m starter.projection --data-root data/synthetic --frame 000000` sinh ra ảnh overlay hợp lệ trong `results/figures/` không lỗi.
- [ ] Chạy kiểm thử projection trên `data/kitti_mini` và `data/nuscenes_mini_subset` đều cho kết quả chính xác, điểm z_cam phía trước > 0.

### Stress Test & Benchmark
- [ ] Có ít nhất 1 file kết quả `.csv` trong thư mục `results/` ghi lại chi tiết các mức độ biến dạng và metric tương ứng.
- [ ] Có biểu đồ thể hiện rõ đường cong suy giảm (degradation curve) của số điểm trên vật thể và sensor health score trong `results/figures/`.
- [ ] Số liệu chạy lại qua script luôn giữ nguyên giá trị do cố định `seed`.

### Failure Cases
- [ ] Có ít nhất 1 ảnh failure case được lưu trong `results/figures/` có tiền tố `fail_*.png` và được phân tích chi tiết trong báo cáo.

### Giao diện Demo
- [ ] Ứng dụng demo khởi chạy thành công qua một lệnh duy nhất (ví dụ: `python src/app.py` hoặc tương đương), giao diện hiển thị trực quan và phản hồi mượt mà với các thanh điều khiển độ suy giảm cảm biến.

### Báo cáo & Submission Gate
- [ ] File `report/REPORT.md` không còn bất kỳ ký tự placeholder `[ĐIỀN]` nào.
- [ ] Thông tin sinh viên trùng khớp chính xác: Họ tên: Ngô Xuân Hoàng, MSSV: 2A202602597, Lớp: H209.
- [ ] Chạy `.venv/Scripts/python.exe tools/check_submission.py` kết thúc với mã trả về 0 và in `KẾT QUẢ: SẴN SÀNG NỘP`.
