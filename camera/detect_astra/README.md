# Nhận diện pallet bằng Astra RGB-D

`simulation/run_r3_live_trial.sh` tự mở camera và pipeline nhận diện khi tới A. `scripts/run_pallet_perception.py` điều phối YOLO tạo seed và bộ theo dõi RGB-D; `publish_pallet_detection.py` và `publish_pallet_target.py` xuất dữ liệu ROS. `astra_rgbd_stream.py` nhận ảnh và đồng bộ RGB/depth.

YOLO tắt sau ít nhất 3 giây, 3 nhận diện mới nhất quán và xác nhận `/pallet/tracking/status` rằng tracker đã nhận seed của đúng phiên, đang bám ảnh RGB mới. RGB-D tiếp tục xác minh hình học khi đã nhả tài nguyên inference. Tắt YOLO chưa phải phép đo sẵn sàng. R3 chỉ chốt mục tiêu sau 5 mẫu RGB-D mới, ổn định, có mẫu sau lúc YOLO tắt và odom đứng yên ghép đúng thời điểm chụp. Log `RECOGNIZING` ghi xác nhận tracker và lý do loại mẫu trong `capture` để phân biệt chờ camera, odom và chất lượng hình học.

Ba xác nhận YOLO phải cùng nhãn/vùng pallet trong cửa sổ tối đa 5 giây; cho phép khoảng cách tối đa 2,5 giây giữa các kết quả để chịu một inference bị loại. Mỗi kết quả vẫn phải dưới 1 giây lúc nhận, và kết quả mới nhất phải còn mới lúc bàn giao. Driver camera dùng DDS bất đồng bộ để việc gửi ảnh không giữ callback UVC; độ phân giải và 30 FPS giữ theo cấu hình driver.

Các đầu vào cần giữ:

- `config/calibration/astra_pro_rgb_640x480_2026-10-11.yaml`: nội tại RGB.
- `config/calibration/astra_pro_depth_x_skew_preview.yaml`: hiệu chỉnh depth dùng chung với đám mây vật cản.
- `config/calibration/pallet_target_bias_robot_2026_11_11.yaml`: hiệu chỉnh phép đo pallet hiện tại.
- `models/best_v3.pt`: model gốc để xuất NCNN.
- `data/runtime_models/best_v3_ncnn_model/`: model NCNN đang chạy, được Git bỏ qua.
- `.venv/`: môi trường Python đang dùng trên Pi.
- `ros2_ws/install/`: kiểu bản tin `detected_pallets_interfaces`.
- `data/alignment_robot_ws/install/`: phần cứng ROS, plugin Nav2 và driver LiDAR đã build.

Build phần cứng và LiDAR từ mã nguồn trong dự án:

```bash
bash /home/admin/tai_robot_one/robot_amr/camera/detect_astra/scripts/build_pallet_alignment_robot.sh
```

`export_pallet_ncnn.py` xuất lại model; `record_pallet_calibration.py` phục vụ đo hiệu chỉnh. Chỉ sử dụng các công cụ này khi có chủ đích thay model/hiệu chỉnh. Hướng dẫn vận hành: [README dự án](../../README.md).
