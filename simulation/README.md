# Luồng chạy robot thật R3

Tên thư mục được giữ để các lệnh đang dùng trên Pi tiếp tục hoạt động. Thư mục này chứa các mô-đun cần cho robot thật.

| Thành phần | Nhiệm vụ |
| --- | --- |
| `run_r3_live_trial.sh` | Điều phối định vị, tới A, nhận diện và chạy tới C |
| `run_r3_localization.sh`, `stop_r3_localization.sh`, `status_r3_localization.sh` | Mở, đóng và xem trạng thái định vị |
| `save_r3_a.sh`, `r3_recognition_goal.py` | Lưu A khi được yêu cầu và điều hướng tới A |
| `r3_global_*`, `r3_amcl_*`, `r3_stationary_localization.py`, `r3_scan_map_check.py` | Khởi tạo và kiểm tra định vị LiDAR/bản đồ |
| `r3_control_odom.py`, `r3_localization_contract.py` | Kết hợp odom/EKF với hiệu chỉnh định vị |
| `r3_live_prepare.py`, `r3_approach_planner.py`, `r3_tracking.py`, `r3_core.py` | Khóa mục tiêu RGB-D, lập và kiểm tra quỹ đạo, bám đường |
| `r3_real_trial.py`, `r3_real_obstacle_filter.py`, `r3_pallet_obstacles.py` | Thực thi chuyển động và kiểm tra vật cản |
| `r3_lift_control.py` | HOME càng và điều khiển nâng với phản hồi ESP32; hỗ trợ chế độ `--pickup` của lệnh chạy chính |
| `r3_insertion_clearance.py` | Kiểm tra khe hở cả hai càng và hình học pallet trong đoạn C→B |
| `r3_floor_aligned_cloud.py` | Đám mây điểm hiệu chỉnh mặt sàn cho dữ liệu vật cản |
| `r3_render_real_result.py`, `r3_navigation_telemetry.py` | Xuất kết quả và dữ liệu của lượt chạy |
| `open_r3_laptop_rviz.sh` | Mở RViz trên laptop Ubuntu |

`r3_tracking.predict_tracking` là phép dự đoán chuyển động để kiểm tra đường trước khi chạy, sử dụng cùng bộ bám đường. Nó vẫn cần cho chương trình thật; không phải kết quả đo độc lập.

`RGBD_READY_PLANNING` xác nhận đã có đủ phép đo mới trước khi tìm đường. Bộ lập đường ưu tiên đoạn thẳng 25 cm trước C; nếu không đủ chỗ bẻ lái, nó thử 20, 15 rồi 12 cm, giữ giới hạn bán kính quay, kiểm tra toàn bộ càng và đoạn thẳng C→B. `PLAN_READY` ghi đoạn thẳng được chọn và thời gian lập đường. `PLAN_BLOCKED` (mã thoát 5) là lỗi lập đường và không khởi động lại YOLO.

Lệnh đầy đủ và cấu hình RViz: [README dự án](../README.md). Dữ liệu đầu vào: [r3/README.md](r3/README.md).
