# ROS cho robot thật

Package `tai_robot_one` gồm serial hardware ros2_control, mô hình hình học/TF, cấu hình encoder–IMU EKF, AMCL/Nav2, vật cản Astra, SLAM và điều khiển tay. Mã driver Astra, LiDAR và ESP32 IMU được giữ trong `src/` cùng nguồn gốc/giấy phép của chúng.

Build phần cứng và driver LiDAR:

```bash
bash /home/admin/tai_robot_one/robot_amr/camera/detect_astra/scripts/build_pallet_alignment_robot.sh
```

Lệnh này build vào `camera/detect_astra/data/alignment_robot_ws/install/`; không mở cổng serial và không chạy bánh. `ros2_ws/install/` là overlay Astra riêng đang được dùng trên Pi. Không xóa các thư mục `install/` khi chỉ dọn log hay kết quả.

Bản đồ hiện tại: `maps/tai_map_real_edited.yaml` và `.pgm`. Cấu hình RViz laptop: `config/r3_laptop.rviz`. [Hướng dẫn phần cứng](docs/real_hardware.md) và [lệnh vận hành đầy đủ](../README.md).
