# Phần cứng Pi 5

| Thiết bị | Cổng ổn định | Vai trò |
| --- | --- | --- |
| ESP32 bánh | `/dev/tai_drive` | PID và encoder bốn bánh |
| ESP32 BNO055/càng | `/dev/tai_imu` | IMU và điều khiển càng |
| LiDAR | `/dev/tai_lidar` | Định vị, bản đồ và vật cản |
| Astra | USB | RGB-D nhận diện pallet và vật cản |

Các quy tắc cổng nằm trong `udev/99-tai-robot-usb.rules`. `description/robot.urdf.xacro` mặc định dùng `tai_robot_one/TaiRobotSerialSystem`; `config/ros2_controllers_real.yaml` cấu hình controller thật, `config/ekf.yaml` cấu hình odom/EKF.

`launch/real_hardware.launch.py` mở phần cứng; `launch/nav_real.launch.py` phục vụ vận hành Nav2 riêng và yêu cầu đường dẫn bản đồ tuyệt đối. Luồng A→C hiện tại dùng các launch R3 trong `camera/detect_astra/launch/` và được điều phối bởi `simulation/run_r3_live_trial.sh`.

Build và lệnh vận hành được ghi tại [README dự án](../../README.md).
