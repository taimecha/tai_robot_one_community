# Nav2 GitHub + Astra Pro trên Raspberry Pi 5

Nền Nav2 bắt đầu từ commit `381bbe6` của `taimecha/tai_robot_one`. Mã camera
hiện dùng `taimecha/detect_astra` commit
`fbb2d113753a76ae2c8afbbd788be2a493527b00` trong `camera/detect_astra/`.
Launch `astra_obstacles.launch.py` lấy cloud Depth hiệu chỉnh rồi chuyển sang
`/camera/obstacle_scan`; chi tiết tại [astra_detect_astra_scan.md](astra_detect_astra_scan.md).
Behavior tree thêm
một lần xóa global/local costmap cho mỗi goal khi không tìm được đường, đợi cả
hai costmap nhận dữ liệu cảm biến mới rồi thử lại. Nó không xóa costmap định kỳ khi xe đang
đi theo một đường hợp lệ. Bộ lọc
dây camera thử trước đó không có trong bản này.

Camera thật: Depth đăng ký theo RGB ở 640×480 @ 30 fps, RGB 640×480 @ 15 fps.
Node cloud dùng hiệu chuẩn RGB và hệ số sửa Depth theo cột ảnh, lấy mẫu cách
2 pixel ở tối đa 15 Hz; YOLO không chạy khi điều hướng. Scan dùng bước góc
0,5°, lọc theo cao độ 6–75 cm và tầm 0,6–6 m. Góc camera trong URDF là +20°.
Cloud gốc của driver được tắt để giảm tải CPU.
Hai camera obstacle layer chấp nhận tia không có vật cản để xóa dấu camera cũ;
lớp bản đồ tĩnh và LiDAR độc lập.

Giá trị mặc định trên Pi 5:

| Thành phần | Giá trị cấu hình | Đã đo khi chỉ chạy camera |
| --- | ---: | ---: |
| `/camera/obstacle_scan` | tối đa 15 Hz | khoảng 15 Hz |
| Local costmap update | 10 Hz | chờ thử cùng Nav2 |
| Local costmap publish | 4 Hz | chờ thử cùng Nav2 |
| Global costmap update | 2,5 Hz | chờ thử cùng Nav2 |
| Global costmap publish | 1 Hz | chờ thử cùng Nav2 |

Pipeline mới đã được thử riêng camera, TF và bộ chuyển cloud trên Pi 5;
chưa đo tải khi chạy cùng Nav2. Với 10 Hz local costmap, độ dịch
chuyển ở tốc độ 0,28 m/s vào khoảng 2,8 cm mỗi lần cập nhật, gần độ phân giải
2,5 cm của costmap. Giá trị này chưa được xác nhận khi robot di chuyển.

Mở Nav2 trên Pi 5 sau khi kiểm tra không còn phiên cũ:

```bash
source /opt/ros/jazzy/setup.bash
source /home/admin/tai_robot_one/tai_robot_one/install/local_setup.bash
source /home/admin/tai_robot_one/tai_robot_one/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
ros2 launch tai_robot_one nav_real.launch.py \
  map_file:=/home/admin/tai_robot_one/tai_robot_one/ros2_ws/maps/tai_map_real_edited.yaml \
  start_navigation:=true use_camera:=true use_cm029_teleop:=false
```

Các tham số dòng lệnh có thể đổi mà không sửa Nav2:
`camera_scan_hz:=15.0`, `camera_cloud_pixel_stride:=2`, `local_costmap_update_hz:=10.0`,
`local_costmap_publish_hz:=4.0`, `global_costmap_update_hz:=2.5`, và
`global_costmap_publish_hz:=1.0`. Kiểm tra bằng `ros2 topic hz /camera/obstacle_scan` và
`ros2 param get /local_costmap/local_costmap update_frequency`;
`ros2 param get /global_costmap/global_costmap update_frequency`.

RViz trên laptop nhận `/camera/obstacle_scan` nếu cùng ROS domain. File
`config/nav_robot.rviz` của repo có display màu vàng sẫm tên `Camera Obstacle Scan`.
Nếu laptop dùng bản RViz config riêng, mở file mới này hoặc thêm display
LaserScan với topic trên và QoS Best Effort.
