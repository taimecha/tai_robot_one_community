# Nav2 GitHub + Astra Pro trên Raspberry Pi 5

Nền Nav2 là commit `381bbe6` của `taimecha/tai_robot_one`. Mã camera được
chép nguyên từ commit `a41f409` của `taimecha/detect_astra_pro` vào
`camera/detect_astra_pro/`; launch `astra_obstacles.launch.py` lấy thông số
depth và quét vật cản từ `pallet_obstacle_preview.launch.py`. Không thay đổi
planner, controller hoặc bộ lọc LiDAR của nền GitHub. Behavior tree chỉ thêm
một lần xóa global/local costmap cho mỗi goal khi không tìm được đường, đợi cả
hai costmap nhận dữ liệu cảm biến mới rồi thử lại. Nó không xóa costmap định kỳ khi xe đang
đi theo một đường hợp lệ. Bộ lọc
dây camera thử trước đó không có trong bản này.

Camera thật: depth 320×240 @ 30 fps, chỉ bật depth và point cloud; RGB/YOLO
không chạy khi điều hướng. Điểm 3D được đổi trực tiếp sang scan gồm 61 tia,
lọc theo cao độ 6–75 cm và tầm 0,6–6 m. Góc camera trong URDF là +20° như
preview. Chỉ scan nhỏ được giới hạn tần số; không giới hạn point cloud lớn.
Hai camera obstacle layer chấp nhận tia không có vật cản để xóa dấu camera cũ;
lớp bản đồ tĩnh và LiDAR độc lập.

Giá trị mặc định trên Pi 5:

| Thành phần | Giá trị cấu hình | Đã đo khi chỉ chạy camera |
| --- | ---: | ---: |
| `/camera/obstacle_scan` | tối đa 20 Hz | khoảng 16,6 Hz |
| Local costmap update | 10 Hz | chờ thử cùng Nav2 |
| Local costmap publish | 4 Hz | chờ thử cùng Nav2 |
| Global costmap update | 2,5 Hz | chờ thử cùng Nav2 |
| Global costmap publish | 1 Hz | chờ thử cùng Nav2 |

Trước tích hợp lại, một phiên Nav2 trên Pi đo được camera scan khoảng 19 Hz
khi không giới hạn, nhưng việc phát nhiều scan hơn mức costmap cần làm tăng
tải. Ở phép thử camera riêng, đặt bộ giới hạn 15 Hz chỉ đạt khoảng 11 Hz do
nhịp depth 30 fps; đặt 20 Hz đạt 16,6 Hz. Với 10 Hz local costmap, độ dịch
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
`camera_scan_hz:=20.0`, `local_costmap_update_hz:=10.0`,
`local_costmap_publish_hz:=4.0`, `global_costmap_update_hz:=2.5`, và
`global_costmap_publish_hz:=1.0`. Nếu CPU gần đầy, thử `15.0`, `8.0`, `2.0`
theo đúng thứ tự đó. Kiểm tra bằng `ros2 topic hz /camera/obstacle_scan` và
`ros2 param get /local_costmap/local_costmap update_frequency`;
`ros2 param get /global_costmap/global_costmap update_frequency`.

RViz trên laptop nhận `/camera/obstacle_scan` nếu cùng ROS domain. File
`config/nav_robot.rviz` của repo có display màu vàng sẫm tên `Camera Obstacle Scan`.
Nếu laptop dùng bản RViz config riêng, mở file mới này hoặc thêm display
LaserScan với topic trên và QoS Best Effort.
