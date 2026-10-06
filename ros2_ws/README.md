# TAI Robot One

## Mô phỏng camera độ sâu Astra Pro

Mô hình Gazebo mặc định dùng cấu hình tiết kiệm tài nguyên 320 x 240, 10 Hz,
tầm đo từ 0.6 m đến 8.0 m. Góc nhìn và hình học 3D vẫn giống camera Astra Pro;
lượng pixel xử lý mỗi giây giảm 12 lần so với 640 x 480, 30 Hz. Camera nằm ở
độ cao 0.8 m và nhìn theo hướng tiến của robot. Một lần render tạo ra bốn
topic ROS 2:

| Topic | Kiểu message | Nội dung |
| --- | --- | --- |
| `/camera/image` | `sensor_msgs/msg/Image` | Ảnh màu RGB |
| `/camera/depth_image` | `sensor_msgs/msg/Image` | Khoảng cách theo từng pixel, đơn vị mét |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Ma trận nội tại dùng để chiếu pixel sang tia 3D |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | Mây điểm XYZ có màu, dựng từ RGB + depth |

Các topic mô phỏng mang `frame_id: camera_link` vì Gazebo Harmonic xuất trực
tiếp tọa độ mây điểm theo quy ước X tiến, Y trái, Z lên. Không được đổi riêng
header thành `camera_link_optical`: thao tác đó không xoay dữ liệu XYZ và sẽ
làm nền nhà dựng đứng trong RViz.

Build package từ thư mục workspace:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon --log-base log/tai_robot_one_community build \
  --base-paths src/tai_robot_one_community/ros2_ws \
  --build-base build/tai_robot_one_community \
  --install-base install \
  --symlink-install
source install/setup.bash
```

`--base-paths` là cần thiết trong workspace hiện tại vì đang có nhiều thư mục
cùng khai báo package tên `tai_robot_one`.

Package cài một environment hook để `source install/setup.bash` tự chọn
`rmw_fastrtps_cpp`. Fast DDS đã được kiểm tra với Gazebo, `ros2_control`, các
topic cảm biến và cấu hình WSL mirrored hiện tại. Kiểm tra middleware đang
dùng bằng:

```bash
echo "$RMW_IMPLEMENTATION"
```

Kết quả mặc định phải là `rmw_fastrtps_cpp`. Tất cả terminal chạy Gazebo,
teleop và lệnh kiểm tra ROS phải source cùng workspace này.

Khởi động Gazebo và RViz2:

```bash
ros2 launch tai_robot_one gazebo_sim.launch.py
```

## Điều khiển mô phỏng bằng điện thoại

`gazebo_sim.launch.py` mặc định khởi động node `phone_teleop` trên cổng TCP
`8080`. Điện thoại không cần cài ứng dụng ROS: chỉ cần nối cùng Wi-Fi với
laptop và mở địa chỉ được node in ra terminal. Giao diện gửi `/cmd_vel`, gửi
setpoint càng vào `/lift_controller/commands`, đồng thời hiển thị phản hồi thực
từ `/odom` và `/joint_states`.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch tai_robot_one gazebo_sim.launch.py
```

Nếu WSL dùng mirrored networking, mở trên điện thoại:

```text
http://<IPv4-của-laptop-Windows>:8080
```

Nếu WSL đang dùng NAT mặc định và điện thoại không mở được trang, lấy IP WSL
bằng `wsl hostname -I`, sau đó chạy PowerShell **Run as Administrator** trên
Windows (thay `<WSL_IP>` bằng địa chỉ vừa lấy):

```powershell
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=8080 connectaddress=<WSL_IP> connectport=8080
New-NetFirewallRule -DisplayName "TAI Robot Phone Teleop 8080" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8080 -Profile Private
```

Sau đó dùng `ipconfig` lấy IPv4 của Wi-Fi Windows và mở
`http://<IPv4-Windows>:8080` trên điện thoại. IP WSL có thể thay đổi sau khi
khởi động lại WSL; khi đó cần cập nhật rule `portproxy`.

Các đặc tính an toàn của giao diện mô phỏng:

- Nút hướng phải được giữ liên tục; thả tay sẽ gửi vận tốc 0.
- Trình duyệt gửi heartbeat mỗi 100 ms. Node dừng xe nếu quá 300 ms không có
  lệnh mới; `diff_drive_controller` còn có timeout riêng 250 ms.
- Node chặn tốc độ tịnh tiến tuyệt đối ở `0.50 m/s` và tốc độ quay ở
  `1.20 rad/s`, kể cả khi client gửi giá trị lớn hơn.
- Chuyển ứng dụng, ẩn trang hoặc mất Wi-Fi đều dẫn đến dừng bằng watchdog.
- Nút `DỪNG` trên web chỉ là dừng phần mềm; không thay thế E-stop cứng khi đưa
  lên robot thật.

SLAM cũng dùng điện thoại theo mặc định và không còn tự mở cửa sổ bàn phím:

```bash
ros2 launch tai_robot_one slam_sim.launch.py
```

Chỉ bật bàn phím dự phòng khi không dùng điện thoại, để tránh hai nguồn đồng
thời phát `/cmd_vel`:

```bash
ros2 launch tai_robot_one slam_sim.launch.py use_teleop:=true use_phone_teleop:=false
```

Có thể đổi cổng hoặc tắt web teleop trong launch mô phỏng:

```bash
ros2 launch tai_robot_one gazebo_sim.launch.py phone_teleop_port:=8081
ros2 launch tai_robot_one gazebo_sim.launch.py use_phone_teleop:=false
```

Nếu Gazebo đã chạy mà web teleop đã bị tắt, có thể mở riêng:

```bash
ros2 launch tai_robot_one phone_teleop.launch.py port:=8080
```

Trên laptop 8 GB RAM, không nên mở thêm VMware cùng lúc. Khi chỉ cần kiểm tra
thế giới Gazebo mà chưa cần RViz, có thể giảm thêm tải bằng lệnh:

```bash
ros2 launch tai_robot_one gazebo_sim.launch.py use_rviz:=false
```

Khi tập trung xem camera trong RViz, nên tắt riêng giao diện 3D của Gazebo;
mô phỏng và toàn bộ topic camera vẫn chạy:

```bash
ros2 launch tai_robot_one gazebo_sim.launch.py headless:=true
```

RViz2 được cấu hình sẵn để hiển thị đồng thời robot, ảnh RGB, ảnh depth và
mây điểm. `Fixed Frame` phải là `odom`. Trong danh sách `Displays`, ba mục cần
bật là:

- `Astra Pro RGB - 0.800 m`
- `Astra Pro Depth - display 0.6 to 4.5 m`
- `Astra Pro RGB-D Point Cloud`

Ảnh depth sáng/tối biểu diễn khoảng cách chứ không phải màu vật thể. Mây điểm
là cách dễ nhất để kiểm tra hình học 3D: kéo góc nhìn RViz quanh robot để thấy
các mặt kệ, pallet và tường nằm đúng vị trí trong không gian.

Display depth dùng thang màu cố định 0.6--4.5 m dù cảm biến vẫn đo tới 8 m.
Không bật `Normalize Range` cho topic `32FC1`: các tia không gặp vật mang giá
trị vô cực và có thể làm phép tự chuẩn hóa của RViz hiển thị toàn ảnh màu đen.

Kiểm tra dữ liệu khi không thấy hình:

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/depth_image
ros2 topic hz /camera/points
ros2 topic echo /camera/camera_info --once
```

Robot được truyền trực tiếp cho Gazebo qua parameter của tiến trình `create`,
không phụ thuộc vào việc node này có khám phá kịp topic
`/robot_description` hay không. Giữ `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`
khi cần kết nối laptop với Raspberry Pi; chỉ đặt `LOCALHOST` cho một phiên mô
phỏng hoàn toàn nằm trên một máy.

## ros2_control, SLAM Toolbox và Nav2

Cài các dependency ROS 2 Jazzy trước khi build package này:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-robot-localization \
  ros-jazzy-rmw-fastrtps-cpp
```

Gazebo và robot thật dùng cùng contract lệnh: bốn joint bánh độc lập được ghép
thành hai phía skid-steer bởi `base_controller`; setpoint càng luôn đi vào
`/lift_controller/commands`. Trong mô phỏng, node
`lift_position_controller` đổi setpoint vị trí thành lực cho controller tầng
thấp `lift_effort_controller`; trên robot thật, `lift_controller` gửi trực
tiếp vị trí cho ESP32 càng. Vận tốc xe bị giới hạn ở `0.50 m/s`. Các lệnh
chính:

`base_controller` phát odometry bánh thô ở `/base_controller/odom` và không
phát TF động. EKF `robot_localization` là nguồn duy nhất phát `/odom` cùng TF
`odom -> base_footprint`: trong Gazebo nó kết hợp odometry bánh với `/imu`; ở
robot thật nó có thể chạy wheel-only cho tới khi đường phần cứng IMU được chốt.
Thiết kế này tránh hai node cùng phát một TF.

```bash
# Xe: Twist được đóng dấu rồi chuyển cho diff_drive_controller.
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20}, angular: {z: 0.0}}"

# Càng: vị trí tuyệt đối theo mét.
ros2 topic pub --once /lift_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.10]}"
```

Lệnh xe gửi một lần sẽ hết hạn sau khoảng `0.25 s`; `cmd_vel_stamper` cố ý
không phát lại lệnh cũ. Lệnh càng là một setpoint vị trí nên được controller
giữ cho tới khi đạt vị trí hoặc có setpoint mới. Khi chạy thật, backend vẫn gửi
frame serial mới ở 50 Hz; mất controller manager/USB thì watchdog ESP32 dừng
actuator trong 300 ms.

Các luồng mô phỏng:

```bash
# Chỉ Gazebo + ros2_control
ros2 launch tai_robot_one gazebo_sim.launch.py

# Lập bản đồ bằng SLAM Toolbox
ros2 launch tai_robot_one slam_sim.launch.py

# Điều hướng trong khi đang mapping
ros2 launch tai_robot_one slam_nav_sim.launch.py

# Điều hướng trên map đã lưu + AMCL; bắt buộc đường dẫn tuyệt đối
ros2 launch tai_robot_one nav_sim.launch.py \
  map_file:=/home/<user>/ros2_ws/maps/warehouse.yaml
```

`nav_sim.launch.py`, `slam_nav_sim.launch.py` và `nav_real.launch.py` mặc định
dùng chung cây điều hướng kiểm tra khoảng trống và tuyến thẳng/cung tròn.
Hai launch Nav2 mô phỏng đưa lệnh đã lọc `/cmd_vel_safe` tới bộ điều khiển
bánh xe. Vùng dừng khẩn `VelocityStop` được tắt riêng trong mô phỏng;
`FootprintApproach` và kiểm tra va chạm của controller vẫn hoạt động.
Nav2 so sánh chiều dài các tuyến trực tiếp, XY, cung liên tục `SE2Arc` và
lattice có thể xoay tại chỗ sau kiểm tra toàn bộ thân xe. Tuyến ngắn nhất
được thử trước; chỉ dùng tuyến dài hơn khi tuyến ngắn bị chặn hoặc controller
không thực hiện được. Độ mượt chỉ phân hạng khi chiều dài gần như bằng nhau
(sai số số học 0,1 mm). Khi các tuyến dự phòng không đi được,
cây hành vi chờ dữ liệu mới và thử lại thêm tối đa hai lượt trước khi báo thất bại.
Trước khi nhận đường vòng dài, Nav2 so sánh thêm các phương án tiến một đoạn
ngắn rồi rẽ (15–120 cm), gồm cả đoạn tiến trong chiều dài và kiểm tra thân xe.
Các phương án này được xét ngay từ đầu, thay vì đợi đường vòng chạy lỗi.
Bộ điều khiển báo trạng thái chặn xoay lặp để cây hành vi bỏ qua các lượt thử
đường vô ích tại cùng vị trí và chuyển sang thao tác thoát đã kiểm tra.
Thử trên Gazebo với bản đồ kho mô phỏng của repo (không dùng bản đồ xe thật):

```bash
cd /home/tai/tai_robot_one_community
ros2 launch tai_robot_one nav_sim.launch.py \
  map_file:="$PWD/maps/tai_warehouse.yaml" \
  use_phone_teleop:=false
```

Trong RViz, chọn **Nav2 Goal** để thử một đích thẳng phía trước, một đích
chéo phía trước có hướng cuối theo tiếp tuyến cung, rồi một đích có vật cản
để quan sát đường dự phòng. Chỉ đường thẳng/cung tròn đã qua kiểm tra toàn bộ
footprint mới được so sánh và gửi cho controller. Không có thứ tự cố định
ưu tiên cung tròn hay xoay tại chỗ: ưu tiên chiều dài tuyến khả thi.

Kiểm tra tự động việc đi từ gần `(0, 0)` tới cạnh kệ `(1.83, -1.16)` rồi
quay về, với hướng cuối ngang (`yaw=0`) và giới hạn 180 giây mỗi goal:

```bash
ros2 run tai_robot_one test_nav_round_trip --scenario recent_goals
```

Thử chuỗi goal gần nhất: lên `(0, 2)`, sang `(.99, 2.02)`, rồi về `(0, 0)`,
với hướng cuối `yaw=0`:

```bash
ros2 run tai_robot_one test_nav_round_trip --scenario latest_short_route
```

Goal từng xoay lặp rồi bị hủy tại khu vực phía bắc có bài thử riêng. Mở mô
phỏng với `spawn_x:=0.1296116 spawn_y:=2.4149426 spawn_yaw:=0.4636476`, rồi chạy:

```bash
ros2 run tai_robot_one test_nav_round_trip --scenario latest_turn_escape
```

Bài thử này dùng đích `(-1.731213, 1.048116)`, hướng cuối `-1.571597 rad`
đọc từ tuyến đã bị hủy. Bộ bám đường bỏ qua điểm xoay đã hoàn thành; khi
giới hạn xoay yêu cầu dịch chuyển, cây hành vi vẫn cho phép tiến/lùi đã kiểm tra.

Hai goal cạnh kệ từng lỗi có bài thử `latest_rack_pair`. Mở mô phỏng với
`spawn_x:=0.42 spawn_y:=0.17 spawn_yaw:=0.0`, rồi chạy:

```bash
ros2 run tai_robot_one test_nav_round_trip --scenario latest_rack_pair
```

Goal đầu `(1.650, -1.106)` dùng hướng sang phải `yaw=0` do người dùng xác nhận;
goal thứ hai `(1.481171, -1.186490)` dùng hướng `0.102340 rad` từ tuyến cũ.
Để thử riêng goal thứ hai từ vị trí từng bị lỗi, mở mô phỏng với
`spawn_x:=0.90 spawn_y:=-0.44 spawn_yaw:=-1.11`, rồi chạy:

```bash
ros2 run tai_robot_one test_nav_round_trip --scenario latest_rack_second
```

Khi cần tìm thêm tuyến, hệ thống xét cả điểm chỉnh hướng cách đích
25/50/75 cm rồi đi thẳng vào, với kiểm tra toàn bộ thân xe và chiều dài tuyến.
FollowPath thành công trong recovery kết thúc goal ngay; tuyến bị chặn khi
đang chạy được tính lại từ vị trí mới.

Ở vùng trống, ứng viên `Direct` gồm xoay tại chỗ theo hướng đích, đi thẳng
rồi canh hướng cuối. Nó được so chiều dài với `DirectArc` và các planner;
độ lệch hướng xuất phát không còn loại đường thẳng khỏi danh sách.
Hai goal qua lại ở phía bắc có bài thử `latest_open_pair`. Mở mô phỏng với
`spawn_x:=2.01 spawn_y:=2.96 spawn_yaw:=0.0`, rồi chạy:

```bash
ros2 run tai_robot_one test_nav_round_trip --scenario latest_open_pair
```

Lệnh này gửi goal và làm xe mô phỏng di chuyển; không đặt thêm goal trong RViz
trong lúc kiểm tra. Tọa độ này chỉ dùng cho bản đồ kho mô phỏng của repo.

Trong hai launch Nav2 mô phỏng, point cloud camera được chuyển một lần thành
`/camera/obstacle_scan` với tốc độ mục tiêu 10 Hz. Scan ảo thấp này giúp costmap và Collision
Monitor phát hiện cột kệ cao `0.680 m` và pallet có tải trong vùng nhìn mà lidar
đặt ở cao độ `0.385 m` có thể quét vượt qua. LiDAR mô phỏng và xe thật cùng
dùng transform từ `base_footprint`: `x=-0.2115 m`, `y=0`, `z=0.385 m`,
`yaw=-2°`. Camera mô phỏng ở cao độ `0.8 m`, có góc pitch URDF `+20°` để
quét xuống phía trước; camera xe thật cũng dùng góc pitch `+20°` theo bản
preview của `detect_astra_pro`.
Mô phỏng bật camera theo mặc định; `/camera/obstacle_scan` cấp vật cản
cho cả costmap và Collision Monitor. Camera không bảo đảm thấy pallet trống
cao khoảng `35 mm` ở vùng gần dưới tầm sâu `0.6 m`, vật phía sau hoặc ngoài
FOV; đây không phải cảm biến an toàn. SLAM Toolbox lập occupancy map từ LiDAR
`/scan_filtered`; camera scan là lớp vật cản động khi điều hướng.

LiDAR Gazebo lọc thêm tia đơn lẻ không có tia liền kề ở khoảng cách tương tự
(chênh lệch tối đa 0.10 m), để điểm nhiễu không tồn tại như vật cản giả trong
costmap. Tia bị loại là `NaN`, không xóa vật cản phía sau. Bộ lọc này chỉ bật
trong mô phỏng; vật cản chỉ xuất hiện trên một tia có thể bị bỏ sót.

Trong terminal đã source cùng workspace, kiểm tra camera đang quét bằng
`ros2 topic hz /camera/obstacle_scan`; xem các tia đo bằng
`ros2 topic echo --once /camera/obstacle_scan`.

Chỉ sau khi các gate mô phỏng đạt mới dùng
`real_hardware.launch.py` trên Raspberry Pi 5. Cấu hình hai ESP32 hiện tại mở
`/dev/tai_drive` cho bốn bánh và `/dev/tai_imu` cho BNO055 + càng nâng. Đặt
`use_lift:=false` để bridge dùng chung cổng IMU/càng, không phải để khóa càng.
Hướng dẫn chạy Pi, RViz và teleop laptop nằm trong
`docs/real_hardware.md`. Checklist nghiệm thu đầy đủ nằm ở
`../docs/SIMULATION_TO_REAL_ROADMAP.md`.

## Project BNO055 và bridge GitHub

Firmware production ESP32 số 2 nằm tại
`esp32_firmware/Projects/bno055_imu_monitor`. Project không có Wi-Fi/Web/BLE;
nó phát quaternion, gyro, acceleration, calibration và trạng thái càng qua USB
115200 baud. IMU chạy 50 Hz, còn telemetry càng chạy 10 Hz.

Package `ros2_ws/src/esp32_imu_bridge` đã được đối chiếu với repository
`taimecha/agv_robot_pi5` rồi tích hợp thêm port parameter, reconnect, kiểm tra
NaN/quaternion/sequence và covariance. Với một ESP32 IMU standalone, chạy:

```bash
ros2 launch tai_robot_one imu_bridge.launch.py port:=/dev/tai_imu
ros2 topic hz /imu
```

IMU bridge hiện được bật tự động trong `real_hardware.launch.py`. Không trỏ
bridge vào `/dev/tai_drive`; cổng đúng là `/dev/tai_imu`.

## Tay cầm CM029 qua Wi-Fi

Receiver 2.4 GHz cắm vào laptop; chỉ `joy_node` chạy trên laptop:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run joy joy_node --ros-args -p device_id:=0 -p deadzone:=0.05 \
  -p autorepeat_rate:=20.0 -p coalesce_interval_ms:=1
```

Pi nhận `/joy` qua DDS và chạy teleop cùng launch phần cứng. Lần thử đầu phải
kê cả bốn bánh khỏi sàn và giảm giới hạn:

```bash
ros2 launch tai_robot_one real_hardware.launch.py \
  use_cm029_teleop:=true use_rviz:=false use_lift:=false \
  cm029_linear_speed:=0.10 cm029_angular_speed:=0.25
```

Node luôn khởi động khóa. Nhả rồi nhấn Start để mở khóa; B gửi zero, giữ càng
theo feedback `lift_joint` và khóa lại. Mất `/joy` quá 0.30 giây cũng khóa;
reconnect không tự mở khóa. Không chạy phone teleop, keyboard teleop hoặc Nav2
cùng lúc. B là stop phần mềm, không thay thế E-stop vật lý.
