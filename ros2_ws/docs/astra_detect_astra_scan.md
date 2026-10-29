# detect_astra trong Nav2 trên Pi 5

Mã nguồn, cấu hình và model từ `taimecha/detect_astra` commit
`fbb2d113753a76ae2c8afbbd788be2a493527b00` nằm ở
`camera/detect_astra/` (bỏ các ảnh chụp thử). Bản sao nguyên gốc của node tạo
cloud nằm tại `ros2_ws/scripts/publish_rgb_calibrated_cloud.py` để package
`tai_robot_one` cài đặt được. File launch dùng trên xe là
`ros2_ws/launch/astra_obstacles.launch.py`.

Repo nguồn không trực tiếp phát `/camera/obstacle_scan`. Pipeline mới:

```text
Astra: Depth đăng ký theo RGB, 640×480
  -> /camera/depth/image_raw + /camera/color/camera_info
  -> /camera/depth/points_nav_corrected (node detect_astra, tối đa 15 Hz)
  -> pointcloud_to_laserscan (cao độ 0,06–0,75 m)
  -> /camera/obstacle_scan
  -> Nav2 costmap và collision monitor
```

Calibration RGB và hệ số sửa lệch Depth theo cột ảnh nằm trong
`ros2_ws/config/calibration/`. `camera_scan_hz:=15.0` đổi tốc độ tối đa của
cloud và scan; `camera_cloud_pixel_stride:=2` đổi khoảng lấy mẫu pixel.
Point cloud gốc của driver bị tắt để giảm tải CPU. Node YOLO/pallet của
`detect_astra` được giữ trong thư mục dự án nguồn nhưng không được khởi động
cùng Nav2.

Build trong đúng workspace đang mở:

```bash
cd /home/admin/tai_robot_one/tai_robot_one/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select tai_robot_one --symlink-install
source install/setup.bash
```

Không chạy đồng thời camera launch cũ hoặc script mở camera của `detect_astra`.
Kiểm tra `ros2 topic hz /camera/obstacle_scan` và hình vật cản trong RViz trước
khi cho xe chạy. Hệ số sửa Depth của repo nguồn là thử nghiệm từ mặt sàn; chưa
được chứng minh là hiệu chuẩn khoảng cách tuyệt đối cho toàn ảnh.
