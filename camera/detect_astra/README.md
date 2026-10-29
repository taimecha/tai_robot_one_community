# Astra Pallets

Nhận diện và phân đoạn pallet từ camera RGB Astra Pro bằng YOLO. Model có một lớp `pallets` và được lưu tại `models/best_v2.pt`.

## Cài đặt

Dự án chạy trên Linux hoặc WSL2 có giao diện đồ họa, sử dụng camera V4L2 tại `/dev/video0`. Môi trường đã chạy dùng Python 3.12.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Chương trình tự dùng CUDA nếu PyTorch nhận GPU, nếu không sẽ dùng CPU.

## Kết nối camera trên WSL2

Cắm camera vào máy. Với usbipd-win đã cài trên Windows, kiểm tra BUSID của `Astra Pro HD Camera` bằng PowerShell:

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

Lệnh `bind` cần PowerShell chạy với quyền quản trị và chỉ cần khi thiết bị chưa được chia sẻ. BUSID có thể thay đổi khi đổi cổng USB.

Kiểm tra `/dev/video0` đã xuất hiện trong WSL và người dùng có quyền truy cập camera. Nếu camera dùng đường dẫn khác, sửa biến `DEVICE` trong script tương ứng.

## Chạy

Chạy từng script riêng để tránh tranh chấp camera:

```bash
# Kiểm tra camera: 640x480, 15 FPS
python src/test_camera.py

# Nhận diện pallet: 640x480, 30 FPS, ngưỡng confidence 0.70
python src/test_pallets.py

# Chụp ảnh dữ liệu: 1280x720, 30 FPS
python src/capture_rgb.py
```

Nhấn **Q** trong cửa sổ camera để thoát. Với `capture_rgb.py`, nhấn **S** để lưu ảnh vào `data/rgb/` (thư mục tự tạo). Ảnh chụp và môi trường Python không được đưa vào Git.

## Xem khoảng cách bằng RViz trên máy ROS 2 Jazzy

Để xem point cloud mà `yolo_depth.py` dùng, chạy Astra với `depth_registration=true`. Lệnh bên dưới dùng ROS và driver đã cài trên máy này. Nếu script báo thiếu quyền USB, chạy đúng lệnh `sudo chgrp ... && sudo chmod ...` mà nó in ra, rồi chạy lại. Giữ terminal camera mở.

```bash
cd /home/tai/detected_pallets
bash scripts/open_registered_astra.sh
```

Mở terminal thứ hai:

```bash
cd /home/tai/detected_pallets
bash scripts/open_pallet_distance_rviz.sh
```

RViz mở `config/pallet_distance.rviz`: point cloud `/camera/depth/points`, công cụ **Select** và panel **Selection**. Trong khung 3D, chọn một điểm trên mặt pallet rồi mở `Astra depth point cloud` trong Selection để đọc `Position`. Với fixed frame `camera_color_optical_frame`, `Z` là khoảng cách theo trục nhìn của camera tính bằng mét; khoảng cách thẳng từ tâm quang học tới điểm là `sqrt(X² + Y² + Z²)`. Một điểm chọn được không tự xác nhận đó là mặt trước pallet hoặc điểm YOLO đã chọn. Kết quả này vẫn cần đối chiếu thước đo thực tế trước khi dùng điều khiển xe.

### Xem point cloud với nội tại RGB đã hiệu chuẩn

`open_registered_astra.sh` nạp `config/calibration/astra_pro_rgb_640x480_2026-10-11.yaml` vào CameraInfo RGB. Point cloud mặc định của driver vẫn dùng tiêu cự Depth khoảng 570 px. Để xem XYZ chiếu theo nội tại RGB (`fx≈613`, `fy≈611` và hệ số méo), giữ Astra chạy và mở terminal khác:

```bash
cd /home/tai/detected_pallets
bash scripts/open_rgb_calibrated_depth_rviz.sh
```

Lệnh tạo `/camera/depth/points_rgb_calibrated` ở tối đa 10 Hz, lấy một điểm mỗi 2 pixel, rồi mở RViz có **Select** và panel **Selection**. Cách tính này giả định ảnh Depth đã đăng ký cùng pixel với RGB gốc. Nó chưa hiệu chỉnh sai số đo khoảng cách của cảm biến Depth; khi thử trên bàn cờ ở khoảng 1,1 m, Depth và khoảng cách suy từ bàn cờ còn lệch khoảng 6 cm.

### Xem thử hiệu chỉnh Depth lệch trái–phải

Sau khi giữ camera với góc roll 0°, các hàng sàn phẳng ở ba khoảng cách cho thấy độ dốc nghịch đảo Depth theo cột ảnh khoảng `0.00015807 1/m/pixel`. Giữ Astra chạy, mở terminal khác:

```bash
cd /home/tai/detected_pallets
bash scripts/open_spatial_corrected_depth_rviz.sh
```

RViz hiển thị `/camera/depth/points_spatial_corrected_preview` với **Select** và panel **Selection**. Công thức xem thử là `1/Z_mới = 1/Z_gốc + 0.00015807 × (370-u)`, trong đó `u` là cột pixel RGB. Cột `u=370` giữ nguyên Depth gốc; X/Y tính lại từ nội tại RGB đã hiệu chuẩn. Topic Depth gốc và cloud RGB cũ vẫn giữ nguyên để đối chiếu. Hệ số này lấy từ vùng sàn, chưa kiểm chứng độ chính xác tuyệt đối bằng thước trên toàn khung hình; không dùng cloud xem thử để điều khiển xe.

Cloud này dùng Depth cảm biến đã đăng ký theo RGB, nội tại/méo ống kính từ
`/camera/color/camera_info`, rồi áp dụng hệ số sửa Depth theo cột ảnh. Nó không
suy ra Depth mới từ độ sáng RGB hay model Depth AI. Point cloud giảm mật độ
(`pixel_stride=3`) chỉ để xem trong RViz; `/pallet/target` dùng cùng công thức
sửa tại pixel Depth gốc đủ 640×480 và dùng ảnh RGB/YOLO để chọn chân pallet.
Vì vậy phép đo pallet không phụ thuộc mật độ point cloud. Hiện chưa có topic
ảnh Depth **đã sửa** riêng cho các chức năng khác của xe; nếu cần bản đồ Depth
đầy đủ, phải xuất topic đó từ cùng công thức và kiểm chứng bằng thước trước khi
điều khiển chuyển động.

### Phát mục tiêu pallet riêng cho việc đo thử

Giữ `open_registered_astra.sh` chạy. Node mới lấy ảnh RGB và depth gốc
640×480, ghép theo timestamp tối đa 50 ms, kiểm tra CameraInfo và cấu hình
đăng ký depth, rồi dùng YOLO11-seg để tìm hai chân trước. Công thức hiệu chỉnh
được áp dụng trên pixel depth gốc, độc lập với mật độ point cloud RViz.

```bash
cd /home/tai/detected_pallets/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select detected_pallets_interfaces

# Terminal mới, sau khi đã mở Astra:
cd /home/tai/detected_pallets
bash scripts/open_pallet_target.sh

# Terminal theo dõi kết quả:
source /opt/ros/jazzy/setup.bash
source /home/tai/detected_pallets/ros2_ws/install/setup.bash
ros2 topic echo /pallet/target
```

Script mặc định dùng Python có ROS và Ultralytics tại
`/home/tai/astra_pallets/.venv/bin/python`; có thể chọn interpreter khác bằng
`PALLET_PYTHON=/đường/dẫn/python bash scripts/open_pallet_target.sh`.
`valid=false` và các số đo là NaN khi ảnh không đồng bộ, thiếu chân pallet,
CameraInfo không hợp lệ hoặc kết quả quá cũ. Bộ nhận phải kiểm tra `valid` và
tuổi của `header.stamp`, không giữ tọa độ cũ khi mất pallet.

Trục `forward` song song mặt sàn theo hướng xe chạy; `lateral` dương về bên
**trái** xe. `left_forward_m` và `right_forward_m` là khoảng cách hai chân đến
camera theo phương tiến. Hai chân được lấy Depth trong cùng một dải hàng hẹp
ở giữa phần chân nhìn thấy được; từng điểm được chiếu về phương song song mặt
sàn theo góc camera 20°. `camera_forward_m` là trung bình hai chân;
`chassis_front_forward_m` cộng thêm **6,65 cm** từ mặt trước ống kính tới mép
trước khung xe theo số đo lại của người vận hành; giá trị này cũng trùng với
thiết kế trong URDF.
`opening_lateral_m` là độ lệch tâm lỗ so với trục camera (giả định camera gắn
đúng giữa xe). `face_yaw_rad` dương nghĩa chân **trái xa hơn chân phải**;
`entry_heading_rad = -face_yaw_rad` là hướng tiến vuông góc mặt pallet.

`opening_width_m` lấy hai mép trong của chân; `opening_height_estimate_m` lấy
mép dưới dầm và chân từ mask RGB, giả định chúng cùng mặt phẳng trước. Chiều
cao này kém chắc chắn: mẫu camera hiện tại cho khoảng 7,5 cm trong khi thước
đo của người vận hành là 10 cm. Chiều rộng ảnh khoảng 23,7 cm so với thước
25,2 cm. Hệ số sửa depth, góc lắp camera và vị trí ống kính chưa được xác nhận
độc lập. `valid=true` chỉ nói rằng kiểm tra hình học của khung hình hiện tại
đã qua; chưa chứng nhận đủ chính xác để điều khiển AGV hay đút càng. Node này
**không phát lệnh xe**.
