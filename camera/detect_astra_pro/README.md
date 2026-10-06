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

Kiểm tra `/dev/video0` đã xuất hiện trong WSL và người dùng có quyền truy cập camera. Riêng script nhận diện pallet có thể chọn camera khác bằng biến môi trường `PALLET_CAMERA`, ví dụ `PALLET_CAMERA=/dev/video2 python src/test_pallets.py`.

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

## Xem ảnh depth và vật cản trong ROS 2

Các script trong `scripts/` dùng ROS 2 Jazzy và Astra driver đã build trong `/home/tai/tai_robot_one_community/ros2_ws`. Cấu hình hiện tại dành cho máy `tai`; nếu dùng máy khác, cần cập nhật các đường dẫn ROS workspace và thư viện trong script. Camera thật được đặt cao 0,8 m, chúc xuống 20°; URDF của robot phải khai báo góc pitch `+20°` để `/camera/obstacle_scan` lọc sàn theo đúng độ cao.

Mở depth đơn thuần bằng hai terminal:

```bash
bash scripts/open_depth.sh
bash scripts/open_depth_rviz.sh
```

Xem trước scan vật cản mà không khởi động động cơ hoặc Nav2:

```bash
bash scripts/preview_pallet_obstacles.sh
bash scripts/open_pallet_obstacle_rviz.sh
```

Mở mỗi lệnh trong một terminal riêng. RViz dùng `base_footprint` làm gốc tọa độ, hiển thị `/camera/obstacle_scan` và ẩn point cloud thô để không lẫn sàn vào hình vật cản. Preview chưa thay thế việc kiểm tra trên robot thật trước khi cho xe di chuyển.
