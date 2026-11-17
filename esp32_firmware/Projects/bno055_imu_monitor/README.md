# ESP32 số 2 — IMU BNO055 và càng nâng

Đây là firmware production duy nhất cho ESP32 số 2. File build chính:
`src/main.cpp`.

Firmware không tạo Wi-Fi/Web server và không chạy Bluetooth. Giao tiếp duy
nhất với Raspberry Pi 5 là USB Serial 115200 baud qua `/dev/tai_imu`.

## Chân kết nối

| Thiết bị | GPIO ESP32 |
| --- | ---: |
| BNO055 SDA | 21 |
| BNO055 SCL | 22 |
| TB6600 PUL- | 23 |
| TB6600 DIR- | 4 |
| TB6600 EN- | 19, giữ HIGH theo cấu hình đã thử |
| Công tắc giới hạn chiều nâng | 33, active LOW |
| Công tắc đáy/HOME | 32, active LOW |

TB6600 đặt 1/4 bước: 800 bước/vòng. Vít me T8x8 tương ứng 100000 bước/m.
Tốc độ tối đa 1000 bước/s và gia tốc 500 bước/s².

## Phân chia CPU

- Core 0: BNO055 50 Hz, tự đặt zero sau khoảng 2 giây, USB Serial và telemetry.
- Core 1: AccelStepper và công tắc hành trình.

## Build và nạp

```bash
cd /home/admin/tai_robot_one/robot_amr/esp32_firmware/Projects/bno055_imu_monitor
pio run
pio run --target upload --upload-port /dev/tai_imu
```

Không mở PlatformIO monitor khi ROS bridge đang dùng `/dev/tai_imu`.

## Protocol Serial

- `IMU,1,...`: quaternion, gyro, acceleration và calibration.
- `LIFT,HOME`: kiểm tra công tắc đáy trước; nếu đã chạm ổn định thì đặt HOME=0 không phát xung. Nếu chưa chạm thì tìm đáy, từ lần chạm đầu hạ thêm đúng 100 bước (1 mm), chờ ổn định rồi đặt vị trí 0.
- `LIFT,INFO`: trả `INFO,LIFT,HOME_SKIP_ACTIVE_1MM_NEW_CONTACT_V3`; luồng lấy hàng dùng để kiểm tra cấu hình homing.
- `LIFT,SET,0.02000`: đặt chiều cao tuyệt đối theo mét.
- `LIFT,STOP`: dừng phát xung.
- `LIFT,HB`: heartbeat khi HOME.
- `LIFT,2,...`: phản hồi vị trí, tốc độ, công tắc, số lần HOME đã hoàn thành và cờ bỏ qua chuyển động do công tắc đã chạm. Bridge vẫn đọc được protocol `LIFT,1` cũ.
