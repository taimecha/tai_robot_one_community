# Dữ liệu R3 đang dùng

- `recognition_A.json`: điểm nhận diện A đã dạy trong hệ tọa độ `map`, kèm nhận dạng bản đồ và bằng chứng tại lúc lưu. Đây là dữ liệu riêng của máy, được Git bỏ qua. Không tự ghi đè A khi chạy lại.
- `delivery_goal.json`: điểm giao hàng trong `map`, chỉ tạo sau khi xe tới goal, dừng ổn định và người vận hành xác nhận “lưu”. Tọa độ là tư thế xe đo được; gắn với bản đồ, không ghi đè A hoặc điểm giao hàng hiện có. Dùng `teach_r3_delivery.sh` để dạy, xem hướng dẫn tại README dự án.
- `robot_geometry.json`: bộ thông số hình học gốc mà `r3_live_prepare.provisional_geometry()` đọc để lập đường. Một số kích thước vẫn là giả định được ghi rõ trong dữ liệu; không phải toàn bộ hình học đã được đo kiểm.
- `real_trial_lateral_calibration.json`: hiệu chỉnh ngang theo trục vào pallet, được áp một lần khi khóa mục tiêu.

Chỉ khi muốn dạy lại A, mở phiên định vị tại vị trí mong muốn và chạy ở terminal Pi khác:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/save_r3_a.sh
```

Công cụ kiểm tra điều kiện và từ chối ghi đè điểm A đã tồn tại; xem `--help` để chọn đường dẫn A mới khi cần.
