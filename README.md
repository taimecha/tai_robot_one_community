# TAI AMR — robot thật

ROS 2 Jazzy trên Raspberry Pi 5: ESP32 điều khiển bánh và encoder, ESP32 BNO055/càng, LiDAR và camera Astra RGB-D. Bản đồ đang dùng là `ros2_ws/maps/tai_map_real_edited.yaml`.

## Chạy từ terminal VS Code SSH vào Pi

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

bash /home/admin/tai_robot_one/robot_amr/simulation/stop_r3_localization.sh && \
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh --nav-to-a --camera-obstacles --pickup
```

Luồng lấy hàng (`--pickup`): định vị LiDAR/bản đồ → Nav2 tới A đã lưu → homing càng → nhận diện pallet bằng RGB-D → chạy liên tục A→C→B bằng phản hồi odom/EKF và hiệu chỉnh định vị → dừng bánh tại B → tự nâng càng lên 8 cm. C là điểm nối tiếp tuyến, không yêu cầu nhấn Enter hoặc xoay tại C. Làm theo lời nhắc Enter trước khi xe bắt đầu đi.

Model YOLO nạp sẵn ở trạng thái tạm dừng trước bước tới A; mỗi lần nhận diện dùng một yêu cầu tạo phiên mới. Nếu mất heartbeat, mất luồng RGB hoặc suy luận bị kẹt trước khi xe chạy, chương trình tự mở lại riêng phần nhận diện tối đa hai lần và báo tiến độ. Mỗi lượt chờ phép đo tối đa 30 giây; các ngưỡng tuổi ảnh, hiệu chỉnh và chất lượng RGB-D giữ nguyên.

Ở bước Nav2 tới A, controller giữ nguyên goal và phát vận tốc 0 khi phép biến đổi TF tạm thiếu dữ liệu, tối đa 0,75 giây. Controller chỉ tiếp tục sau khi toàn bộ tính toán với TF và kiểm tra va chạm thành công; TF không phục hồi đúng hạn vẫn làm Nav2 dừng. Không dùng TF cũ hoặc thay timestamp để lái tiếp.

AMCL dùng mô hình likelihood theo tích xác suất và bỏ tia không khớp do vật thể mới, với 500–2.000 hạt và 90 tia mỗi lần cập nhật để chừa CPU cho Nav2/odom/RGB-D trên Pi. Riêng AMCL gửi DDS bất đồng bộ để giảm việc chờ đầu đọc RViz từ laptop. Khởi tạo bằng scan toàn bản đồ và các điều kiện kiểm tra định vị vẫn áp dụng. Log `navigation_motion.jsonl` lưu thời gian nhận/capture của LiDAR gốc, LiDAR đã lọc, odom, IMU, AMCL và timestamp hai cạnh TF để phân biệt nguồn gây trễ; timestamp TF AMCL có cộng tolerance, không phải thời điểm đo.

Homing kiểm tra công tắc đáy trước khi phát xung. Nếu đã chạm ổn định, firmware đặt HOME=0 tại chỗ. Khi đi tìm đáy, V4 dừng ngay tại cạnh tín hiệu đầu tiên để xác nhận công tắc ổn định, sau đó hạ đúng 100 bước (1 mm) tính từ cạnh đó. Firmware kiểm tra lại công tắc trước khi đặt HOME=0. Luồng lấy hàng kiểm tra phiên bản firmware, phản hồi càng và quyền phát lệnh trước khi chạy. Chỉ nâng khi xe đã dừng đạt điều kiện tại B. `PICKUP_COMPLETE` xác nhận hành trình và vị trí càng theo odom/ESP32; không có cảm biến tải để xác nhận độc lập pallet đã được nhấc.

Sau kết nối USB hoặc ESP32 khởi động lại, bridge tự hỏi `LIFT,INFO` cách nhau ít nhất một giây, tối đa tám lần nếu chưa nhận được phiên bản. `/lift/status` lưu phiên bản thực và số lần hỏi. Bước HOME cần firmware HOME 1 mm V3 hoặc V4 và telemetry mới hợp lệ. Mỗi lượt HOME ghi trạng thái và phản hồi công tắc trong `lift_home_telemetry.jsonl` để đối chiếu khi có lỗi.

Bỏ `--pickup` để dùng luồng A→C như trước. Xem [firmware ESP32 số 2](esp32_firmware/Projects/bno055_imu_monitor/README.md) nếu cần nạp lại firmware.

Trong khi HOME, kiểm tra thân xe đứng yên dùng `/base_controller/odom` từ encoder bánh (`open_loop=false`, `position_feedback=true`). Rung IMU/EKF và một mẫu vận tốc nhiễu không tự hủy HOME; gián đoạn odom ngắn được chờ khôi phục, còn dịch chuyển encoder rõ ràng hoặc mất nguồn odom kéo dài vẫn dừng.

Sau khi kiểm tra tư thế và khe hở tại B, bước nâng giữ lệnh bánh bằng 0 và kiểm tra encoder bánh, quyền phát lệnh, giới hạn hành trình và phản hồi ESP32. Gián đoạn LiDAR/TF của bộ điều hướng trong lúc xe đứng yên được ghi riêng, để lỗi thời gian của scan không dừng giữa chừng bước nâng. Tín hiệu `StopZone`, encoder hết hạn hoặc xe dịch chuyển vẫn dừng nâng. Trước khi đi D, Nav2 phải kiểm tra lại định vị và dữ liệu hiện tại.

Tại C và trong đoạn chèn, chương trình kiểm tra toàn bộ hai càng với các mép trong chân pallet, dự phòng tối thiểu 10 mm và kiểm tra hướng tiến tới B. Sai lệch tâm lớn hơn 9 mm tự nó không hủy lượt nếu hai càng vẫn đủ khe hở. Kiểm tra thân xe, vật cản và vị trí dừng dọc tại B vẫn được áp dụng. Khe hở ước lượng được lưu trong `insertion_clearance` của kết quả lượt chạy.

Từ 40 cm trước C, bộ phát odom chuyển dần về mốc đã dùng khi khóa pallet tại A, với giới hạn hiệu chỉnh 3 cm/s và 0,5°/s; chỉ xác nhận khóa chèn khi đã tới mốc này. Xe, mục tiêu pallet và lọc LiDAR vì vậy dùng chung một mốc trong C→B. AMCL vẫn định vị cho Nav2 và hiệu chỉnh đoạn tiếp cận trước vùng này, đồng thời tiếp tục hiển thị trên RViz. Nếu chưa xác nhận mốc chung khi còn 12 cm tới C, xe giữ vận tốc 0 chờ xác nhận.

Đường C→B song song trục pallet đã đo tại A. Đoạn thẳng dẫn vào C dài 25 cm để ổn định góc sớm. Bộ bám dùng góc `/odom` gắn mốc A và tốc độ quay từ cùng bản tin `/odom` để giữ mục tiêu 0°. Khi bánh chênh tốc mà thân xe chưa quay đủ, tích phân tăng dần quyền sửa trong giới hạn `min(0,06 rad/s, 1,2 × tốc độ tiến)`. Khi góc nằm trong ±0,20° và tốc độ quay nhỏ, ngừng tích phân và giữ lệnh quay 0; chỉ mở lại khi vượt ±0,35° hoặc thân xe đang quay rõ. Tích phân được xóa khi qua góc 0° hoặc phục hồi sau gián đoạn dữ liệu để hạn chế quay ngược lại. Kiểm tra đầy đủ cả hai càng, dự đoán hình học theo cả mốc camera và mốc điều khiển, cùng chống vật cản vẫn áp dụng. Các dải góc này là mục tiêu điều khiển theo cảm biến, không phải chứng nhận độ chính xác cơ khí.

Khi `StopZone` dừng lượt, chương trình lưu tối đa bốn scan gần nhất và các điểm còn lại trong vùng dừng vào `stop_zone_evidence.json`. Tệp gồm timestamp gốc, scan trước/sau lọc, phép biến đổi đã dùng và tọa độ điểm để phân biệt điểm pallet với vật cản mới; bản tin StopZone không tự xác định chính xác tia nào đã kích hoạt dừng.

Trong lúc bám đường, mẫu odom quá 0,20 giây hoặc trạng thái định vị giao tới chậm khiến xe tạm giữ vận tốc 0. Khi nguồn dữ liệu mới được kiểm tra hợp lệ, cùng mốc và phục hồi trong một giây, xe tiếp tục với tốc độ tăng lại từ 0. Mất dữ liệu kéo dài, đổi phiên/bản đồ, khởi động lại bộ phát odom hoặc odom nhảy vẫn hủy lượt. Bộ bám đọc `/odom` từ EKF hiện tại. Bản tin `/imu` thô chỉ được lưu trong log chẩn đoán. Đường gửi lệnh dùng Fast DDS bất đồng bộ, giữ cấu hình tuổi lệnh DDS 0,20 giây và hàng đợi một mẫu để lệnh 0 mới thay thế lệnh chạy cũ. Kiểm tra độ mới của dữ liệu trong bộ điều khiển vẫn bắt buộc; cấu hình tuổi lệnh DDS không thay thế kiểm tra này. Vật cản vẫn do collision monitor xử lý.

Hình kết quả mặc định dùng góc nhìn laptop với Y dương hướng xuống; góc âm được vẽ xéo càng lên khi pallet ở bên phải hình. Toàn bộ xe, càng, pallet và quỹ đạo cùng đổi góc nhìn; dấu góc ROS trong số liệu vẫn giữ nguyên. Cả tâm odom và tâm gốc càng được vẽ từ cùng một tư thế encoder/IMU gắn mốc A. Hai tâm cách nhau 36,55 cm dọc xe; khi đi thẳng, chúng cùng trục nhưng không cùng tọa độ tại một thời điểm. Đường điều khiển có hiệu chỉnh AMCL được vẽ riêng để nhận biết sự dịch mốc. Hình là ước lượng, không phải phép đo lại pallet tại B.

Chỉ mở định vị và cảm biến, không gửi goal:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_localization.sh
```

Dừng phiên định vị:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/stop_r3_localization.sh
```

## Dạy điểm giao hàng bằng RViz đang mở

Đặt xe tại một vị trí trong vùng bản đồ đã đo, rồi chạy trên Pi:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/stop_r3_localization.sh && \
bash /home/admin/tai_robot_one/robot_amr/simulation/teach_r3_delivery.sh --camera-obstacles
```

Xe tìm vị trí bằng LiDAR/bản đồ khi đứng yên. Giữ nguyên cửa sổ `r3_laptop.rviz` trên laptop. Chờ chữ `DELIVERY: READY_FOR_GOAL`, chọn **Nav2 Goal**, bấm vị trí giao hàng và kéo mũi tên theo hướng xe cần đứng. Goal chỉ được nhận khi định vị và Nav2 đã sẵn sàng. Nếu chưa xác định được vị trí đáng tin cậy, phiên dừng; có thể dùng `--initial-pose X Y YAW_DEG` khi biết tư thế hiện tại. Không thể bảo đảm tự định vị tại mọi nơi, đặc biệt ở vùng bản đồ có hình học giống nhau.

Khi xe tới goal, chương trình kiểm tra tư thế và dừng ổn định. Kiểm tra vị trí/hướng thực tế rồi nói **“lưu”** trong cuộc trò chuyện. Chương trình chưa tự tạo điểm giao hàng khi chọn goal hoặc khi Nav2 báo tới. Xem trạng thái tại terminal Pi khác:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/status_r3_delivery.sh
```

Chỉ sau xác nhận lưu của người vận hành, dùng lệnh:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/save_r3_delivery.sh --confirm 'lưu'
```

Điểm được ghi riêng vào `simulation/r3/delivery_goal.json`, gắn với nội dung bản đồ và bằng chứng định vị. Lưu X/Y/yaw đo được của xe sau khi dừng, kèm goal đã chọn; không lấy tọa độ goal thay cho tư thế thực tế. Không ghi đè điểm đã có hoặc điểm nhận diện A. Ngưỡng xác nhận tới goal là dưới 10 cm và tối đa 5° theo ước lượng định vị, không phải chứng nhận độ chính xác đặt pallet. Tư thế lưu là tư thế xe tại nơi giao hàng, không phải tọa độ tâm pallet.

## Lấy pallet và giao tới D

Sau khi lưu D, chạy trên Pi:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/stop_r3_localization.sh && \
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh --nav-to-a --camera-obstacles --pickup --deliver-after-pickup
```

Làm theo lời nhắc Enter để xe tới A, rồi xác nhận lấy và giao pallet. Sau khi ESP32 xác nhận đã nâng 8 cm, xe tự chở pallet tới D đã lưu bằng Nav2 với vùng kiểm tra rộng hơn ở đầu càng. Khi xe đã tới D và các bánh đứng yên, càng hạ tới vị trí 2 mm trên mốc HOME. Thao tác kết thúc khi ESP32 xác nhận càng đã dừng ở độ cao giao. Càng vẫn nằm dưới pallet để người vận hành kiểm tra tại chỗ và rút càng sau. Chuyến giao có tải cần thử có giám sát trên xe thật.

Lệnh `--nav-to-a --pickup --deliver-after-pickup` giữ cùng một phiên Nav2 cho cả A và D. Nav2 dùng vùng tránh va chạm đủ cho pallet ở hai chặng và tạm ngưng khi bộ lấy hàng điều khiển bánh ở A→C→B. AMCL tiếp tục cập nhật trong cùng phiên. Khi ESP32 xác nhận đã nâng và bộ lấy hàng nhả quyền bánh, Nav2 tiếp tục trong phiên đang giữ. Chương trình kiểm tra lại LiDAR và odom rồi tự gửi goal D. Nếu phải lấy lại bằng chứng tại B, chương trình yêu cầu một cập nhật AMCL tại chỗ. `delivery_arrival.json` ghi thời điểm nâng xong, sẵn sàng giao hàng, gửi goal và phát lệnh bánh đầu tiên.

## RViz trên laptop Ubuntu

Chạy trong terminal desktop của laptop, cùng mạng với Pi:

```bash
mkdir -p "$HOME/robot_rviz"
scp admin@10.181.80.50:/home/admin/tai_robot_one/robot_amr/ros2_ws/config/r3_laptop.rviz "$HOME/robot_rviz/"
scp admin@10.181.80.50:/home/admin/tai_robot_one/robot_amr/simulation/open_r3_laptop_rviz.sh "$HOME/robot_rviz/"
bash "$HOME/robot_rviz/open_r3_laptop_rviz.sh"
```

## Mã nguồn và dữ liệu đang dùng

| Thư mục | Vai trò |
| --- | --- |
| `simulation/` | Các lệnh và mô-đun chạy robot thật; giữ tên thư mục để lệnh hiện có tiếp tục dùng được |
| `camera/detect_astra/` | Nhận diện pallet, theo dõi RGB-D, hiệu chỉnh và model |
| `ros2_ws/` | Phần cứng ROS, Nav2, mô hình xe, bản đồ, cấu hình RViz và driver cảm biến |
| `esp32_firmware/` | Firmware và thư viện PID |

Điểm A nằm tại `simulation/r3/recognition_A.json`, là dữ liệu riêng của xe/bản đồ và không đưa vào Git. Các file hiệu chỉnh và thông số hình học là đầu vào chạy xe, cần giữ lại. Xem [dữ liệu R3](simulation/r3/README.md), [camera](camera/detect_astra/README.md) và [build phần cứng](ros2_ws/README.md).

Các kết quả cũ, demo mô phỏng và bộ kiểm thử được đưa ra khỏi cây dự án. Khi chạy xe, chương trình vẫn tạo log/bằng chứng mới trong `simulation/output/r3_live/`; đây là dữ liệu sinh tự động, đã được Git bỏ qua.
