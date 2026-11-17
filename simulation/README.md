# Luồng chạy robot thật R3

Tên thư mục được giữ để các lệnh đang dùng trên Pi tiếp tục hoạt động. Thư mục này chứa các mô-đun cần cho robot thật.

| Thành phần | Nhiệm vụ |
| --- | --- |
| `run_r3_live_trial.sh` | Điều phối định vị, tới A, nhận diện và chạy tới C |
| `run_r3_localization.sh`, `stop_r3_localization.sh`, `status_r3_localization.sh` | Mở, đóng và xem trạng thái định vị |
| `save_r3_a.sh`, `r3_recognition_goal.py` | Lưu A khi được yêu cầu và điều hướng tới A |
| `r3_global_*`, `r3_amcl_*`, `r3_stationary_localization.py`, `r3_scan_map_check.py` | Khởi tạo và kiểm tra định vị LiDAR/bản đồ |
| `r3_control_odom.py`, `r3_localization_contract.py` | Kết hợp odom/EKF với hiệu chỉnh định vị |
| `r3_live_prepare.py`, `r3_approach_planner.py`, `r3_tracking.py`, `r3_core.py` | Khóa mục tiêu RGB-D, lập và kiểm tra quỹ đạo, bám đường |
| `r3_real_trial.py`, `r3_real_obstacle_filter.py`, `r3_pallet_obstacles.py` | Thực thi chuyển động và kiểm tra vật cản |
| `r3_lift_control.py` | HOME càng và điều khiển nâng với phản hồi ESP32; hỗ trợ chế độ `--pickup` của lệnh chạy chính |
| `r3_insertion_clearance.py` | Kiểm tra khe hở cả hai càng và hình học pallet trong đoạn C→B |
| `r3_floor_aligned_cloud.py` | Đám mây điểm hiệu chỉnh mặt sàn cho dữ liệu vật cản |
| `r3_render_real_result.py`, `r3_navigation_telemetry.py` | Xuất kết quả và dữ liệu của lượt chạy |
| `open_r3_laptop_rviz.sh` | Mở RViz trên laptop Ubuntu |

`r3_tracking.predict_tracking` là phép dự đoán chuyển động để kiểm tra đường trước khi chạy, sử dụng cùng bộ bám đường. Nó vẫn cần cho chương trình thật; không phải kết quả đo độc lập.

Nav2 tới A mặc định chọn `GridShortest` (`ExactGoalSmac2D`). Plugin thử đoạn nối thẳng start→goal trước, kiểm tra footprint có padding của thân/càng, quãng xoay đầu và quãng xoay cuối trong toàn bộ vùng chấp nhận XY của goal. Nếu thông thoáng, đoạn thẳng được chọn dù có cost inflation mềm; ô vật cản, ô chưa biết hoặc footprint ra ngoài bản đồ sẽ chặn nhánh này. Khi đoạn thẳng hoặc vùng xoay bị chặn, plugin tìm đường bằng Smac 2D như trước. `GridBased` vẫn có thể được chọn với trọng số cost cao hơn khi tìm đường vòng; các planner Hybrid/Lattice không được nạp trong luồng R3 tới A.

`navigation_plans.jsonl` lưu từng đường `/plan` mới cùng chiều dài để đối chiếu với ảnh RViz. Tiến trình ghi dữ liệu không phát lệnh chuyển động.

Goal cuối giữ nguyên X/Y/yaw trong `map` và dùng timestamp 0 để bộ kiểm tra tới đích biến đổi bằng TF mới nhất, cùng mốc định vị với bộ bám đường. Khi chuyển giữa hướng bám tuyến và góc cuối A, controller giữ vận tốc 0, chờ phản hồi dừng ổn định rồi chọn lại quãng xoay; góc quay còn lại của mục tiêu trước không được dùng cho mục tiêu mới. Lệnh quay được giới hạn lại sau bước giới hạn gia tốc để phản hồi tốc độ cao không đẩy lệnh vượt trần cấu hình.

Node odom lấy hàng khởi động sau khi Nav2 nhả điều khiển. Tính liên tục của odom điều khiển bắt đầu từ lần đầu chấp nhận mốc trial hợp lệ; dữ liệu khi chưa bàn giao không được khóa lỗi của lượt lấy hàng sau đó. Sau khi đã xuất odom điều khiển, các lỗi nhảy pose, đảo thời gian, gián đoạn trên 500 ms và đổi mốc/phiên vẫn bị khóa như trước. File `control_odom_status.jsonl` lưu trạng thái, nguyên nhân từ chối và các timestamp đầu vào/đầu ra. Báo cáo bàn giao lỗi cũng lưu trạng thái node này thay vì chỉ báo thiếu odom chung chung.

Nhận diện chỉ đếm ứng viên pallet đạt ít nhất 75%, cùng ngưỡng với bước xác nhận semantic; một nhận nhầm yếu ở nền không khóa ứng viên đủ chuẩn. Hai pallet riêng biệt cùng đủ chuẩn vẫn bị từ chối vì chưa chọn được đích duy nhất. Log `RECOGNIZING` lưu `pallet_candidates` gồm nhãn, confidence, bounding box và trạng thái đủ chuẩn. Kiểm tra ROS graph/tham số camera chạy ở callback group riêng với executor hai luồng để tiếp tục nhận ảnh trong khi kiểm tra; phát hiện thêm nguồn ảnh sẽ vô hiệu hóa xác nhận cũ. Giới hạn tuổi ảnh, đồng bộ RGB-D và phép đo vẫn áp dụng.

`RGBD_READY_PLANNING` xác nhận đã có đủ phép đo mới trước khi tìm đường. Bộ lập đường ưu tiên đoạn thẳng 25 cm trước C; nếu không đủ chỗ bẻ lái, nó thử 20, 15 rồi 12 cm, giữ giới hạn bán kính quay, kiểm tra toàn bộ càng và đoạn thẳng C→B. `PLAN_READY` ghi đoạn thẳng được chọn và thời gian lập đường. `PLAN_BLOCKED` (mã thoát 5) là lỗi lập đường và không khởi động lại YOLO.

Khi `--pickup` đang chạy, kiểm tra hình học xét đường cong của lệnh hiện tại/lệnh mới và toàn bộ quãng phanh, gồm trễ lệnh và giới hạn giảm tốc thẳng/quay. Hai trục giảm tốc độc lập: thời gian phanh thẳng không được kéo dài thời gian quay của một mẫu gyro. Bốn chân pallet được nới thêm biên 10 mm; kiểm tra vẫn bao gồm hai càng, thân xe, mặt pallet, sàn và khoảng trống phía sau. Đoạn giữ góc kiểm tra cả hệ điều khiển và hệ camera khóa tại A, gồm vận tốc đo được khi chuyển bộ điều khiển. Không dùng giả định giữ nguyên góc chạy thẳng tới B để khóa dừng trước khi bộ điều khiển kịp sửa góc. LiDAR và các kiểm tra tính hợp lệ của dữ liệu vẫn giữ quyền dừng xe.

Giảm tốc tới B xét cả tốc độ đo được trong quãng trễ lệnh và một chu kỳ điều khiển tiếp theo. Phản hồi PI tốc độ dọc có giới hạn tích phân, tốc độ và gia tốc; vận tốc tham chiếu được tách khỏi lệnh bù để giữ đúng hướng bám đường. Sau một lần giữ vận tốc 0, chương trình xóa tham chiếu và tích phân trước khi tiếp tục. Nếu xe đã gần đứng yên trong vùng B ±5 mm, bộ bám chuyển sang kiểm tra dừng thay vì cố đuổi vùng ±3 mm dành cho xe còn chạy. Điều kiện sau phanh và trước nâng vẫn kiểm tra lại vị trí, tốc độ dừng và khe hở. Lượt `20261117_174934_zVNeJe` dừng do không tiến ở lệnh nhỏ; lượt `20261117_175312_v4Zodb` dừng do quãng phanh dự đoán chạm thân/chân pallet. Dữ liệu các lượt mới lưu `longitudinal_control` và `/wheel_pid_telemetry` trong chẩn đoán để đối chiếu lệnh, tốc độ encoder, target MCU và PWM.

Kiểm tra hồi quy không kết nối ROS hay phần cứng, gồm 122 mẫu odom của lượt `20261117_120900_8MXCL8`, 167 mẫu của lượt `20261117_123354_b4ien3`, dữ liệu hai lượt dừng cuối `20261117_174934_zVNeJe`/`20261117_175312_v4Zodb`, các trường hợp va chạm/quãng phanh và mô hình đáp ứng tốc độ có trễ. Kết quả mô hình không xác nhận độ chính xác hay thành công lấy hàng thật:

```bash
cd /home/admin/tai_robot_one/robot_amr
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover -s simulation/tests -v
```

Khóa phiên `.trial.lock` được giữ bởi shell điều phối; các tiến trình ROS, camera, Python và `tee` chạy lâu đóng descriptor khóa khi khởi động. `stop_r3_localization.sh` chờ shell dọn xong; nếu shell đã thoát mà tiến trình con còn thừa kế khóa từ phiên cũ, lệnh dừng xác minh inode/flock, danh tính và dự án trước khi đóng đúng tiến trình giữ khóa. Ctrl+C ngắt lệnh chờ với thông báo ngắn, không in traceback.

Lệnh đầy đủ và cấu hình RViz: [README dự án](../README.md). Dữ liệu đầu vào: [r3/README.md](r3/README.md).
