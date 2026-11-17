#!/usr/bin/env bash
# One terminal: start missing sources, recognize, plan, track, save image, stop owned nodes.
set -eo pipefail
capture_only=0
localization=amcl
localization_timeout=120
map_file=''
initial_pose_args=()
use_rviz=0
nav_to_a=0
nav_check_only=0
recognize_here=0
save_a=0
goal_file=''
global_localization=0
localization_only=0
camera_obstacles=0
pickup=0
while [[ $# -gt 0 ]]; do
case "$1" in
  --capture-only) capture_only=1; shift ;;
  --localization) [[ $# -ge 2 ]] || exit 2; localization=$2; shift 2 ;;
  --localization-timeout) [[ $# -ge 2 ]] || exit 2; localization_timeout=$2; shift 2 ;;
  --map) [[ $# -ge 2 ]] || exit 2; map_file=$2; shift 2 ;;
  --initial-pose) [[ $# -ge 4 ]] || exit 2; initial_pose_args=(--initial-pose "$2" "$3" "$4"); shift 4 ;;
  --rviz) use_rviz=1; shift ;;
  --nav-to-a) nav_to_a=1; localization=amcl; shift ;;
  --nav-check-only) nav_check_only=1; nav_to_a=1; localization=amcl; shift ;;
  --recognize-here) recognize_here=1; localization=amcl; shift ;;
  --save-a) save_a=1; localization=amcl; shift ;;
  --a-file) [[ $# -ge 2 ]] || exit 2; goal_file=$2; shift 2 ;;
  --global-localization) global_localization=1; localization=amcl; shift ;;
  --localization-only) localization_only=1; localization=amcl; shift ;;
  --camera-obstacles) camera_obstacles=1; shift ;;
  --pickup) pickup=1; shift ;;
  --help|-h)
    echo 'Usage: bash simulation/run_r3_live_trial.sh [--capture-only | --localization-only] [--localization amcl|odom] [--localization-timeout SECONDS] [--map ABS.yaml] [--initial-pose X Y YAW_DEG] [--rviz] [--nav-to-a | --nav-check-only | --recognize-here | --save-a] [--pickup] [--a-file NEW.json] [--global-localization]'
    echo '--capture-only: chốt RGB-D và lưu bằng chứng; không khởi chạy bộ bám quỹ đạo.'
    echo 'Mặc định: AMCL định vị LiDAR + bản đồ chỉnh sửa; tự tìm toàn bản đồ nếu không có initial-pose/RViz. Dùng --localization odom để chạy theo odom cũ.'
    echo '--localization-timeout: tối đa 120 giây mặc định để định vị trước khi đo/chạy; hết hạn sẽ dừng an toàn.'
    echo '--save-a: định vị và lưu A mới tại vị trí đang đứng; không chạy bánh.'
    echo '--nav-to-a: tự định vị, Nav2 tới A mới đã lưu, rồi nhận diện và chạy thử A→C.'
    echo '--nav-check-only: kiểm tra định vị và Nav2 cho luồng tới A; không gửi goal và không chạy A→C.'
    echo '--recognize-here: định vị LiDAR/bản đồ tại chỗ, bỏ bước Nav2 tới A; đo pallet theo tư thế thực tế rồi hỏi Enter trước khi chạy tới C. Có thể thêm --capture-only để chỉ đo và lưu.'
    echo '--localization-only: LiDAR + AMCL + camera vật cản + global/local costmap + kiểm tra A tại chỗ; Ctrl+C để đóng.'
    echo '--camera-obstacles: thêm dữ liệu camera vật cản để xem RViz trong lượt thử (không bật YOLO).'
    echo '--pickup: sau Nav2 tới A (hoặc recognize-here), HOME càng, nhận diện, chạy liên tục A→C→B rồi nâng 8 cm; cần firmware HOME 1 mm mới.'
    exit 0 ;;
  *) echo "Tham số không hỗ trợ: $1" >&2; exit 2 ;;
esac
done
[[ $pickup -eq 0 || ( ( $nav_to_a -eq 1 || $recognize_here -eq 1 ) && $nav_check_only -eq 0 && $localization_only -eq 0 && $capture_only -eq 0 && $save_a -eq 0 ) ]] || {
  echo '--pickup cần --nav-to-a hoặc --recognize-here, không dùng với chế độ chỉ kiểm tra/đo/lưu A.' >&2; exit 2;
}
[[ $localization_only -eq 0 || ( $nav_to_a -eq 0 && $recognize_here -eq 0 && $save_a -eq 0 && $capture_only -eq 0 ) ]] || {
  echo '--localization-only không kết hợp với nav-to-a/recognize-here/save-a/capture-only' >&2; exit 2;
}
[[ $recognize_here -eq 0 || ( $nav_to_a -eq 0 && $save_a -eq 0 ) ]] || {
  echo '--recognize-here không kết hợp với nav-to-a/nav-check-only/save-a' >&2; exit 2;
}
if [[ $localization_only -eq 1 ]]; then
  localization=amcl
  camera_obstacles=1
fi
[[ $nav_to_a -eq 0 || $save_a -eq 0 ]] || { echo 'Chọn --nav-to-a hoặc --save-a' >&2; exit 2; }
[[ $nav_to_a -eq 0 || $capture_only -eq 0 ]] || { echo '--capture-only không đi Nav2; bỏ --nav-to-a' >&2; exit 2; }
if [[ $nav_to_a -eq 1 || $recognize_here -eq 1 || $save_a -eq 1 ]]; then
  localization=amcl
fi
if [[ $localization == amcl && ${#initial_pose_args[@]} -eq 0 && $use_rviz -eq 0 ]]; then
  global_localization=1
fi
[[ $global_localization -eq 0 || ${#initial_pose_args[@]} -eq 0 ]] || { echo 'Không dùng đồng thời global-localization và initial-pose' >&2; exit 2; }
[[ $localization == odom || $localization == amcl ]] || { echo 'localization phải là odom hoặc amcl' >&2; exit 2; }
[[ $localization == amcl || $global_localization -eq 0 ]] || { echo '--global-localization cần AMCL' >&2; exit 2; }
[[ $localization_timeout =~ ^[1-9][0-9]*$ ]] || { echo '--localization-timeout phải là số giây nguyên dương' >&2; exit 2; }
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
[[ -n "$map_file" ]] || map_file="$root/ros2_ws/maps/tai_map_real_edited.yaml"
[[ -n "$goal_file" ]] || goal_file="$root/simulation/r3/recognition_A.json"
source /opt/ros/jazzy/setup.bash
source camera/detect_astra/ros2_ws/install/setup.bash
source camera/detect_astra/data/alignment_robot_ws/install/setup.bash
set -u
[[ -t 0 && -t 1 ]] || { echo 'Chạy lệnh này trong terminal tương tác của VS Code.' >&2; exit 1; }
if [[ $localization == amcl ]]; then
  python3 - "$map_file" <<'PY'
import importlib.util,sys
spec=importlib.util.spec_from_file_location('r3loc','camera/detect_astra/launch/r3_localization.launch.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
module.validate_map_file(sys.argv[1])
PY
  if [[ $use_rviz -eq 1 && -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    echo 'Phiên này chưa có màn hình đồ họa để mở RViz; dùng --initial-pose X Y YAW_DEG hoặc mở RViz từ máy đã cấu hình ROS.' >&2
    exit 2
  fi
fi
if [[ $nav_to_a -eq 1 || $recognize_here -eq 1 ]]; then
  python3 simulation/r3_recognition_goal.py inspect --map "$map_file" --goal "$goal_file"
fi

for device in /dev/tai_drive /dev/tai_imu /dev/tai_lidar; do
  [[ -e "$device" ]] || { echo "Thiếu thiết bị $device" >&2; exit 1; }
done
[[ -x camera/detect_astra/.venv/bin/python ]] || { echo 'Thiếu Python camera' >&2; exit 1; }
[[ $localization_only -eq 1 || -d camera/detect_astra/data/runtime_models/best_v3_ncnn_model ]] || {
  echo 'Thiếu model NCNN pallet' >&2; exit 1;
}

mkdir -p simulation/output/r3_live
exec 9>simulation/output/r3_live/.trial.lock
flock -n 9 || { echo 'Đã có một lượt r3 đang chạy' >&2; exit 1; }
run_dir=$(mktemp -d "$root/simulation/output/r3_live/$(date +%Y%m%d_%H%M%S)_XXXXXX")
plan="$run_dir/plan/frozen_plan.json"
camera_pid=''
bringup_pid=''
perception_pid=''
safety_pid=''
localization_pid=''
localization_bridge_pid=''
control_odom_pid=''
global_initializer_pid=''
rviz_pid=''
navigation_pid=''
camera_obstacles_pid=''
navigation_telemetry_pid=''
inspection_costmaps_pid=''
a_watch_pid=''

group_running() {
  local group=$1
  ps -eo pgid=,stat= | awk -v wanted="$group" \
    '$1 == wanted && $2 !~ /^Z/ { found = 1 } END { exit !found }'
}

stop_owned() {
  local pid=$1
  [[ -n "$pid" ]] || return 0
  if group_running "$pid"; then
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in {1..20}; do
      group_running "$pid" || break
      sleep .2
    done
    if group_running "$pid"; then
      kill -TERM -- "-$pid" 2>/dev/null || true
      for _ in {1..10}; do
        group_running "$pid" || break
        sleep .2
      done
    fi
    if group_running "$pid"; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
  fi
  wait "$pid" 2>/dev/null || true
}

start_perception() {
  if [[ -n "$perception_pid" ]] && kill -0 "$perception_pid" 2>/dev/null; then
    return 0
  fi
  echo 'Nạp sẵn model YOLO; nhận diện và tracking đang tạm dừng tới khi xe ở A.'
  setsid bash camera/detect_astra/scripts/open_pallet_target.sh \
    --model camera/detect_astra/data/runtime_models/best_v3_ncnn_model \
    --measurement-bias-yaml camera/detect_astra/config/calibration/pallet_target_bias_robot_2026_11_11.yaml \
    --measurement-evidence-dir "$run_dir/camera_evidence" \
    --heading-floor-reference --start-paused --max-fps 1 --track-max-fps 8 \
    --preview-fps 2 --cpu-threads 2 >>"$run_dir/perception.log" 2>&1 &
  perception_pid=$!
}

start_navigation() {
  local attempt
  for attempt in 1 2 3; do
    echo "Khởi động Nav2 (lần $attempt/3); chưa gửi goal chuyển động."
    setsid ros2 launch camera/detect_astra/launch/r3_navigation.launch.py \
      >"$run_dir/navigation.log" 2>&1 &
    navigation_pid=$!
    if python3 simulation/r3_wait_localization_active.py --timeout 60 \
        --nodes controller_server planner_server behavior_server velocity_smoother collision_monitor bt_navigator \
        --startup-log "$run_dir/navigation.log" \
        2>&1 | tee -a "$run_dir/navigation_activation.log"; then
      return 0
    fi
    stop_owned "$navigation_pid"
    navigation_pid=''
    mv "$run_dir/navigation.log" "$run_dir/navigation_attempt_${attempt}.log"
    if [[ $attempt -lt 3 ]]; then
      echo 'Nav2 khởi động lỗi dịch vụ; đã đóng nhóm lỗi và tự thử lại, giữ nguyên AMCL và kết quả khớp bản đồ.' >&2
      sleep 1
    fi
  done
  echo "Không kích hoạt được Nav2 sau 3 lần; xem $run_dir/navigation_attempt_*.log. Chưa gửi goal." >&2
  return 2
}

wait_for_localization() {
  local ready_file=$1
  local bridge_log=$2
  local deadline=$((SECONDS + localization_timeout))
  while [[ ! -s "$ready_file" ]]; do
    kill -0 "$localization_pid" 2>/dev/null || { echo 'AMCL đã dừng; xem localization.log' >&2; return 1; }
    kill -0 "$localization_bridge_pid" 2>/dev/null || { echo "Cầu định vị đã dừng; xem $bridge_log" >&2; return 1; }
    if [[ -n "$control_odom_pid" ]]; then
      kill -0 "$control_odom_pid" 2>/dev/null || { echo 'Tiến trình odom điều khiển đã dừng; xem control_odom.log' >&2; return 1; }
    fi
    kill -0 "$bringup_pid" 2>/dev/null || { echo 'Bringup đã dừng; xem bringup.log' >&2; return 1; }
    if (( SECONDS >= deadline )); then
      echo "Không đạt định vị sau $localization_timeout giây; xe không chạy. Xem $bridge_log để phân biệt lỗi dịch vụ ROS và lỗi khớp LiDAR/bản đồ." >&2
      if [[ -r "$bridge_log" ]]; then tail -n 2 "$bridge_log" >&2; fi
      return 2
    fi
    sleep .2
  done
}

localization_nodes_running() {
  local nodes
  if ! nodes=$(ros2 node list --no-daemon); then
    echo 'Không kiểm tra được ROS graph; dừng để tránh mở trùng AMCL/map_server.' >&2
    return 2
  fi
  printf '%s\n' "$nodes" | grep -E '^/(amcl|map_server)$' >/dev/null
}

cleanup() {
  local result=$?
  trap - EXIT
  if [[ $pickup -eq 1 && -n "$bringup_pid" ]]; then
    python3 simulation/r3_lift_control.py stop --execute >"$run_dir/lift_stop.log" 2>&1 || true
  fi
  stop_owned "$global_initializer_pid"
  stop_owned "$safety_pid"
  stop_owned "$navigation_pid"
  stop_owned "$a_watch_pid"
  stop_owned "$inspection_costmaps_pid"
  stop_owned "$camera_obstacles_pid"
  stop_owned "$navigation_telemetry_pid"
  stop_owned "$perception_pid"
  stop_owned "$rviz_pid"
  stop_owned "$control_odom_pid"
  stop_owned "$localization_bridge_pid"
  stop_owned "$localization_pid"
  stop_owned "$bringup_pid"
  stop_owned "$camera_pid"
  if [[ -f "$run_dir/trial/mission_result.json" &&
        ! -s "$run_dir/trial/executed_odom_base.csv" ]]; then
    echo 'Lượt chạy không có mẫu odom; không thể tạo ảnh ba quỹ đạo.' >&2
    [[ $result -ne 0 ]] || result=3
  fi
  if [[ -f "$plan" && -s "$run_dir/trial/executed_odom_base.csv" &&
        -f "$run_dir/trial/mission_result.json" ]]; then
    if camera/detect_astra/.venv/bin/python simulation/r3_render_real_result.py \
       --plan "$plan" --trial "$run_dir/trial" \
       --output "$run_dir/trajectory_result.png" --view laptop; then
      echo "Ảnh ba quỹ đạo: $run_dir/trajectory_result.png"
    else
      echo 'Không vẽ được ảnh từ log odom; xem các tệp trong thư mục lượt chạy.' >&2
      [[ $result -ne 0 ]] || result=3
    fi
  fi
  echo "Kết quả và log: $run_dir"
  exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

if [[ $localization_only -eq 1 || $nav_to_a -eq 1 || $recognize_here -eq 1 ]]; then
  session_mode=stationary_inspection
  if [[ $nav_to_a -eq 1 ]]; then session_mode=navigation_to_A; fi
  if [[ $recognize_here -eq 1 ]]; then session_mode=recognition_here; fi
  python3 - "$run_dir" "$map_file" "$$" "$session_mode" <<'PY'
import json,sys
from pathlib import Path
run,map_file,pid,mode=sys.argv[1:]
started=int(Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19])
path=Path(run).parent/'localization_session.json'
temporary=path.with_suffix('.json.tmp')
temporary.write_text(json.dumps(dict(pid=int(pid),process_start_ticks=started,
    run_dir=run,map_file=map_file,mode=mode),indent=2)+'\n')
temporary.replace(path)
PY
fi

if pgrep -f 'ros2_control_node( |$)|cmd_vel_stamper( |$)|sllidar_node( |$)|r3_real_obstacle_filter.py|r3_real_trial.py' >/dev/null; then
  echo 'Đã có node điều khiển xe hoặc LiDAR ngoài lệnh này; dừng lượt đó trước.' >&2
  exit 1
fi
if pgrep -f 'run_pallet_perception.py|publish_pallet_target.py|publish_pallet_detection.py' >/dev/null; then
  echo 'Đã có node đo pallet khác chạy; dừng lượt đó trước.' >&2
  exit 1
fi
if [[ $localization == amcl ]]; then
  if localization_nodes_running; then
    echo 'Đã có AMCL/map_server bên ngoài lượt này; dừng phiên định vị cũ trước.' >&2
    exit 1
  else
    node_check_rc=$?
    [[ $node_check_rc -eq 1 ]] || exit "$node_check_rc"
  fi
fi

if pgrep -f 'astra_camera_node( |$)' >/dev/null; then
  echo 'Dùng lại camera Astra đang chạy.'
  ros2 param get /camera/camera depth_registration | grep -E 'True$' >/dev/null || {
    echo 'Camera đang chạy nhưng depth_registration chưa bật.' >&2; exit 1;
  }
else
  echo 'Mở camera Astra...'
  ASTRA_POINT_CLOUD=false setsid bash camera/detect_astra/scripts/open_registered_astra.sh \
    >"$run_dir/camera.log" 2>&1 &
  camera_pid=$!
fi

echo 'Mở phần cứng, odom và LiDAR dừng vật cản...'
setsid ros2 launch camera/detect_astra/launch/r3_real_bringup.launch.py \
  >"$run_dir/bringup.log" 2>&1 &
bringup_pid=$!
if [[ $localization == amcl ]]; then
  echo "Mở AMCL định vị LiDAR với bản đồ: $map_file"
  localization_active=0
  inspection_profile=false
  [[ $localization_only -eq 0 ]] || inspection_profile=true
  precision_profile=$inspection_profile
  [[ $nav_to_a -eq 0 && $recognize_here -eq 0 ]] || precision_profile=true
  for localization_attempt in 1 2; do
    setsid ros2 launch camera/detect_astra/launch/r3_localization.launch.py map_file:="$map_file" \
      stationary_inspection:="$inspection_profile" \
      precision_localization:="$precision_profile" \
      >"$run_dir/localization.log" 2>&1 &
    localization_pid=$!
    if python3 simulation/r3_wait_localization_active.py --timeout 30 \
        --startup-log "$run_dir/localization.log" \
        2>&1 | tee -a "$run_dir/localization_activation.log"; then
      localization_active=1
      break
    fi
    stop_owned "$localization_pid"
    localization_pid=''
    mv "$run_dir/localization.log" "$run_dir/localization_attempt_${localization_attempt}.log"
    if [[ $localization_attempt -lt 2 ]]; then
      echo 'Map server/AMCL chưa kích hoạt; đã đóng lượt lỗi, thử khởi động lại một lần.' >&2
    fi
  done
  [[ $localization_active -eq 1 ]] || {
    echo "Không kích hoạt được map server/AMCL; xem $run_dir/localization_attempt_*.log. Phiên kết thúc." >&2
    exit 2
  }
  echo 'Map server và AMCL đã ACTIVE; bản đồ đã sẵn sàng để RViz nhận.'
  global_args=()
  if [[ $global_localization -eq 1 ]]; then
    echo 'Tìm vị trí toàn bản đồ bằng khớp LiDAR; kiểm tra bằng scan mới trước khi khởi tạo AMCL. Xe đứng yên.'
    echo "Tiến độ: $run_dir/global_scan_initialization.log"
    setsid env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 simulation/r3_global_localization.py \
        --map "$map_file" --output "$run_dir/global_scan_initialization.json" --timeout 150 --apply \
        >"$run_dir/global_scan_initialization.log" 2>&1 &
    global_initializer_pid=$!
    # Waiting on an owned background group allows TERM to interrupt startup
    # immediately; Bash defers traps during a foreground Python pipeline.
    if ! wait "$global_initializer_pid"; then
      global_initializer_pid=''
      tail -n 12 "$run_dir/global_scan_initialization.log" >&2
      echo "Chưa xác định được vị trí đáng tin cậy; xem $run_dir/global_scan_initialization.json. Không lưu A." >&2
      exit 2
    fi
    global_initializer_pid=''
    global_args=(--global-seed-ready)
  fi
  if [[ $nav_to_a -eq 1 ]]; then
    # Configure and activate Nav2 before the single parked AMCL refresh. Its
    # startup load must not interrupt the localization evidence being taught.
    start_navigation
    if [[ $nav_check_only -eq 0 ]]; then
      setsid python3 simulation/r3_navigation_telemetry.py --output "$run_dir/navigation_motion.jsonl" \
        >"$run_dir/navigation_telemetry.log" 2>&1 &
      navigation_telemetry_pid=$!
    fi
  fi
  [[ $localization_only -eq 0 ]] || global_args+=(--stationary-inspection)
  [[ $nav_to_a -eq 0 && $recognize_here -eq 0 ]] || global_args+=(--continuous-localization --navigation-monitor --external-control-odom)
  setsid python3 simulation/r3_amcl_bridge.py --map "$map_file" \
    --wait-ready "$run_dir/localization_ready.json" "${initial_pose_args[@]}" "${global_args[@]}" \
    >"$run_dir/localization_bridge.log" 2>&1 &
  localization_bridge_pid=$!
  if [[ $nav_to_a -eq 1 || $recognize_here -eq 1 ]]; then
    setsid python3 simulation/r3_control_odom.py --map "$map_file" >"$run_dir/control_odom.log" 2>&1 &
    control_odom_pid=$!
  fi
  if [[ $use_rviz -eq 1 ]]; then
    setsid rviz2 -d "$root/ros2_ws/config/localization_real.rviz" >"$run_dir/rviz.log" 2>&1 &
    rviz_pid=$!
  fi
  if [[ $global_localization -eq 1 ]]; then
    echo 'Đã khởi tạo AMCL từ khớp scan toàn bản đồ hiện tại; tiếp tục kiểm tra độ tin cậy và khớp tường.'
  else
    echo 'Chờ AMCL ổn định. Dùng 2D Pose Estimate trong RViz để đặt đúng vị trí/hướng hiện tại trên bản đồ.'
  fi
  echo "Trạng thái chi tiết: $run_dir/localization_bridge.log"
  if [[ $localization_only -eq 0 ]]; then
    wait_for_localization "$run_dir/localization_ready.json" "$run_dir/localization_bridge.log"
    echo 'AMCL đã sẵn sàng; khóa mốc bản đồ để đo pallet và lập quỹ đạo.'
  fi
fi

if [[ $camera_obstacles -eq 1 ]]; then
  echo 'Mở vật cản RGB-D dùng chung hiệu chỉnh độ sâu với đo pallet: /camera/obstacle_scan (chấm vàng sẫm).'
  floor_alignment=false
  [[ $localization_only -eq 0 ]] || floor_alignment=true
  setsid ros2 launch camera/detect_astra/launch/r3_existing_camera_obstacles.launch.py \
    floor_alignment:="$floor_alignment" floor_alignment_report:="$run_dir/camera_floor_alignment.json" \
    >"$run_dir/camera_obstacles.log" 2>&1 &
  camera_obstacles_pid=$!
fi

if [[ $localization_only -eq 1 ]]; then
  setsid ros2 launch camera/detect_astra/launch/r3_inspection_costmaps.launch.py \
    >"$run_dir/inspection_costmaps.log" 2>&1 &
  inspection_costmaps_pid=$!
  setsid python3 simulation/r3_localization_watch.py --map "$map_file" \
    --output "$run_dir/a_readiness.json" >"$run_dir/a_readiness.log" 2>&1 &
  a_watch_pid=$!
  echo 'Đã mở global/local costmap chỉ để quan sát, gồm LiDAR và camera vật cản đã áp hiệu chỉnh hiện có.'
  echo "Điều kiện lưu A trực tiếp: $run_dir/a_readiness.json"
  echo 'Phiên này giữ nguyên vị trí A chưa lưu. Khi được yêu cầu, dùng save_r3_a.sh ở terminal khác để lưu từ chính phiên đang chạy.'
  echo 'Có thể đặt 2D Pose Estimate trên RViz laptop khi AMCL chưa sẵn sàng. Ctrl+C để đóng.'
  while kill -0 "$localization_pid" 2>/dev/null && kill -0 "$localization_bridge_pid" 2>/dev/null \
      && kill -0 "$bringup_pid" 2>/dev/null && kill -0 "$camera_obstacles_pid" 2>/dev/null \
      && kill -0 "$inspection_costmaps_pid" 2>/dev/null && kill -0 "$a_watch_pid" 2>/dev/null; do
    sleep 1
  done
  echo 'Một tiến trình định vị đã dừng; xem log.' >&2
  exit 1
fi

if [[ $save_a -eq 1 ]]; then
  python3 simulation/r3_recognition_goal.py save --map "$map_file" --output "$goal_file"
  echo "Đã lưu A mới: $goal_file"
  exit 0
fi

if [[ $nav_check_only -eq 0 ]]; then
  start_perception
fi

if [[ $nav_to_a -eq 1 ]]; then
  echo "Nav2 sẽ đi tới A mới trong: $goal_file"
  if [[ $nav_check_only -eq 0 ]]; then
    # Show the actual current estimate before authorizing motion. A small
    # map-position error can place A behind a forward-only skid-steer robot.
    if ! python3 simulation/r3_recognition_goal.py navigate --map "$map_file" \
        --goal "$goal_file" --output "$run_dir/navigation_start_check.json" \
        --continuous-localization --check-only >"$run_dir/navigation_start_check.log" 2>&1; then
      cat "$run_dir/navigation_start_check.log" >&2
      exit 2
    fi
    python3 simulation/r3_navigation_preview.py --report "$run_dir/navigation_start_check.json"
    read -r -p 'Nhấn Enter để tiếp tục đến vùng A / nhận diện; Ctrl+C để hủy: '
  fi
  # Keep the sensor-only localization monitor throughout Nav2 transit. It
  # never publishes control odometry and does not impose short A->C bounds.
  navigation_args=(--continuous-localization)
  navigation_report="$run_dir/navigation_to_A.json"
  if [[ $nav_check_only -eq 1 ]]; then
    navigation_args+=(--check-only)
    navigation_report="$run_dir/navigation_check.json"
  fi
  python3 simulation/r3_recognition_goal.py navigate --map "$map_file" \
    --goal "$goal_file" --output "$navigation_report" "${navigation_args[@]}"
elif [[ $recognize_here -eq 1 ]]; then
  echo 'Định vị và nhận diện ngay tại vị trí thực tế; không điều chỉnh xe tới tọa độ A.'
  python3 simulation/r3_recognition_goal.py navigate --map "$map_file" \
    --goal "$goal_file" --output "$run_dir/recognition_here.json" \
    --continuous-localization --recognize-here
fi
if [[ $nav_to_a -eq 1 || $recognize_here -eq 1 ]]; then
  # Nav2 must fully release all velocity publishers before the r3 monitor
  # and fixed path follower are started. No concurrent command ownership.
  if [[ $nav_to_a -eq 1 ]]; then
    stop_owned "$navigation_pid"
    navigation_pid=''
    if [[ -n ${navigation_telemetry_pid:-} ]]; then
      stop_owned "$navigation_telemetry_pid"
      navigation_telemetry_pid=''
    fi
  fi
  # Preserve the accepted AMCL posterior. Restarting this monitor and forcing
  # the same parked scene through AMCL again can collapse particle support.
  handoff_name=localization_at_A
  [[ $recognize_here -eq 0 ]] || handoff_name=localization_at_recognition
  python3 simulation/r3_enable_trial.py --map "$map_file" \
    --output "$run_dir/$handoff_name.json" --timeout 30 \
    2>&1 | tee "$run_dir/$handoff_name.log"
  if [[ $nav_check_only -eq 1 ]]; then
    echo 'Đã kiểm tra định vị, Nav2 và bàn giao A→C; không gửi goal và không chạy bánh.'
    exit 0
  fi
  if [[ $recognize_here -eq 1 ]]; then
    echo 'Đã kiểm tra định vị tại chỗ; giữ nguyên tư thế thực tế để nhận diện pallet.'
  else
    echo 'Đã xác nhận xe ở A; giữ nguyên định vị, bắt đầu nhận diện pallet tại vị trí thực tế.'
  fi
fi

if [[ $pickup -eq 1 ]]; then
  python3 simulation/r3_lift_control.py home --execute 2>&1 | tee "$run_dir/lift_home.log"
fi

echo 'Bắt đầu nhận diện YOLO và đo pallet RGB-D mới tại A...'
start_perception
perception_restarts=0

while true; do
  kill -0 "$bringup_pid" 2>/dev/null || { echo 'Bringup đã dừng; xem bringup.log' >&2; exit 1; }
  prepare_rc=4
  if kill -0 "$perception_pid" 2>/dev/null \
      && python3 simulation/r3_perception_switch.py recognize --wait-timeout 20; then
    if python3 simulation/r3_live_prepare.py --output "$run_dir/plan" --timeout 30 --localization "$localization"; then
      break
    else
      prepare_rc=$?
    fi
  fi
  if [[ $prepare_rc -eq 3 ]]; then
    echo 'RGB-D mất dấu trước khi khóa đích; bắt đầu phiên YOLO mới, xe vẫn đứng yên.'
    sleep .2
  elif [[ $prepare_rc -eq 5 ]]; then
    echo 'Đã nhận diện được pallet nhưng chưa có quỹ đạo hợp lệ; xem PLAN_BLOCKED phía trên. Không khởi động lại YOLO vì lỗi lập đường.' >&2
    exit "$prepare_rc"
  elif [[ $prepare_rc -eq 2 || $prepare_rc -eq 4 ]]; then
    if [[ $perception_restarts -ge 2 ]]; then
      echo "Nhận diện chưa khôi phục sau 2 lần mở lại; xe đứng yên. Xem $run_dir/perception.log." >&2
      exit "$prepare_rc"
    fi
    perception_restarts=$((perception_restarts + 1))
    echo "Khởi động lại phần nhận diện tại A ($perception_restarts/2); giữ camera, bản đồ và vị trí xe."
    stop_owned "$perception_pid"
    perception_pid=''
    start_perception
  else
    exit "$prepare_rc"
  fi
done
python3 simulation/r3_real_preflight.py --plan "$plan" --allow-provisional
kill -0 "$perception_pid" 2>/dev/null || {
  echo 'Node đo pallet đã dừng trước khi tắt tracking; xem perception.log' >&2
  exit 1
}
python3 simulation/r3_perception_switch.py pause --wait-timeout 15
echo 'Đã chốt vị trí pallet và tắt YOLO; dừng tracking pallet để giảm tải Pi.'
echo "Ảnh kế hoạch hai tâm: $run_dir/plan/trajectory.svg"
if [[ $capture_only -eq 1 ]]; then
  [[ -s "$run_dir/camera_evidence/index.json" ]] || {
    echo 'Chưa lưu được RGB-D gốc; xem perception.log.' >&2
    exit 3
  }
  echo 'Đã lưu phép đo để kiểm tra hiệu chuẩn. Bộ bám quỹ đạo chưa được khởi chạy.'
  echo "RGB-D gốc và vùng nhận diện: $run_dir/camera_evidence/index.json"
  exit 0
fi

setsid ros2 launch camera/detect_astra/launch/r3_safety.launch.py plan:="$plan" diagnostics_dir:="$run_dir" \
  >"$run_dir/safety.log" 2>&1 &
safety_pid=$!
sleep 2
kill -0 "$safety_pid" 2>/dev/null || { echo 'Bộ dừng vật cản đã dừng; xem safety.log' >&2; exit 1; }

trial_mode=(--arrive-at-c)
[[ $pickup -eq 0 ]] || trial_mode=(--pickup)
set +e
python3 simulation/r3_real_trial.py --plan "$plan" --output "$run_dir/trial" \
  --allow-provisional --execute "${trial_mode[@]}" 2>&1 | tee "$run_dir/trial.log"
trial_rc=${PIPESTATUS[0]}
set -e
[[ $trial_rc -eq 0 ]] || exit "$trial_rc"
if [[ ! -f "$run_dir/trial/mission_result.json" ]]; then
  echo 'Lượt chạy đã hủy trước khi xe bắt đầu.' >&2
  exit 2
fi
mission_status=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' \
  "$run_dir/trial/mission_result.json")
if [[ "$mission_status" == PICKUP_COMPLETE ]]; then
  echo 'Đã chạy A→C→B, dừng bánh tại B và nâng càng lên 8 cm theo phản hồi ESP32.'
elif [[ "$mission_status" == C_ARRIVED ]]; then
  echo 'Đã hoàn thành A→C và dừng tại C; chưa đưa càng vào hoặc nâng pallet.'
elif [[ "$mission_status" == C_ALIGNMENT_WARNING ]]; then
  echo 'Đã đi hết đoạn A→C và dừng; còn sai số tại C được ghi trong c_arrival, không tự xoay chỉnh thêm.'
elif [[ "$mission_status" == C_DIAGNOSTIC_DONE ]]; then
  echo 'Đã dừng chẩn đoán trước C; chưa đưa càng vào pallet. Xem góc odom cuối và ảnh.'
elif [[ "$mission_status" != DONE_TRACKING ]]; then
  echo "Xe dừng ở trạng thái $mission_status; ảnh sẽ ghi quỹ đạo odom đã đi." >&2
  exit 2
else
  echo 'Xe đã tới B theo điều kiện odom; đang lưu ảnh ba quỹ đạo.'
fi
