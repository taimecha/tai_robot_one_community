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
retry_home=0
teach_delivery=0
deliver_after_pickup=0
auto_mode=0
resume_run=''
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
  --retry-home) retry_home=1; shift ;;
  --teach-delivery) teach_delivery=1; localization=amcl; shift ;;
  --deliver-after-pickup) deliver_after_pickup=1; shift ;;
  --auto) auto_mode=1; shift ;;
  --resume-delivery) [[ $# -ge 2 && $2 != --* ]] || { echo '--resume-delivery cần thư mục lượt đã nâng pallet.' >&2; exit 2; }; resume_run=$2; shift 2 ;;
  --help|-h)
    echo 'Usage: bash simulation/run_r3_live_trial.sh [--capture-only | --localization-only | --teach-delivery] [--localization amcl|odom] [--localization-timeout SECONDS] [--map ABS.yaml] [--initial-pose X Y YAW_DEG] [--rviz] [--nav-to-a | --nav-check-only | --recognize-here | --save-a] [--pickup [--retry-home] [--deliver-after-pickup]] [--a-file NEW.json] [--global-localization]'
    echo '       bash simulation/run_r3_live_trial.sh --resume-delivery RUN_DIR [--map ABS.yaml] [--camera-obstacles] [--initial-pose X Y YAW_DEG]'
    echo '--capture-only: chốt RGB-D và lưu bằng chứng; không khởi chạy bộ bám quỹ đạo.'
    echo 'Mặc định: AMCL định vị LiDAR + bản đồ chỉnh sửa; tự tìm toàn bản đồ nếu không có initial-pose/RViz. Dùng --localization odom để chạy theo odom cũ.'
    echo '--localization-timeout: tối đa 120 giây mặc định để định vị trước khi đo/chạy; hết hạn sẽ dừng an toàn.'
    echo '--save-a: định vị và lưu A mới tại vị trí đang đứng; không chạy bánh.'
    echo '--nav-to-a: tự định vị, Nav2 tới A mới đã lưu, rồi nhận diện và chạy thử A→C.'
    echo '--nav-check-only: kiểm tra định vị và Nav2 cho luồng tới A; không gửi goal và không chạy A→C.'
    echo '--recognize-here: định vị LiDAR/bản đồ tại chỗ, bỏ bước Nav2 tới A; đo pallet theo tư thế thực tế rồi hỏi Enter trước khi chạy tới C. Có thể thêm --capture-only để chỉ đo và lưu.'
    echo '--localization-only: LiDAR + AMCL + camera vật cản + global/local costmap + kiểm tra A tại chỗ; Ctrl+C để đóng.'
    echo '--camera-obstacles: hiển thị vật cản RGB-D trên RViz. Với --nav-to-a, mở sau khi xe đến A.'
    echo '--pickup: sau Nav2 tới A (hoặc recognize-here), HOME càng, nhận diện, chạy liên tục A→C→B rồi nâng 8 cm; cần firmware HOME 1 mm mới.'
    echo '--retry-home: dùng với --pickup sau khi đã nâng càng rời công tắc đáy; thử lại một lần khi ESP32 còn giữ lỗi HOME_SETTLE_LOWER_UNSTABLE của lượt trước.'
    echo '--teach-delivery: tự định vị, nhận Nav2 Goal từ RViz laptop; tới và dừng rồi chờ xác nhận lưu. Không điều khiển càng.'
    echo '--deliver-after-pickup: dùng với --pickup; sau khi nâng 8 cm, tự tới D đã lưu và hạ càng khi bánh dừng.'
    echo '--resume-delivery RUN_DIR: tiếp tục từ lượt đã nâng thành công và dừng trước khi gửi D. Xe tự giao khi càng và định vị hiện tại đã được xác nhận.'
    exit 0 ;;
  *) echo "Tham số không hỗ trợ: $1" >&2; exit 2 ;;
esac
done
[[ $deliver_after_pickup -eq 0 || $pickup -eq 1 ]] || {
  echo '--deliver-after-pickup cần --pickup.' >&2; exit 2;
}
[[ $retry_home -eq 0 || $pickup -eq 1 ]] || {
  echo '--retry-home cần --pickup.' >&2; exit 2;
}
[[ -z $resume_run || ( $localization == amcl && $pickup -eq 0 && $nav_to_a -eq 0 && $recognize_here -eq 0 &&
                       $teach_delivery -eq 0 && $capture_only -eq 0 && $localization_only -eq 0 && $save_a -eq 0 ) ]] || {
  echo '--resume-delivery dùng riêng với định vị AMCL.' >&2; exit 2;
}
[[ $teach_delivery -eq 0 || ( $pickup -eq 0 && $nav_to_a -eq 0 && $recognize_here -eq 0 && $save_a -eq 0 && $capture_only -eq 0 && $localization_only -eq 0 ) ]] || {
  echo '--teach-delivery dùng riêng, không kết hợp với lấy hàng/nhận diện/lưu A.' >&2; exit 2;
}
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
if [[ $nav_to_a -eq 1 || $recognize_here -eq 1 || $save_a -eq 1 || $teach_delivery -eq 1 ]]; then
  localization=amcl
fi
shared_navigation=0
if [[ $nav_to_a -eq 1 && $deliver_after_pickup -eq 1 ]]; then
  shared_navigation=1
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
if [[ $deliver_after_pickup -eq 1 ]]; then
  python3 simulation/r3_delivery_mission.py inspect --map "$map_file"
fi
if [[ -n $resume_run ]]; then
  resume_run=$(realpath -e -- "$resume_run")
  pickup_to_resume="$resume_run/trial/mission_result.json"
  python3 simulation/r3_delivery_mission.py inspect --map "$map_file" --goal "$resume_run/delivery_goal.json" \
    --pickup-result "$pickup_to_resume" --resume-delivery
fi

for device in /dev/tai_drive /dev/tai_imu /dev/tai_lidar; do
  [[ -e "$device" ]] || { echo "Thiếu thiết bị $device" >&2; exit 1; }
done
[[ -x camera/detect_astra/.venv/bin/python ]] || { echo 'Thiếu Python camera' >&2; exit 1; }
[[ $localization_only -eq 1 || $teach_delivery -eq 1 || -n $resume_run || -d camera/detect_astra/data/runtime_models/best_v3_ncnn_model ]] || {
  echo 'Thiếu model NCNN pallet' >&2; exit 1;
}

mkdir -p simulation/output/r3_live
exec 9>simulation/output/r3_live/.trial.lock
flock -n 9 || { echo 'Đã có một lượt r3 đang chạy' >&2; exit 1; }
# Keep the session lock in this owner shell only. A surviving camera/ROS or
# foreground Python/tee process must not retain fd 9 after the owner exits.
run_dir=$(mktemp -d "$root/simulation/output/r3_live/$(date +%Y%m%d_%H%M%S)_XXXXXX")
cycle_index=1
cycle_dir="$run_dir"
cycle_start_wall=''
cycle_end_wall=''
cycle_rendered=0
plan="$cycle_dir/plan/frozen_plan.json"
delivery_goal="$cycle_dir/delivery_goal.json"
if [[ $deliver_after_pickup -eq 1 ]]; then
  python3 9>&- simulation/r3_delivery_mission.py inspect --map "$map_file" --output "$delivery_goal"
elif [[ -n $resume_run ]]; then
  python3 9>&- simulation/r3_delivery_mission.py inspect --map "$map_file" --goal "$resume_run/delivery_goal.json" \
    --pickup-result "$pickup_to_resume" --resume-delivery --output "$delivery_goal"
fi
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
navigation_nodes=(controller_server planner_server behavior_server velocity_smoother collision_monitor bt_navigator)
camera_obstacles_pid=''
navigation_telemetry_pid=''
inspection_costmaps_pid=''
a_watch_pid=''
delivery_pid=''
trajectory_viz_pid=''

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
    for _ in {1..30}; do
      group_running "$pid" || break
      sleep .05
    done
    if group_running "$pid"; then
      kill -TERM -- "-$pid" 2>/dev/null || true
      for _ in {1..20}; do
        group_running "$pid" || break
        sleep .05
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
  echo 'Nạp model YOLO ở chế độ tạm dừng trước khi bắt đầu nhận diện.'
  local out_dir="${cycle_dir:-$run_dir}"
  setsid 9>&- bash camera/detect_astra/scripts/open_pallet_target.sh \
    --model camera/detect_astra/data/runtime_models/best_v3_ncnn_model \
    --measurement-bias-yaml camera/detect_astra/config/calibration/pallet_target_bias_robot_2026_11_11.yaml \
    --measurement-evidence-dir "$out_dir/camera_evidence" \
    --heading-floor-reference --start-paused --max-fps 2 --track-max-fps 8 \
    --preview-fps 2 --cpu-threads 2 >>"$out_dir/perception.log" 2>&1 &
  perception_pid=$!
}

start_camera_obstacles() {
  [[ ${camera_obstacles:-0} -eq 1 ]] || return 0
  if [[ -n "${camera_obstacles_pid:-}" ]] && kill -0 "$camera_obstacles_pid" 2>/dev/null; then
    return 0
  fi
  echo 'Mở vật cản RGB-D dùng chung hiệu chỉnh độ sâu với đo pallet: /camera/obstacle_scan (chấm vàng sẫm).'
  local floor_alignment=false
  [[ $localization_only -eq 0 ]] || floor_alignment=true
  setsid 9>&- ros2 launch camera/detect_astra/launch/r3_existing_camera_obstacles.launch.py \
    floor_alignment:="$floor_alignment" floor_alignment_report:="$run_dir/camera_floor_alignment.json" \
    >"$run_dir/camera_obstacles.log" 2>&1 &
  camera_obstacles_pid=$!
}

wait_navigation_active() {
  local log_file=$1
  python3 9>&- simulation/r3_wait_localization_active.py --timeout 60 \
    --nodes "${navigation_nodes[@]}" \
    --startup-log "$log_file" \
    2>&1 | tee 9>&- -a "$run_dir/navigation_activation.log"
}

navigation_transition() {
  python3 9>&- simulation/r3_wait_localization_active.py --timeout 20 \
    --nodes "${navigation_nodes[@]}" --navigation-transition "$1" \
    2>&1 | tee 9>&- -a "$run_dir/navigation_activation.log"
}

start_navigation() {
  local loaded=${1:-false}
  local attempt
  local out_dir="${cycle_dir:-$run_dir}"
  local logfile="$out_dir/navigation.log"
  local archive="$out_dir/navigation_attempt"
  local navigation_options=()
  [[ $teach_delivery -eq 0 ]] || navigation_options+=(delivery_teaching:=true)
  if [[ $loaded == true ]]; then
    navigation_options+=(loaded_delivery:=true)
    logfile="$out_dir/navigation_D.log"
    archive="$out_dir/navigation_D_attempt"
  elif [[ $shared_navigation -eq 1 ]]; then
    navigation_options+=(shared_navigation:=true)
  fi
  for attempt in 1 2 3; do
    echo "Khởi động Nav2 (lần $attempt/3); chưa gửi goal chuyển động."
    setsid 9>&- ros2 launch camera/detect_astra/launch/r3_navigation.launch.py "${navigation_options[@]}" \
      >"$logfile" 2>&1 &
    navigation_pid=$!
    if wait_navigation_active "$logfile"; then
      return 0
    fi
    stop_owned "$navigation_pid"
    navigation_pid=''
    mv "$logfile" "${archive}_${attempt}.log"
    if [[ $attempt -lt 3 ]]; then
      echo 'Nav2 khởi động lỗi dịch vụ; đã đóng nhóm lỗi và tự thử lại, giữ nguyên AMCL và kết quả khớp bản đồ.' >&2
      sleep 1
    fi
  done
  echo "Không kích hoạt được Nav2 sau 3 lần; xem ${archive}_*.log. Chưa gửi goal." >&2
  return 2
}

deliver_pallet() {
  local pickup_result=$1
  shift
  local out_dir="${cycle_dir:-$run_dir}"
  if [[ ${camera_obstacles:-0} -eq 1 ]] && declare -F start_camera_obstacles >/dev/null; then
    start_camera_obstacles
  fi
  local delivery_args=(--map "$map_file" --goal "$delivery_goal" --pickup-result "$pickup_result" "$@")
  if [[ $shared_navigation -eq 1 ]]; then
    echo 'Tiếp tục cùng phiên Nav2 để giao tới D.'
    if ! kill -0 "$navigation_pid" 2>/dev/null || ! navigation_transition resume; then
      echo 'Phiên Nav2 đang giữ không phục hồi được. Khởi động lại cho chặng D.' >&2
      stop_owned "$navigation_pid"
      navigation_pid=''
      start_navigation true
    fi
  else
    start_navigation true
  fi
  python3 9>&- simulation/r3_delivery_mission.py navigate "${delivery_args[@]}" \
    --handoff-output "$out_dir/delivery_handoff.json" --output "$out_dir/delivery_arrival.json"
  if [[ ${auto_mode:-0} -eq 1 && $shared_navigation -eq 1 ]] && kill -0 "$navigation_pid" 2>/dev/null && navigation_transition pause; then
    echo 'Đã tạm ngưng Nav2 tại D, giữ phiên sẵn sàng cho chu trình tiếp theo.'
  else
    stop_owned "$navigation_pid"
    navigation_pid=''
  fi
  python3 9>&- simulation/r3_delivery_mission.py lower "${delivery_args[@]}" \
    --arrival "$out_dir/delivery_arrival.json" --output "$out_dir/delivery_result.json"
  echo 'Đã tới D và hoàn tất hạ càng theo phản hồi ESP32. Xe đứng tại D.'
}

render_cycle_outputs() {
  local target_dir="${cycle_dir:-$run_dir}"
  local target_plan="$target_dir/plan/frozen_plan.json"
  stop_owned "$trajectory_viz_pid"
  trajectory_viz_pid=''
  if [[ -f "$target_plan" && -s "$target_dir/trial/executed_odom_base.csv" &&
        -f "$target_dir/trial/mission_result.json" ]]; then
    if camera/detect_astra/.venv/bin/python 9>&- simulation/r3_render_real_result.py \
       --plan "$target_plan" --trial "$target_dir/trial" \
       --output "$target_dir/trajectory_result.png" --view laptop; then
      echo "Ảnh ba quỹ đạo A→C→B: $target_dir/trajectory_result.png"
    else
      echo 'Không vẽ được ảnh A→C→B từ log odom.' >&2
    fi
  fi
  local report_args=(--cycle-dir "$target_dir" --map "$map_file" --cycle-index "${cycle_index:-1}")
  [[ -z "${cycle_start_wall:-}" ]] || report_args+=(--cycle-start-wall "$cycle_start_wall")
  [[ -z "${cycle_end_wall:-}" ]] || report_args+=(--cycle-end-wall "$cycle_end_wall")
  if camera/detect_astra/.venv/bin/python 9>&- simulation/r3_cycle_report.py "${report_args[@]}"; then
    cycle_rendered=1
  else
    echo 'Không vẽ được ảnh quỹ đạo toàn bộ; xem các tệp trong thư mục chu trình.' >&2
  fi
}

run_repeat_cycle() {
  cycle_index=$((cycle_index + 1))
  cycle_dir="$run_dir/cycle_$(printf '%02d' "$cycle_index")"
  mkdir -p "$cycle_dir"
  plan="$cycle_dir/plan/frozen_plan.json"
  delivery_goal="$cycle_dir/delivery_goal.json"
  cp "$run_dir/delivery_goal.json" "$delivery_goal"
  cycle_rendered=0
  echo ""
  echo "============================================================"
  echo "BẮT ĐẦU CHU TRÌNH #$cycle_index (giữ nguyên định vị AMCL ban đầu)"
  echo "Thư mục chu trình: $cycle_dir"
  echo "============================================================"

  # Stop camera_obstacles during repeat cycle's return & YOLO phase if running.
  if [[ -n "${camera_obstacles_pid:-}" ]]; then
    stop_owned "$camera_obstacles_pid"
    camera_obstacles_pid=''
  fi

  python3 9>&- simulation/r3_reset_cycle.py --timeout 20

  setsid 9>&- python3 simulation/r3_trajectory_viz.py --run-dir "$run_dir" --cycle-dir "$cycle_dir" \
    >"$cycle_dir/trajectory_viz.log" 2>&1 &
  trajectory_viz_pid=$!

  shared_navigation=1
  if [[ -n "${navigation_pid:-}" ]] && kill -0 "$navigation_pid" 2>/dev/null && navigation_transition resume; then
    echo 'Đã kích hoạt lại phiên Nav2 đang giữ để quay về A.'
  else
    stop_owned "$navigation_pid"
    navigation_pid=''
    start_navigation
  fi

  cycle_start_wall=$(date +%s.%N)
  local navigation_args=(--continuous-localization --shared-navigation)
  python3 9>&- simulation/r3_recognition_goal.py navigate --map "$map_file" \
    --goal "$goal_file" --output "$cycle_dir/navigation_to_A.json" "${navigation_args[@]}"

  if navigation_transition pause; then
    echo 'Đã tạm ngưng Nav2 để lấy pallet, giữ phiên cho chặng D.'
  else
    shared_navigation=0
    stop_owned "$navigation_pid"
    navigation_pid=''
  fi

  start_perception

  setsid 9>&- python3 simulation/r3_control_odom.py --map "$map_file" \
    --diagnostics "$cycle_dir/control_odom_status.jsonl" >"$cycle_dir/control_odom.log" 2>&1 &
  control_odom_pid=$!
  kill -0 "$control_odom_pid" 2>/dev/null || {
    echo "Tiến trình odom điều khiển đã dừng trước bàn giao; xem $cycle_dir/control_odom.log." >&2
    return 2
  }
  python3 9>&- simulation/r3_enable_trial.py --map "$map_file" \
    --output "$cycle_dir/localization_at_A.json" --timeout 30 \
    2>&1 | tee 9>&- "$cycle_dir/localization_at_A.log"

  local home_args=(--execute --telemetry "$cycle_dir/lift_home_telemetry.jsonl")
  [[ $retry_home -eq 0 ]] || home_args+=(--retry-home)
  python3 9>&- simulation/r3_lift_control.py home "${home_args[@]}" 2>&1 | tee 9>&- "$cycle_dir/lift_home.log"

  echo "Bắt đầu nhận diện YOLO và đo pallet RGB-D tại A (chu trình #$cycle_index)..."
  start_perception
  local perception_restarts=0
  local prepare_rc=4
  while true; do
    kill -0 "$bringup_pid" 2>/dev/null || { echo 'Bringup đã dừng; xem bringup.log' >&2; return 1; }
    prepare_rc=4
    if kill -0 "$perception_pid" 2>/dev/null \
        && python3 9>&- simulation/r3_perception_switch.py recognize --wait-timeout 20; then
      if python3 9>&- simulation/r3_live_prepare.py --output "$cycle_dir/plan" --timeout 30 --localization "$localization"; then
        break
      else
        prepare_rc=$?
      fi
    fi
    if [[ $prepare_rc -eq 3 ]]; then
      echo 'RGB-D mất dấu trước khi khóa đích; bắt đầu phiên YOLO mới, xe vẫn đứng yên.'
      sleep .2
    elif [[ $prepare_rc -eq 5 ]]; then
      echo 'Đã nhận diện được pallet nhưng chưa có quỹ đạo hợp lệ; xem PLAN_BLOCKED phía trên.' >&2
      return "$prepare_rc"
    elif [[ $prepare_rc -eq 2 || $prepare_rc -eq 4 ]]; then
      if [[ $perception_restarts -ge 2 ]]; then
        echo "Nhận diện chưa khôi phục sau 2 lần mở lại; xem $cycle_dir/perception.log." >&2
        return "$prepare_rc"
      fi
      perception_restarts=$((perception_restarts + 1))
      stop_owned "$perception_pid"
      perception_pid=''
      start_perception
    else
      return "$prepare_rc"
    fi
  done

  python3 9>&- simulation/r3_real_preflight.py --plan "$plan" --allow-provisional
  kill -0 "$perception_pid" 2>/dev/null || {
    echo "Node đo pallet đã dừng; xem $cycle_dir/perception.log" >&2
    return 1
  }
  setsid 9>&- ros2 launch camera/detect_astra/launch/r3_safety.launch.py plan:="$plan" diagnostics_dir:="$cycle_dir" \
    >"$cycle_dir/safety.log" 2>&1 &
  safety_pid=$!
  python3 9>&- simulation/r3_perception_switch.py pause --wait-timeout 15
  for _ in {1..25}; do
    grep -q 'Managed nodes are active' "$cycle_dir/safety.log" 2>/dev/null && break
    kill -0 "$safety_pid" 2>/dev/null || break
    sleep .1
  done
  kill -0 "$safety_pid" 2>/dev/null || { echo "Bộ dừng vật cản đã dừng; xem $cycle_dir/safety.log" >&2; return 1; }

  local rep_trial_mode=(--pickup --deliver-after-pickup --yes)
  set +e
  python3 9>&- simulation/r3_real_trial.py --plan "$plan" --output "$cycle_dir/trial" \
    --allow-provisional --execute "${rep_trial_mode[@]}" 2>&1 | tee 9>&- "$cycle_dir/trial.log"
  local rep_rc=${PIPESTATUS[0]}
  set -e
  [[ $rep_rc -eq 0 ]] || return "$rep_rc"
  [[ -f "$cycle_dir/trial/mission_result.json" ]] || return 2
  local rep_status
  rep_status=$(python3 9>&- -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' \
    "$cycle_dir/trial/mission_result.json")
  if [[ "$rep_status" != PICKUP_COMPLETE ]]; then
    echo "Chu trình #$cycle_index dừng ở trạng thái $rep_status." >&2
    return 2
  fi

  stop_owned "$safety_pid"
  safety_pid=''
  stop_owned "$control_odom_pid"
  control_odom_pid=''
  stop_owned "$perception_pid"
  perception_pid=''
  deliver_pallet "$cycle_dir/trial/mission_result.json"
  cycle_end_wall=$(date +%s.%N)
  render_cycle_outputs
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
  if ! nodes=$(ros2 9>&- node list --no-daemon); then
    echo 'Không kiểm tra được ROS graph; dừng để tránh mở trùng AMCL/map_server.' >&2
    return 2
  fi
  printf '%s\n' "$nodes" | grep -E '^/(amcl|map_server)$' >/dev/null
}

cleanup() {
  local result=$?
  trap - EXIT
  stop_owned "$navigation_pid"
  if [[ $pickup -eq 1 && -n "$bringup_pid" ]]; then
    python3 9>&- simulation/r3_lift_control.py stop --execute >"$run_dir/lift_stop.log" 2>&1 || true
  fi
  stop_owned "$global_initializer_pid"
  stop_owned "$safety_pid"
  stop_owned "$delivery_pid"
  stop_owned "$a_watch_pid"
  stop_owned "$inspection_costmaps_pid"
  stop_owned "$camera_obstacles_pid"
  stop_owned "$navigation_telemetry_pid"
  stop_owned "$perception_pid"
  stop_owned "$rviz_pid"
  stop_owned "$trajectory_viz_pid"
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
  if [[ ${cycle_rendered:-0} -eq 0 && -f "$plan" && -s "${cycle_dir:-$run_dir}/trial/executed_odom_base.csv" &&
        -f "${cycle_dir:-$run_dir}/trial/mission_result.json" ]]; then
    if camera/detect_astra/.venv/bin/python 9>&- simulation/r3_render_real_result.py \
       --plan "$plan" --trial "${cycle_dir:-$run_dir}/trial" \
       --output "${cycle_dir:-$run_dir}/trajectory_result.png" --view laptop; then
      echo "Ảnh ba quỹ đạo: ${cycle_dir:-$run_dir}/trajectory_result.png"
    else
      echo 'Không vẽ được ảnh từ log odom; xem các tệp trong thư mục lượt chạy.' >&2
      [[ $result -ne 0 ]] || result=3
    fi
    if [[ -f "${cycle_dir:-$run_dir}/full_trajectory_actual.csv" || -f "${cycle_dir:-$run_dir}/delivery_result.json" ]]; then
      local c_args=(--cycle-dir "${cycle_dir:-$run_dir}" --map "$map_file" --cycle-index "${cycle_index:-1}")
      [[ -z "${cycle_start_wall:-}" ]] || c_args+=(--cycle-start-wall "$cycle_start_wall")
      [[ -z "${cycle_end_wall:-}" ]] || c_args+=(--cycle-end-wall "$cycle_end_wall")
      camera/detect_astra/.venv/bin/python 9>&- simulation/r3_cycle_report.py "${c_args[@]}" || true
    fi
  fi
  echo "Kết quả và log: $run_dir"
  exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

if [[ $localization_only -eq 1 || $nav_to_a -eq 1 || $recognize_here -eq 1 || $teach_delivery -eq 1 || -n $resume_run ]]; then
  session_mode=stationary_inspection
  if [[ $nav_to_a -eq 1 ]]; then session_mode=navigation_to_A; fi
  if [[ $recognize_here -eq 1 ]]; then session_mode=recognition_here; fi
  if [[ $teach_delivery -eq 1 ]]; then session_mode=delivery_teaching; fi
  if [[ -n $resume_run ]]; then session_mode=delivery_resume; fi
  python3 9>&- - "$run_dir" "$map_file" "$$" "$session_mode" <<'PY'
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
  ros2 9>&- param get /camera/camera depth_registration | grep 9>&- -E 'True$' >/dev/null || {
    echo 'Camera đang chạy nhưng depth_registration chưa bật.' >&2; exit 1;
  }
else
  echo 'Mở camera Astra...'
  ASTRA_POINT_CLOUD=false setsid 9>&- bash camera/detect_astra/scripts/open_registered_astra.sh \
    >"$run_dir/camera.log" 2>&1 &
  camera_pid=$!
fi

echo 'Mở phần cứng, odom và LiDAR dừng vật cản...'
setsid 9>&- ros2 launch camera/detect_astra/launch/r3_real_bringup.launch.py \
  >"$run_dir/bringup.log" 2>&1 &
bringup_pid=$!
if [[ $localization == amcl ]]; then
  echo "Mở AMCL định vị LiDAR với bản đồ: $map_file"
  localization_active=0
  inspection_profile=false
  [[ $localization_only -eq 0 ]] || inspection_profile=true
  precision_profile=$inspection_profile
  [[ $nav_to_a -eq 0 && $recognize_here -eq 0 && $teach_delivery -eq 0 && -z $resume_run ]] || precision_profile=true
  for localization_attempt in 1 2; do
    setsid 9>&- ros2 launch camera/detect_astra/launch/r3_localization.launch.py map_file:="$map_file" \
      stationary_inspection:="$inspection_profile" \
      precision_localization:="$precision_profile" \
      >"$run_dir/localization.log" 2>&1 &
    localization_pid=$!
    if python3 9>&- simulation/r3_wait_localization_active.py --timeout 30 \
        --startup-log "$run_dir/localization.log" \
        2>&1 | tee 9>&- -a "$run_dir/localization_activation.log"; then
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
    setsid 9>&- env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 simulation/r3_global_localization.py \
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
  if [[ $nav_to_a -eq 1 || $teach_delivery -eq 1 ]]; then
    # Configure and activate Nav2 before the single parked AMCL refresh. Its
    # startup load must not interrupt the localization evidence being taught.
    start_navigation
    if [[ $nav_check_only -eq 0 ]]; then
      telemetry_args=()
      [[ $shared_navigation -eq 0 ]] || telemetry_args+=(--shared-navigation)
      setsid 9>&- python3 simulation/r3_navigation_telemetry.py --output "$run_dir/navigation_motion.jsonl" \
        "${telemetry_args[@]}" \
        >"$run_dir/navigation_telemetry.log" 2>&1 &
      navigation_telemetry_pid=$!
    fi
  fi
  [[ $localization_only -eq 0 ]] || global_args+=(--stationary-inspection)
  [[ $nav_to_a -eq 0 && $recognize_here -eq 0 && $teach_delivery -eq 0 && -z $resume_run ]] || global_args+=(--continuous-localization --navigation-monitor --external-control-odom)
  [[ -z $resume_run ]] || global_args+=(--resume-delivery)
  setsid 9>&- python3 simulation/r3_amcl_bridge.py --map "$map_file" \
    --wait-ready "$run_dir/localization_ready.json" "${initial_pose_args[@]}" "${global_args[@]}" \
    >"$run_dir/localization_bridge.log" 2>&1 &
  localization_bridge_pid=$!
  setsid 9>&- python3 simulation/r3_trajectory_viz.py --run-dir "$run_dir" \
    >"$run_dir/trajectory_viz.log" 2>&1 &
  trajectory_viz_pid=$!
  if [[ $use_rviz -eq 1 ]]; then
    rviz_config="$root/ros2_ws/config/localization_real.rviz"
    [[ $teach_delivery -eq 0 ]] || rviz_config="$root/ros2_ws/config/r3_laptop.rviz"
    setsid 9>&- rviz2 -d "$rviz_config" >"$run_dir/rviz.log" 2>&1 &
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

# Nav2 to A uses LiDAR. Start the optional RGB-D processing after that leg
# and HOME so its startup load does not compete with the odom/TF handoff.
if [[ $nav_to_a -eq 0 && $recognize_here -eq 0 ]]; then
  start_camera_obstacles
fi

if [[ -n $resume_run ]]; then
  echo 'Tiếp tục chuyến giao hàng với pallet đã nâng. Đang kiểm tra xe và vị trí hiện tại.'
  deliver_pallet "$pickup_to_resume" --resume-delivery
  exit 0
fi

if [[ $teach_delivery -eq 1 ]]; then
  setsid 9>&- python3 simulation/r3_delivery_goal.py serve --map "$map_file" \
    --output "$run_dir/delivery_status.json" >"$run_dir/delivery.log" 2>&1 &
  delivery_pid=$!
  echo 'Giữ cửa sổ RViz laptop hiện có. Khi DELIVERY: READY_FOR_GOAL, chọn Nav2 Goal và kéo hướng xe.'
  echo 'Sau khi xe tới/dừng, chỉ lưu khi người vận hành xác nhận “lưu”; không tự lưu hoặc điều khiển càng.'
  echo "Trạng thái: $run_dir/delivery_status.json"
  while kill -0 "$localization_pid" 2>/dev/null && kill -0 "$localization_bridge_pid" 2>/dev/null \
      && kill -0 "$bringup_pid" 2>/dev/null && kill -0 "$navigation_pid" 2>/dev/null \
      && kill -0 "$delivery_pid" 2>/dev/null; do
    sleep 1
  done
  echo 'Một tiến trình giao hàng đã dừng; xem delivery.log/navigation.log/localization_bridge.log.' >&2
  exit 1
fi

if [[ $localization_only -eq 1 ]]; then
  setsid 9>&- ros2 launch camera/detect_astra/launch/r3_inspection_costmaps.launch.py \
    >"$run_dir/inspection_costmaps.log" 2>&1 &
  inspection_costmaps_pid=$!
  setsid 9>&- python3 simulation/r3_localization_watch.py --map "$map_file" \
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
  python3 9>&- simulation/r3_recognition_goal.py save --map "$map_file" --output "$goal_file"
  echo "Đã lưu A mới: $goal_file"
  exit 0
fi

if [[ $nav_to_a -eq 1 ]]; then
  echo "Nav2 sẽ đi tới A mới trong: $goal_file"
  navigation_args=(--continuous-localization)
  [[ $shared_navigation -eq 0 ]] || navigation_args+=(--shared-navigation)
  if [[ $nav_check_only -eq 0 ]]; then
    # Show the actual current estimate before authorizing motion. A small
    # map-position error can place A behind a forward-only skid-steer robot.
    if ! python3 9>&- simulation/r3_recognition_goal.py navigate --map "$map_file" \
        --goal "$goal_file" --output "$run_dir/navigation_start_check.json" \
        "${navigation_args[@]}" --check-only >"$run_dir/navigation_start_check.log" 2>&1; then
      cat "$run_dir/navigation_start_check.log" >&2
      exit 2
    fi
    python3 9>&- simulation/r3_navigation_preview.py --report "$run_dir/navigation_start_check.json"
    if [[ $auto_mode -eq 1 ]]; then
      echo 'Chế độ tự động (--auto): tự bắt đầu chạy tới A.'
    else
      read -r -p 'Nhấn Enter để tiếp tục đến vùng A / nhận diện; Ctrl+C để hủy: '
    fi
  fi
  # Keep the sensor-only localization monitor throughout Nav2 transit. It
  # never publishes control odometry and does not impose short A->C bounds.
  navigation_report="$run_dir/navigation_to_A.json"
  if [[ $nav_check_only -eq 1 ]]; then
    navigation_args+=(--check-only)
    navigation_report="$run_dir/navigation_check.json"
  fi
  cycle_start_wall=$(date +%s.%N)
  python3 9>&- simulation/r3_recognition_goal.py navigate --map "$map_file" \
    --goal "$goal_file" --output "$navigation_report" "${navigation_args[@]}"
elif [[ $recognize_here -eq 1 ]]; then
  echo 'Định vị và nhận diện ngay tại vị trí thực tế; không điều chỉnh xe tới tọa độ A.'
  python3 9>&- simulation/r3_recognition_goal.py navigate --map "$map_file" \
    --goal "$goal_file" --output "$run_dir/recognition_here.json" \
    --continuous-localization --recognize-here
fi
if [[ $nav_to_a -eq 1 || $recognize_here -eq 1 ]]; then
  # A's relay has exited. The shared Nav2 instance uses only private topics
  # and is paused during docking and lift so it leaves the Pi time to control.
  if [[ $nav_to_a -eq 1 ]]; then
    if [[ -n ${navigation_telemetry_pid:-} ]]; then
      kill -INT -- "-$navigation_telemetry_pid" 2>/dev/null || true
    fi
    if [[ $shared_navigation -eq 1 ]] && navigation_transition pause; then
      echo 'Đã tạm ngưng Nav2 để lấy pallet, giữ phiên cho chặng D.'
    else
      shared_navigation=0
      stop_owned "$navigation_pid"
      navigation_pid=''
    fi
    if [[ -n ${navigation_telemetry_pid:-} ]]; then
      stop_owned "$navigation_telemetry_pid"
      navigation_telemetry_pid=''
    fi
  fi
  # Preload YOLO (paused, zero actuators/velocity topics) in parallel with
  # trial localization handoff and lift homing so model warmup is hidden.
  if [[ $nav_check_only -eq 0 ]]; then
    start_perception
  fi
  # Preserve the accepted AMCL posterior. Restarting this monitor and forcing
  # the same parked scene through AMCL again can collapse particle support.
  handoff_name=localization_at_A
  [[ $recognize_here -eq 0 ]] || handoff_name=localization_at_recognition
  # Start the trial-only odometry owner after Nav2 has released its role.
  # Navigation/startup traffic must not become a failed trial input epoch.
  setsid 9>&- python3 simulation/r3_control_odom.py --map "$map_file" \
    --diagnostics "$run_dir/control_odom_status.jsonl" >"$run_dir/control_odom.log" 2>&1 &
  control_odom_pid=$!
  kill -0 "$control_odom_pid" 2>/dev/null || {
    echo "Tiến trình odom điều khiển đã dừng trước bàn giao; xem $run_dir/control_odom.log." >&2
    exit 2
  }
  python3 9>&- simulation/r3_enable_trial.py --map "$map_file" \
    --output "$run_dir/$handoff_name.json" --timeout 30 \
    2>&1 | tee 9>&- "$run_dir/$handoff_name.log"
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
  home_args=(--execute --telemetry "$run_dir/lift_home_telemetry.jsonl")
  [[ $retry_home -eq 0 ]] || home_args+=(--retry-home)
  python3 9>&- simulation/r3_lift_control.py home "${home_args[@]}" 2>&1 | tee 9>&- "$run_dir/lift_home.log"
fi

echo 'Bắt đầu nhận diện YOLO và đo pallet RGB-D mới tại A...'
start_perception
perception_restarts=0

while true; do
  kill -0 "$bringup_pid" 2>/dev/null || { echo 'Bringup đã dừng; xem bringup.log' >&2; exit 1; }
  prepare_rc=4
  if kill -0 "$perception_pid" 2>/dev/null \
      && python3 9>&- simulation/r3_perception_switch.py recognize --wait-timeout 20; then
    if python3 9>&- simulation/r3_live_prepare.py --output "$run_dir/plan" --timeout 30 --localization "$localization"; then
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
python3 9>&- simulation/r3_real_preflight.py --plan "$plan" --allow-provisional
kill -0 "$perception_pid" 2>/dev/null || {
  echo 'Node đo pallet đã dừng trước khi tắt tracking; xem perception.log' >&2
  exit 1
}
if [[ $capture_only -eq 0 ]]; then
  setsid 9>&- ros2 launch camera/detect_astra/launch/r3_safety.launch.py plan:="$plan" diagnostics_dir:="$run_dir" \
    >"$run_dir/safety.log" 2>&1 &
  safety_pid=$!
fi
python3 9>&- simulation/r3_perception_switch.py pause --wait-timeout 15
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

for _ in {1..25}; do
  grep -q 'Managed nodes are active' "$run_dir/safety.log" 2>/dev/null && break
  kill -0 "$safety_pid" 2>/dev/null || break
  sleep .1
done
kill -0 "$safety_pid" 2>/dev/null || { echo 'Bộ dừng vật cản đã dừng; xem safety.log' >&2; exit 1; }

trial_mode=(--arrive-at-c)
[[ $pickup -eq 0 ]] || trial_mode=(--pickup)
[[ $deliver_after_pickup -eq 0 ]] || trial_mode+=(--deliver-after-pickup)
[[ ${auto_mode:-0} -eq 0 ]] || trial_mode+=(--yes)
set +e
python3 9>&- simulation/r3_real_trial.py --plan "$plan" --output "$run_dir/trial" \
  --allow-provisional --execute "${trial_mode[@]}" 2>&1 | tee 9>&- "$run_dir/trial.log"
trial_rc=${PIPESTATUS[0]}
set -e
[[ $trial_rc -eq 0 ]] || exit "$trial_rc"
if [[ ! -f "$run_dir/trial/mission_result.json" ]]; then
  echo 'Lượt chạy đã hủy trước khi xe bắt đầu.' >&2
  exit 2
fi
mission_status=$(python3 9>&- -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' \
  "$run_dir/trial/mission_result.json")
if [[ "$mission_status" == PICKUP_COMPLETE ]]; then
  echo 'Đã chạy A→C→B, dừng bánh tại B và nâng càng lên 8 cm theo phản hồi ESP32.'
  if [[ $deliver_after_pickup -eq 1 ]]; then
    stop_owned "$safety_pid"
    safety_pid=''
    stop_owned "$control_odom_pid"
    control_odom_pid=''
    stop_owned "$perception_pid"
    perception_pid=''
    deliver_pallet "$run_dir/trial/mission_result.json"
    if [[ ${auto_mode:-0} -eq 1 ]]; then
      cycle_end_wall=$(date +%s.%N)
      render_cycle_outputs
      while [[ -t 0 ]]; do
        echo ''
        if ! read -r -p 'Nhập "r" rồi Enter để quay về A tiếp tục lấy và giao (hoặc "q" / Ctrl+C để thoát): ' next_cmd; then
          break
        fi
        if [[ "$next_cmd" == q || "$next_cmd" == Q ]]; then
          break
        fi
        if [[ "$next_cmd" != r && "$next_cmd" != R ]]; then
          echo 'Chỉ nhận "r" để tiếp tục quay về A hoặc "q" để kết thúc.'
          continue
        fi
        run_repeat_cycle
      done
    fi
  fi
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
