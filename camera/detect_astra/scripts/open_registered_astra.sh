#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
astra_overlay=${ASTRA_OVERLAY:-/home/tai/tai_robot_one_community/ros2_ws/install/astra_depth}
source "$astra_overlay/setup.bash"
set -u

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
rgb_calibration_file="$project_dir/config/calibration/astra_pro_rgb_640x480_2026-10-11.yaml"
if [[ ! -r "$rgb_calibration_file" ]]; then
  echo "Không đọc được file hiệu chuẩn RGB: $rgb_calibration_file" >&2
  exit 1
fi

for astra_product_id in 0403 0501; do
  astra_usb_device=$(lsusb -d "2bc5:$astra_product_id" | awk 'NR == 1 { gsub(":", "", $4); printf "/dev/bus/usb/%s/%s", $2, $4 }')
  if [[ -z "$astra_usb_device" ]]; then
    echo "Chưa thấy thiết bị Astra USB 2bc5:$astra_product_id." >&2
    exit 1
  fi
  if [[ ! -w "$astra_usb_device" ]]; then
    echo "Astra chưa có quyền truy cập: $astra_usb_device (2bc5:$astra_product_id)" >&2
    echo "Chạy: sudo chgrp plugdev $astra_usb_device && sudo chmod g+rw $astra_usb_device" >&2
    exit 1
  fi
done

if pgrep -f 'astra_camera_node( |$)' >/dev/null; then
  echo 'Astra camera node đã chạy. Kiểm tra depth_registration trước khi mở thêm node.' >&2
  exit 1
fi

export LD_LIBRARY_PATH="/home/tai/tai_robot_one_community/ros2_ws/install/astra_depth/astra_camera/lib:/home/tai/.local/orbbec_ros2_astra_camera/astra_camera/openni2_redist/x64:/home/tai/.local/ros_jazzy_astra_deps/usr/lib/x86_64-linux-gnu:/home/tai/.local/ros_jazzy_astra_deps/opt/ros/jazzy/lib:${LD_LIBRARY_PATH:-}"

exec ros2 launch astra_camera astra_pro.launch.xml \
  depth_registration:=true \
  enable_color:=true \
  use_uvc_camera:=true \
  enable_point_cloud:=true \
  enable_colored_point_cloud:=false \
  color_info_url:="file://$rgb_calibration_file"
