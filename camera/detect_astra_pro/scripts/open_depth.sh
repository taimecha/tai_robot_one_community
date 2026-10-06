#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source /home/tai/tai_robot_one_community/ros2_ws/install/astra_depth/setup.bash

astra_usb_device=$(lsusb -d 2bc5:0403 | awk 'NR == 1 { gsub(":", "", $4); printf "/dev/bus/usb/%s/%s", $2, $4 }')
if [[ -n "$astra_usb_device" && ! -w "$astra_usb_device" ]]; then
  echo "Astra depth chưa có quyền truy cập: $astra_usb_device" >&2
  echo "Chạy: sudo chgrp plugdev $astra_usb_device && sudo chmod g+rw $astra_usb_device" >&2
  exit 1
fi

existing_depth_pid=$(pgrep -f '^/home/tai/tai_robot_one_community/ros2_ws/install/astra_depth/astra_camera/lib/astra_camera/astra_camera_node( |$)' | head -n 1 || true)
if [[ -n "$existing_depth_pid" ]]; then
  echo "Astra depth đang chạy (PID $existing_depth_pid). Dùng RViz để xem luồng hiện có; dừng tiến trình này trước khi mở lại." >&2
  exit 0
fi

export LD_LIBRARY_PATH="/home/tai/tai_robot_one_community/ros2_ws/install/astra_depth/astra_camera/lib:/home/tai/.local/orbbec_ros2_astra_camera/astra_camera/openni2_redist/x64:/home/tai/.local/ros_jazzy_astra_deps/usr/lib/x86_64-linux-gnu:/home/tai/.local/ros_jazzy_astra_deps/opt/ros/jazzy/lib:${LD_LIBRARY_PATH:-}"

exec ros2 launch astra_camera astra_pro.launch.xml enable_color:=false use_uvc_camera:=false enable_point_cloud:=true
