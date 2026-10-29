#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
config="$project_dir/config/pallet_distance_spatial_corrected_preview.rviz"

if ros2 node list 2>/dev/null | grep -Fxq '/spatial_depth_cloud'; then
  exec rviz2 -d "$config"
fi

python3 "$project_dir/scripts/publish_rgb_calibrated_cloud.py" --ros-args \
  -r __node:=spatial_depth_cloud \
  --params-file "$project_dir/config/calibration/astra_pro_depth_x_skew_preview.yaml" &
cloud_pid=$!
cleanup() {
  kill -INT "$cloud_pid" 2>/dev/null || true
  wait "$cloud_pid" 2>/dev/null || true
}
trap cleanup EXIT

rviz2 -d "$config"
