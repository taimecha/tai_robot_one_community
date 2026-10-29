#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

if ros2 node list 2>/dev/null | grep -Fxq '/rgb_calibrated_cloud'; then
  exec rviz2 -d "$project_dir/config/pallet_distance_rgb_calibrated.rviz"
fi

python3 "$project_dir/scripts/publish_rgb_calibrated_cloud.py" &
cloud_pid=$!
cleanup() {
  kill -INT "$cloud_pid" 2>/dev/null || true
  wait "$cloud_pid" 2>/dev/null || true
}
trap cleanup EXIT

rviz2 -d "$project_dir/config/pallet_distance_rgb_calibrated.rviz"
