#!/usr/bin/env bash
# Build the robot and lidar in an isolated prefix. Never opens serial devices.
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
set -u
robot_source="$project_dir/../../ros2_ws"
lidar_source=${PALLET_LIDAR_SOURCE:-$robot_source/src/sllidar_ros2}
if [[ ! -f "$lidar_source/package.xml" ]]; then
  echo "Không tìm thấy mã driver lidar: $lidar_source" >&2
  exit 1
fi
cd "$project_dir"
export MAKEFLAGS='-j1 -l1'
export CMAKE_BUILD_PARALLEL_LEVEL=1
exec colcon --log-base data/alignment_robot_ws/log build \
  --base-paths "$robot_source" "$lidar_source" \
  --packages-select tai_robot_one sllidar_ros2 \
  --build-base data/alignment_robot_ws/build \
  --install-base data/alignment_robot_ws/install \
  --executor sequential --cmake-args -DBUILD_TESTING=OFF
