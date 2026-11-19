#!/usr/bin/env bash
# Only run after the operator explicitly confirms saving the delivery pose.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
source /opt/ros/jazzy/setup.bash
exec python3 "$root/simulation/r3_delivery_goal.py" save "$@"
