#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

exec rviz2 -d /home/tai/astra_pallets/config/astra_depth.rviz
