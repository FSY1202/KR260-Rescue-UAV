#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/humble/setup.bash
export MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --base-paths src --executor sequential --symlink-install \
  --packages-up-to kr260_ego_pkg top_launch_pkg kr260_ground_station \
  --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble \
  -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG -g0' \
  -DKR260_LOW_MEMORY_BUILD=ON -DBUILD_TESTING=OFF
