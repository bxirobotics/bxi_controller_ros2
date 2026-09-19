#!/bin/bash
set -e
if [ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]; then
    source /opt/bxi/bxi_ros2_pkg/setup.bash
fi
BUILD_TYPE=Release
colcon build \
        --merge-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On"\
        -Wall -Wextra -Wpedantic