#!/usr/bin/env bash
. /opt/ros/$ROS_DISTRO/setup.bash
cd /ros_ws
colcon build --base-paths src/* "$@" \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_INSTALL_PREFIX=/ros_ws/install
``