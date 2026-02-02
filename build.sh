#!/usr/bin/env bash
. /opt/ros/$ROS_DISTRO/setup.bash
cd /irdl_ws
colcon build --symlink-install --base-paths src/* "$@" --packages-skip ouster_ros ros_deep_learning 4fn_basic_viewer \
    --cmake-args \
    -DCMAKE_MODULE_PATH=/tmp/OpenCV/opencv-4.10.0/cmake \
    -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_INSTALL_PREFIX=/irdl_ws/install
``