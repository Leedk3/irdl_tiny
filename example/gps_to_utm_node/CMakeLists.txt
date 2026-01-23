cmake_minimum_required(VERSION 3.8)
project(gps_to_utm_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PkgConfig REQUIRED)
pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

include_directories(include ${GEOGRAPHICLIB_INCLUDE_DIRS})

# Library 
ament_auto_add_library(gps_to_utm SHARED
  src/gps_to_utm.cpp
)

ament_target_dependencies(gps_to_utm
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(gps_to_utm ${GEOGRAPHICLIB_LIBRARIES})

# Executable 
ament_auto_add_executable(gps_to_utm_exe
  src/gps_to_utm_node.cpp)

# 실행 파일에 라이브러리 및 ROS 2 의존성 연결
target_link_libraries(gps_to_utm_exe gps_to_utm)
ament_target_dependencies(gps_to_utm_exe 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)

ament_auto_package()