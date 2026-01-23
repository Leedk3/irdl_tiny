cmake_minimum_required(VERSION 3.8)
project(radar_monitor_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies()

include_directories(include)

# Library 
ament_auto_add_library(radar_monitor SHARED
  src/radar_monitor.cpp
)

# Executable  
ament_auto_add_executable(radar_monitor_exe
  src/radar_monitor_node.cpp
)

target_link_libraries(radar_monitor_exe
  radar_monitor
)

ament_target_dependencies(radar_monitor_exe
  rclcpp
  geometry_msgs
)

ament_auto_package()