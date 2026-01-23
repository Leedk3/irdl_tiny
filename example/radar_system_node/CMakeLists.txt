cmake_minimum_required(VERSION 3.8)
project(radar_system_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include_directories(include)

# Library 
ament_auto_add_library(radar_system SHARED
  src/radar_system.cpp)

# Executable 
ament_auto_add_executable(radar_system_exe
  src/radar_system_node.cpp
)

target_link_libraries(radar_system_exe 
radar_system
)

ament_target_dependencies(radar_system_exe
  rclcpp
  nav_msgs
  visualization_msgs
  geometry_msgs
)

ament_auto_package()