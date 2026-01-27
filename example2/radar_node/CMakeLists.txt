cmake_minimum_required(VERSION 3.8)
project(radar_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Library 
ament_auto_add_library(radar SHARED
  src/radar.cpp
)

target_include_directories(radar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Executable 
ament_auto_add_executable(radar_node_exe
  src/radar_node.cpp
)

target_link_libraries(radar_node_exe
  radar
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)