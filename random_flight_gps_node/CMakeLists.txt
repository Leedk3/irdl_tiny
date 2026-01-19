cmake_minimum_required(VERSION 3.8)
project(random_flight_gps)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

# Library 
ament_auto_add_library(random_flight_gps_lib SHARED
  src/random_flight_gps.cpp
)

# Executable 
ament_auto_add_executable(random_flight_gps_node
  src/random_flight_gps_node.cpp
)
target_link_libraries(random_flight_gps_node random_flight_gps_lib)

ament_auto_package()