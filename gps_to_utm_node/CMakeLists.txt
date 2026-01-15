cmake_minimum_required(VERSION 3.8)
project(gps_to_utm_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PkgConfig REQUIRED)
pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

include_directories(include ${GEOGRAPHICLIB_INCLUDE_DIRS})

#Library
ament_auto_add_library(gps_to_utm SHARED
  src/gps_to_utm.cpp)

target_link_libraries(gps_to_utm 
${GEOGRAPHICLIB_LIBRARIES}
)

#Executable
ament_auto_add_executable(gps_to_utm_exe
  src/gps_to_utm_node.cpp
)
target_link_libraries(gps_to_utm_exe 
  gps_to_utm)

ament_auto_package()