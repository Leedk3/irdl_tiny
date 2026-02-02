cmake_minimum_required(VERSION 3.8)
project(intruder_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include_directories(include)

# Library 
ament_auto_add_library(intruder SHARED
  src/intruder.cpp
)

# Executable 
ament_auto_add_executable(intruder_exe
  src/intruder_node.cpp
)

target_link_libraries(intruder_exe
  intruder
)

install(TARGETS intruder_exe
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package()