cmake_minimum_required(VERSION 3.8)
project(ownship_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

# Library
ament_auto_add_library(ownship SHARED
  src/ownship.cpp
)

# Executable
ament_auto_add_executable(ownship_exe
  src/ownship_node.cpp
)

target_link_libraries(ownship_exe
  ownship
)

install(TARGETS ownship_exe
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package()