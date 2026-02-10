cmake_minimum_required(VERSION 3.8)
project(udp_communication_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

find_path(
  GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

include_directories(
    include
    ${GeographicLib_INCLUDE_DIRS}
)

add_definitions(
    -DBOOST_COROUTINE_NO_DEPRECATION_WARNING=1
    -DBOOST_ALL_DYN_LINK=1
    -DBOOST_ERROR_CODE_HEADER_ONLY=1
)

# Library
ament_auto_add_library(udp_communication SHARED
      src/udp_communication.cpp
      src/udp_receiver.cpp
      src/udp_sender.cpp)


target_link_libraries(udp_communication
    ${OpenCV_LIBS}
    boost_system
    boost_filesystem
    ${GeographicLib_LIBRARIES}
)
 
# Executable
ament_auto_add_executable(udp_communication_exe
    src/udp_communication_node.cpp)

target_link_libraries(udp_communication_exe
  udp_communication
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
        config
        launch)