cmake_minimum_required(VERSION 3.8)
project(zeromq_bridge_node)

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

find_package(OpenCV 4 REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

include_directories(
    include
)

add_definitions(
    -DBOOST_COROUTINE_NO_DEPRECATION_WARNING=1
    -DBOOST_ALL_DYN_LINK=1
    -DBOOST_ERROR_CODE_HEADER_ONLY=1
)

# Library
ament_auto_add_library(zeromq_bridge_pub SHARED
      src/zeromq_bridge_pub.cpp)


target_link_libraries(zeromq_bridge_pub
    ${OpenCV_LIBS}
    Boost::system
    Boost::filesystem
    zmq
)

ament_auto_add_library(zeromq_bridge_sub SHARED
      src/zeromq_bridge_sub.cpp)


target_link_libraries(zeromq_bridge_sub
    ${OpenCV_LIBS}
    Boost::system
    Boost::filesystem
    zmq
)

# Executable
ament_auto_add_executable(zeromq_bridge_pub_exe
    src/zeromq_bridge_pub_node.cpp)

target_link_libraries(zeromq_bridge_pub_exe
  zeromq_bridge_pub
)

ament_auto_add_executable(zeromq_bridge_sub_exe
    src/zeromq_bridge_sub_node.cpp)

target_link_libraries(zeromq_bridge_sub_exe
  zeromq_bridge_sub
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