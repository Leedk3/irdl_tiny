cmake_minimum_required(VERSION 3.5)
project(uam_route_planner)

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
# find_package(PCL 1.12 REQUIRED)


find_path (PUGIXML_INCLUDE_DIR NAMES pugixml.hpp PATHS ${PUGIXML_HOME}/include)
find_library (PUGIXML_LIBRARY NAMES pugixml PATHS ${PUGIXML_HOME}/lib)
include (FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS (PugiXML DEFAULT_MSG PUGIXML_LIBRARY
PUGIXML_INCLUDE_DIR)

if (PUGIXML_FOUND)
  set (PUGIXML_LIBRARIES ${PUGIXML_LIBRARY})
  message (STATUS "PugiXML include = ${PUGIXML_INCLUDE_DIR}")
  message (STATUS "PugiXML library = ${PUGIXML_LIBRARY}")
else (PUGIXML_FOUND)
  message (STATUS "No PugiXML found")
endif(PUGIXML_FOUND)

mark_as_advanced (PUGIXML_LIBRARY PUGIXML_INCLUDE_DIR)


find_path(
  GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})


include_directories(
    include
    ${PUGIXML_INCLUDE_DIR}
    ${GeographicLib_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}  
)

add_definitions(
    -DBOOST_COROUTINE_NO_DEPRECATION_WARNING=1
    -DBOOST_ALL_DYN_LINK=1
    -DBOOST_ERROR_CODE_HEADER_ONLY=1
)

add_definitions(${GeographicLib_DEFINITIONS})


# Library
ament_auto_add_library(${PROJECT_NAME} 
  src/uam_route_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PUGIXML_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Executable
ament_auto_add_executable(${PROJECT_NAME}_exe 
  src/uam_route_planner_node.cpp
)

target_link_libraries(${PROJECT_NAME}_exe 
  ${PROJECT_NAME}
  ${PUGIXML_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_test
          test/gtest_main.cpp
          )

  target_include_directories(${PROJECT_NAME}_test PRIVATE "include")
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} )
  ament_target_dependencies(${PROJECT_NAME}_test
          "rclcpp")
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  waypoints
  config)
