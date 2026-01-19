cmake_minimum_required(VERSION 3.8)
project(gps_to_utm_node)

# 1. C++ 표준 설정 (GeographicLib 및 tf2 사용을 위해 17 권장)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 2. 의존성 패키지 로드
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 3. 외부 라이브러리 (GeographicLib) 설정
find_package(PkgConfig REQUIRED)
pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

# 4. 헤더 파일 경로 포함
include_directories(include ${GEOGRAPHICLIB_INCLUDE_DIRS})

# 5. Library 빌드 (변환 로직)
ament_auto_add_library(gps_to_utm SHARED
  src/gps_to_utm.cpp
)

# [핵심 수정] 라이브러리 자체도 rclcpp와 tf2 등을 인지해야 링크 에러가 나지 않습니다.
ament_target_dependencies(gps_to_utm
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(gps_to_utm ${GEOGRAPHICLIB_LIBRARIES})

# 6. Executable 빌드 (메인 노드)
ament_auto_add_executable(gps_to_utm_exe
  src/gps_to_utm_node.cpp
)

# 실행 파일에 라이브러리 및 ROS 2 의존성 연결
target_link_libraries(gps_to_utm_exe gps_to_utm)
ament_target_dependencies(gps_to_utm_exe 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)

# 7. 패키지 구성 및 설치 자동 완료
ament_auto_package()