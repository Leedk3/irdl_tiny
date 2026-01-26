cmake_minimum_required(VERSION 3.8)
project(radar_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 라이브러리 빌드
ament_auto_add_library(radar SHARED
  src/radar.cpp
)

# 실행 파일 빌드
ament_auto_add_executable(radar_exe
  src/radar_node.cpp
)

target_link_libraries(radar_exe
  radar
  ${ament_cmake_auto_LIBRARIES}
)

# 헤더 설치 경로 경고 해결 및 패키징
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)