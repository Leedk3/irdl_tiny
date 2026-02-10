cmake_minimum_required(VERSION 3.8)
project(uni_node)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1. 의존성 패키지 불러오기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 2. 헤더 파일 경로 설정 (include 폴더)
include_directories(include)

# 3. 실행 파일 빌드 설정
add_executable(uni_app src/uni_node.cpp)

# 4. 링크 라이브러리 설정 (실행 파일에 rclcpp 연결)
ament_target_dependencies(uni_app rclcpp)

# 5. 설치 경로 설정 (ros2 run을 위해 필요)
install(TARGETS uni_app
  DESTINATION lib/${PROJECT_NAME})

ament_package()