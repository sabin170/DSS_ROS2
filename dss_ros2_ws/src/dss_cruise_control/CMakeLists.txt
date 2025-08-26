cmake_minimum_required(VERSION 3.8)
project(dss_cruise_control)

# C++ 컴파일러 옵션 설정
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# nlohmann_json 찾기 (JSON 파싱용)
find_package(nlohmann_json REQUIRED)

# C++ 실행 파일 생성
add_executable(cruise_control_node
  src/cruise_control_node.cpp
)

# C++17 표준 사용
set_property(TARGET cruise_control_node PROPERTY CXX_STANDARD 17)

# ROS2 의존성 연결
ament_target_dependencies(cruise_control_node
  rclcpp
  std_msgs
  nav_msgs
)

# nlohmann_json 링크
target_link_libraries(cruise_control_node
  nlohmann_json::nlohmann_json
)

# 실행 파일 설치
install(TARGETS
  cruise_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# 런치 파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 테스트 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 패키지 설정 완료
ament_package()
