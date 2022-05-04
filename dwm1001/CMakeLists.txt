cmake_minimum_required(VERSION 3.8)
project(dwm1001)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(dwm1001_interface REQUIRED)
include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(dwm1001 src/rs232.cpp src/dwm1001.cpp)

ament_target_dependencies(dwm1001
  dwm1001_interface
  std_msgs
  rclcpp_lifecycle)

install(TARGETS dwm1001
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
