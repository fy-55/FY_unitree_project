cmake_minimum_required(VERSION 3.8)
project(g1_nav_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_api REQUIRED)

add_executable(g1_velocity_bridge src/g1_velocity_bridge.cpp)
ament_target_dependencies(
  g1_velocity_bridge
  geometry_msgs
  rcl_interfaces
  rclcpp
  sensor_msgs
  unitree_api
)

install(
  TARGETS g1_velocity_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
