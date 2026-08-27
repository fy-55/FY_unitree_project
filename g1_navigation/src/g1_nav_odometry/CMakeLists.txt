cmake_minimum_required(VERSION 3.8)
project(g1_nav_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_go REQUIRED)

add_executable(odom_tf_bridge src/odom_tf_bridge.cpp)
ament_target_dependencies(
  odom_tf_bridge
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)

add_executable(fake_odom_publisher src/fake_odom_publisher.cpp)
ament_target_dependencies(
  fake_odom_publisher
  nav_msgs
  rclcpp
)

add_executable(sportmode_odom_adapter src/sportmode_odom_adapter.cpp)
ament_target_dependencies(
  sportmode_odom_adapter
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  unitree_go
)

install(
  TARGETS
    odom_tf_bridge
    fake_odom_publisher
    sportmode_odom_adapter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
