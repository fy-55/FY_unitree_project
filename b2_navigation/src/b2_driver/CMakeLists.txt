cmake_minimum_required(VERSION 3.8)
project(b2_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(b2_pub "src/b2_pub.cpp")
add_executable(b2_sub "src/b2_sub.cpp")
add_executable(b2_walk "src/b2_walk.cpp" "src/common/ros2_b2_sport_client.cpp")
add_executable(b2_move_test "src/b2_move_test.cpp" "src/common/ros2_b2_sport_client.cpp")
add_executable(b2_keyboard_cmd_vel "src/b2_keyboard_cmd_vel.cpp" "src/common/ros2_b2_sport_client.cpp")
add_executable(rslidar_relay "src/rslidar_relay.cpp")
add_executable(rslidar_simple_relay "src/rslidar_simple_relay.cpp")
add_executable(b2_path_follower "src/b2_path_follower.cpp")
add_executable(b2_path_generator "src/b2_path_generator.cpp")
add_executable(b2_obstacle_detector "src/b2_obstacle_detector.cpp")

target_include_directories(b2_walk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(b2_move_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  b2_pub 
  "rclcpp"
  "unitree_go"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "nav_msgs"
)

ament_target_dependencies(
  b2_sub 
  "rclcpp"
  "unitree_go"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "nav_msgs"
)

ament_target_dependencies(
  b2_walk
  "rclcpp"
  "unitree_go"
  "unitree_api"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "nav_msgs"
)

ament_target_dependencies(
  b2_move_test
  "rclcpp"
  "unitree_go"
  "unitree_api"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "nav_msgs"
)

ament_target_dependencies(
  b2_keyboard_cmd_vel
  "rclcpp"
  "geometry_msgs"
  "unitree_api"
)

target_include_directories(b2_keyboard_cmd_vel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  rslidar_relay
  "rclcpp"
  "sensor_msgs"
)

ament_target_dependencies(
  rslidar_simple_relay
  "rclcpp"
  "sensor_msgs"
)

ament_target_dependencies(
  b2_path_follower
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "tf2_ros"
  "tf2"
)

ament_target_dependencies(
  b2_path_generator
  "rclcpp"
  "nav_msgs"
)

ament_target_dependencies(
  b2_obstacle_detector
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
  "std_msgs"
)

install(TARGETS
  b2_pub
  b2_sub
  b2_walk
  b2_move_test
  b2_keyboard_cmd_vel
  rslidar_relay
  rslidar_simple_relay
  b2_path_follower
  b2_path_generator
  b2_obstacle_detector
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch rviz params
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
