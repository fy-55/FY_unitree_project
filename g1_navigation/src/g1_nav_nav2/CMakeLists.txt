cmake_minimum_required(VERSION 3.8)
project(g1_nav_nav2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    behavior_trees
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/static_footprint_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME static_footprint_publisher
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
