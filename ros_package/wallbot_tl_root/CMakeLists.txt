cmake_minimum_required(VERSION 3.10)
project(wallbot_tl_root)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz urdf yaml world
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/ndvi_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ndvi_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
