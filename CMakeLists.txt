cmake_minimum_required(VERSION 3.8)
project(wallbot)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGoal.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs)

install(PROGRAMS
  src/demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
