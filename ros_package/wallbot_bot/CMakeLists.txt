cmake_minimum_required(VERSION 3.5)
project(wallbot_bot)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install launch/urdf/config/meshes
install(DIRECTORY launch urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()