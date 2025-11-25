cmake_minimum_required(VERSION 3.8)
project(go2_d1_description)

find_package(ament_cmake REQUIRED)

# Install description assets
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
