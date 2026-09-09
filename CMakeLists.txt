cmake_minimum_required(VERSION 3.8)
project(experiment_nav2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
