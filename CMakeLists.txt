cmake_minimum_required(VERSION 3.8)
project(experiment_nav2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz urdf meshes udev
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/imu_guard scripts/odom_test scripts/motor_rpm_test scripts/sector_scan_filter
  DESTINATION lib/${PROJECT_NAME})
ament_package()
