cmake_minimum_required(VERSION 3.8)
project(experiment_cat)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz urdf meshes udev
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/imu_guard scripts/odom_test scripts/motor_rpm_test scripts/sector_scan_filter scripts/odom_source scripts/navigation_safety
  DESTINATION lib/${PROJECT_NAME})
ament_package()
