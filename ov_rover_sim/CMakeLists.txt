cmake_minimum_required(VERSION 3.8)
project(ov_rover_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY launch worlds models urdf config rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/auto_loop.py scripts/key_teleop.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
