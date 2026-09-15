cmake_minimum_required(VERSION 3.3)
project(ov_data)

# ROS 2 only build (ROS 1 support has been removed from this repo)
find_package(ament_cmake REQUIRED)
message(STATUS "ROS *2* version found")
ament_package()
