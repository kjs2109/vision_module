cmake_minimum_required(VERSION 3.14)
project(tier4_map_launch)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  rviz
)
