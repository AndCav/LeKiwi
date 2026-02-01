cmake_minimum_required(VERSION 3.10)
project(lekiwi)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    rviz
    scripts
    urdf
    DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/lekiwi_joint_command_mux.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
