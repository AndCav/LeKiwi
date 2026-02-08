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
    scripts/lekiwi_cmd_vel_to_wheel_velocity.py
    scripts/lekiwi_joint_command_mux.py
    scripts/lekiwi_wheel_velocity_bridge.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
