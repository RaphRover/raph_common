cmake_minimum_required(VERSION 3.5)
project(leo2_teleop)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/leo2_teleop
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
