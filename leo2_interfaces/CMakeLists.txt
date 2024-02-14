cmake_minimum_required(VERSION 3.5)
project(leo2_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryMode.msg"
  "msg/BatteryState.msg"
  "msg/LedArray.msg"
  "msg/LedColor.msg"
  "msg/LedState.msg"
  "msg/PowerSystemState.msg"
  "msg/SteeringMode.msg"
  "srv/SetBatteryModes.srv"
  "srv/SetLedState.srv"
  "srv/SetSteeringMode.srv"
  DEPENDENCIES std_msgs
)

ament_package()