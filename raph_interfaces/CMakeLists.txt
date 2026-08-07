cmake_minimum_required(VERSION 3.5)
project(raph_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryMode.msg"
  "msg/BatteryState.msg"
  "msg/DrivetrainState.msg"
  "msg/ImuDiagnostics.msg"
  "msg/KeyValue.msg"
  "msg/Led.msg"
  "msg/LedColor.msg"
  "msg/LedPanelState.msg"
  "msg/LedState.msg"
  "msg/LedStripState.msg"
  "msg/MotorDiagnostics.msg"
  "msg/PowerSystemState.msg"
  "msg/ServoCommand.msg"
  "msg/SteeringMode.msg"
  "msg/TurnInPlaceDrive.msg"
  "msg/WheelCommand.msg"
  "srv/ApplyTfFramePrefix.srv"
  "srv/GetControllerInfo.srv"
  "srv/GetOsVersion.srv"
  "srv/SetBatteryModes.srv"
  "srv/SetLeds.srv"
  "srv/SetSteeringMode.srv"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()