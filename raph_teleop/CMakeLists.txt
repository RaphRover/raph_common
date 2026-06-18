cmake_minimum_required(VERSION 3.5)
project(raph_teleop)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_module(raph_teleop_params
  src/raph_teleop_params.yaml
)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/raph_teleop
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Set local config that's compatible with black formatter
  set(ament_cmake_flake8_CONFIG_FILE "ament_flake8.ini")

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
