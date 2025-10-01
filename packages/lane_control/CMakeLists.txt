cmake_minimum_required(VERSION 3.10)
project(lane_control)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch resource
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    ${PROJECT_NAME}/lane_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

