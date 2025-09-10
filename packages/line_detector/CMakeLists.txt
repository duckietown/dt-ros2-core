cmake_minimum_required(VERSION 3.5)
project(line_detector)

find_package(ament_cmake REQUIRED)

# Install the Python node script
install(PROGRAMS
  src/line_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

ament_package()
