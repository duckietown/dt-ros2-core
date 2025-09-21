cmake_minimum_required(VERSION 3.5)
project(car_interface)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
