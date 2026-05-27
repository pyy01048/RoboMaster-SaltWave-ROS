cmake_minimum_required(VERSION 3.8)
project(vision_aiming)

find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
