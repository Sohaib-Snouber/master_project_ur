cmake_minimum_required(VERSION 3.5)
project(amrl_husky_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch stl urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
