cmake_minimum_required(VERSION 3.8)
project(allbot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
