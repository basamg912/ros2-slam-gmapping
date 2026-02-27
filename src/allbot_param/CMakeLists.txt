cmake_minimum_required(VERSION 3.8)
project(allbot_param)

find_package(ament_cmake REQUIRED)

install(DIRECTORY ekf imu navigation
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
