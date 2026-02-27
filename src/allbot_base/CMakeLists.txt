cmake_minimum_required(VERSION 3.8)
project(allbot_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(allbot_interface REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/allbot_base.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  allbot_interface
)

add_executable(allbot_base_node
  src/allbot_base_node.cpp
)
target_link_libraries(allbot_base_node ${PROJECT_NAME})
ament_target_dependencies(allbot_base_node
  rclcpp
  allbot_interface
)

install(TARGETS
  ${PROJECT_NAME}
  allbot_base_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
