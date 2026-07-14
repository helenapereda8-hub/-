cmake_minimum_required(VERSION 3.5)
project(robot_driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 99)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(origincar_msg REQUIRED)
find_package(serial REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robot_localization REQUIRED)

include_directories(include)

add_executable(robot_driver_node
  src/robot_core.cpp
  src/attitude_solver.cpp
)

ament_target_dependencies(robot_driver_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  serial
  ackermann_msgs
  origincar_msg
  robot_localization
)

install(TARGETS robot_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()