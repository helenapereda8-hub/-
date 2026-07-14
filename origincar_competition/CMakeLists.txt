cmake_minimum_required(VERSION 3.5)
project(robot_decision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 99)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(origincar_msg REQUIRED)
find_package(OpenCV REQUIRED)    # 虽未直接用，但保留以防万一
find_package(cv_bridge REQUIRED) # 同
find_package(hbm_img_msgs REQUIRED)

add_definitions(-DBUILD_HBMEM_MSG)

link_directories(/usr/hobot/lib)

add_executable(robot_decision_node
  src/decision_node.cpp
)

target_link_libraries(robot_decision_node
  hbmem
  ion
  alog
)

ament_target_dependencies(robot_decision_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ai_msgs
  origincar_msg
  hbm_img_msgs
)

install(TARGETS robot_decision_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()