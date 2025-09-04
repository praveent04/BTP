cmake_minimum_required(VERSION 3.8)
project(sensor_drivers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(threat_detection_interfaces REQUIRED)

# IR Camera Node
add_executable(ir_camera_node src/ir_camera_node.cpp)
ament_target_dependencies(ir_camera_node rclcpp sensor_msgs)

# EO Camera Node
add_executable(eo_camera_node src/eo_camera_node.cpp)
ament_target_dependencies(eo_camera_node rclcpp sensor_msgs)

# Radar Node
add_executable(radar_node src/radar_node.cpp)
ament_target_dependencies(radar_node rclcpp threat_detection_interfaces)

# UV Sensor Node
add_executable(uv_sensor_node src/uv_sensor_node.cpp)
ament_target_dependencies(uv_sensor_node rclcpp sensor_msgs)

install(TARGETS
  ir_camera_node
  eo_camera_node
  radar_node
  uv_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
