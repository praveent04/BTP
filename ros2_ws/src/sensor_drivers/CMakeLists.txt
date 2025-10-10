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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# Hardware Interface Library
add_library(hardware_interface_lib
  src/hardware_interface.cpp
)
ament_target_dependencies(hardware_interface_lib rclcpp sensor_msgs cv_bridge image_transport OpenCV)

# ML Detector Library
add_library(ml_detector_lib
  src/ml_detector.cpp
)
ament_target_dependencies(ml_detector_lib rclcpp sensor_msgs cv_bridge image_transport OpenCV)

# IR Camera Node
add_executable(ir_camera_node src/ir_camera_node.cpp src/ir_driver.cpp)
ament_target_dependencies(ir_camera_node rclcpp sensor_msgs cv_bridge image_transport OpenCV)
target_link_libraries(ir_camera_node hardware_interface_lib)

# EO Camera Node
add_executable(eo_camera_node src/eo_camera_node.cpp src/eo_driver.cpp)
ament_target_dependencies(eo_camera_node rclcpp sensor_msgs cv_bridge image_transport OpenCV)
target_link_libraries(eo_camera_node hardware_interface_lib)

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

# Install Python scripts
install(PROGRAMS
  src/preprocessing_node.py
  src/time_sync_monitor.py
  src/enhanced_preprocessing_node.py
  src/system_monitor.py
  src/threat_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install test scripts
install(PROGRAMS
  test_sensor_pipeline.py
  validate_code.py
  test_enhanced_system.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
