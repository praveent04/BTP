# Sensor Data Ingestion & Preprocessing Module

This ROS 2 package implements a comprehensive sensor data ingestion and preprocessing system for threat detection applications, featuring IR and EO (Electro-Optical) sensors with advanced image processing capabilities.

## Architecture

### Components

1. **C++ Sensor Drivers**
   - `IRDriver`: Handles IR camera communication with heat signature detection
   - `EOOpticalDriver`: Manages EO camera with optical image processing

2. **ROS 2 Nodes**
   - `ir_camera_node`: Publishes IR images with preprocessing
   - `eo_camera_node`: Publishes EO images with preprocessing
   - `radar_node`: Publishes radar scan data
   - `uv_sensor_node`: Publishes UV intensity data

3. **Python Processing Nodes**
   - `preprocessing_node.py`: Advanced image processing with NumPy/OpenCV
   - `time_sync_monitor.py`: Monitors time synchronization across sensors

## Features

### IR Sensor Processing
- Heat signature detection and enhancement
- Noise filtering with Gaussian blur
- Contrast enhancement with CLAHE
- Hot spot detection and bounding boxes

### EO Sensor Processing
- Color correction and white balance
- Edge detection and contour analysis
- Lens distortion correction (configurable)
- Object detection and tracking

### Time Synchronization
- Automatic ROS 2 timestamp synchronization
- Real-time monitoring of sensor timing
- Warning system for synchronization issues

## Configuration

### Parameters

**IR Camera Node:**
- `use_simulation`: Enable/disable simulation mode (default: true)
- `device_path`: Hardware device path (default: "/dev/ir_sensor")
- `frame_rate`: Capture frame rate in Hz (default: 10.0)
- `width/height`: Image resolution (default: 640x480)

**EO Camera Node:**
- `use_simulation`: Enable/disable simulation mode (default: true)
- `device_path`: Hardware device path (default: "/dev/eo_camera")
- `frame_rate`: Capture frame rate in Hz (default: 30.0)
- `width/height`: Image resolution (default: 1920x1080)
- `exposure`: Camera exposure time in ms (default: 10.0)
- `gain`: Camera gain setting (default: 1.0)

## Topics

### Published Topics
- `sensors/ir_image`: Raw IR camera images
- `sensors/eo_image`: Raw EO camera images
- `sensors/radar_scan`: Radar scan data
- `sensors/uv_intensity`: UV sensor intensity
- `sensors/ir_processed`: Processed IR images with heat signatures
- `sensors/eo_processed`: Processed EO images with object detection

## Building and Running

### Prerequisites
- ROS 2 (Humble or later)
- OpenCV 4.x with Python bindings
- cv_bridge and image_transport packages

### Build
```bash
cd ros2_ws
colcon build --packages-select sensor_drivers
```

### Run
```bash
# Source the workspace
source install/setup.bash

# Launch all sensor nodes
ros2 launch sensor_drivers sensor_drivers_launch.py
```

### Individual Node Testing
```bash
# Test IR camera node
ros2 run sensor_drivers ir_camera_node --ros-args -p use_simulation:=true

# Test preprocessing node
ros2 run sensor_drivers preprocessing_node.py
```

## Hardware Integration

### Real Hardware Setup
1. Set `use_simulation:=false` in launch parameters
2. Update `device_path` to match your hardware device
3. Configure camera parameters (resolution, frame rate, etc.)
4. Ensure proper permissions for device access

### Driver Implementation
The driver classes (`IRDriver`, `EOOpticalDriver`) provide interfaces for:
- Hardware initialization and connection
- Frame capture and data processing
- Parameter configuration (resolution, exposure, gain)
- Error handling and fallback to simulation

## File Structure

```
sensor_drivers/
├── CMakeLists.txt
├── package.xml
├── include/sensor_drivers/
│   ├── ir_driver.hpp
│   └── eo_driver.hpp
├── src/
│   ├── ir_driver.cpp
│   ├── eo_driver.cpp
│   ├── ir_camera_node.cpp
│   ├── eo_camera.cpp
│   ├── radar_node.cpp
│   ├── uv_sensor_node.cpp
│   ├── preprocessing_node.py
│   └── time_sync_monitor.py
├── launch/
│   └── sensor_drivers_launch.py
└── README.md
```

## Dependencies

- `rclcpp`: ROS 2 C++ client library
- `sensor_msgs`: Standard sensor message types
- `cv_bridge`: OpenCV-ROS image conversion
- `image_transport`: Compressed image transport
- `OpenCV`: Computer vision library
- `rclpy`: ROS 2 Python client library
- `numpy`: Numerical computing
- `threat_detection_interfaces`: Custom message types

## Performance Considerations

- Frame rates are configurable per sensor type
- Image processing is optimized for real-time operation
- Memory usage scales with image resolution
- Time synchronization monitoring runs at 1 Hz

## Troubleshooting

### Common Issues
1. **OpenCV not found**: Ensure OpenCV is installed and properly configured
2. **Device access denied**: Check permissions for hardware devices
3. **Time sync warnings**: Verify all sensors are publishing at expected rates
4. **Memory usage**: Reduce image resolution if processing is too slow

### Debug Mode
Enable verbose logging:
```bash
ros2 launch sensor_drivers sensor_drivers_launch.py --ros-args --log-level debug
```

## Future Enhancements

- GPU acceleration for image processing
- Multi-camera calibration and stereo vision
- Machine learning-based object detection
- Advanced thermal analysis algorithms
- Real-time video streaming compression