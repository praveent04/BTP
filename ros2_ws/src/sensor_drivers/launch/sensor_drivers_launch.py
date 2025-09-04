from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_drivers',
            executable='ir_camera_node',
            name='ir_camera_node',
            output='screen',
            parameters=[
                {'use_simulation': True},  # Set to False for real hardware
                {'device_path': '/dev/ir_sensor'},
                {'frame_rate': 10.0},
                {'width': 640},
                {'height': 480}
            ]
        ),
        Node(
            package='sensor_drivers',
            executable='eo_camera_node',
            name='eo_camera_node',
            output='screen',
            parameters=[
                {'use_simulation': True},  # Set to False for real hardware
                {'device_path': '/dev/eo_camera'},
                {'frame_rate': 30.0},
                {'width': 1920},
                {'height': 1080},
                {'exposure': 10.0},
                {'gain': 1.0}
            ]
        ),
        Node(
            package='sensor_drivers',
            executable='radar_node',
            name='radar_node',
            output='screen'
        ),
        Node(
            package='sensor_drivers',
            executable='uv_sensor_node',
            name='uv_sensor_node',
            output='screen'
        ),
        Node(
            package='sensor_drivers',
            executable='preprocessing_node.py',
            name='preprocessing_node',
            output='screen'
        ),
        Node(
            package='sensor_drivers',
            executable='time_sync_monitor.py',
            name='time_sync_monitor',
            output='screen'
        ),
    ])
