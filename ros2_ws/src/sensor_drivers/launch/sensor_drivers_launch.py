from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_drivers',
            executable='ir_camera_node',
            name='ir_camera_node',
            output='screen'
        ),
        Node(
            package='sensor_drivers',
            executable='eo_camera_node',
            name='eo_camera_node',
            output='screen'
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
    ])
