#!/usr/bin/env python3

"""
test_integration.py

Integration test for the complete threat detection pipeline.
"""

import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch.actions import ExecuteProcess
import time
import requests

class TestIntegration(unittest.TestCase):
    """
    Test suite for the integrated threat detection system.
    """

    @classmethod
    def setUpClass(cls):
        """
        Launch the ROS2 nodes before the tests.
        """
        rclpy.init()
        cls.node = rclpy.create_node('test_integration_node')
        
        # Launch the sensor drivers and the threat detector
        ld = LaunchDescription([
            LaunchNode(
                package='sensor_drivers',
                executable='ir_camera_node',
                name='ir_camera_node',
            ),
            LaunchNode(
                package='sensor_drivers',
                executable='eo_camera_node',
                name='eo_camera_node',
            ),
            LaunchNode(
                package='sensor_drivers',
                executable='threat_detector_node.py',
                name='threat_detector_node',
            ),
        ])
        
        # Use a process to launch the nodes
        cls.launch_process = ExecuteProcess(
            cmd=['ros2', 'launch', 'sensor_drivers', 'sensor_drivers_launch.py'],
            output='screen'
        )
        
        # Give the nodes time to start up
        time.sleep(10)

    @classmethod
    def tearDownClass(cls):
        """
        Shutdown the ROS2 nodes after the tests.
        """
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_map_generation(self):
        """
        Test that the web server is generating a valid map.
        """
        try:
            response = requests.get('http://localhost:8080')
            self.assertEqual(response.status_code, 200, "Web server should be running.")
            self.assertIn("folium.js", response.text, "Map should contain Folium JavaScript.")
        except requests.exceptions.ConnectionError as e:
            self.fail(f"Could not connect to the web server: {e}")

if __name__ == '__main__':
    unittest.main()