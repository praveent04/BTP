#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Illuminance
from threat_detection_interfaces.msg import RadarScan
import time
from collections import deque

class TimeSyncMonitor(Node):
    def __init__(self):
        super().__init__('time_sync_monitor')

        # Subscribers for all sensor topics
        self.ir_sub = self.create_subscription(
            Image,
            'sensors/ir_image',
            self.ir_callback,
            10
        )

        self.eo_sub = self.create_subscription(
            Image,
            'sensors/eo_image',
            self.eo_callback,
            10
        )

        self.radar_sub = self.create_subscription(
            RadarScan,
            'sensors/radar_scan',
            self.radar_callback,
            10
        )

        self.uv_sub = self.create_subscription(
            Illuminance,
            'sensors/uv_intensity',
            self.uv_callback,
            10
        )

        # Store recent timestamps (last 10 messages per sensor)
        self.timestamps = {
            'ir': deque(maxlen=10),
            'eo': deque(maxlen=10),
            'radar': deque(maxlen=10),
            'uv': deque(maxlen=10)
        }

        # Timer to check synchronization every second
        self.timer = self.create_timer(1.0, self.check_sync)

        self.get_logger().info('Time Synchronization Monitor started')

    def ir_callback(self, msg):
        self.timestamps['ir'].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def eo_callback(self, msg):
        self.timestamps['eo'].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def radar_callback(self, msg):
        self.timestamps['radar'].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def uv_callback(self, msg):
        self.timestamps['uv'].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def check_sync(self):
        """Check time synchronization between all sensor streams"""
        current_time = time.time()

        # Check if we have data from all sensors
        active_sensors = [sensor for sensor, stamps in self.timestamps.items() if len(stamps) > 0]

        if len(active_sensors) < 2:
            self.get_logger().info('Waiting for data from multiple sensors...')
            return

        # Calculate average timestamps for each sensor
        avg_timestamps = {}
        for sensor, stamps in self.timestamps.items():
            if len(stamps) > 0:
                avg_timestamps[sensor] = sum(stamps) / len(stamps)

        # Find the reference sensor (the one with the most recent average timestamp)
        reference_sensor = max(avg_timestamps, key=avg_timestamps.get)
        reference_time = avg_timestamps[reference_sensor]

        # Check time differences
        max_diff = 0.0
        sync_status = "GOOD"

        for sensor, timestamp in avg_timestamps.items():
            if sensor != reference_sensor:
                diff = abs(timestamp - reference_time)
                max_diff = max(max_diff, diff)

                if diff > 0.1:  # 100ms threshold
                    sync_status = "WARNING"
                if diff > 0.5:  # 500ms threshold
                    sync_status = "CRITICAL"

        # Log synchronization status
        self.get_logger().info(
            f'Time Sync Status: {sync_status} | Max diff: {max_diff:.3f}s | '
            f'Active sensors: {len(active_sensors)} | Reference: {reference_sensor}'
        )

        # Detailed logging if there are issues
        if sync_status != "GOOD":
            for sensor, timestamp in avg_timestamps.items():
                diff = abs(timestamp - reference_time)
                self.get_logger().warn(
                    f'{sensor.upper()}: avg_time={timestamp:.3f}, diff={diff:.3f}s'
                )

def main(args=None):
    rclpy.init(args=args)
    node = TimeSyncMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()