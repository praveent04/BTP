#!/usr/bin/env python3

"""
System Health Monitoring and Alert System
Real-time monitoring of sensor system health and performance
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
import psutil
import time
import threading
from collections import deque
from typing import Dict, List, Optional
import json
import os

class SystemMetrics:
    """System performance metrics collector"""

    def __init__(self):
        self.cpu_percent = 0.0
        self.memory_percent = 0.0
        self.memory_used_mb = 0.0
        self.disk_usage_percent = 0.0
        self.network_bytes_sent = 0
        self.network_bytes_recv = 0
        self.timestamp = time.time()

    def update(self):
        """Update all system metrics"""
        self.cpu_percent = psutil.cpu_percent(interval=1)
        memory = psutil.virtual_memory()
        self.memory_percent = memory.percent
        self.memory_used_mb = memory.used / (1024 * 1024)

        disk = psutil.disk_usage('/')
        self.disk_usage_percent = disk.percent

        network = psutil.net_io_counters()
        self.network_bytes_sent = network.bytes_sent
        self.network_bytes_recv = network.bytes_recv

        self.timestamp = time.time()

    def to_dict(self) -> Dict:
        """Convert metrics to dictionary"""
        return {
            'cpu_percent': self.cpu_percent,
            'memory_percent': self.memory_percent,
            'memory_used_mb': self.memory_used_mb,
            'disk_usage_percent': self.disk_usage_percent,
            'network_bytes_sent': self.network_bytes_sent,
            'network_bytes_recv': self.network_bytes_recv,
            'timestamp': self.timestamp
        }

class AlertSystem:
    """Intelligent alert system for system monitoring"""

    def __init__(self):
        self.alerts = deque(maxlen=100)
        self.active_alerts = set()

        # Alert thresholds
        self.thresholds = {
            'cpu_percent': {'warning': 70.0, 'critical': 90.0},
            'memory_percent': {'warning': 75.0, 'critical': 90.0},
            'disk_usage_percent': {'warning': 80.0, 'critical': 95.0},
            'processing_time': {'warning': 50.0, 'critical': 100.0},  # ms
            'frame_rate': {'warning': 10.0, 'critical': 5.0},  # fps
        }

    def check_thresholds(self, metrics: Dict) -> List[Dict]:
        """Check metrics against thresholds and generate alerts"""
        new_alerts = []

        for metric_name, value in metrics.items():
            if metric_name in self.thresholds:
                thresholds = self.thresholds[metric_name]

                alert_level = None
                if value >= thresholds.get('critical', float('inf')):
                    alert_level = 'CRITICAL'
                elif value >= thresholds.get('warning', float('inf')):
                    alert_level = 'WARNING'

                if alert_level:
                    alert = {
                        'timestamp': time.time(),
                        'metric': metric_name,
                        'value': value,
                        'level': alert_level,
                        'threshold': thresholds.get(alert_level.lower(), 0),
                        'message': f"{metric_name} is {alert_level}: {value}"
                    }

                    # Only add if not already active
                    alert_key = f"{metric_name}_{alert_level}"
                    if alert_key not in self.active_alerts:
                        self.active_alerts.add(alert_key)
                        new_alerts.append(alert)
                        self.alerts.append(alert)

        return new_alerts

    def clear_resolved_alerts(self, metrics: Dict):
        """Clear alerts that are no longer active"""
        resolved_alerts = []

        for alert_key in list(self.active_alerts):
            metric_name, level = alert_key.split('_', 1)
            if metric_name in metrics:
                value = metrics[metric_name]
                thresholds = self.thresholds.get(metric_name, {})

                threshold_value = thresholds.get(level.lower(), float('inf'))
                if value < threshold_value:
                    resolved_alerts.append(alert_key)

        for alert_key in resolved_alerts:
            self.active_alerts.remove(alert_key)

    def get_active_alerts(self) -> List[Dict]:
        """Get currently active alerts"""
        return [alert for alert in self.alerts if alert['timestamp'] > time.time() - 300]  # Last 5 minutes

class SensorHealthMonitor:
    """Monitor health of individual sensors"""

    def __init__(self):
        self.sensor_status = {
            'ir_camera': {'status': 'UNKNOWN', 'last_seen': 0, 'frame_count': 0},
            'eo_camera': {'status': 'UNKNOWN', 'last_seen': 0, 'frame_count': 0},
            'radar': {'status': 'UNKNOWN', 'last_seen': 0, 'frame_count': 0},
            'uv_sensor': {'status': 'UNKNOWN', 'last_seen': 0, 'frame_count': 0}
        }

    def update_sensor_status(self, sensor_name: str, status: str = 'ACTIVE'):
        """Update sensor status"""
        if sensor_name in self.sensor_status:
            self.sensor_status[sensor_name]['status'] = status
            self.sensor_status[sensor_name]['last_seen'] = time.time()
            if status == 'ACTIVE':
                self.sensor_status[sensor_name]['frame_count'] += 1

    def check_sensor_health(self) -> List[Dict]:
        """Check health of all sensors"""
        alerts = []
        current_time = time.time()

        for sensor_name, status in self.sensor_status.items():
            time_since_last_seen = current_time - status['last_seen']

            if time_since_last_seen > 30:  # 30 seconds timeout
                if status['status'] != 'OFFLINE':
                    alerts.append({
                        'timestamp': current_time,
                        'sensor': sensor_name,
                        'level': 'WARNING',
                        'message': f"Sensor {sensor_name} is offline (last seen {time_since_last_seen:.1f}s ago)"
                    })
                    status['status'] = 'OFFLINE'
            elif time_since_last_seen > 10:  # 10 seconds delay warning
                if status['status'] != 'DELAYED':
                    alerts.append({
                        'timestamp': current_time,
                        'sensor': sensor_name,
                        'level': 'INFO',
                        'message': f"Sensor {sensor_name} delayed (last seen {time_since_last_seen:.1f}s ago)"
                    })
                    status['status'] = 'DELAYED'

        return alerts

    def get_sensor_status(self) -> Dict:
        """Get status of all sensors"""
        return self.sensor_status.copy()

class SystemMonitorNode(Node):
    """ROS2 node for comprehensive system monitoring"""

    def __init__(self):
        super().__init__('system_monitor')

        # Initialize monitoring components
        self.system_metrics = SystemMetrics()
        self.alert_system = AlertSystem()
        self.sensor_monitor = SensorHealthMonitor()

        # Publishers
        self.alerts_pub = self.create_publisher(String, 'system/alerts', 10)
        self.metrics_pub = self.create_publisher(String, 'system/metrics', 10)
        self.health_pub = self.create_publisher(String, 'system/health', 10)

        # Subscribers for sensor monitoring
        self.ir_sub = self.create_subscription(Image, 'sensors/ir_image',
                                             lambda msg: self.sensor_callback('ir_camera', msg), 10)
        self.eo_sub = self.create_subscription(Image, 'sensors/eo_image',
                                             lambda msg: self.sensor_callback('eo_camera', msg), 10)

        # Timers
        self.monitoring_timer = self.create_timer(5.0, self.monitoring_callback)
        self.health_check_timer = self.create_timer(10.0, self.health_check_callback)

        # Configuration
        self.declare_parameter('enable_alerts', True)
        self.declare_parameter('alert_log_file', 'logs/system_alerts.log')
        self.declare_parameter('metrics_log_file', 'logs/system_metrics.log')

        self.enable_alerts = self.get_parameter('enable_alerts').value
        self.alert_log_file = self.get_parameter('alert_log_file').value
        self.metrics_log_file = self.get_parameter('metrics_log_file').value

        # Create log directories
        os.makedirs(os.path.dirname(self.alert_log_file), exist_ok=True)
        os.makedirs(os.path.dirname(self.metrics_log_file), exist_ok=True)

        self.get_logger().info('System Monitor Node initialized')

    def sensor_callback(self, sensor_name: str, msg):
        """Callback for sensor data reception"""
        self.sensor_monitor.update_sensor_status(sensor_name, 'ACTIVE')

    def monitoring_callback(self):
        """Main monitoring callback"""
        # Update system metrics
        self.system_metrics.update()

        # Get metrics as dictionary
        metrics = self.system_metrics.to_dict()

        # Add sensor-specific metrics
        sensor_status = self.sensor_monitor.get_sensor_status()
        metrics['active_sensors'] = sum(1 for s in sensor_status.values() if s['status'] == 'ACTIVE')
        metrics['total_sensor_frames'] = sum(s['frame_count'] for s in sensor_status.values())

        # Check for alerts
        if self.enable_alerts:
            alerts = self.alert_system.check_thresholds(metrics)
            self.alert_system.clear_resolved_alerts(metrics)

            # Publish alerts
            for alert in alerts:
                alert_msg = String()
                alert_msg.data = json.dumps(alert)
                self.alerts_pub.publish(alert_msg)

                # Log alert
                self.log_alert(alert)

        # Publish metrics
        metrics_msg = String()
        metrics_msg.data = json.dumps(metrics)
        self.metrics_pub.publish(metrics_msg)

        # Log metrics
        self.log_metrics(metrics)

    def health_check_callback(self):
        """Health check callback"""
        # Check sensor health
        sensor_alerts = self.sensor_monitor.check_sensor_health()

        # Publish sensor alerts
        for alert in sensor_alerts:
            alert_msg = String()
            alert_msg.data = json.dumps(alert)
            self.alerts_pub.publish(alert_msg)

        # Publish overall health status
        health_status = {
            'timestamp': time.time(),
            'system_status': 'HEALTHY',
            'sensor_status': self.sensor_monitor.get_sensor_status(),
            'active_alerts': len(self.alert_system.get_active_alerts()),
            'uptime': time.time() - self.get_clock().now().seconds
        }

        # Determine overall system status
        active_alerts = self.alert_system.get_active_alerts()
        critical_alerts = [a for a in active_alerts if a['level'] == 'CRITICAL']

        if critical_alerts:
            health_status['system_status'] = 'CRITICAL'
        elif active_alerts:
            health_status['system_status'] = 'WARNING'

        health_msg = String()
        health_msg.data = json.dumps(health_status)
        self.health_pub.publish(health_msg)

    def log_alert(self, alert: Dict):
        """Log alert to file"""
        try:
            with open(self.alert_log_file, 'a') as f:
                log_entry = f"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(alert['timestamp']))} "
                log_entry += f"[{alert['level']}] {alert['message']}\n"
                f.write(log_entry)
        except Exception as e:
            self.get_logger().error(f"Failed to log alert: {e}")

    def log_metrics(self, metrics: Dict):
        """Log metrics to file"""
        try:
            with open(self.metrics_log_file, 'a') as f:
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(metrics['timestamp']))
                log_entry = f"{timestamp} CPU:{metrics['cpu_percent']:.1f}% "
                log_entry += f"MEM:{metrics['memory_percent']:.1f}% "
                log_entry += f"DISK:{metrics['disk_usage_percent']:.1f}% "
                log_entry += f"SENSORS:{metrics['active_sensors']}\n"
                f.write(log_entry)
        except Exception as e:
            self.get_logger().error(f"Failed to log metrics: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()