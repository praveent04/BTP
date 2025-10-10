#!/usr/bin/env python3

"""
Enhanced Preprocessing Node with ML Integration
Advanced sensor data processing with machine learning capabilities
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import deque
import threading
import json
from typing import List, Dict, Optional

class PerformanceMonitor:
    """Real-time performance monitoring"""

    def __init__(self):
        self.metrics = {
            'processing_time': deque(maxlen=100),
            'frame_rate': deque(maxlen=100),
            'memory_usage': deque(maxlen=100),
            'cpu_usage': deque(maxlen=100),
            'detection_count': deque(maxlen=100)
        }
        self.last_update = time.time()

    def update_metric(self, metric_name: str, value: float):
        """Update a performance metric"""
        if metric_name in self.metrics:
            self.metrics[metric_name].append(value)

    def get_average(self, metric_name: str) -> float:
        """Get average value for a metric"""
        if metric_name in self.metrics and len(self.metrics[metric_name]) > 0:
            return sum(self.metrics[metric_name]) / len(self.metrics[metric_name])
        return 0.0

    def get_stats(self) -> Dict:
        """Get comprehensive performance statistics"""
        stats = {}
        for metric, values in self.metrics.items():
            if len(values) > 0:
                stats[metric] = {
                    'current': values[-1] if values else 0.0,
                    'average': sum(values) / len(values),
                    'min': min(values),
                    'max': max(values),
                    'count': len(values)
                }
        return stats

class EnhancedPreprocessingNode(Node):
    """Enhanced preprocessing node with ML and performance monitoring"""

    def __init__(self, testing=False):
        # This setup ensures the node can run for testing without ROS `super().__init__`
        if not testing:
            super().__init__('enhanced_preprocessing_node')
            self.get_logger().info('Enhanced Preprocessing Node initialized with ML capabilities')

        # Universal initializations
        self.ml_detector = MockMLDetector()
        self.thermal_detector = MockThermalDetector()
        self.use_ml = True
        self.thermal_enhancement = True
        self.object_tracking = True

        try:
            self.bridge = CvBridge()
        except NameError: # pragma: no cover
            self.bridge = None  # Mock bridge if cv_bridge is not available

        self.performance_monitor = PerformanceMonitor()
        self.frame_buffers = {'ir': deque(maxlen=10), 'eo': deque(maxlen=10)}
        
        if not testing:
            # ROS-only initializations
            self.declare_parameters()
            self.load_configuration()
            self.initialize_ml_detectors() # Re-initialize in case params disable ML

            # Subscribers/Publishers/Timers
            self.ir_sub = self.create_subscription(Image, 'sensors/ir_image', self.ir_callback, 10)
            self.eo_sub = self.create_subscription(Image, 'sensors/eo_image', self.eo_callback, 10)
            self.ir_processed_pub = self.create_publisher(Image, 'sensors/ir_processed', 10)
            self.eo_processed_pub = self.create_publisher(Image, 'sensors/eo_processed', 10)
            self.detections_pub = self.create_publisher(Image, 'sensors/detections_overlay', 10)
            self.performance_timer = self.create_timer(5.0, self.publish_performance_metrics)

    def declare_parameters(self):
        """Declare ROS2 parameters"""
        self.declare_parameter('use_ml_detection', True)
        self.declare_parameter('enable_performance_monitoring', True)
        self.declare_parameter('thermal_enhancement', True)
        self.declare_parameter('object_tracking', True)
        self.declare_parameter('alert_system_enabled', True)
        self.declare_parameter('model_config_path', 'config/models.json')

    def load_configuration(self):
        """Load configuration from parameters"""
        self.use_ml = self.get_parameter('use_ml_detection').value
        self.enable_monitoring = self.get_parameter('enable_performance_monitoring').value
        self.thermal_enhancement = self.get_parameter('thermal_enhancement').value
        self.object_tracking = self.get_parameter('object_tracking').value
        self.alert_enabled = self.get_parameter('alert_system_enabled').value
        self.model_config_path = self.get_parameter('model_config_path').value

    def initialize_ml_detectors(self):
        """Initialize ML detectors with fallback to traditional CV"""
        try:
            if self.use_ml:
                # Try to initialize ML detectors
                self.initialize_yolo_detector()
                self.initialize_thermal_detector()
                self.get_logger().info('ML detectors initialized successfully')
            else:
                self.get_logger().info('ML detection disabled, using traditional CV methods')
            
            # Ensure detectors are initialized for testing
            if not hasattr(self, 'ml_detector') or self.ml_detector is None:
                self.ml_detector = MockMLDetector()
            if not hasattr(self, 'thermal_detector') or self.thermal_detector is None:
                self.thermal_detector = MockThermalDetector()
        except Exception as e:
            self.get_logger().warning(f'Failed to initialize ML detectors: {e}')
            self.get_logger().info('Falling back to traditional computer vision methods')
            self.use_ml = False

    def initialize_yolo_detector(self):
        """Initialize YOLO object detector"""
        try:
            # This would load actual YOLO model in production
            # For demo, we'll simulate the detector
            self.ml_detector = MockMLDetector()
            self.get_logger().info('YOLO detector initialized')
        except Exception as e:
            raise RuntimeError(f"YOLO initialization failed: {e}")

    def initialize_thermal_detector(self):
        """Initialize thermal signature detector"""
        try:
            self.thermal_detector = MockThermalDetector()
            self.get_logger().info('Thermal detector initialized')
        except Exception as e:
            raise RuntimeError(f"Thermal detector initialization failed: {e}")

    def ir_callback(self, msg):
        """Process IR camera frames"""
        start_time = time.time()

        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            # Store in buffer for temporal processing
            self.frame_buffers['ir'].append((cv_image, msg.header.stamp))

            # Apply enhanced processing
            processed, detections = self.process_ir_frame(cv_image)

            # Performance monitoring
            processing_time = (time.time() - start_time) * 1000  # ms
            if self.enable_monitoring:
                self.performance_monitor.update_metric('processing_time', processing_time)
                self.performance_monitor.update_metric('detection_count', len(detections))

            # Convert back to ROS message
            processed_msg = self.bridge.cv2_to_imgmsg(processed, encoding='mono8')
            processed_msg.header = msg.header

            # Publish processed image
            self.ir_processed_pub.publish(processed_msg)

            # Publish detections overlay if available
            if detections:
                overlay = self.create_detection_overlay(cv_image, detections)
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='mono8')
                overlay_msg.header = msg.header
                self.detections_pub.publish(overlay_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing IR frame: {str(e)}')

    def eo_callback(self, msg):
        """Process EO camera frames"""
        start_time = time.time()

        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Store in buffer for temporal processing
            self.frame_buffers['eo'].append((cv_image, msg.header.stamp))

            # Apply enhanced processing
            processed, detections = self.process_eo_frame(cv_image)

            # Performance monitoring
            processing_time = (time.time() - start_time) * 1000  # ms
            if self.enable_monitoring:
                self.performance_monitor.update_metric('processing_time', processing_time)
                self.performance_monitor.update_metric('detection_count', len(detections))

            # Convert back to ROS message
            processed_msg = self.bridge.cv2_to_imgmsg(processed, encoding='rgb8')
            processed_msg.header = msg.header

            # Publish processed image
            self.eo_processed_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing EO frame: {str(e)}')

    def process_ir_frame(self, frame):
        """Enhanced IR frame processing with ML capabilities"""
        detections = []

        if self.use_ml and self.thermal_detector:
            # Use ML-based thermal detection
            detections = self.thermal_detector.detect_thermal_signatures(frame)
        else:
            # Fallback to traditional thermal processing
            detections = self.traditional_thermal_detection(frame)

        # Apply thermal enhancement
        if self.thermal_enhancement:
            frame = self.enhance_thermal_image(frame)

        return frame, detections

    def process_eo_frame(self, frame):
        """Enhanced EO frame processing with ML capabilities"""
        detections = []

        if self.use_ml and self.ml_detector:
            # Use ML-based object detection
            detections = self.ml_detector.detect_objects(frame)
        else:
            # Fallback to traditional object detection
            detections = self.traditional_object_detection(frame)

        # Apply image enhancement
        frame = self.enhance_visual_image(frame)

        return frame, detections

    def traditional_thermal_detection(self, frame):
        """Traditional computer vision thermal detection"""
        detections = []

        # Apply Gaussian blur for noise reduction
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)

        # CLAHE for contrast enhancement
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(blurred)

        # Threshold for hot areas
        _, threshold = cv2.threshold(enhanced, 200, 255, cv2.THRESH_BINARY)

        # Find contours
        contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 50:
                x, y, w, h = cv2.boundingRect(contour)
                detections.append({
                    'bbox': (x, y, w, h),
                    'confidence': 0.8,
                    'class': 'heat_source'
                })

        return detections

    def traditional_object_detection(self, frame):
        """Traditional computer vision object detection"""
        detections = []

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # Edge detection
        edges = cv2.Canny(gray, 50, 150)

        # Morphological operations
        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 100:
                x, y, w, h = cv2.boundingRect(contour)
                detections.append({
                    'bbox': (x, y, w, h),
                    'confidence': 0.7,
                    'class': 'object'
                })

        return detections

    def enhance_thermal_image(self, frame):
        """Advanced thermal image enhancement"""
        # Apply bilateral filter for noise reduction while preserving edges
        enhanced = cv2.bilateralFilter(frame, 9, 75, 75)

        # CLAHE for contrast enhancement
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(enhanced)

        # Sharpen the image
        kernel = np.array([[-1, -1, -1],
                          [-1,  9, -1],
                          [-1, -1, -1]])
        enhanced = cv2.filter2D(enhanced, -1, kernel)

        return enhanced

    def enhance_visual_image(self, frame):
        """Advanced visual image enhancement"""
        # Convert to LAB color space for better enhancement
        lab = cv2.cvtColor(frame, cv2.COLOR_RGB2LAB)

        # Split channels
        l, a, b = cv2.split(lab)

        # Apply CLAHE to L channel
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)

        # Merge channels
        lab = cv2.merge([l, a, b])

        # Convert back to RGB
        enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2RGB)

        return enhanced

    def create_detection_overlay(self, frame, detections):
        """Create detection overlay for visualization"""
        overlay = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)

        for detection in detections:
            x, y, w, h = detection['bbox']
            confidence = detection.get('confidence', 0.0)
            class_name = detection.get('class', 'unknown')

            # Color based on confidence
            if confidence > 0.8:
                color = (0, 255, 0)  # Green for high confidence
            elif confidence > 0.6:
                color = (0, 255, 255)  # Yellow for medium confidence
            else:
                color = (0, 0, 255)  # Red for low confidence

            # Draw bounding box
            cv2.rectangle(overlay, (x, y), (x + w, y + h), color, 2)

            # Draw label
            label = ".2f"
            cv2.putText(overlay, label, (x, y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return overlay

    def publish_performance_metrics(self):
        """Publish performance metrics"""
        if not self.enable_monitoring:
            return

        stats = self.performance_monitor.get_stats()

        # Log performance metrics
        self.get_logger().info("=== Performance Metrics ===")
        for metric, values in stats.items():
            self.get_logger().info(".2f")

        # Check for alerts
        if self.alert_enabled:
            self.check_alerts(stats)

    def check_alerts(self, stats):
        """Check for performance alerts"""
        alerts = []

        if 'processing_time' in stats:
            avg_time = stats['processing_time']['average']
            if avg_time > self.alert_thresholds['processing_time']:
                alerts.append(f"High processing time: {avg_time:.1f}ms")

        if 'frame_rate' in stats and stats['frame_rate']['current'] < self.alert_thresholds['frame_rate']:
            alerts.append(f"Low frame rate: {stats['frame_rate']['current']:.1f}fps")

        for alert in alerts:
            self.get_logger().warning(f"ALERT: {alert}")

# Mock ML Detectors for demonstration
class MockMLDetector:
    """Mock ML detector for demonstration purposes"""

    def detect_objects(self, frame):
        """Mock object detection"""
        detections = []

        # Simulate some detections
        height, width = frame.shape[:2]

        # Add some mock detections
        for i in range(np.random.randint(1, 4)):
            x = np.random.randint(0, width - 100)
            y = np.random.randint(0, height - 100)
            w = np.random.randint(50, 150)
            h = np.random.randint(50, 150)

            detections.append({
                'bbox': (x, y, w, h),
                'confidence': np.random.uniform(0.6, 0.95),
                'class': 'mock_object'
            })

        return detections

class MockThermalDetector:
    """Mock thermal detector for demonstration purposes"""

    def detect_thermal_signatures(self, frame):
        """Mock thermal signature detection"""
        detections = []

        # Simulate thermal signatures
        height, width = frame.shape[:2]

        # Add mock heat signatures
        for i in range(np.random.randint(1, 3)):
            x = np.random.randint(0, width - 50)
            y = np.random.randint(0, height - 50)
            w = np.random.randint(20, 80)
            h = np.random.randint(20, 80)

            detections.append({
                'bbox': (x, y, w, h),
                'confidence': np.random.uniform(0.7, 0.95),
                'class': 'heat_source'
            })

        return detections

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedPreprocessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()