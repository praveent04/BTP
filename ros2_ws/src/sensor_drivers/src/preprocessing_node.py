#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PreprocessingNode(Node):
    def __init__(self):
        super().__init__('preprocessing_node')

        self.bridge = CvBridge()

        # Subscribers
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

        # Publishers
        self.ir_processed_pub = self.create_publisher(Image, 'sensors/ir_processed', 10)
        self.eo_processed_pub = self.create_publisher(Image, 'sensors/eo_processed', 10)

        self.get_logger().info('Preprocessing Node started')

    def ir_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            # Apply advanced preprocessing
            processed = self.process_ir_image(cv_image)

            # Convert back to ROS message
            processed_msg = self.bridge.cv2_to_imgmsg(processed, encoding='mono8')
            processed_msg.header = msg.header

            self.ir_processed_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing IR image: {str(e)}')

    def eo_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Apply advanced preprocessing
            processed = self.process_eo_image(cv_image)

            # Convert back to ROS message
            processed_msg = self.bridge.cv2_to_imgmsg(processed, encoding='rgb8')
            processed_msg.header = msg.header

            self.eo_processed_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing EO image: {str(e)}')

    def process_ir_image(self, image):
        """Advanced IR image processing for heat signature detection"""
        # Convert to float for processing
        img_float = image.astype(np.float32)

        # Apply Gaussian blur for noise reduction
        blurred = cv2.GaussianBlur(img_float, (5, 5), 0)

        # Enhance contrast using CLAHE
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        enhanced = clahe.apply((blurred * 255 / blurred.max()).astype(np.uint8))

        # Thermal analysis - detect hot spots
        threshold = cv2.threshold(enhanced, 200, 255, cv2.THRESH_BINARY)[1]

        # Morphological operations to clean up hot spots
        kernel = np.ones((3, 3), np.uint8)
        cleaned = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

        # Find contours of heat signatures
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw bounding boxes around detected heat signatures
        result = cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)
        for contour in contours:
            if cv2.contourArea(contour) > 50:  # Filter small detections
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)

        return cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

    def process_eo_image(self, image):
        """Advanced EO image processing for object detection"""
        # Convert to grayscale for processing
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        # Apply bilateral filter for noise reduction while preserving edges
        filtered = cv2.bilateralFilter(gray, 9, 75, 75)

        # Enhance contrast
        enhanced = cv2.equalizeHist(filtered)

        # Edge detection using Canny
        edges = cv2.Canny(enhanced, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours on original image
        result = image.copy()
        cv2.drawContours(result, contours, -1, (0, 255, 0), 2)

        # Simple motion detection placeholder (would need previous frame in real implementation)
        # For now, just apply sharpening
        kernel = np.array([[-1, -1, -1],
                          [-1,  9, -1],
                          [-1, -1, -1]])
        sharpened = cv2.filter2D(result, -1, kernel)

        return sharpened

def main(args=None):
    rclpy.init(args=args)
    node = PreprocessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()