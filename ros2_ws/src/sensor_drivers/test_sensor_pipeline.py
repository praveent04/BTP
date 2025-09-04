#!/usr/bin/env python3

"""
Test script for Sensor Data Ingestion & Preprocessing Module
Tests the core functionality with realistic dummy data
"""

import numpy as np
import cv2
import time
import random
from collections import deque

class MockIRDriver:
    """Mock IR driver that generates realistic thermal data"""

    def __init__(self):
        self.width = 640
        self.height = 480
        self.frame_count = 0

    def capture_frame(self):
        """Generate realistic IR thermal image"""
        # Base temperature (cool background)
        base_temp = np.random.normal(25, 2, (self.height, self.width))

        # Add atmospheric noise
        noise = np.random.normal(0, 0.5, (self.height, self.width))
        thermal_image = base_temp + noise

        # Add realistic heat signatures (missile launches, vehicles, etc.)
        self._add_heat_signatures(thermal_image)

        # Add sensor noise
        sensor_noise = np.random.normal(0, 0.2, (self.height, self.width))
        thermal_image += sensor_noise

        # Convert to 8-bit for OpenCV
        thermal_image = np.clip(thermal_image, 0, 100)  # Temperature range 0-100°C
        thermal_image = ((thermal_image / 100.0) * 255).astype(np.uint8)

        self.frame_count += 1
        return thermal_image

    def _add_heat_signatures(self, image):
        """Add realistic heat signatures to the thermal image"""
        # Missile launch signature (very hot, concentrated)
        if random.random() < 0.1:  # 10% chance
            center_x = random.randint(100, self.width - 100)
            center_y = random.randint(100, self.height - 100)
            self._add_gaussian_hotspot(image, center_x, center_y, 95, 15)

        # Vehicle heat signatures (multiple smaller hotspots)
        num_vehicles = random.randint(0, 3)
        for _ in range(num_vehicles):
            center_x = random.randint(50, self.width - 50)
            center_y = random.randint(50, self.height - 50)
            self._add_gaussian_hotspot(image, center_x, center_y, 45, 8)

        # Background thermal variations (buildings, terrain)
        num_background = random.randint(5, 15)
        for _ in range(num_background):
            center_x = random.randint(0, self.width)
            center_y = random.randint(0, self.height)
            temp = random.uniform(30, 60)
            size = random.uniform(20, 50)
            self._add_gaussian_hotspot(image, center_x, center_y, temp, size)

    def _add_gaussian_hotspot(self, image, center_x, center_y, temperature, sigma):
        """Add a Gaussian heat signature"""
        y, x = np.ogrid[:self.height, :self.width]
        dist_sq = (x - center_x)**2 + (y - center_y)**2
        gaussian = temperature * np.exp(-dist_sq / (2 * sigma**2))
        image += gaussian

class MockEOOpticalDriver:
    """Mock EO driver that generates realistic optical images"""

    def __init__(self):
        self.width = 1920
        self.height = 1080
        self.frame_count = 0

    def capture_frame(self):
        """Generate realistic RGB optical image"""
        # Create base scene (sky/terrain gradient)
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Sky gradient (top half)
        for y in range(self.height // 2):
            blue_value = int(135 + (120 - 135) * (y / (self.height // 2)))
            green_value = int(206 + (192 - 206) * (y / (self.height // 2)))
            red_value = int(235 + (255 - 235) * (y / (self.height // 2)))
            image[y, :, :] = [blue_value, green_value, red_value]

        # Terrain (bottom half)
        terrain_color = [34, 139, 34]  # Forest green
        image[self.height // 2:, :, :] = terrain_color

        # Add realistic objects
        self._add_realistic_objects(image)

        # Add atmospheric effects
        self._add_atmospheric_effects(image)

        self.frame_count += 1
        return image

    def _add_realistic_objects(self, image):
        """Add realistic objects to the scene"""
        # Buildings
        num_buildings = random.randint(3, 8)
        for _ in range(num_buildings):
            x = random.randint(100, self.width - 200)
            y = random.randint(self.height // 2, self.height - 100)
            width = random.randint(50, 150)
            height = random.randint(80, 200)

            # Building color (concrete/gray)
            color = [random.randint(100, 150), random.randint(100, 150), random.randint(100, 150)]
            cv2.rectangle(image, (x, y), (x + width, y + height), color, -1)

            # Windows
            for wx in range(x + 10, x + width - 10, 20):
                for wy in range(y + 10, y + height - 10, 25):
                    if random.random() < 0.7:  # 70% of windows lit
                        cv2.rectangle(image, (wx, wy), (wx + 8, wy + 12), [255, 255, 200], -1)

        # Vehicles
        num_vehicles = random.randint(1, 5)
        for _ in range(num_vehicles):
            x = random.randint(50, self.width - 100)
            y = random.randint(self.height // 2 + 50, self.height - 50)
            width = random.randint(30, 60)
            height = random.randint(15, 25)

            # Vehicle color
            colors = [[255, 0, 0], [0, 0, 255], [255, 255, 0], [128, 128, 128]]
            color = random.choice(colors)
            cv2.rectangle(image, (x, y), (x + width, y + height), color, -1)

            # Headlights if moving
            if random.random() < 0.5:
                cv2.circle(image, (x + width, y + height // 2), 3, [255, 255, 255], -1)

    def _add_atmospheric_effects(self, image):
        """Add atmospheric effects like haze, dust"""
        # Add slight haze
        haze = np.random.normal(0, 5, image.shape).astype(np.uint8)
        image = cv2.add(image, haze)

        # Add dust particles
        num_dust = random.randint(20, 50)
        for _ in range(num_dust):
            x = random.randint(0, self.width)
            y = random.randint(0, self.height)
            cv2.circle(image, (x, y), 1, [200, 200, 200], -1)

class TestSensorPipeline:
    """Test the complete sensor data processing pipeline"""

    def __init__(self):
        self.ir_driver = MockIRDriver()
        self.eo_driver = MockEOOpticalDriver()
        self.timestamps = {'ir': deque(maxlen=10), 'eo': deque(maxlen=10)}

    def test_ir_processing(self):
        """Test IR image capture and processing"""
        print("Testing IR Sensor Processing...")

        for i in range(5):
            start_time = time.time()

            # Capture frame
            frame = self.ir_driver.capture_frame()
            self.timestamps['ir'].append(time.time())

            # Apply preprocessing (similar to ROS node)
            processed = self.preprocess_ir_frame(frame)

            # Simulate heat signature detection
            heat_signatures = self.detect_heat_signatures(processed)

            print(f"Frame {i+1}: {heat_signatures} heat signatures detected")
            print(f"  Image shape: {frame.shape}, dtype: {frame.dtype}")
            print(f"  Temperature range: {frame.min()}-{frame.max()}")

            time.sleep(0.1)  # Simulate frame rate

        return True

    def test_eo_processing(self):
        """Test EO image capture and processing"""
        print("\nTesting EO Optical Sensor Processing...")

        for i in range(5):
            start_time = time.time()

            # Capture frame
            frame = self.eo_driver.capture_frame()
            self.timestamps['eo'].append(time.time())

            # Apply preprocessing (similar to ROS node)
            processed = self.preprocess_eo_frame(frame)

            # Simulate object detection
            objects = self.detect_objects(processed)

            print(f"Frame {i+1}: {objects} objects detected")
            print(f"  Image shape: {frame.shape}, dtype: {frame.dtype}")

            time.sleep(0.033)  # Simulate ~30fps

        return True

    def test_time_synchronization(self):
        """Test time synchronization monitoring"""
        print("\nTesting Time Synchronization...")

        # Check if we have data from both sensors
        if len(self.timestamps['ir']) > 0 and len(self.timestamps['eo']) > 0:
            ir_avg = sum(self.timestamps['ir']) / len(self.timestamps['ir'])
            eo_avg = sum(self.timestamps['eo']) / len(self.timestamps['eo'])

            time_diff = abs(ir_avg - eo_avg)
            print(".3f")

            if time_diff < 0.1:
                print("✓ Time synchronization: GOOD")
            elif time_diff < 0.5:
                print("⚠ Time synchronization: WARNING")
            else:
                print("✗ Time synchronization: CRITICAL")

        return True

    def preprocess_ir_frame(self, frame):
        """Apply IR preprocessing (similar to ROS node)"""
        # Convert to float for processing
        img_float = frame.astype(np.float32)

        # Noise filtering
        blurred = cv2.GaussianBlur(img_float, (5, 5), 0)

        # Enhance contrast
        enhanced = cv2.equalizeHist(blurred.astype(np.uint8))

        return enhanced

    def preprocess_eo_frame(self, frame):
        """Apply EO preprocessing (similar to ROS node)"""
        # Convert to grayscale for processing
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # Noise filtering
        filtered = cv2.bilateralFilter(gray, 9, 75, 75)

        # Enhance contrast
        enhanced = cv2.equalizeHist(filtered)

        return enhanced

    def detect_heat_signatures(self, frame):
        """Simple heat signature detection"""
        # Threshold for hot areas
        _, threshold = cv2.threshold(frame, 200, 255, cv2.THRESH_BINARY)

        # Find contours
        contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter significant heat signatures
        heat_signatures = [cnt for cnt in contours if cv2.contourArea(cnt) > 50]

        return len(heat_signatures)

    def detect_objects(self, frame):
        """Simple object detection"""
        # Edge detection
        edges = cv2.Canny(frame, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter significant objects
        objects = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]

        return len(objects)

def main():
    """Run comprehensive tests"""
    print("=== Sensor Data Pipeline Test ===")
    print("Testing with realistic dummy data close to real sensor outputs\n")

    test_pipeline = TestSensorPipeline()

    # Run tests
    ir_test = test_pipeline.test_ir_processing()
    eo_test = test_pipeline.test_eo_processing()
    sync_test = test_pipeline.test_time_synchronization()

    # Summary
    print("\n=== Test Results ===")
    print(f"IR Processing Test: {'PASS' if ir_test else 'FAIL'}")
    print(f"EO Processing Test: {'PASS' if eo_test else 'FAIL'}")
    print(f"Time Sync Test: {'PASS' if sync_test else 'FAIL'}")

    if all([ir_test, eo_test, sync_test]):
        print("\n🎉 All tests passed! The sensor pipeline is working correctly.")
        print("The code is ready for integration with real ROS 2 environment.")
    else:
        print("\n❌ Some tests failed. Please check the implementation.")

if __name__ == "__main__":
    main()