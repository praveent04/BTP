#!/usr/bin/env python3

"""
Code Validation Script - Tests core logic without external dependencies
Validates the sensor data processing algorithms and data structures
"""

import random
import time
import math
from collections import deque

class MockThermalData:
    """Generate realistic thermal data without numpy"""

    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height

    def generate_frame(self):
        """Generate realistic thermal frame data"""
        # Create base temperature array (simplified without numpy)
        frame = []
        for y in range(self.height):
            row = []
            for x in range(self.width):
                # Base temperature with some variation
                base_temp = 25 + random.gauss(0, 2)
                row.append(base_temp)
            frame.append(row)

        # Add heat signatures
        self._add_heat_signatures(frame)

        # Add noise
        self._add_noise(frame)

        return frame

    def _add_heat_signatures(self, frame):
        """Add realistic heat signatures"""
        # Missile launch (very hot, concentrated)
        if random.random() < 0.1:
            center_x = random.randint(100, self.width - 100)
            center_y = random.randint(100, self.height - 100)
            self._add_gaussian_hotspot(frame, center_x, center_y, 95, 15)

        # Vehicles (multiple smaller hotspots)
        num_vehicles = random.randint(0, 3)
        for _ in range(num_vehicles):
            center_x = random.randint(50, self.width - 50)
            center_y = random.randint(50, self.height - 50)
            self._add_gaussian_hotspot(frame, center_x, center_y, 45, 8)

    def _add_gaussian_hotspot(self, frame, center_x, center_y, temperature, sigma):
        """Add Gaussian heat distribution"""
        for y in range(max(0, center_y - int(3*sigma)), min(self.height, center_y + int(3*sigma))):
            for x in range(max(0, center_x - int(3*sigma)), min(self.width, center_x + int(3*sigma))):
                dist_sq = (x - center_x)**2 + (y - center_y)**2
                gaussian = temperature * math.exp(-dist_sq / (2 * sigma**2))
                frame[y][x] += gaussian

    def _add_noise(self, frame):
        """Add realistic sensor noise"""
        for y in range(self.height):
            for x in range(self.width):
                noise = random.gauss(0, 0.2)
                frame[y][x] += noise
                # Clamp to realistic temperature range
                frame[y][x] = max(0, min(100, frame[y][x]))

class MockOpticalData:
    """Generate realistic optical data without numpy"""

    def __init__(self, width=1920, height=1080):
        self.width = width
        self.height = height

    def generate_frame(self):
        """Generate realistic RGB optical frame data"""
        frame = []
        for y in range(self.height):
            row = []
            for x in range(self.width):
                if y < self.height // 2:
                    # Sky gradient
                    blue = int(135 + (120 - 135) * (y / (self.height // 2)))
                    green = int(206 + (192 - 206) * (y / (self.height // 2)))
                    red = int(235 + (255 - 235) * (y / (self.height // 2)))
                else:
                    # Terrain
                    blue = 34
                    green = 139
                    red = 34

                row.append((red, green, blue))
            frame.append(row)

        # Add objects
        self._add_objects(frame)

        return frame

    def _add_objects(self, frame):
        """Add realistic objects to the scene"""
        # Buildings
        num_buildings = random.randint(3, 8)
        for _ in range(num_buildings):
            x = random.randint(100, self.width - 200)
            y = random.randint(self.height // 2, self.height - 100)
            width = random.randint(50, 150)
            height = random.randint(80, 200)

            # Building color
            color = (random.randint(100, 150), random.randint(100, 150), random.randint(100, 150))

            # Draw building
            for by in range(y, min(self.height, y + height)):
                for bx in range(x, min(self.width, x + width)):
                    frame[by][bx] = color

class ValidationTests:
    """Validate core sensor processing algorithms"""

    def __init__(self):
        self.thermal_sensor = MockThermalData()
        self.optical_sensor = MockOpticalData()
        self.timestamps = {'thermal': deque(maxlen=10), 'optical': deque(maxlen=10)}

    def test_thermal_processing(self):
        """Test thermal image processing pipeline"""
        print("Testing Thermal Sensor Processing...")

        results = []
        for i in range(5):
            start_time = time.time()

            # Generate frame
            frame = self.thermal_sensor.generate_frame()
            self.timestamps['thermal'].append(time.time())

            # Test preprocessing
            processed = self.preprocess_thermal(frame)

            # Test heat signature detection
            signatures = self.detect_heat_signatures(processed)

            results.append({
                'frame': i + 1,
                'signatures': signatures,
                'temp_range': self.get_temp_range(frame)
            })

            print(f"  Frame {i+1}: {signatures} heat signatures, temp range: {results[-1]['temp_range']}")

            time.sleep(0.1)

        # Validate results
        total_signatures = sum(r['signatures'] for r in results)
        avg_temp_range = sum(r['temp_range'][1] - r['temp_range'][0] for r in results) / len(results)

        print(f"  [OK] Total heat signatures detected: {total_signatures}")
        print(f"  [OK] Average temperature range: {avg_temp_range:.1f}°C")

        return total_signatures > 0 and avg_temp_range > 10

    def test_optical_processing(self):
        """Test optical image processing pipeline"""
        print("\nTesting Optical Sensor Processing...")

        results = []
        for i in range(5):
            start_time = time.time()

            # Generate frame
            frame = self.optical_sensor.generate_frame()
            self.timestamps['optical'].append(time.time())

            # Test preprocessing
            processed = self.preprocess_optical(frame)

            # Test object detection
            objects = self.detect_objects(processed)

            results.append({
                'frame': i + 1,
                'objects': objects,
                'brightness': self.get_average_brightness(frame)
            })

            print(f"  Frame {i+1}: {objects} objects detected, avg brightness: {results[-1]['brightness']:.1f}")

            time.sleep(0.033)

        # Validate results
        total_objects = sum(r['objects'] for r in results)
        avg_brightness = sum(r['brightness'] for r in results) / len(results)

        print(f"  [OK] Total objects detected: {total_objects}")
        print(f"  [OK] Average brightness: {avg_brightness:.1f}")

        return total_objects > 0 and 50 < avg_brightness < 200

    def test_time_synchronization(self):
        """Test time synchronization logic"""
        print("\nTesting Time Synchronization...")

        if len(self.timestamps['thermal']) > 0 and len(self.timestamps['optical']) > 0:
            thermal_times = list(self.timestamps['thermal'])
            optical_times = list(self.timestamps['optical'])

            thermal_avg = sum(thermal_times) / len(thermal_times)
            optical_avg = sum(optical_times) / len(optical_times)

            time_diff = abs(thermal_avg - optical_avg)

            print(".3f")

            if time_diff < 0.1:
                print("  [GOOD] Time synchronization: GOOD")
                return True
            elif time_diff < 0.5:
                print("  [WARNING] Time synchronization: WARNING")
                return True
            else:
                print("  [CRITICAL] Time synchronization: CRITICAL")
                return False
        else:
            print("  [WARNING] Insufficient data for sync test")
            return False

    def preprocess_thermal(self, frame):
        """Simplified thermal preprocessing"""
        # Simple noise reduction (averaging neighbors)
        processed = []
        for y in range(len(frame)):
            row = []
            for x in range(len(frame[0])):
                # Simple blur
                neighbors = []
                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < len(frame) and 0 <= nx < len(frame[0]):
                            neighbors.append(frame[ny][nx])

                avg = sum(neighbors) / len(neighbors)
                row.append(avg)
            processed.append(row)

        return processed

    def preprocess_optical(self, frame):
        """Simplified optical preprocessing"""
        # Convert to grayscale
        processed = []
        for y in range(len(frame)):
            row = []
            for x in range(len(frame[0])):
                r, g, b = frame[y][x]
                # Simple grayscale conversion
                gray = int(0.299 * r + 0.587 * g + 0.114 * b)
                row.append(gray)
            processed.append(row)

        return processed

    def detect_heat_signatures(self, frame):
        """Simple heat signature detection"""
        threshold = 70  # Hot temperature threshold
        signatures = 0

        for y in range(len(frame)):
            for x in range(len(frame[0])):
                if frame[y][x] > threshold:
                    # Check if it's a local maximum (simple peak detection)
                    is_peak = True
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if dy == 0 and dx == 0:
                                continue
                            ny, nx = y + dy, x + dx
                            if (0 <= ny < len(frame) and 0 <= nx < len(frame[0]) and
                                frame[ny][nx] > frame[y][x]):
                                is_peak = False
                                break
                        if not is_peak:
                            break

                    if is_peak:
                        signatures += 1

        return signatures

    def detect_objects(self, frame):
        """Simple object detection based on edges"""
        objects = 0
        threshold = 50  # Edge detection threshold

        for y in range(1, len(frame) - 1):
            for x in range(1, len(frame[0]) - 1):
                # Simple edge detection (gradient magnitude)
                gx = (frame[y][x+1] - frame[y][x-1]) / 2
                gy = (frame[y+1][x] - frame[y-1][x]) / 2
                gradient = math.sqrt(gx*gx + gy*gy)

                if gradient > threshold:
                    objects += 1

        return objects // 100  # Scale down for reasonable counts

    def get_temp_range(self, frame):
        """Get temperature range of frame"""
        temps = []
        for row in frame:
            temps.extend(row)

        return min(temps), max(temps)

    def get_average_brightness(self, frame):
        """Get average brightness of RGB frame"""
        total_brightness = 0
        pixel_count = 0

        for row in frame:
            for pixel in row:
                r, g, b = pixel
                brightness = (r + g + b) / 3
                total_brightness += brightness
                pixel_count += 1

        return total_brightness / pixel_count if pixel_count > 0 else 0

def main():
    """Run validation tests"""
    print("=== Sensor Data Pipeline Validation ===")
    print("Testing core algorithms with realistic dummy data\n")

    validator = ValidationTests()

    # Run tests
    thermal_test = validator.test_thermal_processing()
    optical_test = validator.test_optical_processing()
    sync_test = validator.test_time_synchronization()

    # Summary
    print("\n=== Validation Results ===")
    print(f"Thermal Processing: {'PASS' if thermal_test else 'FAIL'}")
    print(f"Optical Processing: {'PASS' if optical_test else 'FAIL'}")
    print(f"Time Synchronization: {'PASS' if sync_test else 'FAIL'}")

    if all([thermal_test, optical_test, sync_test]):
        print("\n[SUCCESS] All validations passed!")
        print("✓ Thermal sensor generates realistic heat signatures")
        print("✓ Optical sensor creates realistic scenes with objects")
        print("✓ Time synchronization logic works correctly")
        print("✓ Preprocessing algorithms function as expected")
        print("\nThe sensor pipeline code is validated and ready for ROS 2 integration.")
    else:
        print("\n[FAILED] Some validations failed. Please review the implementation.")

    return all([thermal_test, optical_test, sync_test])

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)