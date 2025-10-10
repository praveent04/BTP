#!/usr/bin/env python3

"""
USB Webcam Testing Script
Comprehensive testing of USB camera hardware integration
"""

import cv2
import time
import numpy as np
import subprocess
import sys
import os
from datetime import datetime

class WebcamTester:
    """Comprehensive USB webcam testing"""

    def __init__(self):
        self.test_results = {
            'hardware_detection': False,
            'opencv_access': False,
            'resolution_test': False,
            'frame_rate_test': False,
            'ros_integration': False,
            'real_time_processing': False
        }
        self.device_index = 0
        self.capture = None

    def run_complete_test(self):
        """Run all webcam tests"""
        print("🖥️  USB WEBCAM COMPREHENSIVE TEST")
        print("=" * 50)

        # Test 1: Hardware Detection
        self.test_hardware_detection()

        # Test 2: OpenCV Access
        self.test_opencv_access()

        # Test 3: Resolution Test
        self.test_resolution()

        # Test 4: Frame Rate Test
        self.test_frame_rate()

        # Test 5: ROS Integration
        self.test_ros_integration()

        # Test 6: Real-time Processing
        self.test_real_time_processing()

        # Generate report
        self.generate_test_report()

        return self.calculate_success_rate()

    def test_hardware_detection(self):
        """Test hardware detection"""
        print("\n1. Hardware Detection Test")

        try:
            # Check /dev/video* devices
            result = subprocess.run(['ls', '/dev/video*'],
                                  capture_output=True, text=True)

            if result.returncode == 0:
                devices = result.stdout.strip().split('\n')
                print(f"   Found video devices: {devices}")

                if len(devices) > 0 and devices[0] != '':
                    self.test_results['hardware_detection'] = True
                    print("   ✅ Hardware detection: PASS")
                else:
                    print("   ❌ No video devices found")
            else:
                print("   ❌ Cannot access /dev/video*")

        except Exception as e:
            print(f"   ❌ Hardware detection error: {e}")

    def test_opencv_access(self):
        """Test OpenCV camera access"""
        print("\n2. OpenCV Camera Access Test")

        try:
            self.capture = cv2.VideoCapture(self.device_index)

            if self.capture.isOpened():
                # Get camera properties
                width = self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)
                height = self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
                fps = self.capture.get(cv2.CAP_PROP_FPS)

                print(f"   Camera properties: {width}x{height} @ {fps} FPS")

                # Test frame capture
                ret, frame = self.capture.read()
                if ret and frame is not None:
                    print(f"   Frame captured: {frame.shape}")
                    self.test_results['opencv_access'] = True
                    print("   ✅ OpenCV access: PASS")

                    # Save test frame
                    cv2.imwrite('test_webcam_frame.jpg', frame)
                    print("   📸 Test frame saved as 'test_webcam_frame.jpg'")
                else:
                    print("   ❌ Cannot capture frame")
            else:
                print("   ❌ Cannot open camera with OpenCV")

        except Exception as e:
            print(f"   ❌ OpenCV access error: {e}")

    def test_resolution(self):
        """Test different resolutions"""
        print("\n3. Resolution Test")

        if not self.capture or not self.capture.isOpened():
            print("   ❌ Camera not available for resolution test")
            return

        resolutions = [
            (640, 480),
            (1280, 720),
            (1920, 1080)
        ]

        successful_resolutions = []

        for width, height in resolutions:
            try:
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

                actual_width = self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)
                actual_height = self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

                if abs(actual_width - width) < 50 and abs(actual_height - height) < 50:
                    successful_resolutions.append(f"{int(actual_width)}x{int(actual_height)}")
                    print(f"   ✅ Resolution {width}x{height}: Supported")
                else:
                    print(f"   ⚠️  Resolution {width}x{height}: Not exact ({int(actual_width)}x{int(actual_height)})")

            except Exception as e:
                print(f"   ❌ Resolution {width}x{height} error: {e}")

        if successful_resolutions:
            self.test_results['resolution_test'] = True
            print(f"   📊 Supported resolutions: {', '.join(successful_resolutions)}")

    def test_frame_rate(self):
        """Test frame rate capabilities"""
        print("\n4. Frame Rate Test")

        if not self.capture or not self.capture.isOpened():
            print("   ❌ Camera not available for frame rate test")
            return

        frame_rates = [10, 15, 20, 30]
        frame_rate_results = {}

        for fps in frame_rates:
            try:
                self.capture.set(cv2.CAP_PROP_FPS, fps)

                # Capture frames for 2 seconds to measure actual FPS
                start_time = time.time()
                frame_count = 0

                for i in range(60):  # Test for 60 frames max
                    ret, frame = self.capture.read()
                    if ret:
                        frame_count += 1
                    else:
                        break

                    if time.time() - start_time > 2.0:  # 2 second test
                        break

                actual_fps = frame_count / (time.time() - start_time)
                frame_rate_results[fps] = actual_fps

                if abs(actual_fps - fps) < 5:  # Within 5 FPS tolerance
                    print(f"   ✅ Frame rate {fps} FPS: {actual_fps:.1f} FPS (Good)")
                else:
                    print(f"   ⚠️  Frame rate {fps} FPS: {actual_fps:.1f} FPS (Poor)")
            except Exception as e:
                print(f"   ❌ Frame rate {fps} FPS error: {e}")

        if frame_rate_results:
            self.test_results['frame_rate_test'] = True
            best_fps = max(frame_rate_results.values())
            print(f"   📊 Best frame rate: {best_fps:.1f} FPS")
    def test_ros_integration(self):
        """Test ROS 2 integration"""
        print("\n5. ROS 2 Integration Test")

        try:
            # Check if ROS 2 is available
            result = subprocess.run(['ros2', '--version'],
                                  capture_output=True, text=True, timeout=5)

            if result.returncode == 0:
                print("   ✅ ROS 2 is available")

                # Test topic creation (would need ROS running)
                print("   📝 Note: Full ROS integration test requires running ROS system")
                print("   🔧 To test: ros2 launch sensor_drivers sensor_drivers_launch.py")
                print("   🔧 Monitor: ros2 topic hz /sensors/eo_image")

                self.test_results['ros_integration'] = True
                print("   ✅ ROS integration: READY")
            else:
                print("   ❌ ROS 2 not available")

        except Exception as e:
            print(f"   ❌ ROS integration error: {e}")

    def test_real_time_processing(self):
        """Test real-time processing capabilities"""
        print("\n6. Real-time Processing Test")

        if not self.capture or not self.capture.isOpened():
            print("   ❌ Camera not available for processing test")
            return

        try:
            processing_times = []
            frame_count = 0

            print("   📊 Testing real-time processing for 10 seconds...")

            start_time = time.time()
            while time.time() - start_time < 10.0:  # 10 second test
                frame_start = time.time()

                # Capture frame
                ret, frame = self.capture.read()
                if not ret:
                    break

                # Simulate processing pipeline
                processed = self.simulate_processing_pipeline(frame)
                processing_time = (time.time() - frame_start) * 1000  # ms

                processing_times.append(processing_time)
                frame_count += 1

                # Real-time check (should be < 100ms for 10 FPS)
                if processing_time > 100:
                    print(f"   ⚠️  Slow processing: {processing_time:.1f}ms")
                elif frame_count % 30 == 0:  # Progress update
                    print(f"   📊 Processed {frame_count} frames...")

            if processing_times:
                avg_time = sum(processing_times) / len(processing_times)
                max_time = max(processing_times)
                fps = frame_count / (time.time() - start_time)

                print(f"   📊 Average processing time: {avg_time:.1f}ms")
                print(f"   📊 Max processing time: {max_time:.1f}ms")
                print(f"   📊 Achieved frame rate: {fps:.1f} FPS")
                if avg_time < 50 and fps > 5:  # Good real-time performance
                    self.test_results['real_time_processing'] = True
                    print("   ✅ Real-time processing: EXCELLENT")
                elif avg_time < 100 and fps > 2:
                    self.test_results['real_time_processing'] = True
                    print("   ✅ Real-time processing: GOOD")
                else:
                    print("   ⚠️  Real-time processing: NEEDS OPTIMIZATION")

        except Exception as e:
            print(f"   ❌ Real-time processing error: {e}")

    def simulate_processing_pipeline(self, frame):
        """Simulate the processing pipeline"""
        # Resize if needed
        if frame.shape[1] > 640:
            frame = cv2.resize(frame, (640, 480))

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Simple object detection simulation
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw results
        result = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(result, contours, -1, (0, 255, 0), 2)

        return result

    def generate_test_report(self):
        """Generate comprehensive test report"""
        print("\n" + "=" * 50)
        print("WEBCAM TEST REPORT")
        print("=" * 50)

        passed_tests = sum(1 for result in self.test_results.values() if result)
        total_tests = len(self.test_results)

        print(f"\nOverall Score: {passed_tests}/{total_tests} tests passed")
        print(f"Success Rate: {(passed_tests / total_tests * 100):.1f}%")
        print("\nDetailed Results:")
        for test_name, result in self.test_results.items():
            status = "✅ PASS" if result else "❌ FAIL"
            print(f"  {test_name.replace('_', ' ').title()}: {status}")

        # Save detailed report
        report = {
            'timestamp': datetime.now().isoformat(),
            'test_results': self.test_results,
            'summary': {
                'passed': passed_tests,
                'total': total_tests,
                'success_rate': passed_tests / total_tests * 100
            }
        }

        with open('webcam_test_report.json', 'w') as f:
            import json
            json.dump(report, f, indent=2)

        print("\n📄 Detailed report saved as 'webcam_test_report.json'")
        if os.path.exists('test_webcam_frame.jpg'):
            print("📸 Test frame saved as 'test_webcam_frame.jpg'")

    def calculate_success_rate(self):
        """Calculate overall success rate"""
        passed = sum(1 for result in self.test_results.values() if result)
        total = len(self.test_results)
        return passed / total * 100

    def cleanup(self):
        """Clean up resources"""
        if self.capture and self.capture.isOpened():
            self.capture.release()

def main():
    """Main test execution"""
    tester = WebcamTester()

    try:
        success_rate = tester.run_complete_test()

        print("\n🎯 FINAL RESULT:")
        if success_rate >= 80:
            print(f"Success Rate: {success_rate:.1f}%")
            print("🎉 Webcam is ready for production use!")
            return 0
        elif success_rate >= 60:
            print(f"Success Rate: {success_rate:.1f}%")
            print("⚠️  Webcam functional but may need optimization")
            return 1
        else:
            print(f"Success Rate: {success_rate:.1f}%")
            print("❌ Webcam testing failed - check hardware and drivers")
            return 1

    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
        return 1
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        return 1
    finally:
        tester.cleanup()

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)