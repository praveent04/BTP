"""
Test Main System: Demo version for Windows presentation
Uses simulated camera feeds to demonstrate full system functionality
"""

import cv2
import time
import numpy as np
from datetime import datetime

# Import test camera manager instead of real one
from test_camera_manager import TestCameraManager

# Import all other production modules (they work with any image data)
from ir_processor import IRProcessor
from optical_processor import OpticalProcessor
from fusion_engine import FusionEngine
from ukf_tracker import UKFTracker
from coordinate_transformer import CoordinateTransformer
from gis_plotter import GISPlotter
from web_server import WebServer


class TestThreatDetectionSystem:
    """
    Test/Demo version of the threat detection system for Windows presentation.
    Uses simulated camera feeds instead of real hardware.
    """

    def __init__(self, config=None):
        """
        Initializes the test threat detection system.

        Args:
            config (dict): Configuration dictionary with system parameters
        """
        # Default configuration
        self.config = config or {
            'frame_width': 640,
            'frame_height': 480,
            'ir_threshold': 200,  # Lower threshold for test (simulated IR is less bright)
            'min_hotspot_area': 50,  # Smaller area for test
            'confidence_threshold': 0.90,  # Lower threshold for demo
            'base_lat': 28.6139,
            'base_lon': 77.2090,
            'altitude': 100.0,
            'fps': 30,
            'web_server_port': 5000
        }

        # Initialize components
        print("\n" + "="*60)
        print("Initializing TEST/DEMO Threat Detection System...")
        print("="*60)
        
        # Test camera management (simulated)
        self.camera_manager = TestCameraManager(
            width=self.config['frame_width'],
            height=self.config['frame_height'],
            fps=self.config['fps']
        )

        # Image processors (same as production)
        self.ir_processor = IRProcessor(
            threshold_value=self.config['ir_threshold'],
            min_area=self.config['min_hotspot_area']
        )
        self.optical_processor = OpticalProcessor()

        # Fusion engine (same as production)
        self.fusion_engine = FusionEngine()

        # Tracker (same as production)
        dt = 1.0 / self.config['fps']
        self.tracker = UKFTracker(dt=dt)

        # Coordinate transformer (same as production)
        self.coord_transformer = CoordinateTransformer(
            base_lat=self.config['base_lat'],
            base_lon=self.config['base_lon'],
            altitude=self.config['altitude'],
            image_width=self.config['frame_width'],
            image_height=self.config['frame_height']
        )

        # GIS plotter (same as production)
        self.gis_plotter = GISPlotter(
            center_lat=self.config['base_lat'],
            center_lon=self.config['base_lon']
        )

        # Web server (same as production)
        self.web_server = WebServer(
            map_file='test_threat_map.html',
            port=self.config['web_server_port']
        )

        # State variables
        self.tracking_active = False
        self.launch_detected = False
        self.frame_count = 0
        self.detection_time = None
        
        # Statistics for demo
        self.stats = {
            'total_frames': 0,
            'detections': 0,
            'tracking_frames': 0,
            'max_confidence': 0.0
        }

    def initialize(self):
        """
        Initializes all system components.

        Returns:
            bool: True if initialization successful
        """
        print("\nInitializing simulated cameras...")
        if not self.camera_manager.initialize_cameras():
            print("ERROR: Failed to initialize cameras!")
            return False

        print("Starting web server...")
        self.web_server.start(threaded=True)

        print("\n" + "="*60)
        print("✅ System initialization complete!")
        print("="*60)
        print("\nDEMO MODE: Using simulated missile launch")
        print(f"Web interface: http://localhost:{self.config['web_server_port']}")
        print("="*60 + "\n")
        
        return True

    def process_frame(self, ir_frame, optical_frame):
        """
        Processes a single frame pair through the detection and tracking pipeline.

        Args:
            ir_frame: IR camera frame
            optical_frame: Optical camera frame

        Returns:
            dict: Processing results
        """
        results = {
            'ir_hotspot': None,
            'visual_confirmation': False,
            'launch_probability': 0.0,
            'launch_detected': False,
            'tracked_position': None,
            'geographic_coords': None
        }

        # Stage 1: IR Hotspot Detection
        hotspot_coords = self.ir_processor.find_hotspot(ir_frame)
        results['ir_hotspot'] = hotspot_coords

        if hotspot_coords is None:
            return results

        self.stats['detections'] += 1

        # Stage 2: Visual Confirmation
        visual_confirmed = self.optical_processor.detect_motion(optical_frame, hotspot_coords)
        results['visual_confirmation'] = visual_confirmed

        # Stage 3: Bayesian Fusion
        is_launch, confidence = self.fusion_engine.is_launch_detected(
            ir_detected=True,
            visual_confirmed=visual_confirmed,
            threshold=self.config['confidence_threshold']
        )
        results['launch_probability'] = confidence
        results['launch_detected'] = is_launch
        
        # Update stats
        self.stats['max_confidence'] = max(self.stats['max_confidence'], confidence)

        # If launch detected, start/continue tracking
        if is_launch or self.tracking_active:
            if not self.tracking_active:
                # First detection
                self.tracking_active = True
                self.launch_detected = True
                self.detection_time = datetime.now()
                print(f"\n{'='*60}")
                print(f"🚨 LAUNCH DETECTED! (DEMO)")
                print(f"   Confidence: {confidence:.2%}")
                print(f"   Time: {self.detection_time}")
                print(f"   Position: {hotspot_coords}")
                print(f"   Frame: {self.frame_count}")
                print(f"{'='*60}\n")

                # Add launch point to map
                lat, lon = self.coord_transformer.pixel_to_geographic(*hotspot_coords)
                self.gis_plotter.add_launch_point(
                    lat, lon,
                    timestamp=self.detection_time.strftime("%Y-%m-%d %H:%M:%S")
                )

            # Update tracker
            state = self.tracker.update(hotspot_coords)
            if state:
                x, vx, y, vy = state
                results['tracked_position'] = (x, y)
                
                self.stats['tracking_frames'] += 1

                # Convert to geographic coordinates
                lat, lon = self.coord_transformer.pixel_to_geographic(x, y)
                results['geographic_coords'] = (lat, lon)

                # Update map
                self.gis_plotter.add_trajectory_point(lat, lon)
                self.gis_plotter.update_current_position(lat, lon, velocity=(vx, vy))
                
                # Print tracking info
                if self.stats['tracking_frames'] % 10 == 0:
                    print(f"📍 Tracking... Frame {self.frame_count} | "
                          f"Pos: ({x:.1f}, {y:.1f}) | "
                          f"Vel: ({vx:.1f}, {vy:.1f}) px/s | "
                          f"GPS: ({lat:.6f}, {lon:.6f})")

        return results

    def run(self, max_frames=300, display=True, save_interval=5):
        """
        Main demo loop.

        Args:
            max_frames (int): Maximum frames to process (for demo)
            display (bool): Whether to display video feeds
            save_interval (int): Number of frames between map saves
        """
        if not self.initialize():
            return

        print("Starting demo loop...")
        print(f"Will run for {max_frames} frames\n")
        print("Press 'q' to quit early\n")

        try:
            while self.frame_count < max_frames:
                # Get simulated frames
                ir_frame, optical_frame = self.camera_manager.get_frames()

                if ir_frame is None or optical_frame is None:
                    print("ERROR: Failed to capture frames")
                    break

                # Process the frames
                results = self.process_frame(ir_frame, optical_frame)
                self.stats['total_frames'] += 1

                # Display (optional)
                if display:
                    # Annotate frames
                    display_ir = ir_frame.copy()
                    display_optical = optical_frame.copy()

                    if results['ir_hotspot']:
                        x, y = results['ir_hotspot']
                        cv2.circle(display_ir, (int(x), int(y)), 10, (0, 0, 255), 2)
                        cv2.circle(display_optical, (int(x), int(y)), 10, (0, 255, 0), 2)
                        cv2.putText(display_ir, "HOTSPOT", (int(x) + 15, int(y)),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    if results['tracked_position']:
                        x, y = results['tracked_position']
                        cv2.circle(display_ir, (int(x), int(y)), 15, (255, 0, 0), 3)
                        cv2.circle(display_optical, (int(x), int(y)), 15, (255, 0, 0), 3)
                        cv2.putText(display_optical, f"TRACKING", (int(x) + 20, int(y)),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    # Add status text
                    status_text = f"Confidence: {results['launch_probability']:.1%}"
                    cv2.putText(display_ir, status_text, (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(display_ir, "DEMO MODE", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    if self.tracking_active:
                        cv2.putText(display_optical, "🎯 TRACKING ACTIVE", (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        cv2.putText(display_optical, f"Track Frames: {self.stats['tracking_frames']}", 
                                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                    # Show frames
                    cv2.imshow('IR Camera (Simulated)', display_ir)
                    cv2.imshow('Optical Camera (Simulated)', display_optical)

                # Save map periodically (without verbose output)
                if self.tracking_active and self.frame_count % save_interval == 0:
                    self.gis_plotter.save_map('test_threat_map.html', verbose=False)

                self.frame_count += 1

                # Check for quit
                key = cv2.waitKey(30) & 0xFF
                if key == ord('q'):
                    print("\nEarly exit requested...")
                    break

        except KeyboardInterrupt:
            print("\n\nShutdown requested by user...")

        finally:
            self.shutdown()

    def shutdown(self):
        """
        Cleanly shuts down all system components and displays statistics.
        """
        print("\n" + "="*60)
        print("SHUTTING DOWN DEMO SYSTEM")
        print("="*60)
        
        # Save final map
        if self.tracking_active:
            self.gis_plotter.save_map('test_threat_map_final.html')
            print(f"\n✅ Final trajectory map saved: test_threat_map_final.html")

        # Display statistics
        print("\n" + "="*60)
        print("DEMO SESSION STATISTICS")
        print("="*60)
        print(f"Total Frames Processed:  {self.stats['total_frames']}")
        print(f"IR Hotspot Detections:   {self.stats['detections']}")
        print(f"Tracking Frames:         {self.stats['tracking_frames']}")
        print(f"Max Confidence Score:    {self.stats['max_confidence']:.2%}")
        if self.detection_time:
            print(f"Launch Detection Time:   {self.detection_time}")
        print("="*60)

        # Release cameras
        self.camera_manager.release_cameras()
        
        # Close windows
        cv2.destroyAllWindows()

        print("\n✅ Demo system shutdown complete.")
        print(f"\n📊 View the trajectory map at: http://localhost:{self.config['web_server_port']}")
        print("   Or open 'test_threat_map_final.html' in a browser\n")


def main():
    """
    Main entry point for the test/demo application.
    """
    print("\n" + "="*60)
    print("THREAT DETECTION SYSTEM - DEMO MODE")
    print("="*60)
    print("\nThis is a SIMULATION for presentation purposes.")
    print("It uses synthetic camera feeds with a simulated missile launch.")
    print("All algorithms are identical to the production system.\n")
    
    # Configuration for demo
    config = {
        'frame_width': 640,
        'frame_height': 480,
        'ir_threshold': 200,             # Adjusted for simulated IR
        'min_hotspot_area': 50,          # Smaller for demo
        'confidence_threshold': 0.90,    # Slightly lower for demo
        'base_lat': 28.6139,            # New Delhi
        'base_lon': 77.2090,
        'altitude': 100.0,
        'fps': 30,
        'web_server_port': 5000
    }

    # Create and run the test system
    # Save map every 30 frames (approximately every 1 second at 30 fps)
    system = TestThreatDetectionSystem(config=config)
    system.run(max_frames=300, display=True, save_interval=30)


if __name__ == '__main__':
    main()
