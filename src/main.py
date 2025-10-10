"""
Main System Integration: Threat Detection, Tracking, and Visualization
Production version for Raspberry Pi deployment
"""

import cv2
import time
import numpy as np
from datetime import datetime

from camera_manager import CameraManager
from ir_processor import IRProcessor
from optical_processor import OpticalProcessor
from fusion_engine import FusionEngine
from ukf_tracker import UKFTracker
from coordinate_transformer import CoordinateTransformer
from gis_plotter import GISPlotter
from web_server import WebServer


class ThreatDetectionSystem:
    """
    Main system that integrates all components for threat detection and tracking.
    """

    def __init__(self, config=None):
        """
        Initializes the threat detection system.

        Args:
            config (dict): Configuration dictionary with system parameters
        """
        # Default configuration
        self.config = config or {
            'ir_camera_index': 1,
            'optical_camera_index': 0,
            'ir_threshold': 220,
            'min_hotspot_area': 100,
            'confidence_threshold': 0.95,
            'base_lat': 28.6139,
            'base_lon': 77.2090,
            'altitude': 100.0,
            'fps': 30,
            'web_server_port': 5000
        }

        # Initialize components
        print("Initializing Threat Detection System...")
        
        # Camera management
        self.camera_manager = CameraManager(
            ir_camera_index=self.config['ir_camera_index'],
            optical_camera_index=self.config['optical_camera_index']
        )

        # Image processors
        self.ir_processor = IRProcessor(
            threshold_value=self.config['ir_threshold'],
            min_area=self.config['min_hotspot_area']
        )
        self.optical_processor = OpticalProcessor()

        # Fusion engine
        self.fusion_engine = FusionEngine()

        # Tracker
        dt = 1.0 / self.config['fps']
        self.tracker = UKFTracker(dt=dt)

        # Coordinate transformer
        self.coord_transformer = CoordinateTransformer(
            base_lat=self.config['base_lat'],
            base_lon=self.config['base_lon'],
            altitude=self.config['altitude']
        )

        # GIS plotter
        self.gis_plotter = GISPlotter(
            center_lat=self.config['base_lat'],
            center_lon=self.config['base_lon']
        )

        # Web server
        self.web_server = WebServer(
            map_file='threat_map.html',
            port=self.config['web_server_port']
        )

        # State variables
        self.tracking_active = False
        self.launch_detected = False
        self.frame_count = 0
        self.detection_time = None

    def initialize(self):
        """
        Initializes all system components.

        Returns:
            bool: True if initialization successful
        """
        print("Initializing cameras...")
        if not self.camera_manager.initialize_cameras():
            print("ERROR: Failed to initialize cameras!")
            return False

        print("Starting web server...")
        self.web_server.start(threaded=True)

        print("System initialization complete!")
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

        # If launch detected, start/continue tracking
        if is_launch or self.tracking_active:
            if not self.tracking_active:
                # First detection
                self.tracking_active = True
                self.launch_detected = True
                self.detection_time = datetime.now()
                print(f"\n{'='*60}")
                print(f"🚨 LAUNCH DETECTED! Confidence: {confidence:.2%}")
                print(f"   Time: {self.detection_time}")
                print(f"   Position: {hotspot_coords}")
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

                # Convert to geographic coordinates
                lat, lon = self.coord_transformer.pixel_to_geographic(x, y)
                results['geographic_coords'] = (lat, lon)

                # Update map
                self.gis_plotter.add_trajectory_point(lat, lon)
                self.gis_plotter.update_current_position(lat, lon, velocity=(vx, vy))

        return results

    def run(self, display=True, save_interval=10):
        """
        Main system loop.

        Args:
            display (bool): Whether to display video feeds
            save_interval (int): Number of frames between map saves
        """
        if not self.initialize():
            return

        print("\n" + "="*60)
        print("THREAT DETECTION SYSTEM ACTIVE")
        print("="*60)
        print(f"Web interface: http://localhost:{self.config['web_server_port']}")
        print("Press 'q' to quit")
        print("="*60 + "\n")

        try:
            while True:
                # Get frames from cameras
                ir_frame, optical_frame = self.camera_manager.get_frames()

                if ir_frame is None or optical_frame is None:
                    print("ERROR: Failed to capture frames")
                    break

                # Process the frames
                results = self.process_frame(ir_frame, optical_frame)

                # Display (optional)
                if display:
                    # Annotate frames
                    display_ir = ir_frame.copy()
                    display_optical = optical_frame.copy()

                    if results['ir_hotspot']:
                        x, y = results['ir_hotspot']
                        cv2.circle(display_ir, (int(x), int(y)), 10, (0, 0, 255), 2)
                        cv2.circle(display_optical, (int(x), int(y)), 10, (0, 255, 0), 2)

                    if results['tracked_position']:
                        x, y = results['tracked_position']
                        cv2.circle(display_ir, (int(x), int(y)), 15, (255, 0, 0), 3)
                        cv2.circle(display_optical, (int(x), int(y)), 15, (255, 0, 0), 3)

                    # Add status text
                    status_text = f"Launch: {results['launch_probability']:.2%}"
                    cv2.putText(display_ir, status_text, (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    if self.tracking_active:
                        cv2.putText(display_optical, "TRACKING ACTIVE", (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                    # Show frames
                    cv2.imshow('IR Camera', display_ir)
                    cv2.imshow('Optical Camera', display_optical)

                # Save map periodically (without verbose output)
                if self.tracking_active and self.frame_count % save_interval == 0:
                    self.gis_plotter.save_map('threat_map.html', verbose=False)

                self.frame_count += 1

                # Check for quit
                if display and cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\nShutdown requested...")
                    break

                # Small delay to control frame rate
                time.sleep(1.0 / self.config['fps'])

        except KeyboardInterrupt:
            print("\n\nShutdown requested by user...")

        finally:
            self.shutdown()

    def shutdown(self):
        """
        Cleanly shuts down all system components.
        """
        print("\nShutting down system...")
        
        # Save final map
        if self.tracking_active:
            self.gis_plotter.save_map('threat_map_final.html')
            print("Final trajectory map saved to: threat_map_final.html")

        # Release cameras
        self.camera_manager.release_cameras()
        
        # Close windows
        cv2.destroyAllWindows()

        print("System shutdown complete.")


def main():
    """
    Main entry point for the application.
    """
    # Configuration for Raspberry Pi deployment
    config = {
        'ir_camera_index': 1,          # IR camera device index
        'optical_camera_index': 0,      # Optical camera device index
        'ir_threshold': 220,             # IR hotspot threshold
        'min_hotspot_area': 100,         # Minimum hotspot area in pixels
        'confidence_threshold': 0.95,    # Launch detection confidence threshold
        'base_lat': 28.6139,            # Base location latitude (New Delhi)
        'base_lon': 77.2090,            # Base location longitude
        'altitude': 100.0,               # Camera altitude in meters
        'fps': 30,                       # Target frame rate
        'web_server_port': 5000          # Web server port
    }

    # Create and run the system
    # Save map every 30 frames for production
    system = ThreatDetectionSystem(config=config)
    system.run(display=True, save_interval=30)


if __name__ == '__main__':
    main()
