"""
Enhanced Multi-Missile Test System
Supports:
- Multiple simultaneous missile tracking
- Custom tracking duration
- Full detection → tracking pipeline
- Individual trajectory visualization per missile
"""

import cv2
import sys
import os
from datetime import datetime
import time

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from test_camera_manager_multi import TestCameraManagerMulti
from ir_processor import IRProcessor
from optical_processor import OpticalProcessor
from fusion_engine import FusionEngine
from ukf_tracker import UKFTracker
from coordinate_transformer import CoordinateTransformer
from gis_plotter import GISPlotter
from web_server import WebServer


class MultiMissileTracker:
    """
    Tracks a single missile through detection and UKF tracking.
    """
    
    def __init__(self, missile_id, config):
        """
        Initialize tracker for one missile.
        
        Args:
            missile_id (int): Unique ID for this missile
            config (dict): Configuration dictionary
        """
        self.id = missile_id
        self.tracker = UKFTracker(dt=1/config['fps'])
        self.coord_transformer = CoordinateTransformer(
            base_lat=config.get('gis_center_lat', 28.6139),
            base_lon=config.get('gis_center_lon', 77.2090),
            altitude=config.get('camera_altitude', 100.0),
            camera_fov_horizontal=config.get('camera_fov', 62.2),
            image_width=config.get('frame_width', 640),
            image_height=config.get('frame_height', 480)
        )
        
        # State
        self.active = False
        self.detection_time = None
        self.tracking_count = 0
        self.last_position = None
        self.trajectory = []  # List of (lat, lon, timestamp) tuples
        
        # Color for this missile
        colors = [
            'red', 'blue', 'green', 'purple', 'orange', 
            'darkred', 'darkblue', 'darkgreen', 'pink', 'cadetblue'
        ]
        self.color = colors[missile_id % len(colors)]
    
    def activate(self, position, timestamp):
        """
        Activate tracking for this missile.
        
        Args:
            position (tuple): (x, y) pixel coordinates
            timestamp (datetime): Detection timestamp
        """
        self.active = True
        self.detection_time = timestamp
        self.last_position = position
        
        # Initialize UKF
        self.tracker.update(position)
        
        # Convert to geographic
        lat, lon = self.coord_transformer.pixel_to_geographic(*position)
        self.trajectory.append((lat, lon, timestamp))
    
    def update(self, position):
        """
        Update tracker with new measurement.
        
        Args:
            position (tuple): (x, y) pixel coordinates
            
        Returns:
            tuple: (lat, lon, vx, vy) or None
        """
        if not self.active:
            return None
        
        # Update UKF
        state = self.tracker.update(position)
        if state is None:
            return None
        
        x, vx, y, vy = state
        self.last_position = (x, y)
        self.tracking_count += 1
        
        # Convert to geographic
        lat, lon = self.coord_transformer.pixel_to_geographic(x, y)
        self.trajectory.append((lat, lon, datetime.now()))
        
        return (lat, lon, vx, vy)


class MultiThreatDetectionSystem:
    """
    Multi-missile threat detection and tracking system for testing.
    Implements full detection → tracking pipeline for multiple targets.
    """

    def __init__(self, config, num_missiles=3, tracking_duration=None):
        """
        Initializes the multi-missile test system.

        Args:
            config (dict): Configuration dictionary
            num_missiles (int): Number of missiles to simulate
            tracking_duration (float): Tracking duration in seconds (None for auto)
        """
        self.config = config
        self.num_missiles = num_missiles
        self.tracking_duration = tracking_duration
        
        # Calculate launch interval based on duration
        if tracking_duration:
            # Spread launches across 1/3 of duration
            launch_interval = int((tracking_duration * config['fps']) / (3 * max(num_missiles, 1)))
        else:
            launch_interval = 50  # Default: 50 frames between launches
        
        # Initialize components
        print("\n" + "="*60)
        print("MULTI-MISSILE THREAT DETECTION SYSTEM")
        print("="*60)
        
        # Camera manager with multi-missile support
        self.camera_manager = TestCameraManagerMulti(
            width=config.get('frame_width', 640),
            height=config.get('frame_height', 480),
            fps=config['fps'],
            num_missiles=num_missiles,
            tracking_duration=tracking_duration,
            launch_interval=launch_interval
        )
        
        # Detection components (shared across all missiles)
        self.ir_processor = IRProcessor(
            threshold_value=config.get('ir_threshold', 220),
            min_area=config.get('ir_min_area', 100)
        )
        
        self.optical_processor = OpticalProcessor(
            bg_subtractor_type=config.get('bg_subtractor', 'MOG2')
        )
        
        self.fusion_engine = FusionEngine()
        
        # Individual trackers for each missile
        self.trackers = {}  # missile_id -> MultiMissileTracker
        
        # GIS and web server
        self.gis_plotter = GISPlotter(
            center_lat=config['gis_center_lat'],
            center_lon=config['gis_center_lon'],
            zoom_start=config['gis_zoom']
        )
        
        self.web_server = WebServer(
            self.gis_plotter,
            port=config.get('web_server_port', 5000)
        )
        
        # Global state
        self.frame_count = 0
        self.next_tracker_id = 0
        
        # Statistics
        self.stats = {
            'total_frames': 0,
            'detections': 0,
            'active_tracks': 0,
            'completed_tracks': 0,
            'max_simultaneous': 0
        }

    def initialize(self):
        """
        Initializes all system components.

        Returns:
            bool: True if initialization successful
        """
        print("\n✅ Initializing multi-missile system...")
        if not self.camera_manager.initialize_cameras():
            print("❌ ERROR: Failed to initialize cameras!")
            return False

        print("✅ Starting web server...")
        self.web_server.start(threaded=True)

        print("\n" + "="*60)
        print("✅ SYSTEM READY!")
        print("="*60)
        print(f"🎯 Missiles to track: {self.num_missiles}")
        print(f"🌐 Web interface: http://localhost:{self.config.get('web_server_port', 5000)}")
        print("="*60 + "\n")
        
        return True

    def process_frame(self, ir_frame, optical_frame):
        """
        Process frame through DETECTION → TRACKING pipeline.
        
        Workflow:
        1. DETECTION: Find IR hotspots
        2. CONFIRMATION: Visual verification
        3. FUSION: Bayesian belief network
        4. TRACKING: Assign to existing tracker or create new
        5. UPDATE: Update GIS map
        
        Args:
            ir_frame: IR camera frame
            optical_frame: Optical camera frame
            
        Returns:
            dict: Processing results
        """
        results = {
            'detections': [],
            'active_tracks': 0,
            'new_tracks': 0
        }
        
        # ==========================================
        # STAGE 1: DETECTION - Find all IR hotspots
        # ==========================================
        hotspot_coords = self.ir_processor.find_hotspot(ir_frame)
        
        if hotspot_coords is None:
            # No detection this frame - just continue without updating trackers
            # Trackers will continue to predict when they get new measurements
            for tracker in self.trackers.values():
                if tracker.active:
                    results['active_tracks'] += 1
            return results
        
        self.stats['detections'] += 1
        
        # ================================================
        # STAGE 2: CONFIRMATION - Visual verification
        # ================================================
        visual_confirmed = self.optical_processor.detect_motion(optical_frame, hotspot_coords)
        
        # =====================================================
        # STAGE 3: FUSION - Bayesian belief network
        # =====================================================
        is_launch, confidence = self.fusion_engine.is_launch_detected(
            ir_detected=True,
            visual_confirmed=visual_confirmed,
            threshold=self.config['confidence_threshold']
        )
        
        if not is_launch:
            return results
        
        # =====================================================
        # STAGE 4: TRACKING - Assign to tracker
        # =====================================================
        
        # Try to assign to existing tracker (closest within threshold)
        assigned = False
        min_distance = float('inf')
        closest_tracker = None
        
        for tracker in self.trackers.values():
            if tracker.active and tracker.last_position:
                dist = ((hotspot_coords[0] - tracker.last_position[0])**2 + 
                       (hotspot_coords[1] - tracker.last_position[1])**2) ** 0.5
                if dist < 50 and dist < min_distance:  # 50 pixel threshold
                    min_distance = dist
                    closest_tracker = tracker
        
        if closest_tracker:
            # Update existing tracker
            result = closest_tracker.update(hotspot_coords)
            if result:
                lat, lon, vx, vy = result
                results['detections'].append({
                    'id': closest_tracker.id,
                    'position': hotspot_coords,
                    'confidence': confidence,
                    'coords': (lat, lon),
                    'velocity': (vx, vy),
                    'new': False
                })
                results['active_tracks'] += 1
        else:
            # Create new tracker for new missile
            tracker_id = self.next_tracker_id
            self.next_tracker_id += 1
            
            new_tracker = MultiMissileTracker(tracker_id, self.config)
            new_tracker.activate(hotspot_coords, datetime.now())
            self.trackers[tracker_id] = new_tracker
            
            print(f"\n{'='*60}")
            print(f"🚨 NEW MISSILE DETECTED - TRACK #{tracker_id + 1}")
            print(f"   Confidence: {confidence:.2%}")
            print(f"   Time: {new_tracker.detection_time}")
            print(f"   Position: {hotspot_coords}")
            print(f"   Frame: {self.frame_count}")
            print(f"{'='*60}\n")
            
            # Add to map
            lat, lon = new_tracker.trajectory[0][:2]
            self.gis_plotter.add_launch_point(
                lat, lon,
                timestamp=new_tracker.detection_time.strftime("%Y-%m-%d %H:%M:%S")
            )
            
            results['detections'].append({
                'id': tracker_id,
                'position': hotspot_coords,
                'confidence': confidence,
                'coords': (lat, lon),
                'new': True
            })
            results['new_tracks'] += 1
            results['active_tracks'] += 1
        
        # Update statistics
        active_count = sum(1 for t in self.trackers.values() if t.active)
        self.stats['active_tracks'] = active_count
        self.stats['max_simultaneous'] = max(self.stats['max_simultaneous'], active_count)
        
        return results

    def run(self, display=True, save_interval=30):
        """
        Main demo loop with multi-missile tracking.

        Args:
            display (bool): Whether to display video feeds
            save_interval (int): Frames between map saves
        """
        if not self.initialize():
            return

        print("🚀 Starting multi-missile tracking demo...\n")
        print("Press 'q' to quit early\n")

        try:
            while not self.camera_manager.is_simulation_complete():
                # Get frames
                ir_frame, optical_frame = self.camera_manager.get_frames()

                if ir_frame is None or optical_frame is None:
                    break

                # Process through detection → tracking pipeline
                results = self.process_frame(ir_frame, optical_frame)
                self.stats['total_frames'] += 1
                self.frame_count += 1

                # Display
                if display:
                    display_ir = ir_frame.copy()
                    display_optical = optical_frame.copy()

                    # Draw all active tracks
                    for detection in results['detections']:
                        x, y = detection['position']
                        tracker = self.trackers[detection['id']]
                        
                        # Draw on IR
                        cv2.circle(display_ir, (int(x), int(y)), 15, (0, 0, 255), 2)
                        cv2.putText(display_ir, f"T{detection['id']+1}", (int(x) + 20, int(y)),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        
                        # Draw on optical
                        cv2.circle(display_optical, (int(x), int(y)), 15, (0, 255, 0), 2)
                        cv2.putText(display_optical, f"Track #{detection['id']+1}", (int(x) + 20, int(y)),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    # Status overlay
                    cv2.putText(display_ir, f"Active Tracks: {results['active_tracks']}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(display_optical, f"Detections: {self.stats['detections']}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    cv2.imshow('IR Camera - Multi-Missile', display_ir)
                    cv2.imshow('Optical Camera - Multi-Missile', display_optical)

                # Update map
                if results['active_tracks'] > 0:
                    # Update all active trackers on map
                    for tracker in self.trackers.values():
                        if tracker.active and len(tracker.trajectory) > 0:
                            for lat, lon, _ in tracker.trajectory[-1:]:  # Last point
                                self.gis_plotter.add_trajectory_point(lat, lon)
                    
                    # Save periodically
                    if self.frame_count % save_interval == 0:
                        self.gis_plotter.save_map('test_multi_threat_map.html', verbose=False)

                # Print tracking updates
                if self.frame_count % 20 == 0 and results['active_tracks'] > 0:
                    print(f"📊 Frame {self.frame_count} | Active Tracks: {results['active_tracks']} | "
                          f"Total Detections: {self.stats['detections']}")

                # Check for quit
                if display:
                    key = cv2.waitKey(30) & 0xFF
                    if key == ord('q'):
                        print("\n⏹️  Early exit requested...")
                        break

        except KeyboardInterrupt:
            print("\n\n⏹️  Shutdown requested by user...")

        finally:
            self.shutdown()

    def shutdown(self):
        """
        Cleanly shuts down system and saves results.
        """
        print("\n" + "="*60)
        print("SHUTTING DOWN MULTI-MISSILE SYSTEM")
        print("="*60)
        
        # Save final map with all trajectories
        self.gis_plotter.save_map('test_multi_threat_map_final.html')
        print(f"\n✅ Final multi-missile map saved: test_multi_threat_map_final.html")

        # Display statistics
        print("\n" + "="*60)
        print("MULTI-MISSILE TRACKING STATISTICS")
        print("="*60)
        print(f"Total Frames:          {self.stats['total_frames']}")
        print(f"Total Detections:      {self.stats['detections']}")
        print(f"Missiles Tracked:      {len(self.trackers)}")
        print(f"Max Simultaneous:      {self.stats['max_simultaneous']}")
        print(f"Tracking Duration:     {self.stats['total_frames'] / self.config['fps']:.1f}s")
        
        print(f"\n{'='*60}")
        print("PER-MISSILE TRACKING DETAILS")
        print(f"{'='*60}")
        for tracker_id, tracker in self.trackers.items():
            print(f"  Track #{tracker_id + 1}:")
            print(f"    Detection Time:  {tracker.detection_time}")
            print(f"    Tracking Frames: {tracker.tracking_count}")
            print(f"    Trajectory Pts:  {len(tracker.trajectory)}")
        print("="*60)
        
        self.camera_manager.release_cameras()
        print("\n✅ Multi-missile system shutdown complete.")


def main():
    """
    Main entry point for multi-missile demo.
    """
    # Get user input
    print("\n" + "="*70)
    print(" "*15 + "MULTI-MISSILE TRACKING DEMO")
    print("="*70)
    
    try:
        num_missiles = input("\nHow many missiles to track? (1-10, default=3): ").strip()
        num_missiles = int(num_missiles) if num_missiles else 3
        num_missiles = max(1, min(num_missiles, 10))
    except ValueError:
        num_missiles = 3
    
    try:
        duration = input("Tracking duration in seconds? (leave empty for auto): ").strip()
        tracking_duration = float(duration) if duration else None
    except ValueError:
        tracking_duration = None
    
    print("\n" + "="*70)
    print(f"🎯 Configuration:")
    print(f"   Missiles: {num_missiles}")
    print(f"   Duration: {'Auto-detect' if tracking_duration is None else f'{tracking_duration}s'}")
    print("="*70)
    
    input("\nPress Enter to start multi-missile demo...")
    
    # Configuration
    config = {
        'fps': 30,
        'frame_width': 640,
        'frame_height': 480,
        'ir_threshold': 220,
        'ir_min_area': 100,
        'bg_subtractor': 'MOG2',
        'confidence_threshold': 0.7,
        'camera_fov': 62.2,
        'camera_altitude': 100.0,
        'gis_center_lat': 28.6139,
        'gis_center_lon': 77.2090,
        'gis_zoom': 15,
        'web_server_port': 5000
    }

    # Create and run system
    system = MultiThreatDetectionSystem(
        config=config,
        num_missiles=num_missiles,
        tracking_duration=tracking_duration
    )
    system.run(display=True, save_interval=30)


if __name__ == '__main__':
    main()
