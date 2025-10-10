"""
Enhanced Test Camera Manager: Simulates multiple missile launches
Supports custom number of missiles and tracking duration
"""

import cv2
import numpy as np
import math
import random
from datetime import datetime


class Missile:
    """
    Represents a single missile with its own trajectory.
    """
    
    def __init__(self, launch_frame, width, height, missile_id=0):
        """
        Initialize a missile.
        
        Args:
            launch_frame (int): Frame number when missile launches
            width (int): Frame width
            height (int): Frame height
            missile_id (int): Unique identifier for this missile
        """
        self.id = missile_id
        self.launch_frame = launch_frame
        self.launched = False
        self.active = True
        
        # Random launch position (from different locations)
        positions = [
            (width // 4, height - 50),      # Left
            (width // 2, height - 50),      # Center
            (3 * width // 4, height - 50),  # Right
            (width // 6, height - 30),      # Left-low
            (5 * width // 6, height - 30),  # Right-low
        ]
        self.x, self.y = positions[missile_id % len(positions)]
        
        # Random velocity (varied trajectories)
        self.vx = random.uniform(2.0, 4.0) * random.choice([-1, 1])  # Left or right
        self.vy = random.uniform(-5.0, -3.0)  # Upward
        self.gravity = 0.1
        
        # Color for visualization (unique per missile)
        colors = [
            (255, 100, 100),  # Red-ish
            (100, 255, 100),  # Green-ish
            (100, 100, 255),  # Blue-ish
            (255, 255, 100),  # Yellow-ish
            (255, 100, 255),  # Magenta-ish
        ]
        self.color = colors[missile_id % len(colors)]
        
    def update(self):
        """
        Update missile position with physics.
        """
        if not self.launched or not self.active:
            return
            
        # Update position
        self.x += self.vx
        self.y += self.vy
        self.vy += self.gravity  # Apply gravity
        
        # Deactivate if out of bounds
        if self.y > 600 or self.x < -100 or self.x > 800:
            self.active = False
    
    def get_position(self):
        """
        Get current position.
        
        Returns:
            tuple: (x, y) coordinates
        """
        return (int(self.x), int(self.y))


class TestCameraManagerMulti:
    """
    Simulates camera feeds with multiple missile launches for advanced demo.
    """

    def __init__(self, width=640, height=480, fps=30, num_missiles=3, 
                 tracking_duration=30.0, launch_interval=50):
        """
        Initializes the multi-missile test camera manager.

        Args:
            width (int): Frame width
            height (int): Frame height
            fps (int): Frames per second
            num_missiles (int): Number of missiles to simulate (1-10)
            tracking_duration (float): How long to track in seconds (None for auto)
            launch_interval (int): Frames between each missile launch
        """
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_count = 0
        
        # Multi-missile configuration
        self.num_missiles = max(1, min(num_missiles, 10))  # Clamp to 1-10
        self.tracking_duration = tracking_duration
        self.launch_interval = launch_interval
        
        # Calculate max frames
        if tracking_duration is None:
            # Auto: Track until all missiles are done (max 30 seconds)
            self.max_frames = min(fps * 30, fps * 60)
        else:
            self.max_frames = int(fps * tracking_duration)
        
        # Create missiles with staggered launch times
        self.missiles = []
        for i in range(self.num_missiles):
            launch_frame = 50 + (i * launch_interval)  # Stagger launches
            if launch_frame < self.max_frames - 100:  # Ensure enough tracking time
                missile = Missile(launch_frame, width, height, missile_id=i)
                self.missiles.append(missile)
        
        # Update actual count (some may not fit in time window)
        self.num_missiles = len(self.missiles)
        
        # Background
        self.background_ir = self._generate_background('ir')
        self.background_optical = self._generate_background('optical')
        
        print(f"\n{'='*60}")
        print(f"🎯 MULTI-MISSILE SIMULATION INITIALIZED")
        print(f"{'='*60}")
        print(f"Number of missiles:    {self.num_missiles}")
        print(f"Tracking duration:     {self.max_frames / fps:.1f} seconds ({self.max_frames} frames)")
        print(f"Launch interval:       {launch_interval} frames ({launch_interval/fps:.1f}s)")
        print(f"Frame rate:            {fps} FPS")
        print(f"{'='*60}\n")

    def _generate_background(self, camera_type):
        """
        Generates a realistic background for the camera type.

        Args:
            camera_type (str): 'ir' or 'optical'

        Returns:
            numpy.ndarray: Background image
        """
        background = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        if camera_type == 'ir':
            # IR background: mostly dark with some thermal variations
            background[:, :] = (20, 20, 30)
            # Add some thermal noise
            noise = np.random.randint(0, 20, (self.height, self.width, 3), dtype=np.uint8)
            background = cv2.add(background, noise)
            
            # Add some hot spots (buildings, etc.)
            for _ in range(5):
                x = np.random.randint(50, self.width - 50)
                y = np.random.randint(50, self.height - 50)
                cv2.circle(background, (x, y), np.random.randint(10, 30), 
                          (80, 80, 100), -1)
        else:
            # Optical background: sky and ground
            # Sky gradient
            for y in range(self.height // 2):
                color = int(180 - y * 0.3)
                background[y, :] = (color, color - 20, 100)
            
            # Ground
            background[self.height // 2:, :] = (60, 120, 60)
            
            # Add some clouds
            for _ in range(10):
                x = np.random.randint(0, self.width)
                y = np.random.randint(0, self.height // 2)
                cv2.circle(background, (x, y), np.random.randint(20, 60), 
                          (240, 240, 250), -1)
        
        return background

    def initialize_cameras(self):
        """
        Mock camera initialization (always succeeds for testing).

        Returns:
            bool: Always True
        """
        print("Multi-missile test cameras initialized (simulated)")
        return True

    def get_frames(self):
        """
        Generates synthetic IR and optical frames with multiple missile simulation.

        Returns:
            tuple: (ir_frame, optical_frame)
        """
        self.frame_count += 1
        
        # Create frames from background
        ir_frame = self.background_ir.copy()
        optical_frame = self.background_optical.copy()
        
        # Add random noise
        noise = np.random.randint(-10, 10, ir_frame.shape, dtype=np.int16)
        ir_frame = np.clip(ir_frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        # Process each missile
        active_missiles = 0
        for missile in self.missiles:
            # Launch missile at its designated frame
            if self.frame_count == missile.launch_frame and not missile.launched:
                missile.launched = True
                print(f"\n{'='*60}")
                print(f"🚀 MISSILE #{missile.id + 1} LAUNCHED!")
                print(f"   Frame: {self.frame_count} ({self.frame_count/self.fps:.1f}s)")
                print(f"   Position: {missile.get_position()}")
                print(f"   Velocity: ({missile.vx:.1f}, {missile.vy:.1f}) px/frame")
                print(f"{'='*60}\n")
            
            # Update and draw missile if launched and active
            if missile.launched and missile.active:
                active_missiles += 1
                missile.update()
                x, y = missile.get_position()
                
                # Draw on IR frame (bright hotspot)
                if 0 <= x < self.width and 0 <= y < self.height:
                    # Main hotspot
                    cv2.circle(ir_frame, (x, y), 15, (255, 255, 255), -1)
                    # Glow effect
                    cv2.circle(ir_frame, (x, y), 25, (200, 200, 200), 5)
                    cv2.circle(ir_frame, (x, y), 35, (150, 150, 150), 3)
                    
                    # Draw on optical frame (missile body + smoke trail)
                    # Missile with unique color
                    cv2.circle(optical_frame, (x, y), 8, missile.color, -1)
                    cv2.circle(optical_frame, (x, y), 10, (80, 80, 80), 2)
                    
                    # Smoke trail
                    for i in range(5):
                        trail_x = int(x - i * missile.vx * 1.5)
                        trail_y = int(y - i * missile.vy * 1.5)
                        if 0 <= trail_x < self.width and 0 <= trail_y < self.height:
                            alpha = 1.0 - (i * 0.15)
                            smoke_color = int(150 * alpha)
                            cv2.circle(optical_frame, (trail_x, trail_y), 
                                     5 + i, (smoke_color, smoke_color, smoke_color), -1)
                    
                    # Label missile number
                    cv2.putText(optical_frame, f"M{missile.id + 1}", (x + 15, y - 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, missile.color, 2)
        
        # Add status info
        cv2.putText(ir_frame, f"Frame: {self.frame_count} | Active: {active_missiles}/{self.num_missiles}", 
                   (10, self.height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(optical_frame, f"Frame: {self.frame_count} | Time: {self.frame_count/self.fps:.1f}s", 
                   (10, self.height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return ir_frame, optical_frame

    def is_simulation_complete(self):
        """
        Check if simulation should end.
        
        Returns:
            bool: True if all missiles inactive or max frames reached
        """
        if self.frame_count >= self.max_frames:
            return True
        
        # If auto-detect mode, check if all missiles are done
        if self.tracking_duration is None:
            all_inactive = all(not m.active for m in self.missiles)
            # Give extra 100 frames after last missile goes inactive
            if all_inactive and self.frame_count > 100:
                return True
        
        return False

    def release_cameras(self):
        """
        Mock camera release.
        """
        print("Multi-missile test cameras released (simulated)")
        cv2.destroyAllWindows()

    def reset(self):
        """
        Resets the simulation.
        """
        self.frame_count = 0
        for missile in self.missiles:
            missile.launched = False
            missile.active = True

    def get_statistics(self):
        """
        Get simulation statistics.
        
        Returns:
            dict: Statistics about the simulation
        """
        launched = sum(1 for m in self.missiles if m.launched)
        active = sum(1 for m in self.missiles if m.active)
        
        return {
            'total_missiles': self.num_missiles,
            'launched': launched,
            'active': active,
            'completed': launched - active,
            'frame_count': self.frame_count,
            'elapsed_time': self.frame_count / self.fps
        }
