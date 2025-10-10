"""
Test Camera Manager: Simulates camera feeds for Windows demo
Generates synthetic IR and optical video feeds with missile simulation
"""

import cv2
import numpy as np
import math


class TestCameraManager:
    """
    Simulates camera feeds with synthetic missile launch for demo purposes.
    """

    def __init__(self, width=640, height=480, fps=30):
        """
        Initializes the test camera manager.

        Args:
            width (int): Frame width
            height (int): Frame height
            fps (int): Frames per second
        """
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_count = 0
        
        # Missile trajectory parameters
        self.missile_launched = False
        self.launch_frame = 50  # Launch after 50 frames
        self.missile_x = None
        self.missile_y = None
        self.missile_vx = 3.0  # Horizontal velocity (pixels/frame)
        self.missile_vy = -4.0  # Vertical velocity (pixels/frame, negative = up)
        self.gravity = 0.1  # Gravity acceleration (pixels/frame²)
        
        # Background
        self.background_ir = self._generate_background('ir')
        self.background_optical = self._generate_background('optical')

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
        print("Test cameras initialized (simulated)")
        return True

    def get_frames(self):
        """
        Generates synthetic IR and optical frames with missile simulation.

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
        
        # Launch missile at specific frame
        if self.frame_count == self.launch_frame:
            self.missile_launched = True
            self.missile_x = self.width // 4  # Launch from left side
            self.missile_y = self.height - 50  # Near bottom
            print(f"\n{'='*60}")
            print(f"🚀 SIMULATED MISSILE LAUNCH!")
            print(f"   Frame: {self.frame_count}")
            print(f"   Initial position: ({self.missile_x}, {self.missile_y})")
            print(f"{'='*60}\n")
        
        # Update and draw missile if launched
        if self.missile_launched and self.missile_y > -50:  # Continue until missile exits frame
            # Update position
            self.missile_x += self.missile_vx
            self.missile_y += self.missile_vy
            self.missile_vy += self.gravity  # Apply gravity
            
            x, y = int(self.missile_x), int(self.missile_y)
            
            # Draw on IR frame (bright hotspot)
            if 0 <= x < self.width and 0 <= y < self.height:
                # Main hotspot
                cv2.circle(ir_frame, (x, y), 15, (255, 255, 255), -1)
                # Glow effect
                cv2.circle(ir_frame, (x, y), 25, (200, 200, 200), 5)
                cv2.circle(ir_frame, (x, y), 35, (150, 150, 150), 3)
                
                # Draw on optical frame (missile body + smoke trail)
                # Missile
                cv2.circle(optical_frame, (x, y), 8, (100, 100, 100), -1)
                cv2.circle(optical_frame, (x, y), 10, (80, 80, 80), 2)
                
                # Smoke trail
                for i in range(5):
                    trail_x = int(x - i * self.missile_vx * 1.5)
                    trail_y = int(y - i * self.missile_vy * 1.5)
                    if 0 <= trail_x < self.width and 0 <= trail_y < self.height:
                        alpha = 1.0 - (i * 0.15)
                        smoke_color = int(150 * alpha)
                        cv2.circle(optical_frame, (trail_x, trail_y), 
                                 5 + i, (smoke_color, smoke_color, smoke_color), -1)
        
        # Add frame counter
        cv2.putText(ir_frame, f"Frame: {self.frame_count}", (10, self.height - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(optical_frame, f"Frame: {self.frame_count}", (10, self.height - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return ir_frame, optical_frame

    def release_cameras(self):
        """
        Mock camera release.
        """
        print("Test cameras released (simulated)")
        cv2.destroyAllWindows()

    def reset(self):
        """
        Resets the simulation.
        """
        self.frame_count = 0
        self.missile_launched = False
        self.missile_x = None
        self.missile_y = None
        self.missile_vy = -4.0
