#!/usr/bin/env python3

"""
coordinate_transformer.py

Transforms 2D pixel coordinates into 3D geographic coordinates.
"""

import numpy as np

class CoordinateTransformer:
    """
    Handles the conversion of pixel coordinates to geographic coordinates.
    """
    def __init__(self, camera_matrix, dist_coeffs, camera_altitude, camera_fov):
        """
        Initializes the transformer with camera parameters.

        Args:
            camera_matrix: The camera's intrinsic matrix.
            dist_coeffs: The camera's distortion coefficients.
            camera_altitude: The altitude of the camera in meters.
            camera_fov: The camera's field of view in degrees.
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.camera_altitude = camera_altitude
        self.camera_fov_rad = np.deg2rad(camera_fov)
    
    def pixel_to_geo(self, pixel_x, pixel_y, drone_lat, drone_lon):
        """
        Converts a single pixel coordinate to a geographic coordinate.

        Args:
            pixel_x: The x-coordinate of the pixel.
            pixel_y: The y-coordinate of the pixel.
            drone_lat: The latitude of the drone.
            drone_lon: The longitude of the drone.

        Returns:
            A tuple of (latitude, longitude).
        """
        # For this PoC, we use a simplified trigonometric approximation
        # Assumes the camera is pointing straight down (nadir)
        
        # Get image dimensions from camera matrix
        img_width = self.camera_matrix[0, 2] * 2
        img_height = self.camera_matrix[1, 2] * 2

        # Calculate the ground distance per pixel
        ground_width = 2 * self.camera_altitude * np.tan(self.camera_fov_rad / 2)
        pixels_per_meter = img_width / ground_width

        # Calculate distance from center in meters
        dx = (pixel_x - img_width / 2) / pixels_per_meter
        dy = (pixel_y - img_height / 2) / pixels_per_meter

        # Convert meters to decimal degrees (approximation)
        # 1 degree of latitude is approx. 111,111 meters
        d_lat = dy / 111111.0
        d_lon = dx / (111111.0 * np.cos(np.deg2rad(drone_lat)))

        return drone_lat + d_lat, drone_lon + d_lon