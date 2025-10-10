"""
Coordinate Transformer: Converts pixel coordinates to geographic coordinates
"""

import numpy as np
import math


class CoordinateTransformer:
    """
    Converts pixel coordinates to geographic (latitude, longitude) coordinates.
    Uses camera parameters and platform position for transformation.
    """

    def __init__(self, base_lat=28.6139, base_lon=77.2090, altitude=100.0,
                 camera_fov_horizontal=62.2, camera_fov_vertical=48.8,
                 image_width=640, image_height=480, camera_tilt=0):
        """
        Initializes the coordinate transformer.

        Args:
            base_lat (float): Latitude of the camera platform (default: New Delhi)
            base_lon (float): Longitude of the camera platform
            altitude (float): Altitude of the camera above ground in meters
            camera_fov_horizontal (float): Horizontal field of view in degrees
            camera_fov_vertical (float): Vertical field of view in degrees
            image_width (int): Width of the image in pixels
            image_height (int): Height of the image in pixels
            camera_tilt (float): Camera tilt angle in degrees (0 = pointing down)
        """
        self.base_lat = base_lat
        self.base_lon = base_lon
        self.altitude = altitude
        self.camera_fov_h = math.radians(camera_fov_horizontal)
        self.camera_fov_v = math.radians(camera_fov_vertical)
        self.image_width = image_width
        self.image_height = image_height
        self.camera_tilt = math.radians(camera_tilt)

        # Earth radius in meters (approximate)
        self.earth_radius = 6371000

        # Calculate ground coverage
        self._calculate_ground_coverage()

    def _calculate_ground_coverage(self):
        """
        Calculates the ground coverage of the camera at the current altitude.
        """
        # Calculate ground distance covered by camera FOV
        # Using simple trigonometry for small angles
        self.ground_width = 2 * self.altitude * math.tan(self.camera_fov_h / 2)
        self.ground_height = 2 * self.altitude * math.tan(self.camera_fov_v / 2)

        # Meters per pixel
        self.meters_per_pixel_x = self.ground_width / self.image_width
        self.meters_per_pixel_y = self.ground_height / self.image_height

    def update_position(self, lat, lon, altitude):
        """
        Updates the camera platform position.

        Args:
            lat (float): New latitude
            lon (float): New longitude
            altitude (float): New altitude in meters
        """
        self.base_lat = lat
        self.base_lon = lon
        self.altitude = altitude
        self._calculate_ground_coverage()

    def pixel_to_geographic(self, pixel_x, pixel_y, estimated_altitude=None):
        """
        Converts pixel coordinates to geographic coordinates.

        Args:
            pixel_x (float): X coordinate in pixels
            pixel_y (float): Y coordinate in pixels
            estimated_altitude (float): Estimated altitude of the object (if None, assumes ground level)

        Returns:
            tuple: (latitude, longitude) in decimal degrees
        """
        # Calculate offset from image center in pixels
        center_x = self.image_width / 2
        center_y = self.image_height / 2
        
        offset_x = pixel_x - center_x
        offset_y = pixel_y - center_y

        # Convert to meters
        offset_meters_x = offset_x * self.meters_per_pixel_x
        offset_meters_y = -offset_y * self.meters_per_pixel_y  # Negative because y increases downward in images

        # Convert meters to degrees
        # Longitude offset
        lat_rad = math.radians(self.base_lat)
        delta_lon = math.degrees(offset_meters_x / (self.earth_radius * math.cos(lat_rad)))
        
        # Latitude offset
        delta_lat = math.degrees(offset_meters_y / self.earth_radius)

        # Calculate final coordinates
        final_lat = self.base_lat + delta_lat
        final_lon = self.base_lon + delta_lon

        return final_lat, final_lon

    def batch_pixel_to_geographic(self, pixel_coords):
        """
        Converts multiple pixel coordinates to geographic coordinates.

        Args:
            pixel_coords (list): List of (x, y) pixel coordinate tuples

        Returns:
            list: List of (latitude, longitude) tuples
        """
        return [self.pixel_to_geographic(x, y) for x, y in pixel_coords]

    def get_ground_coverage(self):
        """
        Returns the ground coverage information.

        Returns:
            dict: Dictionary with ground coverage information
        """
        return {
            'ground_width_meters': self.ground_width,
            'ground_height_meters': self.ground_height,
            'meters_per_pixel_x': self.meters_per_pixel_x,
            'meters_per_pixel_y': self.meters_per_pixel_y
        }
