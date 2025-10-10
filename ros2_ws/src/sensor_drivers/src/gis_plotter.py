#!/usr/bin/env python3

"""
gis_plotter.py

Creates and updates a Folium map to visualize the threat trajectory.
"""

import folium

class GISPlotter:
    """
    Handles the creation and updating of the GIS map.
    """
    def __init__(self, map_center):
        """
        Initializes the plotter with a center point for the map.

        Args:
            map_center: A tuple of (latitude, longitude) for the map center.
        """
        self.map_center = map_center
        self.map = folium.Map(location=self.map_center, zoom_start=15)
    
    def add_trajectory(self, geo_trajectory):
        """
        Adds a trajectory to the map.

        Args:
            geo_trajectory: A list of (latitude, longitude) tuples.
        """
        if len(geo_trajectory) > 1:
            folium.PolyLine(geo_trajectory, color="red", weight=2.5, opacity=1).add_to(self.map)

    def save_map(self, path="map.html"):
        """
        Saves the map to an HTML file.
        """
        self.map.save(path)