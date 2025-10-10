"""
GIS Plotter: Creates and updates interactive maps with trajectory visualization
"""

import folium
from folium import plugins
import os


class GISPlotter:
    """
    Creates and updates a Folium map to visualize the projectile trajectory.
    """

    def __init__(self, center_lat=28.6139, center_lon=77.2090, zoom_start=15):
        """
        Initializes the GIS plotter.

        Args:
            center_lat (float): Latitude for map center
            center_lon (float): Longitude for map center
            zoom_start (int): Initial zoom level
        """
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.zoom_start = zoom_start
        
        # Create the base map
        self.map = folium.Map(
            location=[center_lat, center_lon],
            zoom_start=zoom_start,
            tiles='OpenStreetMap'
        )

        # Add satellite/terrain imagery options
        # Using CartoDB which is still actively maintained
        folium.TileLayer(
            'https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png',
            attr='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors &copy; <a href="https://carto.com/attributions">CARTO</a>',
            name='CartoDB Voyager',
            max_zoom=20
        ).add_to(self.map)
        
        # Add layer control
        folium.LayerControl().add_to(self.map)
        
        # Storage for trajectory points
        self.trajectory_points = []
        
        # Marker for launch point
        self.launch_marker = None
        
        # Marker for current position
        self.current_marker = None
        
        # Polyline for trajectory
        self.trajectory_line = None

    def add_launch_point(self, lat, lon, timestamp=None):
        """
        Adds a marker for the launch point.

        Args:
            lat (float): Latitude of launch point
            lon (float): Longitude of launch point
            timestamp (str): Timestamp of detection (optional)
        """
        popup_text = f"Launch Detected"
        if timestamp:
            popup_text += f"<br>Time: {timestamp}"
            
        self.launch_marker = folium.Marker(
            location=[lat, lon],
            popup=folium.Popup(popup_text, max_width=250),
            icon=folium.Icon(color='red', icon='warning-sign')
        )
        self.launch_marker.add_to(self.map)
        
        # Add to trajectory
        self.trajectory_points.append([lat, lon])

    def add_trajectory_point(self, lat, lon):
        """
        Adds a point to the trajectory.

        Args:
            lat (float): Latitude
            lon (float): Longitude
        """
        self.trajectory_points.append([lat, lon])

    def update_current_position(self, lat, lon, velocity=None):
        """
        Updates the current position marker.

        Args:
            lat (float): Current latitude
            lon (float): Current longitude
            velocity (tuple): (vx, vy) velocity components (optional)
        """
        # Remove old marker if exists
        if self.current_marker:
            # Folium doesn't support removing, so we'll recreate the map if needed
            pass
        
        popup_text = f"Current Position<br>Lat: {lat:.6f}<br>Lon: {lon:.6f}"
        if velocity:
            vx, vy = velocity
            speed = (vx**2 + vy**2)**0.5
            popup_text += f"<br>Speed: {speed:.2f} px/s"
            
        self.current_marker = folium.Marker(
            location=[lat, lon],
            popup=folium.Popup(popup_text, max_width=250),
            icon=folium.Icon(color='blue', icon='record')
        )
        self.current_marker.add_to(self.map)

    def draw_trajectory(self, color='red', weight=3):
        """
        Draws the trajectory line on the map.

        Args:
            color (str): Line color
            weight (int): Line weight
        """
        if len(self.trajectory_points) > 1:
            self.trajectory_line = folium.PolyLine(
                locations=self.trajectory_points,
                color=color,
                weight=weight,
                opacity=0.8
            )
            self.trajectory_line.add_to(self.map)

    def add_predicted_impact(self, lat, lon):
        """
        Adds a marker for predicted impact point.

        Args:
            lat (float): Predicted impact latitude
            lon (float): Predicted impact longitude
        """
        impact_marker = folium.Marker(
            location=[lat, lon],
            popup=folium.Popup("Predicted Impact Zone", max_width=250),
            icon=folium.Icon(color='darkred', icon='remove-sign')
        )
        impact_marker.add_to(self.map)
        
        # Add a circle around impact point
        folium.Circle(
            location=[lat, lon],
            radius=50,  # 50 meters radius
            color='red',
            fill=True,
            fillOpacity=0.3
        ).add_to(self.map)

    def add_heat_map(self, points):
        """
        Adds a heat map overlay for trajectory density.

        Args:
            points (list): List of [lat, lon] coordinates
        """
        if points:
            heat_data = [[pt[0], pt[1]] for pt in points]
            plugins.HeatMap(heat_data).add_to(self.map)

    def save_map(self, filename='threat_map.html', verbose=True):
        """
        Saves the map to an HTML file.

        Args:
            filename (str): Output filename
            verbose (bool): Whether to print save confirmation
        """
        # Draw trajectory before saving
        self.draw_trajectory()
        
        # Save the map
        self.map.save(filename)
        if verbose:
            print(f"Map saved to {filename}")

    def clear_trajectory(self):
        """
        Clears the current trajectory data.
        """
        self.trajectory_points = []
        
        # Recreate the map
        self.map = folium.Map(
            location=[self.center_lat, self.center_lon],
            zoom_start=self.zoom_start,
            tiles='OpenStreetMap'
        )

    def get_map_html(self):
        """
        Returns the map as HTML string.

        Returns:
            str: HTML representation of the map
        """
        self.draw_trajectory()
        return self.map._repr_html_()

    def create_animated_trajectory(self, points_with_time, filename='animated_map.html'):
        """
        Creates an animated trajectory visualization.

        Args:
            points_with_time (list): List of (lat, lon, timestamp) tuples
            filename (str): Output filename
        """
        features = []
        for i, (lat, lon, timestamp) in enumerate(points_with_time):
            feature = {
                'type': 'Feature',
                'geometry': {
                    'type': 'Point',
                    'coordinates': [lon, lat]
                },
                'properties': {
                    'time': timestamp,
                    'style': {'color': 'red'},
                    'icon': 'circle',
                    'iconstyle': {
                        'fillColor': 'red',
                        'fillOpacity': 0.8,
                        'stroke': 'true',
                        'radius': 5
                    }
                }
            }
            features.append(feature)

        plugins.TimestampedGeoJson(
            {'type': 'FeatureCollection', 'features': features},
            period='PT1S',
            add_last_point=True,
            auto_play=True,
            loop=False,
            max_speed=1,
            loop_button=True,
            date_options='YYYY-MM-DD HH:mm:ss',
            time_slider_drag_update=True
        ).add_to(self.map)

        self.map.save(filename)
        print(f"Animated map saved to {filename}")
