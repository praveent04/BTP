#!/usr/bin/env python3

"""
web_server.py

A simple Flask web server to display the real-time map visualization.
"""

from flask import Flask, render_template_string
import threading
import os

class WebServer:
    """
    A Flask-based web server to serve the GIS map.
    """
    def __init__(self, host='0.0.0.0', port=8080):
        self.app = Flask(__name__)
        self.host = host
        self.port = port

        @self.app.route('/')
        def index():
            # Check if the map exists
            if not os.path.exists('map.html'):
                return "Map not yet generated. Please wait.", 404
            
            # Read the map content
            with open('map.html', 'r') as f:
                map_content = f.read()

            # Add a meta refresh tag to the HTML
            refresh_template = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>Threat Trajectory</title>
                <meta http-equiv="refresh" content="3">
            </head>
            <body>
                {map_content}
            </body>
            </html>
            """.format(map_content=map_content)
            
            return refresh_template

    def run(self):
        """
        Runs the web server in a separate thread.
        """
        server_thread = threading.Thread(
            target=self.app.run,
            kwargs={'host': self.host, 'port': self.port}
        )
        server_thread.daemon = True
        server_thread.start()