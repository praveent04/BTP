"""
Web Server: Flask-based server to display real-time threat map
"""

from flask import Flask, render_template, send_from_directory, jsonify
import os
import threading
import time


class WebServer:
    """
    Simple Flask web server to display the GIS map.
    """

    def __init__(self, map_file='threat_map.html', port=5000, host='0.0.0.0'):
        """
        Initializes the web server.

        Args:
            map_file (str): Path to the map HTML file
            port (int): Port number for the server
            host (str): Host address (0.0.0.0 allows external access)
        """
        self.map_file = map_file
        self.port = port
        self.host = host
        self.app = Flask(__name__)
        self.server_thread = None
        
        # Setup routes
        self._setup_routes()

    def _setup_routes(self):
        """
        Sets up the Flask routes.
        """
        @self.app.route('/')
        def index():
            """Main page that displays the map with auto-refresh."""
            return '''
            <!DOCTYPE html>
            <html>
            <head>
                <title>Threat Detection System - Live Map</title>
                <meta http-equiv="refresh" content="2">
                <style>
                    body {
                        margin: 0;
                        padding: 0;
                        font-family: Arial, sans-serif;
                    }
                    .header {
                        background-color: #2c3e50;
                        color: white;
                        padding: 15px;
                        text-align: center;
                    }
                    .status {
                        background-color: #27ae60;
                        color: white;
                        padding: 10px;
                        text-align: center;
                    }
                    iframe {
                        width: 100%;
                        height: calc(100vh - 100px);
                        border: none;
                    }
                </style>
            </head>
            <body>
                <div class="header">
                    <h1>🎯 Threat Detection & Tracking System</h1>
                    <p>Real-time Trajectory Visualization</p>
                </div>
                <div class="status">
                    System Status: ACTIVE | Auto-refresh: 2s | Last Update: <span id="time"></span>
                </div>
                <iframe src="/map"></iframe>
                <script>
                    document.getElementById('time').textContent = new Date().toLocaleTimeString();
                </script>
            </body>
            </html>
            '''

        @self.app.route('/map')
        def map_view():
            """Serves the actual map file."""
            try:
                map_dir = os.path.dirname(os.path.abspath(self.map_file))
                map_filename = os.path.basename(self.map_file)
                return send_from_directory(map_dir, map_filename)
            except Exception as e:
                return f"<h1>Map not yet generated</h1><p>Error: {str(e)}</p><p>Waiting for threat detection...</p>"

        @self.app.route('/status')
        def status():
            """Returns server status as JSON."""
            return jsonify({
                'status': 'running',
                'map_file': self.map_file,
                'timestamp': time.time()
            })

    def start(self, threaded=True):
        """
        Starts the web server.

        Args:
            threaded (bool): If True, runs server in a separate thread
        """
        if threaded:
            self.server_thread = threading.Thread(
                target=self._run_server,
                daemon=True
            )
            self.server_thread.start()
            print(f"Web server started at http://{self.host}:{self.port}")
            print(f"Access the map from any device on the network")
        else:
            self._run_server()

    def _run_server(self):
        """
        Internal method to run the Flask server.
        """
        self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False)

    def stop(self):
        """
        Stops the web server.
        Note: Flask doesn't provide a clean way to stop, so this is a placeholder.
        """
        print("Server stop requested. Note: Flask server continues until process ends.")


if __name__ == '__main__':
    # Test the web server
    server = WebServer(map_file='threat_map.html', port=5000)
    server.start(threaded=False)
