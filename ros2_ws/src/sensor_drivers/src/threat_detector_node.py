#!/usr/bin/env python3

"""
threat_detector_node.py

ROS2 node for detecting, tracking, and visualizing shelling threats by fusing
data from IR and optical cameras.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np
from pgmpy.models import DiscreteBayesianNetwork as BayesianNetwork
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination
from ukf_tracker import UKFTracker
from coordinate_transformer import CoordinateTransformer
from gis_plotter import GISPlotter
from web_server import WebServer

class FusionEngine:
    """
    A Bayesian Belief Network for fusing sensor data to determine the
    probability of a launch event.
    """
    def __init__(self):
        # Define the structure of the Bayesian Network
        self.model = BayesianNetwork([
            ('Launch_Event', 'IR_Hotspot_Detected'),
            ('Launch_Event', 'Visual_Confirmation')
        ])

        # Define the Conditional Probability Tables (CPTs)
        # P(Launch_Event)
        cpd_launch = TabularCPD(
            variable='Launch_Event', variable_card=2,
            values=[[0.999], [0.001]] # High prior for no launch
        )
        
        # P(IR_Hotspot_Detected | Launch_Event)
        cpd_ir = TabularCPD(
            variable='IR_Hotspot_Detected', variable_card=2,
            values=[
                [0.95, 0.01], # P(IR=F|L=T), P(IR=F|L=F)
                [0.05, 0.99]  # P(IR=T|L=T), P(IR=T|L=F)
            ],
            evidence=['Launch_Event'], evidence_card=[2]
        )

        # P(Visual_Confirmation | Launch_Event)
        cpd_visual = TabularCPD(
            variable='Visual_Confirmation', variable_card=2,
            values=[
                [0.98, 0.05], # P(VC=F|L=T), P(VC=F|L=F)
                [0.02, 0.95]  # P(VC=T|L=T), P(VC=T|L=F)
            ],
            evidence=['Launch_Event'], evidence_card=[2]
        )

        self.model.add_cpds(cpd_launch, cpd_ir, cpd_visual)
        self.model.check_model()

        # Initialize the inference engine
        self.inference = VariableElimination(self.model)

    def get_launch_probability(self, ir_detected, visual_confirmed):
        """
        Calculates the posterior probability of a launch event given the evidence.
        """
        evidence = {
            'IR_Hotspot_Detected': 1 if ir_detected else 0,
            'Visual_Confirmation': 1 if visual_confirmed else 0
        }
        result = self.inference.query(variables=['Launch_Event'], evidence=evidence)
        return result.values[1] # Return the probability of Launch_Event=True

class ThreatDetectorNode(Node):
    """
    Fuses sensor data to detect and track threats, and visualizes the trajectory.
    """
    CONFIDENCE_THRESHOLD = 0.95 # 95% confidence needed to declare a threat

    def __init__(self, testing=False):
        """
        Initializes the node, subscribers, publishers, and all processing modules.
        """
        if not testing:
            super().__init__('threat_detector_node')
            self.get_logger().info('Threat Detector Node has been initialized.')
            # ROS-specific initializations
            ir_sub = message_filters.Subscriber(self, Image, 'sensors/ir_image')
            eo_sub = message_filters.Subscriber(self, Image, 'sensors/eo_image')
            self.ts = message_filters.ApproximateTimeSynchronizer([ir_sub, eo_sub], queue_size=10, slop=0.1)
            self.ts.registerCallback(self.synchronized_callback)
            self.get_logger().info('Subscribers and Time Synchronizer are set up.')
            self.web_server = WebServer()
            self.web_server.run()
        
        # Common initializations
        try:
            self.bridge = CvBridge()
        except NameError:
            self.bridge = None # In case cv_bridge is not available
        self.fusion_engine = FusionEngine()
        self.optical_background = None
        self.ukf_tracker = None
        self.tracked_trajectory = []
        self.geo_trajectory = []

        # Mock Camera Parameters
        camera_matrix = np.array([[1000, 0, 960], [0, 1000, 540], [0, 0, 1]])
        dist_coeffs = np.zeros(4)
        camera_altitude = 100
        camera_fov = 90

        self.transformer = CoordinateTransformer(camera_matrix, dist_coeffs, camera_altitude, camera_fov)
        self.gis_plotter = GISPlotter(map_center=(40.7128, -74.0060))

    def synchronized_callback(self, ir_msg, eo_msg):
        """
        Callback for synchronized IR and EO messages.
        This is the main entry point for the processing pipeline.
        """
        self.get_logger().info('Received synchronized IR and EO frames.')
        
        try:
            # Convert ROS Image messages to OpenCV images
            ir_image = self.bridge.imgmsg_to_cv2(ir_msg, desired_encoding='mono8')
            eo_image = self.bridge.imgmsg_to_cv2(eo_msg, desired_encoding='rgb8')

            # --- Processing Pipeline Starts Here ---
            # 1. IR Processing
            hotspots = self.process_ir_frame(ir_image)
            
            # --- Visualization of Hotspots ---
            ir_display = cv2.cvtColor(ir_image, cv2.COLOR_GRAY2BGR)
            for hotspot in hotspots:
                center_x, center_y = hotspot
                cv2.circle(ir_display, (center_x, center_y), 10, (0, 0, 255), 2)
            # --- End Visualization ---

            # 2. Optical Processing
            visual_confirmation = self.process_optical_frame(eo_image, hotspots)

            # 3. Fusion
            ir_detected = len(hotspots) > 0
            launch_prob = self.fusion_engine.get_launch_probability(ir_detected, visual_confirmation)
            
            self.get_logger().info(f'Launch Probability: {launch_prob:.4f}')

            if launch_prob > self.CONFIDENCE_THRESHOLD:
                self.get_logger().warn('>>> High probability of launch event detected! <<<')

                # 4. Tracking
                if self.ukf_tracker is None:
                    # Initialize the tracker on the first detection
                    dt = 1.0 / 30.0 # Approximate time step
                    self.ukf_tracker = UKFTracker(dt)
                
                # Update the tracker with the first hotspot found
                if ir_detected:
                    z = np.array(hotspots[0])
                    self.ukf_tracker.update(z)
                    
                    # Store the smoothed trajectory
                    tracked_x = int(self.ukf_tracker.ukf.x[0])
                    tracked_y = int(self.ukf_tracker.ukf.x[2])
                    self.tracked_trajectory.append((tracked_x, tracked_y))
                    
                    # Convert to geographic coordinates
                    # Mock drone position (replace with actual GPS data)
                    drone_lat, drone_lon = 40.7128, -74.0060
                    geo_lat, geo_lon = self.transformer.pixel_to_geo(tracked_x, tracked_y, drone_lat, drone_lon)
                    self.geo_trajectory.append((geo_lat, geo_lon))

            # 5. Visualization
            # Draw the tracked path
            for i in range(1, len(self.tracked_trajectory)):
                cv2.line(eo_image, self.tracked_trajectory[i-1], self.tracked_trajectory[i], (0, 255, 0), 2)
            
            # Update and save the map
            if self.geo_trajectory:
                self.gis_plotter.add_trajectory(self.geo_trajectory)
                self.gis_plotter.save_map()
            # --- Processing Pipeline Ends Here ---

            # For now, just display the images to confirm they are received
            cv2.imshow("IR Image", ir_display)
            cv2.imshow("EO Image", eo_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error in synchronized_callback: {str(e)}')

    def process_ir_frame(self, ir_image):
        """
        Processes the raw IR frame to detect high-intensity thermal hotspots.

        Returns:
            A list of (x, y) coordinates for each detected hotspot.
        """
        # Noise reduction
        blurred = cv2.GaussianBlur(ir_image, (5, 5), 0)

        # High-intensity thresholding to isolate potential launch signatures
        _, threshold = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)
        
        # Find contours of the hot regions
        contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        hotspots = []
        for contour in contours:
            # Filter out small, noisy detections
            if cv2.contourArea(contour) > 20:
                # Calculate the centroid of the contour
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    hotspots.append((center_x, center_y))
        
        return hotspots

    def process_optical_frame(self, eo_image, hotspots):
        """
        Processes the optical frame to find corroborating evidence of a launch.

        Args:
            eo_image: The raw optical image.
            hotspots: A list of (x, y) coordinates from the IR processor.

        Returns:
            True if visual confirmation is found, False otherwise.
        """
        # Convert to grayscale for motion detection
        gray_frame = cv2.cvtColor(eo_image, cv2.COLOR_BGR2GRAY)
        gray_frame = cv2.GaussianBlur(gray_frame, (21, 21), 0)

        # Initialize background model on the first frame
        if self.optical_background is None:
            self.optical_background = gray_frame.copy().astype("float")
            return False

        # Update the background model
        cv2.accumulateWeighted(gray_frame, self.optical_background, 0.5)
        background = cv2.absdiff(gray_frame, cv2.convertScaleAbs(self.optical_background))

        # Check for motion in the ROI of each hotspot
        for hotspot in hotspots:
            x, y = hotspot
            # Define a Region of Interest (ROI) around the hotspot
            roi_size = 100  # 100x100 pixel ROI
            x1 = max(0, x - roi_size // 2)
            y1 = max(0, y - roi_size // 2)
            x2 = min(gray_frame.shape[1], x + roi_size // 2)
            y2 = min(gray_frame.shape[0], y + roi_size // 2)

            roi_background = background[y1:y2, x1:x2]
            
            # Threshold the difference image to find significant motion
            _, thresh = cv2.threshold(roi_background, 30, 255, cv2.THRESH_BINARY)
            
            # If there's enough motion, we have visual confirmation
            if cv2.countNonZero(thresh) > 50: # Threshold for motion significance
                return True # Visual confirmation found

        return False # No visual confirmation

def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    threat_detector_node = ThreatDetectorNode()
    
    try:
        rclpy.spin(threat_detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanly destroy the node
        threat_detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()