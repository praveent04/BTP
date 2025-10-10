"""
Unscented Kalman Filter (UKF) Tracker for Trajectory Estimation
Tracks projectile position and velocity using visual measurements
"""

import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints


class UKFTracker:
    """
    Implements an Unscented Kalman Filter for tracking a projectile's trajectory.
    """

    def __init__(self, dt=0.033):
        """
        Initializes the UKF tracker.

        Args:
            dt (float): Time step between measurements (in seconds). Default is ~30 fps.
        """
        self.dt = dt
        
        # State vector: [x, vx, y, vy]
        # x, y: pixel coordinates
        # vx, vy: velocities in pixels/second
        self.dim_x = 4  # State dimension
        self.dim_z = 2  # Measurement dimension (x, y positions only)

        # Create sigma points
        points = MerweScaledSigmaPoints(n=self.dim_x, alpha=0.1, beta=2., kappa=1.)
        
        # Create UKF
        self.ukf = UKF(dim_x=self.dim_x, dim_z=self.dim_z, dt=dt,
                       fx=self._state_transition, hx=self._measurement_function,
                       points=points)

        # Initialize state
        self.ukf.x = np.array([0., 0., 0., 0.])  # [x, vx, y, vy]

        # Initialize covariance matrix (high uncertainty initially)
        self.ukf.P = np.eye(self.dim_x) * 500

        # Process noise covariance
        self.ukf.Q = np.eye(self.dim_x) * 0.1
        self.ukf.Q[0, 0] = 0.1  # Position noise x
        self.ukf.Q[1, 1] = 10   # Velocity noise vx
        self.ukf.Q[2, 2] = 0.1  # Position noise y
        self.ukf.Q[3, 3] = 10   # Velocity noise vy

        # Measurement noise covariance
        self.ukf.R = np.eye(self.dim_z) * 5

        self.initialized = False

    def _state_transition(self, x, dt):
        """
        State transition function (constant velocity model with gravity).
        
        Args:
            x (numpy.ndarray): Current state [x, vx, y, vy]
            dt (float): Time step
            
        Returns:
            numpy.ndarray: Predicted next state
        """
        # Constant velocity model: x_new = x + vx*dt
        F = np.array([
            [1, dt, 0,  0],
            [0,  1, 0,  0],
            [0,  0, 1, dt],
            [0,  0, 0,  1]
        ])
        return F @ x

    def _measurement_function(self, x):
        """
        Measurement function: maps state to measurement space.
        We can only measure position (x, y), not velocity.
        
        Args:
            x (numpy.ndarray): State vector [x, vx, y, vy]
            
        Returns:
            numpy.ndarray: Measurement [x, y]
        """
        return np.array([x[0], x[2]])  # Return only positions

    def initialize(self, x, y):
        """
        Initializes the tracker with the first measurement.

        Args:
            x (float): Initial x coordinate
            y (float): Initial y coordinate
        """
        self.ukf.x = np.array([x, 0., y, 0.])  # Initialize with zero velocity
        self.initialized = True

    def update(self, measurement):
        """
        Updates the tracker with a new measurement.

        Args:
            measurement (tuple): The (x, y) coordinates of the detected object.

        Returns:
            tuple: The estimated state (x, vx, y, vy) or None if not initialized.
        """
        if measurement is None:
            # If no measurement, just predict
            if self.initialized:
                self.ukf.predict()
            return None

        x, y = measurement

        if not self.initialized:
            self.initialize(x, y)
            return (x, 0., y, 0.)

        # Predict step
        self.ukf.predict()

        # Update step with measurement
        z = np.array([x, y])
        self.ukf.update(z)

        # Return estimated state
        return tuple(self.ukf.x)

    def get_state(self):
        """
        Gets the current estimated state.

        Returns:
            tuple: The current state (x, vx, y, vy) or None if not initialized.
        """
        if not self.initialized:
            return None
        return tuple(self.ukf.x)

    def get_position(self):
        """
        Gets the current estimated position.

        Returns:
            tuple: The current position (x, y) or None if not initialized.
        """
        if not self.initialized:
            return None
        return (self.ukf.x[0], self.ukf.x[2])

    def get_velocity(self):
        """
        Gets the current estimated velocity.

        Returns:
            tuple: The current velocity (vx, vy) or None if not initialized.
        """
        if not self.initialized:
            return None
        return (self.ukf.x[1], self.ukf.x[3])
