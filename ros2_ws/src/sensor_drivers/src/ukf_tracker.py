#!/usr/bin/env python3

"""
ukf_tracker.py

An Unscented Kalman Filter (UKF) for tracking the real-time trajectory of 
a detected threat.
"""

import numpy as np
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints

class UKFTracker:
    """
    Encapsulates the UKF for trajectory tracking.
    """
    def __init__(self, dt):
        """
        Initializes the UKF with the necessary parameters.

        Args:
            dt: The time step between filter updates.
        """
        # Define the state vector [x, vx, y, vy]
        self.dim_x = 4
        # Define the measurement vector [x, y]
        self.dim_z = 2

        # Create sigma points
        self.points = MerweScaledSigmaPoints(self.dim_x, alpha=0.1, beta=2., kappa=1.)

        # Create the UKF
        self.ukf = UnscentedKalmanFilter(
            dim_x=self.dim_x, 
            dim_z=self.dim_z, 
            dt=dt, 
            hx=self.hx, 
            fx=self.fx, 
            points=self.points
        )

        # Initialize the state
        self.ukf.x = np.zeros(self.dim_x)
        # State covariance matrix
        self.ukf.P *= 10
        # Process noise covariance matrix
        self.ukf.Q = np.diag([0.1, 1.0, 0.1, 1.0])
        # Measurement noise covariance matrix
        self.ukf.R = np.diag([5, 5])

    def fx(self, x, dt):
        """
        State transition function.
        """
        F = np.array([
            [1, dt, 0,  0],
            [0,  1, 0,  0],
            [0,  0, 1, dt],
            [0,  0, 0,  1]
        ])
        return F @ x

    def hx(self, x):
        """
        Measurement function.
        """
        return x[[0, 2]]

    def update(self, z):
        """
        Updates the filter with a new measurement.
        """
        self.ukf.predict()
        self.ukf.update(z)
        return self.ukf.x