#!/usr/bin/env python3

"""
test_ukf_tracker.py

Unit tests for the Unscented Kalman Filter tracker.
"""

import unittest
import numpy as np
import sys
import os

# Add the source directory to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from ukf_tracker import UKFTracker

class TestUKFTracker(unittest.TestCase):
    """
    Test suite for the UKFTracker class.
    """

    def setUp(self):
        """
        Initialize the UKFTracker before each test.
        """
        self.tracker = UKFTracker(dt=0.1)

    def test_initialization(self):
        """
        Test that the UKF is initialized with the correct dimensions.
        """
        self.assertEqual(self.tracker.ukf.dim_x, 4, "State dimension should be 4.")
        self.assertEqual(self.tracker.ukf.dim_z, 2, "Measurement dimension should be 2.")

    def test_update(self):
        """
        Test the update function with a series of noisy measurements.
        """
        # True initial position and velocity
        true_x, true_y = 100, 200
        true_vx, true_vy = 10, 5

        # Generate a series of noisy measurements
        measurements = []
        for i in range(10):
            true_x += true_vx * 0.1
            true_y += true_vy * 0.1
            noise = np.random.normal(0, 5, size=2)
            measurements.append(np.array([true_x, true_y]) + noise)

        # Update the tracker
        for z in measurements:
            self.tracker.update(z)

        # The final state should be closer to the true state than the last measurement
        final_state = self.tracker.ukf.x
        final_measurement = measurements[-1]

        true_final_pos = np.array([true_x, true_y])
        
        error_state = np.linalg.norm(final_state[[0, 2]] - true_final_pos)
        error_measurement = np.linalg.norm(final_measurement - true_final_pos)

        self.assertLess(error_state, error_measurement, "UKF should reduce measurement noise.")

if __name__ == '__main__':
    unittest.main()