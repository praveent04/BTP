#!/usr/bin/env python3

"""
test_fusion_engine.py

Unit tests for the Bayesian Fusion Engine.
"""

import unittest
import sys
import os

# Add the source directory to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from threat_detector_node import FusionEngine

class TestFusionEngine(unittest.TestCase):
    """
    Test suite for the FusionEngine class.
    """

    def setUp(self):
        """
        Initialize the FusionEngine before each test.
        """
        self.fusion_engine = FusionEngine()

    def test_no_evidence(self):
        """
        Test the probability with no evidence provided.
        It should return the prior probability of a launch.
        """
        prob = self.fusion_engine.get_launch_probability(ir_detected=False, visual_confirmed=False)
        self.assertLess(prob, 0.01, "Probability should be very low with no evidence.")

    def test_ir_only(self):
        """
        Test the probability with only an IR hotspot detected.
        The probability should increase significantly but may not cross the threshold.
        """
        prob_no_evidence = self.fusion_engine.get_launch_probability(ir_detected=False, visual_confirmed=False)
        prob_ir_only = self.fusion_engine.get_launch_probability(ir_detected=True, visual_confirmed=False)
        self.assertGreater(prob_ir_only, prob_no_evidence, "IR detection should increase launch probability.")

    def test_visual_only(self):
        """
        Test the probability with only a visual confirmation.
        This is unlikely but should still increase the probability slightly.
        """
        prob_no_evidence = self.fusion_engine.get_launch_probability(ir_detected=False, visual_confirmed=False)
        prob_visual_only = self.fusion_engine.get_launch_probability(ir_detected=False, visual_confirmed=True)
        self.assertGreater(prob_visual_only, prob_no_evidence, "Visual confirmation should increase launch probability.")

    def test_both_ir_and_visual(self):
        """
        Test the probability with both IR and visual confirmation.
        The probability of a launch should be very high, likely exceeding the threshold.
        """
        prob = self.fusion_engine.get_launch_probability(ir_detected=True, visual_confirmed=True)
        self.assertGreater(prob, 0.9, "With both IR and visual evidence, probability should be high.")

if __name__ == '__main__':
    unittest.main()