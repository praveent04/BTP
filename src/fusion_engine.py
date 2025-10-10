"""
Bayesian Fusion Engine for Launch Event Detection
Uses a Bayesian Belief Network to fuse IR and Optical sensor evidence
"""

from pgmpy.models import DiscreteBayesianNetwork
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination


class FusionEngine:
    """
    Implements a Bayesian Belief Network to fuse evidence from IR and optical sensors.
    """

    def __init__(self):
        """
        Initializes the Bayesian Belief Network.
        """
        # Define the network structure
        # Launch_Event is influenced by IR_Hotspot_Detected and Visual_Confirmation
        self.model = DiscreteBayesianNetwork([
            ('IR_Hotspot_Detected', 'Launch_Event'),
            ('Visual_Confirmation', 'Launch_Event')
        ])

        # Define Conditional Probability Tables (CPTs)
        
        # P(IR_Hotspot_Detected) - Prior probability
        cpd_ir = TabularCPD(
            variable='IR_Hotspot_Detected',
            variable_card=2,
            values=[[0.95], [0.05]]  # [False, True]
        )

        # P(Visual_Confirmation) - Prior probability
        cpd_visual = TabularCPD(
            variable='Visual_Confirmation',
            variable_card=2,
            values=[[0.98], [0.02]]  # [False, True]
        )

        # P(Launch_Event | IR_Hotspot_Detected, Visual_Confirmation)
        # Values represent: P(Launch_Event | IR, Visual)
        # Column order: [IR=F,V=F], [IR=F,V=T], [IR=T,V=F], [IR=T,V=T]
        cpd_launch = TabularCPD(
            variable='Launch_Event',
            variable_card=2,
            values=[
                [0.9999, 0.95, 0.90, 0.01],  # P(Launch=False | IR, Visual)
                [0.0001, 0.05, 0.10, 0.99]   # P(Launch=True | IR, Visual)
            ],
            evidence=['IR_Hotspot_Detected', 'Visual_Confirmation'],
            evidence_card=[2, 2]
        )

        # Add CPDs to the model
        self.model.add_cpds(cpd_ir, cpd_visual, cpd_launch)

        # Verify the model
        assert self.model.check_model(), "BN model is not valid!"

        # Create inference engine
        self.inference = VariableElimination(self.model)

    def calculate_launch_probability(self, ir_detected, visual_confirmed):
        """
        Calculates the posterior probability of a launch event given the evidence.

        Args:
            ir_detected (bool): Whether an IR hotspot was detected.
            visual_confirmed (bool): Whether visual confirmation was obtained.

        Returns:
            float: The probability of a launch event (0.0 to 1.0).
        """
        # Convert boolean to integer (0=False, 1=True)
        ir_state = 1 if ir_detected else 0
        visual_state = 1 if visual_confirmed else 0

        # Perform inference
        result = self.inference.query(
            variables=['Launch_Event'],
            evidence={
                'IR_Hotspot_Detected': ir_state,
                'Visual_Confirmation': visual_state
            }
        )

        # Return probability of Launch_Event = True
        return result.values[1]

    def is_launch_detected(self, ir_detected, visual_confirmed, threshold=0.95):
        """
        Determines if a launch event is detected based on the confidence threshold.

        Args:
            ir_detected (bool): Whether an IR hotspot was detected.
            visual_confirmed (bool): Whether visual confirmation was obtained.
            threshold (float): Confidence threshold for declaring a launch (default: 0.95).

        Returns:
            tuple: (bool, float) - (Is launch detected, confidence score)
        """
        confidence = self.calculate_launch_probability(ir_detected, visual_confirmed)
        return confidence >= threshold, confidence
