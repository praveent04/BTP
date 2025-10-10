import cv2
import numpy as np

class IRProcessor:
    """
    Processes IR frames to identify potential "hotspots".
    """

    def __init__(self, threshold_value=220, min_area=100):
        """
        Initializes the IR processor.

        Args:
            threshold_value (int): The threshold value for binary thresholding.
            min_area (int): The minimum area of a contour to be considered a hotspot.
        """
        self.threshold_value = threshold_value
        self.min_area = min_area

    def find_hotspot(self, ir_frame):
        """
        Finds a hotspot in the IR frame.

        Args:
            ir_frame (numpy.ndarray): The IR frame.

        Returns:
            tuple: The center coordinates (x, y) of the hotspot, or None if no hotspot is found.
        """
        if ir_frame is None:
            return None

        gray = cv2.cvtColor(ir_frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, self.threshold_value, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > self.min_area:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    return (cX, cY)
        return None