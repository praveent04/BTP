import cv2
import numpy as np

class OpticalProcessor:
    """
    Processes optical frames to detect motion.
    """

    def __init__(self, bg_subtractor_type='MOG2'):
        """
        Initializes the optical processor.

        Args:
            bg_subtractor_type (str): The type of background subtractor to use ('MOG2' or 'KNN').
        """
        if bg_subtractor_type == 'MOG2':
            self.backSub = cv2.createBackgroundSubtractorMOG2()
        else:
            self.backSub = cv2.createBackgroundSubtractorKNN()
        self.roi_size = 100  # Size of the region of interest around the hotspot

    def detect_motion(self, optical_frame, hotspot_coords):
        """
        Detects motion in a region of interest (ROI) in the optical frame.

        Args:
            optical_frame (numpy.ndarray): The optical frame.
            hotspot_coords (tuple): The (x, y) coordinates of the IR hotspot.

        Returns:
            bool: True if significant motion is detected in the ROI, False otherwise.
        """
        if optical_frame is None or hotspot_coords is None:
            return False

        x, y = hotspot_coords
        h, w, _ = optical_frame.shape

        # Define the ROI around the hotspot
        half_roi = self.roi_size // 2
        x_start = max(0, x - half_roi)
        y_start = max(0, y - half_roi)
        x_end = min(w, x + half_roi)
        y_end = min(h, y + half_roi)

        roi = optical_frame[y_start:y_end, x_start:x_end]

        if roi.size == 0:
            return False
            
        fg_mask = self.backSub.apply(roi)

        # Check for significant motion in the foreground mask
        motion_detected = np.sum(fg_mask > 200) > 50  # More than 50 white pixels
        
        return motion_detected