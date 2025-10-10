import cv2

class CameraManager:
    """
    Manages the connections to the IR and standard optical cameras.
    """

    def __init__(self, ir_camera_index=1, optical_camera_index=0):
        """
        Initializes the camera manager.

        Args:
            ir_camera_index (int): The index of the IR camera.
            optical_camera_index (int): The index of the standard optical camera.
        """
        self.ir_camera_index = ir_camera_index
        self.optical_camera_index = optical_camera_index
        self.ir_cap = None
        self.optical_cap = None

    def initialize_cameras(self):
        """
        Initializes the connections to the cameras.

        Returns:
            bool: True if both cameras are initialized successfully, False otherwise.
        """
        self.ir_cap = cv2.VideoCapture(self.ir_camera_index)
        self.optical_cap = cv2.VideoCapture(self.optical_camera_index)

        if not self.ir_cap.isOpened() or not self.optical_cap.isOpened():
            print("Error: Could not open one or both cameras.")
            return False
        return True

    def get_frames(self):
        """
        Gets a synchronized pair of frames from the cameras.

        Returns:
            tuple: A tuple containing the IR frame and the optical frame.
                   Returns (None, None) if there is an error.
        """
        if not self.ir_cap.isOpened() or not self.optical_cap.isOpened():
            return None, None

        ir_ret, ir_frame = self.ir_cap.read()
        optical_ret, optical_frame = self.optical_cap.read()

        if not ir_ret or not optical_ret:
            print("Error: Could not read frames from one or both cameras.")
            return None, None

        return ir_frame, optical_frame

    def release_cameras(self):
        """
        Releases the connections to the cameras.
        """
        if self.ir_cap:
            self.ir_cap.release()
        if self.optical_cap:
            self.optical_cap.release()
        cv2.destroyAllWindows()
