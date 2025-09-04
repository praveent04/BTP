# Custom message for a single radar detection (scan)
std_msgs/Header header

# Unique identifier for the detected target
uint32 target_id

# Target position in spherical coordinates relative to the sensor
float32 range        # meters
float32 azimuth      # radians
float32 elevation    # radians

# Target's radial velocity (Doppler)
float32 doppler_velocity # meters/second