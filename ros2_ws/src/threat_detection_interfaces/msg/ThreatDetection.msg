# Output message from the Stage 1 Fusion Engine
std_msgs/Header header

# True if a threat is detected with high confidence
bool is_detected

# The posterior probability of a launch event from the Bayesian model (0.0 to 1.0)
float32 confidence

# Initial estimated position of the threat from sensor data
# For now, we can use the radar's initial report
float32 range        # meters
float32 azimuth      # radians
float32 elevation    # radians
