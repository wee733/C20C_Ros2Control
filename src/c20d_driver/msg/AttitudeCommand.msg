# LOCK: both axes stabilize camera attitude, using the vendor attitude reference.
# This is not REP-103 Euler orientation. Zero stamp uses reception time.
std_msgs/Header header
# Degrees. Pitch positive raises the camera, negative lowers it.
float64 roll_deg
float64 pitch_deg
# One-shot requests: use --once. Each true toggles the vendor center bit.
bool center_roll
bool center_pitch
