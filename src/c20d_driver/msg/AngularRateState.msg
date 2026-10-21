# Camera IMU body angular rates, NOT joint velocities or general Euler derivatives.
# Degrees/second; pitch positive raises the camera, negative lowers it.
# Roll/yaw retain vendor conventions. This is NOT a REP-103 vector.
std_msgs/Header header
float64 roll_deg_s
float64 pitch_deg_s
float64 yaw_deg_s
