# Topic specifies relative encoder angles or stabilized camera attitude.
# Degrees; pitch positive raises the camera, negative lowers it.
# Roll/yaw retain vendor conventions. This is NOT REP-103 Euler orientation.
# Yaw is telemetry only; the horizontal C20D has no controlled yaw motor.
std_msgs/Header header
float64 roll_deg
float64 pitch_deg
float64 yaw_deg
