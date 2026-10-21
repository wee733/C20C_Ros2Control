# Explicit vendor convention; do not connect a generic ROS IMU without conversion.
std_msgs/Header header
bool valid
# roll/pitch/yaw degrees, vendor carrier coordinates; roll/yaw [-180,180), pitch [-90,90].
float64[3] attitude_rpy
# Earth north/east/up acceleration in m/s^2, per vendor protocol (not raw accelerometer).
float64[3] acceleration_neu
