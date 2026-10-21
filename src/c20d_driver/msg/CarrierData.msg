# Explicit vendor convention; do not connect a generic ROS IMU without conversion.
std_msgs/Header header
bool valid
# roll/pitch/yaw radians, vendor carrier coordinates; roll/yaw [-pi,pi), pitch [-pi/2,pi/2].
float64[3] attitude_rpy
# Earth north/east/up acceleration in m/s^2, per vendor protocol (not raw accelerometer).
float64[3] acceleration_neu
