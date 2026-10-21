# LOCK true camera angular rate. Refresh before command_timeout; zero holds attitude.
# Zero stamp uses reception time; stale/future commands are rejected.
std_msgs/Header header
# Degrees/second. Positive pitch raises the camera, negative lowers it.
float64 roll_deg_s
float64 pitch_deg_s
