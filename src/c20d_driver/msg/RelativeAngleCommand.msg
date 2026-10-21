# FPV: both axes follow the carrier; targets are relative to the mounting platform.
# Zero stamp uses reception time; stale/future commands are rejected.
std_msgs/Header header
# Degrees. Pitch positive raises the camera, negative lowers it.
float64 roll_deg
float64 pitch_deg
# Vendor FPV response sensitivity, [-16, 15].
int8 fpv_sensitivity
# One-shot requests: use --once. Each true toggles the vendor center bit.
bool center_roll
bool center_pitch
