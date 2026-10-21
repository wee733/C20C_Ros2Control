# Full feedback snapshot. Converted pitch is head-up positive, NOT REP-103 ENU.
std_msgs/Header header
uint8 firmware_version_raw
uint8 hardware_error
bool upright
uint8 gimbal_state
bool temperature_ready
uint8 command_status
uint8 command_code
# Original signed wire values and ordering (lossless).
int16[3] camera_rate_raw_pyr
int16[3] camera_attitude_raw_rpy
int16[3] relative_angle_raw_pyr
# Converted values: angles in degrees, rates in degrees/second; roll, pitch, yaw.
# Pitch: relative and rate negate vendor feedback; attitude retains vendor sign.
# Roll/yaw retain vendor conventions. Raw fields above are never sign-converted.
float64[3] camera_rate_rpy
float64[3] camera_attitude_rpy
float64[3] relative_angle_rpy
uint8[26] raw_frame
