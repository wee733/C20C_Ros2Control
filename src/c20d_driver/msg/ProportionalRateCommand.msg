# LOCK proportional rate. Requires fresh nonzero CameraData FOV and zoom.
# Vendor scales these base rates by its FOV/zoom coefficient: NOT actual target rates.
# Refresh before command_timeout; zero holds attitude. Zero stamp uses reception time.
std_msgs/Header header
# Base degrees/second. Positive pitch raises the camera, negative lowers it.
float64 roll_deg_s
float64 pitch_deg_s
