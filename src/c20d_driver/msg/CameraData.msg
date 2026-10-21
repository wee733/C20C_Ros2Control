std_msgs/Header header
# Protocol encodes whole degrees (0..127). Zoom in x, quantized to 0.001x.
uint8 vertical_fov_deg
float64 zoom
# radians: horizontal positive right, vertical positive down.
float64 horizontal_offset
float64 vertical_offset
