# Zero stamp means use reception time. Nonzero stamps older than command_timeout are rejected.
std_msgs/Header header
uint8 LOCK=1
uint8 FPV=2
uint8 ANGLE=0
uint8 PROPORTIONAL_RATE=1
uint8 RATE=2
# Horizontal C20D: roll and pitch only. Follow mode is prohibited by the manual.
# FPV must be applied to BOTH axes and supports ANGLE only.
uint8 roll_mode 1
uint8 pitch_mode 1
uint8 roll_type 0
uint8 pitch_type 0
# Degrees (ANGLE), degrees/second (RATE/PROPORTIONAL_RATE).
# LOCK angle = camera attitude; FPV angle = camera relative angle.
# FPV pitch uses fpv_pitch_command_sign=-1.0, verified on upright firmware raw 36.
# Set that parameter to +1.0 for raw wire sign. LOCK and rate signs remain native.
float64 roll
float64 pitch
int8 fpv_sensitivity
# A true request toggles the respective protocol center bit once for this received message.
bool center_roll
bool center_pitch
