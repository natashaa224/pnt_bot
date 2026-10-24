# EncoderFeedback.msg
# Portenta -> SBC
# Wheel encoder ticks + velocities + Z-axis position
# Published at 50 Hz — primary input to robot_localization EKF
# SBC portenta_bridge_node converts ticks -> nav_msgs/Odometry for EKF

std_msgs/Header header

# --- Drive Wheels ---
int32 left_ticks            # Cumulative left wheel encoder ticks (signed)
int32 right_ticks           # Cumulative right wheel encoder ticks (signed)
float32 left_velocity_rpm   # Current left wheel velocity in RPM
float32 right_velocity_rpm  # Current right wheel velocity in RPM

# --- Z-Axis ---
float32 z_position_mm       # Current Z position in mm from home
float32 z_velocity_mm_s     # Current Z velocity in mm/s
