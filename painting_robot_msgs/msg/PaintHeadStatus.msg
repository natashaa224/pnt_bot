# PaintHeadStatus.msg
# Portenta -> SBC
# Actual state of Z-axis, wrist, and paint hardware
# Published at 10 Hz
# Used by FSM to confirm head_mode transitions and detect faults

std_msgs/Header header

# --- Paint Hardware ---
float32 paint_level         # Normalized 0.0-1.0 from paint level sensor

# --- Z-Axis ---
# No physical limit switches exist on this machine — limit_lower_z instead
# carries the manual teach-homing "homed" signal (see GantryLimitCmd.msg).
# limit_upper_z has no hardware source and is always false.
bool limit_upper_z          # True = upper Z limit switch triggered
bool limit_lower_z          # True = lower Z limit switch triggered / homed
float32 z_actual_pos_mm     # Actual Z position in mm from home
float32 z_soft_lower_mm     # Taught soft lower Z limit, mm from home (valid once homed)
float32 z_soft_upper_mm     # Taught soft upper Z limit, mm from home (valid once homed)

# --- Wrist ---
float32 wrist_actual_deg    # Actual wrist angle in degrees

# --- Head State ---
PaintMotionModes head_mode

