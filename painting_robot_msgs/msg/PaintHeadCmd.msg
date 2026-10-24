# PaintHeadCmd.msg
# SBC -> Portenta
# High-level goal state for Z-axis + wrist + trigger assembly
# Portenta firmware handles internal sequencing and coordination
# SBC does not command individual axis moves directly

std_msgs/Header header

# --- Head Mode ---
PaintMotionModes head_mode

# --- Z-Axis ---
float32 z_target_pos_mm     # Target Z position in mm from home

uint8 z_mode

# --- Wrist ---
float32 wrist_target_deg    # Target spray angle in degrees

# --- Trigger ---
float32 trigger_position    # 0.0 = fully released, 1.0 = fully pulled
                            # Portenta thresholds to binary if hardware is on/off only



#-- z modes


uint8 Z_POSITION=0          # Move to z_target_pos_mm
uint8 Z_HOME=2              # Ignore z_target_pos_mm, run homing