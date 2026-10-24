# ZAxisCmd.msg
# Command sent by paint_executor_node to actuator_bridge_node


std_msgs/Header header

# --- Z-Axis ---
float32 z_target_pos_mm     # Target Z position in mm from home
int32 rpm
uint8 z_mode
int32 z_velocity            # signed rpm for Z_VEL mode: +up / -down

#-- z modes
uint8 Z_POSITION=0          # Move to z_target_pos_mm
uint8 Z_HOME=2              # Ignore z_target_pos_mm, run homing
uint8 Z_STOP=3
uint8 Z_VEL=4               # Continuous jog at z_velocity (rpm)