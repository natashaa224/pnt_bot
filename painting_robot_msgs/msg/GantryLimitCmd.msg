# GantryLimitCmd.msg
# Sent by HMI / joy_teleop_node to actuator_bridge_node.
#
# No physical Z limit switches exist on this machine — this message IS how
# homing happens. After /z_axis_cmd Z_HOME arms teach mode, the operator
# jogs the gantry and sends this to record the travel range:
#   set_lower (joystick A) — first press (pre-homing) defines the Z zero
#                             datum at the current position; later presses
#                             just re-teach the soft lower bound.
#   set_upper (joystick Y) — first press (pre-homing, requires set_lower
#                             already sent) completes homing; later presses
#                             just re-teach the soft upper bound.

std_msgs/Header header
bool set_lower     # capture current Z as soft lower limit (zero on first teach)
bool set_upper     # capture current Z as soft upper limit (completes homing on first teach)