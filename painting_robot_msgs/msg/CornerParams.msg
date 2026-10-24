std_msgs/Header header
string corner_type      # INTERNAL / EXTERNAL / EXTERNAL_EDGE
float32 radian_diff     # signed: + CCW, - CW
float32 angle_deg       # absolute turn angle in degrees
float32 distance        # front_wall_distance at preempt (m)
float32 turn_radius     # arc radius of the turn for internal cornrs 


# Bump-out second arc (ARC2 / BACK). Only populated when corner_type == 'BUMPOUT'.
# ARC1 reuses radian_diff (+phi) and turn_radius (R1); these carry (-phi, R2).
float32 arc2_radian_diff
float32 arc2_turn_radius