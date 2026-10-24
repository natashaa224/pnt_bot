# WallDepthProfile.msg
# Published by wallface_depth_node after each stroke completes.
# Consumed by paint_planner_node to build StripMap for strip N+1.

std_msgs/Header header

int32   strip_index        # strip this profile describes (strip_index + 1 from RobotCommand)

float32 z_min              # lowest Z sample collected (m)
float32 z_max              # highest Z sample collected (m)

float32[] z_positions      # Z height of each sample (m), from encoder
bool[]    paintable        # per-sample paintability decision
float32[] depth_mm         # raw depth reading per sample, for diagnostics/tuning