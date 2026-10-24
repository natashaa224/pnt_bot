# StripMap.msg
# Describes the vertical paintability map for one strip.
# paint_planner_node publishes this.
# paint_executor_node consumes this.

# Physical stroke limits — executor always moves within these bounds
float32 strip_z_min       # metres from home (bottom)
float32 strip_z_max       # metres from home (top)

# Wrist flick permission at physical ends
# False when that end borders a fixture (no flick onto fixture edge)
bool flick_at_min         # allow wrist flick at strip_z_min
bool flick_at_max         # allow wrist flick at strip_z_max

# Zone arrays — parallel, index-matched, contiguous, cover full span
# z_start[0] == strip_z_min, z_end[-1] == strip_z_max
float32[] z_start
float32[] z_end
bool[]    paint_on
