# EnvStatus.msg
# Published continuously by env_monitoring_node at 10 Hz.
# Carries corner awareness state only — collision is handled separately


std_msgs/Header header

# ── Corner state constants ────────────────────────────────────────────────────
uint8 CORNER_CLEAR       = 0   # no corner detected or approaching
uint8 CORNER_APPROACHING = 1   # both walls visible, front wall within approach distance
uint8 CORNER_PREEMPT     = 2   # front wall within standoff threshold — stop and turn now
uint8 CORNER_ABORT=3          # NEW —   wall lost → STOP

# ── Corner type constants ─────────────────────────────────────────────────────
uint8 INTERNAL = 0   # concave / room corner — turn towards wall
uint8 EXTERNAL = 1   # convex / pillar corner — reserved for future use
uint8 BUMPOUT  = 2
uint8 NONE     = 255


# ── Fields ───────────────────────────────────────────────────────────────────
uint8   corner_state         # one of CORNER_CLEAR / APPROACHING / PREEMPT
uint8   corner_type          # one of INTERNAL / EXTERNAL (valid when not CLEAR)
float32 front_wall_distance  # current front wall distance in metres (0.0 if not detected)
bool sprayhead_past_vertex   # true once spray head clears the vertex — stop approach paint
uint8 latched_corner_type
float32 corner_depth   # latched bump depth (m); 0.0 when N/A

float32 corner_turn_radius
float32 corner_radian_diff
float32 corner_angle_deg
string  corner_turn_dir
float32 corner_vertex_x
float32 corner_vertex_y
float32 crawl_remaining      # distance (m) to crawl in CORNER_APPROACH; -1.0 when not approaching
float32 crawl_backup         # distance (m) to REVERSE before an INTERNAL turn (robot already inside the trigger distance); 0.0 when none
