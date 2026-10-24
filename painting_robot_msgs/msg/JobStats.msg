# JobStats.msg
# Published by logging_node @ ~1 Hz.
# One message carries both HMI tiers:
#   - live block   -> main HMI screen (current run)
#   - battery block -> main screen (inert until BMS node exists)
#   - lifetime block -> maintenance tab (persistent counters)

std_msgs/Header header

# ── Live / current run (main HMI screen) ─────────────────────────────
bool     run_active
uint32   run_number
float32  runtime_s              # current run elapsed seconds
uint32   strokes                # paint strokes this run
float32  coverage_m2            # painted area this run
float32  paint_consumed         # 0-1 paint_level delta this run (0 until sensor)
float32  dist_error_mean        # mean |distance_error| this run
float32  angle_error_mean       # mean |angle_error| this run
float32  strip_z_min            # run vertical envelope, m from home
float32  strip_z_max
string   fsm_state              # current FSM state name (display mirror)

# ── Battery (main screen — populated once BMS node publishes) ─────────
float32  battery_soc            # %   (battery_valid=false until BMS node up)
float32  battery_voltage        # V
bool     battery_valid

# ── Persistent / lifetime (maintenance tab) ──────────────────────────
uint32   total_runs
uint32   total_strokes
uint32   tip_cycles_total
uint32   strokes_since_tip_change    # HMI compares to threshold -> wear alert (item 19)
uint32   runs_since_clean            # HMI compares to threshold -> clean reminder (item 20)
float32  total_runtime_hr
float32  total_coverage_m2
