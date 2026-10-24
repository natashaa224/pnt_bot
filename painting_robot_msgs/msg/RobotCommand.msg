# RobotCommand.msg
# FSM → actuator nodes
# Published on every action-state transition.
# command_id increments on each new action — consumers use this
# to trigger once per command, not once per state message tick.

string state          # current FSM state name
uint32 command_id     # monotonically increasing, new value = new action
int32  strip_index  # current strip being painted; wallfacing depth node scans strip_index + 1
string phase  # for within state phases