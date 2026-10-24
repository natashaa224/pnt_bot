# SafetyMode.msg
# SBC -> Portenta (via bridge).
# Safety and inhibit commands. The bridge maps each mode to a PMC command:
#   PAUSE / FAULT -> CMD_STOP        (motion halt, watchdog keeps relay live)
#   ESTOP         -> CMD_STOP now; CMD_ESTOP relay-drop is a firmware TODO
#   RESET         -> CMD_RESET_FAULT
# QoS: Reliable + Transient Local — Portenta must receive this even on late connect.

uint8 mode
uint8 NOMINAL=0            # normal operation
uint8 SAFETY_HOLD=1       # halt motion, relay stays live, state preserved (sensor hold)
uint8 FAULT=2             # halt motion, relay stays live, controlled stop
uint8 ESTOP=3            # critical — relay-drop intent (pending firmware CMD_ESTOP)
uint8 RESET=4            # clear fault / acknowledge, ready to resume

bool paint_inhibit          # True = no-paint zone active, gate valve/pump/trigger
                            # Motion continues unaffected when only this is set

string reason               # Human readable: "no_paint_zone", "cliff", "estop", "fault"
                            # Used for logging and HMI display
