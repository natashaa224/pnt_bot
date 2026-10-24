# SafetyCommand.msg
# safety_monitor_node -> FSM.
# Carries the safety TIER and the flush policy the FSM must apply.
# The FSM does NOT hardcode tier->flush mapping; safety_node owns policy and
# sets the flush flags explicitly.

std_msgs/Header header

uint8 tier
uint8 NOMINAL     = 0
uint8 SAFETY_HOLD = 1    # sensor/obstacle hold — auto-resume when clear, else escalate
uint8 FAULT       = 2    # controlled stop, flush queue + gantry limits
uint8 ESTOP       = 3    # critical, flush everything incl limits (relay-drop pending fw)

uint8 source
uint8 SRC_NONE     = 0
uint8 SRC_OBSTACLE = 1   # moving obstacle / transient — SAFETY_HOLD auto-resumes
uint8 SRC_OPERATOR = 2   # joystick / HMI STOP or ESTOP (operator-triggered fault)
uint8 SRC_SENSOR   = 3   # cliff / bms / ultrasound / comms
uint8 SRC_PMC      = 4   # McuStatus / DiStatus fault bits

string reason            # human readable, for log/HMI ("cliff", "battery_low", ...)

bool flush_queue         # FSM: clear motion queue
bool flush_limits        # FSM: clear operator gantry limits (FAULT + ESTOP)
