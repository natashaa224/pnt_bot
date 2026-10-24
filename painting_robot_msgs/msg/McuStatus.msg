# McuStatus.msg
# Portenta -> SBC
# Overall MCU health, fault reporting, and safety state
# Published at 10 Hz

std_msgs/Header header

# --- MCU State ---
uint8 mcu_state
uint8 STATE_IDLE=0
uint8 STATE_RUNNING=1
uint8 STATE_FAULT=2
uint8 STATE_ESTOP=3

# --- Fault Reporting ---
# fault_code:     bitmask of active faults
# fault_severity: bitmask of severity per fault bit
#                 0 = WARNING  (log, notify, continue operation)
#                 1 = CRITICAL (halt motors and/or paint immediately)
#
# Bit assignments:
#   0  FAULT_DRIVE_LEFT      Left drive motor / ELD2 driver fault       CRITICAL
#   1  FAULT_DRIVE_RIGHT     Right drive motor / ELD2 driver fault      CRITICAL
#   2  FAULT_ZAXIS           Z-axis motor / ELD2 driver fault           CRITICAL
#   3  FAULT_SERVO_BUS       Wrist or trigger servo bus fault           WARNING
#   4  FAULT_ESTOP           E-stop activated (dual channel)            CRITICAL
#   5  FAULT_SAFETY_RELAY    Safety relay dropped unexpectedly          CRITICAL
#   6  FAULT_LIMIT_UPPER     Z upper limit hit outside homing           CRITICAL
#   7  FAULT_LIMIT_LOWER     Z lower limit hit outside homing           CRITICAL
#   8  FAULT_ULTRASONIC      One or more ultrasonic sensors not responding WARNING
#   9  FAULT_PAINT_LOW       Paint level below threshold                WARNING
#   10 FAULT_PRESSURE        Pressure switch fault / out of range       WARNING
#   11 FAULT_TEMPERATURE     Over temp — severity escalates dynamically WARNING->CRITICAL
#   12 FAULT_HEARTBEAT       SBC heartbeat timeout exceeded             CRITICAL
#   13-15 reserved

uint16 fault_code           # Active faults bitmask
uint16 fault_severity       # Severity per bit: 0=WARNING, 1=CRITICAL

# --- System Vitals ---
float32 battery_voltage     # Battery voltage in V
float32 temperature_c       # MCU/system temperature in Celsius

# --- Safety Hardware State ---
bool watchdog_ok            # True = safety relay watchdog pulse active
bool safety_relay_ok        # True = safety relay energized and healthy
bool estop_active           # True = e-stop currently pressed
