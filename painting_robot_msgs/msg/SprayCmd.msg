# SprayCmd.msg
# SBC -> Portenta
# Paint pump and solenoid valve control
# paint_inhibit in SafetyMode overrides this immediately if active

std_msgs/Header header

bool pump_enable            # Enable airless paint pump via relay/inverter
bool valve_open             # Open solenoid valve
float32 flow_setpoint       # Normalized flow setpoint 0.0-1.0 (future PWM use)
