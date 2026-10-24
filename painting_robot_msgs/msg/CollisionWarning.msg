# Status constants
uint8 CLEAR=0
uint8 WARNING=1
uint8 CRITICAL=2

uint8 status
float32 min_distance   # meters — closest detected obstacle
string source          # "lidar", "realsense", "ultrasonic", etc.
