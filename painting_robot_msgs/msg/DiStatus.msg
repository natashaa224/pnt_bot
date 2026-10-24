# DiStatus.msg
# Portenta -> SBC (via bridge).
# Logical digital-input states, decoded by the bridge from di_raw using
# pmc_io_map.yaml. Field names are LOGICAL SIGNALS, not raw channels — the
# bridge owns the bit/polarity mapping so these names stay stable across
# PMC rewiring. Disabled signals (per the yaml) are always False.
# Published at 20 Hz (source: PMC STATUS_DI 0x83).

std_msgs/Header header

uint8 di_raw             # raw digital input byte, for diagnostics

# ── Safety-relevant (observed; hardware EMG chain owns the actual trip) ───────
bool estop               # dual-channel E-stop asserted
bool emg_limits          # combined EMG limit chain asserted
bool ceiling_detect      # ceiling proximity (ESTOP-class) — disabled until wired
bool upper_limit         # Z upper limit — disabled until wired
bool lower_limit         # Z lower limit
bool relay_status        # safety relay health (True = healthy)
bool paint_level_ok      # True = paint above threshold (False = low, FAULT-class)

# ── Non-safety (FSM / mode / master power own these) ──────────────────────────
bool pause_resume        # operator pause/resume (FSM job control)
bool auto_manual         # mode select (True = auto)
bool on_off              # master on/off
