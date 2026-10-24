# BmsStatus.msg
# Published by bms_node @ ~1 Hz. Relevant safety + health subset of the
# full KVMS register map (driver decodes everything; node exposes this).

std_msgs/Header header

bool      valid            # comms alive (life heartbeat register incrementing)
float32   voltage          # pack voltage, V
float32   current          # A, +discharge / -charge
float32   soc              # %
float32   remaining_ah     # remaining capacity, Ah
float32   max_temp         # hottest of cell temps + MOS temp, deg C
uint8     charge_state     # 0 idle, 1 charging, 2 discharging
float32   cell_delta_mv    # max-min cell spread, mV (imbalance indicator)
uint16    cycles           # charge cycle count
bool      any_fault        # rollup: any fault register non-zero
uint16[7] fault_raw        # regs 0x6D-0x73, raw for downstream decode
