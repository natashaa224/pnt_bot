#!/usr/bin/env python3
"""
KVMS BMS RS485/UART Driver
Standalone script — mirrors leadshine_driver.py idiom (iso-8859-1 byte-strings,
chr()-packed CRC16-MODBUS, _send_recv with COM_DELAY, _crc_ok).

Bus     : RS485 (USB adapter) — 9600 8N1, no parity.
Protocol: Modbus RTU, big-endian words. Read fn 0x03.
Address : request addr 0x81, response addr 0x51 (device convention — not an echo).
State   : entire real-time map lives in regs 0x00-0x7E, pulled in one block read.

Bench test: run as __main__, prints a decoded snapshot on a loop.
"""

import serial
import time

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
FUNC_READ          = '\x03'
FUNC_WRITE_SINGLE  = '\x06'
FUNC_WRITE_MULTI   = '\x10'

REQ_ADDR           = 0x81          # address byte we send
RESP_ADDR          = 0x51          # address byte device replies with

COM_DELAY          = 0.05          # s between write and read (as leadshine)

# ── Register map (see KVMS protocol sheet) ────────────────────────────────────
REG_CELL_V_START   = 0x00          # 0x00-0x2F : up to 48 cell voltages, x0.001 V
REG_CELL_V_MAX_N   = 48
REG_TEMP_START     = 0x30          # 0x30-0x37 : up to 8 pack temps, offset -40
REG_TEMP_MAX_N     = 8
REG_TOTAL_V        = 0x38          # x0.1 V
REG_CURRENT        = 0x39          # x0.1 A, offset 30000 (>30000 discharge, <charge)
REG_SOC            = 0x3A          # x0.001 -> fraction (800 = 80%)
REG_LIFE           = 0x3B          # heartbeat, increments
REG_CELL_COUNT     = 0x3C
REG_TEMP_COUNT     = 0x3D
REG_MAX_CELL_MV    = 0x3E          # mV
REG_MAX_CELL_IDX   = 0x3F
REG_MIN_CELL_MV    = 0x40          # mV
REG_MIN_CELL_IDX   = 0x41
REG_CELL_DELTA_MV  = 0x42          # mV
REG_MAX_CELL_TEMP  = 0x43          # offset -40
REG_MAX_CTEMP_IDX  = 0x44
REG_MIN_CELL_TEMP  = 0x45          # offset -40
REG_MIN_CTEMP_IDX  = 0x46
REG_CTEMP_DELTA    = 0x47
REG_CHG_DIS_STATUS = 0x48          # 0 idle, 1 charging, 2 discharging
REG_CHARGER_STAT   = 0x49          # 0/1
REG_LOAD_STAT      = 0x4A          # 0/1
REG_REMAIN_CAP     = 0x4B          # x0.1 Ah
REG_CYCLES         = 0x4C
REG_BALANCE_STATE  = 0x4D          # 0 off,1 passive,2 active
REG_CHG_MOS        = 0x52          # 0/1
REG_DISCHG_MOS     = 0x53          # 0/1
REG_PRECHG_MOS     = 0x54          # 0/1
REG_HEAT_MOS       = 0x55          # 0/1
REG_FAN_MOS        = 0x56          # 0/1
REG_AVG_CELL_MV    = 0x57          # mV
REG_POWER          = 0x58          # W
REG_ENERGY         = 0x59          # Wh
REG_MOS_TEMP       = 0x5A          # offset -40
REG_AMBIENT_TEMP   = 0x5B          # offset -40
REG_HEATING_TEMP   = 0x5C          # offset -40
REG_REMAIN_CHG_MIN = 0x64          # min
REG_DIDO_STATUS    = 0x65          # low byte DI1-8, high byte DO1-8
REG_FAULT_START    = 0x6D          # 0x6D-0x73 : 7 fault-code registers
REG_FAULT_N        = 7

REG_BLOCK_START    = 0x00
REG_BLOCK_END      = 0x7E          # inclusive
REG_BLOCK_COUNT    = REG_BLOCK_END - REG_BLOCK_START + 1   # 127 regs

TEMP_OFFSET        = 40
CURRENT_OFFSET     = 30000


# ---------------------------------------------------------------------------
# KvmsBms — one instance per device
# ---------------------------------------------------------------------------
class KvmsBms:

    def __init__(self, ser: serial.Serial,
                 req_addr: int = REQ_ADDR,
                 resp_addr: int = RESP_ADDR):
        self.ser       = ser
        self.req_addr  = chr(req_addr)
        self.resp_addr = resp_addr

    # ------------------------------------------------------------------
    # Private — Modbus primitives (identical CRC to leadshine)
    # ------------------------------------------------------------------
    def _crc16(self, data: str) -> str:
        raw  = bytearray(data, encoding='iso-8859-1')
        poly = 0xA001
        crc  = 0xFFFF
        for b in raw:
            crc ^= (0xFF & b)
            for _ in range(8):
                crc = ((crc >> 1) & 0xFFFF) ^ poly if (crc & 0x0001) else (crc >> 1) & 0xFFFF
        lo = crc & 0xFF
        hi = (crc >> 8) & 0xFF
        return chr(lo) + chr(hi)

    def _to_word(self, val: int) -> str:
        """Pack unsigned int into 2-byte big-endian string."""
        val = val & 0xFFFF
        return chr((val >> 8) & 0xFF) + chr(val & 0xFF)

    def _send_recv(self, frame: str, expected_len: int, timeout_s: float = 0.5) -> str:
        """
        Deadline-based read: flush stale bytes, send, then accumulate until the
        full expected frame arrives or timeout. Fixes truncation + desync on the
        long (259-byte) BMS block reads that COM_DELAY was too short for.
        """
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.write(bytes(frame, 'iso-8859-1'))
        deadline = time.monotonic() + timeout_s
        buf = bytearray()
        while time.monotonic() < deadline:
            n = self.ser.in_waiting
            if n:
                buf.extend(self.ser.read(n))
                if len(buf) >= expected_len:
                    break
            else:
                time.sleep(0.002)
        return buf.decode('iso-8859-1')

    def _crc_ok(self, data: str) -> bool:
        if len(data) < 4:
            return False
        calc = self._crc16(data[:-2])
        return calc == (data[-2] + data[-1])

    # ------------------------------------------------------------------
    # Private — block read
    # ------------------------------------------------------------------
    def read_block(self, start: int, count: int):
        """
        Modbus fn 0x03 block read. Returns list[int] of `count` u16 registers,
        or None on comms/CRC/length failure.
        Response layout: ADDR CMD LEN d0_hi d0_lo ... CRC_L CRC_H
        """
        data  = (self.req_addr + FUNC_READ +
                 self._to_word(start) + self._to_word(count))
        frame = data + self._crc16(data)
        expected_len = 3 + 2 * count + 2      # addr cmd len + data + crc
        resp  = self._send_recv(frame, expected_len)

        if len(resp) < 5:
            print(f"[WARN] short response ({len(resp)} bytes)")
            return None
        if ord(resp[0]) != self.resp_addr:
            print(f"[WARN] addr mismatch: got 0x{ord(resp[0]):02X} "
                  f"expected 0x{self.resp_addr:02X}")
            return None
        if ord(resp[1]) != 0x03:
            print(f"[WARN] func mismatch: got 0x{ord(resp[1]):02X}")
            return None
        length = ord(resp[2])                      # = 2 * count
        if length != 2 * count:
            print(f"[WARN] length mismatch: got {length} expected {2*count}")
            return None
        if len(resp) < 3 + length + 2:
            print(f"[WARN] truncated payload: {len(resp)} < {3+length+2}")
            return None
        if not self._crc_ok(resp[:3 + length + 2]):
            print("[WARN] CRC fail")
            return None

        regs = []
        for i in range(count):
            hi = ord(resp[3 + 2 * i])
            lo = ord(resp[3 + 2 * i + 1])
            regs.append((hi << 8) | lo)
        return regs

    # ------------------------------------------------------------------
    # Public — read entire real-time map, decode
    # ------------------------------------------------------------------
    def read_all(self):
        """
        Single block read 0x00-0x7E, decode into a dict with scaling applied.
        Returns None on failure.
        """
        regs = self.read_block(REG_BLOCK_START, REG_BLOCK_COUNT)
        if regs is None:
            return None

        def r(addr):
            return regs[addr - REG_BLOCK_START]

        def temp(addr):
            return r(addr) - TEMP_OFFSET

        n_cells = r(REG_CELL_COUNT)
        n_temps = r(REG_TEMP_COUNT)
        n_cells = max(0, min(n_cells, REG_CELL_V_MAX_N))
        n_temps = max(0, min(n_temps, REG_TEMP_MAX_N))

        cell_v = [r(REG_CELL_V_START + i) * 0.001 for i in range(n_cells)]
        cell_t = [r(REG_TEMP_START + i) - TEMP_OFFSET for i in range(n_temps)]

        raw_current = r(REG_CURRENT)
        current_a   = (raw_current - CURRENT_OFFSET) * 0.1   # + discharge, - charge

        faults = [r(REG_FAULT_START + i) for i in range(REG_FAULT_N)]

        return {
            # headline
            'pack_voltage'    : r(REG_TOTAL_V) * 0.1,
            'current'         : current_a,
            'soc'             : r(REG_SOC) * 0.001 * 100.0,     # percent
            'remaining_ah'    : r(REG_REMAIN_CAP) * 0.1,
            'power_w'         : r(REG_POWER),
            'energy_wh'       : r(REG_ENERGY),
            'cycles'          : r(REG_CYCLES),
            'life'            : r(REG_LIFE),

            # cells
            'cell_count'      : n_cells,
            'cell_voltages'   : cell_v,
            'avg_cell_mv'     : r(REG_AVG_CELL_MV),
            'max_cell_mv'     : r(REG_MAX_CELL_MV),
            'max_cell_idx'    : r(REG_MAX_CELL_IDX),
            'min_cell_mv'     : r(REG_MIN_CELL_MV),
            'min_cell_idx'    : r(REG_MIN_CELL_IDX),
            'cell_delta_mv'   : r(REG_CELL_DELTA_MV),

            # temps
            'temp_count'      : n_temps,
            'cell_temps'      : cell_t,
            'max_cell_temp'   : temp(REG_MAX_CELL_TEMP),
            'max_ctemp_idx'   : r(REG_MAX_CTEMP_IDX),
            'min_cell_temp'   : temp(REG_MIN_CELL_TEMP),
            'min_ctemp_idx'   : r(REG_MIN_CTEMP_IDX),
            'mos_temp'        : temp(REG_MOS_TEMP),
            'ambient_temp'    : temp(REG_AMBIENT_TEMP),
            'heating_temp'    : temp(REG_HEATING_TEMP),

            # status
            'chg_dis_status'  : r(REG_CHG_DIS_STATUS),   # 0 idle 1 chg 2 dischg
            'charger_present' : r(REG_CHARGER_STAT),
            'load_present'    : r(REG_LOAD_STAT),
            'balance_state'   : r(REG_BALANCE_STATE),
            'chg_mos'         : r(REG_CHG_MOS),
            'dischg_mos'      : r(REG_DISCHG_MOS),
            'prechg_mos'      : r(REG_PRECHG_MOS),
            'heat_mos'        : r(REG_HEAT_MOS),
            'fan_mos'         : r(REG_FAN_MOS),
            'remain_chg_min'  : r(REG_REMAIN_CHG_MIN),
            'dido'            : r(REG_DIDO_STATUS),

            # faults — raw registers 0x6D..0x73 (decode downstream / in node)
            'fault_raw'       : faults,
            'any_fault'       : any(f != 0 for f in faults),
        }


# ---------------------------------------------------------------------------
# Pretty-print helper for bench test
# ---------------------------------------------------------------------------
def print_snapshot(d):
    if d is None:
        print("  <no data>")
        return
    st = {0: 'idle', 1: 'charging', 2: 'discharging'}.get(d['chg_dis_status'], '?')
    print(f"  Pack : {d['pack_voltage']:.1f} V   {d['current']:+.1f} A   "
          f"SOC {d['soc']:.1f}%   [{st}]")
    print(f"  Cap  : {d['remaining_ah']:.1f} Ah   {d['power_w']} W   "
          f"{d['energy_wh']} Wh   cycles {d['cycles']}   life {d['life']}")
    print(f"  Cells: n={d['cell_count']}  avg {d['avg_cell_mv']}mV  "
          f"max {d['max_cell_mv']}mV(#{d['max_cell_idx']})  "
          f"min {d['min_cell_mv']}mV(#{d['min_cell_idx']})  "
          f"Δ {d['cell_delta_mv']}mV")
    print(f"  Temps: max {d['max_cell_temp']}°C  min {d['min_cell_temp']}°C  "
          f"mos {d['mos_temp']}°C  amb {d['ambient_temp']}°C")
    print(f"  MOS  : chg {d['chg_mos']}  dischg {d['dischg_mos']}  "
          f"prechg {d['prechg_mos']}  heat {d['heat_mos']}  fan {d['fan_mos']}")
    print(f"  Fault: any={d['any_fault']}  raw=" +
          " ".join(f"{f:04X}" for f in d['fault_raw']))
    if d['cell_voltages']:
        cv = "  ".join(f"{v:.3f}" for v in d['cell_voltages'])
        print(f"  CellV: {cv}")


# ---------------------------------------------------------------------------
# Bench test — comms check
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    PORT = '/dev/ttyUSB0'    # confirm with `dmesg | grep ttyUSB` when battery connected
    BAUD = 9600

    ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=1,
                        parity=serial.PARITY_NONE,
                        bytesize=serial.EIGHTBITS,
                        stopbits=serial.STOPBITS_ONE)

    bms = KvmsBms(ser)

    print("-- KVMS BMS read loop (Ctrl-C to stop) --")
    try:
        while True:
            snap = bms.read_all()
            print("-" * 60)
            print_snapshot(snap)
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()