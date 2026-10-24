#!/usr/bin/env python3
"""
wrist_driver.py

Feetech / Waveshare ST/STS series serial servo driver.
Mirrors leadshine_driver.py structure — standalone class, no ROS dependency.
Intended to be imported by actuator_bridge_node.py SBC backend.

Hardware:
    Port  : TBD — likely /dev/ttyACM0 or /dev/ttyUSB1 via USB hub
    Baud  : 1,000,000 (1 Mbaud)
    Servo : ST/STS series, single servo on wrist joint

Packet format (Feetech protocol):
    0xFF 0xFF | ID | LENGTH | INSTRUCTION | PARAMS... | CHECKSUM
    Checksum = (~sum(ID + LENGTH + INSTRUCTION + PARAMS)) & 0xFF

Position calibration (TBD during homing):
    SERVO_CENTER_POS  — raw position unit at 0° (neutral)
    SERVO_COUNTS_PER_DEG — raw units per degree
    Typical ST series: 0–4095 over 0–360°, ~11.375 counts/deg
"""

import serial
import time

# ---------------------------------------------------------------------------
# Constants — adjust during bench calibration
# ---------------------------------------------------------------------------
SERVO_ID            = 1        # TBD — confirm with hardware
PORT                = '/dev/ttyACM3'   # TBD — confirm via USB hub enumeration
BAUD                = 1_000_000

SERVO_CENTER_POS    = 852     # TBD — raw position unit at wrist neutral (0°)
SERVO_COUNTS_PER_DEG = 11.375  # TBD — typical ST series (4096 counts / 360°)
SERVO_MIN_POS       = 0     # TBD — hard lower limit (software clamp)
SERVO_MAX_POS       = 4095     # TBD — hard upper limit (software clamp)

DEFAULT_SPEED       = 600      # raw speed units
DEFAULT_ACCEL       = 60       # raw acceleration units

# Feetech instruction set
INST_WRITE          = 0x03
INST_READ           = 0x02

# Register addresses (ST/STS series)
REG_GOAL_ACC        = 0x29     # 41 dec — acceleration
REG_GOAL_POS        = 0x2A     # 42 dec — goal position (2 bytes)
REG_GOAL_TIME       = 0x2C     # 44 dec — movement time (2 bytes, set 0 = speed mode)
REG_GOAL_SPEED      = 0x2E     # 46 dec — goal speed (2 bytes)
REG_PRESENT_POS     = 0x38     # 56 dec — current position (2 bytes)
REG_PRESENT_SPEED   = 0x3A     # 58 dec — current speed (2 bytes)
REG_TORQUE_ENABLE   = 0x28     # 40 dec — torque enable

COM_DELAY           = 0.05     # seconds — read wait after write


# ---------------------------------------------------------------------------
# WristServo
# ---------------------------------------------------------------------------
class WristServo:

    def __init__(self, ser: serial.Serial,
                 servo_id: int          = SERVO_ID,
                 center_pos: int        = SERVO_CENTER_POS,
                 counts_per_deg: float  = SERVO_COUNTS_PER_DEG,
                 min_pos: int           = SERVO_MIN_POS,
                 max_pos: int           = SERVO_MAX_POS,
                 default_speed: int     = DEFAULT_SPEED,
                 default_accel: int     = DEFAULT_ACCEL):
        self.ser            = ser
        self.servo_id       = servo_id
        self.center_pos     = center_pos
        self.counts_per_deg = counts_per_deg
        self.min_pos        = min_pos
        self.max_pos        = max_pos
        self.default_speed  = default_speed
        self.default_accel  = default_accel

    # ------------------------------------------------------------------
    # Private — packet helpers
    # ------------------------------------------------------------------
    def _checksum(self, data: list) -> int:
        return (~sum(data)) & 0xFF

    def _build_write_packet(self, register: int, params: list) -> bytes:
        """Build a write packet for given register and parameter bytes."""
        length = len(params) + 3   # instruction + register + params + checksum
        body   = [self.servo_id, length, INST_WRITE, register] + params
        cs     = self._checksum(body)
        return bytes([0xFF, 0xFF] + body + [cs])

    def _build_read_packet(self, register: int, num_bytes: int) -> bytes:
        body = [self.servo_id, 0x04, INST_READ, register, num_bytes]
        cs   = self._checksum(body)
        return bytes([0xFF, 0xFF] + body + [cs])

    def _send_recv(self, packet: bytes, read_bytes: int = 32) -> bytes:
        self.ser.reset_input_buffer()
        self.ser.write(packet)
        time.sleep(COM_DELAY)
        return self.ser.read(read_bytes)

    def _send_fire_forget(self, packet: bytes):
        """Write only — no response expected (e.g. move commands)."""
        self.ser.reset_input_buffer()
        self.ser.write(packet)

    # ------------------------------------------------------------------
    # Private — unit conversion
    # ------------------------------------------------------------------
    def _deg_to_pos(self, angle_deg: float) -> int:
        raw = int(self.center_pos + angle_deg * self.counts_per_deg)
        return max(self.min_pos, min(self.max_pos, raw))

    def _pos_to_deg(self, pos: int) -> float:
        return (pos - self.center_pos) / self.counts_per_deg

    # ------------------------------------------------------------------
    # Public — motion commands
    # ------------------------------------------------------------------
    def move_to_deg(self, angle_deg: float,
                    speed: int = None,
                    accel: int = None) -> bool:
        """
        Move wrist to angle_deg relative to neutral (0°).
        Positive = up, negative = down (TBD — confirm with hardware).
        """
        speed = speed if speed is not None else self.default_speed
        accel = accel if accel is not None else self.default_accel
        pos   = self._deg_to_pos(angle_deg)

        params = [
            accel & 0xFF,
            pos   & 0xFF, (pos   >> 8) & 0xFF,
            0x00,  0x00,                          # time = 0 (speed mode)
            speed & 0xFF, (speed >> 8) & 0xFF,
        ]
        packet = self._build_write_packet(REG_GOAL_ACC, params)
        self._send_fire_forget(packet)
        return True

    def move_to_raw(self, pos: int,
                    speed: int = None,
                    accel: int = None) -> bool:
        """Move to raw position units — for calibration use."""
        speed = speed if speed is not None else self.default_speed
        accel = accel if accel is not None else self.default_accel
        pos   = max(self.min_pos, min(self.max_pos, pos))

        params = [
            accel & 0xFF,
            pos   & 0xFF, (pos   >> 8) & 0xFF,
            0x00,  0x00,
            speed & 0xFF, (speed >> 8) & 0xFF,
        ]
        packet = self._build_write_packet(REG_GOAL_ACC, params)
        self._send_fire_forget(packet)
        return True

    def set_neutral(self):
        """Move to center position — call during homing."""
        return self.move_to_raw(self.center_pos)

    def torque_enable(self, on: bool):
        val    = [0x01 if on else 0x00]
        packet = self._build_write_packet(REG_TORQUE_ENABLE, val)
        self._send_fire_forget(packet)

    # ------------------------------------------------------------------
    # Public — feedback
    # ------------------------------------------------------------------
    def read_position_deg(self) -> float:
        """Returns current position in degrees relative to neutral."""
        packet = self._build_read_packet(REG_PRESENT_POS, 2)
        resp   = self._send_recv(packet)
        try:
            if len(resp) >= 7:
                pos = resp[5] | (resp[6] << 8)
                return self._pos_to_deg(pos)
        except Exception as e:
            print(f'[WARN] wrist read_position: {e}')
        return 0.0

    def read_position_raw(self) -> int:
        """Returns raw position units — for calibration."""
        packet = self._build_read_packet(REG_PRESENT_POS, 2)
        resp   = self._send_recv(packet)
        try:
            if len(resp) >= 7:
                return resp[5] | (resp[6] << 8)
        except Exception as e:
            print(f'[WARN] wrist read_position_raw: {e}')
        return 0


# ---------------------------------------------------------------------------
# Bench test
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    PORT_TEST = '/dev/ttyACM3'   # TBD
    BAUD_TEST = 1_000_000

    ser = serial.Serial(PORT_TEST, BAUD_TEST, timeout=0.1)
    wrist = WristServo(ser)

    print('Wrist servo test — confirm neutral/center before running')
    print(f'Center pos: {SERVO_CENTER_POS}  Speed: {DEFAULT_SPEED}')

    # First find true center:
    # wrist.torque_enable(False)
    # # manually move to neutral position
    # input("Move to neutral, press Enter")
    # wrist.torque_enable(True)
    # print("Raw center:", wrist.read_position_raw())

    try:
        print('Torque ON')
        wrist.torque_enable(True)
        time.sleep(0.5)

        print('Move to neutral (0°)')
        wrist.set_neutral()
        time.sleep(2)
        print(f'Position: {wrist.read_position_deg():.1f}°  raw: {wrist.read_position_raw()}')

        print('Move to +25°')
        wrist.move_to_deg(25.0)
        time.sleep(2)
        print(f'Position: {wrist.read_position_deg():.1f}°  raw: {wrist.read_position_raw()}')

        print('Move to -25°')
        wrist.move_to_deg(-25.0)
        time.sleep(2)
        print(f'Position: {wrist.read_position_deg():.1f}°  raw: {wrist.read_position_raw()}')

        print('Return to neutral')
        wrist.set_neutral()
        time.sleep(2)

    except KeyboardInterrupt:
        print('\nStopped')
    finally:
        wrist.torque_enable(False)
        ser.close()
