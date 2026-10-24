#!/usr/bin/env python3
"""
Leadshine ELD2 Motor Driver
Standalone script — replicates test code in a clean class structure.
Port: single RS485 bus, all motors daisy-chained.
"""

import serial
import time

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
FUNC_READ           = '\x03'
FUNC_WRITE_SINGLE   = '\x06'
FUNC_WRITE_MULTI    = '\x10'

REG_PR_COMMAND      = '\x62\x00'   # Pr9.00 — PR immediate trigger (0x6200)
REG_ENCODER         = '\x60\x2C'   # actual encoder position (0x602C)
REG_ALARM           = '\x0B\x03'   # alarm register

PR_MODE_VELOCITY    = '\x00\x02'
PR_MODE_ABS         = '\x00\x01'
PR_MODE_REL         = '\x00\x41'

DEFAULT_ACCEL       = 100           # ms / 1000 rpm
DEFAULT_DECEL       = 100
DEFAULT_ACCEL_ZAxis = 150
DEFAULT_DECEL_ZAxis = 100
COM_DELAY           = 0.05          # seconds between write and read


# ---------------------------------------------------------------------------
# LeadshineMotor — one instance per slave
# ---------------------------------------------------------------------------
class LeadshineMotor:

    def __init__(self, ser: serial.Serial, slave_id: int,
                 accel: int = DEFAULT_ACCEL,
                 decel: int = DEFAULT_DECEL,
                 counts_per_rev: int = 10000):
        self.ser            = ser
        self.slave          = chr(slave_id)
        self.accel          = accel
        self.decel          = decel
        self.counts_per_rev = counts_per_rev
        print(f"[LeadshineMotor] slave={slave_id} accel={self.accel} decel={self.decel} cpr={self.counts_per_rev}")

    # ------------------------------------------------------------------
    # Private — Modbus primitives
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
        """Pack signed int into 2-byte big-endian string."""
        val = ((val + (1 << 16)) % (1 << 16))
        return chr((val >> 8) & 0xFF) + chr(val & 0xFF)

    def _to_dword(self, val: int) -> str:
        """Pack signed int into 4-byte big-endian string (32-bit two's complement)."""
        val = ((val + (1 << 32)) % (1 << 32))
        return (chr((val >> 24) & 0xFF) + chr((val >> 16) & 0xFF) +
                chr((val >>  8) & 0xFF) + chr( val        & 0xFF))

    def _send_recv(self, frame: str) -> str:
        self.ser.flush()
        self.ser.write(bytes(frame, 'iso-8859-1'))
        time.sleep(COM_DELAY)
        out = ''
        try:
            while self.ser.inWaiting():
                out += self.ser.read().decode('iso-8859-1')
        except Exception as e:
            print(f"[WARN] recv error: {e}")
        return out
    
    # def _send_recv(self, frame: str, expected_min: int = 1, timeout_s: float = 0.1) -> str:
    #     self.ser.reset_input_buffer()
    #     self.ser.reset_output_buffer()
    #     self.ser.write(bytes(frame, 'iso-8859-1'))
    #     deadline = time.monotonic() + timeout_s
    #     buf = bytearray()
    #     while time.monotonic() < deadline:
    #         n = self.ser.in_waiting
    #         if n:
    #             buf.extend(self.ser.read(n))
    #             if len(buf) >= expected_min:
    #                 return buf.decode('iso-8859-1')
    #         time.sleep(0.002)
    #     raise TimeoutError(f'No RS485 response from slave {ord(self.slave)} after {timeout_s}s')

    def _crc_ok(self, data: str) -> bool:
        if len(data) < 4:
            return False
        calc = self._crc16(data[:-2])
        return calc == (data[-2] + data[-1])

    # ------------------------------------------------------------------
    # Private — PR frame builder
    # ------------------------------------------------------------------
    def _build_pr_frame(self, mode: str, position_counts: int,
                        rpm: int, accel: int, decel: int) -> str:
        """
        Build the 0x6200 PR immediate-trigger frame (8 words / 16 bytes).
        Pr9.00 mode | Pr9.01-02 position | Pr9.03 speed |
        Pr9.04 accel | Pr9.05 decel | Pr9.06 delay | Pr9.07 trigger
        """
        data = (self.slave
                + FUNC_WRITE_MULTI
                + REG_PR_COMMAND
                + '\x00\x08'            # 8 registers
                + '\x10'                # 16 data bytes
                + mode                  # Pr9.00
                + self._to_dword(position_counts)   # Pr9.01-02
                + self._to_word(rpm)    # Pr9.03
                + self._to_word(accel)  # Pr9.04
                + self._to_word(decel)  # Pr9.05
                + '\x00\x00'            # Pr9.06 delay = 0
                + '\x00\x10')           # Pr9.07 trigger immediately
        return data + self._crc16(data)

    # ------------------------------------------------------------------
    # Public — motion commands
    # ------------------------------------------------------------------
    def command_velocity(self, rpm: int) -> bool:
        """Continuous velocity. Negative rpm = reverse."""
        frame = self._build_pr_frame(PR_MODE_VELOCITY, 0, rpm,
                                     self.accel, self.decel)
        # direction handled by sign — TODO: confirm sign convention with hardware
        resp = self._send_recv(frame)
        return self._crc_ok(resp)

    def command_position(self, counts: int, rpm: int,
                         relative: bool = True) -> bool:
        """Move by counts (relative) or to counts (absolute)."""
        mode  = PR_MODE_REL if relative else PR_MODE_ABS
        frame = self._build_pr_frame(mode, counts, rpm,
                                     self.accel, self.decel)
        resp  = self._send_recv(frame)
        return self._crc_ok(resp)

    def stop(self) -> bool:
        """Decelerate to zero via velocity mode at 0 rpm."""
        frame = self._build_pr_frame(PR_MODE_VELOCITY, 0, 0,
                                     self.accel, self.decel)
        resp  = self._send_recv(frame)
        return self._crc_ok(resp)

    def emergency_stop(self):
        """Fire-and-forget e-stop. No ACK wait."""
        # E-stop register 0x6002, value 0x0040 per manual pg 125
        data  = self.slave + FUNC_WRITE_SINGLE + '\x60\x02' + '\x00\x40'
        frame = data + self._crc16(data)
        self.ser.write(bytes(frame, 'iso-8859-1'))

    # ------------------------------------------------------------------
    # Public — feedback
    # ------------------------------------------------------------------
    def read_encoder(self) -> int:
        """Returns signed 32-bit encoder count."""
        data  = self.slave + FUNC_READ + REG_ENCODER + '\x00\x02'
        frame = data + self._crc16(data)
        resp  = self._send_recv(frame)
        try:
            if self._crc_ok(resp) and len(resp) >= 7:
                ticks = (ord(resp[3]) << 24 | ord(resp[4]) << 16 |
                         ord(resp[5]) <<  8 | ord(resp[6]))
                # two's complement 32-bit
                return ticks - (1 << 32) if ticks & 0x80000000 else ticks
        except Exception as e:
            print(f"[WARN] encoder parse: {e}")
        return 0

    def read_velocity(self) -> int:
        """Stub — read actual velocity feedback register. TODO: confirm register address."""
        pass

    # ------------------------------------------------------------------
    # Public — drive enable/disable 
    # ------------------------------------------------------------------
    def enable(self):
        """TODO: write 0x0083 to register 0x0405."""
        pass

    def disable(self):
        """TODO: write 0x0003 to register 0x0405."""
        pass

    # ------------------------------------------------------------------
    # Public — alarms (stubs for now)
    # ------------------------------------------------------------------
    def read_alarm(self) -> str:
        """TODO: read REG_ALARM, decode alarm code."""
        pass

    def clear_alarm(self) -> bool:
        """TODO: write 0x1111 to alarm clear register 0x019A."""
        pass


# ---------------------------------------------------------------------------
# BaseDrive — diff drive, owns kinematics
# ---------------------------------------------------------------------------
class BaseDrive:

    def __init__(self, ser: serial.Serial,
                 left_id: int, right_id: int,
                 wheel_dia_m: float = 0.1524,   # 6 inches
                 track_width_m: float = 0.7,
                 gear_ratio: float = 20.0,
                 counts_per_rev: int = 10000,
                 left_sign: int = 1,
                 right_sign: int = -1):
        self.left        = LeadshineMotor(ser, left_id,  counts_per_rev=counts_per_rev)
        self.right       = LeadshineMotor(ser, right_id, counts_per_rev=counts_per_rev)
        self.wheel_dia   = wheel_dia_m
        self.track_width = track_width_m
        self.gear_ratio  = gear_ratio
        self.counts_per_rev = counts_per_rev
        self.left_sign   = left_sign
        self.right_sign  = right_sign

    def rpm_from_mps(self, mps: float) -> float:
        """Linear wheel speed (m/s) → motor RPM (accounts for gear ratio)."""
        import math
        wheel_rps = mps / (math.pi * self.wheel_dia)
        return wheel_rps * 60.0 * self.gear_ratio

    def drive_velocity(self, linear_mps: float, angular_radps: float):
        """Differential drive kinematics → command both motors."""
        v_left  = linear_mps - (angular_radps * self.track_width / 2.0)
        v_right = linear_mps + (angular_radps * self.track_width / 2.0)
        rpm_l   = int(self.rpm_from_mps(v_left)  * self.left_sign)
        rpm_r   = int(self.rpm_from_mps(v_right) * self.right_sign)
        self.left.command_velocity(rpm_l)
        self.right.command_velocity(rpm_r)

    def stop(self):
        self.left.stop()
        self.right.stop()

    def emergency_stop(self):
        self.left.emergency_stop()
        self.right.emergency_stop()

    def read_encoders(self) -> tuple:
        return self.left.read_encoder(), self.right.read_encoder()

    # Stubs — not needed for initial test
    def drive_position(self, distance_m: float, rpm: int):
        """TODO: convert distance to counts, command both motors."""
        pass

    def pivot(self, angle_rad: float, rpm: int):
        """TODO: zero-turn radius. Opposite counts on each motor."""
        pass

    def meters_to_counts(self, m: float) -> int:
        """TODO: m → encoder counts via wheel dia + gear ratio."""
        pass

    def counts_to_meters(self, counts: int) -> float:
        """TODO: encoder counts → meters."""
        pass


# ---------------------------------------------------------------------------
# ZAxis
# ---------------------------------------------------------------------------
class ZAxis:

    def __init__(self, ser: serial.Serial, slave_id: int,
                 counts_per_rev: int = 10000,
                 counts_per_mm: float = 100.0,
                 min_height_mm: float = 0.0,
                 max_height_mm: float = 1800.0,
                 accel: int = DEFAULT_ACCEL_ZAxis,
                 decel: int = DEFAULT_DECEL_ZAxis):
        
        self.motor = LeadshineMotor(ser, slave_id,
                                    accel=accel, decel=decel,
                                    counts_per_rev=counts_per_rev)
        self.counts_per_mm = counts_per_mm
        self.min_height    = min_height_mm
        self.max_height    = max_height_mm

    def _clamp(self, height_mm: float) -> float:
        return max(self.min_height, min(self.max_height, height_mm))

    def move_to(self, height_mm: float, rpm: int) -> bool:
        height_mm = self._clamp(height_mm)
        counts    = int(height_mm * self.counts_per_mm)
        return self.motor.command_position(counts, rpm, relative=False)

    def move_by(self, delta_mm: float, rpm: int) -> bool:
        counts = int(delta_mm * self.counts_per_mm)
        return self.motor.command_position(counts, rpm, relative=True)

    def stop(self) -> bool:
        return self.motor.stop()

    def emergency_stop(self):
        self.motor.emergency_stop()

    def read_position_mm(self) -> float:
        counts = self.motor.read_encoder()
        return counts / self.counts_per_mm



# ---------------------------------------------------------------------------
#  bench test — 
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    # PORT  = 'COM7'   # or COM6
    PORT = '/dev/ttyACM0'
    BAUD  = 115200

    ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=0.5,
                        parity=serial.PARITY_NONE,
                        bytesize=serial.EIGHTBITS,
                        stopbits=serial.STOPBITS_ONE)

    base = BaseDrive(ser, left_id=2, right_id=3)
    zax  = ZAxis(ser, slave_id=1,accel= DEFAULT_ACCEL_ZAxis, decel= DEFAULT_DECEL_ZAxis)

    CPM       = 576.03               # counts per mm (post-gearbox)
    SPEED_MPS = 0.6                  # test speed (half of 0.6 target)
        # ── absolute up/down toggle based on where we are ──────────
    BOTTOM = 2030            # your start/home encoder count
    TOP    = 2030 + 576030   # +1 m  = 578060 counts
    rpm    = int(SPEED_MPS * (CPM * 1000 / 10000) * 60)   # 0.3 m/s

    now = zax.motor.read_encoder()
    print(f"current: {now} counts ({now/CPM/1000:.3f} m)")

    # if closer to bottom, go up; else go down
    midpoint = (BOTTOM + TOP) / 2
    target = TOP if now < midpoint else BOTTOM
    print(f"moving {'UP → TOP' if target==TOP else 'DOWN → BOTTOM'} "
          f"({target} counts) at {rpm} rpm")

    zax.motor.command_position(target, rpm, relative=False)   # absolute
    time.sleep(6.0)          # 1 m at 0.3 m/s ≈ 3.3s + ramp; margin
    end = zax.motor.read_encoder()
    print(f"ended at: {end} counts ({end/CPM/1000:.3f} m)")
    ser.close()

    # # ── Z-axis speed bench test ─────────────────────────────────


    # rpm       = int(SPEED_MPS * (CPM * 1000 / 10000) * 60)   # 0.3 m/s -> 1036 rpm
    # counts_1m = int(1000 * CPM)                              # 1 m -> 576030 counts

    # # 1. read current position
    # start = zax.motor.read_encoder()
    # print(f"start encoder: {start} counts  ({start/CPM/1000:.3f} m)")

    # # 2. command +1 m at 0.3 m/s (relative)
    # #    NOTE: sign — if +counts drives the wrong way, flip to -counts_1m
    # print(f"commanding +1 m ({counts_1m} counts) at {rpm} rpm ({SPEED_MPS} m/s)")
    # t0 = time.time()
    # zax.motor.command_position(counts_1m, rpm, relative=True)

    # # 3. wait, then read where it ended up
    # time.sleep(5.0)
    # end = zax.motor.read_encoder()
    # dt  = time.time() - t0
    # moved_m = (end - start) / CPM / 1000
    # print(f"end encoder:   {end} counts  ({end/CPM/1000:.3f} m)")
    # print(f"moved {moved_m:.3f} m in ~{dt:.1f} s  "
    #       f"(avg {moved_m/dt:.3f} m/s incl. ramp)")

    #     # ── return to starting pose ─────────────────────────────────
    # print(f"returning to start ({start} counts) at {rpm} rpm")
    # zax.motor.command_position(start, rpm, relative=False)   # absolute → start
    # time.sleep(8.0)     # 0.5 m at 0.1 m/s ≈ 5s; give margin for full travel
    # now = zax.motor.read_encoder()
    # print(f"returned to: {now} counts  ({now/CPM/1000:.3f} m)")      

    # ser.close()

    # print("-- velocity 100 RPM --")
    # base.left.command_velocity(100)
    # time.sleep(1)
    # print("left encoder:", base.left.read_encoder())
    # base.left.stop()
    # time.sleep(0.5)

    # base.right.command_velocity(100)
    # time.sleep(1)
    # print("right encoder:", base.right.read_encoder())
    # base.right.stop()
    # time.sleep(0.5)

    # print("-- paired velocity test --")
    # base.drive_velocity(0.1, 0.0)  # 0.1 m/s straight, no rotation
    # time.sleep(1)
    # base.stop()

    # print("-- move up 100mm --")
    # zax.move_by(-100.0, rpm=500)
    # time.sleep(1)
    # print("position:", zax.read_position_mm())

    # print("-- move back to 0 --")
    # zax.move_to(100.0, rpm=500)
    # time.sleep(1)
    # print("position:", zax.read_position_mm())

    # print("-- velocity mode --")
    # zax.motor.command_velocity(200)
    # time.sleep(2)
    # zax.motor.stop()
    # time.sleep(0.5)

    # print("-- velocity reverse --")
    # zax.motor.command_velocity(-200)
    # time.sleep(2)
    # zax.motor.stop()


    # ser.close()


