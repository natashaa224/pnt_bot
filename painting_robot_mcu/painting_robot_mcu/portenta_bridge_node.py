import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float32MultiArray, Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

from painting_robot_msgs.msg import (
    McuStatus,
    DiStatus,
    
    SafetyMode,
    RobotCommand,
    PaintHeadStatus,
   
    SprayCmd,
    # WristCmd,   
    # ZAxisCmd,   
)

from painting_robot_mcu.pmc_io_map import PmcIoMap
from ament_index_python.packages import get_package_share_directory
import os

import math
import socket
import struct
import threading
import time

# ─────────────────────────────────────────────────────────────────────────────
# Protocol constants — must match protocol_definitions.h exactly
# Locked version — see comms_test_handoff.md
# ─────────────────────────────────────────────────────────────────────────────
PROTO_SYNC1         = 0xAA
PROTO_SYNC2         = 0x55
PROTO_MAX_PAYLOAD   = 64
PROTO_OVERHEAD      = 6     # SYNC1+SYNC2+CMD_ID+PAYLOAD_LEN+CRC_LO+CRC_HI

# SBC -> Portenta command IDs
CMD_HEARTBEAT       = 0x01
CMD_MODE_SET        = 0x02
CMD_STOP            = 0x03
CMD_RESET_FAULT     = 0x04
CMD_IO              = 0x05

# Portenta -> SBC status IDs
STATUS_MCU          = 0x81
STATUS_HB_ACK       = 0x82
STATUS_DI           = 0x83
STATUS_ANALOG       = 0x84
STATUS_ULTRASONIC   = 0x85
STATUS_IMU          = 0x86

# Mode values
MODE_IDLE           = 0x00
MODE_MANUAL         = 0x01
MODE_AUTO           = 0x02
MODE_HALT           = 0x03

# MCU state values
MCU_STATE_BOOT      = 0x00
MCU_STATE_READY     = 0x01
MCU_STATE_RUN       = 0x02
MCU_STATE_FAULT     = 0x03
MCU_STATE_ESTOP     = 0x04

# IO_CMD bitmask — must match protocol_definitions.h exactly (v2.1)
IO_SSR              = 1 << 0   # DO00 — ssr_on
IO_RELAY1           = 1 << 1   # DO01 — relay1_on
IO_SOLENOID         = 1 << 2   # DO02 — solenoid_on
IO_RELAY2           = 1 << 3   # DO03 — relay2_on
IO_RELAY3           = 1 << 4   # DO04 — relay3_on
IO_RELAY4           = 1 << 5   # DO05 — relay4_on
IO_TOWER_GREEN      = 1 << 6   # DO06 — tower_green
IO_TOWER_AMBER      = 1 << 7   # DO07 — tower_amber
IO_TOWER_RED        = 1 << 8   # DIO02-OUT — tower_red
IO_BUZZER           = 1 << 9   # DIO03-OUT — buzzer_on

# Fault bitmask — matches current PMC firmware (comms_test_handoff.md)
# Motors are SBC-owned (RS485) for now — no drive fault bits here.
# If motors move to PMC later, this map needs revision.
FAULT_NAMES = {
    0:  'FAULT_ESTOP',
    1:  'FAULT_SAFETY_RELAY',
    2:  'FAULT_HEARTBEAT',
    3:  'FAULT_LIMIT_UPPER',
    4:  'FAULT_LIMIT_LOWER',
    5:  'FAULT_CLIFF',
    6:  'FAULT_PAINT_LOW',
    7:  'FAULT_PRESSURE',
    8:  'FAULT_TEMPERATURE',
    9:  'FAULT_ULTRASONIC',
    10: 'FAULT_IMU',
    11: 'FAULT_BATTERY_LOW',
}

# ─────────────────────────────────────────────────────────────────────────────
# CRC16-MODBUS — mirrors crc16_modbus() in protocol_definitions.h
# Covers CMD_ID + PAYLOAD_LEN + PAYLOAD
# ─────────────────────────────────────────────────────────────────────────────
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# ─────────────────────────────────────────────────────────────────────────────
# Packet builder
# ─────────────────────────────────────────────────────────────────────────────
def build_packet(cmd_id: int, payload: bytes = b'') -> bytes:
    assert len(payload) <= PROTO_MAX_PAYLOAD
    payload_len = len(payload)
    crc_data    = bytes([cmd_id, payload_len]) + payload
    crc         = crc16_modbus(crc_data)
    packet      = bytes([PROTO_SYNC1, PROTO_SYNC2, cmd_id, payload_len])
    packet     += payload
    packet     += struct.pack('<H', crc)    # little-endian CRC
    return packet


# ─────────────────────────────────────────────────────────────────────────────
# TCP client — owns socket, reconnect loop, recv ring buffer
# Runs in a background thread — thread-safe via lock for send
# ─────────────────────────────────────────────────────────────────────────────
class PortentaTcpClient:

    RECONNECT_INTERVAL_S = 2.0
    RECV_BUFFER_SIZE     = 512

    def __init__(self, host: str, port: int, on_packet_cb):
        self._host          = host
        self._port          = port
        self._on_packet_cb  = on_packet_cb  # called with (cmd_id, payload bytes)
        self._sock          = None
        self._connected     = False
        self._lock          = threading.Lock()
        self._ring          = bytearray()
        self._stop_event    = threading.Event()
        self._thread        = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._disconnect()

    def is_connected(self) -> bool:
        return self._connected

    def send(self, data: bytes) -> bool:
        with self._lock:
            if not self._connected or self._sock is None:
                return False
            try:
                self._sock.sendall(data)
                return True
            except OSError:
                self._connected = False
                return False

    # ── Internal ──────────────────────────────────────────────────────────────

    def _run(self):
        while not self._stop_event.is_set():
            if not self._connected:
                self._try_connect()
            else:
                self._recv_loop()

    def _try_connect(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((self._host, self._port))
            sock.settimeout(0.1)    # non-blocking-ish for recv
            with self._lock:
                self._sock      = sock
                self._connected = True
            self._ring.clear()
        except OSError:
            time.sleep(self.RECONNECT_INTERVAL_S)

    def _recv_loop(self):
        try:
            data = self._sock.recv(self.RECV_BUFFER_SIZE)
            if not data:
                self._disconnect()
                return
            self._ring.extend(data)
            self._parse_ring()
        except socket.timeout:
            pass    # normal — no data this cycle
        except OSError:
            self._disconnect()

    def _disconnect(self):
        with self._lock:
            self._connected = False
            if self._sock:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None

    def _parse_ring(self):
        """Scan ring buffer for valid packets and dispatch them."""
        while len(self._ring) >= PROTO_OVERHEAD:
            # Find sync
            try:
                idx = self._ring.index(PROTO_SYNC1)
            except ValueError:
                self._ring.clear()
                return

            # Discard bytes before sync
            if idx > 0:
                del self._ring[:idx]

            if len(self._ring) < 2:
                return
            if self._ring[1] != PROTO_SYNC2:
                # False sync — discard one byte and retry
                del self._ring[0]
                continue

            if len(self._ring) < 4:
                return  # wait for more bytes

            cmd_id      = self._ring[2]
            payload_len = self._ring[3]

            if payload_len > PROTO_MAX_PAYLOAD:
                # Bad length — discard sync and resync
                del self._ring[:2]
                continue

            total_len = PROTO_OVERHEAD + payload_len
            if len(self._ring) < total_len:
                return  # wait for more bytes

            # Extract and verify CRC
            crc_data    = bytes(self._ring[2:4 + payload_len])
            crc_calc    = crc16_modbus(crc_data)
            crc_rx      = struct.unpack_from('<H', self._ring, 4 + payload_len)[0]

            if crc_calc != crc_rx:
                # Bad CRC — discard sync bytes and resync
                del self._ring[:2]
                continue

            # Valid packet
            payload = bytes(self._ring[4:4 + payload_len])
            del self._ring[:total_len]

            self._on_packet_cb(cmd_id, payload)


# ─────────────────────────────────────────────────────────────────────────────
# PortentaBridgeNode
# ─────────────────────────────────────────────────────────────────────────────
class PortentaBridgeNode(Node):
    """
    SBC-side bridge between the ROS 2 graph and the Portenta MCU (PMC).

    Transport: raw TCP packets per protocol_definitions.h / comms_test_handoff.md

    Responsibilities:
      - Send HEARTBEAT at 10 Hz
      - Send MODE_SET, STOP, RESET_FAULT, IO_CMD over TCP
      - Receive MCU_STATUS, DI_STATUS, ANALOG_STATUS, ULTRASONIC_STATUS, IMU_STATUS
      - Republish decoded PMC data onto ROS graph
      - Publish /mcu_bridge_ok connection health

    NOTE — Motor control placeholder:
      Drive motors (RS485 ELD2) are currently controlled directly from the
      SBC via a separate rs485_motor_control_node, NOT through the PMC.
      _cmd_vel_cb is therefore a no-op here. If motor control moves to the
      PMC later, re-enable a MOVE command path here and update the PMC
      firmware command set accordingly. Left as an explicit placeholder
      rather than removed.

    This node does NOT make safety decisions — it only moves data.
    Safety decisions are the responsibility of safety_monitor_node.
    """

    # ── Robot geometry — retained for when/if motor control returns to PMC ────
    WHEEL_BASE_M        = 0.70      # metres — tune to real hardware
    WHEEL_RADIUS_M      = 0.075     # metres
    MAX_RPM             = 150.0
    HEARTBEAT_RATE_HZ   = 10.0

    # Portenta IP and port — must match system_config.h
    PORTENTA_IP         = '192.168.1.100'
    PORTENTA_PORT       = 5000

    # Watchdog — consider Portenta lost if no STATUS_MCU for this long
    MCU_TIMEOUT_S       = 0.5

    def __init__(self):
        super().__init__('portenta_bridge_node')

        # ── QoS ──────────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        safety_qos   = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=5,
        )
        best_effort_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)

        # PMC I/O map — single source of truth for DI/DO bit assignments.
        # The bridge is the only place that touches bits.
        io_map_path = os.path.join(
            get_package_share_directory('painting_robot_bringup'),
            'config', 'pmc_io_map.yaml')
        try:
            self._io_map = PmcIoMap(io_map_path)
            self.get_logger().info(f'[BRIDGE] loaded PMC I/O map: {io_map_path}')
        except Exception as e:
            self.get_logger().error(f'[BRIDGE] failed to load pmc_io_map.yaml: {e}')
            raise

        # ── State ─────────────────────────────────────────────────────────────
        self._heartbeat_seq     = 0
        self._last_mcu_stamp    = None
        self._mcu_connected     = False
        self._current_mode      = MODE_IDLE

        # Solenoid flags — isolated, never touched by tower/LED logic
        self._solenoid_flags    = 0x0000
        # Tower/LED flags — isolated, never touches solenoid bits
        self._tower_flags       = 0x0000
        self._estop_flash_state = False   # toggles for ESTOP flash
        self._fsm_state         = 'IDLE'

        # ── TCP client ────────────────────────────────────────────────────────
        self._tcp = PortentaTcpClient(
            host            = self.PORTENTA_IP,
            port            = self.PORTENTA_PORT,
            on_packet_cb    = self._on_packet_received,
        )

    

        # ── ROS subscribers (ROS graph -> bridge -> TCP -> Portenta) ───────────

        # Motor control is SBC/RS485-owned for now — see class docstring.
        # Subscription kept so the hook exists; callback is a no-op placeholder.
        self.create_subscription(
            Twist,
            '/paint_bot/cmd_vel',
            self._cmd_vel_cb,
            reliable_qos,
        )
        self.create_subscription(
            SafetyMode,
            '/safety_mode',
            self._safety_mode_cb,
            safety_qos,
        )
        self.create_subscription(
            SprayCmd,
            '/spray_cmd',
            self._spray_cb,
            reliable_qos,
        )
        self.create_subscription(
            String,
            '/robot_state',
            self._robot_state_cb,
            reliable_qos,
        )
        # Stage 2+ — paint commands, paint head commands 
        # self.create_subscription(PaintCmd,     '/paint_cmd',      ...)
        # self.create_subscription(PaintHeadCmd, '/paint_head_cmd', ...)

        # ── ROS publishers (TCP -> bridge -> ROS graph) ────────────────────────
        self._mcu_status_pub    = self.create_publisher(McuStatus,         '/mcu_status_ros',        reliable_qos)
        self._di_status_pub     = self.create_publisher(DiStatus,          '/di_status_ros',         reliable_qos)
        self._imu_pub           = self.create_publisher(Imu,               '/imu/out',              best_effort_qos)
        self._bridge_ok_pub     = self.create_publisher(Bool,              '/mcu_bridge_ok',         reliable_qos)
        # Stage 2+
        # self._paint_head_status_pub = self.create_publisher(PaintHeadStatus, '/paint_head_status_ros', ...)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / self.HEARTBEAT_RATE_HZ, self._heartbeat_loop)
        self.create_timer(self.MCU_TIMEOUT_S,            self._connection_watchdog)
        self.create_timer(0.5,                           self._tower_flash_loop)

         #Start TCP last — ensures all publishers exist before packets arrive
        self.get_logger().info(
            f'[BRIDGE] TCP client started — connecting to '
            f'{self.PORTENTA_IP}:{self.PORTENTA_PORT}'
        )
        self._tcp.start()
        
        self.get_logger().info('[BRIDGE] PortentaBridgeNode started')



    def destroy_node(self):
        self._tcp.stop()
        super().destroy_node()

    # ─── KINEMATIC CONVERSION (retained for future PMC-owned motor control) ──

    def _twist_to_rpm(self, linear_x: float, angular_z: float):
        v_left  = linear_x - (angular_z * self.WHEEL_BASE_M / 2.0)
        v_right = linear_x + (angular_z * self.WHEEL_BASE_M / 2.0)

        rpm_left  = (v_left  / (2.0 * math.pi * self.WHEEL_RADIUS_M)) * 60.0
        rpm_right = (v_right / (2.0 * math.pi * self.WHEEL_RADIUS_M)) * 60.0

        rpm_left  = max(-self.MAX_RPM, min(self.MAX_RPM, rpm_left))
        rpm_right = max(-self.MAX_RPM, min(self.MAX_RPM, rpm_right))

        return rpm_left, rpm_right

    # ─── ROS → TCP ───────────────────────────────────────────────────────────

    def _spray_cb(self, msg: SprayCmd):
        # Solenoid is fully isolated -- never shares flags with tower/LED logic
        if msg.valve_open:
            self._solenoid_flags |= IO_SOLENOID
        else:
            self._solenoid_flags &= ~IO_SOLENOID
        self._send_solenoid()
        self.get_logger().info(
            f'[BRIDGE] spray → {"ON" if msg.valve_open else "OFF"} '
            f'(solenoid_flags=0x{self._solenoid_flags:04x})')

    def _send_solenoid(self):
        # Must combine with tower flags -- PMC replaces full DO word each CMD_IO
        combined = self._solenoid_flags | self._tower_flags
        payload = struct.pack('<H', combined)
        self._tcp.send(build_packet(CMD_IO, payload))

    def _robot_state_cb(self, msg: String):
        self._fsm_state = msg.data
        self._update_tower()

    def _update_tower(self):
        state = self._fsm_state
        if state == 'FAULT':
            return  # flash loop owns FAULT state
        elif state == 'IDLE':
            self._tower_flags = IO_TOWER_GREEN
        elif state == 'SYSTEM_CHECK':
            self._tower_flags = IO_TOWER_AMBER
        elif state in ('WALL_ALIGN', 'WALL_FOLLOW', 'PAINT_EXECUTE',
                       'STEP_REALIGN', 'CORNER_APPROACH', 'CORNER_LOGIC'):
            self._tower_flags = IO_TOWER_GREEN | IO_TOWER_AMBER
        else:
            self._tower_flags = IO_TOWER_GREEN
        self._send_tower()

    def _tower_flash_loop(self):
        if self._fsm_state == 'FAULT':
            self._estop_flash_state = not self._estop_flash_state
            self._tower_flags = IO_TOWER_RED | (IO_BUZZER if self._estop_flash_state else 0)
            self._send_tower()

    def _send_tower(self):
        # Must combine with solenoid flags -- PMC replaces full DO word each CMD_IO
        combined = self._solenoid_flags | self._tower_flags
        payload = struct.pack('<H', combined)
        self._tcp.send(build_packet(CMD_IO, payload))

    def _cmd_vel_cb(self, msg: Twist):
        # PLACEHOLDER — motors are SBC/RS485-owned for now.
        # rs485_motor_control_node handles /diff_cont/cmd_vel directly.
        # If motor control moves to PMC later: re-enable _twist_to_rpm() here,
        # pack into a MOVE command, and add the corresponding CMD_MOVE id +
        # firmware handler back to the protocol.
        pass

    def _safety_mode_cb(self, msg: SafetyMode):
        # New SafetyMode tiers (SAFETY_HOLD / FAULT / ESTOP / RESET / NOMINAL).
        # All stop tiers -> PMC MODE_HALT for now; the true ESTOP relay-drop is a
        # firmware TODO (needs a CMD that drops DO03 / gates the watchdog).
        if msg.mode in (SafetyMode.SAFETY_HOLD, SafetyMode.FAULT, SafetyMode.ESTOP):
            mode_byte = MODE_HALT
            self.get_logger().warn(
                f'[BRIDGE] SafetyMode {msg.mode} (HALT motion) → Portenta '
                f'— reason: {msg.reason}'
            )
            payload = struct.pack('<B', mode_byte)
            self._tcp.send(build_packet(CMD_MODE_SET, payload))
            self._current_mode = mode_byte

        elif msg.mode == SafetyMode.RESET:
            self.get_logger().info('[BRIDGE] SafetyMode RESET → CMD_RESET_FAULT')
            self._tcp.send(build_packet(CMD_RESET_FAULT))
            payload = struct.pack('<B', MODE_MANUAL)
            self._tcp.send(build_packet(CMD_MODE_SET, payload))
            self._current_mode = MODE_MANUAL

        elif msg.mode == SafetyMode.NOMINAL:
            # resume only if we were halted; leave a running mode alone
            if self._current_mode not in (MODE_MANUAL, MODE_AUTO):
                self.get_logger().info('[BRIDGE] SafetyMode NOMINAL → resume MANUAL')
                payload = struct.pack('<B', MODE_MANUAL)
                self._tcp.send(build_packet(CMD_MODE_SET, payload))
                self._current_mode = MODE_MANUAL

    def send_mode(self, mode: int):
        """External call — e.g. from operator HMI node."""
        payload = struct.pack('<B', mode)
        self._tcp.send(build_packet(CMD_MODE_SET, payload))
        self._current_mode = mode

    def send_reset_fault(self):
        """External call — operator reset after fault cleared."""
        self._tcp.send(build_packet(CMD_RESET_FAULT))

    def send_io_cmd(self, io_flags: int):
        """
        External call — set tower lights, buzzer, solenoid, pump,
        compressor, contactor via bitmask. See IO_* constants.
        """
        payload = struct.pack('<H', io_flags)
        self._tcp.send(build_packet(CMD_IO, payload))

    # ─── HEARTBEAT ───────────────────────────────────────────────────────────

    def _heartbeat_loop(self):
        seq     = self._heartbeat_seq % 65535
        payload = struct.pack('<I', seq)
        self._tcp.send(build_packet(CMD_HEARTBEAT, payload))
        self._heartbeat_seq += 1

    # ─── TCP → ROS (incoming packet dispatcher) ───────────────────────────────

    def _on_packet_received(self, cmd_id: int, payload: bytes):
        """Called from TCP client thread — dispatch to handler."""
        # self.get_logger().info(f'RX cmd=0x{cmd_id:02X} len={len(payload)}')    

        if   cmd_id == STATUS_MCU:        self._handle_mcu_status(payload)
        elif cmd_id == STATUS_HB_ACK:     self._handle_hb_ack(payload)
        elif cmd_id == STATUS_DI:         self._handle_di_status(payload)
        elif cmd_id == STATUS_ANALOG:     self._handle_analog_status(payload)
        elif cmd_id == STATUS_ULTRASONIC: self._handle_ultrasonic_status(payload)
        elif cmd_id == STATUS_IMU:        self._handle_imu_status(payload)
        else:
            self.get_logger().warn(f'[BRIDGE] Unknown status ID: 0x{cmd_id:02X}')

    def _handle_mcu_status(self, payload: bytes):
        # state(B) fault_code(H) fault_severity(H) watchdog_ok(B) safety_relay_ok(B) estop_active(B)
        if len(payload) < 8:
            return
        state, fault_code, fault_severity, watchdog_ok, safety_relay_ok, estop_active = \
            struct.unpack_from('<BHHBBB', payload)

        # self.get_logger().info(f'[BRIDGE] MCU fault_code=0x{fault_code:04X} estop_active={estop_active}')

        self._last_mcu_stamp = self.get_clock().now()

        if not self._mcu_connected:
            self._mcu_connected = True
            self.get_logger().info('[BRIDGE] Portenta TCP connection established')

        #deactiving fault triggering till pmc code is fixed   
        # if fault_code != 0:
        #     self._log_faults(fault_code, fault_severity)

        msg                    = McuStatus()
        msg.header.stamp       = self.get_clock().now().to_msg()
        msg.mcu_state          = state
        msg.fault_code         = fault_code
        msg.fault_severity     = fault_severity
        msg.watchdog_ok        = bool(watchdog_ok)
        msg.safety_relay_ok    = bool(safety_relay_ok)
        msg.estop_active       = bool(estop_active)
        self._mcu_status_pub.publish(msg)

    def _handle_hb_ack(self, payload: bytes):
        if len(payload) < 4:
            return
        seq = struct.unpack_from('<I', payload)[0]
        self.get_logger().debug(f'[BRIDGE] HB_ACK seq={seq}')

    def _handle_di_status(self, payload: bytes):
        # PayloadDiStatus is still 10 bytes on the wire, but the named bytes are
        # mislabeled / in flux on the PMC side. We decode EVERYTHING from di_raw
        # via pmc_io_map.yaml so the logical DiStatus fields are correct
        # regardless of how the firmware currently labels its channels.
        if len(payload) < 1:
            return
        di_raw = payload[0]
        # self.get_logger().info(f'DI payload: {payload.hex()}')


        sig = self._io_map.decode_inputs(di_raw)   # logical name -> bool

        msg                 = DiStatus()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.di_raw          = di_raw
        # safety-relevant (observed; hardware EMG chain owns the actual trip)
        msg.estop           = sig.get('estop', False)
        msg.emg_limits      = sig.get('emg_limits', False)
        msg.ceiling_detect  = sig.get('ceiling_detect', False)
        msg.upper_limit     = sig.get('upper_limit', False)
        msg.lower_limit     = sig.get('lower_limit', False)
        msg.relay_status    = sig.get('relay_status', False)
        msg.paint_level_ok  = sig.get('paint_level_ok', False)
        # non-safety (FSM / mode / master power own these)
        msg.pause_resume    = sig.get('pause_resume', False)
        msg.auto_manual     = sig.get('auto_manual', False)
        msg.on_off          = sig.get('on_off', False)
        self._di_status_pub.publish(msg)


    def _handle_analog_status(self, payload: bytes):
        # battery_v(f) temp_c(f) — 2 Hz
        # currently logged only; fold into McuStatus.msg fields if needed later
        if len(payload) < 8:
            return
        battery_v, temp_c = struct.unpack_from('<ff', payload)
        self.get_logger().debug(
            f'[BRIDGE] ANALOG — battery={battery_v:.2f}V temp={temp_c:.1f}C'
        )

    def _handle_ultrasonic_status(self, payload: bytes):
        # front(f) left(f) right(f) rear(f)
        if len(payload) < 16:
            return
        front, left, right, rear = struct.unpack_from('<ffff', payload)

        msg       = Float32MultiArray()
        msg.data  = [front, left, right, rear]
        self._ultrasonic_pub.publish(msg)

    def _handle_imu_status(self, payload: bytes):
        # qr qi qj qk roll pitch yaw — quaternion + euler angles (degrees)
        if len(payload) < 28:
            return
        (qr, qi, qj, qk,
         roll, pitch, yaw) = struct.unpack_from('<fffffff', payload)

        msg                 = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Orientation (BNO085 rotation vector, already normalized)
        msg.orientation.w = qr
        msg.orientation.x = qi
        msg.orientation.y = qj
        msg.orientation.z = qk

        # The current PMC IMU frame does not include gyro or linear acceleration
        # values, so mark those covariance blocks as unavailable.
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        # Orientation covariance — rotation vector is trusted.
        msg.orientation_covariance[0]      = 0.01
        msg.orientation_covariance[4]      = 0.01
        msg.orientation_covariance[8]      = 0.01

        self.get_logger().debug(
            f'[BRIDGE] IMU — q=({qr:.4f}, {qi:.4f}, {qj:.4f}, {qk:.4f}) '
            f'rpy=({roll:.2f}°, {pitch:.2f}°, {yaw:.2f}°)'
        )

        self._imu_pub.publish(msg)

    # ─── CONNECTION WATCHDOG ─────────────────────────────────────────────────

    def _connection_watchdog(self):
        ok      = Bool()
        tcp_up  = self._tcp.is_connected()

        if self._last_mcu_stamp is None or not tcp_up:
            ok.data = False
        else:
            elapsed = (self.get_clock().now() - self._last_mcu_stamp).nanoseconds / 1e9
            ok.data = elapsed < self.MCU_TIMEOUT_S

        if self._mcu_connected and not ok.data:
            self._mcu_connected = False
            self.get_logger().error('[BRIDGE] Portenta connection lost — MCU_STATUS timeout')

        self._bridge_ok_pub.publish(ok)

    # ─── FAULT LOGGING ───────────────────────────────────────────────────────

    def _log_faults(self, fault_code: int, fault_severity: int):
        for bit, name in FAULT_NAMES.items():
            if (fault_code >> bit) & 1:
                is_critical = (fault_severity >> bit) & 1
                if is_critical:
                    self.get_logger().error(
                        f'[FAULT] CRITICAL — {name}',
                        throttle_duration_sec=2.0,
                    )
                else:
                    self.get_logger().warn(
                        f'[FAULT] WARNING — {name}',
                        throttle_duration_sec=2.0,
                    )


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PortentaBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()