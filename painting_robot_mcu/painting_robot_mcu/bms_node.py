#!/usr/bin/env python3
"""
bms_node.py

Thin ROS 2 wrapper around the standalone KvmsBms driver (kvms_bms_driver.py).
Polls the pack over RS485 at ~1 Hz, maps the relevant subset to BmsStatus,
publishes /bms_status. Sensing-only — no display, no control.

Liveness: the BMS 'life' register increments each read. If it stops changing
across LIFE_STALL_LIMIT polls, comms are considered lost (valid=False), last
good values held.

Publishes:
  /bms_status   (painting_robot_msgs/BmsStatus)   ~1 Hz
"""

import rclpy
from rclpy.node import Node
from painting_robot_msgs.msg import BmsStatus
import serial

from painting_robot_mcu.kvms_bms_driver import KvmsBms

LIFE_STALL_LIMIT = 3        # polls with unchanged 'life' before flagging comms-lost


class BmsNode(Node):

    def __init__(self):
        super().__init__('bms_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyUSB0')   # confirm per boot order
        self.declare_parameter('baud', 9600)
        self.declare_parameter('poll_rate', 1.0)         # Hz

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        rate = self.get_parameter('poll_rate').value

        # ── Serial + driver ───────────────────────────────────────────────────
        try:
            self._ser = serial.Serial(
                port=port, baudrate=baud, timeout=1,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE)
        except serial.SerialException as e:
            self.get_logger().error(f'cannot open {port}: {e}')
            raise
        self._bms = KvmsBms(self._ser)

        # ── Liveness tracking ─────────────────────────────────────────────────
        self._prev_life  = None
        self._stall      = 0
        self._last_good  = None      # last decoded dict, for hold-on-loss

        self._pub = self.create_publisher(BmsStatus, '/bms_status', 10)
        self.create_timer(1.0 / rate, self._poll)

        self.get_logger().info(f'bms_node started on {port}')

    def _poll(self):
        d = self._bms.read_all()

        if d is None:
            # read/CRC failure — hold last good, flag invalid
            self._stall = LIFE_STALL_LIMIT
            self.get_logger().warn('BMS read failed', throttle_duration_sec=5.0)
            self._publish(self._last_good, valid=False)
            return

        # liveness via 'life' heartbeat
        life = d.get('life')
        if life == self._prev_life:
            self._stall = min(self._stall + 1, LIFE_STALL_LIMIT)
        else:
            self._stall = 0
        self._prev_life = life
        valid = self._stall < LIFE_STALL_LIMIT
        if not valid:
            self.get_logger().warn('BMS life stalled — comms lost?',
                                   throttle_duration_sec=5.0)

        self._last_good = d
        self._publish(d, valid=valid)

    def _publish(self, d, valid):
        m = BmsStatus()
        m.header.stamp = self.get_clock().now().to_msg()

        if d is None:
            m.valid = False
            self._pub.publish(m)
            return

        # hottest of cell temps + MOS temp; ambient excluded (unfitted -> 215C garbage)
        temps = list(d.get('cell_temps', []))
        if d.get('mos_temp') is not None:
            temps.append(d['mos_temp'])
        max_temp = max(temps) if temps else 0.0

        m.valid         = bool(valid)
        m.voltage       = float(d['pack_voltage'])
        m.current       = float(d['current'])
        m.soc           = float(d['soc'])
        m.remaining_ah  = float(d['remaining_ah'])
        m.max_temp      = float(max_temp)
        m.charge_state  = int(d['chg_dis_status']) & 0xFF
        m.cell_delta_mv = float(d['cell_delta_mv'])
        m.cycles        = int(d['cycles']) & 0xFFFF
        m.any_fault     = bool(d['any_fault'])
        m.fault_raw     = [int(f) & 0xFFFF for f in d['fault_raw']]

        self._pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = BmsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
