#!/usr/bin/env python3
"""
ultrasound_node.py

Dumb 4-channel ultrasonic sensor node (DYP-style, USB-TTL CH340 -> /dev/ttyACM*).
Trigger-then-read (0x55), sync on 0xFF header, checksum-validate, publish raw mm.
Consumers (env_monitoring / FSM) apply thresholds — node stays sensing-only.

Publishes:
  /ultrasound_status  (painting_robot_msgs/UltrasoundStatus)

Frame (10 bytes): FF  S1hi S1lo  S2hi S2lo  S3hi S3lo  S4hi S4lo  CSUM
  CSUM = low byte of sum(bytes[0:9])
  0xFFFx range values = out-of-range / no-echo error
"""

import rclpy
from rclpy.node import Node
from painting_robot_msgs.msg import UltrasoundStatus
import serial

TRIGGER_BYTE = bytes.fromhex("55")
FRAME_LEN    = 10
ERR_THRESH   = 0xFFF0          # readings >= this are sensor error codes
N_SENSORS    = 4


class UltrasoundNode(Node):

    def __init__(self):
        super().__init__('ultrasound_node')

        # ── Parameters ────────────────────────────────────────────────────────
        # ACM number shifts on replug/boot order — keep as param.
        self.declare_parameter('port', '/dev/ttyUSB2')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('publish_rate', 3.0)   # Hz — blocking read fits ~3Hz
        self.declare_parameter('trigger_delay', 0.1)  # s — after 0x55 before read

        self._port          = self.get_parameter('port').value
        self._baud          = self.get_parameter('baud').value
        self._trigger_delay = self.get_parameter('trigger_delay').value
        rate                = self.get_parameter('publish_rate').value

        # ── Serial ────────────────────────────────────────────────────────────
        try:
            self._ser = serial.Serial(
                port=self._port, baudrate=self._baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.5)
        except serial.SerialException as e:
            self.get_logger().error(f'cannot open {self._port}: {e}')
            raise

        # last good frame, republished on read failure with valid=False
        self._last_dist  = [0] * N_SENSORS

        self._pub = self.create_publisher(UltrasoundStatus, '/ultrasound_status', 10)
        self.create_timer(1.0 / rate, self._process)

        self.get_logger().info(f'ultrasound_node started on {self._port}')

    # ── Frame read ─────────────────────────────────────────────────────────────

    def _read_frame(self):
        """
        Trigger, sync on 0xFF header, read remaining 9 bytes, checksum-validate.
        Returns 10-byte frame or None on any failure.
        """
        try:
            self._ser.reset_input_buffer()
            self._ser.write(TRIGGER_BYTE)
            self._ser.flush()
        except serial.SerialException as e:
            self.get_logger().warn(f'serial write failed: {e}', throttle_duration_sec=2.0)
            return None

        # crude but effective delay for the sensor to answer (blocking, ~3Hz loop)
        import time
        time.sleep(self._trigger_delay)

        try:
            if self._ser.read(1) != b'\xff':
                return None
            rest = self._ser.read(FRAME_LEN - 1)
        except serial.SerialException as e:
            self.get_logger().warn(f'serial read failed: {e}', throttle_duration_sec=2.0)
            return None

        if len(rest) != FRAME_LEN - 1:
            return None
        frame = b'\xff' + rest
        if (sum(frame[0:9]) & 0xFF) != frame[9]:
            return None
        return frame

    @staticmethod
    def _parse(frame):
        return [ (frame[1 + 2 * i] << 8) | frame[2 + 2 * i]
                 for i in range(N_SENSORS) ]

    # ── Timer loop ─────────────────────────────────────────────────────────────

    def _process(self):
        frame = self._read_frame()
        msg = UltrasoundStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasound'

        if frame is None:
            # no valid frame this cycle — republish last distances, all invalid
            msg.distances_mm = [int(d) for d in self._last_dist]
            msg.valid        = [False] * N_SENSORS
            self.get_logger().warn('no valid ultrasound frame', throttle_duration_sec=2.0)
            self._pub.publish(msg)
            return

        dist  = self._parse(frame)
        valid = [d < ERR_THRESH for d in dist]
        self._last_dist = dist

        msg.distances_mm = [int(d) for d in dist]
        msg.valid        = valid
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasoundNode()
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