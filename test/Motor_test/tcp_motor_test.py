#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from painting_robot_msgs.msg import ZAxisCmd, DiStatus

RPM         = 100
TARGET_UP   = 4000.0   # mm — adjust to your travel range
TARGET_DOWN = -4010.0    # mm

class ZLimitTest(Node):
    def __init__(self):
        super().__init__('z_limit_test')
        self._dir = 1  # 1 = going up, -1 = going down

        self._z_pub = self.create_publisher(ZAxisCmd, '/z_axis_cmd', 10)
        self.create_subscription(DiStatus, '/di_status_ros', self._di_cb, 10)

        # Send initial command
        self.create_timer(0.1, self._send_cmd)
        self.get_logger().info('Z limit test started — moving up')

    def _di_cb(self, msg: DiStatus):
        if msg.limit_upper and self._dir == 1:
            self.get_logger().info('Upper limit hit — reversing DOWN')
            self._dir = -1
            self._send_cmd()
        elif msg.limit_lower and self._dir == -1:
            self.get_logger().info('Lower limit hit — reversing UP')
            self._dir = 1
            self._send_cmd()

    def _send_cmd(self):
        cmd = ZAxisCmd()
        cmd.z_mode          = 0  # Z_POSITION
        cmd.z_target_pos_mm = float(TARGET_UP if self._dir == 1 else TARGET_DOWN)
        cmd.rpm             = RPM
        self._z_pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(ZLimitTest())
    rclpy.shutdown()

if __name__ == '__main__':
    main()