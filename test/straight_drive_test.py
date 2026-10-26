#!/usr/bin/env python3
"""Drive the base straight at a fixed speed for a fixed time, then stop.

Used to calibrate wheel_dia_m / base_gear_ratio in actuator_bridge_params.yaml:
commanded distance = speed * duration (default 0.2 m/s * 10 s = 2.0 m). Mark the
robot's start position, run this, measure how far it really went, and compare it
with the commanded distance and with the odometry distance printed at the end.

Needs the robot bringup (actuator_bridge_node) running and safety NOMINAL: the
bridge ignores /paint_bot/cmd_vel in HOLD / FAULT / E-STOP.

    python3 straight_drive_test.py                       # 0.2 m/s for 10 s
    python3 straight_drive_test.py --speed 0.2 --duration 10
    python3 straight_drive_test.py --speed -0.2          # reverse

SAFETY: the bridge's command watchdog is disabled, so the robot keeps its last
command if this script dies. This script always sends zero velocity on exit
(normal end, Ctrl-C, or an exception) - keep the e-stop within reach anyway.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class StraightDrive(Node):
    def __init__(self, topic, odom_topic):
        super().__init__('straight_drive_test')
        self.pub = self.create_publisher(Twist, topic, 1)
        self.odom = None
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y)

    def send(self, linear, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def stop(self):
        for _ in range(10):                      # repeat so one dropped packet cannot leave it moving
            self.send(0.0)
            time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--speed', type=float, default=0.2, help='linear speed in m/s (default 0.2)')
    ap.add_argument('--duration', type=float, default=10.0, help='drive time in seconds (default 10)')
    ap.add_argument('--rate', type=float, default=20.0, help='command publish rate in Hz (default 20)')
    ap.add_argument('--topic', default='/paint_bot/cmd_vel')
    ap.add_argument('--odom-topic', default='/paint_bot/odom')
    ap.add_argument('--no-confirm', action='store_true', help='start without waiting for Enter')
    args = ap.parse_args()

    if abs(args.speed) > 0.5 or args.duration > 60.0:
        raise SystemExit('Refusing: |speed| > 0.5 m/s or duration > 60 s. Edit the script if you really need it.')

    rclpy.init()
    node = StraightDrive(args.topic, args.odom_topic)
    try:
        # Wait for odometry so the start position is known.
        t_wait = time.time()
        while node.odom is None and time.time() - t_wait < 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom is None:
            print('WARNING: no odometry received - will still drive, but no odometry distance is reported.')

        expected = args.speed * args.duration
        print(f'Will drive {args.speed:+.3f} m/s for {args.duration:.1f} s '
              f'(commanded distance {expected:+.3f} m) on {args.topic}.')
        if not args.no_confirm:
            input('Clear the path, mark the start position, then press Enter to go (Ctrl-C to abort)... ')

        start_odom = node.odom
        period = 1.0 / args.rate
        t0 = time.time()
        next_t = t0
        while True:
            now = time.time()
            if now - t0 >= args.duration:
                break
            node.send(args.speed)
            rclpy.spin_once(node, timeout_sec=0.0)
            next_t += period
            time.sleep(max(0.0, next_t - time.time()))
        t_drive = time.time() - t0
        node.stop()

        # Let the robot settle and take the final odometry sample.
        t_settle = time.time()
        while time.time() - t_settle < 1.0:
            rclpy.spin_once(node, timeout_sec=0.05)

        print(f'Drove for {t_drive:.2f} s (stop sent).')
        if start_odom is not None and node.odom is not None:
            d_odom = math.hypot(node.odom[0] - start_odom[0], node.odom[1] - start_odom[1])
            print(f'Commanded distance : {abs(expected):.3f} m')
            print(f'Odometry distance  : {d_odom:.3f} m')
        print('Now measure the real distance with a tape and compare:')
        print('  new base_gear_ratio = old base_gear_ratio * commanded / measured   (keep wheel_dia_m = measured wheel)')
    except KeyboardInterrupt:
        print('\nAborted - stopping the robot.')
    finally:
        try:
            node.stop()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
