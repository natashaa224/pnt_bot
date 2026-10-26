#!/usr/bin/env python3
"""Spin the robot in place by a commanded angle (default 360 deg) and check how far it really turned.

Used to calibrate track_width_m in actuator_bridge_params.yaml. The robot is commanded open-loop
(angular z for a fixed time), so the commanded angle = |angular speed| * duration. Compare it with:

  * odometry yaw change  (/paint_bot/odom)   - NOT independent: it uses the same track_width_m, so it
                                               will read close to the commanded angle even if the real
                                               turn is off. Shown only as a sanity check.
  * IMU yaw change       (/imu/out)          - independent reference, if the IMU publishes it
                                               (orientation and/or gyro z, both are reported).
  * the real angle you measure on the floor  - the reference to trust. Mark the robot's heading on the
                                               floor (tape line along a side edge) before the run.

    python3 rotation_test.py                      # 360 deg at +0.4 rad/s (~15.7 s)
    python3 rotation_test.py --angle 360 --speed -0.4 --track 0.625

The suggested track is:  new_track = old_track * odometry_yaw / real_angle
(it makes the odometry heading - which the turn controller stops on - equal the real heading; if there is no
odometry it falls back to old_track * commanded_angle / real_angle).

SAFETY: the bridge's command watchdog is disabled, so the robot keeps its last command if this script
dies. The script always sends zero velocity on exit (normal end, Ctrl-C or error) - keep the e-stop in
reach and keep the area clear: the robot spins in place with the spray gantry on its right side.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Unwrapper:
    """Accumulates a yaw angle across the +-pi wrap."""
    def __init__(self):
        self.last = None
        self.total = 0.0

    def feed(self, yaw):
        if self.last is not None:
            d = math.atan2(math.sin(yaw - self.last), math.cos(yaw - self.last))
            self.total += d
        self.last = yaw


class RotationTest(Node):
    def __init__(self, cmd_topic, odom_topic, imu_topic):
        super().__init__('rotation_test')
        self.pub = self.create_publisher(Twist, cmd_topic, 1)
        self.odom_yaw = Unwrapper(); self.odom_n = 0
        self.imu_yaw = Unwrapper();  self.imu_n = 0
        self.gyro = 0.0; self.gyro_last_t = None; self.gyro_n = 0
        self.recording = False
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=20)
        self.create_subscription(Imu, imu_topic, self._imu_cb, qos)

    def _odom_cb(self, msg):
        y = yaw_of(msg.pose.pose.orientation)
        if self.recording or self.odom_yaw.last is None:
            self.odom_yaw.feed(y); self.odom_n += 1

    def _imu_cb(self, msg):
        o = msg.orientation
        if not (o.x == 0.0 and o.y == 0.0 and o.z == 0.0 and o.w == 0.0):
            if self.recording or self.imu_yaw.last is None:
                self.imu_yaw.feed(yaw_of(o)); self.imu_n += 1
        t = time.time()
        if self.recording and self.gyro_last_t is not None:
            self.gyro += msg.angular_velocity.z * (t - self.gyro_last_t); self.gyro_n += 1
        self.gyro_last_t = t

    def send(self, angular):
        msg = Twist()
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def stop(self):
        for _ in range(10):
            self.send(0.0)
            time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--angle', type=float, default=360.0, help='commanded rotation in degrees (default 360)')
    ap.add_argument('--speed', type=float, default=0.4, help='angular speed in rad/s, sign = direction (default +0.4)')
    ap.add_argument('--track', type=float, default=0.625, help='track_width_m currently in actuator_bridge_params.yaml')
    ap.add_argument('--rate', type=float, default=20.0, help='command publish rate in Hz')
    ap.add_argument('--topic', default='/paint_bot/cmd_vel')
    ap.add_argument('--odom-topic', default='/paint_bot/odom')
    ap.add_argument('--imu-topic', default='/imu/out')
    ap.add_argument('--no-confirm', action='store_true', help='start without waiting for Enter')
    args = ap.parse_args()

    w = abs(args.speed)
    if w < 0.05 or w > 1.0:
        raise SystemExit('Refusing: |speed| must be between 0.05 and 1.0 rad/s.')
    angle_cmd = math.radians(abs(args.angle))
    duration = angle_cmd / w
    if duration > 90.0:
        raise SystemExit(f'Refusing: {duration:.0f} s is longer than 90 s. Use a higher speed or a smaller angle.')
    direction = 1.0 if args.speed >= 0 else -1.0

    rclpy.init()
    node = RotationTest(args.topic, args.odom_topic, args.imu_topic)
    try:
        t_wait = time.time()
        while time.time() - t_wait < 2.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f'Odometry samples: {"yes" if node.odom_n else "NO"}   IMU orientation: '
              f'{"yes" if node.imu_n else "no"}   IMU gyro: {"yes" if node.gyro_last_t else "no"}')
        print(f'Will rotate {direction * args.angle:+.0f} deg in place: {args.speed:+.3f} rad/s for {duration:.1f} s '
              f'(current track_width_m = {args.track}).')
        if not args.no_confirm:
            input('Clear the area, mark the robot heading on the floor, then press Enter to go (Ctrl-C to abort)... ')

        node.recording = True
        t0 = time.time()
        next_t = t0
        period = 1.0 / args.rate
        while time.time() - t0 < duration:
            node.send(args.speed)
            rclpy.spin_once(node, timeout_sec=0.0)
            next_t += period
            time.sleep(max(0.0, next_t - time.time()))
        t_drive = time.time() - t0
        node.stop()
        t_settle = time.time()
        while time.time() - t_settle < 1.5:           # let it coast to a stop and catch the last samples
            rclpy.spin_once(node, timeout_sec=0.05)
        node.recording = False

        commanded = direction * args.angle
        print(f'\nRotated for {t_drive:.2f} s, stop sent.')
        print(f'Commanded rotation       : {commanded:+.1f} deg')
        if node.odom_n > 1:
            print(f'Odometry yaw change      : {math.degrees(node.odom_yaw.total):+.1f} deg   (same track as the command: will read close to commanded)')
        if node.imu_n > 1:
            imu_deg = math.degrees(node.imu_yaw.total)
            if abs(imu_deg) > 10:
                odom_deg = math.degrees(node.odom_yaw.total) if node.odom_n > 1 else None
                line = f'IMU orientation change   : {imu_deg:+.1f} deg'
                if odom_deg:
                    line += f'   -> track that makes odometry match the IMU: {args.track * abs(odom_deg / imu_deg):.4f}'
                line += f'   (command-based: {args.track * abs(commanded / imu_deg):.4f})'
                print(line)
            else:
                print(f'IMU orientation change   : {imu_deg:+.1f} deg   (too small to use)')
        if node.gyro_n > 1:
            g_deg = math.degrees(node.gyro)
            print(f'IMU gyro-z integrated    : {g_deg:+.1f} deg   -> suggested track {args.track * abs(commanded / g_deg):.4f}' if abs(g_deg) > 10 else
                  f'IMU gyro-z integrated    : {g_deg:+.1f} deg   (too small to use)')
        print('\nNow measure the REAL rotation on the floor (the turn is open loop, so only the real angle counts).')
        print('Give the final heading error against your start mark: + if it ended PAST the mark, - if SHORT of it.')
        try:
            s = input('Heading error in degrees (e.g. +6 or -4), or press Enter to skip: ').strip()
        except EOFError:
            s = ''
        if s:
            real = commanded + float(s) * (1.0 if commanded >= 0 else -1.0)
            print(f'\nReal rotation = commanded {commanded:+.1f} + error {float(s):+.1f} = {real:+.1f} deg')
            if node.odom_n > 1 and abs(math.degrees(node.odom_yaw.total)) > 10:
                odom_deg = math.copysign(abs(math.degrees(node.odom_yaw.total)), commanded)
                new_track = args.track * odom_deg / real
                print(f'  new track_width_m = {args.track} x odometry {odom_deg:+.1f} / real {real:+.1f} = {new_track:.4f}'
                      '   (makes the odometry heading, which the turn controller uses, equal the real heading)')
            else:
                new_track = args.track * commanded / real
                print(f'  new track_width_m = {args.track} x {commanded:+.0f} / {real:+.1f} = {new_track:.4f}')
            print('  One run is noisy (+-1-3 %): repeat 3 times and average before editing actuator_bridge_params.yaml.')
    except KeyboardInterrupt:
        print('\nAborted - stopping the robot.')
    finally:
        try:
            if rclpy.ok():
                node.stop()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
