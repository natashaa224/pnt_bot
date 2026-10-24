#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class Odom90Test(Node):

    def __init__(self):
        super().__init__('odom_90_test')

        # Change this between tests:
        self.declare_parameter('odom_topic', '/paint_bot/odom')

        self.declare_parameter('cmd_topic', '/paint_bot/cmd_vel')

        # CW
        self.declare_parameter('angular_speed', -0.15)

        self.odom_topic = (
            self.get_parameter('odom_topic')
            .get_parameter_value()
            .string_value
        )

        self.cmd_topic = (
            self.get_parameter('cmd_topic')
            .get_parameter_value()
            .string_value
        )

        self.angular_speed = (
            self.get_parameter('angular_speed')
            .get_parameter_value()
            .double_value
        )

        self.pub = self.create_publisher(
            Twist,
            self.cmd_topic,
            10
        )

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            10
        )

        self.start_yaw = None
        self.finished = False

        self.get_logger().info(
            f'90-deg test using {self.odom_topic}'
        )

    def stop(self):
        msg = Twist()

        # Send several zeros because this is a standalone test
        for _ in range(5):
            self.pub.publish(msg)

    def odom_cb(self, msg):

        if self.finished:
            self.stop()
            return

        yaw = yaw_from_quat(msg.pose.pose.orientation)

        if self.start_yaw is None:
            self.start_yaw = yaw

            self.get_logger().info(
                f'Start yaw = '
                f'{math.degrees(yaw):.2f} deg'
            )

            return

        delta = wrap_angle(yaw - self.start_yaw)

        # CW => negative
        travelled = abs(delta)

        self.get_logger().info(
            f'Yaw travelled = '
            f'{math.degrees(travelled):.2f} deg',
            throttle_duration_sec=0.5
        )

        target = math.radians(90.0)

        if travelled >= target:

            self.stop()
            self.finished = True

            self.get_logger().info(
                '=============================='
            )

            self.get_logger().info(
                f'ODOM SAYS 90 DEG COMPLETE'
            )

            self.get_logger().info(
                f'Start yaw : '
                f'{math.degrees(self.start_yaw):.2f}'
            )

            self.get_logger().info(
                f'End yaw   : '
                f'{math.degrees(yaw):.2f}'
            )

            self.get_logger().info(
                f'Delta     : '
                f'{math.degrees(travelled):.2f}'
            )

            self.get_logger().info(
                'MEASURE PHYSICAL ROBOT ANGLE NOW'
            )

            self.get_logger().info(
                '=============================='
            )

            return

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = self.angular_speed

        self.pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = Odom90Test()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()