#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    # yaw from geometry_msgs/Quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def deg(rad):
    return rad * 180.0 / math.pi


class YawMonitor(Node):
    def __init__(self):
        super().__init__('yaw_monitor')

        self.imu_msg = None
        self.odom_msg = None
        self.ekf_msg = None

        self.create_subscription(Imu, '/imu/out', self.imu_cb, 10)
        self.create_subscription(Odometry, '/paint_bot/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/paint_bot/odom_filtered', self.ekf_cb, 10)

        self.create_timer(1.0, self.print_status)

        self.get_logger().info('Yaw monitor running at 1 Hz')

    def imu_cb(self, msg):
        self.imu_msg = msg

    def odom_cb(self, msg):
        self.odom_msg = msg

    def ekf_cb(self, msg):
        self.ekf_msg = msg

    def fmt_imu(self):
        if self.imu_msg is None:
            return 'IMU:  no data'

        q = self.imu_msg.orientation
        wz = self.imu_msg.angular_velocity.z
        yaw = deg(yaw_from_quat(q))

        return (
            f'IMU:  yaw={yaw:8.2f} deg | '
            f'wz={wz:8.4f} rad/s | '
            f'qz={q.z: .4f} qw={q.w: .4f}'
        )

    def fmt_odom(self):
        if self.odom_msg is None:
            return 'ODOM: no data'

        q = self.odom_msg.pose.pose.orientation
        wz = self.odom_msg.twist.twist.angular.z
        yaw = deg(yaw_from_quat(q))

        return (
            f'ODOM: yaw={yaw:8.2f} deg | '
            f'wz={wz:8.4f} rad/s | '
            f'qz={q.z: .4f} qw={q.w: .4f}'
        )

    def fmt_ekf(self):
        if self.ekf_msg is None:
            return 'EKF:  no data'

        q = self.ekf_msg.pose.pose.orientation
        wz = self.ekf_msg.twist.twist.angular.z
        yaw = deg(yaw_from_quat(q))

        return (
            f'EKF:  yaw={yaw:8.2f} deg | '
            f'wz={wz:8.4f} rad/s | '
            f'qz={q.z: .4f} qw={q.w: .4f}'
        )

    def print_status(self):
        print('\n' + '-' * 95)
        print(self.fmt_imu())
        print(self.fmt_odom())
        print(self.fmt_ekf())


def main():
    rclpy.init()
    node = YawMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()