#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class Arc90Test(Node):

    def __init__(self):
        super().__init__('arc_90_test')

        # ----------------------------------------------------
        # PARAMETERS
        # ----------------------------------------------------
        self.declare_parameter('odom_topic', '/paint_bot/odom')
        self.declare_parameter('cmd_topic', '/paint_bot/cmd_vel')

        self.declare_parameter('radius', 0.90)
        self.declare_parameter('linear_speed', 0.15)

        # -1 = CW, +1 = CCW
        self.declare_parameter('turn_dir', -1)

        self.declare_parameter('target_angle_deg', 90.0)

        self.odom_topic = self.get_parameter(
            'odom_topic').value

        self.cmd_topic = self.get_parameter(
            'cmd_topic').value

        self.radius = float(
            self.get_parameter('radius').value)

        self.linear_speed = float(
            self.get_parameter('linear_speed').value)

        self.turn_dir = int(
            self.get_parameter('turn_dir').value)

        self.target_angle = math.radians(
            float(self.get_parameter('target_angle_deg').value)
        )

        # ----------------------------------------------------
        # STATE
        # ----------------------------------------------------
        self.start_yaw = None
        self.start_x = None
        self.start_y = None

        self.current_yaw = None
        self.current_x = None
        self.current_y = None

        self.finished = False

        # ----------------------------------------------------
        # ROS
        # ----------------------------------------------------
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

        # Commands at 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        # Allows:
        # ros2 param set /arc_90_test radius 0.85
        self.add_on_set_parameters_callback(
            self.parameter_callback
        )

        self.get_logger().info(
            '===================================='
        )
        self.get_logger().info(
            'ARC 90 TEST'
        )
        self.get_logger().info(
            f'odom       = {self.odom_topic}'
        )
        self.get_logger().info(
            f'cmd_vel    = {self.cmd_topic}'
        )
        self.get_logger().info(
            f'R          = {self.radius:.3f} m'
        )
        self.get_logger().info(
            f'v          = {self.linear_speed:.3f} m/s'
        )
        self.get_logger().info(
            f'omega      = '
            f'{self.turn_dir * self.linear_speed / self.radius:.4f} rad/s'
        )
        self.get_logger().info(
            f'direction  = '
            f'{"CCW" if self.turn_dir > 0 else "CW"}'
        )
        self.get_logger().info(
            'Waiting for first odometry message...'
        )
        self.get_logger().info(
            '===================================='
        )

    # --------------------------------------------------------
    # DYNAMIC PARAMETERS
    # --------------------------------------------------------

    def parameter_callback(self, params):

        for p in params:

            if p.name == 'radius':

                if p.value <= 0.05:
                    return SetParametersResult(
                        successful=False,
                        reason='radius must be > 0.05 m'
                    )

                self.radius = float(p.value)

                self.get_logger().warn(
                    f'RADIUS CHANGED -> '
                    f'{self.radius:.3f} m'
                )

            elif p.name == 'linear_speed':

                if p.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='linear_speed must be > 0'
                    )

                self.linear_speed = float(p.value)

                self.get_logger().warn(
                    f'LINEAR SPEED CHANGED -> '
                    f'{self.linear_speed:.3f} m/s'
                )

        return SetParametersResult(
            successful=True
        )

    # --------------------------------------------------------
    # ODOM
    # --------------------------------------------------------

    def odom_cb(self, msg):

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quat(
            msg.pose.pose.orientation
        )

        if self.start_yaw is None:

            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw

            self.get_logger().info(
                f'START pose: '
                f'x={self.start_x:.3f} '
                f'y={self.start_y:.3f} '
                f'yaw={math.degrees(self.start_yaw):.2f} deg'
            )

    # --------------------------------------------------------
    # CONTROL
    # --------------------------------------------------------

    def control_loop(self):

        if self.finished:
            self.publish_stop()
            return

        if self.start_yaw is None:
            return

        delta = wrap_angle(
            self.current_yaw - self.start_yaw
        )

        # Signed progress in intended turn direction
        progress = self.turn_dir * delta

        if progress < 0:
            progress = 0.0

        # ----------------------------------------------------
        # STOP AT ODOM 90 DEG
        # ----------------------------------------------------

        if progress >= self.target_angle:

            self.finished = True

            self.publish_stop()

            self.report_result(
                progress
            )

            return

        # ----------------------------------------------------
        # CONSTANT-RADIUS ARC
        # ----------------------------------------------------

        omega = (
            self.turn_dir
            * self.linear_speed
            / self.radius
        )

        cmd = Twist()

        cmd.linear.x = self.linear_speed
        cmd.angular.z = omega

        self.pub.publish(cmd)

        self.get_logger().info(
            f'yaw={math.degrees(progress):5.1f} deg  '
            f'R={self.radius:.3f}  '
            f'v={self.linear_speed:.3f}  '
            f'w={omega:+.4f}',
            throttle_duration_sec=0.5
        )

    # --------------------------------------------------------
    # RESULT
    # --------------------------------------------------------

    def report_result(self, progress):

        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y

        chord = math.hypot(dx, dy)

        angle = progress

        if abs(math.sin(angle / 2.0)) > 1e-6:

            inferred_radius = (
                chord
                / (2.0 * math.sin(angle / 2.0))
            )

        else:
            inferred_radius = float('nan')

        expected_chord = (
            2.0
            * self.radius
            * math.sin(self.target_angle / 2.0)
        )

        self.get_logger().info(
            ''
        )

        self.get_logger().info(
            '===================================='
        )

        self.get_logger().info(
            'ODOM SAYS ARC COMPLETE'
        )

        self.get_logger().info(
            f'Commanded radius     = '
            f'{self.radius:.4f} m'
        )

        self.get_logger().info(
            f'Yaw change           = '
            f'{math.degrees(progress):.2f} deg'
        )

        self.get_logger().info(
            f'Start odom           = '
            f'({self.start_x:.3f}, {self.start_y:.3f})'
        )

        self.get_logger().info(
            f'End odom             = '
            f'({self.current_x:.3f}, {self.current_y:.3f})'
        )

        self.get_logger().info(
            f'Odom chord           = '
            f'{chord:.4f} m'
        )

        self.get_logger().info(
            f'Expected chord       = '
            f'{expected_chord:.4f} m'
        )

        self.get_logger().info(
            f'Odom inferred radius = '
            f'{inferred_radius:.4f} m'
        )

        self.get_logger().info(
            ''
        )

        self.get_logger().info(
            'NOW MEASURE PHYSICALLY:'
        )

        self.get_logger().info(
            '1. Actual robot heading'
        )

        self.get_logger().info(
            '2. Turn-axis START -> END chord'
        )

        self.get_logger().info(
            ''
        )

        self.get_logger().info(
            'For physical 90 deg:'
        )

        self.get_logger().info(
            'R_physical = chord / 1.41421356'
        )

        self.get_logger().info(
            '===================================='
        )

    # --------------------------------------------------------

    def publish_stop(self):

        msg = Twist()

        # one timer tick is normally enough, but standalone
        # testing benefits from repeated zeros
        self.pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = Arc90Test()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        for _ in range(10):
            node.publish_stop()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
