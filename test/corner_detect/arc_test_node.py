#!/usr/bin/env python3
"""
arc_test_node.py — standalone external-corner arc tester (no FSM, no env_monitoring).

Spawn the bot in line with the vertex (turn axis at the vertex, heading along the
approach wall). This node then, in pure odom/turn-axis frame:

  1. CRAWL forward until the turn axis is PAST_VERTEX_DIST past the vertex.
  2. ARC: command lin + ang(=lin/RADIUS) until yaw has swept TURN_ANGLE.
  3. STOP, fit a circle to the recorded turn-axis path -> measured radius.
  4. Report where the sprayhead (offset behind axis) ended up.

Everything is turn-axis frame. Convert to lidar-frame trigger AFTER this works.

Tunables at top. Run against a bare spawn:
    ros2 run <pkg> arc_test_node
or
    python3 arc_test_node.py
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np

# ─── TUNABLES ───────────────────────────────────────────────
RADIUS           = 0.8    # arc radius (turn-axis), m   -> ang = lin/RADIUS
PAST_VERTEX_DIST = 0.104  # how far past vertex to start the turn, m
LIN_SPEED        = 0.15    # forward speed during crawl and arc, m/s
TURN_ANGLE_DEG   = 90.0    # arc sweep, deg
TURN_DIR         = -1      # -1 = CW (yaw decreases), +1 = CCW
SPRAY_OFFSET     = 0.60    # sprayhead distance behind turn axis, m (for endpoint report)
ODOM_TOPIC       = '/paint_bot/odom'
CMD_TOPIC        = '/paint_bot/cmd_vel'   # adjust to your cmd_vel topic
# ────────────────────────────────────────────────────────────


class ArcTestNode(Node):
    def __init__(self):
        super().__init__('arc_test_node')
        self._cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, 10)

        self._x = self._y = self._yaw = None
        self._start_x = self._start_y = None       # spawn (= vertex) position
        self._turn_start_yaw = None
        self._phase = 'INIT'                        # INIT -> CRAWL -> ARC -> DONE
        self._path = []                             # recorded (x,y) during ARC

        self._ang = LIN_SPEED / RADIUS              # arc angular rate
        self._sweep = math.radians(TURN_ANGLE_DEG)

        self.create_timer(0.05, self._loop)         # 20 Hz control
        self.get_logger().info(
            f'arc_test: R={RADIUS} past_vertex={PAST_VERTEX_DIST} '
            f'lin={LIN_SPEED} ang={self._ang:.4f} dir={"CW" if TURN_DIR<0 else "CCW"}')

    def _odom_cb(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # yaw from quaternion (flat)
        self._yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                               1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def _pub(self, lin, ang):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self._cmd_pub.publish(t)

    def _loop(self):
        if self._x is None:
            return   # no odom yet

        # latch spawn = vertex
        if self._phase == 'INIT':
            self._start_x, self._start_y = self._x, self._y
            self._phase = 'CRAWL'
            self.get_logger().info(
                f'[CRAWL] vertex latched at ({self._start_x:.3f},{self._start_y:.3f}) '
                f'-> crawl {PAST_VERTEX_DIST}m past')
            return

        if self._phase == 'CRAWL':
            travelled = math.hypot(self._x - self._start_x, self._y - self._start_y)
            if travelled >= PAST_VERTEX_DIST:
                self._pub(0.0, 0.0)
                self._turn_start_yaw = self._yaw
                self._phase = 'ARC'
                self.get_logger().info(
                    f'[ARC] start at ({self._x:.3f},{self._y:.3f}) '
                    f'yaw={math.degrees(self._yaw):.1f}° — arcing')
            else:
                self._pub(LIN_SPEED, 0.0)
            return

        if self._phase == 'ARC':
            self._path.append((self._x, self._y))
            delta = self._angle_diff(self._yaw, self._turn_start_yaw)
            if abs(delta) >= self._sweep:
                self._pub(0.0, 0.0)
                self._phase = 'DONE'
                self._report()
            else:
                self._pub(LIN_SPEED, TURN_DIR * self._ang)
            return

        if self._phase == 'DONE':
            self._pub(0.0, 0.0)

    @staticmethod
    def _angle_diff(a, b):
        d = a - b
        while d >  math.pi: d -= 2*math.pi
        while d < -math.pi: d += 2*math.pi
        return d

    def _fit_circle(self, pts):
        """Least-squares circle fit. Returns (cx, cy, R)."""
        p = np.array(pts)
        x, y = p[:,0], p[:,1]
        A = np.column_stack([2*x, 2*y, np.ones(len(x))])
        b = x*x + y*y
        cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
        R = math.sqrt(c + cx*cx + cy*cy)
        return cx, cy, R

    def _report(self):
        if len(self._path) < 5:
            self.get_logger().warn('[DONE] not enough path points to fit')
            return
        cx, cy, R_meas = self._fit_circle(self._path)
        # sprayhead endpoint: behind the axis by SPRAY_OFFSET along -heading
        ex, ey = self._path[-1]
        sx = ex - SPRAY_OFFSET * math.cos(self._yaw)
        sy = ey - SPRAY_OFFSET * math.sin(self._yaw)
        self.get_logger().info('─────────── ARC TEST RESULT ───────────')
        self.get_logger().info(f'  commanded R      = {RADIUS:.3f} m')
        self.get_logger().info(f'  MEASURED  R      = {R_meas:.3f} m   (fit center {cx:.2f},{cy:.2f})')
        self.get_logger().info(f'  axis start->end  = ({self._path[0][0]:.2f},{self._path[0][1]:.2f}) '
                               f'-> ({ex:.2f},{ey:.2f})')
        self.get_logger().info(f'  sprayhead end    = ({sx:.2f},{sy:.2f})')
        self.get_logger().info(f'  final yaw        = {math.degrees(self._yaw):.1f}°')
        self.get_logger().info('───────────────────────────────────────')
        if abs(R_meas - RADIUS) > 0.05:
            self.get_logger().warn(
                f'  ** MEASURED R off by {R_meas-RADIUS:+.3f} m — execution not faithful **')
        else:
            self.get_logger().info('  ** R executed faithfully — arc is correct **')


def main(args=None):
    rclpy.init(args=args)
    node = ArcTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
