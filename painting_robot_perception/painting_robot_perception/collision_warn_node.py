# VRai
# Front-wall collision warning.
# Watches /front_wall_pose; if front wall closer than threshold, reports a
# fault to safety_monitor. Mode-agnostic (fires in IDLE and AUTO alike).

import rclpy
from rclpy.node import Node
from painting_robot_msgs.msg import WallPose
from std_msgs.msg import String


class CollisionWarnNode(Node):

    def __init__(self):
        super().__init__('collision_warn_node')

        self.declare_parameter('warn_distance', 0.5)      # m
        self.declare_parameter('clear_distance', 0.6)     # m (hysteresis)
        self.declare_parameter('fault_code', 'COLLISION_WARN')
        self.warn_dist  = self.get_parameter('warn_distance').value
        self.clear_dist = self.get_parameter('clear_distance').value
        self.code       = self.get_parameter('fault_code').value

        self._active = False   # edge guard — report/clear only on transition

        self.create_subscription(WallPose, '/front_wall_pose', self._front_cb, 10)
        self._fault_pub = self.create_publisher(String, '/fault_report', 10)

        self.get_logger().info(
            f'CollisionWarnNode started — warn<{self.warn_dist:.2f}m '
            f'clear>{self.clear_dist:.2f}m')

    def _front_cb(self, msg):
        if not msg.valid:
            return   # no front wall seen — don't touch latch

        d = msg.distance

        if not self._active and d < self.warn_dist:
            self._active = True
            self._report(self.code)
            self.get_logger().warn(
                f'COLLISION WARN — front wall {d:.2f}m < {self.warn_dist:.2f}m')

        elif self._active and d > self.clear_dist:
            self._active = False
            self._report(f'CLEAR:{self.code}')
            self.get_logger().info(
                f'collision warn cleared — front wall {d:.2f}m')

    def _report(self, payload):
        m = String()
        m.data = payload
        self._fault_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionWarnNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
