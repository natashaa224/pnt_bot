#!/usr/bin/env python3
"""
Paint Demo Node
Runs a preset painting sequence:
  1. Drive one strip length forward
  2. Raise gantry to top
  3. Lower gantry to bottom  (simulates spray pass)
  4. Repeat

Run after launching the robot + controllers:
  ros2 run <your_pkg> paint_demo_node.py
  or just: python3 paint_demo_node.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

# ---------------------------------------------------------------------------
# Tunable params
# ---------------------------------------------------------------------------
STRIP_LENGTH_M      = 0.2       # metres per strip
DRIVE_SPEED_MPS     = 0.1       # m/s forward speed
GANTRY_TOP_M        = 1.0       # zAxis_joint upper position (m)
GANTRY_BOTTOM_M     = 0.0       # zAxis_joint lower position (m)
SPRAY_GUN_POS_M     = 0.0       # wrist_joint — fixed for now
GANTRY_MOVE_TIME_S  = 6         # seconds to complete gantry travel
STRIP_DRIVE_TIME_S  = STRIP_LENGTH_M / DRIVE_SPEED_MPS
NUM_STRIPS          = 4


class PaintDemoNode(Node):

    def __init__(self):
        super().__init__('paint_demo_node')

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/paint_bot/cmd_vel', 10)

        self.gantry_pub = self.create_publisher(
            JointTrajectory, '/paint_axis_controller/joint_trajectory', 10)

        self.get_logger().info('Paint demo node ready. Starting sequence...')

    # ------------------------------------------------------------------
    # Motion primitives
    # ------------------------------------------------------------------
    def drive_forward(self, duration_s: float):
        self.get_logger().info(f'Driving forward {duration_s:.1f}s')
        msg = Twist()
        msg.linear.x = DRIVE_SPEED_MPS
        end = time.time() + duration_s
        while time.time() < end:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.1)
        self._stop_drive()

    def _stop_drive(self):
        self.cmd_vel_pub.publish(Twist())   # zero twist = stop
        time.sleep(0.3)

    def move_gantry(self, height_m: float, duration_s: int = GANTRY_MOVE_TIME_S):
        self.get_logger().info(f'Moving gantry to {height_m:.2f}m')
        msg = JointTrajectory()
        msg.joint_names = ['zAxis_joint', 'wrist_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [height_m, SPRAY_GUN_POS_M]
        pt.time_from_start = Duration(sec=duration_s, nanosec=0)
        msg.points = [pt]
        self.gantry_pub.publish(msg)
        time.sleep(duration_s + 0.5)    # wait for move to complete + buffer

    # ------------------------------------------------------------------
    # Sequence
    # ------------------------------------------------------------------
    def run_sequence(self):
        time.sleep(1.0)     # let publishers settle

        for i in range(NUM_STRIPS):
            self.get_logger().info(f'--- Strip {i+1}/{NUM_STRIPS} ---')

            # 1. Drive one strip
            self.drive_forward(STRIP_DRIVE_TIME_S)

            # 2. Raise gantry
            self.move_gantry(GANTRY_TOP_M)

            # 3. Lower gantry (simulates spray pass)
            self.move_gantry(GANTRY_BOTTOM_M)

        self.get_logger().info('Sequence complete.')


def main():
    rclpy.init()
    node = PaintDemoNode()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
