"""
scan_capture.py  —  single-shot LiDAR scan capture
Saves one scan + pose to:
    ~/ros_ws/painting_robot_ws/captured_data/single_capture/run_YYYYMMDD_HHMMSS/scan.npy

Pose source (in priority order):
  1. TF: base_footprint → map  (live odometry, most accurate)
  2. ROS param  --ros-args -p spawn_x:=<x> -p spawn_y:=<y> -p spawn_yaw:=<yaw>
  3. Hardcoded fallback below

Usage:
  ros2 run painting_robot_perception scan_capture
  # or with pose override:
  ros2 run painting_robot_perception scan_capture \
      --ros-args -p spawn_x:=-12.25 -p spawn_y:=12.1 -p spawn_yaw:=3.1416
"""

import os
import math
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

# ── Output root ───────────────────────────────────────────────────────────────
BASE_DIR = os.path.expanduser(
    '~/ros_ws/painting_robot_ws/captured_data/single_capture'
)

# ── Hardcoded fallback pose (used only if TF and params both unavailable) ─────
FALLBACK_X   = 2.55
FALLBACK_Y   =  7
FALLBACK_YAW = 1.5* math.pi   # radians



class ScanCaptureNode(Node):

    def __init__(self):
        super().__init__('scan_capture_node')

        # ── Declare pose params (can be set from launch or --ros-args) ────────
        self.declare_parameter('spawn_x',   FALLBACK_X)
        self.declare_parameter('spawn_y',   FALLBACK_Y)
        self.declare_parameter('spawn_yaw', FALLBACK_YAW)

        # ── TF listener ───────────────────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Output directory ──────────────────────────────────────────────────
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._run_dir = os.path.join(BASE_DIR, f'run_{ts}')
        os.makedirs(self._run_dir, exist_ok=True)
        self.get_logger().info(f'[CAPTURE] run dir: {self._run_dir}')

        # ── Subscribe to scan — fires once then shuts down ────────────────────
        self._sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)

    # ─────────────────────────────────────────────────────────────────────────

    def _get_pose(self):
        """Try TF first, fall back to params, then hardcoded fallback."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            t = tf.transform.translation
            q = tf.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw  = math.atan2(siny, cosy)
            self.get_logger().info('[CAPTURE] pose from TF (map→base_footprint)')
            return float(t.x), float(t.y), float(yaw), 'tf'
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f'[CAPTURE] TF unavailable ({e}) — trying params')

        # Param fallback
        x   = self.get_parameter('spawn_x').value
        y   = self.get_parameter('spawn_y').value
        yaw = self.get_parameter('spawn_yaw').value

        # Check if params were actually set (differ from fallback defaults)
        source = 'params' if (x != FALLBACK_X or y != FALLBACK_Y or yaw != FALLBACK_YAW) \
                 else 'hardcoded_fallback'
        self.get_logger().info(f'[CAPTURE] pose from {source}')
        return float(x), float(y), float(yaw), source

    def _scan_cb(self, msg: LaserScan):
        x, y, yaw, pose_source = self._get_pose()

        data = {
            'robot_x':         x,
            'robot_y':         y,
            'robot_yaw':       yaw,
            'robot_yaw_deg':   math.degrees(yaw),
            'pose_source':     pose_source,
            'ranges':          list(msg.ranges),
            'angle_min':       msg.angle_min,
            'angle_max':       msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_min':       msg.range_min,
            'range_max':       msg.range_max,
            'timestamp':       datetime.now().isoformat(),
        }

        out_path = os.path.join(self._run_dir, 'scan.npy')
        np.save(out_path, data)

        self.get_logger().info(
            f'[CAPTURE] saved — '
            f'pos=({x:.3f}, {y:.3f}) '
            f'yaw={math.degrees(yaw):.1f}° '
            f'source={pose_source} '
            f'→ {out_path}')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ScanCaptureNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()