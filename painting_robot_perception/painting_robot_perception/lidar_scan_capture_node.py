#!/usr/bin/env python3
"""
lidar_scan_capture_node.py
==========================
Part of: painting_robot_perception

Drives the robot forward at a fixed speed while capturing LiDAR scans
at regular intervals. Pose from /odometry/filtered is logged with every capture.

Stop condition: robot halts after TRAVEL_DIST_LIM metres (odometry-tracked).

Saves to:
    ~/ros_ws/painting_robot_ws/captured_data/lidar_scans/run_<timestamp>/
        scans/scan_NNNN.npy      ← ranges + scan metadata dict
        scan_log.csv             ← pose + scan stats per capture

Each .npy file is a dict with keys matching the existing notebook loader:
    {
        'ranges'         : list[float],   # full 360° range array
        'angle_min'      : float,         # radians
        'angle_max'      : float,         # radians
        'angle_increment': float,         # radians per beam
        'range_min'      : float,         # m
        'range_max'      : float,         # m
        'scan_index'     : int,
        'odom_x'         : float,         # robot x in odom frame
        'odom_y'         : float,         # robot y in odom frame
        'yaw_rad'        : float,         # robot heading
        'dist_travelled' : float,         # metres from start
    }
"""

import os
import csv
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from datetime import datetime

# ── Parameters ────────────────────────────────────────────────────────────────
FWD_SPEED        = 0.1          # m/s — slow approach toward stub
CAPTURE_INTERVAL = 1.0           # seconds between scan saves
TRAVEL_DIST_LIM  = 1.5           # metres — stop after this distance

BASE_DIR = os.path.expanduser(
    '~/ros_ws/painting_robot_ws/captured_data/lidar_scans'
)

SCAN_TOPIC   = '/scan'
ODOM_TOPIC   = '/odometry/filtered'
CMDVEL_TOPIC = '/diff_cont/cmd_vel_unstamped'
# ──────────────────────────────────────────────────────────────────────────────


class LidarScanCaptureNode(Node):

    def __init__(self):
        super().__init__('lidar_scan_capture_node')

        # ── Output dirs ───────────────────────────────────────────────────────
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        run_dir       = os.path.join(BASE_DIR, f'run_{ts}')
        self.scan_dir = os.path.join(run_dir, 'scans')
        os.makedirs(self.scan_dir, exist_ok=True)

        # ── State ─────────────────────────────────────────────────────────────
        self.latest_scan   = None   # raw LaserScan msg
        self.capture_index = 0
        self.stopped       = False

        self.start_x   = None
        self.start_y   = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # ── CSV log ───────────────────────────────────────────────────────────
        csv_path    = os.path.join(run_dir, 'scan_log.csv')
        self._csv   = open(csv_path, 'w', newline='')
        self._writer = csv.writer(self._csv)
        self._writer.writerow([
            'idx', 'odom_x', 'odom_y', 'yaw_rad', 'dist_travelled_m',
            'range_min_m', 'range_max_m', 'valid_beams', 'total_beams'
        ])

        # ── ROS interfaces ────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, CMDVEL_TOPIC, 10)

        self.create_subscription(LaserScan, SCAN_TOPIC, self._scan_cb,  10)
        self.create_subscription(Odometry,  ODOM_TOPIC, self._odom_cb,  10)

        # ── Capture timer ─────────────────────────────────────────────────────
        self.create_timer(CAPTURE_INTERVAL, self._capture_cb)

        self.get_logger().info(
            f'LidarScanCaptureNode ready\n'
            f'  speed={FWD_SPEED} m/s  '
            f'interval={CAPTURE_INTERVAL}s  '
            f'limit={TRAVEL_DIST_LIM}m\n'
            f'  output → {run_dir}'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.current_x = pos.x
        self.current_y = pos.y

        q    = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

        if self.start_x is None:
            self.start_x = pos.x
            self.start_y = pos.y
            self.get_logger().info(
                f'Origin locked: ({self.start_x:.3f}, {self.start_y:.3f})'
            )
            return

        dist = math.hypot(pos.x - self.start_x, pos.y - self.start_y)
        if not self.stopped and dist >= TRAVEL_DIST_LIM:
            self._stop()

    # def _capture_cb(self):
    #     if self.stopped:
    #         return

    #     # ── Drive forward ─────────────────────────────────────────────────────
    #     cmd = Twist()
    #     cmd.linear.x = FWD_SPEED
    #     self._cmd_pub.publish(cmd)

    #     # ── Wait for data ─────────────────────────────────────────────────────
    #     if self.latest_scan is None:
    #         self.get_logger().warning('Waiting for /scan...')
    #         return

    #     if self.start_x is None:
    #         self.get_logger().warning('Waiting for odometry...')
    #         return

    #     # ── Distance travelled ────────────────────────────────────────────────
    #     dist = math.hypot(
    #         self.current_x - self.start_x,
    #         self.current_y - self.start_y
    #     )

    #     # ── Build scan dict — matches notebook loader format ──────────────────
    #     msg  = self.latest_scan
    #     ranges = list(msg.ranges)   # full 360° array

    #     scan_dict = {
    #         'ranges'         : ranges,
    #         'angle_min'      : float(msg.angle_min),
    #         'angle_max'      : float(msg.angle_max),
    #         'angle_increment': float(msg.angle_increment),
    #         'range_min'      : float(msg.range_min),
    #         'range_max'      : float(msg.range_max),
    #         'scan_index'     : self.capture_index,
    #         'odom_x'         : self.current_x,
    #         'odom_y'         : self.current_y,
    #         'yaw_rad'        : self.current_yaw,
    #         'dist_travelled' : dist,
    #     }

    #     # ── Save .npy ─────────────────────────────────────────────────────────
    #     idx      = self.capture_index
    #     out_path = os.path.join(self.scan_dir, f'scan_{idx:04d}.npy')
    #     np.save(out_path, scan_dict)

    #     # ── Scan stats for CSV ────────────────────────────────────────────────
    #     r_arr     = np.array(ranges, dtype=np.float32)
    #     valid     = np.isfinite(r_arr) & (r_arr > msg.range_min) & (r_arr < msg.range_max)
    #     r_valid   = r_arr[valid]
    #     r_min     = float(r_valid.min())  if r_valid.size else 0.0
    #     r_max     = float(r_valid.max())  if r_valid.size else 0.0

    #     self._writer.writerow([
    #         idx,
    #         f'{self.current_x:.4f}', f'{self.current_y:.4f}',
    #         f'{self.current_yaw:.4f}',
    #         f'{dist:.4f}',
    #         f'{r_min:.3f}', f'{r_max:.3f}',
    #         int(valid.sum()), len(ranges)
    #     ])
    #     self._csv.flush()

    #     self.get_logger().info(
    #         f'[{idx:04d}] dist={dist:.2f}m  '
    #         f'pos=({self.current_x:.2f},{self.current_y:.2f})  '
    #         f'yaw={math.degrees(self.current_yaw):.1f}°  '
    #         f'valid_beams={valid.sum()}/{len(ranges)}  '
    #         f'range=[{r_min:.2f},{r_max:.2f}]m  '
    #         f'→ {os.path.basename(out_path)}'
    #     )

    #     self.capture_index += 1

    # drive 
    def _capture_cb(self):
        if self.stopped:
            return

        # ── Drive forward ─────────────────────────────────────────────────────
        # BENCH: driving disabled
        # cmd = Twist()
        # cmd.linear.x = FWD_SPEED
        # self._cmd_pub.publish(cmd)

        # ── Wait for data ─────────────────────────────────────────────────────
        if self.latest_scan is None:
            self.get_logger().warning('Waiting for /scan...')
            return

        # BENCH: no odom — fake origin so capture proceeds
        if self.start_x is None:
            self.start_x = 0.0
            self.start_y = 0.0

        # ── Distance travelled ────────────────────────────────────────────────
        dist = math.hypot(
            self.current_x - self.start_x,
            self.current_y - self.start_y
        )

        # ── Build scan dict — matches notebook loader format ──────────────────
        msg  = self.latest_scan
        ranges = list(msg.ranges)   # full 360° array

        scan_dict = {
            'ranges'         : ranges,
            'angle_min'      : float(msg.angle_min),
            'angle_max'      : float(msg.angle_max),
            'angle_increment': float(msg.angle_increment),
            'range_min'      : float(msg.range_min),
            'range_max'      : float(msg.range_max),
            'scan_index'     : self.capture_index,
            'odom_x'         : self.current_x,
            'odom_y'         : self.current_y,
            'yaw_rad'        : self.current_yaw,
            'dist_travelled' : dist,
        }

        # ── Save .npy ─────────────────────────────────────────────────────────
        idx      = self.capture_index
        out_path = os.path.join(self.scan_dir, f'scan_{idx:04d}.npy')
        np.save(out_path, scan_dict)

        # ── Scan stats for CSV ────────────────────────────────────────────────
        r_arr     = np.array(ranges, dtype=np.float32)
        valid     = np.isfinite(r_arr) & (r_arr > msg.range_min) & (r_arr < msg.range_max)
        r_valid   = r_arr[valid]
        r_min     = float(r_valid.min())  if r_valid.size else 0.0
        r_max     = float(r_valid.max())  if r_valid.size else 0.0

        self._writer.writerow([
            idx,
            f'{self.current_x:.4f}', f'{self.current_y:.4f}',
            f'{self.current_yaw:.4f}',
            f'{dist:.4f}',
            f'{r_min:.3f}', f'{r_max:.3f}',
            int(valid.sum()), len(ranges)
        ])
        self._csv.flush()

        self.get_logger().info(
            f'[{idx:04d}] valid_beams={valid.sum()}/{len(ranges)}  '
            f'range=[{r_min:.2f},{r_max:.2f}]m  → {os.path.basename(out_path)}'
        )

        self.capture_index += 1

    # ── Stop ───────────────────────────────────────────────────────────────────

    def _stop(self):
        self.stopped = True
        self._cmd_pub.publish(Twist())   # zero velocity
        self._csv.close()
        self.get_logger().info(
            f'Travel limit reached — {self.capture_index} scans captured. '
            f'Node stopped.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()