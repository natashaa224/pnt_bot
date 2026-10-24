#!/usr/bin/env python3
"""
Minimal /scan_cliff capture. Saves N frames to a single .npy then exits.
Usage:
    python3 capture_cliff_scans.py flat.npy 40
    python3 capture_cliff_scans.py step.npy 40
Reload:
    d = np.load('flat.npy', allow_pickle=True).item()
    d['ranges']          # (N, beams) float32, inf/0 preserved
    d['angle_min'], d['angle_increment']
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class CliffCapture(Node):
    def __init__(self, out_path, n_frames):
        super().__init__('cliff_capture')
        self.out_path = out_path
        self.n_frames = n_frames
        self.frames = []
        self.angle_min = []
        self.angle_inc = []
        self.meta = None
        self.create_subscription(LaserScan, '/scan_cliff', self._cb, 10)
        self.get_logger().info(f'capturing {n_frames} frames -> {out_path}')

    def _cb(self, msg):
        if self.meta is None:
            self.meta = (msg.range_min, msg.range_max)
        self.frames.append(np.asarray(msg.ranges, dtype=np.float32))
        # per-frame angle params — beam count varies, so don't assume one global set
        self.angle_min.append(float(msg.angle_min))
        self.angle_inc.append(float(msg.angle_increment))
        n = len(self.frames)
        if n % 10 == 0:
            self.get_logger().info(f'{n}/{self.n_frames}')
        if n >= self.n_frames:
            self._save()
            rclpy.shutdown()

    def _save(self):
        rmin, rmax = self.meta
        # RPLidar beam count varies per rev -> store ragged as object array.
        # angle_min / angle_increment saved PER FRAME so the notebook can
        # rebuild each frame's angles as: angle_min[i] + arange(len) * angle_inc[i]
        arr = np.empty(len(self.frames), dtype=object)
        for i, f in enumerate(self.frames):
            arr[i] = f
        np.save(self.out_path, {
            'ranges':          arr,                              # ranges[i] = (beams_i,)
            'angle_min':       np.asarray(self.angle_min),       # (N,)
            'angle_increment': np.asarray(self.angle_inc),       # (N,)
            'range_min':       float(rmin),
            'range_max':       float(rmax),
        }, allow_pickle=True)
        self.get_logger().info(f'saved {arr.shape} to {self.out_path}')


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'cliff_scan.npy'
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 40
    rclpy.init()
    node = CliffCapture(out, n)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()