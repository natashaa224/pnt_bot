# VRai
# Cliff detection — downward-angled RPLidar C1 on /scan_cliff.
# Forward arc floor monitor: range rise over flat baseline = drop/step ahead.
# Validated on captured scans: flat 0/40 false, step 39/40 (1 corrupt frame -> fault/hold).

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from painting_robot_msgs.msg import CliffStatus
import numpy as np
from collections import deque

# ── LiDAR mounting ────────────────────────────────────────────────────────────
# Same C1 convention as wall detection: arrow→code 0° (front), scan in [-π, π).
LIDAR_YAW_OFFSET = -np.pi

# ── Forward floor arc ─────────────────────────────────────────────────────────
ARC_CENTER_DEG   = 0.0     # forward
ARC_SPREAD_DEG   = 15.0    # ±15° fan onto floor

# ── Detection tuning (validated against real flat/step captures) ──────────────
DROP_THRESHOLD   = 0.05    # m — range rise over baseline that counts as a drop
CLIFF_FRAC       = 0.30    # fraction of arc beams "dropped" to call a cliff
MIN_VALID_BEAMS  = 5       # below this → sensor fault, hold last state
MIN_VALID_RANGE  = 0.05    # m — sensor noise floor

# ── Temporal persistence (glitch rejection) ──────────────────────────────────
PERSIST_WINDOW   = 3       # look at last N frames
PERSIST_HITS     = 2       # need this many cliff frames in window to publish True


def arc_mask(angles_rad, center_deg, spread_deg):
    """Boolean mask of beams within ±spread of center. Handles wrap."""
    c = np.radians(center_deg)
    s = np.radians(spread_deg)
    diff = (angles_rad - c + np.pi) % (2 * np.pi) - np.pi
    return np.abs(diff) <= s


class CliffDetectionNode(Node):

    def __init__(self):
        super().__init__('cliff_detection_node')

        # Only the baseline is a param (shared env_monitoring_params.yaml).
        # Everything else is a module constant, matching wall_detection style.
        self.declare_parameter('cliff_baseline_range', 0.188)
        self.declare_parameter('publish_rate', 10.0)
        self.baseline = self.get_parameter('cliff_baseline_range').value

        # Scan state
        self.scan = None

        # Live (non-latching) cliff state + persistence window
        self._history      = deque(maxlen=PERSIST_WINDOW)
        self._last_cliff   = False          # held across fault frames

        # Subscriptions
        self.create_subscription(LaserScan, '/scan_cliff', self._scan_cb, 10)

        # Publisher — combined bool + floor range
        self._status_pub = self.create_publisher(CliffStatus, '/cliff_status', 10)

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._detection_loop)

        self.get_logger().info(
            f'CliffDetectionNode started — baseline={self.baseline:.3f}m')

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _scan_cb(self, msg):
        self.scan = {
            'ranges'          : np.asarray(msg.ranges, dtype=np.float32),
            'angle_min'       : msg.angle_min,
            'angle_increment' : msg.angle_increment,
        }

    # ── Detection ──────────────────────────────────────────────────────────────

    def _analyse(self, scan):
        """
        Returns (cliff_this_frame: bool|None, floor_range: float).
        cliff = None  → sensor fault (too few valid beams); caller holds last.
        floor_range   → median of valid arc beams (NaN if none) for calibration.
        """
        r = scan['ranges']
        angles = scan['angle_min'] + np.arange(len(r)) * scan['angle_increment']
        angles = (angles + LIDAR_YAW_OFFSET) % (2 * np.pi)

        m  = arc_mask(angles, ARC_CENTER_DEG, ARC_SPREAD_DEG)
        rr = r[m]
        if rr.size == 0:
            return None, float('nan')

        valid = np.isfinite(rr) & (rr > MIN_VALID_RANGE)
        if valid.sum() < MIN_VALID_BEAMS:
            return None, float('nan')

        floor_range = float(np.median(rr[valid]))

        # dropped beams: range risen past baseline, OR abyss (inf/oor) beams
        dropped = (rr[valid] - self.baseline) > DROP_THRESHOLD
        abyss   = (~valid).sum()
        frac    = (dropped.sum() + abyss) / rr.size

        return bool(frac >= CLIFF_FRAC), floor_range

    def _detection_loop(self):
        if self.scan is None:
            return

        cliff_frame, floor_range = self._analyse(self.scan)

        if cliff_frame is None:
            # Sensor fault / corrupt frame — hold last cliff state, don't clear.
            self.get_logger().warn(
                'cliff scan fault — too few valid beams, holding last state',
                throttle_duration_sec=2.0)
            self._publish(self._last_cliff, floor_range)
            return

        # Persistence: cliff only if PERSIST_HITS in last PERSIST_WINDOW frames.
        self._history.append(cliff_frame)
        cliff = sum(self._history) >= PERSIST_HITS
        self._last_cliff = cliff

        self.get_logger().info(
            f'floor={floor_range:.3f}m  cliff={cliff}',
            throttle_duration_sec=40.0)

        self._publish(cliff, floor_range)

    # ── Publishing ─────────────────────────────────────────────────────────────

    def _publish(self, cliff, floor_range):
        msg = CliffStatus()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'cliff_laser'
        msg.cliff_detected  = bool(cliff)
        msg.floor_range     = float(floor_range) if np.isfinite(floor_range) else 0.0
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CliffDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
