#!/usr/bin/env python3
"""
paint_planner_node.py

Publishes a StripMap for the current strip.
For now: default full-ON map, full wall height, both ends flickable.
When depth camera skip-zone detection is ready, this node will compute
zone boundaries from fixture detections and update the map per strip.

Z bounds (strip_z_min/strip_z_max) come from actuator_bridge_node's taught
homing limits (/paint_head_status z_soft_lower_mm/z_soft_upper_mm) once
homed — there are no physical Z limit switches on this machine, so the
travel range is whatever the operator taught via joystick LB/A/Y, not a
fixed launch-file guess. The strip_z_min/strip_z_max params are only a
fallback used before homing completes.

Publishes: /strip_map (painting_robot_msgs/StripMap) — latched, 1Hz refresh
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from painting_robot_msgs.msg import StripMap, WallDepthProfile, PaintHeadStatus

TAUGHT_BOUNDS_EPS_M = 0.001   # ignore sub-mm noise when deciding to rebuild


class PaintPlannerNode(Node):

    def __init__(self):
        super().__init__('paint_planner_node')

        # Parameters — fallback bounds, used only until homing is taught
        self.declare_parameter('strip_z_min',  0.0)
        self.declare_parameter('strip_z_max',  2.44)
        self.declare_parameter('flick_at_min', True)
        self.declare_parameter('flick_at_max', True)

        # Latched publisher — new subscribers always get the latest map
        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._pub = self.create_publisher(StripMap, '/strip_map', latch_qos)

        self.create_subscription(WallDepthProfile, '/wall_depth_profile', self._depth_profile_cb, 10)
        self.create_subscription(PaintHeadStatus, '/paint_head_status', self._head_status_cb, 10)

        self._latest_profile: WallDepthProfile | None = None
        self._taught_z_min = None   # metres — set once actuator_bridge_node reports homed
        self._taught_z_max = None

        # Publish once on startup, then refresh at 1Hz in case params change
        self._publish()
        self.create_timer(1.0, self._publish)
        self.get_logger().info('paint_planner_node started — publishing default full-ON StripMap')


    def _depth_profile_cb(self, msg: WallDepthProfile):
        self._latest_profile = msg
        self._publish()          # rebuild and publish immediately on new profile

    def _head_status_cb(self, msg: PaintHeadStatus):

        if not msg.limit_lower_z:
            return
        z_min = msg.z_soft_lower_mm / 1000.0
        z_max = msg.z_soft_upper_mm / 1000.0
        if z_max <= z_min:
            return   # not taught yet / invalid — keep current bounds
        changed = (self._taught_z_min is None
                   or abs(z_min - self._taught_z_min) > TAUGHT_BOUNDS_EPS_M
                   or abs(z_max - self._taught_z_max) > TAUGHT_BOUNDS_EPS_M)
        if not changed:
            return
        self._taught_z_min = z_min
        self._taught_z_max = z_max
        self.get_logger().info(
            f'[planner] strip Z bounds ← taught homing limits '
            f'[{z_min:.3f}, {z_max:.3f}]m')
        self._publish()

    def _publish(self):
        if self._taught_z_min is not None:
            z_min, z_max = self._taught_z_min, self._taught_z_max
        else:
            z_min = self.get_parameter('strip_z_min').value
            z_max = self.get_parameter('strip_z_max').value

        # Adapt stroke upper bound to wherever wall is detected by vertical LiDAR
        if self._latest_profile is not None and self._latest_profile.z_positions:
            wall_zs = [z for z, p in zip(self._latest_profile.z_positions, self._latest_profile.paintable) if p]
            if wall_zs:
                detected_top = max(wall_zs)
                z_max = max(float(z_min) + 0.20, min(float(z_max), float(detected_top)))

        msg = StripMap()
        msg.strip_z_min  = float(z_min)
        msg.strip_z_max  = float(z_max)
        msg.flick_at_min = self.get_parameter('flick_at_min').value
        msg.flick_at_max = self.get_parameter('flick_at_max').value

        if self._latest_profile is None:
            # No depth data yet — default full-ON
            msg.z_start  = [float(z_min)]
            msg.z_end    = [float(z_max)]
            msg.paint_on = [True]
        else:
            msg.z_start, msg.z_end, msg.paint_on = \
                self._build_zones(self._latest_profile, z_min, z_max)

        self._pub.publish(msg)

    def _build_zones(self, profile: WallDepthProfile,
                    z_min: float, z_max: float):
        """Run-length encode per-sample paintability into contiguous zones."""
        # Pair and sort by Z within [z_min, z_max]
        samples = [s for s in sorted(zip(profile.z_positions, profile.paintable))
                   if float(z_min) <= s[0] <= float(z_max)]
        if not samples:
            return [float(z_min)], [float(z_max)], [True]

        z_starts, z_ends, paint_flags = [], [], []
        cur_paint = samples[0][1]
        cur_start = float(z_min)

        for z, paint in samples:
            if paint != cur_paint:
                z_ends.append(z)
                z_starts.append(cur_start)
                paint_flags.append(cur_paint)
                cur_start = z
                cur_paint = paint

        # Close final zone
        z_starts.append(cur_start)
        z_ends.append(float(z_max))
        paint_flags.append(cur_paint)

        # Derive flick permission — no flick if zone at boundary is paint-off
        # (caller can override via params if needed)
        return z_starts, z_ends, paint_flags    


def main(args=None):
    rclpy.init(args=args)
    node = PaintPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
