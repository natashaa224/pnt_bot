#!/usr/bin/env python3
"""
paint_visualizer_node.py

Visualizes:
1. Active Paint Spray Plume (/visual_spray_plume)
   - Cone emanating from paint_spray_gun_link toward the wall when spray is ON
   - Immediately vanishes when spray is OFF
2. Painted Wall Surface (/painted_wall_layer)
   - Persistent green painted layer deposited onto the wall surface while spray is ON
   - Unpainted bare wall remains visible over obstacles and margin zones
3. Real-Time Visual Status Panel HUD (/paint_status_panel)
   - Live formatted status table (LiDAR status, Obstacle detection, Type, Position,
     Spray ON/OFF, Controller state, Safety margin, Strip index)
4. Color Status Indicator Beacon (/paint_status_indicator)
   - GREEN: Spray ON / Safe to paint
   - RED: Spray OFF / Obstacle detected
   - YELLOW: Obstacle approaching / Waiting for clearance
   - GREY: Unpainted / Idle
5. Effective Spray Footprint (/effective_spray_footprint)
   - 3D box at wall face colored green/yellow/red indicating nozzle coverage
6. Validation Metrics Aggregator (/paint_validation_metrics)
   - Live JSON and console summaries of detection accuracy, paint-stop/resume counts,
     and obstacle clearance distances.
"""

import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header, ColorRGBA, String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from painting_robot_msgs.msg import (
    SprayCmd, PaintHeadStatus, StripMap
)


class PaintVisualizerNode(Node):

    def __init__(self):
        super().__init__('paint_visualizer_node')

        self.declare_parameter('wall_y_plane', -0.750)
        self.declare_parameter('paint_patch_width_m', 0.08)
        self.declare_parameter('paint_patch_height_m', 0.02)
        self.declare_parameter('max_paint_markers', 2000)

        self._wall_y = float(self.get_parameter('wall_y_plane').value)
        self._patch_w = float(self.get_parameter('paint_patch_width_m').value)
        self._patch_h = float(self.get_parameter('paint_patch_height_m').value)
        self._max_markers = int(self.get_parameter('max_paint_markers').value)

        # State tracking
        self._spray_on = False
        self._spray_valve_open = False
        self._last_spray_change_t = time.time()
        self._last_paint_stamp_t = 0.0
        self._current_z_mm = 0.0
        self._wrist_deg = 0.0
        self._controller_state = 'IDLE'
        self._strip_index = 1
        self._strip_map = None
        self._lidar_scanning = False
        self._last_scan_t = time.time()

        # Obstacle summary from wall_depth_profile_node
        self._obstacle_detected = False
        self._obstacle_type = 'CLEAR'
        self._obs_z_min = 0.0
        self._obs_z_max = 0.0
        self._safety_margin = 0.05
        self._d_nominal = 0.41

        # Validation metrics
        self._metrics = {
            'obstacles_placed': 6,
            'obstacles_detected': 0,
            'missed_obstacles': 0,
            'false_detections': 0,
            'successful_stops': 0,
            'successful_resumes': 0,
            'paint_on_obstacle': False,
            'last_shutoff_dist_to_obs': 0.0,
            'last_resume_dist_to_obs': 0.0,
            'stop_events': [],
            'resume_events': []
        }

        # Painted patches storage
        self._painted_patches = []
        self._marker_id_counter = 0

        # TF Buffer & Listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Subscriptions
        self.create_subscription(SprayCmd, '/spray_cmd', self._spray_cb, 10)
        self.create_subscription(PaintHeadStatus, '/paint_head_status', self._head_status_cb, 10)
        self.create_subscription(String, '/paint_controller_state', self._ctrl_state_cb, 10)
        self.create_subscription(String, '/wall_obstacle_summary', self._obs_summary_cb, 10)
        self.create_subscription(StripMap, '/strip_map', self._strip_map_cb, 10)
        self.create_subscription(LaserScan, '/wall_scan', self._scan_cb, rclcpp.SensorDataQoS())

        # Publishers
        self._plume_pub = self.create_publisher(Marker, '/visual_spray_plume', 10)
        self._paint_layer_pub = self.create_publisher(MarkerArray, '/painted_wall_layer', 10)
        self._footprint_pub = self.create_publisher(Marker, '/effective_spray_footprint', 10)
        self._hud_pub = self.create_publisher(Marker, '/paint_status_panel', 10)
        self._indicator_pub = self.create_publisher(Marker, '/paint_status_indicator', 10)
        self._metrics_pub = self.create_publisher(String, '/paint_validation_metrics', 10)

        # 50 Hz paint stamping and plume update
        self.create_timer(0.02, self._paint_loop)
        # 10 Hz HUD and metrics refresh
        self.create_timer(0.10, self._hud_loop)

        self.get_logger().info('PaintVisualizerNode initialized: live spray plume, wall paint layer, and HUD panel active.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _spray_cb(self, msg: SprayCmd):
        prev = self._spray_valve_open
        self._spray_valve_open = bool(msg.valve_open)
        now = time.time()

        if not prev and self._spray_valve_open:
            # Transition: OFF -> ON (Resume)
            self._metrics['successful_resumes'] += 1
            if self._obstacle_detected and self._obs_z_max > self._obs_z_min:
                z_curr = self._current_z_mm / 1000.0
                dist_to_obs = abs(z_curr - self._obs_z_max) if z_curr > self._obs_z_max else abs(z_curr - self._obs_z_min)
                self._metrics['last_resume_dist_to_obs'] = dist_to_obs
                self._metrics['resume_events'].append({'z': z_curr, 'dist_to_obs': dist_to_obs, 'time': now})
                self.get_logger().info(f'[visualizer] SPRAY RESUMED at z={z_curr:.3f}m (dist_to_obs={dist_to_obs:.3f}m)')

        elif prev and not self._spray_valve_open:
            # Transition: ON -> OFF (Stop)
            self._metrics['successful_stops'] += 1
            if self._obstacle_detected and self._obs_z_max > self._obs_z_min:
                z_curr = self._current_z_mm / 1000.0
                dist_to_obs = abs(z_curr - self._obs_z_min) if z_curr < self._obs_z_min else abs(z_curr - self._obs_z_max)
                self._metrics['last_shutoff_dist_to_obs'] = dist_to_obs
                self._metrics['stop_events'].append({'z': z_curr, 'dist_to_obs': dist_to_obs, 'time': now})
                self.get_logger().info(f'[visualizer] SPRAY STOPPED at z={z_curr:.3f}m (dist_to_obs={dist_to_obs:.3f}m)')

        self._last_spray_change_t = now

    def _head_status_cb(self, msg: PaintHeadStatus):
        self._current_z_mm = msg.z_actual_pos_mm
        self._wrist_deg = msg.wrist_actual_deg

    def _ctrl_state_cb(self, msg: String):
        self._controller_state = msg.data

    def _strip_map_cb(self, msg: StripMap):
        self._strip_map = msg

    def _scan_cb(self, msg: LaserScan):
        self._lidar_scanning = True
        self._last_scan_t = time.time()

    def _obs_summary_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            prev_det = self._obstacle_detected
            self._obstacle_detected = bool(data.get('detected', False))
            self._obstacle_type = str(data.get('classification', 'CLEAR'))
            self._obs_z_min = float(data.get('z_min', 0.0))
            self._obs_z_max = float(data.get('z_max', 0.0))
            self._safety_margin = float(data.get('margin', 0.05))
            self._d_nominal = float(data.get('d_nominal', 0.41))

            if not prev_det and self._obstacle_detected:
                self._metrics['obstacles_detected'] += 1
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Loop 1: Spray plume & Wall paint stamping (50 Hz)
    # ------------------------------------------------------------------
    def _paint_loop(self):
        now = self.get_clock().now()

        # Update LiDAR status
        if (time.time() - self._last_scan_t) > 0.5:
            self._lidar_scanning = False

        # 1. Spray Plume Visualization (in paint_spray_gun_link frame)
        plume_msg = Marker()
        plume_msg.header.stamp = now.to_msg()
        plume_msg.header.frame_id = 'paint_spray_gun_link'
        plume_msg.ns = 'spray_plume'
        plume_msg.id = 1
        plume_msg.type = Marker.LINE_LIST

        if self._spray_valve_open:
            plume_msg.action = Marker.ADD
            plume_msg.scale.x = 0.005  # line width

            # Realistic conical spray fan rays (16 streams)
            # Nozzle sprays forward along link's X axis towards wall
            cone_length = 0.22
            spread_angle = math.radians(22.0)
            num_rays = 16

            for i in range(num_rays):
                theta = 2.0 * math.pi * (i / float(num_rays))
                ray_end = Point()
                # Fan out in Y and Z, extending in X
                ray_end.x = cone_length * math.cos(spread_angle * 0.5)
                ray_end.y = cone_length * math.sin(spread_angle * 0.5) * math.cos(theta)
                ray_end.z = cone_length * math.sin(spread_angle * 0.5) * math.sin(theta)

                # Stream from origin (0,0,0) to ray_end
                plume_msg.points.append(Point(x=0.0, y=0.0, z=0.0))
                plume_msg.points.append(ray_end)

                # Color: Bright Cyan/White mist
                c_start = ColorRGBA(r=0.4, g=0.9, b=1.0, a=0.85)
                c_end = ColorRGBA(r=0.2, g=0.7, b=0.9, a=0.15)
                plume_msg.colors.append(c_start)
                plume_msg.colors.append(c_end)
        else:
            plume_msg.action = Marker.DELETE

        self._plume_pub.publish(plume_msg)

        # 2. Wall Paint Layer Stamping
        # Query spray gun pose in world/odom frame
        try:
            t = self._tf_buffer.lookup_transform(
                'odom', 'paint_spray_gun_link', rclpy.time.Time())
            gun_x = t.transform.translation.x
            gun_y = t.transform.translation.y
            gun_z = t.transform.translation.z
        except Exception:
            # Fallback to base_footprint if odom not available
            try:
                t = self._tf_buffer.lookup_transform(
                    'base_footprint', 'paint_spray_gun_link', rclpy.time.Time())
                gun_x = t.transform.translation.x
                gun_y = t.transform.translation.y
                gun_z = t.transform.translation.z
            except Exception:
                return

        # Effective footprint on the wall
        footprint_marker = Marker()
        footprint_marker.header.stamp = now.to_msg()
        footprint_marker.header.frame_id = t.header.frame_id
        footprint_marker.ns = 'effective_spray_footprint'
        footprint_marker.id = 2
        footprint_marker.type = Marker.CUBE
        footprint_marker.action = Marker.ADD

        # Position at the wall surface plane
        footprint_marker.pose.position.x = gun_x
        footprint_marker.pose.position.y = self._wall_y + 0.01  # just on wall face
        footprint_marker.pose.position.z = gun_z
        footprint_marker.scale.x = self._patch_w
        footprint_marker.scale.y = 0.010
        footprint_marker.scale.z = self._patch_h

        # Color code: GREEN=Spraying & Safe, RED=Over Obstacle, YELLOW=Margin clearance, GREY=Spray OFF
        if self._spray_valve_open:
            footprint_marker.color = ColorRGBA(r=0.15, g=0.90, b=0.25, a=0.85)
        elif self._obstacle_detected:
            footprint_marker.color = ColorRGBA(r=0.95, g=0.15, b=0.15, a=0.85)
        elif self._controller_state == 'RESUMING':
            footprint_marker.color = ColorRGBA(r=0.95, g=0.85, b=0.10, a=0.85)
        else:
            footprint_marker.color = ColorRGBA(r=0.60, g=0.60, b=0.65, a=0.50)

        self._footprint_pub.publish(footprint_marker)

        # Deposit paint onto wall only when spray is actively ON
        if self._spray_valve_open:
            # Verify if paint is accidentally hitting an obstacle (validation check)
            z_stroke = gun_z - 0.410  # convert to gantry stroke coordinates
            if self._obstacle_detected and (self._obs_z_min <= z_stroke <= self._obs_z_max):
                self._metrics['paint_on_obstacle'] = True
                self.get_logger().error(f'[ALERT] Paint applied over obstacle at z={z_stroke:.3f}m!')

            # Stamp paint patch
            patch_marker = Marker()
            patch_marker.header.stamp = now.to_msg()
            patch_marker.header.frame_id = t.header.frame_id
            patch_marker.ns = 'painted_wall'
            patch_marker.id = self._marker_id_counter
            self._marker_id_counter = (self._marker_id_counter + 1) % self._max_markers
            patch_marker.type = Marker.CUBE
            patch_marker.action = Marker.ADD

            patch_marker.pose.position.x = gun_x
            patch_marker.pose.position.y = self._wall_y + 0.005  # thin paint coat on wall
            patch_marker.pose.position.z = gun_z

            patch_marker.scale.x = self._patch_w
            patch_marker.scale.y = 0.004  # 4mm paint coat thickness
            patch_marker.scale.z = self._patch_h

            # Distinct bright paint color (Emerald Green)
            patch_marker.color = ColorRGBA(r=0.10, g=0.85, b=0.25, a=0.95)

            self._painted_patches.append(patch_marker)
            if len(self._painted_patches) > self._max_markers:
                self._painted_patches.pop(0)

            # Publish updated painted layer (every 5 stamps to avoid bus saturation)
            if self._marker_id_counter % 3 == 0:
                array_msg = MarkerArray()
                array_msg.markers = self._painted_patches[-150:]  # send recent window
                self._paint_layer_pub.publish(array_msg)

    # ------------------------------------------------------------------
    # Loop 2: Real-time HUD Status Panel & Validation Metrics (10 Hz)
    # ------------------------------------------------------------------
    def _hud_loop(self):
        now = self.get_clock().now()

        # 1. 3D Floating Status Panel HUD
        hud_marker = Marker()
        hud_marker.header.stamp = now.to_msg()
        hud_marker.header.frame_id = 'base_link'
        hud_marker.ns = 'paint_status_panel'
        hud_marker.id = 100
        hud_marker.type = Marker.TEXT_VIEW_FACING
        hud_marker.action = Marker.ADD

        # Position HUD floating comfortably above the robot and wall
        hud_marker.pose.position.x = 0.20
        hud_marker.pose.position.y = -0.30
        hud_marker.pose.position.z = 1.65
        hud_marker.scale.z = 0.065  # text size (6.5 cm)

        # Indicator styling
        lidar_str = "SCANNING" if self._lidar_scanning else "ERROR / IDLE"
        obs_status = "DETECTED" if self._obstacle_detected else "CLEAR"
        spray_str = "ON" if self._spray_valve_open else "OFF"
        z_curr_m = self._current_z_mm / 1000.0

        pos_str = f"Z: [{self._obs_z_min:.2f}m, {self._obs_z_max:.2f}m]" if self._obstacle_detected else "NONE"

        panel_text = (
            "╔══════════════════════════════════════════════════╗\n"
            "║         PAINT BOT - REAL-TIME STATUS PANEL       ║\n"
            "╠══════════════════════════════════════════════════╣\n"
            f"║  LiDAR Status       : {lidar_str:<26} ║\n"
            f"║  Obstacle Detection : {obs_status:<26} ║\n"
            f"║  Obstacle Type      : {self._obstacle_type:<26} ║\n"
            f"║  Obstacle Position  : {pos_str:<26} ║\n"
            f"║  Paint Spray        : {spray_str:<26} ║\n"
            f"║  Paint Controller   : {self._controller_state:<26} ║\n"
            f"║  Safety Margin      : {self._safety_margin:.3f} m                   ║\n"
            f"║  Current Nozzle Z   : {z_curr_m:.3f} m                   ║\n"
            "╚══════════════════════════════════════════════════╝"
        )
        hud_marker.text = panel_text

        # Text color
        if self._spray_valve_open:
            hud_marker.color = ColorRGBA(r=0.2, g=1.0, b=0.4, a=1.0)
        elif self._obstacle_detected:
            hud_marker.color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=1.0)
        else:
            hud_marker.color = ColorRGBA(r=1.0, g=0.9, b=0.2, a=1.0)

        self._hud_pub.publish(hud_marker)

        # 2. Prominent Spherical Beacon Indicator
        ind_marker = Marker()
        ind_marker.header.stamp = now.to_msg()
        ind_marker.header.frame_id = 'paint_spray_gun_link'
        ind_marker.ns = 'status_beacon'
        ind_marker.id = 101
        ind_marker.type = Marker.SPHERE
        ind_marker.action = Marker.ADD
        ind_marker.pose.position.x = 0.05
        ind_marker.pose.position.y = 0.0
        ind_marker.pose.position.z = 0.08
        ind_marker.scale.x = 0.06
        ind_marker.scale.y = 0.06
        ind_marker.scale.z = 0.06

        if self._spray_valve_open:
            ind_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.95)  # GREEN: Spray ON / Safe
        elif self._obstacle_detected:
            ind_marker.color = ColorRGBA(r=1.0, g=0.05, b=0.05, a=0.95) # RED: Obstacle detected / OFF
        elif self._controller_state == 'RESUMING':
            ind_marker.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.95)  # YELLOW: Resuming / Clearance
        else:
            ind_marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.6)   # GREY: Idle / Unpainted

        self._indicator_pub.publish(ind_marker)

        # 3. Validation Metrics Publisher
        metrics_msg = String()
        metrics_msg.data = json.dumps(self._metrics)
        self._metrics_pub.publish(metrics_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PaintVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
