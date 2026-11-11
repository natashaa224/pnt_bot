#!/usr/bin/env python3
"""
paint_executor_node.py

Executes one Z-axis paint stroke per activation.
Activated when FSM enters PAINT_EXECUTE state.

Subscribes:
    /strip_map          (painting_robot_msgs/StripMap)
    /paint_head_status  (painting_robot_msgs/PaintHeadStatus)
    /robot_state        (std_msgs/String)

Publishes:
    /z_axis_cmd         (painting_robot_msgs/ZAxisCmd)
    /wrist_cmd          (painting_robot_msgs/WristCmd)
    /spray_cmd          (painting_robot_msgs/SprayCmd)
    /extension_cmd      (std_msgs/Float32)  -- auxiliary extension position (m)
                                                ONLY sent when strip_z_max > Z_NORMAL_MAX_M
    /stroke_complete    (std_msgs/Bool)
    /paint_controller_state (std_msgs/String) -- PAINTING | OBSTACLE_AVOIDANCE | RESUMING | IDLE

Obstacle Avoidance & Delay Compensation:
    - Accounts for spray shutoff delay: lead distance = z_speed * shutoff_delay_s
    - Disables spraying BEFORE the nozzle enters the obstacle boundary
    - Keeps the gantry moving past the obstacle without pausing the stroke
    - Resumes spraying once the nozzle has completely cleared the obstacle + safety margin
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from painting_robot_msgs.msg import (
    StripMap, ZAxisCmd, WristCmd, SprayCmd,
    PaintHeadStatus, RobotCommand
)

WRIST_NEUTRAL_DEG   =  0.0
WRIST_UP_DEG        = -45.0
WRIST_DOWN_DEG      =  45.0
WRIST_FLICK_DIST_M  =  0.6
POSITION_TOL_M      =  0.006
WRIST_RAMP_LEAD_M   =  0.10
WRIST_SETTLE_TOL_DEG = 3.0
WRIST_SETTLE_TIMEOUT_S = 0.45
Z_TRAVEL_RPM        =  0

STROKE_TIMEOUT_S    =  20.0
Z_NORMAL_MAX_M      =  1.2   # normal gantry (zAxis_joint) upper limit [m]
EXT_MAX_M           =  0.6   # extension_joint upper limit [m]
EXT_TOL_M           =  0.01  # extension settle tolerance [m]
EXT_SETTLE_TIMEOUT_S = 10.0


class PaintExecutorNode(Node):

    def __init__(self):
        super().__init__('paint_executor_node')

        self.declare_parameter('z_travel_rpm',        Z_TRAVEL_RPM)
        self.declare_parameter('wrist_flick_dist_m',  WRIST_FLICK_DIST_M)
        self.declare_parameter('wrist_ramp_lead_m',   WRIST_RAMP_LEAD_M)
        self.declare_parameter('wrist_settle_tol_deg',  WRIST_SETTLE_TOL_DEG)
        self.declare_parameter('wrist_settle_timeout_s', WRIST_SETTLE_TIMEOUT_S)
        self.declare_parameter('position_tol_m',      POSITION_TOL_M)
        self.declare_parameter('stroke_timeout_s',    STROKE_TIMEOUT_S)
        self.declare_parameter('stationary_speed_tol_mps', 0.02)
        self.declare_parameter('stationary_yaw_tol_radps', 0.02)
        self.declare_parameter('stationary_timeout_s',    2.0)
        self.declare_parameter('z_normal_max_m',      Z_NORMAL_MAX_M)
        self.declare_parameter('ext_max_m',           EXT_MAX_M)
        self.declare_parameter('ext_tol_m',           EXT_TOL_M)
        self.declare_parameter('ext_settle_timeout_s', EXT_SETTLE_TIMEOUT_S)

        # Shutoff & delay compensation parameters
        self.declare_parameter('spray_shutoff_delay_s', 0.050) # 50 ms valve/pump delay
        self.declare_parameter('z_nominal_speed_mps',   0.80)  # gantry stroke speed

        self._strip_map     = None
        self._z_state       = None
        self._direction     = 1
        self._active        = False
        self._spray_state   = False
        self._wrist_flicked = False
        self._fsm_state     = ''
        self._last_command_id = -1
        self._z_homed         = False
        self._stroke_start_time = None
        self._wrist_init_done = False
        self._finish_start    = None

        self._base_speed         = 0.0
        self._base_yaw_rate      = 0.0
        self._has_odom           = False
        self._pending_stroke     = False
        self._pending_wait_start = None

        # Extension tracking
        self._ext_pos_m          = 0.0
        self._ext_moving         = False
        self._ext_target_m       = 0.0
        self._ext_settle_start   = None
        self._awaiting_extension = False

        # Obstacle avoidance tracking
        self._in_obstacle_zone   = False
        self._paint_controller_state = 'IDLE'

        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(StripMap,       '/strip_map',         self._strip_map_cb,    latch_qos)
        self.create_subscription(PaintHeadStatus,'/paint_head_status', self._z_state_cb,      10)
        self.create_subscription(String,         '/robot_state',       self._fsm_state_cb,    10)
        self.create_subscription(RobotCommand,   '/robot_command',     self._robot_command_cb, 10)
        self.create_subscription(Odometry,       '/paint_bot/odom',    self._odom_cb,         10)
        self.create_subscription(JointState,     '/joint_states',      self._joint_state_cb,  10)

        self._z_pub        = self.create_publisher(ZAxisCmd,  '/z_axis_cmd',     10)
        self._wrist_pub    = self.create_publisher(WristCmd,  '/wrist_cmd',      10)
        self._spray_pub    = self.create_publisher(SprayCmd,  '/spray_cmd',      10)
        self._ext_pub      = self.create_publisher(Float32,   '/extension_cmd',  10)
        self._complete_pub = self.create_publisher(Bool,      '/stroke_complete', 10)
        self._homing_pub   = self.create_publisher(Bool,      '/homing_complete', 10)
        self._stop_base_pub = self.create_publisher(Twist,    '/paint_bot/cmd_vel', 10)
        self._ctrl_state_pub = self.create_publisher(String,  '/paint_controller_state', 10)

        self.create_timer(0.02, self._monitor_loop)
        self.get_logger().info('paint_executor_node started with obstacle spray-control & delay compensation')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _strip_map_cb(self, msg: StripMap):
        if not self._active:
            self._strip_map = msg

    def _z_state_cb(self, msg: PaintHeadStatus):
        self._z_state = msg

        if not self._wrist_init_done:
            self._set_wrist(WRIST_NEUTRAL_DEG)
            self._wrist_init_done = True

        if msg.limit_lower_z and not self._z_homed:
            self._z_homed = True
            self.get_logger().info('[executor] Homing complete')
            done = Bool()
            done.data = True
            self._homing_pub.publish(done)
        elif not msg.limit_lower_z and self._z_homed:
            self._z_homed = False
            self.get_logger().warn('[executor] homed cleared (actuator un-homed)')

    def _joint_state_cb(self, msg: JointState):
        if 'extension_joint' in msg.name:
            idx = msg.name.index('extension_joint')
            self._ext_pos_m = msg.position[idx]

    def _odom_cb(self, msg: Odometry):
        self._has_odom = True
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        self._base_speed = math.hypot(vx, vy)
        self._base_yaw_rate = abs(wz)

    def _fsm_state_cb(self, msg: String):
        prev = self._fsm_state
        self._fsm_state = msg.data
        if prev == 'PAINT_EXECUTE' and msg.data != 'PAINT_EXECUTE':
            if self._active or self._pending_stroke or self._awaiting_extension:
                self.get_logger().warn('[executor] FSM left PAINT_EXECUTE — aborting')
                self._abort_stroke()

    def _robot_command_cb(self, msg: RobotCommand):
        if msg.state == 'PAINT_EXECUTE' and msg.command_id != self._last_command_id:
            self._last_command_id = msg.command_id
            self.get_logger().info(
                f'[executor] New command id={msg.command_id} → halting base and waiting until stationary')
            self._stop_base_pub.publish(Twist())
            self._pending_stroke = True
            self._pending_wait_start = self.get_clock().now()

        elif msg.state == 'SYSTEM_CHECK' and msg.command_id != self._last_command_id:
            self._last_command_id = msg.command_id
            self._do_homing()

    # ------------------------------------------------------------------
    # Extension helpers
    # ------------------------------------------------------------------
    def _required_extension_m(self) -> float:
        z_normal_max = self.get_parameter('z_normal_max_m').value
        ext_max      = self.get_parameter('ext_max_m').value
        smap = self._strip_map
        if smap is None:
            return 0.0
        overshoot = smap.strip_z_max - z_normal_max
        if overshoot <= 0.0:
            return 0.0
        return min(overshoot, ext_max)

    def _command_extension(self, target_m: float):
        cmd = Float32()
        cmd.data = float(target_m)
        self._ext_pub.publish(cmd)
        self._ext_target_m = target_m
        self._ext_settle_start = self.get_clock().now()
        self._awaiting_extension = True
        self.get_logger().info(
            f'[executor] Extension command → {target_m:.3f}m (current={self._ext_pos_m:.3f}m)')

    def _extension_settled(self) -> bool:
        tol = self.get_parameter('ext_tol_m').value
        return abs(self._ext_pos_m - self._ext_target_m) <= tol

    def _prepare_stroke(self):
        req_ext = self._required_extension_m()
        ext_tol = self.get_parameter('ext_tol_m').value

        if abs(req_ext - self._ext_target_m) <= ext_tol and not self._awaiting_extension:
            if abs(self._ext_pos_m - req_ext) <= ext_tol:
                return True
            self._command_extension(req_ext)
            return False

        if abs(req_ext - self._ext_target_m) > ext_tol:
            self._command_extension(req_ext)
            return False

        return self._extension_settled()

    # ------------------------------------------------------------------
    # Stroke control
    # ------------------------------------------------------------------
    def _start_stroke(self):
        if self._strip_map is None or self._z_state is None or not self._z_state.limit_lower_z:
            self.get_logger().error('[executor] Preconditions not met — cannot start stroke')
            return

        self._active        = True
        self._spray_state   = False
        self._wrist_flicked = False
        self._finish_start  = None
        self._in_obstacle_zone = False
        self._stroke_start_time = self.get_clock().now()

        tol  = self.get_parameter('position_tol_m').value
        z_m  = self._z_state.z_actual_pos_mm / 1000.0
        smap = self._strip_map

        z_normal_max = self.get_parameter('z_normal_max_m').value
        effective_z_max = min(smap.strip_z_max, z_normal_max)

        at_target = (z_m >= effective_z_max - tol) if self._direction > 0 \
                    else (z_m <= smap.strip_z_min + tol)
        if at_target:
            self._direction *= -1
            self.get_logger().warn(
                f'[executor] Z already at stroke target ({z_m:.3f}m) — flipping direction to traverse')

        target_mm = (effective_z_max * 1000.0 if self._direction > 0
                     else smap.strip_z_min * 1000.0)

        self.get_logger().info(
            f'[executor] Stroke {"UP" if self._direction > 0 else "DOWN"} → {target_mm/1000.0:.2f}m '
            f'[ext={self._ext_pos_m:.3f}m, effective_z_max={effective_z_max:.3f}m]')

        self._send_z(ZAxisCmd.Z_POSITION, target_mm,
                     self.get_parameter('z_travel_rpm').value)

        initial_spray = self._spray_for_position(z_m)
        self._set_spray(initial_spray)
        self._publish_ctrl_state('PAINTING' if initial_spray else 'OBSTACLE_AVOIDANCE')

    def _abort_stroke(self):
        if not self._active and not self._pending_stroke and not self._awaiting_extension:
            return
        self._active = False
        self._pending_stroke = False
        self._awaiting_extension = False
        self._stroke_start_time = None
        self._finish_start = None
        self._set_spray(False)
        self._set_wrist(WRIST_NEUTRAL_DEG)
        self._send_z(ZAxisCmd.Z_STOP, 0.0, 0)
        self._publish_ctrl_state('IDLE')

    def _complete_stroke(self):
        self._active = False
        self._stroke_start_time = None
        self._finish_start = None
        self._direction *= -1
        self._set_spray(False)
        self._publish_ctrl_state('IDLE')
        msg = Bool()
        msg.data = True
        self._complete_pub.publish(msg)
        self.get_logger().info('[executor] Stroke complete')

    def _timeout_stroke(self, current_z_mm: float, target_mm: float):
        self.get_logger().error(
            f'[executor] Z stroke timeout: target={target_mm:.1f}mm current={current_z_mm:.1f}mm — aborting stroke')
        self._abort_stroke()
        msg = Bool()
        msg.data = True
        self._complete_pub.publish(msg)

    def _lerp(self, a, b, f):
        f = max(0.0, min(1.0, f))
        return a + (b - a) * f

    def _final_wrist_deg(self, direction):
        return WRIST_UP_DEG if direction > 0 else WRIST_DOWN_DEG

    def _wrist_target_for_position(self, z, z_min, z_max, direction):
        w    = self.get_parameter('wrist_flick_dist_m').value
        lead = self.get_parameter('wrist_ramp_lead_m').value
        w    = min(w, 0.5 * (z_max - z_min))
        lead = min(lead, 0.5 * w)
        span = max(w - lead, 1e-3)

        if direction > 0:
            if z < z_max - w:
                return WRIST_NEUTRAL_DEG
            else:
                return self._lerp(WRIST_NEUTRAL_DEG, WRIST_UP_DEG, (z - (z_max - w)) / span)
        else:
            if z > z_min + w:
                return WRIST_NEUTRAL_DEG
            else:
                return self._lerp(WRIST_NEUTRAL_DEG, WRIST_DOWN_DEG, ((z_min + w) - z) / span)

    # ------------------------------------------------------------------
    # Monitor loop at 50Hz
    # ------------------------------------------------------------------
    def _monitor_loop(self):
        if self._pending_stroke:
            now = self.get_clock().now()
            elapsed = (now - self._pending_wait_start).nanoseconds * 1e-9
            speed_tol = self.get_parameter('stationary_speed_tol_mps').value
            yaw_tol = self.get_parameter('stationary_yaw_tol_radps').value
            timeout = self.get_parameter('stationary_timeout_s').value

            is_stationary = (not self._has_odom or (self._base_speed <= speed_tol and self._base_yaw_rate <= yaw_tol))
            if is_stationary or elapsed >= timeout:
                self._pending_stroke = False
                if not self._prepare_stroke():
                    return
                self._start_stroke()
            return

        if self._awaiting_extension:
            now = self.get_clock().now()
            if self._extension_settled():
                self.get_logger().info(f'[executor] Extension settled at {self._ext_pos_m:.3f}m → starting stroke')
                self._awaiting_extension = False
                self._start_stroke()
            else:
                if self._ext_settle_start is not None:
                    elapsed = (now - self._ext_settle_start).nanoseconds * 1e-9
                    if elapsed >= self.get_parameter('ext_settle_timeout_s').value:
                        self.get_logger().error(f'[executor] Extension settle timeout ({elapsed:.1f}s)')
                        self._awaiting_extension = False
                        self._abort_stroke()
                        msg = Bool()
                        msg.data = True
                        self._complete_pub.publish(msg)
            return

        if not self._active or self._z_state is None or self._strip_map is None:
            return

        z           = self._z_state.z_actual_pos_mm / 1000.0
        smap        = self._strip_map
        tol         = self.get_parameter('position_tol_m').value
        z_normal_max = self.get_parameter('z_normal_max_m').value
        effective_z_max = min(smap.strip_z_max, z_normal_max)

        done = (z >= effective_z_max - tol if self._direction > 0
                else z <= smap.strip_z_min + tol)
        if done:
            final_deg = self._final_wrist_deg(self._direction)
            self._set_wrist(final_deg)

            now = self.get_clock().now()
            if self._finish_start is None:
                self._finish_start = now
            waited_s   = (now - self._finish_start).nanoseconds * 1e-9
            err_deg    = abs(self._z_state.wrist_actual_deg - final_deg)
            settled    = err_deg <= self.get_parameter('wrist_settle_tol_deg').value
            timed_out  = waited_s >= self.get_parameter('wrist_settle_timeout_s').value
            if settled or timed_out:
                self._complete_stroke()
            return

        target_mm = (effective_z_max * 1000.0 if self._direction > 0
                     else smap.strip_z_min * 1000.0)
        if self._stroke_start_time is not None:
            elapsed_s = (self.get_clock().now() - self._stroke_start_time).nanoseconds * 1e-9
            if elapsed_s > self.get_parameter('stroke_timeout_s').value:
                self._timeout_stroke(self._z_state.z_actual_pos_mm, target_mm)
                return

        # -------------------------------------------------------------
        # Delay-compensated obstacle detection & automatic spray control
        # -------------------------------------------------------------
        desired = self._spray_for_position(z)

        if desired != self._spray_state:
            self._set_spray(desired)
            if not desired:
                self._in_obstacle_zone = True
                self._publish_ctrl_state('OBSTACLE_AVOIDANCE')
                self.get_logger().info(
                    f'[executor] OBSTACLE DETECTED at z={z:.3f}m → SPRAY DISABLED (Passing obstacle)')
            else:
                self._in_obstacle_zone = False
                self._publish_ctrl_state('RESUMING')
                self.get_logger().info(
                    f'[executor] OBSTACLE CLEARED at z={z:.3f}m → SPRAY RESUMED')

        wrist_deg = self._wrist_target_for_position(z, smap.strip_z_min, effective_z_max, self._direction)
        self._set_wrist(wrist_deg)

    # ------------------------------------------------------------------
    # Spray & Lookahead calculation
    # ------------------------------------------------------------------
    def _spray_for_position(self, z: float) -> bool:
        smap = self._strip_map
        if smap is None or not smap.z_start:
            return True

        z_clamped = max(smap.strip_z_min, min(smap.strip_z_max, z))

        # Check raw paintability at current nozzle position
        current_paintable = False
        for i in range(len(smap.z_start)):
            if smap.z_start[i] <= z_clamped <= smap.z_end[i]:
                current_paintable = smap.paint_on[i]
                break

        # Delay compensation: anticipate shutoff ahead of travel direction
        shutoff_delay_s = self.get_parameter('spray_shutoff_delay_s').value
        z_speed = self.get_parameter('z_nominal_speed_mps').value
        lead_dist = z_speed * shutoff_delay_s  # e.g. 0.8 * 0.05 = 0.040 m

        lookahead_z = z_clamped + (lead_dist if self._direction > 0 else -lead_dist)
        lookahead_z = max(smap.strip_z_min, min(smap.strip_z_max, lookahead_z))

        lookahead_paintable = False
        for i in range(len(smap.z_start)):
            if smap.z_start[i] <= lookahead_z <= smap.z_end[i]:
                lookahead_paintable = smap.paint_on[i]
                break

        # Spray is permitted ONLY if both current position AND lookahead position are paintable.
        # This guarantees shutoff occurs BEFORE crossing into the obstacle/margin zone.
        return bool(current_paintable and lookahead_paintable)

    # ------------------------------------------------------------------
    # Actuator helpers
    # ------------------------------------------------------------------
    def _do_homing(self):
        self._set_wrist(WRIST_NEUTRAL_DEG)
        if self._z_homed:
            self.get_logger().info('[executor] Already homed — skipping re-home')
            done = Bool()
            done.data = True
            self._homing_pub.publish(done)
            return

        self.get_logger().warn('[executor] Not homed — arming teach-homing')
        cmd = ZAxisCmd()
        cmd.z_mode = ZAxisCmd.Z_HOME
        self._z_pub.publish(cmd)

    def _send_z(self, z_mode: int, target_mm: float = 0.0, rpm: int = 0):
        cmd = ZAxisCmd()
        cmd.z_mode         = z_mode
        cmd.z_target_pos_mm = float(target_mm)
        cmd.rpm = int(rpm)
        self._z_pub.publish(cmd)

    def _set_wrist(self, angle_deg: float):
        cmd = WristCmd()
        cmd.wrist_target_deg = float(angle_deg)
        self._wrist_pub.publish(cmd)

    def _set_spray(self, on: bool):
        if on == self._spray_state:
            return
        self._spray_state = on
        msg = SprayCmd()
        msg.valve_open    = on
        msg.pump_enable   = on
        msg.flow_setpoint = 1.0 if on else 0.0
        self._spray_pub.publish(msg)
        self.get_logger().debug(f'[executor] spray → {"ON" if on else "OFF"}')

    def _publish_ctrl_state(self, state: str):
        self._paint_controller_state = state
        msg = String()
        msg.data = state
        self._ctrl_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PaintExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
