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
    /stroke_complete    (std_msgs/Bool)   -- also published (True) on stroke
                                              timeout, so a stuck stroke can't
                                              hang PAINT_EXECUTE forever; the
                                              FSM's existing stroke_complete
                                              handling returns it to WALL_FOLLOW

Extension control logic:
    - Z_NORMAL_MAX_M: maximum reach of the normal gantry (zAxis_joint) = 1.2 m
    - EXT_MAX_M: maximum reach of the auxiliary extension (extension_joint) = 0.6 m
    - If strip_z_max <= Z_NORMAL_MAX_M:
        * Keep extension at 0.0 m (stationary)
        * Run stroke with normal zAxis_joint only
    - If strip_z_max > Z_NORMAL_MAX_M:
        * Required extension = strip_z_max - Z_NORMAL_MAX_M (clamped to EXT_MAX_M)
        * Stop spray, command extension to required position
        * Wait until extension is stationary (position settled within EXT_TOL_M)
        * Then run normal stroke with zAxis_joint up to Z_NORMAL_MAX_M
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from painting_robot_msgs.msg import (
    StripMap, ZAxisCmd, WristCmd, SprayCmd,
    PaintHeadStatus, RobotCommand
)

WRIST_NEUTRAL_DEG   =  0.0
WRIST_UP_DEG        = -45.0
WRIST_DOWN_DEG      = 45.0
WRIST_FLICK_DIST_M  =  0.6
POSITION_TOL_M      =  0.006
# Wrist ramp finishes this far BEFORE the stroke end so the (laggy) servo has
# travel distance to physically arrive before Z stops and the nozzle closes.
WRIST_RAMP_LEAD_M   =  0.10
# End-of-stroke: keep spray open until the measured wrist angle is within this
# many degrees of the final flick angle...
WRIST_SETTLE_TOL_DEG = 3.0
# ...but never longer than this (failsafe: wrist stalled / no feedback / no driver).
WRIST_SETTLE_TIMEOUT_S = 0.45
Z_TRAVEL_RPM        =  0  # let Actuator params decide speed

# Fixed failsafe timeout for a commanded Z stroke. Not distance/speed derived:
# actual mm/s depends on hardware calibration constants (counts_per_mm,
# motor counts_per_rev) that live in actuator_bridge_node/leadshine_driver,
# not here — duplicating them into this node would risk silently drifting
# out of sync with the real hardware config. Override via the
# 'stroke_timeout_s' param if 20s proves too tight/loose on real hardware.
STROKE_TIMEOUT_S    =  20.0

# Auxiliary extension parameters
# Must match URDF limits in gantry.xacro (extension_joint upper="0.6")
Z_NORMAL_MAX_M      =  1.2   # normal gantry (zAxis_joint) upper limit [m]
EXT_MAX_M           =  0.6   # extension_joint upper limit [m]
EXT_TOL_M           =  0.01  # extension settle tolerance [m]
EXT_SETTLE_TIMEOUT_S = 10.0  # failsafe: give up waiting for extension after this


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

        self._strip_map     = None
        self._z_state       = None
        self._direction     = 1
        self._active        = False
        self._spray_state   = False
        self._wrist_flicked = False
        self._fsm_state     = ''
        self._last_command_id = -1       # gate: only act on new command_id
        self._z_homed         = False
        self._stroke_start_time = None   # set on stroke start; gates timeout check
        self._wrist_init_done = False
        self._finish_start    = None     # set when Z reaches stroke end; wrist-settle phase

        self._base_speed         = 0.0
        self._base_yaw_rate      = 0.0
        self._has_odom           = False
        self._pending_stroke     = False
        self._pending_wait_start = None

        # --- Auxiliary extension state ---
        # _ext_pos_m: last known extension position (from /joint_states or estimated)
        # _ext_moving: True while waiting for the extension to reach its target
        # _ext_target_m: commanded extension position for the current strip
        # _ext_settle_start: time when we sent the extension command (failsafe gate)
        from sensor_msgs.msg import JointState
        self._ext_pos_m          = 0.0
        self._ext_moving         = False
        self._ext_target_m       = 0.0
        self._ext_settle_start   = None
        self._awaiting_extension  = False   # True: blocked in extension-settle phase

        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(StripMap,       '/strip_map',         self._strip_map_cb,    latch_qos)
        self.create_subscription(PaintHeadStatus,'/paint_head_status', self._z_state_cb,      10)
        self.create_subscription(String,         '/robot_state',       self._fsm_state_cb,    10)
        self.create_subscription(RobotCommand,   '/robot_command',     self._robot_command_cb, 10)
        self.create_subscription(Odometry,       '/paint_bot/odom',    self._odom_cb,         10)

        # Monitor extension_joint position from /joint_states
        from sensor_msgs.msg import JointState
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        self._z_pub        = self.create_publisher(ZAxisCmd,  '/z_axis_cmd',     10)
        self._wrist_pub    = self.create_publisher(WristCmd,  '/wrist_cmd',      10)
        self._spray_pub    = self.create_publisher(SprayCmd,  '/spray_cmd',      10)
        self._ext_pub      = self.create_publisher(Float32,   '/extension_cmd',  10)
        self._complete_pub = self.create_publisher(Bool,      '/stroke_complete', 10)
        self._homing_pub   = self.create_publisher(Bool,      '/homing_complete', 10)
        self._stop_base_pub = self.create_publisher(Twist,    '/paint_bot/cmd_vel', 10)


        self.create_timer(0.02, self._monitor_loop)
        self.get_logger().info('paint_executor_node started')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _strip_map_cb(self, msg: StripMap):
        if not self._active:
            self._strip_map = msg
            # self.get_logger().info(
            #     f'[executor] StripMap received — '
            #     f'z=[{msg.strip_z_min:.2f},{msg.strip_z_max:.2f}] '
            #     f'zones={len(msg.z_start)}')

    def _z_state_cb(self, msg: PaintHeadStatus):
        self._z_state = msg

        if not self._wrist_init_done:
            self.get_logger().info('neutral wrist set here')
            self._set_wrist(WRIST_NEUTRAL_DEG)
            self._wrist_init_done = True

        # Track the actuator's homed truth in BOTH directions. limit_lower_z
        # carries _homed from actuator_bridge. It latches our local _z_homed on
        # the rising edge (homing complete), and — critically — clears it on the
        # falling edge. An ESTOP clears the actuator's limits (_homed→False), so
        # limit_lower_z goes False; without clearing _z_homed here, _do_homing()
        # would see a stale "already homed", skip re-teach, and every stroke would
        # fail "Z not homed". This is the single point that keeps the executor's
        # homed view in sync with the actuator (the authority).
        if msg.limit_lower_z and not self._z_homed:
            self._z_homed = True
            self.get_logger().info('[executor] Homing complete')
            done = Bool()
            done.data = True
            self._homing_pub.publish(done)
        elif not msg.limit_lower_z and self._z_homed:
            self._z_homed = False
            self.get_logger().warn('[executor] homed cleared (actuator un-homed — ESTOP/re-teach)')

    def _joint_state_cb(self, msg):
        """Track extension_joint position for extension settle monitoring."""
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
        """Only used for abort — if FSM leaves PAINT_EXECUTE unexpectedly."""
        prev = self._fsm_state
        self._fsm_state = msg.data
        if prev == 'PAINT_EXECUTE' and msg.data != 'PAINT_EXECUTE':
             if self._active or self._pending_stroke or self._awaiting_extension:
                self.get_logger().warn('[executor] FSM left PAINT_EXECUTE — aborting')
                self._abort_stroke()

    def _robot_command_cb(self, msg: RobotCommand):
        """Trigger stroke on new PAINT_EXECUTE command_id only."""
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
        """Return required extension position for the current strip.
        0.0 if strip_z_max is within normal gantry reach.
        Clamped to EXT_MAX_M.
        """
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
        """Send extension command and start settle monitoring."""
        cmd = Float32()
        cmd.data = float(target_m)
        self._ext_pub.publish(cmd)
        self._ext_target_m = target_m
        self._ext_settle_start = self.get_clock().now()
        self._awaiting_extension = True
        self.get_logger().info(
            f'[executor] Extension command → {target_m:.3f}m '
            f'(current={self._ext_pos_m:.3f}m)')

    def _extension_settled(self) -> bool:
        """True if extension has reached its target within tolerance."""
        tol = self.get_parameter('ext_tol_m').value
        return abs(self._ext_pos_m - self._ext_target_m) <= tol

    # ------------------------------------------------------------------
    # Stroke control
    # ------------------------------------------------------------------
    def _prepare_stroke(self):
        """Check if extension needs to move before starting the stroke.
        Returns True if we can proceed immediately (extension already in position).
        Returns False if we need to wait for the extension (sets _awaiting_extension).
        """
        req_ext = self._required_extension_m()
        ext_tol = self.get_parameter('ext_tol_m').value

        if abs(req_ext - self._ext_target_m) <= ext_tol and not self._awaiting_extension:
            # Extension is already at the right position (or not needed)
            if abs(self._ext_pos_m - req_ext) <= ext_tol:
                return True   # ready to stroke immediately
            # Extension not physically there yet — command it and wait
            self._command_extension(req_ext)
            return False

        if abs(req_ext - self._ext_target_m) > ext_tol:
            # Different extension target needed — command it
            self._command_extension(req_ext)
            return False

        return self._extension_settled()

    def _start_stroke(self):
        if self._strip_map is None:
            self.get_logger().error('[executor] No StripMap — cannot start stroke')
            return
        if self._z_state is None:
            self.get_logger().error('[executor] No PaintHeadStatus — cannot start stroke')
            return
        if not self._z_state.limit_lower_z:
            self.get_logger().error('[executor] Z not homed — cannot start stroke')
            return

        self._active        = True
        self._spray_state   = False
        self._wrist_flicked = False
        self._finish_start  = None
        self._stroke_start_time = self.get_clock().now()

        # Guard against a degenerate (zero-length) stroke after an interruption.
        # Normal painting alternates _direction each completed stroke. But after a
        # HOLD abort, Z is parked wherever it stopped and _direction may point at
        # the end Z is ALREADY at — so the monitor loop's done-check fires on the
        # first tick, the stroke "completes" in one tick, and spray toggles on→off
        # before anything is painted ("moves but no paint"). If the current target
        # end is where Z already sits, flip direction so the stroke actually
        # traverses the strip. This preserves normal alternation and only corrects
        # the post-interruption case.
        tol  = self.get_parameter('position_tol_m').value
        z_m  = self._z_state.z_actual_pos_mm / 1000.0
        smap = self._strip_map

        # Clamp stroke target to normal gantry range (z_normal_max_m).
        # The extension provides extra reach — the actual painted range is
        # [strip_z_min, min(strip_z_max, z_normal_max)] within zAxis_joint space,
        # because extension_joint already raised the whole assembly.
        z_normal_max = self.get_parameter('z_normal_max_m').value
        effective_z_max = min(smap.strip_z_max, z_normal_max)

        at_target = (z_m >= effective_z_max - tol) if self._direction > 0 \
                    else (z_m <= smap.strip_z_min + tol)
        if at_target:
            self._direction *= -1
            self.get_logger().warn(
                f'[executor] Z already at stroke target ({z_m:.3f}m) — '
                f'flipping direction to traverse')

        target_mm = (effective_z_max * 1000.0 if self._direction > 0
                     else smap.strip_z_min * 1000.0)

        self.get_logger().info(
            f'[executor] Stroke {"UP" if self._direction > 0 else "DOWN"} '
            f'→ {target_mm/1000.0:.2f}m '
            f'[ext={self._ext_pos_m:.3f}m, effective_z_max={effective_z_max:.3f}m]')

        self._send_z(ZAxisCmd.Z_POSITION, target_mm,
                     self.get_parameter('z_travel_rpm').value)
        initial_spray = self._spray_for_position(z_m)
        self._set_spray(initial_spray)
        # self._set_wrist(WRIST_NEUTRAL_DEG)

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

    def _complete_stroke(self):
        self._active    = False
        self._stroke_start_time = None
        self._finish_start = None
        self._direction *= -1
        self._set_spray(False)
        # self._set_wrist(WRIST_NEUTRAL_DEG)
        msg = Bool()
        msg.data = True
        self._complete_pub.publish(msg)
        self.get_logger().info('[executor] Stroke complete')

    def _timeout_stroke(self, current_z_mm: float, target_mm: float):
        """Z didn't reach target within stroke_timeout_s — stop Z, abort the
        stroke, then publish /stroke_complete anyway (direction is NOT
        flipped, unlike a real completion) so the FSM's existing
        stroke_complete handling frees PAINT_EXECUTE and resumes
        WALL_FOLLOW instead of hanging forever. The next PAINT_EXECUTE
        entry will retry the same unfinished direction/target."""
        self.get_logger().error(
            f'[executor] Z stroke timeout: target={target_mm:.1f}mm '
            f'current={current_z_mm:.1f}mm — aborting stroke, resuming WALL_FOLLOW')
        self._abort_stroke()
        msg = Bool()
        msg.data = True
        self._complete_pub.publish(msg)

    def _lerp(self, a, b, f):
        f = max(0.0, min(1.0, f))
        return a + (b - a) * f

    def _final_wrist_deg(self, direction):
        """Flick angle commanded at the END of a stroke in this direction."""
        return WRIST_UP_DEG if direction > 0 else WRIST_DOWN_DEG

    def _wrist_target_for_position(self, z, z_min, z_max, direction):
        w    = self.get_parameter('wrist_flick_dist_m').value
        lead = self.get_parameter('wrist_ramp_lead_m').value
        # Short strips (taught range <= 2*w): the start ramp and end ramp would
        # overlap and the "flick at end" branch is never reached. Cap each ramp
        # to half the strip so both ends always get their own ramp.
        w    = min(w, 0.5 * (z_max - z_min))
        lead = min(lead, 0.5 * w)
        # Ramp at the stroke END completes `lead` metres early (span < w) so the
        # servo is already at the final angle when Z arrives / spray closes.
        span = max(w - lead, 1e-3)

        if direction > 0:   # UPSTROKE
            # Transit (start through middle) stays at neutral; only the end flicks.
            if z < z_max - w:
                return WRIST_NEUTRAL_DEG
            else:
                return self._lerp(WRIST_NEUTRAL_DEG, WRIST_UP_DEG, (z - (z_max - w)) / span)
        else:               # DOWNSTROKE
            if z > z_min + w:
                return WRIST_NEUTRAL_DEG
            else:
                return self._lerp(WRIST_NEUTRAL_DEG, WRIST_DOWN_DEG, ((z_min + w) - z) / span)

    # ------------------------------------------------------------------
    # Monitor loop at 50Hz
    # ------------------------------------------------------------------
    def _monitor_loop(self):
        # --- Phase 1: Wait for base to be stationary ---
        if self._pending_stroke:
            now = self.get_clock().now()
            elapsed = (now - self._pending_wait_start).nanoseconds * 1e-9
            speed_tol = self.get_parameter('stationary_speed_tol_mps').value
            yaw_tol = self.get_parameter('stationary_yaw_tol_radps').value
            timeout = self.get_parameter('stationary_timeout_s').value

            is_stationary = (not self._has_odom or (self._base_speed <= speed_tol and self._base_yaw_rate <= yaw_tol))
            if is_stationary or elapsed >= timeout:
                if elapsed >= timeout and not is_stationary:
                    self.get_logger().warn(
                        f'[executor] stationary timeout ({elapsed:.2f}s) — '
                        f'speed={self._base_speed:.3f}m/s, yaw_rate={self._base_yaw_rate:.3f}rad/s, proceeding with stroke')
                else:
                    self.get_logger().info(
                        f'[executor] base is stationary (speed={self._base_speed:.3f}m/s, yaw={self._base_yaw_rate:.3f}rad/s) → checking extension')
                self._pending_stroke = False

                # --- Phase 2: Extension check before stroke ---
                if not self._prepare_stroke():
                    # _awaiting_extension is True — monitor_loop will handle it
                    return
                # Extension already in position — go straight to stroke
                self._start_stroke()
            return

        # --- Phase 2: Waiting for extension to reach its target ---
        if self._awaiting_extension:
            now = self.get_clock().now()
            if self._extension_settled():
                self.get_logger().info(
                    f'[executor] Extension settled at {self._ext_pos_m:.3f}m → starting stroke')
                self._awaiting_extension = False
                self._start_stroke()
            else:
                # Failsafe: if extension takes too long, abort
                if self._ext_settle_start is not None:
                    elapsed = (now - self._ext_settle_start).nanoseconds * 1e-9
                    if elapsed >= self.get_parameter('ext_settle_timeout_s').value:
                        self.get_logger().error(
                            f'[executor] Extension settle TIMEOUT ({elapsed:.1f}s) '
                            f'— pos={self._ext_pos_m:.3f}m target={self._ext_target_m:.3f}m '
                            f'— aborting stroke')
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
            # Z is at the stroke end. Do NOT close the nozzle yet: command the
            # final flick angle and keep spray open until the MEASURED wrist
            # angle has arrived (or the settle timeout expires), then complete.
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
                if timed_out and not settled:
                    self.get_logger().warn(
                        f'[executor] wrist settle timeout ({waited_s:.2f}s) — '
                        f'actual={self._z_state.wrist_actual_deg:.1f}° target={final_deg:.1f}°')
                self._complete_stroke()
            return

        target_mm = (effective_z_max * 1000.0 if self._direction > 0
                     else smap.strip_z_min * 1000.0)
        if self._stroke_start_time is not None:
            elapsed_s = (self.get_clock().now() - self._stroke_start_time).nanoseconds * 1e-9
            if elapsed_s > self.get_parameter('stroke_timeout_s').value:
                self._timeout_stroke(self._z_state.z_actual_pos_mm, target_mm)
                return

        desired = self._spray_for_position(z)
        if desired != self._spray_state:
            self._set_spray(desired)

        wrist_deg = self._wrist_target_for_position(z, smap.strip_z_min, effective_z_max, self._direction)
        self._set_wrist(wrist_deg)

    def _spray_for_position(self, z: float) -> bool:
        smap = self._strip_map
        z = max(smap.strip_z_min, min(smap.strip_z_max, z)) # clamp to prevent -0.00 from triggering issues
        self.get_logger().debug(f'[spray] z={z:.3f} dir={self._direction} min={smap.strip_z_min} max={smap.strip_z_max}')

        for i in range(len(smap.z_start)):
            if smap.z_start[i] <= z <= smap.z_end[i]:
                return smap.paint_on[i]
        return False

    # ------------------------------------------------------------------
    # Actuator helpers
    # ------------------------------------------------------------------

    def _do_homing(self):
        # Homing is manual (joystick LB/A/Y — no physical limit switches on
        # this machine) and is meant to be taught once, then reused across
        # every run. If it's already been taught, don't re-arm teach mode
        # here — that would wipe the recorded limits every SYSTEM_CHECK.
        # Just re-affirm completion so the FSM proceeds immediately.

        self._set_wrist(WRIST_NEUTRAL_DEG)

        if self._z_homed:
            self.get_logger().info('[executor] Already homed — skipping re-home')
            done = Bool()
            done.data = True
            self._homing_pub.publish(done)
            return

        self.get_logger().warn(
            '[executor] Not homed — arming teach-homing '
            '(joystick: LB=arm, A=set lower, Y=set upper)')
        cmd = ZAxisCmd()
        cmd.z_mode = ZAxisCmd.Z_HOME
        self._z_pub.publish(cmd)
        # PaintHeadStatus cb will detect limit_lower_z and publish homing_complete

   

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


def main(args=None):
    rclpy.init(args=args)
    node = PaintExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from painting_robot_msgs.msg import (
    StripMap, ZAxisCmd, WristCmd, SprayCmd,
    PaintHeadStatus, RobotCommand
)

WRIST_NEUTRAL_DEG   =  0.0
WRIST_UP_DEG        = -45.0
WRIST_DOWN_DEG      = 45.0
WRIST_FLICK_DIST_M  =  0.6
POSITION_TOL_M      =  0.006
# Wrist ramp finishes this far BEFORE the stroke end so the (laggy) servo has
# travel distance to physically arrive before Z stops and the nozzle closes.
WRIST_RAMP_LEAD_M   =  0.10
# End-of-stroke: keep spray open until the measured wrist angle is within this
# many degrees of the final flick angle...
WRIST_SETTLE_TOL_DEG = 3.0
# ...but never longer than this (failsafe: wrist stalled / no feedback / no driver).
WRIST_SETTLE_TIMEOUT_S = 0.45
Z_TRAVEL_RPM        =  0  # let Actuator params decide speed

# Fixed failsafe timeout for a commanded Z stroke. Not distance/speed derived:
# actual mm/s depends on hardware calibration constants (counts_per_mm,
# motor counts_per_rev) that live in actuator_bridge_node/leadshine_driver,
# not here — duplicating them into this node would risk silently drifting
# out of sync with the real hardware config. Override via the
# 'stroke_timeout_s' param if 20s proves too tight/loose on real hardware.
STROKE_TIMEOUT_S    =  20.0


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

        self._strip_map     = None
        self._z_state       = None
        self._direction     = 1
        self._active        = False
        self._spray_state   = False
        self._wrist_flicked = False
        self._fsm_state     = ''
        self._last_command_id = -1       # gate: only act on new command_id
        self._z_homed         = False
        self._stroke_start_time = None   # set on stroke start; gates timeout check
        self._wrist_init_done = False
        self._finish_start    = None     # set when Z reaches stroke end; wrist-settle phase

        self._base_speed         = 0.0
        self._base_yaw_rate      = 0.0
        self._has_odom           = False
        self._pending_stroke     = False
        self._pending_wait_start = None

        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(StripMap,       '/strip_map',         self._strip_map_cb,    latch_qos)
        self.create_subscription(PaintHeadStatus,'/paint_head_status', self._z_state_cb,      10)
        self.create_subscription(String,         '/robot_state',       self._fsm_state_cb,    10)
        self.create_subscription(RobotCommand,   '/robot_command',     self._robot_command_cb, 10)
        self.create_subscription(Odometry,       '/paint_bot/odom',    self._odom_cb,         10)

        self._z_pub        = self.create_publisher(ZAxisCmd,  '/z_axis_cmd',     10)
        self._wrist_pub    = self.create_publisher(WristCmd,  '/wrist_cmd',      10)
        self._spray_pub    = self.create_publisher(SprayCmd,  '/spray_cmd',      10)
        self._complete_pub = self.create_publisher(Bool,      '/stroke_complete', 10)
        self._homing_pub   = self.create_publisher(Bool,      '/homing_complete', 10)
        self._stop_base_pub = self.create_publisher(Twist,    '/paint_bot/cmd_vel', 10)


        self.create_timer(0.02, self._monitor_loop)
        self.get_logger().info('paint_executor_node started')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _strip_map_cb(self, msg: StripMap):
        if not self._active:
            self._strip_map = msg
            # self.get_logger().info(
            #     f'[executor] StripMap received — '
            #     f'z=[{msg.strip_z_min:.2f},{msg.strip_z_max:.2f}] '
            #     f'zones={len(msg.z_start)}')

    def _z_state_cb(self, msg: PaintHeadStatus):
        self._z_state = msg

        if not self._wrist_init_done:
            self.get_logger().info('neutral wrist set here')
            self._set_wrist(WRIST_NEUTRAL_DEG)
            self._wrist_init_done = True

        # Track the actuator's homed truth in BOTH directions. limit_lower_z
        # carries _homed from actuator_bridge. It latches our local _z_homed on
        # the rising edge (homing complete), and — critically — clears it on the
        # falling edge. An ESTOP clears the actuator's limits (_homed→False), so
        # limit_lower_z goes False; without clearing _z_homed here, _do_homing()
        # would see a stale "already homed", skip re-teach, and every stroke would
        # fail "Z not homed". This is the single point that keeps the executor's
        # homed view in sync with the actuator (the authority).
        if msg.limit_lower_z and not self._z_homed:
            self._z_homed = True
            self.get_logger().info('[executor] Homing complete')
            done = Bool()
            done.data = True
            self._homing_pub.publish(done)
        elif not msg.limit_lower_z and self._z_homed:
            self._z_homed = False
            self.get_logger().warn('[executor] homed cleared (actuator un-homed — ESTOP/re-teach)')

    def _odom_cb(self, msg: Odometry):
        self._has_odom = True
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        self._base_speed = math.hypot(vx, vy)
        self._base_yaw_rate = abs(wz)

    def _fsm_state_cb(self, msg: String):
        """Only used for abort — if FSM leaves PAINT_EXECUTE unexpectedly."""
        prev = self._fsm_state
        self._fsm_state = msg.data
        if prev == 'PAINT_EXECUTE' and msg.data != 'PAINT_EXECUTE':
             if self._active or self._pending_stroke:
                self.get_logger().warn('[executor] FSM left PAINT_EXECUTE — aborting')
                self._abort_stroke()

    def _robot_command_cb(self, msg: RobotCommand):
        """Trigger stroke on new PAINT_EXECUTE command_id only."""
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
    # Stroke control
    # ------------------------------------------------------------------
    def _start_stroke(self):
        if self._strip_map is None:
            self.get_logger().error('[executor] No StripMap — cannot start stroke')
            return
        if self._z_state is None:
            self.get_logger().error('[executor] No PaintHeadStatus — cannot start stroke')
            return
        if not self._z_state.limit_lower_z:
            self.get_logger().error('[executor] Z not homed — cannot start stroke')
            return

        self._active        = True
        self._spray_state   = False
        self._wrist_flicked = False
        self._finish_start  = None
        self._stroke_start_time = self.get_clock().now()

        # Guard against a degenerate (zero-length) stroke after an interruption.
        # Normal painting alternates _direction each completed stroke. But after a
        # HOLD abort, Z is parked wherever it stopped and _direction may point at
        # the end Z is ALREADY at — so the monitor loop's done-check fires on the
        # first tick, the stroke "completes" in one tick, and spray toggles on→off
        # before anything is painted ("moves but no paint"). If the current target
        # end is where Z already sits, flip direction so the stroke actually
        # traverses the strip. This preserves normal alternation and only corrects
        # the post-interruption case.
        tol  = self.get_parameter('position_tol_m').value
        z_m  = self._z_state.z_actual_pos_mm / 1000.0
        smap = self._strip_map
        at_target = (z_m >= smap.strip_z_max - tol) if self._direction > 0 \
                    else (z_m <= smap.strip_z_min + tol)
        if at_target:
            self._direction *= -1
            self.get_logger().warn(
                f'[executor] Z already at stroke target ({z_m:.3f}m) — '
                f'flipping direction to traverse')

        target_mm = (smap.strip_z_max * 1000.0 if self._direction > 0
                     else smap.strip_z_min * 1000.0)

        self.get_logger().info(
            f'[executor] Stroke {"UP" if self._direction > 0 else "DOWN"} '
            f'→ {target_mm/1000.0:.2f}m')

        self._send_z(ZAxisCmd.Z_POSITION, target_mm,
                     self.get_parameter('z_travel_rpm').value)
        initial_spray = self._spray_for_position(z_m)
        self._set_spray(initial_spray)
        # self._set_wrist(WRIST_NEUTRAL_DEG)

    def _abort_stroke(self):
        if not self._active and not self._pending_stroke:
            return
        self._active = False
        self._pending_stroke = False
        self._stroke_start_time = None
        self._finish_start = None
        self._set_spray(False)
        self._set_wrist(WRIST_NEUTRAL_DEG)
        self._send_z(ZAxisCmd.Z_STOP, 0.0, 0)

    def _complete_stroke(self):
        self._active    = False
        self._stroke_start_time = None
        self._finish_start = None
        self._direction *= -1
        self._set_spray(False)
        # self._set_wrist(WRIST_NEUTRAL_DEG)
        msg = Bool()
        msg.data = True
        self._complete_pub.publish(msg)
        self.get_logger().info('[executor] Stroke complete')

    def _timeout_stroke(self, current_z_mm: float, target_mm: float):
        """Z didn't reach target within stroke_timeout_s — stop Z, abort the
        stroke, then publish /stroke_complete anyway (direction is NOT
        flipped, unlike a real completion) so the FSM's existing
        stroke_complete handling frees PAINT_EXECUTE and resumes
        WALL_FOLLOW instead of hanging forever. The next PAINT_EXECUTE
        entry will retry the same unfinished direction/target."""
        self.get_logger().error(
            f'[executor] Z stroke timeout: target={target_mm:.1f}mm '
            f'current={current_z_mm:.1f}mm — aborting stroke, resuming WALL_FOLLOW')
        self._abort_stroke()
        msg = Bool()
        msg.data = True
        self._complete_pub.publish(msg)

    def _lerp(self, a, b, f):
        f = max(0.0, min(1.0, f))
        return a + (b - a) * f

    def _final_wrist_deg(self, direction):
        """Flick angle commanded at the END of a stroke in this direction."""
        return WRIST_UP_DEG if direction > 0 else WRIST_DOWN_DEG

    def _wrist_target_for_position(self, z, z_min, z_max, direction):
        w    = self.get_parameter('wrist_flick_dist_m').value
        lead = self.get_parameter('wrist_ramp_lead_m').value
        # Short strips (taught range <= 2*w): the start ramp and end ramp would
        # overlap and the "flick at end" branch is never reached. Cap each ramp
        # to half the strip so both ends always get their own ramp.
        w    = min(w, 0.5 * (z_max - z_min))
        lead = min(lead, 0.5 * w)
        # Ramp at the stroke END completes `lead` metres early (span < w) so the
        # servo is already at the final angle when Z arrives / spray closes.
        span = max(w - lead, 1e-3)

        if direction > 0:   # UPSTROKE
            # Transit (start through middle) stays at neutral; only the end flicks.
            if z < z_max - w:
                return WRIST_NEUTRAL_DEG
            else:
                return self._lerp(WRIST_NEUTRAL_DEG, WRIST_UP_DEG, (z - (z_max - w)) / span)
        else:               # DOWNSTROKE
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
                if elapsed >= timeout and not is_stationary:
                    self.get_logger().warn(
                        f'[executor] stationary timeout ({elapsed:.2f}s) — '
                        f'speed={self._base_speed:.3f}m/s, yaw_rate={self._base_yaw_rate:.3f}rad/s, proceeding with stroke')
                else:
                    self.get_logger().info(
                        f'[executor] base is stationary (speed={self._base_speed:.3f}m/s, yaw={self._base_yaw_rate:.3f}rad/s) → starting stroke')
                self._pending_stroke = False
                self._start_stroke()
            return

        if not self._active or self._z_state is None or self._strip_map is None:
            return

        z           = self._z_state.z_actual_pos_mm / 1000.0
        smap        = self._strip_map
        tol         = self.get_parameter('position_tol_m').value

        done = (z >= smap.strip_z_max - tol if self._direction > 0
                else z <= smap.strip_z_min + tol)
        if done:
            # Z is at the stroke end. Do NOT close the nozzle yet: command the
            # final flick angle and keep spray open until the MEASURED wrist
            # angle has arrived (or the settle timeout expires), then complete.
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
                if timed_out and not settled:
                    self.get_logger().warn(
                        f'[executor] wrist settle timeout ({waited_s:.2f}s) — '
                        f'actual={self._z_state.wrist_actual_deg:.1f}° target={final_deg:.1f}°')
                self._complete_stroke()
            return

        target_mm = (smap.strip_z_max * 1000.0 if self._direction > 0
                     else smap.strip_z_min * 1000.0)
        if self._stroke_start_time is not None:
            elapsed_s = (self.get_clock().now() - self._stroke_start_time).nanoseconds * 1e-9
            if elapsed_s > self.get_parameter('stroke_timeout_s').value:
                self._timeout_stroke(self._z_state.z_actual_pos_mm, target_mm)
                return

        desired = self._spray_for_position(z)
        if desired != self._spray_state:
            self._set_spray(desired)

        wrist_deg = self._wrist_target_for_position(z, smap.strip_z_min, smap.strip_z_max, self._direction)
        self._set_wrist(wrist_deg)

    def _spray_for_position(self, z: float) -> bool:
        smap = self._strip_map
        z = max(smap.strip_z_min, min(smap.strip_z_max, z)) # clamp to prevent -0.00 from triggering issues
        self.get_logger().debug(f'[spray] z={z:.3f} dir={self._direction} min={smap.strip_z_min} max={smap.strip_z_max}')

        for i in range(len(smap.z_start)):
            if smap.z_start[i] <= z <= smap.z_end[i]:
                return smap.paint_on[i]
        return False

    # ------------------------------------------------------------------
    # Actuator helpers
    # ------------------------------------------------------------------

    def _do_homing(self):
        # Homing is manual (joystick LB/A/Y — no physical limit switches on
        # this machine) and is meant to be taught once, then reused across
        # every run. If it's already been taught, don't re-arm teach mode
        # here — that would wipe the recorded limits every SYSTEM_CHECK.
        # Just re-affirm completion so the FSM proceeds immediately.

        self._set_wrist(WRIST_NEUTRAL_DEG)

        if self._z_homed:
            self.get_logger().info('[executor] Already homed — skipping re-home')
            done = Bool()
            done.data = True
            self._homing_pub.publish(done)
            return

        self.get_logger().warn(
            '[executor] Not homed — arming teach-homing '
            '(joystick: LB=arm, A=set lower, Y=set upper)')
        cmd = ZAxisCmd()
        cmd.z_mode = ZAxisCmd.Z_HOME
        self._z_pub.publish(cmd)
        # PaintHeadStatus cb will detect limit_lower_z and publish homing_complete

   

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


def main(args=None):
    rclpy.init(args=args)
    node = PaintExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()