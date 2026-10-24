#!/usr/bin/env python3
"""
joy_teleop_node.py

Headless joystick translator for EdgeBox hardware deployment.
Subscribes to /joy (from joy_node daemon) and translates to robot topics.

Logitech F310 — XInput mode.

Operator commands all publish to /start_cmd (std_msgs/String). Two consumers
filter it: the FSM acts on job-control verbs (START / PAUSE / RESUME /
TEST_STROKE); safety_node acts on safety verbs (ESTOP / RESET). Joy does not
pre-classify — it just reports what the operator pressed.

Button / Axis map:
  Right stick Y (axis 4)  — linear_x drive          (RB deadman required)
  Right stick X (axis 3)  — angular_z steer          (RB deadman required)
  RB        (btn 5)        — deadman hold
  Start     (btn 7)        — START if IDLE, else PAUSE/RESUME toggle (not in FAULT)
  Back+Start together      — RESET  (safety-routed)
  X         (btn 2)        — ESTOP  (safety-routed → relay off → FAULT)
  RB + RT                  — TEST_STROKE
  LB        (btn 4)        — arm manual teach-homing → /z_axis_cmd Z_HOME
  LT        (axis 2)       — Solenoid hold → /spray_cmd
  D-pad up/dn (axis 7)     — Z jog (IDLE / SYSTEM_CHECK only)
  A         (btn 0)        — set LOWER Z limit → /gantry_limit_cmd
  Y         (btn 3)        — set UPPER Z limit → /gantry_limit_cmd
  B         (btn 1)        — toggle scan recording

Publishes:
  /paint_bot/cmd_vel  (geometry_msgs/Twist)
  /start_cmd          (std_msgs/String)   — START|PAUSE|RESUME|ESTOP|RESET|TEST_STROKE
  /spray_cmd          (painting_robot_msgs/SprayCmd)
  /z_axis_cmd         (painting_robot_msgs/ZAxisCmd)
  /gantry_limit_cmd   (painting_robot_msgs/GantryLimitCmd)

Subscribes:
  /joy          (sensor_msgs/Joy)
  /robot_state  (std_msgs/String)   — gates Z jog + drives Start-button context
"""

import os
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from painting_robot_msgs.msg import SprayCmd, ZAxisCmd, GantryLimitCmd


# ── F310 XInput axis / button indices ────────────────────────────────────────
AXIS_RIGHT_Y  = 4      # linear_x
AXIS_RIGHT_X  = 3      # angular_z
AXIS_LT       = 2      # solenoid
AXIS_DPAD_Y   = 7      # D-pad up(+1)/down(-1)
AXIS_RT       = 5      # right trigger

BTN_A         = 0      # set LOWER Z limit
BTN_B         = 1      # toggle scan recording
BTN_X         = 2      # ESTOP
BTN_Y         = 3      # set UPPER Z limit
BTN_LB        = 4      # arm manual teach-homing
BTN_RB        = 5      # deadman
BTN_BACK      = 6      # (RESET only via Back+Start combo)
BTN_START     = 7      # START / PAUSE-RESUME context toggle

# ── Thresholds / constants ────────────────────────────────────────────────────
RT_THRESHOLD      = -0.5
LT_THRESHOLD      = -0.5
MAX_LINEAR_MPS    =  0.20
MAX_ANGULAR_RADPS =  0.40
Z_JOG_RPM         =  700
DPAD_THRESHOLD    =  0.5

SCAN_CAPTURE_BASE = os.path.expanduser(
    '~/ros_ws/painting_robot_ws/captured_data/lidar_scans')


class JoyTeleopNode(Node):

    def __init__(self):
        super().__init__('joy_teleop_node')

        # ── State ─────────────────────────────────────────────────────────────
        self._fsm_state  = 'IDLE'
        self._reset_sent = False
        self._paused     = False    # local view of pause toggle

        self._prev_buttons = []
        self._solenoid_on  = False
        self._test_stroke_sent = False
        self._z_jogging = 0

        self._recording    = False
        self._record_dir   = None
        self._record_count = 0

        # ── Publishers ────────────────────────────────────────────────────────
        self._vel_pub    = self.create_publisher(Twist, '/paint_bot/cmd_vel', 10)
        self._start_pub  = self.create_publisher(String, '/start_cmd', 10)
        self._spray_pub  = self.create_publisher(SprayCmd, '/spray_cmd', 10)
        self._zaxis_pub  = self.create_publisher(ZAxisCmd, '/z_axis_cmd', 10)
        self._gantry_pub = self.create_publisher(GantryLimitCmd, '/gantry_limit_cmd', 10)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Joy,    '/joy',         self._joy_cb,   10)
        self.create_subscription(String, '/robot_state', self._state_cb, 10)
        self.create_subscription(LaserScan, '/scan',      self._scan_cb,  10)
        # Live safety tier for the X escalation ladder. /safety_reason is the only
        # source that distinguishes FAULT from ESTOP (/robot_state collapses both).
        self.create_subscription(String, '/safety_reason', self._safety_reason_cb, 10)
        self._safety_tier = 'NOMINAL'

        self.get_logger().info('joy_teleop_node started — F310 XInput mode')

    # ── State ──────────────────────────────────────────────────────────────────
    def _state_cb(self, msg: String):
        self._fsm_state = msg.data
        # keep local pause view honest if FSM left a running/idle state
        if self._fsm_state in ('IDLE', 'FAULT'):
            self._paused = False

    def _safety_reason_cb(self, msg: String):
        # "NOMINAL" or "<TIER>: <reason>" — take the leading word as current tier.
        self._safety_tier = msg.data.split(':', 1)[0].strip().upper() or 'NOMINAL'

    # ── Scan recording ─────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        if not self._recording:
            return
        scan_dict = {
            'ranges':          list(msg.ranges),
            'angle_min':       float(msg.angle_min),
            'angle_max':       float(msg.angle_max),
            'angle_increment': float(msg.angle_increment),
            'range_min':       float(msg.range_min),
            'range_max':       float(msg.range_max),
            'scan_index':      self._record_count,
        }
        np.save(os.path.join(self._record_dir, f'scan_{self._record_count:04d}.npy'),
                scan_dict)
        self._record_count += 1

    def _toggle_recording(self):
        if self._recording:
            self._recording = False
            self.get_logger().info(
                f'[JOY] Recording stopped — {self._record_count} frames → {self._record_dir}')
        else:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._record_dir = os.path.join(SCAN_CAPTURE_BASE, f'run_{ts}')
            os.makedirs(self._record_dir, exist_ok=True)
            self._record_count = 0
            self._recording = True
            self.get_logger().info(f'[JOY] Recording started → {self._record_dir}')

    # ── Main joy callback ──────────────────────────────────────────────────────
    def _joy_cb(self, msg: Joy):
        axes    = msg.axes
        buttons = list(msg.buttons)
        if not self._prev_buttons:
            self._prev_buttons = [0] * len(buttons)

        deadman = buttons[BTN_RB] if len(buttons) > BTN_RB else 0

        self._handle_drive(axes, deadman)
        self._handle_buttons(buttons)
        self._handle_solenoid(axes)
        self._handle_z_jog(axes)
        self._handle_test_stroke(axes, buttons)

        self._prev_buttons = buttons

    # ── Drive ───────────────────────────────────────────────────────────────────
    def _handle_drive(self, axes, deadman: int):
        cmd = Twist()
        if deadman:
            lx = axes[AXIS_RIGHT_Y] if len(axes) > AXIS_RIGHT_Y else 0.0
            az = axes[AXIS_RIGHT_X] if len(axes) > AXIS_RIGHT_X else 0.0
            cmd.linear.x  = lx * MAX_LINEAR_MPS
            cmd.angular.z = az * MAX_ANGULAR_RADPS
            self._vel_pub.publish(cmd)

    # ── Test stroke (RB + RT) ────────────────────────────────────────────────────
    def _handle_test_stroke(self, axes, buttons):
        rb = buttons[BTN_RB] if len(buttons) > BTN_RB else 0
        rt = axes[AXIS_RT] if len(axes) > AXIS_RT else 0.0
        if rb and rt < RT_THRESHOLD:
            if not self._test_stroke_sent:
                self._send_cmd('TEST_STROKE')
                self._test_stroke_sent = True
        else:
            self._test_stroke_sent = False

    # ── Buttons ──────────────────────────────────────────────────────────────────
    def _handle_buttons(self, buttons):
        def pressed(idx):
            if idx >= len(buttons) or idx >= len(self._prev_buttons):
                return False
            return buttons[idx] == 1 and self._prev_buttons[idx] == 0

        start_held = buttons[BTN_START] if len(buttons) > BTN_START else 0
        back_held  = buttons[BTN_BACK]  if len(buttons) > BTN_BACK  else 0

        # Back + Start together → RESET (safety-routed). Check before individual.
        if start_held and back_held:
            if not self._reset_sent:
                self._send_cmd('RESET')
                self._reset_sent = True
            return
        else:
            self._reset_sent = False

        # Start alone → context toggle:
        #   IDLE            -> START
        #   FAULT           -> ignored (use RESET)
        #   any running st. -> PAUSE, then RESUME on next press
        if pressed(BTN_START):
            if self._fsm_state == 'IDLE':
                self._send_cmd('START')
                self._paused = False
            elif self._fsm_state == 'FAULT':
                self.get_logger().warn('[JOY] Start ignored in FAULT — use RESET (Back+Start)')
            else:
                if self._paused:
                    self._send_cmd('RESUME')
                    self._paused = False
                else:
                    self._send_cmd('PAUSE')
                    self._paused = True

        # X → escalation ladder, stepping from the CURRENT safety tier:
        #   NOMINAL              → HOLD
        #   SAFETY_HOLD/HOLD     → FAULT
        #   FAULT                → ESTOP
        #   ESTOP                → (top — repeat ESTOP)
        # Safety node owns the tiers; joy only requests the next step. Reading the
        # live tier (not a press counter) keeps joy in sync even when a sensor
        # raised the tier on its own. Ladder resets itself once RESET → NOMINAL.
        if pressed(BTN_X):
            tier = self._safety_tier
            if tier == 'NOMINAL':
                nxt = 'HOLD'
            elif tier in ('SAFETY_HOLD', 'HOLD'):
                nxt = 'FAULT'
            else:                      # FAULT or ESTOP
                nxt = 'ESTOP'
            self._send_cmd(nxt)
            self.get_logger().warn(f'[JOY] X escalation: tier={tier} → {nxt}')

        # Homing / limits / recording
        if pressed(BTN_LB):
            self._send_homing()
        if pressed(BTN_A):
            self._send_gantry_limit(set_lower=True)
        if pressed(BTN_Y):
            self._send_gantry_limit(set_upper=True)
        if pressed(BTN_B):
            self._toggle_recording()

    # ── Solenoid (LT hold) ───────────────────────────────────────────────────────
    def _handle_solenoid(self, axes):
        lt_val  = axes[AXIS_LT] if len(axes) > AXIS_LT else 0.0
        want_on = lt_val < LT_THRESHOLD
        if want_on == self._solenoid_on:
            return
        self._solenoid_on = want_on
        cmd = SprayCmd()
        cmd.valve_open    = want_on
        cmd.pump_enable   = False
        cmd.flow_setpoint = 0.0
        self._spray_pub.publish(cmd)
        self.get_logger().info(f'[JOY] Solenoid → {"ON" if want_on else "OFF"}')

    # ── Z jog (D-pad, IDLE/SYSTEM_CHECK only) ────────────────────────────────────
    def _handle_z_jog(self, axes):
        dpad_y = axes[AXIS_DPAD_Y] if len(axes) > AXIS_DPAD_Y else 0.0
        if dpad_y > DPAD_THRESHOLD:
            direction = 1
        elif dpad_y < -DPAD_THRESHOLD:
            direction = -1
        else:
            direction = 0

        if direction == self._z_jogging:
            return
        self._z_jogging = direction

        if self._fsm_state not in ('IDLE', 'SYSTEM_CHECK'):
            if direction != 0:
                self.get_logger().warn(
                    f'[JOY] Z jog blocked — FSM state is {self._fsm_state} (IDLE/SYSTEM_CHECK only)')
            return

        cmd = ZAxisCmd()
        if direction == 0:
            cmd.z_mode     = ZAxisCmd.Z_STOP
            cmd.z_velocity = 0
        else:
            cmd.z_mode     = ZAxisCmd.Z_VEL
            cmd.z_velocity = direction * int(Z_JOG_RPM)
            self.get_logger().info(
                f'[JOY] Z jog {"UP" if direction > 0 else "DOWN"} vel={cmd.z_velocity} rpm')
        self._zaxis_pub.publish(cmd)

    # ── Command helpers ──────────────────────────────────────────────────────────
    def _send_gantry_limit(self, set_lower=False, set_upper=False):
        cmd = GantryLimitCmd()
        cmd.set_lower = set_lower
        cmd.set_upper = set_upper
        self._gantry_pub.publish(cmd)
        self.get_logger().info(f'[JOY] Gantry limit — {"lower" if set_lower else "upper"}')

    def _send_cmd(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._start_pub.publish(msg)
        self.get_logger().info(f'[JOY] /start_cmd → {cmd}')

    def _send_homing(self):
        cmd = ZAxisCmd()
        cmd.z_mode = ZAxisCmd.Z_HOME
        self._zaxis_pub.publish(cmd)
        self.get_logger().info('[JOY] Direct Z_HOME sent')


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
