#!/usr/bin/env python3
"""
hmi_keyboard_node.py

Curses-based keyboard HMI for sim testing.
Combines teleop + FSM control + live status display in one terminal.

Controls (mirrors joystick command model — all operator cmds → /start_cmd):
  Arrow keys  — teleop (non-AUTO only)
  A           — START if IDLE ; PAUSE/RESUME toggle if running ; ignored in FAULT
  X           — ESTOP (→ safety_node → relay off → FAULT)
  A+Backspace — RESET (→ safety_node clears latch → IDLE)
  Space       — zero velocity (hold)
  R           — toggle scan recording
  +/-         — increase/decrease teleop speed
  H=home  W/S=Z jog  [/]=set lower/upper limit
  Q           — quit node

Display (curses, fixed refresh 10Hz):
  FSM state, wall mode, collision status
  Right wall: distance + angle error
  Front wall: min distance from collision warning
  Corner detected indicator
  Recording status + frame count
  Teleop speed multiplier
  Key legend

Teleop disabled in AUTO states (WALL_ALIGN, WALL_FOLLOW, CORNER_DETECT,
CORNER_LOGIC). Must STOP first to re-enable teleop.
"""

import curses
import math
import os
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from painting_robot_msgs.msg import WallPose, WallStatus, CollisionWarning, EnvStatus, ZAxisCmd, GantryLimitCmd, PaintHeadStatus
import rclpy.logging

# States where teleop is disabled
AUTO_STATES = {'SYSTEM_CHECK', 'WALL_ALIGN', 'WALL_FOLLOW',
               'STEP_REALIGN', 'CORNER_LOGIC', 'CORNER_APPROACH', 'PAINT_EXECUTE'}

TELEOP_SPEEDS = [0.05, 0.10, 0.15, 0.20]   # m/s options
TELEOP_ANG    = 0.4                          # rad/s fixed turn speed

Z_JOG_RPM        = 400      # Z jog speed (matches joystick Z_JOG_RPM)
JOG_RELEASE_S    = 0.15     # no key-repeat for this long → treat as released → Z_STOP
A_RESET_WINDOW_S = 1.0      # A+Backspace RESET: Backspace within this long after A
# States where Z jog / teaching is allowed (mirror joystick: IDLE + SYSTEM_CHECK)
Z_TEACH_STATES = {'IDLE', 'SYSTEM_CHECK'}

SCAN_CAPTURE_BASE = os.path.expanduser(
    '~/ros_ws/painting_robot_ws/captured_data/lidar_scans'
)

# Odom topic used only to tag recorded scans with turn progress (yaw).
ODOM_TOPIC = '/paint_bot/odom'


class HMINode(Node):

    def __init__(self):
        super().__init__('hmi_keyboard_node')

        # ── State cache ──────────────────────────────────────────────────────
        self.fsm_state       = 'IDLE'
        self.wall_mode       = '—'
        self.wall_pose       = None      # WallPose msg
        self.front_wall_pose = None

        self.wall_status     = None      # WallStatus msg
        self.collision       = None      # CollisionWarning msg
        self.env_status      = None      # EnvStatus msg
        self.corner_flash    = False     # blink flag
        self.speed_idx       = 1         # index into TELEOP_SPEEDS
        self._paused         = False     # local pause-toggle view (mirrors joystick)
        self.safety_reason   = '—'       # from /safety_reason (safety_node)
        self._last_cmd       = '—'       # last operator cmd (curses display, not logged)

        # ── Odom cache (for tagging recorded scans with turn progress) ───────
        self.yaw      = None             # latest odom yaw, rad
        self._odom_x  = None
        self._odom_y  = None

        # ── Z gantry / teach state ───────────────────────────────────────────
        self.head_status   = None        # PaintHeadStatus msg
        self._z_jogging    = 0           # -1 down, +1 up, 0 stopped (edge-tracked)

        # ── Depth recording state ────────────────────────────────────────────
        self._recording    = False
        self._record_dir   = None
        self._record_count = 0

        # ── Publishers ───────────────────────────────────────────────────────
        self._start_pub  = self.create_publisher(String, '/start_cmd', 10)
        self._teleop_pub = self.create_publisher(
            Twist, '/paint_bot/cmd_vel', 10)
        self._zaxis_pub  = self.create_publisher(ZAxisCmd,       '/z_axis_cmd',       10)
        self._gantry_pub = self.create_publisher(GantryLimitCmd, '/gantry_limit_cmd', 10)

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(String,          '/robot_state',      self._state_cb,          10)
        self.create_subscription(WallPose,        '/right_wall_pose',  self._wall_pose_cb,      10)
        self.create_subscription(WallPose,        '/front_wall_pose',  self._front_wall_pose_cb,10)
        self.create_subscription(WallStatus,      '/wall_status',      self._wall_status_cb,    10)
        self.create_subscription(CollisionWarning,'/collision_warning',self._collision_cb,      10)
        self.create_subscription(EnvStatus,       '/env_status',       self._env_status_cb,     10)
        self.create_subscription(LaserScan,       '/scan',             self._scan_cb,           10)
        self.create_subscription(PaintHeadStatus, '/paint_head_status',self._head_status_cb,    10)
        self.create_subscription(String,          '/safety_reason',    self._safety_reason_cb,  10)
        self.create_subscription(Odometry,        ODOM_TOPIC,          self._odom_cb,           10)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg):
        self.fsm_state = msg.data
        if self.fsm_state in ('IDLE', 'FAULT'):
            self._paused = False
    def _wall_mode_cb(self, msg):       self.wall_mode       = msg.data
    def _wall_pose_cb(self, msg):       self.wall_pose       = msg
    def _front_wall_pose_cb(self, msg): self.front_wall_pose = msg
    def _wall_status_cb(self, msg):     self.wall_status     = msg
    def _collision_cb(self, msg):       self.collision       = msg
    def _head_status_cb(self, msg):     self.head_status     = msg
    def _safety_reason_cb(self, msg):   self.safety_reason   = msg.data

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y

    def _env_status_cb(self, msg):
        self.env_status = msg
        if msg.corner_state in (EnvStatus.CORNER_APPROACHING, EnvStatus.CORNER_PREEMPT):
            self.corner_flash = True

    def _scan_cb(self, msg: LaserScan):
        if not self._recording:
            return
        # live right-wall pose from FSM, if valid (it drops out mid-turn —
        # that drop-out, logged against yaw, is exactly what we want to measure)
        wp = self.wall_pose
        scan_dict = {
            'ranges'         : list(msg.ranges),
            'angle_min'      : float(msg.angle_min),
            'angle_max'      : float(msg.angle_max),
            'angle_increment': float(msg.angle_increment),
            'range_min'      : float(msg.range_min),
            'range_max'      : float(msg.range_max),
            'scan_index'     : self._record_count,
            # ── tags for turn-progress analysis ──
            'stamp'          : msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'yaw'            : self.yaw,
            'odom_x'         : self._odom_x,
            'odom_y'         : self._odom_y,
            'fsm_state'      : self.fsm_state,
            'wall_valid'     : bool(wp.valid) if wp else None,
            'wall_dist'      : float(wp.distance) if (wp and wp.valid) else None,
            'wall_angle'     : float(wp.angle) if (wp and wp.valid) else None,
        }
        fname = os.path.join(self._record_dir, f'scan_{self._record_count:04d}.npy')
        np.save(fname, scan_dict)
        self._record_count += 1

    # ── Recording helpers ─────────────────────────────────────────────────────

    def toggle_recording(self):
        if self._recording:
            self._recording = False
            self._last_cmd = f'rec stopped ({self._record_count} frames)'
        else:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._record_dir = os.path.join(SCAN_CAPTURE_BASE, f'run_{ts}')
            os.makedirs(self._record_dir, exist_ok=True)
            self._record_count = 0
            self._recording    = True
            self._last_cmd = 'rec started'

    # ── Command helpers ───────────────────────────────────────────────────────

    def _pub_cmd(self, cmd: str):
        msg = String(); msg.data = cmd; self._start_pub.publish(msg)
        self._last_cmd = cmd            # shown on curses screen, not logged

    def send_start_toggle(self):
        """Mirror joystick Start: IDLE→START, running→PAUSE/RESUME, FAULT→ignored.
        RESET is A+Backspace (see key handler), not Start alone."""
        if self.fsm_state == 'IDLE':
            self._pub_cmd('START'); self._paused = False
        elif self.fsm_state == 'FAULT':
            self._last_cmd = 'START ignored in FAULT — A+Bksp to RESET'
        else:
            if self._paused:
                self._pub_cmd('RESUME'); self._paused = False
            else:
                self._pub_cmd('PAUSE');  self._paused = True

    def send_estop(self):
        # X escalates one tier from the CURRENT safety tier (read from
        # /safety_reason — the only source that distinguishes FAULT from ESTOP):
        #   NOMINAL → HOLD → FAULT → ESTOP (then stays ESTOP). Resets with RESET.
        tier = self.safety_reason.split(':', 1)[0].strip().upper()
        if tier == 'NOMINAL' or tier == '—' or tier == '':
            nxt = 'HOLD'
        elif tier in ('SAFETY_HOLD', 'HOLD'):
            nxt = 'FAULT'
        else:                      # FAULT or ESTOP
            nxt = 'ESTOP'
        self._pub_cmd(nxt)
        self._teleop_pub.publish(Twist())

    def send_reset(self):
        self._pub_cmd('RESET')

    def send_stop(self):
        # retained for quit-cleanup only: zero velocity, no STOP cmd (dropped)
        self._teleop_pub.publish(Twist())

    def send_teleop(self, lx, az):
        cmd = Twist()
        cmd.linear.x  = lx
        cmd.angular.z = az
        self._teleop_pub.publish(cmd)

    # ── Z gantry helpers (mirror joy_teleop_node) ─────────────────────────────

    def send_z_jog(self, direction: int):
        """direction: +1 up, -1 down, 0 stop. Only publishes on change (edge),
        exactly like the joystick's D-pad handler."""
        if direction == self._z_jogging:
            return
        # Gate to teach-capable states, same as joystick
        if direction != 0 and self.fsm_state not in Z_TEACH_STATES:
            self._last_cmd = f'Z jog blocked ({self.fsm_state})'
            return
        self._z_jogging = direction
        cmd = ZAxisCmd()
        if direction == 0:
            cmd.z_mode     = ZAxisCmd.Z_STOP
            cmd.z_velocity = 0
        else:
            cmd.z_mode     = ZAxisCmd.Z_VEL
            cmd.z_velocity = direction * int(Z_JOG_RPM)
        self._zaxis_pub.publish(cmd)

    def send_home(self):
        """Arm homing → ZAxisCmd.Z_HOME on /z_axis_cmd (same as joystick LB)."""
        cmd = ZAxisCmd()
        cmd.z_mode = ZAxisCmd.Z_HOME
        self._zaxis_pub.publish(cmd)
        self._last_cmd = 'Z_HOME'

    def send_gantry_limit(self, set_lower=False, set_upper=False):
        cmd = GantryLimitCmd()
        cmd.set_lower = set_lower
        cmd.set_upper = set_upper
        self._gantry_pub.publish(cmd)
        self._last_cmd = f'set {"lower" if set_lower else "upper"} limit'

    @property
    def teleop_speed(self):
        return TELEOP_SPEEDS[self.speed_idx]


# ── Curses UI ─────────────────────────────────────────────────────────────────

def run_ui(stdscr, node: HMINode):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)   # 10 Hz refresh

    # Colour pairs
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN,  curses.COLOR_BLACK)  # good
    curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)  # warning
    curses.init_pair(3, curses.COLOR_RED,    curses.COLOR_BLACK)  # fault/critical
    curses.init_pair(4, curses.COLOR_CYAN,   curses.COLOR_BLACK)  # info
    curses.init_pair(5, curses.COLOR_WHITE,  curses.COLOR_BLACK)  # normal

    GREEN  = curses.color_pair(1) | curses.A_BOLD
    YELLOW = curses.color_pair(2) | curses.A_BOLD
    RED    = curses.color_pair(3) | curses.A_BOLD
    CYAN   = curses.color_pair(4)
    NORMAL = curses.color_pair(5)

    flash_counter = 0
    last_jog_key_t = 0.0   # time of last W/S key-repeat — drives release-stop
    _a_armed_t = 0.0       # time A was last pressed — arms A+Backspace RESET combo

    while rclpy.ok():
        key = stdscr.getch()

        in_auto = node.fsm_state in AUTO_STATES

        # ── Key handling ─────────────────────────────────────────────────────
        if key == ord('q') or key == ord('Q'):
            node.send_stop()
            break

        elif key == ord('a') or key == ord('A'):
            # A arms a short RESET window (A+Backspace = RESET, mirrors joy Start+Back).
            _a_armed_t = time.time()
            node.send_start_toggle()

        elif key == ord('x') or key == ord('X'):
            node.send_estop()

        elif key in (curses.KEY_BACKSPACE, 127, 8):
            # RESET only if A was pressed within the combo window (else ignored).
            if (time.time() - _a_armed_t) < A_RESET_WINDOW_S:
                node.send_reset()
                _a_armed_t = 0.0
            else:
                node._last_cmd = 'RESET needs A+Backspace'

        elif key == ord('r') or key == ord('R'):
            node.toggle_recording()

        elif key == ord('+') or key == ord('='):
            node.speed_idx = min(node.speed_idx + 1, len(TELEOP_SPEEDS) - 1)

        elif key == ord('-'):
            node.speed_idx = max(node.speed_idx - 1, 0)

        # ── Z gantry teach controls (IDLE / SYSTEM_CHECK) ────────────────────
        elif key == ord('w') or key == ord('W'):
            node.send_z_jog(+1)          # jog up
            last_jog_key_t = time.time()

        elif key == ord('s') or key == ord('S'):
            node.send_z_jog(-1)          # jog down
            last_jog_key_t = time.time()

        elif key == ord('['):
            node.send_gantry_limit(set_lower=True)

        elif key == ord(']'):
            node.send_gantry_limit(set_upper=True)

        elif key == ord('h') or key == ord('H'):
            node.send_home()

        elif not in_auto:
            spd = node.teleop_speed
            if   key == curses.KEY_UP:    node.send_teleop( spd,  0.0)
            elif key == curses.KEY_DOWN:  node.send_teleop(-spd,  0.0)
            elif key == curses.KEY_LEFT:  node.send_teleop( 0.0,  TELEOP_ANG)
            elif key == curses.KEY_RIGHT: node.send_teleop( 0.0, -TELEOP_ANG)
            elif key == ord(' '):         node.send_teleop( 0.0,  0.0)
        else:
            if key == ord(' '):
                node.send_teleop(0.0, 0.0)

        # ── Z jog release-stop ───────────────────────────────────────────────
        # curses has no key-up event; OS key-repeat refreshes last_jog_key_t
        # while W/S is held. When repeats stop (key released), the timeout
        # fires and we send Z_STOP — same net behaviour as the joystick's
        # D-pad release. JOG_RELEASE_S must exceed the OS key-repeat interval.
        if node._z_jogging != 0 and (time.time() - last_jog_key_t) > JOG_RELEASE_S:
            node.send_z_jog(0)

        # ── Draw ─────────────────────────────────────────────────────────────
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        row = 0

        def put(r, c, text, attr=NORMAL):
            try: stdscr.addstr(r, c, text, attr)
            except curses.error: pass

        # Title
        put(row, 0, '═' * min(w-1, 60), CYAN); row += 1
        put(row, 0, '  PAINTING ROBOT — SIM HMI', CYAN); row += 1
        put(row, 0, '═' * min(w-1, 60), CYAN); row += 1

        # FSM State
        state_color = {
            'IDLE':          NORMAL,
            'WALL_ALIGN':    YELLOW,
            'WALL_FOLLOW':   GREEN,
            'TRANSIT':       YELLOW,
            'CORNER_LOGIC':  YELLOW,
            'FAULT':         RED,
        }.get(node.fsm_state, NORMAL)
        put(row, 0, f'  FSM STATE  : ', NORMAL)
        put(row, 16, f'{node.fsm_state:<20}', state_color)
        row += 1

        # Safety reason (from safety_node) + local pause view
        sr = node.safety_reason
        sr_col = RED if ('ESTOP' in sr or 'FAULT' in sr) else (
                 YELLOW if ('HOLD' in sr or node._paused) else GREEN)
        put(row, 0, f'  SAFETY     : ', NORMAL)
        put(row, 16, f'{sr}{"  [PAUSED]" if node._paused else ""}', sr_col)
        row += 1
        put(row, 0, f'  LAST CMD   : {node._last_cmd}', NORMAL)
        row += 1

        # Wall mode
        put(row, 0, f'  WALL MODE  : {node.wall_mode}', NORMAL); row += 1

        # Teleop mode indicator
        mode_str = '  [TELEOP]' if not in_auto else '  [AUTO — teleop disabled]'
        mode_col = GREEN if not in_auto else YELLOW
        put(row, 0, mode_str, mode_col); row += 1

        put(row, 0, '─' * min(w-1, 60), CYAN); row += 1

        # Right wall
        put(row, 0, '  RIGHT WALL', CYAN); row += 1
        if node.wall_pose:
            if node.wall_pose.valid:
                dist  = node.wall_pose.distance
                angle = math.degrees(node.wall_pose.angle)
                dist_col  = GREEN if abs(dist - 0.305) < 0.02 else YELLOW
                angle_col = GREEN if abs(angle) < 2.0 else YELLOW
                put(row, 0, f'    Distance  : ', NORMAL)
                put(row, 16, f'{dist:.3f} m', dist_col); row += 1
                put(row, 0, f'    Angle err : ', NORMAL)
                put(row, 16, f'{angle:.1f} deg', angle_col); row += 1
                if node.wall_status:
                    aligned = node.wall_status.aligned
                    put(row, 0, f'    Aligned   : ', NORMAL)
                    put(row, 16, 'YES' if aligned else 'NO',
                        GREEN if aligned else YELLOW); row += 1
            else:
                put(row, 0, '    OUT OF CONE — no valid wall', YELLOW); row += 1
        else:
            put(row, 0, '    No wall pose received', YELLOW); row += 1

        put(row, 0, '─' * min(w-1, 60), CYAN); row += 1

        # Forward collision
        put(row, 0, '  FORWARD COLLISION', CYAN); row += 1
        if node.collision:
            c = node.collision
            status_str = {0: 'CLEAR', 1: 'WARNING', 2: 'CRITICAL'}.get(c.status, '?')
            status_col = {0: GREEN, 1: YELLOW, 2: RED}.get(c.status, NORMAL)
            dist_col = GREEN if c.min_distance > 0.5 else (YELLOW if c.min_distance > 0.25 else RED)
            put(row, 0, f'    Distance  : ', NORMAL)
            put(row, 16, f'{c.min_distance:.3f} m', dist_col); row += 1
            put(row, 0, f'    Status    : ', NORMAL)
            put(row, 16, status_str, status_col); row += 1
        else:
            put(row, 0, '    No collision data', YELLOW); row += 1
        put(row, 0, '─' * min(w-1, 60), CYAN); row += 1

        # Front wall
        put(row, 0, '  FRONT WALL', CYAN); row += 1
        if node.front_wall_pose and node.front_wall_pose.valid:
            fdist  = node.front_wall_pose.distance
            fangle = math.degrees(node.front_wall_pose.angle)
            fdist_col  = GREEN if fdist > 0.5 else (YELLOW if fdist > 0.305 else RED)
            fangle_col = GREEN if abs(fangle) < 2.0 else YELLOW
            put(row, 0, f'    Distance  : ', NORMAL)
            put(row, 16, f'{fdist:.3f} m', fdist_col); row += 1
            put(row, 0, f'    Angle err : ', NORMAL)
            put(row, 16, f'{fangle:.1f} deg', fangle_col); row += 1
        elif node.front_wall_pose and not node.front_wall_pose.valid:
            put(row, 0, '    No clear front wall', YELLOW); row += 1
        else:
            put(row, 0, '    No front wall data', YELLOW); row += 1

        put(row, 0, '─' * min(w-1, 60), CYAN); row += 1

        # Corner (latched perception identity + execution phase)
        # Type/Depth come from the HELD fields (latched_corner_type, corner_depth)
        # which env2 keeps populated for the whole corner recipe -> no flicker.
        # Trigger = crawl_remaining (-1.0 when not approaching). Phase is derived
        # from fsm_state since execution phase is not carried in EnvStatus.
        flash_counter += 1
        put(row, 0, '  CORNER', CYAN); row += 1
        if node.env_status:
            es = node.env_status
            cs = es.corner_state

            # EnvStatus constants (fall back gracefully on an older msg build)
            NONE_T = getattr(EnvStatus, 'NONE', 255)
            INT_T  = getattr(EnvStatus, 'INTERNAL', 0)
            EXT_T  = getattr(EnvStatus, 'EXTERNAL', 1)
            BUMP_T = getattr(EnvStatus, 'BUMPOUT', 2)
            lt = getattr(es, 'latched_corner_type', NONE_T)

            type_labels = {INT_T: 'INTERNAL', EXT_T: 'EXTERNAL', BUMP_T: 'BUMPOUT'}
            type_str = type_labels.get(lt, '--')
            latched  = (lt != NONE_T)

            # State drives the colour; latched + PREEMPT is the committed signal.
            state_labels = {0: 'CLEAR', 1: 'APPROACHING', 2: 'PREEMPT', 3: 'ABORT'}
            state_colors = {0: NORMAL, 1: YELLOW, 2: RED, 3: RED}
            state_str = state_labels.get(cs, '?')
            color = state_colors.get(cs, NORMAL) if latched else NORMAL

            # Execution phase derived from FSM state (not in EnvStatus).
            phase_map = {
                'CORNER_APPROACH': 'CRAWL',
                'CORNER_LOGIC':    'TURN',
                'WALL_ALIGN':      'ALIGN',
            }
            phase_str = phase_map.get(node.fsm_state, '--') if latched else '--'

            cr = getattr(es, 'crawl_remaining', -1.0)
            trig_str = f'{cr:.3f} m' if cr >= 0.0 else '--'

            depth = getattr(es, 'corner_depth', 0.0)
            depth_str = f'{depth:.3f} m' if depth > 0.0 else '--'

            put(row, 0, '    Type      : ', NORMAL)
            put(row, 16, type_str, color if latched else NORMAL); row += 1
            put(row, 0, '    State     : ', NORMAL)
            put(row, 16, state_str, color); row += 1
            put(row, 0, '    Phase     : ', NORMAL)
            put(row, 16, phase_str, color if latched else NORMAL); row += 1
            put(row, 0, '    Trigger   : ', NORMAL)
            put(row, 16, trig_str, color if (latched and cr >= 0.0) else NORMAL); row += 1
            put(row, 0, '    Depth     : ', NORMAL)
            put(row, 16, depth_str, color if depth > 0.0 else NORMAL); row += 1
        else:
            put(row, 0, '    no data', NORMAL); row += 1

        put(row, 0, '─' * min(w-1, 60), CYAN); row += 1

        # Depth recording status
        if node._recording:
            yaw_str = f'{math.degrees(node.yaw):.1f}°' if node.yaw is not None else '—'
            rec_label = f'  [● REC]  {node._record_count} frames  yaw={yaw_str}'
            put(row, 0, rec_label, RED)
        else:
            put(row, 0, '  [  REC]  idle', NORMAL)
        row += 1

        put(row, 0, '─' * min(w-1, 60), CYAN); row += 1

        # Z gantry / teach status
        put(row, 0, '  Z GANTRY', CYAN); row += 1
        if node.head_status:
            hs = node.head_status
            homed = bool(hs.limit_lower_z)   # non-breaking: limit_lower_z carries homed
            mode  = hs.head_mode.value if hasattr(hs, 'head_mode') else 0
            mode_name = {0:'IDLE',1:'HOMING',2:'MOVING',3:'PAINTING',4:'FAULT'}.get(mode, str(mode))
            mode_col  = RED if mode == 4 else (GREEN if homed else YELLOW)
            put(row, 0, f'    Pos       : ', NORMAL)
            put(row, 16, f'{hs.z_actual_pos_mm:7.1f} mm', NORMAL); row += 1
            put(row, 0, f'    Homed     : ', NORMAL)
            put(row, 16, 'YES' if homed else 'NO', GREEN if homed else YELLOW); row += 1
            put(row, 0, f'    Mode      : ', NORMAL)
            put(row, 16, mode_name, mode_col); row += 1
            put(row, 0, f'    Taught    : ', NORMAL)
            put(row, 16, f'[{hs.z_soft_lower_mm:.0f}, {hs.z_soft_upper_mm:.0f}] mm', NORMAL); row += 1
        else:
            put(row, 0, '    (no /paint_head_status)', YELLOW); row += 1

        put(row, 0, '─' * min(w-1, 60), CYAN); row += 1

        # Teleop speed
        put(row, 0, f'  Speed      : {node.teleop_speed:.2f} m/s  '
            f'(+/- to change)', NORMAL); row += 1

        put(row, 0, '─' * min(w-1, 60), CYAN); row += 1

        # Key legend
        put(row, 0, '  KEYS', CYAN); row += 1
        put(row, 0, '  ↑↓←→ teleop  A=start/pause/resume  X=HOLD→FAULT→ESTOP  A+Bksp=reset', NORMAL); row += 1
        put(row, 0, '  H=home  W/S=jog up/dn  [=set lower  ]=set upper  R=rec', NORMAL); row += 1
        put(row, 0, '  Space=halt  Q=quit', NORMAL); row += 1

        stdscr.refresh()

        # Spin ROS callbacks
        rclpy.spin_once(node, timeout_sec=0.0)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.set_logger_level('hmi_keyboard_node', rclpy.logging.LoggingSeverity.ERROR)
    node = HMINode()
    try:
        curses.wrapper(run_ui, node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()