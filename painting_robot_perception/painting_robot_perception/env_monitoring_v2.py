#!/usr/bin/env python3
"""
env_monitoring_v2.py - clean perception/trigger node (rebuild of env_monitoring).

Design (locked with Vijeth 2026-10-04):
  CHEAP vs HEAVY split - the heavy RANSAC detector never runs during motion.

  - WALL_ALIGN (stationary, post-align): run the HEAVY detect_motion_events() on
    each fresh LiDAR scan. The first valid actionable event is LATCHED immediately
    with FROZEN geometry. This is the only place normal heavy detection runs.
  - WALL_FOLLOW (moving): nothing heavy.
  - CORNER_APPROACH (crawl, after latch): CHEAP forward-cone front-wall distance +
    odom-carried vertex -> the trigger. No RANSAC during motion. The cone reading at
    the end of the crawl (robot ~stationary) is the sanity check before turning.

All triggers are LiDAR-frame (what the sensor reads). Geometry frozen at latch -
no continuous re-latch, no per-tick angle refresh.

SCOPE (this build): detection recognises INTERNAL / EXTERNAL / BUMPOUT.
All three are executable when enabled. BUMPOUT geometry is planned by
maneuver_planner; env2 owns the maximum allowed bump depth.

EXTERNAL flow (stepped, LiDAR frame):
  LATCH  edge when vertex_fwd  in  (0, ext_latch_max] (vertex is ahead and
         close enough to trust) -> freeze vertex as an ODOM anchor.
  APPROACH  stepped: each WALL_ALIGN stop re-measures the edge and refreshes the
         odom anchor (freshest head-on read, not one blind hop).
  TRIGGER  crawl (odom-carried) until LiDAR is ext_trigger_lidar beyond the vertex;
         there run the HEAVY AperpC confirm ONCE. pass -> PREEMPT + geometry;
         fail / wall-lost -> CORNER_ABORT (state_machine -> STOP).
  PAINT cutoff fires only if the LiDAR passes (vertex + sprayhead_lidar_offset).

Fake paint: with fake_paint=True this node stands in for the (absent) paint
actuator - it publishes /stroke_complete after fake_stroke_ticks whenever the FSM
enters PAINT_EXECUTE, so the queue doesn't hang. Test-only.

Publishes:  /env_status (EnvStatus) @10Hz, /stroke_complete (Bool, fake)
Subscribes: /scan, /robot_state, /paint_bot/odom
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from painting_robot_msgs.msg import EnvStatus, MotionStatus, RobotCommand, SafetyCommand, CornerParams

from painting_robot_perception.geometry_utils import (
    WallGateCfg, detect_motion_events, detect_external_corner, LIDAR_YAW_OFFSET,
)

from painting_robot_perception.maneuver_planner import plan_maneuver

# Heavy detect + latch happen ONLY in this stationary, post-align state.
LATCH_STATE = 'WALL_ALIGN'
# After latch, the crawl runs in this state.
APPROACH_STATE = 'CORNER_APPROACH'

# -- external-corner geometry (turn-axis frame, metres) --
# AXIS_STANDOFF = distance from the turn axis to the wall with the sprayhead at
# standoff = SPRAY_STANDOFF (0.304) + SPRAY_D_ORIGIN (0.600). The external-arc
# trigger is "axis is (AXIS_STANDOFF - Re) past the vertex". Inverting that, the
# radius that lets us turn from the CURRENT vertex position is
#   Re = AXIS_STANDOFF + x_vertex,axis   (x_vertex,axis = vertex_lidar_x + lidar_offset_x)
# External-only turns use two independent YAML parameters:
#   external_turn_radius  (default 0.80 m)
#   ext_trigger_turnaxis  (default 0.104 m past the vertex)
# Keep them independently tunable so a later 0.90 / 0.004 pairing is a YAML-only
# change; env2 does not derive one from the other.
AXIS_STANDOFF = 0.904

STANDARD_ANGLES_DEG = [90.0, 120.0, 135.0, 45.0]
SNAP_TOLERANCE_DEG  = 15.0


class EnvMonitoringV2(Node):

    def __init__(self):
        super().__init__('env_monitoring_v2')

        # -- params (LiDAR frame unless noted) --------------------------------
        # -- robot geometry (shared by internal + external) --------------------
        self.declare_parameter('lidar_offset_x', 0.195)    # from common_alignment_params

        self.declare_parameter('internal_approach_dist', 1.00)  # latch when front wall <= this (m)
        # internal_trigger_turnaxis = front-wall distance from TURN AXIS at which
        # the robot must preempt. Derived from geometry, LOCKED.
        # internal_trigger_lidar = turnaxis - lidar_offset_x: same trigger in
        # LiDAR frame (what the front-cone actually measures).
        self.declare_parameter('internal_trigger_turnaxis', 0.904)
        # Largest overshoot (robot closer to the front wall than the trigger) that
        # is corrected by reversing before the turn. Larger overshoots are left
        # alone (warned) rather than reversed blindly - lidar is blind behind.
        self.declare_parameter('internal_max_backup', 0.60)
        # An INTERNAL candidate this close (odom travel, m) after a BUMPOUT was seen is
        # treated as that bump's front face and deferred until the bump latches.
        self.declare_parameter('bump_defer_internal_m', 1.50)
        # SUCKIN (shallow recess, depth <= 0.60 m) has no recipe in this build. When True it is
        # run as its first corner (EXTERNAL at V1); the INTERNAL return corner is then picked
        # up reactively, exactly like a deep (> 0.60 m) recess.
        self.declare_parameter('suckin_as_external', True)
        self.declare_parameter('front_cone_half_deg',    15.0)  # +/- arc for the cheap front-wall read
        self.declare_parameter('min_valid_range',        0.05)
        self.declare_parameter('enable_internal', True)
        self.declare_parameter('enable_external', True)
        self.declare_parameter('enable_bumpout',  False)
        self.declare_parameter('bumpout_max_depth', 1.20)  # env2 policy; YAML-overridable (sim-verified 0.7-1.2 m)
        # -- external corner (LiDAR frame, all metres) ------------------------
        # Latch window on the vertex's lateral (forward |x|) distance at the edge
        # read. LOWER bound > one strip so there's ALWAYS a real approach step
        # before the vertex (robot can never land past it in a single strip).
        # Latch when vertex is between 0 and ext_latch_max ahead (LiDAR fwd).
        # No lower bound - if vertex is close, odom-carry handles the short crawl.
        self.declare_parameter('external_latch_max', 0.30)
        # -- wall_lost external-corner recovery (back-to-back corner) ----------
        # When safety puts us in a wall_lost HOLD, env searches for an external
        # corner to recover. A wall-lost external corner can ONLY sit at/just
        # behind the turn axis (a POSITIVE vertex means there is still wall on the
        # right and wall_lost would not have fired). So the vertex-axis gate here
        # is a tight band around zero on the NEGATIVE side, not the 0.30 ahead
        # window. v_axis_x must be in [-wall_lost_axis_band, +wall_lost_axis_eps].
        self.declare_parameter('enable_wall_lost_ext_search', False)
        self.declare_parameter('wall_lost_axis_band', 0.20)   # m behind axis (max)
        self.declare_parameter('wall_lost_axis_eps',  0.05)   # m ahead axis (slack)
        # ext_trigger_turnaxis = how far PAST the vertex the robot TURN AXIS
        # (robot center) must be when the arc starts. Derived from the
        # corner_spray_external simulator for R_axis=0.8, LOCKED - do not change.
        # ext_trigger_lidar = ext_trigger_turnaxis + lidar_offset_x: the same
        # trigger expressed in LiDAR frame (what odom-carry actually tracks).
        # trigger_dist_tol = acceptance window around ext_trigger_lidar after crawl:
        #   undershoot: must be within +15mm (remaining <= 0.015)
        #   overshoot:  accept up to 50mm past trigger (remaining >= -0.05)
        #   outside window -> log debug, no AperpC check.
        self.declare_parameter('external_turn_radius', 0.80)
        self.declare_parameter('ext_trigger_turnaxis', 0.104)
        self.declare_parameter('trigger_dist_tol_under', 0.015)
        self.declare_parameter('trigger_dist_tol_over',  0.050)
        # TEMP (dev/test): skip the heavy AperpC confirm at the external trigger
        # and execute the turn purely off prior edge-detection + approach
        # geometry. Set False to restore the strict confirm-or-abort behaviour.
        self.declare_parameter('ext_skip_confirm', True)

        # -- navigation ROI (candidate relevance, NOT wall quality) ------------
        # Applied to the candidate VERTEX (code frame: 0=fwd, 270=right) at latch
        # for INTERNAL + EXTERNAL only. This is bearing+range relevance, not shape:
        # geometry_utils' WallGateCfg already decides "is this a straight wall";
        # here we decide "is this wall in the region we might turn on next".
        #   range: wall sits ~0.77m; accept -0.25/+0.25 -> [0.50, 1.00]m. Also
        #          rejects robot-body/self returns (<0.50m) and far walls (>1.0m).
        #   sector: wrapped arc start->270->end (top-right quadrant), 225->15 deg.
        #          Bearing of the vertex around the lidar, NOT the fitted-line
        #          orientation (a valid wall is still a valid wall whichever way
        #          its line runs; what matters is where it sits relative to us).
        self.declare_parameter('nav_min_range',        0.50)   # m (reject body/too-close)
        self.declare_parameter('nav_max_range',        1.00)   # m (reject far walls)
        self.declare_parameter('nav_sector_start_deg', 225.0)  # wrapped: 225 -> 270 -> 15
        self.declare_parameter('nav_sector_end_deg',    15.0)

        # External-only turn radius + trigger are owned here as YAML parameters.
        # wall_motion executes CornerParams.turn_radius supplied by env/FSM.
        # Forward offset LiDAR -> sprayhead (+ = sprayhead ahead of lidar). Sets
        # where paint must cut: sprayhead crosses the wall edge when the LiDAR is
        # (vertex + sprayhead_lidar_offset) forward. TODO(Vijeth): set the MEASURED
        # value; 0.0 leaves the cutoff inert (sprayhead co-located with lidar).
        self.declare_parameter('sprayhead_lidar_offset', 0.2)
        # fake actuator (no paint node running)
        self.declare_parameter('fake_paint',       True)
        self.declare_parameter('fake_stroke_ticks', 10)   # ~1 s @10 Hz

        g = self.get_parameter
        self.lidar_offset_x         = g('lidar_offset_x').value
        self.internal_approach = g('internal_approach_dist').value
        self.internal_trigger_turnaxis = g('internal_trigger_turnaxis').value
        self.internal_trigger  = self.internal_trigger_turnaxis - self.lidar_offset_x
        self.internal_max_backup = g('internal_max_backup').value
        self.bump_defer_internal = g('bump_defer_internal_m').value
        self.suckin_as_external = g('suckin_as_external').value
        self.cone_half         = g('front_cone_half_deg').value
        self.min_valid         = g('min_valid_range').value
        self.enable_internal   = g('enable_internal').value
        self.enable_external   = g('enable_external').value
        self.enable_bumpout    = g('enable_bumpout').value
        self.bumpout_max_depth = float(g('bumpout_max_depth').value)
        self.ext_latch_max     = g('external_latch_max').value
        self.enable_wall_lost_ext_search = g('enable_wall_lost_ext_search').value
        self.wall_lost_axis_band = g('wall_lost_axis_band').value
        self.wall_lost_axis_eps  = g('wall_lost_axis_eps').value
        self.external_turn_radius   = float(g('external_turn_radius').value)
        self.ext_trigger_turnaxis   = float(g('ext_trigger_turnaxis').value)
        self.ext_trigger_lidar      = self.ext_trigger_turnaxis + self.lidar_offset_x
        self.trigger_dist_tol_under = g('trigger_dist_tol_under').value
        self.trigger_dist_tol_over  = g('trigger_dist_tol_over').value
        self.ext_skip_confirm       = g('ext_skip_confirm').value

        # navigation ROI (candidate relevance)
        self.nav_min_range    = g('nav_min_range').value
        self.nav_max_range    = g('nav_max_range').value
        self.nav_sector_start = g('nav_sector_start_deg').value
        self.nav_sector_end   = g('nav_sector_end_deg').value

        self.sprayhead_lidar_off = g('sprayhead_lidar_offset').value
        self.fake_paint        = g('fake_paint').value
        self.fake_stroke_ticks = int(g('fake_stroke_ticks').value)

        self.gate = WallGateCfg()

        # -- state (minimal) --------------------------------------------------
        self._scan = None
        self._last_stamp = None                 # fresh-frame gate for heavy detect
        self._robot_state = 'IDLE'
        self._prev_state = 'IDLE'
        self._odom_x = self._odom_y = self._odom_yaw = 0.0

        self._latched = False
        self._latched_type = None
        self._latched_event = None              # frozen geometry at latch
        # odom-carry of the front-wall distance (set at latch, decremented by travel)
        self._latch_odom = None                 # (x, y) at latch
        self._latch_yaw  = 0.0                  # heading AT LATCH (project travel on this, not live yaw)
        self._latch_front = 0.0                 # front-wall LiDAR dist at latch
        self._cur_front = 0.0                   # live estimate for publishing
        self._in_recipe = False                 # True once we've entered CORNER_LOGIC this corner
        self._post_turn_align_seen = False       # recipe's own post-turn ALIGN has run
        self._cone_tol = 0.15                   # cone must agree with odom within this to override (m)

        # -- internal-corner odom crawl ----------------------------------------
        self._int_front_at_latch = None         # front wall from RANSAC fits at commit
        self._int_crawl_anchor = None           # (x, y) odom at internal latch
        self._int_crawl_total = None            # total crawl = front_at_latch - internal_trigger

        # -- bump-out frozen plan / odom trigger -------------------------------
        self._bump_v1_odom = None               # V1 fixed in odom at latch
        self._bump_trigger_axis = None           # axis distance BEFORE V1 from planner
        self._bump_depth = None
        self._bump_params_published = False
        # last odom position where a BUMPOUT was seen (even outside the nav ROI);
        # used to stop a pillar/box front face being latched as a room corner.
        self._bump_seen_odom = None

        # -- external-corner anchor (odom frame) -------------------------------
        # Vertex in ODOM at the latest edge re-anchor. The stepped approach
        # re-measures the edge at every stationary stop and refreshes this, so
        # the trigger distance is always computed from the freshest head-on read,
        # NOT a single blind hop off the first (shallow-angle) latch.
        self._ext_vertex_odom = None            # (x, y) in odom of the vertex
        self._ext_abort = False                 # AperpC confirm failed / wall lost -> STOP
        self._ext_confirmed = False             # AperpC passed - latch PREEMPT, never re-evaluate
        self._sprayhead_past = False            # sprayhead crossed the wall edge -> paint must be off
        # reactive external radius + per-latch trigger (chosen from current vertex geometry)
        self._ext_Re = None                     # Re chosen at external latch
        self._ext_trigger_turnaxis_latched = None
        self._ext_trigger_lidar_latched = None
        self._wall_lost_hold = False            # True while in a safety wall_lost HOLD

        # step_complete gate - AperpC confirm only fires after crawl step_complete
        self._crawl_cmd_id = None              # command_id of the CORNER_APPROACH crawl command
        self._step_complete = False            # rising-edge latched from /motion_status
        self._prev_step_complete = False       # for rising-edge detection

        # fake paint bookkeeping
        self._paint_ticks = 0
        self._paint_fired = False

        # -- I/O --------------------------------------------------------------
        self.create_subscription(LaserScan,    '/scan',            self._scan_cb,          10)
        self.create_subscription(String,       '/robot_state',     self._state_cb,         10)
        self.create_subscription(Odometry,     '/paint_bot/odom',  self._odom_cb,          10)
        self.create_subscription(RobotCommand, '/robot_command',   self._robot_command_cb, 10)
        self.create_subscription(MotionStatus, '/motion_status',   self._motion_status_cb, 10)
        # Safety tier/reason - so env knows when it is in a wall_lost HOLD and can
        # search for an external corner to recover from (back-to-back-corner case).
        self.create_subscription(SafetyCommand, '/safety_command',  self._safety_cmd_cb,    10)
        self._env_pub    = self.create_publisher(EnvStatus, '/env_status', 10)
        self._corner_params_pub = self.create_publisher(CornerParams, '/corner_params', 10)
        self._stroke_pub = self.create_publisher(Bool, '/stroke_complete', 10)

        # Keep INTERNAL / EXTERNAL fully safe with an older EnvStatus.msg.
        # BUMPOUT execution is armed only after EnvStatus.msg has a BUMPOUT constant.
        self._env_bumpout_type = getattr(EnvStatus, 'BUMPOUT', None)
        if self.enable_bumpout and self._env_bumpout_type is None:
            self.get_logger().warn(
                '[ENV] enable_bumpout=True but EnvStatus.msg has no BUMPOUT constant; '
                'BUMPOUT will be detected/logged but NOT latched until the message is updated/rebuilt.')

        self.create_timer(0.1, self._process)   # 10 Hz
        self.get_logger().info(
            f'env_monitoring_v2 - lidar_offset_x={self.lidar_offset_x}m '
            f'int_trigger={self.internal_trigger:.3f}m (LiDAR, turnaxis={self.internal_trigger_turnaxis}) '
            f'ext_R={self.external_turn_radius:.3f}m '
            f'ext_trigger={self.ext_trigger_lidar:.3f}m (LiDAR, turnaxis={self.ext_trigger_turnaxis}) | '
            f'heavy detect in {LATCH_STATE} only, fake_paint={self.fake_paint} | '
            f'enable_internal={self.enable_internal} enable_external={self.enable_external} '
            f'enable_bumpout={self.enable_bumpout} bumpout_max_depth={self.bumpout_max_depth:.3f}m')

    # -- callbacks -------------------------------------------------------------
    def _scan_cb(self, msg): self._scan = msg
    def _odom_cb(self, msg):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._odom_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                                    1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def _safety_cmd_cb(self, msg):
        """Track whether we are currently in a wall_lost SAFETY_HOLD. Only that
        combination arms the external-corner recovery search - not operator pause,
        not an obstacle hold, not FAULT/ESTOP. reason is a free string from safety
        (e.g. 'safety:wall_lost' / 'wall_lost'); match on the suffix."""
        try:
            is_hold = (msg.tier == SafetyCommand.SAFETY_HOLD)
        except AttributeError:
            is_hold = (msg.tier == 1)  # SAFETY_HOLD ordinal fallback
        reason = (msg.reason or '').strip().lower()
        new_val = bool(is_hold and reason.endswith('wall_lost'))
        if new_val != self._wall_lost_hold:
            self.get_logger().info(
                f'[ENV] wall_lost_hold {self._wall_lost_hold}->{new_val} '
                f'(tier={msg.tier} reason="{reason}")')
        self._wall_lost_hold = new_val

    def _robot_command_cb(self, msg):
        """Latch the command_id when FSM issues a CORNER_APPROACH crawl command."""
        if msg.state == APPROACH_STATE:
            self._crawl_cmd_id = msg.command_id
            self._step_complete = False
            self._prev_step_complete = False

    def _motion_status_cb(self, msg):
        """Rising-edge latch step_complete, gated on the CORNER_APPROACH command_id."""
        if self._crawl_cmd_id is None or msg.command_id != self._crawl_cmd_id:
            return
        if msg.step_complete and not self._prev_step_complete:
            self._step_complete = True
        self._prev_step_complete = msg.step_complete

    def _state_cb(self, msg):
        self._prev_state = self._robot_state
        self._robot_state = msg.data

        if msg.data in ('IDLE', 'FAULT'):
            self._reset_latch()

        # Entering HOLD after an INTERNAL turn: the internal latch is spent (the
        # turn executed), but _process() returns early while _latched is True, so
        # the wall_lost external search would never run. Retire ONLY a spent
        # INTERNAL latch here so the wall-lost search can run. Leave an EXTERNAL
        # latch alone (a hold mid-external-approach must resume that approach).
        if (msg.data == 'HOLD' and self._latched
                and self._latched_type == 'INTERNAL'):
            self.get_logger().info(
                '[ENV] HOLD after INTERNAL turn - retiring spent internal latch, '
                'arming wall_lost external search')
            self._latched = False
            self._latched_type = None
            self._latched_event = None
            self._int_front_at_latch = None
            self._int_crawl_anchor = None
            self._int_crawl_total = None
            self._step_complete = False
            self._prev_step_complete = False

        # Recipe-completion tracking (internal recipe = CRAWL -> TURN -> ALIGN -> ...).
        # Stage 1: we entered the turn. LATCH_STATE is 'WALL_ALIGN', so the test
        # below (msg.data == LATCH_STATE) fires on the recipe's post-turn WALL_ALIGN.
        # There are NO WALL_ALIGN states DURING a segmented turn - the inter-segment
        # dips are PAINT_EXECUTE - so the first WALL_ALIGN seen while _in_recipe is
        # always the post-turn align, after all turn segments have completed.
        if msg.data == 'CORNER_LOGIC':
            self._in_recipe = True
            self._post_turn_align_seen = False
        # Stage 2: the recipe's OWN post-turn WALL_ALIGN (= LATCH_STATE) is running.
        # The turn is done and the old corner geometry is spent, so RETIRE the latch
        # and reopen detection RIGHT HERE. This lets a nearby next external edge latch
        # during this same post-turn align (back-to-back corners) instead of only
        # after a wasted normal WALL_FOLLOW strip. Safe for segmented turns because
        # no WALL_ALIGN occurs between segments (see Stage 1 note).
        if msg.data == LATCH_STATE and self._in_recipe:
            self._post_turn_align_seen = True
            if self._latched:
                self.get_logger().info(
                    '[ENV] post-turn WALL_ALIGN - corner recipe complete, '
                    'retiring spent latch + reopening detection')
                self._reset_latch()

        # Arm a fresh fake stroke each PAINT_EXECUTE entry.
        if msg.data == 'PAINT_EXECUTE' and self._prev_state != 'PAINT_EXECUTE':
            self._paint_ticks = self.fake_stroke_ticks
            self._paint_fired = False

    def _reset_latch(self):
        self._latched = False
        self._latched_type = None
        self._latched_event = None
        self._latch_odom = None
        self._latch_front = 0.0
        self._in_recipe = False
        self._post_turn_align_seen = False
        # internal crawl
        self._int_front_at_latch = None
        self._int_crawl_anchor = None
        self._int_crawl_total = None
        # bumpout
        self._bump_v1_odom = None
        self._bump_trigger_axis = None
        self._bump_depth = None
        self._bump_params_published = False
        self._bump_seen_odom = None
        # external
        self._ext_vertex_odom = None
        self._ext_abort = False
        self._ext_confirmed = False
        self._sprayhead_past = False
        self._ext_Re = None
        self._ext_trigger_turnaxis_latched = None
        self._ext_trigger_lidar_latched = None
        self._crawl_cmd_id = None
        self._step_complete = False
        self._prev_step_complete = False

    # -- cheap front-wall distance (forward cone median, LiDAR frame) -----------
    def _front_cone_dist(self):
        s = self._scan
        if s is None:
            return 0.0
        r = np.asarray(s.ranges, dtype=np.float32)
        ang = (np.degrees(s.angle_min + np.arange(len(r)) * s.angle_increment
                          + LIDAR_YAW_OFFSET)) % 360.0
        m = ((ang <= self.cone_half) | (ang >= 360.0 - self.cone_half)) \
            & np.isfinite(r) & (r > self.min_valid)
        v = r[m]
        return float(np.median(v)) if v.size >= 3 else 0.0

    # -- front wall of an INTERNAL event = fit whose normal points most forward --
    @staticmethod
    def _event_front_dist(ev, dbg):
        gated = dbg.get("gated_fits", [])
        best_d, best_nx = 0.0, -1.0
        for i in (ev.get("fit_before"), ev.get("fit_after")):
            if i is None or i >= len(gated):
                continue
            fit = gated[i][0] if isinstance(gated[i], tuple) else gated[i]
            n = fit.get("normal")
            if n is None:
                continue
            nx = abs(float(n[0]))
            if nx > best_nx:
                best_nx, best_d = nx, float(fit.get("distance", 0.0))
        return best_d

    def _snap_angle(self, a):
        for s in STANDARD_ANGLES_DEG:
            if abs(a - s) <= SNAP_TOLERANCE_DEG:
                return s
        return a

    # -- navigation ROI gate (candidate relevance, applied at latch) -------------
    def _vertex_in_nav_roi(self, V):
        """Relevance gate on the candidate VERTEX bearing+range.

        The vertex is already in CODE FRAME (0=fwd, 270=right) because the
        geometry detectors apply LIDAR_YAW_OFFSET. This filters on WHERE the
        candidate sits around the lidar (bearing) and HOW FAR (range) - it does
        NOT look at the fitted-line orientation. A valid wall stays valid
        whichever way its line runs; this only decides whether it is in the
        region we could act on for the next turn.

        Accepts the wrapped sector nav_sector_start -> 270 -> nav_sector_end
        (top-right quadrant) and range in [nav_min_range, nav_max_range].
        Returns (ok: bool, reason: str).
        """
        vx, vy = float(V[0]), float(V[1])
        rng = math.hypot(vx, vy)
        if rng < self.nav_min_range or rng > self.nav_max_range:
            return False, (f"range {rng:.2f}m outside "
                           f"[{self.nav_min_range:.2f},{self.nav_max_range:.2f}]")
        brg = math.degrees(math.atan2(vy, vx)) % 360.0        # code frame, no offset
        s, e = self.nav_sector_start, self.nav_sector_end
        inside = (brg >= s or brg <= e) if s > e else (s <= brg <= e)   # wrapped
        if not inside:
            return False, f"bearing {brg:.0f}deg outside sector [{s:.0f}->{e:.0f}]"
        return True, "ok"

    # -- main loop ---------------------------------------------------------------
    def _process(self):
        self._fake_paint_tick()
        if self._latched:
            self._publish(self._trigger())
            return

        # DETECT + LATCH - heavy, stationary, WALL_ALIGN only. No temporal voting:
        # every fresh scan is an independent opportunity to latch the first valid
        # actionable event. Corner-recipe aligns stay blocked.
        if (self._robot_state == LATCH_STATE
                and not self._in_recipe
                and (self.enable_internal or self.enable_external or self.enable_bumpout)):
            self._detect_and_latch()

        # Wall-lost recovery: if safety has us in a wall_lost HOLD and we are not
        # already latched onto something, search for an external corner right at
        # the axis (back-to-back-corner case). FSM state-agnostic by design.
        if (not self._latched and self._wall_lost_hold
                and self.enable_external and self.enable_wall_lost_ext_search):
            self._search_external_on_wall_lost()

        self._publish(EnvStatus.CORNER_CLEAR if not self._latched
                      else self._trigger())

    # -- external-corner search while held on wall_lost --------------------------
    def _search_external_on_wall_lost(self):
        """Run the AperpC confirm from wherever we are. A wall-lost external corner
        can only sit at/just behind the turn axis, so gate the vertex-axis x to a
        tight negative band (a positive vertex means wall still on the right ->
        wall_lost would not have fired). In band -> latch external (reactive Re) and
        start signalling CORNER_APPROACHING/PREEMPT so the FSM can auto-resume."""
        if self._scan is None:
            self.get_logger().warn('[ENV] wall_lost ext search - no scan yet',
                                    throttle_duration_sec=2.0)
            return
        ext = detect_external_corner(self._scan_dict(), self.gate)
        if ext is None:
            self.get_logger().info(
                '[ENV] wall_lost ext search - detect_external_corner returned None '
                '(no AperpC: missing C>=0.15 / not convex / A not behind)',
                throttle_duration_sec=2.0)
            return  # no real AperpC corner (doorway / open space self-reject on C>=0.15)
        V = np.asarray(ext["vertex"], float)
        v_axis_x = float(V[0]) + self.lidar_offset_x
        if not (-self.wall_lost_axis_band <= v_axis_x <= self.wall_lost_axis_eps):
            self.get_logger().info(
                f'[ENV] wall_lost ext search - corner found but v_axis_x={v_axis_x:+.3f}m '
                f'outside band [{-self.wall_lost_axis_band:.2f},{self.wall_lost_axis_eps:.2f}] - ignoring',
                throttle_duration_sec=2.0)
            return
        self.get_logger().info(
            f'[ENV] wall_lost ext search - CORNER in band (v_axis_x={v_axis_x:+.3f}m, '
            f'C_chord={ext.get("C_chord", float("nan")):.2f}) -> latching external')
        # The internal recipe that left us wall-lost is OVER - clear _in_recipe so
        # _publish stamps this fresh external latch with its real corner_type
        # (EnvStatus.EXTERNAL). Otherwise _publish's "not _in_recipe" guard makes it
        # fall through and publish corner_type=INTERNAL, and the FSM's external
        # auto-resume guard (is_external) never fires.
        self._in_recipe = False
        self._post_turn_align_seen = False
        self._latch_external(ext)

    # -- heavy detect + immediate latch (WALL_ALIGN only) -------------------------
    def _detect_and_latch(self):
        if self._scan is None:
            return

        # Fresh-frame gate: process each LiDAR scan at most once.
        stamp = (self._scan.header.stamp.sec, self._scan.header.stamp.nanosec)
        if stamp == self._last_stamp:
            return
        self._last_stamp = stamp

        events, dbg = detect_motion_events(self._scan_dict(), self.gate)

        top = events[0] if events else None
        ttype = None if top is None else \
            ('EXTERNAL' if top["type"] == 'EXTERNAL_EDGE' else top["type"])
        vtx_str = f'vertex={top["vertex"]}' if top is not None else 'no-event'
        self.get_logger().info(
            f'[ENV] detect scan: detected={ttype} {vtx_str}')

        if top is None:
            return

        if ttype == 'BUMPOUT':
            self._bump_seen_odom = (self._odom_x, self._odom_y)

        # Front-fit indices belong to this scan's dbg, so compute the internal
        # front-wall distance immediately from the same detector result.
        front = self._event_front_dist(top, dbg)

        if ttype == 'INTERNAL' and self.enable_internal:
            self._latch_internal(top, front)
        elif ttype == 'EXTERNAL' and self.enable_external:
            self._latch_external(top)
        elif ttype == 'BUMPOUT' and self.enable_bumpout:
            self._latch_bumpout(top)
        elif ttype == 'SUCKIN' and self.suckin_as_external and self.enable_external:
            v1 = top.get("external_vertex", top["vertex"])
            self.get_logger().info(
                f'[ENV] SUCKIN depth={top.get("depth", 0.0):.2f}m has no recipe - '
                f'running it as EXTERNAL at V1=({v1[0]:.2f},{v1[1]:.2f})',
                throttle_duration_sec=2.0)
            self._latch_external({"vertex": np.asarray(v1, float)})
        else:
            self.get_logger().info(
                f'[ENV] {ttype} detected but execution disabled this build - ignoring',
                throttle_duration_sec=2.0)

    def _latch_internal(self, ev, front):
        # nav ROI: candidate vertex must be in the top-right sector and in range.
        ok, why = self._vertex_in_nav_roi(ev["vertex"])
        if not ok:
            self.get_logger().info(
                f'[ENV] INTERNAL vertex outside nav ROI - {why}, ignoring',
                throttle_duration_sec=2.0)
            return
        if front <= 0.0 or front > self.internal_approach:
            return
        if self._bump_seen_odom is not None and self.bump_defer_internal > 0.0:
            trav = math.hypot(self._odom_x - self._bump_seen_odom[0],
                              self._odom_y - self._bump_seen_odom[1])
            if trav <= self.bump_defer_internal:
                self.get_logger().info(
                    f'[ENV] INTERNAL deferred - BUMPOUT seen {trav:.2f}m ago, '
                    f'front face is probably that bump (waiting for it to latch)',
                    throttle_duration_sec=1.0)
                return
        angle = self._snap_angle(float(ev.get("angle_deg", 90.0)))
        rad   = math.copysign(math.radians(angle), ev.get("radian_diff", 1.0))
        self._latched = True
        self._latched_type = 'INTERNAL'
        self._latched_event = {
            "angle_deg": angle, "radian_diff": rad,
            "turn_dir": 'CCW' if rad > 0 else 'CW',
            "turn_radius": 0.0,                       # internal = in-place pivot
            "vertex": np.asarray(ev["vertex"], float),
        }
        self._latch_odom  = (self._odom_x, self._odom_y)
        self._latch_yaw   = self._odom_yaw        # freeze heading for travel projection
        self._latch_front = front
        self._cur_front   = front
        # odom-tracked crawl: latch front wall from heavy detect (RANSAC fits)
        self._int_front_at_latch = front
        self._int_crawl_anchor = (self._odom_x, self._odom_y)
        self._int_crawl_total = front - self.internal_trigger
        self.get_logger().info(
            f'[ENV] INTERNAL LATCH - front_wall={front:.3f}m (LiDAR) angle={angle:.0f}deg '
            f'crawl_total={self._int_crawl_total:.3f}m '
            f'turn={self._latched_event["turn_dir"]} '
            f'vertex=({ev["vertex"][0]:.2f},{ev["vertex"][1]:.2f})')


    # -- bump-out latch ----------------------------------------------------------
    def _latch_bumpout(self, ev):
        """Freeze a complete BUMPOUT before motion, ask maneuver_planner for the
        free two-arc solution, and carry V1 in odom to the planner trigger.

        INTERNAL / EXTERNAL paths are untouched by this function.
        """
        if self._env_bumpout_type is None:
            self.get_logger().warn(
                '[ENV] BUMPOUT detected but EnvStatus.BUMPOUT is unavailable - ignoring safely',
                throttle_duration_sec=2.0)
            return

        depth = float(ev.get('depth', 0.0))
        if depth <= 0.0:
            self.get_logger().warn('[ENV] BUMPOUT has invalid/missing depth - ignoring')
            return

        if depth > self.bumpout_max_depth:
            self.get_logger().info(
                f'[ENV] BUMPOUT depth={depth:.3f}m > bumpout_max_depth='
                f'{self.bumpout_max_depth:.3f}m - not executing as compound bump')
            return

        V1 = np.asarray(ev.get('internal_vertex', ev.get('vertex')), float)
        V2 = np.asarray(ev.get('external_vertex', ev.get('vertex')), float)

        # V1 is the first/internal corner and is the trigger reference.
        ok, why = self._vertex_in_nav_roi(V1)
        if not ok:
            self.get_logger().info(
                f'[ENV] BUMPOUT V1 outside nav ROI - {why}, ignoring',
                throttle_duration_sec=2.0)
            return

        plan = plan_maneuver('BUMP', depth=depth)
        if not plan.feasible or not plan.steps:
            self.get_logger().warn(
                f'[ENV] BUMPOUT planner rejected depth={depth:.3f}m - {plan.reason}')
            return

        step = plan.steps[0]
        if step.R1 is None or step.R2 is None or step.phi is None:
            self.get_logger().warn('[ENV] BUMPOUT planner returned incomplete R1/R2/phi - ignoring')
            return

        self._latched = True
        self._latched_type = 'BUMPOUT'
        self._latched_event = {
            'angle_deg': math.degrees(step.phi),
            'radian_diff': float(step.phi),       # ARC1 = OUT / CCW
            'turn_dir': 'CCW',
            'turn_radius': float(step.R1),        # ARC1 tip radius
            'arc2_radian_diff': -float(step.phi),
            'arc2_turn_radius': float(step.R2),
            'vertex': V1,
            'internal_vertex': V1,
            'external_vertex': V2,
            'depth': depth,
        }

        self._latch_odom = (self._odom_x, self._odom_y)
        self._latch_yaw = self._odom_yaw
        self._bump_v1_odom = self._lidar_to_odom(V1[0], V1[1])
        self._bump_trigger_axis = float(step.trigger_axis)
        self._bump_depth = depth

        self._publish_bump_corner_params()

        rem = self._bump_remaining()
        self.get_logger().info(
            f'[ENV] BUMPOUT LATCH - depth={depth:.3f}m '
            f'R1={step.R1:.3f} R2={step.R2:.3f} '
            f'phi={math.degrees(step.phi):.1f}deg '
            f'trigger_axis={step.trigger_axis:.3f}m '
            f'crawl_remaining={0.0 if rem is None else rem:.3f}m '
            f'V1=({V1[0]:.2f},{V1[1]:.2f}) V2=({V2[0]:.2f},{V2[1]:.2f})')

    def _bump_remaining(self):
        """Axis forward distance still to travel before ARC1 may start.

        planner trigger_axis is a POSITIVE distance BEFORE V1.
        Current axis->V1 comes from the frozen V1 odom anchor.
        """
        if self._bump_v1_odom is None or self._bump_trigger_axis is None:
            return None
        dx = self._bump_v1_odom[0] - self._odom_x
        dy = self._bump_v1_odom[1] - self._odom_y
        axis_to_v1 = dx * math.cos(self._latch_yaw) + dy * math.sin(self._latch_yaw)
        return axis_to_v1 - self._bump_trigger_axis

    def _publish_bump_corner_params(self):
        """Publish the frozen compound geometry directly for wall_motion.

        State-machine INTERNAL / EXTERNAL CornerParams publication is left unchanged.
        """
        if self._bump_params_published or self._latched_type != 'BUMPOUT':
            return
        e = self._latched_event
        if not e:
            return

        p = CornerParams()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'base_link'
        p.corner_type = 'BUMPOUT'
        p.radian_diff = float(e['radian_diff'])
        p.angle_deg = float(e['angle_deg'])
        p.distance = 0.0
        p.turn_radius = float(e['turn_radius'])
        p.arc2_radian_diff = float(e['arc2_radian_diff'])
        p.arc2_turn_radius = float(e['arc2_turn_radius'])
        self._corner_params_pub.publish(p)
        self._bump_params_published = True

    # -- external latch + stepped re-anchor --------------------------------------
    def _lidar_to_odom(self, vx, vy):
        """Map a LiDAR-frame point (forward +x, left +y) to odom (x,y) using the
        CURRENT pose. Used to store the vertex as a fixed odom anchor so travel
        shrinks the remaining distance regardless of heading changes."""
        c, s = math.cos(self._odom_yaw), math.sin(self._odom_yaw)
        return (self._odom_x + vx * c - vy * s,
                self._odom_y + vx * s + vy * c)

    def _latch_external(self, ev):
        V = np.asarray(ev["vertex"], float)
        # nav ROI: candidate vertex must be in the top-right sector and in range.
        ok, why = self._vertex_in_nav_roi(V)
        if not ok:
            self.get_logger().info(
                f'[ENV] EXTERNAL vertex outside nav ROI - {why}, ignoring',
                throttle_duration_sec=2.0)
            return
        fwd = float(V[0])                          # forward (along-wall) distance to vertex
        # Two latch regimes, split on where the vertex sits in LiDAR frame:
        #   fwd  in  (0, ext_latch_max]  -> EDGE-PROMOTION: vertex is ahead, there is a
        #       real approach step before it. Normal crawl-to-trigger path.
        #   fwd <= 0                  -> VERTEX-BEHIND (e.g. landed AT the corner right
        #       after an internal turn, or a genuine back-to-back external corner):
        #       no approach step is possible - the vertex is level with / behind the
        #       LiDAR. We still latch and anchor the vertex in odom; _ext_remaining
        #       then returns a small/negative value so _trigger_external does a short
        #       crawl (or fires at once) and runs the SAME AperpC confirm before turning.
        #       The confirm (needs a real C >= 0.15 wall) is the false-turn guard, so
        #       this path cannot turn into a doorway/open space.
        # fwd > ext_latch_max -> vertex too far to trust the edge yet; wait.
        if fwd > self.ext_latch_max:
            self.get_logger().info(
                f'[ENV] EXTERNAL committed but vertex_fwd={fwd:.3f}m > max {self.ext_latch_max:.2f}m - too far, wait')
            return
        behind = fwd <= 0.0

        # -- external-only radius + trigger (independent YAML parameters) -------
        # Intentionally NOT derived from current vertex geometry. This keeps the
        # validated pair easy to tune, e.g. 0.80 / 0.104 now or 0.90 / 0.004 later.
        v_axis_x = fwd + self.lidar_offset_x
        Re = self.external_turn_radius
        ext_trigger_turnaxis = self.ext_trigger_turnaxis
        ext_trigger_lidar = ext_trigger_turnaxis + self.lidar_offset_x

        self._latched = True
        self._latched_type = 'EXTERNAL'
        self._latched_event = {
            "angle_deg": 90.0, "radian_diff": -math.pi / 2,
            "turn_dir": "CW",
            "turn_radius": Re,    # YAML external-only radius; wall_motion executes THIS
            "vertex": V,
        }
        self._latch_odom = (self._odom_x, self._odom_y)
        self._latch_yaw  = self._odom_yaw
        self._ext_vertex_odom = self._lidar_to_odom(V[0], V[1])
        self._ext_Re = Re
        self._ext_trigger_turnaxis_latched = ext_trigger_turnaxis
        self._ext_trigger_lidar_latched = ext_trigger_lidar
        self._ext_abort = False
        self._ext_confirmed = False
        self._sprayhead_past = False
        self.get_logger().info(
            f'[ENV] EXTERNAL LATCH ({"VERTEX-BEHIND / turn-now" if behind else "edge-promotion"}) - '
            f'vertex_fwd={fwd:.3f}m (LiDAR) v_axis_x={v_axis_x:+.3f}m '
            f'Re={Re:.3f} trig_turnaxis={ext_trigger_turnaxis:.3f} '
            f'trig_lidar={ext_trigger_lidar:.3f} '
            f'odom_anchor=({self._ext_vertex_odom[0]:.2f},{self._ext_vertex_odom[1]:.2f})')



    def _ext_remaining(self):
        """Forward distance (LiDAR +x) still to travel to reach the trigger point
        (vertex + ext_trigger_lidar), carried on odom from the current anchor.
        ext_trigger_lidar = ext_trigger_turnaxis + lidar_offset_x, so this gives
        how far the LiDAR still needs to travel past the vertex."""
        if self._ext_vertex_odom is None:
            return None
        dx = self._ext_vertex_odom[0] - self._odom_x
        dy = self._ext_vertex_odom[1] - self._odom_y
        # project the odom gap onto the LATCH heading = forward distance to vertex
        to_vertex = dx * math.cos(self._latch_yaw) + dy * math.sin(self._latch_yaw)
        # Use the per-latch copy of the YAML trigger; fall back to the node param.
        trig = self._ext_trigger_lidar_latched
        if trig is None:
            trig = self.ext_trigger_lidar
        return to_vertex + trig                        # trigger in LiDAR frame past vertex

    def _int_remaining(self):
        """Odom-tracked distance still to crawl for internal corner approach.
        Mirrors _ext_remaining but simpler: no vertex anchor, just forward travel
        from the latch point subtracted from the total crawl distance."""
        if self._int_crawl_anchor is None:
            return None
        dx = self._int_crawl_anchor[0] - self._odom_x
        dy = self._int_crawl_anchor[1] - self._odom_y
        # project onto LATCH heading - fwd travel makes this negative
        fwd = dx * math.cos(self._latch_yaw) + dy * math.sin(self._latch_yaw)
        return self._int_crawl_total + fwd   # fwd < 0 as robot moves forward -> shrinks

    # -- trigger (after latch) - CHEAP cone + odom-carry, no RANSAC --------------
    def _trigger(self):
        # Once the recipe is actually executing the turn (CORNER_LOGIC seen), the
        # preempt has done its job. Keep asserting PREEMPT past this point and
        # state_machine re-latches corner_preempt=True, which then makes the NEXT
        # normal WALL_FOLLOW instantly ITEM_COMPLETE (WallFollowHandler treats
        # corner_preempt as a reason to end the strip) - the robot never advances.
        # So go quiet the moment the turn begins. CORNER_CLEAR (not APPROACHING):
        # state_machine.env_status_cb only re-splices on an actionable corner, and
        # since _corner_active is already cleared on leaving CORNER_LOGIC, an
        # APPROACHING+INTERNAL frame here passes its splice guard and re-latches a
        # SECOND internal recipe mid-run. CLEAR is the only inert signal.
        if self._in_recipe:
            return EnvStatus.CORNER_CLEAR

        if self._latched_type == 'EXTERNAL':
            return self._trigger_external()

        if self._latched_type == 'BUMPOUT':
            return self._trigger_bumpout()

        if self._latched_type != 'INTERNAL':
            return EnvStatus.CORNER_APPROACHING

        # -- odom-tracked crawl (primary) + front-cone sanity ------------------
        remaining = self._int_remaining()
        if remaining is None:
            return EnvStatus.CORNER_APPROACHING

        # keep _cur_front updated for EnvStatus publishing (odom-based estimate)
        self._cur_front = self.internal_trigger + remaining

        # still travelling - keep crawling
        if remaining > self.trigger_dist_tol_under:
            return EnvStatus.CORNER_APPROACHING

        # odom says we're there - wait for step_complete (robot stopped)
        if not self._step_complete:
            return EnvStatus.CORNER_APPROACHING

        # sanity: front-cone should read ~= internal_trigger +/- tolerance
        front_live = self._front_cone_dist()
        if front_live > 0.0 and abs(front_live - self.internal_trigger) > 0.05:
            self.get_logger().warn(
                f'[ENV] INTERNAL sanity - front_cone={front_live:.3f}m vs '
                f'expected {self.internal_trigger:.3f}m (odom remaining={remaining:.3f}m)')

        self.get_logger().info(
            f'[ENV] INTERNAL PREEMPT - odom remaining={remaining:.3f}m '
            f'front_cone={front_live:.3f}m trigger={self.internal_trigger:.3f}m (LiDAR)')
        return EnvStatus.CORNER_PREEMPT

    def _trigger_bumpout(self):
        """Pure odom trigger for the frozen compound bump plan."""
        remaining = self._bump_remaining()
        if remaining is None:
            return EnvStatus.CORNER_APPROACHING

        if remaining > self.trigger_dist_tol_under:
            return EnvStatus.CORNER_APPROACHING

        # Wait until wall_motion reports the CORNER_APPROACH crawl complete.
        if not self._step_complete:
            return EnvStatus.CORNER_APPROACHING

        if remaining < -self.trigger_dist_tol_over:
            self.get_logger().warn(
                f'[ENV] BUMPOUT crawl overshot trigger by '
                f'{-remaining - self.trigger_dist_tol_over:.3f}m - preempting anyway '
                f'(geometry already frozen)')
        self.get_logger().info(
            f'[ENV] BUMPOUT PREEMPT - odom remaining={remaining:.3f}m '
            f'trigger_axis={self._bump_trigger_axis:.3f}m '
            f'depth={self._bump_depth:.3f}m')
        return EnvStatus.CORNER_PREEMPT

    def _trigger_external(self):
        """External approach -> trigger. Crawl (odom-carried) until the LiDAR is
        past the vertex by past_vertex; at that point run the HEAVY AperpC confirm
        ONCE. Pass -> PREEMPT with geometry (latched). Fail / wall-lost -> ABORT (-> STOP)."""
        if self._ext_confirmed:
            return EnvStatus.CORNER_PREEMPT   # already confirmed - keep signalling PREEMPT
        if self._ext_abort:
            return EnvStatus.CORNER_ABORT

        remaining = self._ext_remaining()
        if remaining is None:
            return EnvStatus.CORNER_APPROACHING

        # paint-cutoff guard: sprayhead crosses the wall edge when the LiDAR is
        # (vertex + sprayhead_lidar_offset) forward. remaining is measured to
        # (vertex + ext_trigger_lidar), so lidar-past-vertex = ext_trigger_lidar - remaining.
        trig_lidar = self._ext_trigger_lidar_latched
        if trig_lidar is None:
            trig_lidar = self.ext_trigger_lidar
        lidar_past_vertex = trig_lidar - remaining
        if (not self._sprayhead_past
                and lidar_past_vertex >= self.sprayhead_lidar_off):
            self._sprayhead_past = True
            self.get_logger().info(
                f'[ENV] EXTERNAL paint cutoff - sprayhead past wall edge '
                f'(lidar {lidar_past_vertex:.3f}m past vertex)')

        # still travelling - keep crawling
        if remaining > self.trigger_dist_tol_under:
            return EnvStatus.CORNER_APPROACHING

        # distance in window - but only check once crawl step_complete is latched
        if not self._step_complete:
            return EnvStatus.CORNER_APPROACHING

        # crawl done. check acceptance window:
        #   remaining in (0, tol_under]  -> undershooting within 15mm -> accept
        #   remaining in [-tol_over, 0]  -> at/overshot within 50mm   -> accept
        #   remaining < -tol_over        -> overshot too far           -> warn, no check
        if remaining < -self.trigger_dist_tol_over:
            self.get_logger().warn(
                f'[ENV] EXTERNAL crawl overshot trigger by '
                f'{-remaining - self.trigger_dist_tol_over:.3f}m - outside window, no AperpC check')
            return EnvStatus.CORNER_APPROACHING

        # -- crawl done, within acceptance window: AperpC confirm ONCE --
        ext = None
        if self._scan is not None:
            ext = detect_external_corner(self._scan_dict(), self.gate)

        if ext is None and self.ext_skip_confirm:
            # TEMP bypass: near-field geometry is too thin to confirm AperpC,
            # but edge detection + approach already committed this as EXTERNAL.
            # Execute off the latched approach geometry instead of aborting.
            self.get_logger().warn(
                '[ENV] EXTERNAL confirm SKIPPED (ext_skip_confirm) - '
                'executing from prior edge/approach geometry')
            ext = {
                'angle_deg': 90.0,
                'measured_angle_deg': 90.0,
                'radian_diff': -math.pi / 2,
                'turn_dir': 'CW',
                'vertex': (self._latched_event or {}).get(
                    'vertex', np.array([0.0, 0.0])),
            }

        if ext is None:
            # doorway (no C) or wall lost -> do not turn into open space. STOP.
            self._ext_abort = True
            self.get_logger().warn(
                '[ENV] EXTERNAL confirm FAILED at trigger (no AperpC / wall lost) -> ABORT/STOP')
            return EnvStatus.CORNER_ABORT

        # confirmed - freeze the confirmed geometry for the recipe and PREEMPT.
        # Carry the configured external-only radius into turn_radius.
        self._latched_event = {
            "angle_deg": float(ext.get("angle_deg", 90.0)),
            "measured_angle_deg": float(ext.get("measured_angle_deg", 90.0)),
            "radian_diff": float(ext.get("radian_diff", -math.pi / 2)),
            "turn_dir": ext.get("turn_dir", "CW"),
            "turn_radius": float(self._ext_Re if self._ext_Re is not None else self.external_turn_radius),
            "vertex": np.asarray(ext["vertex"], float),
        }
        self._ext_confirmed = True
        self.get_logger().info(
            f'[ENV] EXTERNAL PREEMPT - AperpC confirmed, measured='
            f'{self._latched_event["measured_angle_deg"]:.1f}deg '
            f'Re={self._latched_event["turn_radius"]:.3f} '
            f'vertex=({ext["vertex"][0]:.2f},{ext["vertex"][1]:.2f})')
        return EnvStatus.CORNER_PREEMPT

    # -- fake paint actuator -----------------------------------------------------
    def _fake_paint_tick(self):
        if not self.fake_paint:
            return
        if self._robot_state == 'PAINT_EXECUTE' and not self._paint_fired:
            self._paint_ticks -= 1
            if self._paint_ticks <= 0:
                self._stroke_pub.publish(Bool(data=True))
                self._paint_fired = True
                self.get_logger().info('[ENV] fake stroke_complete published')

    # -- helpers / publish -------------------------------------------------------
    def _scan_dict(self):
        s = self._scan
        return {"ranges": np.asarray(s.ranges, dtype=np.float32),
                "angle_min": float(s.angle_min),
                "angle_increment": float(s.angle_increment)}

    def _publish(self, corner_state):
        msg = EnvStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.corner_state = corner_state

        # ------------------------------------------------------------------
        # Persistent HMI-facing corner identity.
        # Remains populated for the full lifetime of the latch, including
        # while the corner recipe is executing.
        # ------------------------------------------------------------------
        msg.latched_corner_type = EnvStatus.NONE
        msg.corner_depth = 0.0

        if self._latched:
            if self._latched_type == 'INTERNAL':
                msg.latched_corner_type = EnvStatus.INTERNAL

            elif self._latched_type == 'EXTERNAL':
                msg.latched_corner_type = EnvStatus.EXTERNAL

            elif (self._latched_type == 'BUMPOUT'
                    and self._env_bumpout_type is not None):
                msg.latched_corner_type = self._env_bumpout_type

                if self._bump_depth is not None:
                    msg.corner_depth = float(self._bump_depth)    

        # ------------------------------------------------------------------
        # Existing actionable corner geometry.
        # Only publish actionable geometry pre-turn so the FSM cannot
        # re-splice the same corner once _in_recipe is active.
        # ------------------------------------------------------------------
        if (self._latched and self._latched_type == 'INTERNAL'
                and not self._in_recipe):
            e = self._latched_event
            msg.corner_type         = EnvStatus.INTERNAL
            msg.corner_radian_diff  = float(e["radian_diff"])
            msg.corner_angle_deg    = float(e["angle_deg"])
            msg.corner_turn_dir     = e["turn_dir"]
            msg.corner_turn_radius  = float(e["turn_radius"])
            msg.corner_vertex_x     = float(e["vertex"][0])
            msg.corner_vertex_y     = float(e["vertex"][1])
            msg.front_wall_distance = float(self._cur_front)

        elif (self._latched and self._latched_type == 'EXTERNAL'
                and not self._in_recipe):
            e = self._latched_event
            msg.corner_type         = EnvStatus.EXTERNAL
            msg.corner_radian_diff  = float(e["radian_diff"])
            msg.corner_angle_deg    = float(e["angle_deg"])
            msg.corner_turn_dir     = e["turn_dir"]
            msg.corner_turn_radius  = float(e["turn_radius"])
            msg.corner_vertex_x     = float(e["vertex"][0])
            msg.corner_vertex_y     = float(e["vertex"][1])
            msg.sprayhead_past_vertex = bool(self._sprayhead_past)

        elif (self._latched and self._latched_type == 'BUMPOUT'
                and not self._in_recipe
                and self._env_bumpout_type is not None):
            e = self._latched_event
            msg.corner_type         = self._env_bumpout_type
            msg.corner_radian_diff  = float(e["radian_diff"])
            msg.corner_angle_deg    = float(e["angle_deg"])
            msg.corner_turn_dir     = e["turn_dir"]
            msg.corner_turn_radius  = float(e["turn_radius"])
            msg.corner_vertex_x     = float(e["vertex"][0])
            msg.corner_vertex_y     = float(e["vertex"][1])

        else:
            msg.corner_type = EnvStatus.INTERNAL

        # ------------------------------------------------------------------
        # crawl_remaining for wall_motion odom-tracked crawl
        # ------------------------------------------------------------------
        if self._robot_state == APPROACH_STATE:
            if self._bump_v1_odom is not None:
                msg.crawl_remaining = max(
                    0.0, self._bump_remaining() or 0.0)

            elif self._ext_vertex_odom is not None:
                msg.crawl_remaining = max(
                    0.0, self._ext_remaining() or 0.0)

            elif self._int_crawl_anchor is not None:
                msg.crawl_remaining = max(
                    0.0, self._int_remaining() or 0.0)

            else:
                msg.crawl_remaining = -1.0
        else:
            msg.crawl_remaining = -1.0

        # INTERNAL overshoot: robot is already closer to the front wall than the
        # trigger (crawl_remaining would be negative, clamped to 0 above). The
        # pivot leaves the right-wall distance ~= front distance at pivot time, so
        # report how far to reverse. Closed loop on odom: backing up raises
        # _int_remaining() back toward 0, so this shrinks to 0 as the robot moves.
        msg.crawl_backup = 0.0
        if (self._robot_state == APPROACH_STATE
                and self._int_crawl_anchor is not None
                and self._bump_v1_odom is None
                and self._ext_vertex_odom is None):
            rem = self._int_remaining()
            if rem is not None and rem < 0.0:
                over = -rem
                if over <= self.internal_max_backup:
                    msg.crawl_backup = float(over)
                else:
                    self.get_logger().warn(
                        f'[ENV] INTERNAL overshoot {over:.3f}m > max backup '
                        f'{self.internal_max_backup:.2f}m - not reversing',
                        throttle_duration_sec=2.0)

        self._env_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EnvMonitoringV2()
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