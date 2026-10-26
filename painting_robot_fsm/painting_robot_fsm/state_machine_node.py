# VRai
# 04/21/26  (drop 1 - queue-driven normal mode)
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from painting_robot_msgs.msg import (
    WallStatus, WallPose, CornerParams, EnvStatus,
    RobotCommand
)
from nav_msgs.msg import Odometry
from painting_robot_msgs.msg import WallStatus, MotionStatus, CollisionWarning, EnvStatus, CornerParams
from painting_robot_fsm.states import (
    RobotState, IdleHandler, WallAlignHandler,
    WallFollowHandler, FaultHandler,
    CornerLogicHandler,
    CornerApproachHandler,
    StepRealignHandler,
    PaintExecuteHandler,
    SystemCheckHandler,
    HoldHandler
)
from painting_robot_fsm.motion_queue import MotionQueue
from std_msgs.msg import Bool
from painting_robot_msgs.msg import SafetyCommand

class StateMachineNode(Node):

    WALL_TIMEOUT = 2.0
    WALL_TIMEOUT_ALIGN = 15.0

    def __init__(self):
        super().__init__('state_machine_node')

        self.state = RobotState.IDLE

        self.handlers = {
            RobotState.IDLE:            IdleHandler(self),
            RobotState.SYSTEM_CHECK:    SystemCheckHandler(self),
            RobotState.WALL_ALIGN:      WallAlignHandler(self),
            RobotState.WALL_FOLLOW:     WallFollowHandler(self),
            RobotState.FAULT:           FaultHandler(self),
            RobotState.CORNER_LOGIC:    CornerLogicHandler(self),
            RobotState.CORNER_APPROACH: CornerApproachHandler(self),
            RobotState.STEP_REALIGN:    StepRealignHandler(self),
            RobotState.PAINT_EXECUTE:   PaintExecuteHandler(self),
            RobotState.HOLD:            HoldHandler(self),
        }

        self.fsm_context = {
            'start_requested': False,
            'wall_status':     None,
            'wall_lost':       False,
            'safety_ok':       True,
            'corner_preempt':  False,
            'homing_complete': False,
            'collision':       None,
            'odom_x':          0.0,
            'odom_y':          0.0,
            'odom_yaw':        0.0,
            'step_complete':   False,
            'stroke_complete': False,
            'strip_count':     0,          # retained for logging/telemetry only
            'strips_per_block': 4,
            'corner_detected': None,
            'last_corner_type': None,
            '_corner_active':  False,
            'corner_reacquired': False,    # set by env when new right wall re-seen
            'corner_turn_complete': False,
            'distance_since_stroke': 0.0,
            'crawl_remaining': -1.0,
            'front_wall_distance': 0.0,
            '_internal_final_crawl_done': False,
            'sprayhead_past_vertex': False,
            '_paint_caller':   None,       # queue-item paint interleave target
        }

        # -- Motion queue - owns the plan; FSM executes it ---------------------
        self.queue = MotionQueue(
            RobotState,
            logger=self.get_logger(),
            strips_per_block=self.fsm_context['strips_per_block'],
        )

        self._last_wall_status_time = None
        self._prev_step_complete = False   # rising-edge detector for step_complete
        self._current_phase = ''           # tracks phase for same-state transitions
        self._safety_ok = False        # gate for START; False until safety_node says healthy

        # -- HOMING-GATE SKIP --------------------------------------------------
        # Bench/sim only. When the homing node/paint executor node, isn't running, /homing_complete
        # never arrives and SYSTEM_CHECK would hang. 
        self.declare_parameter('skip_homing_gate', False)
        self._skip_homing = self.get_parameter('skip_homing_gate').value
        if self._skip_homing:
            self.get_logger().warn('[FSM] HOMING GATE SKIPPED (bench/sim) - safety gate still enforced')

        self.declare_parameter('dist_tolerance', 0.07)
        self.dist_tolerance = self.get_parameter('dist_tolerance').value

        # -- REDUNDANT-STROKE GATE ---------------------------------------------
        # A WALL_ALIGN stroke is skipped when the spray head has moved less than
        # stroke_min_head_travel_m since the previous stroke (robot neither
        # translated nor rotated enough to reach new wall). Head position is the
        # odom pose plus the gantry offset (robot frame, metres).
        self.declare_parameter('stroke_min_head_travel_m', 0.05)
        self.declare_parameter('spray_head_offset_x', 0.0)
        self.declare_parameter('spray_head_offset_y', -0.312)
        self.stroke_min_head_travel = self.get_parameter('stroke_min_head_travel_m').value
        self._head_off = (self.get_parameter('spray_head_offset_x').value,
                          self.get_parameter('spray_head_offset_y').value)
        self._last_stroke_head = None
        # Gate mode: 'pose' = skip unless the robot translated >= stroke_min_robot_travel_m
        # OR rotated >= stroke_min_rotation_deg since the last stroke (robot centre, raw odom);
        # 'head' = the spray-head-travel test above. 9 deg ~ 0.05 m of head sweep at 0.312 m offset;
        # a smaller value would repaint on small align corrections (~4 deg seen after corners).
        self.declare_parameter('stroke_gate_mode', 'pose')
        self.declare_parameter('stroke_min_robot_travel_m', 0.05)
        self.declare_parameter('stroke_min_rotation_deg', 9.0)
        self.stroke_gate_mode = self.get_parameter('stroke_gate_mode').value
        self.stroke_min_robot_travel = self.get_parameter('stroke_min_robot_travel_m').value
        self.stroke_min_rotation = math.radians(self.get_parameter('stroke_min_rotation_deg').value)
        self._last_stroke_pose = None

        # -- POST-EXTERNAL-CORNER DISTANCE RECOVERY ------------------------------
        # After an external-only turn the robot can come out 0.07-0.15 m too far from the new
        # wall (real robot). Instead of faulting on WALL_DIST, WALL_ALIGN / WALL_FOLLOW accept up
        # to post_corner_dist_tol_m for at most post_corner_max_steps strips, skip strokes while
        # the error is still above dist_tolerance, and fault if it does not close. 0 disables it.
        self.declare_parameter('post_corner_dist_tol_m', 0.20)
        self.declare_parameter('post_corner_max_steps', 8)
        self.declare_parameter('post_corner_angle_tol_rad', 0.035)   # ~2 deg: align is angle-only here
        self.post_corner_dist_tol = self.get_parameter('post_corner_dist_tol_m').value
        self.post_corner_max_steps = int(self.get_parameter('post_corner_max_steps').value)
        self.post_corner_angle_tol = self.get_parameter('post_corner_angle_tol_rad').value

        # -- Subscriptions -----------------------------------------------------
 
        self.create_subscription(SafetyCommand, '/safety_command',self.safety_cmd_cb, 10)
        self.create_subscription(Bool, '/safety_status', self._safety_status_cb, 10)
 
        self.create_subscription(WallStatus, '/wall_status', self.wall_status_cb, 10)
        self.create_subscription(MotionStatus, '/motion_status', self.motion_status_cb, 10)
        self.create_subscription(Bool, '/safety_status', self.safety_status_cb, 10)
        self.create_subscription(String, '/start_cmd', self.start_cmd_cb, 10)
        self.create_subscription(Bool, '/stroke_complete', self.stroke_complete_cb, 10)
        self.create_subscription(EnvStatus, '/env_status', self.env_status_cb, 10)
        self.create_subscription(CollisionWarning, '/collision_warning', self.collision_cb, 10)
        self.create_subscription(Odometry, '/paint_bot/odom_filtered', self.odom_cb, 10)
        # Raw wheel odometry (same source wall_motion drives from) for the
        # redundant-stroke gate - the EKF output lags/jumps at turn ends.
        self.create_subscription(Odometry, '/paint_bot/odom', self.raw_odom_cb, 10)
        self.create_subscription(Bool, '/homing_complete', self._homing_complete_cb, 10)

        # -- Publishers --------------------------------------------------------
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self._command_pub = self.create_publisher(RobotCommand, '/robot_command', 10)
        self._command_id = 0
        # self._wall_mode_pub = self.create_publisher(String, '/wall_mode', 10)
        self._corner_params_pub = self.create_publisher(CornerParams, '/corner_params', 10)
        self._queue_pub = self.create_publisher(String, '/motion_queue', 10)  # HMI display
        # FSM-detected faults are REPORTED here, not self-transitioned. safety_node
        # assigns the tier (WALL_LOST -> FAULT), latches, and drives us to FAULT via
        # /safety_command - one door in, one door out. Rising-edge only (see fsm_loop).
        self._fault_report_pub = self.create_publisher(String, '/fault_report', 10)
        self._prev_wall_lost = False
        # Edge tracker for all /fault_report codes (one publish per rising edge,
        # cleared when the condition goes away). Lets handlers call report_fault()
        # freely without spamming. Keyed by code string.
        self._reported = set()

        # -- HOLD / resume state -----------------------------------------------
        # Saved return point so RESUME (2b) can re-enter the item we paused in.
        # _hold_pending defers a hold requested mid-corner until the corner ends.
        self._hold_return_state = None
        self._hold_return_phase = ''
        self._hold_pending      = False
        self._hold_reason       = ''
        # Latched by safety_cmd_cb when the stop was ESTOP tier (not plain FAULT);
        # consumed in _clear_transient_state to clear the homing mirror on ESTOP
        # recovery only. Pairs with actuator_bridge gating clear_limits to ESTOP.
        self._last_stop_was_estop = False

        self.create_timer(0.1, self.fsm_loop)

        self.handlers[self.state].on_enter()
        self.get_logger().info('StateMachineNode started (queue-driven)')

    # --- CALLBACKS ---------------------------------------

    def _homing_complete_cb(self, msg: Bool):
        if msg.data:
            self.fsm_context['homing_complete'] = True

    def stroke_complete_cb(self, msg: Bool):
        if msg.data:
            self.fsm_context['stroke_complete'] = True

    def wall_status_cb(self, msg):
        # Wall geometry/validity from wall_detection: wall_valid, aligned,
        # distance_error, angle_error.
        self.fsm_context['wall_status'] = msg
        self._last_wall_status_time = self.get_clock().now()

    def motion_status_cb(self, msg):
        # Motion sequencing events from wall_motion.
        # Rising-edge latch - step_complete is held True for several publishes.

         # Ignore completions from a command the FSM has already moved past.
        if msg.command_id != self._command_id:
            self.get_logger().warn(f'[FSM] motion REJECTED status_cmd={msg.command_id} fsm_cmd={self._command_id}')

            return

        if msg.step_complete and not self._prev_step_complete:
            self.fsm_context['step_complete'] = True
            self.get_logger().info('[FSM] step_complete rising edge latched')
        self._prev_step_complete = msg.step_complete
        if msg.corner_turn_complete:
            self.fsm_context['corner_turn_complete'] = True
        if msg.corner_reacquired:
            self.fsm_context['corner_reacquired'] = True
        self.fsm_context['distance_since_stroke'] = msg.distance_since_stroke

    def safety_status_cb(self, msg):
        # Only track the health flag. Tier transitions (HOLD/FAULT/ESTOP) are owned
        # by safety_cmd_cb via /safety_command - which distinguishes the tiers.
        # /safety_status is False for ANY non-NOMINAL tier (incl. SAFETY_HOLD), so
        # forcing FAULT here wrongly escalated every HOLD straight to FAULT.
        self.fsm_context['safety_ok'] = msg.data

    def start_cmd_cb(self, msg):
        cmd = msg.data.upper()
 
        if cmd == 'START':
            # Gate on safety health - refuse to start while unsafe.
            if not self._safety_ok:
                self.get_logger().warn('[FSM] START refused - safety not healthy (clear faults first)')
                return
            if self.state == RobotState.IDLE:
                self.fsm_context['start_requested'] = True
            else:
                self.get_logger().warn(f'[FSM] START ignored - not in IDLE (state={self.state.name})')
 
        elif cmd == 'PAUSE':
            # Operator pause - same HOLD machinery as a safety SAFETY_HOLD, same
            # corner-deferral. Preserves queue + state; resume re-issues the item.
            self._enter_hold('operator_pause')

        elif cmd == 'RESUME':
            self._resume_from_hold()
 
        elif cmd == 'TEST_STROKE':
            if self.state == RobotState.IDLE:
                self.get_logger().info('[FSM] TEST_STROKE - entering PAINT_EXECUTE')
                self._transition_to(RobotState.PAINT_EXECUTE)
            else:
                self.get_logger().warn(
                    f'[FSM] TEST_STROKE ignored - not in IDLE (state={self.state.name})')
 
        # ESTOP / RESET intentionally NOT handled here - owned by safety_node,
        # which reacts and drives the FSM via /safety_command.
 
  
    def safety_cmd_cb(self, msg):
        # SafetyCommand.tier: 0 NOMINAL, 1 SAFETY_HOLD, 2 FAULT, 3 ESTOP
        tier = msg.tier
 
        if tier in (msg.FAULT, msg.ESTOP):
            # Remember whether this stop was ESTOP - decides (on the eventual
            # RESET->IDLE) whether the homing mirror is cleared. Only ESTOP clears.
            if tier == msg.ESTOP:
                self._last_stop_was_estop = True
            if self.state != RobotState.FAULT:
                self.get_logger().error(
                    f'[FSM] safety {("ESTOP" if tier==msg.ESTOP else "FAULT")} '
                    f'({msg.reason}) - entering FAULT')
            if msg.flush_queue:
                self.queue.flush()
            # if msg.flush_limits:
            #     self._flush_gantry_limits()
            if self.state != RobotState.FAULT:
                self._transition_to(RobotState.FAULT)
 
        elif tier == msg.SAFETY_HOLD:
            # Recoverable hold - enter HOLD state, preserve everything. Resume is
            # operator-driven (RESUME), never automatic, even if the cause clears.
            self._enter_hold(f'safety:{msg.reason}')
   
        elif tier == msg.NOMINAL:
            if self.state == RobotState.FAULT:
                self.get_logger().info('[FSM] safety NOMINAL - clearing fault, returning to IDLE')
                self.queue.flush()
                self._transition_to(RobotState.IDLE)   # clears transient state
            elif self.state == RobotState.HOLD:
                # RESET while held = ABANDON the job (vs RESUME = continue). Safety
                # dropped to NOMINAL, so go IDLE and drop the saved return point.
                self.get_logger().info('[FSM] safety NOMINAL - RESET from HOLD, abandoning job -> IDLE')
                self._hold_return_state = None
                self._hold_return_phase = ''
                self._hold_pending = False
                self.queue.flush()
                self._transition_to(RobotState.IDLE)

    # --- HOLD / PAUSE --------------------------------------------
    # Corner states can't be re-entered cleanly mid-turn, so a hold requested
    # during one is DEFERRED: _hold_pending is set and the hold is taken at the
    # next ITEM_COMPLETE (fsm_loop checks it). Everywhere else, hold immediately.
    _CORNER_STATES = (RobotState.CORNER_LOGIC, RobotState.CORNER_APPROACH)
    _NO_HOLD_STATES = (RobotState.IDLE, RobotState.FAULT, RobotState.HOLD)

    def _enter_hold(self, reason: str):
        # Already held or already queued to hold - just note the latest reason.
        if self.state == RobotState.HOLD or self._hold_pending:
            self._hold_reason = reason
            return
        # Nothing to pause in these states.
        if self.state in (RobotState.IDLE, RobotState.FAULT):
            self.get_logger().info(f'[FSM] HOLD ignored - state={self.state.name}')
            return
        # Defer if mid-corner - hold at the next clean item boundary.
        if self.state in self._CORNER_STATES:
            self._hold_pending = True
            self._hold_reason  = reason
            self.get_logger().warn(
                f'[FSM] HOLD requested during {self.state.name} ({reason}) '
                f'- deferring until corner completes')
            return
        self._do_enter_hold(reason)

    # --- FSM-detected fault reporting -----------------------------------------
    # Handlers DETECT (dist error, wall lost, corner abort...) but do NOT
    # self-transition to FAULT. They call report_fault(code); safety_node assigns
    # the tier, latches, and drives us via /safety_command. One door in, one door
    # out - and RESET works because safety owns the latch. Edge-guarded so a
    # condition true for many ticks publishes once.
    def report_fault(self, code: str):
        code = code.strip().upper()
        if code in self._reported:
            return
        self._reported.add(code)
        self._fault_report_pub.publish(String(data=code))
        self.get_logger().warn(f'[FSM] reported {code} to safety_node')

    def clear_fault(self, code: str):
        # FSM-internal edge re-arm ONLY. Does NOT notify safety - fsm_loop calls
        # this every tick wall_lost reads false, so publishing a CLEAR here would
        # race the very HOLD it just caused and (safety->NOMINAL while HELD = RESET)
        # abandon the job. Safety is told to clear a recoverable condition ONLY by
        # the deliberate recovery path (publish_fault_clear), after we have already
        # transitioned out of HOLD into an executing state.
        self._reported.discard(code.strip().upper())

    def publish_fault_clear(self, code: str):
        """Explicitly tell safety to drop a recoverable reporter condition. Only
        the deliberate auto-resume path calls this - never the per-tick clear."""
        code = code.strip().upper()
        self._reported.discard(code)
        self._fault_report_pub.publish(String(data=f'CLEAR:{code}'))
        self.get_logger().info(f'[FSM] published CLEAR:{code} to safety_node')

    def _do_enter_hold(self, reason: str):
        # Save the return point so RESUME (2b) can re-enter this item. We do NOT
        # flush the queue or clear transient state - that's what makes it resumable.
        self._hold_return_state = self.state
        self._hold_return_phase = self._current_phase
        self._hold_reason       = reason
        self._hold_pending      = False
        self.get_logger().warn(
            f'[FSM] HOLD ({reason}) - pausing, saved return={self._hold_return_state.name}'
            f'/<{self._hold_return_phase}>')
        self._transition_to(RobotState.HOLD)

    def _resume_from_hold(self):
        """Operator RESUME - continue the job from where it paused. Re-enter the
        saved return state/phase; _transition_to re-issues the RobotCommand with a
        new command_id, so wall_motion/paint_executor re-arm and the current item
        runs again from its start (design B - a half-done stroke is repainted, not
        resumed mid-stroke). Only valid while actually in HOLD."""
        if self.state != RobotState.HOLD:
            self.get_logger().warn(f'[FSM] RESUME ignored - not in HOLD (state={self.state.name})')
            return
        if self._hold_return_state is None:
            self.get_logger().error('[FSM] RESUME - no saved return point, going IDLE')
            self._transition_to(RobotState.IDLE)
            return
        ret, ph = self._hold_return_state, self._hold_return_phase
        self._hold_return_state = None
        self._hold_return_phase = ''
        # A paused PAINT_EXECUTE cannot be resumed mid-stroke: the executor's
        # stroke is stateful (direction, start-Z) and was aborted on the way into
        # HOLD, so re-issuing the same stroke either completes instantly (Z already
        # at target) or stalls (Z elsewhere, timeout). Resume at the item's START
        # instead - WALL_ALIGN - which re-aligns then repaints the strip cleanly
        # from a known state. The queue item ("align+paint") is unchanged, so this
        # just re-runs it from its first phase.
        if ret == RobotState.PAINT_EXECUTE:
            self.get_logger().info('[FSM] RESUME - paused mid-paint; restarting item at WALL_ALIGN')
            ret, ph = RobotState.WALL_ALIGN, 'ALIGN'
        else:
            self.get_logger().info(f'[FSM] RESUME - re-entering {ret.name}/<{ph}>')
        self._transition_to(ret, ph)

    def _safety_status_cb(self, msg):
        self._safety_ok = bool(msg.data)
 
    def spray_head_xy(self):
        """Spray-head position in the odom frame (raw wheel odom pose + gantry offset)."""
        c = self.fsm_context
        yaw = c.get('raw_odom_yaw', 0.0)
        ox, oy = self._head_off
        return (c.get('raw_odom_x', 0.0) + math.cos(yaw) * ox - math.sin(yaw) * oy,
                c.get('raw_odom_y', 0.0) + math.sin(yaw) * ox + math.cos(yaw) * oy)

    def _robot_pose(self):
        c = self.fsm_context
        return (c.get('raw_odom_x', 0.0), c.get('raw_odom_y', 0.0), c.get('raw_odom_yaw', 0.0))

    def note_stroke_started(self):
        self._last_stroke_head = self.spray_head_xy()
        self._last_stroke_pose = self._robot_pose()

    def reset_stroke_gate(self):
        """Forget the last stroke position (job start, HOLD, FAULT): the next
        stroke must always paint."""
        self._last_stroke_head = None
        self._last_stroke_pose = None

    def stroke_is_redundant(self):
        """True if the last stroke already covers this pose (see stroke_gate_mode)."""
        if self.stroke_gate_mode == 'pose':
            return self._stroke_redundant_pose()
        if self._last_stroke_head is None:
            return False
        hx, hy = self.spray_head_xy()
        moved = math.hypot(hx - self._last_stroke_head[0], hy - self._last_stroke_head[1])
        if moved < self.stroke_min_head_travel:
            self.get_logger().info(
                f'[FSM] stroke SKIPPED - spray head moved only {moved:.3f}m '
                f'(< {self.stroke_min_head_travel:.2f}m) since the last stroke')
            return True
        self.get_logger().info(
            f'[FSM] stroke gate: spray head moved {moved:.3f}m since the last stroke - painting')
        return False

    def _stroke_redundant_pose(self):
        if self._last_stroke_pose is None:
            return False
        x, y, yaw = self._robot_pose()
        lx, ly, lyaw = self._last_stroke_pose
        moved = math.hypot(x - lx, y - ly)
        turned = abs(math.atan2(math.sin(yaw - lyaw), math.cos(yaw - lyaw)))   # wrapped to [0, pi]
        if moved < self.stroke_min_robot_travel and turned < self.stroke_min_rotation:
            self.get_logger().info(
                f'[FSM] stroke SKIPPED - robot moved only {moved:.3f}m (< {self.stroke_min_robot_travel:.2f}m) '
                f'and turned only {math.degrees(turned):.1f}deg (< {math.degrees(self.stroke_min_rotation):.0f}deg) '
                f'since the last stroke')
            return True
        self.get_logger().info(
            f'[FSM] stroke gate: robot moved {moved:.3f}m, turned {math.degrees(turned):.1f}deg '
            f'since the last stroke - painting')
        return False

    def _clear_transient_state(self):
        """Wipe all transient run/progress state. Called on every entry to IDLE
        (job complete, stop, or fault-reset) - one place, no duplication."""
        self.reset_stroke_gate()
        ctx = self.fsm_context
        ctx['_corner_active']        = False
        ctx['corner_detected']       = None
        ctx['corner_preempt']        = False
        ctx['corner_turn_complete']  = False
        ctx['corner_reacquired']     = False
        ctx['sprayhead_past_vertex'] = False
        ctx['step_complete']         = False
        ctx['stroke_complete']       = False
        ctx['distance_since_stroke'] = 0.0
        ctx['_paint_caller']         = None
        ctx['_dist_recovery']        = False
        ctx['_dist_recovery_steps']  = 0
        ctx['wall_lost']             = False
        self._reported.clear()       # re-arm /fault_report edges for next run
        # add the ones the old block missed but _clear_fault_context had:
        ctx['strip_count']           = 0
        ctx['last_corner_type']      = None
        ctx['wall_status']           = None
        ctx['_internal_final_crawl_done'] = False
        ctx['crawl_remaining'] = -1.0
        ctx['front_wall_distance'] = 0.0
        # homing_complete is the FSM's MIRROR of the actuator's _homed. It is
        # cleared ONLY on ESTOP (paired with actuator clear_limits), never on an
        # ordinary IDLE/FAULT-reset - otherwise a recoverable fault would force a
        # re-teach. See _enter_estop_clear below; cleared there, not here.
        if self._last_stop_was_estop:
            ctx['homing_complete'] = False
            self._last_stop_was_estop = False
            self.get_logger().warn('[FSM] ESTOP recovery - homing mirror cleared (re-teach required)')
        self._prev_step_complete = False
        self._current_phase = ''

        self.get_logger().info('[FSM] Context cleared')
        
  
    def env_status_cb(self, msg: EnvStatus):
        if self.state in (RobotState.IDLE, RobotState.FAULT):

            return

        is_external = (msg.corner_type == EnvStatus.EXTERNAL)
        is_internal= (msg.corner_type == EnvStatus.INTERNAL)
        is_bumpout = (hasattr(EnvStatus, 'BUMPOUT')
                      and msg.corner_type == EnvStatus.BUMPOUT)


        self.fsm_context['sprayhead_past_vertex'] = msg.sprayhead_past_vertex
        # Forward crawl distance still to cover to the trigger (LiDAR frame). Used
        # by CornerApproachHandler to decide whether a finished chunk needs a
        # re-arm (more crawl) or is the turn-now case (remaining ~ 0, confirm now).
        self.fsm_context['crawl_remaining'] = msg.crawl_remaining
        self.fsm_context['front_wall_distance'] = msg.front_wall_distance
        self.fsm_context['latched_corner_type'] = msg.latched_corner_type

        if msg.corner_state == EnvStatus.CORNER_APPROACHING:

            self.fsm_context['corner_detected']  = msg
            self.fsm_context['last_corner_type']  = msg.corner_type

            # -- Back-to-back corner recovery ----------------------------------
            # If we are HELD because the wall was lost (e.g. landed AT the next
            # external corner right after an internal turn), and env now reports an
            # actionable EXTERNAL corner, auto-RESUME and splice the external recipe.
            # Narrowly gated: ONLY a wall_lost hold, ONLY an external corner. An
            # operator pause or an obstacle hold never auto-resumes here.
            if (self.state == RobotState.HOLD and is_external
                    and 'wall_lost' in (self._hold_reason or '').lower()):
                self._auto_resume_for_external(msg)
                return

            # External terminal-edge latch -> splice the external recipe.
            # Guard: only while in a normal painting state (not already cornering),
            # and only for EXTERNAL. splice_external is itself idempotent.
            if is_external and not self.fsm_context.get('_corner_active', False) \
                    and self._normal_corner_latch_allowed():
                self.get_logger().info('[FSM] external edge latched -> splice_external')
                self.queue.splice_external(msg)
                self.fsm_context['_corner_active'] = True
                self._publish_corner_params(msg)

            elif is_bumpout and not self.fsm_context.get('_corner_active', False) \
                    and self._normal_corner_latch_allowed():
                self.get_logger().info('[FSM] bumpout latched -> splice_bumpout')
                self.queue.splice_bumpout(msg)
                self.fsm_context['_corner_active'] = True
                # env2 publishes the full CornerParams directly for BUMPOUT
                # because EnvStatus only carries ARC1 fields; ARC2 lives in
                # CornerParams.arc2_*.
                    

            elif is_internal and not self.fsm_context.get('_corner_active', False) \
                    and self._normal_corner_latch_allowed():
                self.get_logger().info('[FSM] internal corner latched -> splice_internal')
                self.queue.splice_internal(msg)
                self.fsm_context['_corner_active'] = True
                self._publish_corner_params(msg)


    


        elif msg.corner_state == EnvStatus.CORNER_PREEMPT:
            # Turn gate reached (lidar->vertex range <= threshold). CornerApproach
            # reads this and completes the CRAWL item -> TURN.
            self.fsm_context['corner_preempt'] = True

        elif msg.corner_state == EnvStatus.CORNER_ABORT:
            # External AperpC confirm failed at the trigger (doorway / no wall C /
            # wall lost). Do NOT turn into open space - stop. Report CORNER_ABORT
            # (safety grants FAULT tier, needs RESET) instead of self-transitioning,
            # so RESET recovers through the one-door path.
            if self.fsm_context.get('_corner_active', False):
                self.get_logger().warn(
                    '[FSM] EXTERNAL abort (no AperpC / wall lost at vertex) -> reporting CORNER_ABORT')
                self.report_fault('CORNER_ABORT')

        elif msg.corner_state == EnvStatus.CORNER_CLEAR:
            # env drops the latch; don't clear mid-sequence (corner states manage
            # their own teardown). Only clear if we're not in the corner recipe.
            if not self.fsm_context.get('_corner_active', False):
                self.fsm_context['corner_detected'] = None

    def _normal_corner_latch_allowed(self):
        """True when it is safe to splice a corner recipe after the CURRENT normal
        queue item. Normal painting states (WALL_FOLLOW, STEP_REALIGN, WALL_ALIGN)
        always qualify. PAINT_EXECUTE also qualifies ONLY when the current queue
        item is a normal WALL_ALIGN (i.e. the align+paint item's stroke) - the
        corner is confirmed mid-stroke (Stage A voting now spans the stroke), so we
        splice immediately and the queue is already prepared to advance into
        CORNER_APPROACH when the stroke ends, instead of dropping into a WALL_FOLLOW
        strip first. A PAINT_EXECUTE that belongs to a corner recipe (segment
        swatch / pre-turn strip) is NOT eligible - its current item is a corner
        item, so this returns False there."""
        if self.state in (RobotState.WALL_FOLLOW,
                          RobotState.STEP_REALIGN,
                          RobotState.WALL_ALIGN):
            return True
        if self.state == RobotState.PAINT_EXECUTE:
            item = self.queue.current_item()
            return item is not None and item.state == RobotState.WALL_ALIGN
        return False

    def _auto_resume_for_external(self, env_msg):
        """Leave a wall_lost HOLD because env found the next external corner.
        Narrow recovery for the back-to-back-corner case (deep bump run as two
        independent corners). We:
          1) drop the saved HOLD return point (we are NOT resuming the old item -
             we are splicing a fresh external recipe),
          2) transition into WALL_FOLLOW so the external splice guard is satisfied
             and _transition_to clears wall_lost + refreshes the wall-status clock
             (which makes fsm_loop stop reporting WALL_LOST next tick -> safety
             drops the hold -> NOMINAL),
          3) splice the external recipe and mark the corner active,
          4) ONLY THEN tell safety to clear WALL_LOST.
        ORDER IS CRITICAL. safety->NOMINAL while the FSM is still in HOLD triggers
        the existing 'RESET from HOLD -> abandon job -> IDLE' rule (safety_cmd_cb).
        So we must LEAVE HOLD (transition to WALL_FOLLOW) BEFORE publishing the
        CLEAR that makes safety go NOMINAL - otherwise the NOMINAL lands while
        we're still HELD and the job is thrown away. Only ever called for a
        wall_lost hold + external corner (caller-gated)."""
        self.get_logger().warn(
            '[FSM] wall_lost HOLD + external corner reported by env -> '
            'auto-resume, splicing external recipe')
        self._hold_return_state = None
        self._hold_return_phase = ''
        self._hold_pending      = False
        # 1) LEAVE HOLD FIRST - _transition_to also resets wall_lost + wall clock.
        self.fsm_context['wall_lost'] = False
        self._transition_to(RobotState.WALL_FOLLOW)
        # 2) retire the SPENT internal recipe. We held because the wall was lost
        #    AFTER the internal turn finished, so the queue is still parked on that
        #    recipe's CORNER_LOGIC/TURN item (safety-driven HOLD doesn't advance the
        #    queue, unlike a deferred-at-ITEM_COMPLETE hold). Advance past the corner
        #    items so current_item() is no longer CORNER_*, which lets the NORMAL
        #    splice_external pass its own 'already in corner recipe' guard. No force.
        while self.queue.is_active() and self.queue.current_item() is not None \
                and self.queue.current_item().state in (
                    RobotState.CORNER_APPROACH, RobotState.CORNER_LOGIC):
            self.queue.advance()
        # 3) splice the external recipe the normal way, now that we're clear
        self.queue.splice_external(env_msg)
        self.fsm_context['_corner_active'] = True
        self._publish_corner_params(env_msg)
        # 3) NOW it is safe for safety to go NOMINAL - we are no longer in HOLD,
        #    so the abandon-job rule won't fire. Explicitly clear the hold.
        self.publish_fault_clear('WALL_LOST')

    def collision_cb(self, msg: CollisionWarning):
        self.fsm_context['collision'] = msg

    def raw_odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.fsm_context['raw_odom_x'] = msg.pose.pose.position.x
        self.fsm_context['raw_odom_y'] = msg.pose.pose.position.y
        self.fsm_context['raw_odom_yaw'] = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.fsm_context['odom_x'] = x
        self.fsm_context['odom_y'] = y
        self.fsm_context['odom_yaw'] = yaw

    # --- TRANSITION --------------------

    def _transition_to(self, new_state: RobotState, new_phase: str = ''):
        # Phase recovery: CORNER_LOGIC is dispatched by phase (TURN / ADVANCE / ARC...).
        # Some paths return CORNER_LOGIC directly without supplying the phase.
        # Recover it from the current queue item so genuine new corner commands always
        # reach wall_motion with the correct phase.
        if new_state == RobotState.CORNER_LOGIC and not new_phase:
            item = self.queue.current_item()
            if item is not None and getattr(item, 'phase', ''):
                new_phase = item.phase

        # Nothing to do if state AND phase are unchanged.
        # Same-state phase changes must still pass through (e.g. ARC1 -> ARC2).
        if new_state == self.state and new_phase == self._current_phase:
            return

        old_state = self.state

        # ------------------------------------------------------------------
        # Intermediate paint inside a segmented internal TURN.
        #
        # wall_motion already owns the active turn and is holding:
        #   _corner_seg_index
        #   _corner_waiting_for_paint
        #
        # Returning from PAINT_EXECUTE is therefore a RESUME of the existing
        # motion action, not a new TURN command.
        #
        # Do not publish a fresh RobotCommand here, because a new command_id
        # makes wall_motion call _reset_corner_state() and resets seg_index=0.
        #
        # corner_turn_complete=False distinguishes an intermediate swatch from
        # completion of the whole turn.
        # ------------------------------------------------------------------
        resume_segmented_turn = (
            old_state == RobotState.PAINT_EXECUTE
            and new_state == RobotState.CORNER_LOGIC
            and new_phase in ('TURN', 'ARC1','ARC2')
            and not self.fsm_context.get('corner_turn_complete', False)
        )

        same_state = (new_state == self.state)

        if not same_state:
            self.get_logger().info(
                f'[FSM] {old_state.name} -> {new_state.name}'
            )
            self.handlers[old_state].on_exit()
            self.state = new_state
            self.handlers[self.state].on_enter()
        else:
            self.get_logger().info(
                f'[FSM] {new_state.name} phase '
                f'{self._current_phase}->{new_phase}'
            )

        self._current_phase = new_phase

        if new_state != RobotState.IDLE:
            self.fsm_context['start_requested'] = False

        if new_state == RobotState.IDLE:
            self._clear_transient_state()
            self.get_logger().info(
                '[FSM] IDLE - all transient state cleared'
            )

        # Clear corner-active bookkeeping when leaving CORNER_LOGIC.
        if (
            old_state == RobotState.CORNER_LOGIC
            and new_state != RobotState.CORNER_LOGIC
        ):
            self.fsm_context['_corner_active'] = False
            self.fsm_context['corner_preempt'] = False

        if new_state in (
            RobotState.WALL_ALIGN,
            RobotState.WALL_FOLLOW,
            RobotState.STEP_REALIGN,
            RobotState.CORNER_APPROACH,
            RobotState.PAINT_EXECUTE,
        ):
            self._last_wall_status_time = self.get_clock().now()
            self.fsm_context['wall_lost'] = False

        # Clear stale step_complete when entering states that must see only a
        # freshly-armed step.
        if new_state in (
            RobotState.WALL_ALIGN,
            RobotState.WALL_FOLLOW,
            RobotState.STEP_REALIGN,
            RobotState.PAINT_EXECUTE,
            RobotState.CORNER_APPROACH,
            RobotState.CORNER_LOGIC,
        ):
            self.fsm_context['step_complete'] = False
            self._prev_step_complete = False

        # Clear stale turn-complete only for a genuinely fresh CORNER_LOGIC entry.
        # Do NOT clear it on the paint dip of an existing segmented turn.
        if (
            new_state == RobotState.CORNER_LOGIC
            and old_state not in (
                RobotState.CORNER_LOGIC,
                RobotState.PAINT_EXECUTE,
            )
        ):
            self.fsm_context['corner_turn_complete'] = False

        # ------------------------------------------------------------------
        # Publish RobotCommand for genuinely new motion actions.
        # ------------------------------------------------------------------
        ACTION_STATES = {
            RobotState.PAINT_EXECUTE,
            RobotState.WALL_FOLLOW,
            RobotState.STEP_REALIGN,
            RobotState.WALL_ALIGN,
            RobotState.CORNER_APPROACH,
            RobotState.CORNER_LOGIC,
            RobotState.SYSTEM_CHECK,
        }

        if new_state in ACTION_STATES and not resume_segmented_turn:
            self._command_id += 1

            cmd = RobotCommand()
            cmd.state = new_state.name
            cmd.command_id = self._command_id
            cmd.strip_index = self.fsm_context.get('strip_count', 0)
            cmd.phase = new_phase

            self._command_pub.publish(cmd)

            self.get_logger().info(
                f'[FSM] publishing cmd state={cmd.state} '
                f'phase=<{cmd.phase}> id={cmd.command_id}'
            )

        elif resume_segmented_turn:
            self.get_logger().info(
                '[FSM] segmented TURN resume after paint - '
                'no new RobotCommand; preserving wall_motion segment state'
            )


    # def _transition_to(self, new_state: RobotState, new_phase: str = ''):
    #     # Phase recovery: CORNER_LOGIC is dispatched by phase (TURN / ADVANCE), and
    #     # wall_motion only (re)initialises the turn when it receives phase='TURN'.
    #     # Some transition paths reach CORNER_LOGIC without passing the phase (a
    #     # handler returning the state directly rather than via the ITEM_COMPLETE
    #     # queue-advance that carries nxt.phase), which would publish phase='' and
    #     # make wall_motion silently skip the turn. Derive the phase from the current
    #     # queue item when none was supplied, so the published command always carries
    #     # the real phase regardless of how we got here.
    #     if new_state == RobotState.CORNER_LOGIC and not new_phase:
    #         item = self.queue.current_item()
    #         if item is not None and getattr(item, 'phase', ''):
    #             new_phase = item.phase
    #     # Skip only if BOTH state and phase are unchanged. A same-state phase
    #     # change (e.g. CORNER_LOGIC TURN->REACQUIRE) must still publish a command.
    #     if new_state == self.state and new_phase == self._current_phase:
    #         return
 
    #     old_state = self.state
    #     same_state = (new_state == self.state)
    #     if not same_state:
    #         self.get_logger().info(f'[FSM] {old_state.name} -> {new_state.name}')
    #         self.handlers[old_state].on_exit()
    #         self.state = new_state
    #         self.handlers[self.state].on_enter()
    #     else:
    #         self.get_logger().info(
    #             f'[FSM] {new_state.name} phase {self._current_phase}->{new_phase}')
    #     self._current_phase = new_phase

    #     if new_state != RobotState.IDLE:
    #         self.fsm_context['start_requested'] = False

    #     if new_state == RobotState.IDLE:
    #         self._clear_transient_state()
    #         self.get_logger().info('[FSM] IDLE - all transient state cleared')

    #     # clear corner active on exit from corner logic    

    #     if old_state == RobotState.CORNER_LOGIC and new_state != RobotState.CORNER_LOGIC:
    #         self.fsm_context['_corner_active'] = False   
    #         self.fsm_context['corner_preempt'] = False  

    #     if new_state in (
    #         RobotState.WALL_ALIGN,
    #         RobotState.WALL_FOLLOW,
    #         RobotState.STEP_REALIGN,
    #         RobotState.CORNER_APPROACH,
    #         RobotState.PAINT_EXECUTE,
    #     ):
    #         self._last_wall_status_time = self.get_clock().now()
    #         self.fsm_context['wall_lost'] = False

    #     # Clear stale step_complete when entering states that must see only a
    #     # freshly-armed step. _prev_step_complete=True suppresses re-latch of a
    #     # still-held-high signal from the step that just finished.
    #     if new_state in (RobotState.WALL_ALIGN,
    #                      RobotState.WALL_FOLLOW,
    #                       RobotState.STEP_REALIGN,
    #                      RobotState.PAINT_EXECUTE,
    #                      RobotState.CORNER_APPROACH,
    #                      RobotState.CORNER_LOGIC):
    #         self.fsm_context['step_complete'] = False
    #         self._prev_step_complete = False

    #     # Clear stale corner_turn_complete on entry to CORNER_LOGIC. Like
    #     # step_complete, wall_motion holds corner_turn_complete HIGH for several
    #     # publishes after a turn finishes, so a fresh TURN (e.g. the external turn
    #     # after an internal one in a back-to-back corner) would see the PREVIOUS
    #     # turn's completion on tick 1 and finish instantly without rotating.
    #     # if new_state == RobotState.CORNER_LOGIC:
    #     #     self.fsm_context['corner_turn_complete'] = False

    #     if new_state == RobotState.CORNER_LOGIC and \
    #             old_state not in (RobotState.CORNER_LOGIC, RobotState.PAINT_EXECUTE):
    #         self.fsm_context['corner_turn_complete'] = False    

    #     # -- Publish RobotCommand by STATE  ----------------
    #     ACTION_STATES = {
    #         RobotState.PAINT_EXECUTE,
    #         RobotState.WALL_FOLLOW,
    #         RobotState.STEP_REALIGN,
    #         RobotState.WALL_ALIGN,
    #         RobotState.CORNER_APPROACH,
    #         RobotState.CORNER_LOGIC,
    #         RobotState.SYSTEM_CHECK,
    #     }
    #     if new_state in ACTION_STATES:
    #         self._command_id += 1
    #         cmd = RobotCommand()
    #         cmd.state = new_state.name
    #         cmd.command_id = self._command_id
    #         cmd.strip_index = self.fsm_context.get('strip_count', 0)
    #         cmd.phase = new_phase
    #         self._command_pub.publish(cmd)
    #         self.get_logger().info(
    #             f'[FSM] publishing cmd state={cmd.state} phase=<{cmd.phase}> id={cmd.command_id}')


    def _republish_current_action(self):
        """Re-issue the CURRENT state's command with a fresh command_id, without
        any transition. Used to re-arm a chunked crawl: the FSM stays in
        CORNER_APPROACH across chunks, so there is no transition to trigger a new
        command, yet wall_motion needs a new command_id to reset _step_done and
        reread crawl_remaining for the next (shortened) chunk."""
        self._command_id += 1
        cmd = RobotCommand()
        cmd.state = self.state.name
        cmd.command_id = self._command_id
        cmd.strip_index = self.fsm_context.get('strip_count', 0)
        cmd.phase = self._current_phase
        self._command_pub.publish(cmd)
        self.get_logger().info(
            f'[FSM] re-arm cmd state={cmd.state} phase=<{cmd.phase}> id={cmd.command_id}')


    def fsm_loop(self):
        # -- Wall timeout ------------------------------------------------------
        if self._last_wall_status_time is None:
            self.fsm_context['wall_lost'] = False
        else:
            elapsed = (self.get_clock().now()
                       - self._last_wall_status_time).nanoseconds / 1e9
            timeout = (self.WALL_TIMEOUT_ALIGN
                       if self.state == RobotState.WALL_ALIGN
                       else self.WALL_TIMEOUT)
            self.fsm_context['wall_lost'] = (
                elapsed > timeout and
                self.state not in (RobotState.IDLE, RobotState.FAULT,
                                   RobotState.CORNER_LOGIC, RobotState.CORNER_APPROACH)
            )

        # Report WALL_LOST to safety_node on the rising edge. The FSM owns the
        # DETECTION (it has the state context to know wall-loss matters here and
        # not during a corner turn); safety_node owns the DECISION and routes us
        # back to FAULT via /safety_command. Fire once per loss, not every tick.
        wall_lost_now = self.fsm_context['wall_lost']
        if wall_lost_now:
            self.report_fault('WALL_LOST')
        else:
            self.clear_fault('WALL_LOST')
        self._prev_wall_lost = wall_lost_now

        # -- Evaluate current handler ------------------------------------------
        next_state = self.handlers[self.state].evaluate(self.fsm_context)

        # -- CORNER_APPROACH chunk re-arm --------------------------------------
        # The crawl is chunked: the handler finishes a chunk, consumes
        # step_complete, and (not painting, not yet PREEMPTed) stays in
        # CORNER_APPROACH. That same-state return would never republish a command,
        # so wall_motion would sit stopped. Re-issue the command here so the next
        # shortened chunk starts. Only when we truly stay in CORNER_APPROACH.
        if self.fsm_context.pop('_rearm_corner_approach', False):
            if (self.state == RobotState.CORNER_APPROACH
                    and next_state == RobotState.CORNER_APPROACH):
                self._republish_current_action()

        # -- Sentinel interception: ITEM_COMPLETE advances the queue -----------
        # A handler returning ITEM_COMPLETE means "this queue item is done."
        # We ask the queue for the next item and route to ITS state. The sentinel
        # is never entered as a state.

        if next_state == RobotState.ITEM_COMPLETE:
            # A hold deferred during a corner is taken here, at the clean item
            # boundary: advance the queue to the next item first (so RESUME
            # re-enters that item fresh), then hold instead of running it.
            if self._hold_pending:
                if self.queue.is_active():
                    self.queue.advance()
                self.get_logger().info('[FSM] deferred HOLD - corner item done, holding now')
                self._do_enter_hold(self._hold_reason)
                next_state = self.state
            elif self.queue.is_active():
                nxt = self.queue.advance()
                if nxt is not None:
                    # transition (or same-state phase change) with the new item's
                    # phase - _transition_to publishes the command either way.
                    self._transition_to(nxt.state, nxt.phase)
                    next_state = self.state
                else:
                    next_state = RobotState.IDLE
            else:
                self.get_logger().warn('[FSM] ITEM_COMPLETE with inactive queue -> IDLE')
                next_state = RobotState.IDLE

        # -- Transition --------------------------------------------------------
        if next_state != self.state:
            self._transition_to(next_state)

        # if next_state != self.state:
        #     item = self.queue.current_item()

        #     if item is not None and item.state == next_state:
        #         self._transition_to(next_state, item.phase)
        #     else:
        #         self._transition_to(next_state)    

        # -- Publish state + queue view ----------------------------------------
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

        qmsg = String()
        qmsg.data = ' | '.join(self.queue.upcoming_labels())
        self._queue_pub.publish(qmsg)

    # --- HELPERS ---------------------------------------------------

    # def _publish_wall_mode(self, mode: str):
    #     m = String()
    #     m.data = mode
    #     self._wall_mode_pub.publish(m)

    def _publish_corner_params(self, env_msg=None):
        msg = CornerParams()
        msg.header.stamp = self.get_clock().now().to_msg()
        if env_msg is not None:
            msg.corner_type = ('INTERNAL'      if env_msg.corner_type == EnvStatus.INTERNAL
                               else 'EXTERNAL'  if env_msg.corner_type == EnvStatus.EXTERNAL
                               else 'EXTERNAL_EDGE')
            msg.radian_diff = float(env_msg.corner_radian_diff)
            msg.angle_deg   = float(env_msg.corner_angle_deg)
            msg.distance    = float(env_msg.front_wall_distance)
            msg.turn_radius = float(env_msg.corner_turn_radius)
        self._corner_params_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()