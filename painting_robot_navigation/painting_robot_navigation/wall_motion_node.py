# VRai 
#5/01/26

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from painting_robot_msgs.msg import WallPose, WallStatus, MotionStatus, CornerParams, RobotCommand, EnvStatus

from std_msgs.msg import String
import math

from nav_msgs.msg import Odometry
import math



class WallMotionNode(Node):

    def __init__(self):
        super().__init__('wall_motion_node')

        # Parameters
        self.declare_parameter('target_distance', 0.77)      # 12 inches in meters in lidar frame
        self.declare_parameter('dist_tolerance', 0.07)        # m
        self.declare_parameter('angle_tolerance', 0.02)       # rad (~1.7 deg)
        self.declare_parameter('max_linear_vel', 0.15)        # m/s
        self.declare_parameter('max_angular_vel', 0.45)        # rad/s
        self.declare_parameter('dist_kp', 0.8)                # P gain for distance
        self.declare_parameter('angle_kp', 0.5)               # P gain for angle
        self.declare_parameter('follow_speed', 0.08)          # m/s forward speed
        self.declare_parameter('crawl_speed', 0.06)
        self.declare_parameter('follow_angle_correct', False)
        self.declare_parameter('lidar_offset_x', 0.195)
        self.declare_parameter('lidar_offset_y', -0.147)
       
   
        # Set True on WALL_ALIGN->WALL_FOLLOW transition (see robot_command callback
        # below), consumed (cleared) after the first WALL_FOLLOW step completes.
    

        self.declare_parameter('strafe_dist_threshold', 0.05)   # m - beyond this, strafe; below, angle-only
        self.declare_parameter('recheck_dist_tol_multiplier', 2.0)
        self.declare_parameter('recheck_angle_tol_multiplier', 2.0)

        self.declare_parameter('move_step_size', 0.20)   # m - matches prototype 8 inches  
        self.declare_parameter('step_realign_dist_kp', 0.15)
        self.declare_parameter('step_realign_angle_kp', 0.5)  # softer gain during step
        self.declare_parameter('step_realign_heading_kp', 1.0)
        self.declare_parameter('follow_dist_kp', 0.9)
        # Largest approach angle the wall-follow distance correction may command. A big distance error
        # (e.g. +0.17 m after an external corner on the real robot) gave atan(1.8*0.17)=17 deg, the robot
        # turned into the wall and the lidar lost it ("Right wall - NOT DETECTED"). 0 disables the cap.
        self.declare_parameter('max_approach_angle_deg', 10.0)
        self.declare_parameter('escalation_threshold_m', 0.045)
        self.declare_parameter('follow_heading_kp', 0.5)
#   

        self.declare_parameter('corner_turn_segments',   2)
        self.declare_parameter('bump_turn_segments',     2)    # paint-stroke splits per bump-out arc (ARC1/ARC2); 1 = no intermediate strokes
        self.declare_parameter('corner_turn_speed',      0.3)
        self.declare_parameter('corner_turn_tolerance',  3.0)
        self.declare_parameter('corner_swatch_pause_s',  5.0)
        self.declare_parameter('corner_fault_timeout_s', 10.0)
        self.declare_parameter('corner_turn_kp',         1.2)
        self.declare_parameter('arc_max_ang',            0.6)   # rad/s cap for arc turns (near-spin anti-whip)
        self.declare_parameter('arc_finish_ang_floor',   0.02)  # rad/s floor applied in the last ~8deg so the arc creeps to the target instead of blowing past it

        self.declare_parameter('external_reacquire_dist',  0.5)
        self.declare_parameter('external_reacquire_speed', 0.08)
        self.declare_parameter('external_turn_mode',  'arc')   # 'arc' or 'spin' (spin = tested fallback)
        self.declare_parameter('external_arc_radius', 0.8)      # R_axis for arc mode (m)
        
        self.move_step_size    = self.get_parameter('move_step_size').value

        self.step_realign_dist_kp = self.get_parameter('step_realign_dist_kp').value
        self.strafe_thresh        = self.get_parameter('strafe_dist_threshold').value
        self.follow_heading_kp = self.get_parameter('follow_heading_kp').value
        self.follow_dist_kp = self.get_parameter('follow_dist_kp').value
        self.max_approach = math.radians(self.get_parameter('max_approach_angle_deg').value)
        self.escalation_threshold_m = self.get_parameter('escalation_threshold_m').value
        
        self.recheck_dist_mult    = self.get_parameter('recheck_dist_tol_multiplier').value
        self.recheck_angle_mult   = self.get_parameter('recheck_angle_tol_multiplier').value
        self.step_realign_angle_kp = self.get_parameter('step_realign_angle_kp').value
        self.step_realign_heading_kp = self.get_parameter('step_realign_heading_kp').value
        self.lidar_offset_x = self.get_parameter('lidar_offset_x').value
        self.lidar_offset_y = self.get_parameter('lidar_offset_y').value
        self._align_strafe_angle_error = None

        # Align phase tracker - internal sub-state for strafe sequence
        # Phases: 'DIST_TURN1' | 'DIST_DRIVE' | 'DIST_TURN2' | 'ANGLE' | 'DONE'
        self._align_phase         = 'DONE'
        self._align_start_yaw     = None
        self._align_start_x       = None
        self._align_start_y       = None
        self._align_target_dist   = 0.0
        self._align_drive_direction = 1.0

        
        self._step_start_x   = None
        self._step_start_y   = None
        self._step_start_yaw = 0.0
        self._step_done     = False
        self._strip_mode    = None
        
        # corner turn internal state
        self._corner_seg_index   = 0       # current segment (0-based)
        self._corner_start_yaw   = None    # yaw snapshot at start of each segment
        self._int_origin_yaw     = None    # yaw at start of an INTERNAL corner (drift-free segment targets)
        self._int_seg_target     = 0.0
        self._corner_seg_angle   = 0.0     # target angle per segment (total/N)
        self._corner_pause_start = None    # swatch pause timer
        self._corner_done        = False   # signals FSM completion
        self._corner_turn_dir    = 1.0     # +1 CCW, -1 CW
        self._corner_turn_radius = 0.0     # arc radius for this corner (m), from CornerParams
        self._last_corner_command_id = -1  # gate for command_id-based corner trigger
        self._corner_waiting_for_paint = False
        # Completion latches. These prevent a completed TURN/REACQUIRE from
        # repeatedly publishing stale MotionStatus events on later control ticks.
        self._corner_turn_done = False
        self._corner_reacquire_done = False
        # bump-out second (BACK) arc, stashed on the ARC1 command, consumed on ARC2
        self._bump_arc2_radian_diff = 0.0
        self._bump_arc2_turn_radius = 0.0

        self.target_dist  = self.get_parameter('target_distance').value
        self.dist_tol     = self.get_parameter('dist_tolerance').value
        self.angle_tol    = self.get_parameter('angle_tolerance').value
        self.max_lin      = self.get_parameter('max_linear_vel').value
        self.max_ang      = self.get_parameter('max_angular_vel').value
        self.dist_kp      = self.get_parameter('dist_kp').value
        self.angle_kp     = self.get_parameter('angle_kp').value
        self.follow_speed = self.get_parameter('follow_speed').value
        self.follow_angle_correct = self.get_parameter('follow_angle_correct').value
        self.crawl_speed = self.get_parameter('crawl_speed').value
        self._backup_tol = 0.012   # m - stop reversing within this of the internal trigger

        # turn params
        self._corner_segments   = self.get_parameter('corner_turn_segments').value
        self._bump_segments     = self.get_parameter('bump_turn_segments').value
        self._corner_speed      = self.get_parameter('corner_turn_speed').value
        self._corner_tol        = math.radians(self.get_parameter('corner_turn_tolerance').value)
        self._corner_pause_s    = self.get_parameter('corner_swatch_pause_s').value
        self._corner_fault_s    = self.get_parameter('corner_fault_timeout_s').value
        self._corner_turn_kp = self.get_parameter('corner_turn_kp').value
        self._arc_max_ang    = self.get_parameter('arc_max_ang').value
        self._arc_finish_ang_floor = self.get_parameter('arc_finish_ang_floor').value
        self._arc_accum      = 0.0        # running integrated yaw for the active arc
        self._external_reacquire_dist  = self.get_parameter('external_reacquire_dist').value
        self._external_reacquire_speed = self.get_parameter('external_reacquire_speed').value
        self._external_turn_mode  = self.get_parameter('external_turn_mode').value
        self._external_arc_radius = self.get_parameter('external_arc_radius').value

        self._ext_phase             = None
        self._odom_turn_active      = False
        self._odom_turn_start_yaw   = None
        self._odom_crawl_active     = False
        self._odom_crawl_start_x    = None
        self._odom_crawl_start_y    = None
        self._right_seen_count      = 0
        self._first_follow_step_after_align = False


        self._crawl_dist = 0.0      # distance since last paint stroke during CORNER_APPROACH
        self._crawl_last_x = None
        self._crawl_last_y = None
        self._latest_crawl_remaining = -1.0   # from EnvStatus
        self._latest_crawl_backup = 0.0       # from EnvStatus: m to reverse before an INTERNAL turn

        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_yaw = 0.0

        self.wall_pose    = None
        self.robot_state  = 'IDLE'
        self._gate_prev_state = None   # drive-gate edge tracker (one-shot stop)
        self._last_command_id = -1       # gate for step reset on new command
        self._active_command_id = -1
        self.wall_status = None   # populated by wall_status_cb


        # self.wall_mode  = 'IDLE'
        self.corner_cmd = None
        self._corner_phase = None

        # Subscriptions
        self._corner_params = None
        self.create_subscription(CornerParams, '/corner_params', self._corner_params_cb, 10)
        self.create_subscription(WallPose, '/right_wall_pose', self.wall_pose_cb, 10)
        self.create_subscription(String, '/robot_state', self.robot_state_cb, 10)
        self.create_subscription(RobotCommand, '/robot_command', self._robot_command_cb, 10)
        self.create_subscription(WallStatus, '/wall_status', self.wall_status_cb, 10)
        self.create_subscription(Twist, '/corner_cmd_vel', self.corner_cmd_cb, 10)
        self.create_subscription(EnvStatus, '/env_status', self._env_status_cb, 10)

        # Raw wheel odom for EVERYTHING. The EKF (odom_filtered) is unreliable in sim
        # (diverges during the bump OUT arc's near-spin) and raw odom is sufficient on
        # the real robot too, so nothing consumes the EKF here.
        self.create_subscription(Odometry, '/paint_bot/odom', self._odom_cb, 10)
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/paint_bot/cmd_vel', 10)
        
        self.motion_status_pub = self.create_publisher(
            MotionStatus, '/motion_status', 10)

        self.create_timer(0.1, self.control_loop)  # 10Hz

        self.get_logger().info('WallMotionNode started')


      


    # --- CALLBACKS -----------------------------------------------------------
    def wall_status_cb(self, msg):
        self.wall_status = msg

    def _odom_cb(self, msg):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)

    def wall_pose_cb(self, msg):
        self.wall_pose = msg

    def _env_status_cb(self, msg: EnvStatus):
        self._latest_crawl_remaining = msg.crawl_remaining
        self._latest_crawl_backup = msg.crawl_backup


    def _corner_params_cb(self, msg: CornerParams):
         self._corner_params = msg if msg.corner_type else None

 

    def robot_state_cb(self, msg):
        prev_state = self.robot_state
        self.robot_state = msg.data

        if msg.data == 'IDLE' and prev_state != 'IDLE':
            # full reset - next run starts clean regardless of how prev ended
            self._corner_waiting_for_paint = False
            self._corner_done              = False
            self._corner_seg_index         = 0
            self._odom_turn_active         = False
            self._ext_phase                = None
            self._corner_phase             = None
            self._odom_crawl_active        = False
            self._odom_crawl_start_x       = None
            self._odom_crawl_start_y       = None
            self._right_seen_count         = 0
            self._crawl_last_x             = None
            self._crawl_last_y             = None
            self._step_done                = False
            self._step_start_x             = None
            self._step_start_y             = None
            self._strip_mode               = None
            self._align_phase              = 'DONE'
            self._align_start_yaw          = None
            self._align_strafe_angle_error = None
            self._first_follow_step_after_align = False   # NEW - full reset too
            self.get_logger().info('[CORNER] IDLE - wall_motion state cleared')

        if msg.data == 'WALL_ALIGN' and prev_state != 'WALL_ALIGN':
            self._align_phase = 'DONE'
            self._align_start_yaw = None
            self._align_strafe_angle_error = None

        # NEW - gain scheduling: first WALL_FOLLOW step after an align
        # should use strong (acquisition) gains. 
        if msg.data == 'WALL_FOLLOW' and prev_state == 'WALL_ALIGN':
            self._first_follow_step_after_align = True
            self.get_logger().info(
                '[GAIN] WALL_ALIGN -> WALL_FOLLOW - next step uses strong gains')

        # Resume next corner segment after intermediate paint
        # if prev_state == 'PAINT_EXECUTE' and msg.data == 'CORNER_LOGIC':
        #     self._corner_waiting_for_paint = False
        #     self._odom_turn_active = False
        #     self.get_logger().info('[CORNER] resume after paint - continuing next segment')      


        _seg_total = (self._bump_segments
                      if getattr(self, '_corner_type', None) == 'BUMPOUT'
                      else self._corner_segments)

        # BUMPOUT final-swatch completion for BOTH arcs. The final segment first
        # requests its paint stroke with step_complete only. When PAINT_EXECUTE
        # returns to the same CORNER_LOGIC item, finish that arc here with a clean
        # one-shot corner_turn_complete. This prevents completion leaking across
        # ARC1->ARC2 and gives ARC2 the same segmented-paint handshake as ARC1.
        if (prev_state == 'PAINT_EXECUTE'
                and msg.data == 'CORNER_LOGIC'
                and getattr(self, '_corner_type', None) == 'BUMPOUT'
                and self._corner_phase in ('ARC1', 'ARC2')
                and self._corner_waiting_for_paint
                and self._corner_seg_index >= _seg_total):
            phase = self._corner_phase
            self._corner_waiting_for_paint = False
            self._corner_turn_done = True
            self._corner_done      = True
            self.get_logger().info(
                f'[BUMP] {phase} final swatch done -> {phase} complete')
            self.publish_motion_status(corner_turn_complete=True)
            return

        if (prev_state == 'PAINT_EXECUTE'
                and msg.data == 'CORNER_LOGIC'
                and self._corner_waiting_for_paint
                and self._corner_seg_index > 0
                and self._corner_seg_index < _seg_total):
            self._corner_waiting_for_paint = False
            self._odom_turn_active = False
            self.get_logger().info('[CORNER] resume after swatch - continuing next segment')

   
    def corner_cmd_cb(self, msg):
        self.corner_cmd = msg    

  

    def _robot_command_cb(self, msg: RobotCommand):
        """
        Reset step state on new WALL_FOLLOW or STEP_REALIGN command.
        Reset corner state on new CORNER_LOGIC command.
        command_id gate ensures this fires once per FSM action, not per
        state-string tick - avoids race conditions on retained /robot_state.
        """

    
            
        self._active_command_id = msg.command_id   # <- every command, for status stamping


        if msg.state in ('WALL_FOLLOW', 'STEP_REALIGN','CORNER_APPROACH') and \
                msg.command_id != self._last_command_id:
            self._last_command_id = msg.command_id
            self._step_done    = False
            self._step_start_x = None
            self._step_start_y = None
            self._crawl_last_x = None   
            self._crawl_last_y = None   
            self.get_logger().info(
                f'[STEP] reset for new command id={msg.command_id} state={msg.state}')

        if msg.state == 'CORNER_LOGIC' and \
                msg.command_id != self._last_corner_command_id:
            self._last_corner_command_id = msg.command_id
            self._corner_phase = msg.phase   # 'TURN' or 'REACQUIRE'
            self.get_logger().info(f'[CORNER] cmd received phase=<{msg.phase}>')      
            # Only (re)initialise the corner on the TURN phase. REACQUIRE must
            # NOT reset - it continues into the crawl after the turn.

            if msg.phase in ('TURN', 'ARC1'):
                # TURN  : single-pivot corner (internal/external).
                # ARC1  : first (OUT) arc of a bump-out. Same init path - the
                #         bump's two arcs are both configured here off the one
                #         latched CornerParams; ARC2's (R2,-phi) is stashed for
                #         the ARC2 command to pick up without re-reading params.
                p = self._corner_params
                if p is not None and p.radian_diff != 0.0:
                    turn_dir    = +1 if p.radian_diff > 0 else -1
                    turn_angle  = abs(p.radian_diff)
                    corner_type = p.corner_type
                    turn_radius = p.turn_radius
                else:
                    turn_dir    = +1
                    turn_angle  = math.pi / 2
                    corner_type = 'INTERNAL'
                    turn_radius = 0
                    self.get_logger().warn('[CORNER] no corner_params latched - defaulting CCW 90deg')
                # stash ARC2 (BACK) for the bump - read by the ARC2 command below
                self._bump_arc2_radian_diff = getattr(p, 'arc2_radian_diff', 0.0) if p else 0.0
                self._bump_arc2_turn_radius = getattr(p, 'arc2_turn_radius', 0.0) if p else 0.0
                self._reset_corner_state(
                    turn_direction=turn_dir, turn_angle_rad=turn_angle,
                    corner_type=corner_type, turn_radius=turn_radius)

            elif msg.phase == 'ARC2':
                # Second (BACK) arc of the bump. Reconfigure the corner turn for
                # the opposite-sign swing at R2 - reuse the TURN init but from the
                # stashed ARC2 params. corner_type stays BUMPOUT so _do_corner
                # dispatches the arc branch.
                a2_rd = getattr(self, '_bump_arc2_radian_diff', 0.0)
                a2_rr = getattr(self, '_bump_arc2_turn_radius', 0.0)
                if a2_rd == 0.0:
                    self.get_logger().warn('[BUMP] ARC2 with no stashed params - skipping')
                self._reset_corner_state(
                    turn_direction=(+1 if a2_rd > 0 else -1),
                    turn_angle_rad=abs(a2_rd),
                    corner_type='BUMPOUT', turn_radius=a2_rr)
                self.get_logger().info(
                    f'[BUMP] ARC2 configured - {math.degrees(a2_rd):.1f}deg R2={a2_rr:.3f}m')

            # elif msg.phase == 'REACQUIRE':
            #     # fresh crawl state only - do not touch rotation/corner config
            #     self._odom_crawl_active = False
            #     self._odom_crawl_start_x = None
            #     self._odom_crawl_start_y = None
            #     self._right_seen_count   = 0
            #     self._corner_reacquire_done = False
            #     self._corner_done = False
            #     self.get_logger().info('[CORNER] REACQUIRE phase - crawl init')    


    # --- HELPERS -------------------------------------------------------------
   

    def publish_stop(self):
        self.cmd_pub.publish(Twist())
        self.get_logger().debug('[STOP] zero Twist published', throttle_duration_sec=0.0)

    def publish_motion_status(self, step_complete=False, corner_turn_complete=False,
                              corner_reacquired=False, distance_since_stroke=0.0):
        """Motion sequencing events for the FSM. Wall geometry/alignment lives on
        /wall_status (owned by wall_detection) - NOT published here."""
        msg = MotionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.command_id = self._active_command_id
        msg.step_complete = step_complete
        msg.corner_turn_complete = corner_turn_complete
        msg.corner_reacquired = corner_reacquired
        msg.distance_since_stroke = float(distance_since_stroke)
        self.motion_status_pub.publish(msg)
    # --- CONTROL LOOP --------------------------------------------------------

    # States where wall_motion actively drives the base. Anything else is a
    # non-driving state: publish exactly ONE zero on entry, then stay silent so
    # the latched cmd_vel is zero and whoever owns the base next (joystick in
    # IDLE, paint in PAINT_EXECUTE, nobody in HOLD/FAULT) isn't fought by a
    # stream of zeros. This is the single drive-gate - the one place that decides
    # whether wall_motion may touch /paint_bot/cmd_vel.
    _DRIVE_STATES = ('WALL_ALIGN', 'WALL_FOLLOW', 'STEP_REALIGN',
                     'CORNER_APPROACH', 'CORNER_LOGIC')

    def control_loop(self):
        # -- Drive-gate --------------------------------------------------------
        # Non-driving state (IDLE, PAINT_EXECUTE, HOLD, FAULT, anything unknown):
        # send one zero on the entry edge, then publish nothing. Edge-triggered,
        # not level - that's what stops the zero-flood that was stomping the
        # joystick and bleeding across state changes.
        if self.robot_state not in self._DRIVE_STATES:
            if self.robot_state != self._gate_prev_state:
                self.publish_stop()
                self.get_logger().info(
                    f'[GATE] {self.robot_state} - one stop sent, wall_motion silent')
            self._gate_prev_state = self.robot_state
            return
        self._gate_prev_state = self.robot_state

        if self.robot_state == 'WALL_FOLLOW' and self.wall_status is None:
                self.get_logger().warn('No wall_status yet', throttle_duration_sec=2.0)
                self.publish_stop()
                return

        if self.wall_pose is None or not self.wall_pose.valid:
            if self.robot_state == 'WALL_ALIGN' and self._align_phase not in ('ANGLE', 'DONE'):
                pass  # odom-driven phases, wall pose not needed
            elif self.robot_state == 'CORNER_LOGIC':
                pass  # odom-driven - wall pose irrelevant during turn
            elif self.robot_state == 'CORNER_APPROACH':
                pass  # wall-follow continues but wall_pose may briefly drop near external corner
            else:
                self.get_logger().warn('No valid wall pose', throttle_duration_sec=2.0)
                self.publish_stop()
                return

        
        
        dist        = self.wall_pose.distance if self.wall_pose and self.wall_pose.valid else 0.0
        angle_error = self.wall_pose.angle if self.wall_pose and self.wall_pose.valid else 0.0
        dist_error  = dist - self.target_dist

        cmd = Twist()
        if self.robot_state == 'WALL_ALIGN':
            # if self._align_phase != 'DONE' and self._align_phase != 'ANGLE':
            #     # mid-strafe sequence - always continue hard align regardless of dist_error
            #     aligned = False
            #     self._do_align_hard(cmd, dist_error, angle_error)
                
            # elif abs(dist_error) > self.strafe_thresh and self._align_phase == 'DONE':
            #     self._do_align_hard(cmd, dist_error, angle_error)
            # else:
                self._do_align_soft(cmd, angle_error)

        elif self.robot_state == 'WALL_FOLLOW':
            self._do_wall_follow_strip(cmd, angle_error, dist_error)


        elif self.robot_state == 'CORNER_APPROACH':

            # Completed this crawl command - hold until FSM issues next command.
            if self._step_done:
                self.publish_stop()
                return

            # INTERNAL overshoot: robot is closer to the front wall than the turn
            # trigger. Reverse (straight, slow) until env reports it is back at
            # the trigger, then fall through to the normal (zero-length) final
            # crawl so the FSM proceeds to the turn. No step_complete / paint
            # is emitted while reversing.
            backup = self._latest_crawl_backup
            if backup > self._backup_tol:
                cmd = Twist()
                cmd.linear.x = -self.crawl_speed
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.get_logger().info(
                    f'[BACKUP] reversing {backup:.3f}m to internal trigger',
                    throttle_duration_sec=0.5)
                return

            remaining = self._latest_crawl_remaining

            if remaining >= 0:
                # Latch target ONCE for this crawl command.
                if not self._odom_crawl_active:
                    self._crawl_target = min(remaining, self.move_step_size)

                done, travelled = self._do_odom_crawl(
                    self._crawl_target,
                    self.crawl_speed
                )

                if done:
                    self._step_done = True
                    self.publish_motion_status(
                        step_complete=True,
                        distance_since_stroke=travelled
                    )
            else:
                self.publish_stop()

            return

        elif self.robot_state == 'STEP_REALIGN':
            self._do_step_realign(cmd, angle_error, dist_error, realign=True)

        elif self.robot_state == 'CORNER_LOGIC':
            self._do_corner()
            return # _do_corner handles its own publish_motion_status and cmd_pub

        else:
            if self.robot_state == 'PAINT_EXECUTE':
                # wall geometry/alignment is published by wall_detection on
                # /wall_status; wall_motion only stops here.
                self.publish_stop()
            return

        self.cmd_pub.publish(cmd)

    # --- ALIGN MODE ----------------------------------------------------------

    def _do_align_soft(self, cmd, angle_error):
        """
        Soft align - angle correction only, no strafe.
        Called when dist_error is within strafe_thresh.
        Robot rotates in place until parallel, then stops.
        """
        if abs(angle_error) > self.angle_tol:
            cmd.linear.x  = 0.0
            cmd.angular.z = float(
                max(-self.max_ang,
                    min(self.max_ang, self.angle_kp * angle_error))
            )
            self.get_logger().info(
                f'[ALIGN-SOFT] angle_err={math.degrees(angle_error):.1f}deg',
                throttle_duration_sec=0.3)
        else:
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info(
                '[ALIGN-SOFT] done - angle within tolerance',
                throttle_duration_sec=1.0)

    def _do_align_hard(self, cmd, dist_error, angle_error):
        """
        Hard align - full strafe sequence for large distance errors.
        DIST_TURN1 -> DIST_DRIVE -> DIST_TURN2 -> ANGLE -> DONE
        All phases driven by EKF odometry.
        Called when dist_error > strafe_thresh.
        """
        if self._align_phase == 'DONE':
            self._align_phase              = 'DIST_TURN1'
            self._align_start_yaw          = None
            self._align_target_dist        = abs(dist_error)
            self._align_drive_direction = 1.0 if dist_error > 0 else -1.0

            self._align_strafe_angle_error = angle_error
            self.get_logger().info(
                f'[ALIGN-HARD] dist_err={dist_error:.3f}m '
                f'angle_err={math.degrees(angle_error):.1f}deg -> strafe sequence')

        if self._align_phase == 'DIST_TURN1':
            if self._align_start_yaw is None:
                self._align_start_yaw = self._odom_yaw

            target_delta = -(math.pi / 2.0 + self._align_strafe_angle_error)
            delta        = self._wrap_angle(self._odom_yaw - self._align_start_yaw)
            remaining    = target_delta - delta

            if abs(remaining) < math.radians(3.0):
                self._align_phase     = 'DIST_DRIVE'
                self._align_start_x   = self._odom_x
                self._align_start_y   = self._odom_y
                self._align_start_yaw = None
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info('[ALIGN-HARD] DIST_TURN1 done -> DIST_DRIVE')
            else:
                kp  = 1.2
                spd = max(0.15, min(self.max_ang, abs(remaining) * kp))
                cmd.linear.x  = 0.0
                cmd.angular.z = math.copysign(spd, remaining)
                self.get_logger().info(
                    f'[ALIGN-HARD] DIST_TURN1 remaining={math.degrees(remaining):.1f}deg',
                    throttle_duration_sec=0.3)

        elif self._align_phase == 'DIST_DRIVE':
            dx       = self._odom_x - self._align_start_x
            dy       = self._odom_y - self._align_start_y
            traveled = math.sqrt(dx*dx + dy*dy)
            # direction = 1.0 if dist_error > 0 else -1.0

            if traveled >= self._align_target_dist - 0.01:
                self._align_phase     = 'DIST_TURN2'
                self._align_start_yaw = None
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info('[ALIGN-HARD] DIST_DRIVE done -> DIST_TURN2')
            else:
                # cmd.linear.x  = direction * min(self.max_lin, 0.08)
                cmd.linear.x = self._align_drive_direction * min(self.max_lin, 0.08)
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f'[ALIGN-HARD] DIST_DRIVE traveled={traveled:.3f}m '
                    f'/ target={self._align_target_dist:.3f}m',
                    throttle_duration_sec=0.3)

        elif self._align_phase == 'DIST_TURN2':
            if self._align_start_yaw is None:
                self._align_start_yaw = self._odom_yaw

            target_delta = +math.pi / 2.0
            delta        = self._wrap_angle(self._odom_yaw - self._align_start_yaw)
            remaining    = target_delta - delta

            if abs(remaining) < math.radians(3.0):
                self._align_phase     = 'ANGLE'
                self._align_start_yaw = None
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info('[ALIGN-HARD] DIST_TURN2 done -> ANGLE')
            else:
                kp  = 1.2
                spd = max(0.15, min(self.max_ang, abs(remaining) * kp))
                cmd.linear.x  = 0.0
                cmd.angular.z = math.copysign(spd, remaining)
                self.get_logger().info(
                    f'[ALIGN-HARD] DIST_TURN2 remaining={math.degrees(remaining):.1f}deg',
                    throttle_duration_sec=0.3)

        elif self._align_phase == 'ANGLE':
            if abs(angle_error) > self.angle_tol:
                cmd.linear.x  = 0.0
                cmd.angular.z = float(
                    max(-self.max_ang,
                        min(self.max_ang, self.angle_kp * angle_error))
                )
                self.get_logger().info(
                    f'[ALIGN-HARD] ANGLE angle_err={math.degrees(angle_error):.1f}deg',
                    throttle_duration_sec=0.3)
            else:
                self._align_phase = 'DONE'
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info('[ALIGN-HARD] DONE - strafe + angle complete')
    # --- CRAWL MODE (CORNER_APPROACH) --------------------------------------------

    # def _do_crawl(self, cmd, dist_error, angle_error):
    #     """
    #     Continuous slow forward crawl - used only during CORNER_APPROACH.
    #     No distance target; runs until FSM transitions away (corner_preempt fires).
    #     Optional angle correction via follow_angle_correct param.
    #     """
    #     cmd.linear.x  = self.follow_speed
    #     cmd.angular.z = float(
    #         max(-self.max_ang,
    #             min(self.max_ang, -self.angle_kp * angle_error))
    #     ) if self.follow_angle_correct else 0.0
    #     self.get_logger().info(
    #         f'[CRAWL] dist_err={dist_error:.3f}m  '
    #         f'angle_err={math.degrees(angle_error):.1f}deg',
    #         throttle_duration_sec=0.5)

    def _do_crawl(self, cmd, dist_error, angle_error):
        """
        Continuous slow forward crawl - used only during CORNER_APPROACH.
        No distance target; runs until FSM transitions away (corner_preempt fires).
        Tracks distance since last paint stroke; signals step_complete at
        move_step_size (0.203m) so FSM can trigger a normal paint stroke
        mid-crawl, then resets and continues.
        """
        if self._crawl_last_x is None:
            self._crawl_last_x = self._odom_x
            self._crawl_last_y = self._odom_y

        traveled = math.hypot(self._odom_x - self._crawl_last_x,
                            self._odom_y - self._crawl_last_y)
        self._crawl_dist = traveled

        cmd.linear.x = (self.crawl_speed
                if self.robot_state == 'CORNER_APPROACH'
                else self.follow_speed)

        cmd.angular.z = float(
            max(-self.max_ang,
                min(self.max_ang, -self.angle_kp * angle_error))
        ) if self.follow_angle_correct else 0.0

        step_complete = False
        if traveled >= self.move_step_size:
            step_complete = True
            self._crawl_last_x = self._odom_x
            self._crawl_last_y = self._odom_y
            self._crawl_dist = 0.0
            self.publish_stop()
            self.get_logger().info(f'[CRAWL] strip complete - traveled={traveled:.3f}m')

        self.publish_motion_status(
            step_complete=step_complete,
            distance_since_stroke=self._crawl_dist
        )

        self.get_logger().info(
            f'[CRAWL] dist_err={dist_error:.3f}m  '
            f'angle_err={math.degrees(angle_error):.1f}deg  '
            f'crawl_dist={self._crawl_dist:.3f}m',
            throttle_duration_sec=0.5)


     # ─── WALL FOLLOW codes (drive straight)────────────────────────────────────────────
    

    """
    Following parts implement the "Stanley drive" originally described in 
    Gabriel M. Hoffmann et al. - for the DARPA challenge long ago
     “Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing.”  

        Stanley:
            dist_error_corrected = dist_error + LIDAR_OFFSET * sin(angle_error)
                lidar is 195mm ahead of turn axis; nose-toward-wall understates
                true centre distance, so we add back the offset contribution.
            psi_cmd      = atan(kd * dist_error_corrected)
                desired nose-toward-wall angle to close lateral error.
                positive = want to angle toward wall.
            heading_error = angle_error - psi_cmd
                how far actual heading is from desired, in detector convention.
            omega        = kt * heading_error → cmd.angular.z

    """

    """ wall detection angle normal output and error convention is opposite to ROS, so keep that in mind for command signals 
        Convention (verified against detector + ROS):
            dist_error  > 0  = too far from right wall
            angle_error > 0  = nose toward right wall (CW, detector convention)
            angular.z   > 0  = CCW = away from right wall (ROS convention)
            → these two conventions are opposite, so heading_error = angle_error - psi_cmd
    """

    def _select_gains(self, mode):
        """(dist_kp, heading_kp) for the given strip mode."""
        if mode == 'STRONG':
            return self.step_realign_dist_kp, self.step_realign_heading_kp
        return self.follow_dist_kp, self.follow_heading_kp

    # 

    def _do_wall_follow_strip(self, cmd, angle_error, dist_error):
        """
        WALL_FOLLOW strip execution.

        Gain mode (STRONG vs WEAK) is decided ONCE per strip, at arm time, from
        the corrected dist_error present at that instant, and latched for the
        whole 0.203m translation - no mid-strip re-decision, so gain switching
        never causes a steering discontinuity mid-strip.

        Arm-time decision:
            |dist_error_corrected| > escalation_threshold_m -> 'STRONG'
            otherwise                                       -> 'WEAK'

        The forward/align split below is NOT a latched decision - it falls out
        of `lateral` (odometry-derived, recomputed every tick). Once the robot
        stops translating for the align cleanup, lateral doesn't decrease, so
        the branch stays satisfied on its own; no phase flag needed.

        Sequence:
            1. translate 0.203m using Stanley (dist + heading correction) with
               the latched gain pair
            2. once lateral >= move_step_size: stop translating, and if
               angle_error still exceeds tolerance, rotate in place
               (_do_align_soft) to clean up residual heading error
            3. once angle_error is within tolerance -> step_complete

        FAULT (|dist_error| > dist_tolerance) is NOT decided here - that's
        WallFollowHandler.evaluate() in states.py, checked every tick straight
        off /wall_status, independent of whatever this function is doing.

        Convention (verified against detector + ROS):
            dist_error  > 0  = too far from right wall
            angle_error > 0  = nose toward right wall (CW, detector convention)
            angular.z   > 0  = CCW = away from right wall (ROS convention)
            -> heading_error = angle_error - psi_cmd
        """
        if self._step_done:
            self.publish_stop()
            return

        if self._step_start_x is None:
            self._step_start_x   = self._odom_x
            self._step_start_y   = self._odom_y
            self._step_start_yaw = self._odom_yaw   # lock travel heading for this step

            dist_error_corrected = self.wall_status.distance_error_corrected

            self._strip_mode = (
                'STRONG' if abs(dist_error_corrected) > self.escalation_threshold_m
                else 'WEAK'
            )

            self.get_logger().info(
                f'[STRIP] armed at ({self._step_start_x:.3f}, {self._step_start_y:.3f}) '
                f'mode={self._strip_mode} '
                f'dist_err={dist_error:+.3f}m dist_corr={dist_error_corrected:+.3f}m '
                f'angle_err={math.degrees(angle_error):+.1f}deg')
            return

        dx       = self._odom_x - self._step_start_x
        dy       = self._odom_y - self._step_start_y
        # project displacement onto the heading captured at arm time - "forward
        # along the current wall", not world +X. Odom-relative over one ~0.2 m
        # step, so absolute yaw drift cancels.
        _c, _s   = math.cos(self._step_start_yaw), math.sin(self._step_start_yaw)
        lateral  = dx * _c + dy * _s
        traveled = math.sqrt(dx * dx + dy * dy)     # logged only

        if lateral >= self.move_step_size:
            cmd.linear.x = 0.0
            cmd.angular.z      = 0.0
            self._step_done    = True
            self._step_start_x = None
            self._strip_mode   = None
            self.publish_stop()
            self.publish_motion_status(step_complete=True)
            self.get_logger().info(
                f'[STEP] complete - lateral={lateral:.3f}m traveled={traveled:.3f}m')
            return

        # still translating - Stanley, gain pair fixed by the arm-time mode
        cmd.linear.x = self.follow_speed
        kd, kt = self._select_gains(self._strip_mode)

        dist_error_corrected = self.wall_status.distance_error_corrected

        psi_cmd       = math.atan(kd * dist_error_corrected)
        if self.max_approach > 0.0:
            psi_cmd = max(-self.max_approach, min(self.max_approach, psi_cmd))
        heading_error = angle_error - psi_cmd

        limit = self.max_ang * (0.5 if self._strip_mode == 'STRONG' else 0.3)
        omega = kt * heading_error
        cmd.angular.z = max(-limit, min(limit, omega))

        self.get_logger().info(
            f'[STRIP] mode={self._strip_mode} '
            f'lateral={lateral:.3f}m/{self.move_step_size:.3f}m '
            f'dist_err={dist_error:+.3f}m dist_corr={dist_error_corrected:+.3f}m '
            f'angle_err={math.degrees(angle_error):+.1f}deg '
            f'psi_cmd={math.degrees(psi_cmd):+.1f}deg '
            f'h_err={math.degrees(heading_error):+.1f}deg '
            f'omega={cmd.angular.z:+.3f}',
            throttle_duration_sec=2)    

    # ─── STEP / STEP_REALIGN MODE ─────────────────────────────────────────────────
    # vestigeal for now.. do_follow is the used code..same math but different gains based on distance errors
    # keeping it around because i have a hard time deleting once useful codes :)
    def _do_step_realign(self, cmd, angle_error, dist_error, realign=False):
        """
        Move forward one strip width (move_step_size) using EKF odometry.
        realign=False : WALL_FOLLOW intra-block step — pure forward, no correction.
        realign=True  : STEP_REALIGN inter-block step — Stanley wall-distance correction.


        Termination: lateral progress along wall (odom x, forward = +X) >= move_step_size.
        Uses dx directly - not abs(dx) - so backward drift cannot falsely complete a step.
        """
        if self._step_done:
            self.publish_motion_status(step_complete=True)
            self.publish_stop()
            return

        if self._step_start_x is None:
            self._step_start_x   = self._odom_x
            self._step_start_y   = self._odom_y
            self._step_start_yaw = self._odom_yaw   # lock travel heading for this step
            self.get_logger().info(
                f'[STEP] armed at '
                f'({self._step_start_x:.3f}, {self._step_start_y:.3f}) '
                f'realign={realign}')
            return

        dx      = self._odom_x - self._step_start_x
        dy      = self._odom_y - self._step_start_y
        # project onto arm-time heading (odom-relative) - see _do_wall_follow_strip
        _c, _s   = math.cos(self._step_start_yaw), math.sin(self._step_start_yaw)
        lateral  = dx * _c + dy * _s
        traveled = math.sqrt(dx * dx + dy * dy)   # logged only

        if lateral >= self.move_step_size:
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            self._step_done    = True
            self._step_start_x = None
            self.publish_stop()
            self.publish_motion_status(step_complete=True)
            self.get_logger().info(
                f'[STEP] complete - lateral={lateral:.3f}m '
                f'traveled={traveled:.3f}m realign={realign}')
        else:
            cmd.linear.x = self.follow_speed

            if realign:
                # -- Stanley correction ----------------------------------------
                dist_error_corrected = (
                    dist_error
                    + self.lidar_offset_x * math.sin(angle_error)
                    + self.lidar_offset_y * (1.0 - math.cos(angle_error))
                )

                psi_cmd = math.atan(
                    self.step_realign_dist_kp * dist_error_corrected
                )

                heading_error = angle_error - psi_cmd

                limit = self.max_ang * 0.5
                omega = self.step_realign_heading_kp * heading_error
                cmd.angular.z = max(-limit, min(limit, omega))

                self.get_logger().info(
                    f'[REALIGN] '
                    f'dist_err={dist_error:+.3f}m '
                    f'dist_corr={dist_error_corrected:+.3f}m '
                    f'angle_err={math.degrees(angle_error):+.1f}deg '
                    f'psi_cmd={math.degrees(psi_cmd):+.1f}deg '
                    f'h_err={math.degrees(heading_error):+.1f}deg '
                    f'omega={cmd.angular.z:+.3f}',
                    throttle_duration_sec=0.2)

                # Sanity: expected lidar reading given current heading.
                # Cross-check against what wall_detection_node reports.
                expected_lidar = (
                    (dist_error + self.target_dist)
                    - self.lidar_offset_x * math.sin(angle_error)
                    - self.lidar_offset_y * (1.0 - math.cos(angle_error))
                )
                self.get_logger().debug(
                    f'[REALIGN] expected_lidar={expected_lidar:.3f}m',
                    throttle_duration_sec=0.5)

            else:
                # WALL_FOLLOW: heading hold only, no distance term.
                # Positive angle_error -> positive angular.z (matches WALL_ALIGN convention
                # confirmed in logs: +angle_err corrected by +angular.z).
                # Weak gain - just enough to resist the CW yaw drift.
                FOLLOW_ANGLE_KP = 0.8
                limit_follow    = self.max_ang * 0.3
                omega_follow    = FOLLOW_ANGLE_KP * angle_error
                cmd.angular.z   = max(-limit_follow, min(limit_follow, omega_follow))

            self.get_logger().info(
                f'[STEP] lateral={lateral:.3f}m / {self.move_step_size:.3f}m  '
                f'angle_err={math.degrees(angle_error):.1f}deg realign={realign}',
                throttle_duration_sec=0.3)



    # --- CORNER MODE ------------------------------------------------------------
    @staticmethod
    def _angle_diff(current: float, ref: float) -> float:
        d = current - ref
        while d >  math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d

        
    def _reset_corner_state(self, turn_direction: int = 1, 
                        turn_angle_rad: float = math.pi / 2,
                        corner_type: str = 'INTERNAL',
                        turn_radius: float = 0.0):
        self.publish_stop()
        self._corner_seg_index   = 0
        self._corner_start_yaw   = None
        self._int_origin_yaw     = None
        self._corner_pause_start = None
        self._corner_done        = False
        self._corner_turn_dir    = float(turn_direction)
        self._corner_turn_angle  = turn_angle_rad
        self._corner_type        = corner_type
        self._corner_turn_radius = turn_radius
        self._critical_start     = None

        self._ext_phase             = None
        self._odom_turn_active      = False
        self._odom_turn_start_yaw   = None
        self._odom_crawl_active     = False
        self._odom_crawl_start_x    = None
        self._odom_crawl_start_y    = None
        self._right_seen_count      = 0
        self._corner_waiting_for_paint = False
        self._corner_turn_done = False
        self._corner_reacquire_done = False

        self.get_logger().info(
            f'[CORNER] reset - type={corner_type} segments={self._corner_segments} '
            f'dir={"CCW" if turn_direction > 0 else "CW"} '
            f'angle={math.degrees(turn_angle_rad):.1f}deg '
            f'radius={turn_radius:.3f}m')

    def _do_odom_arc_turn(self, radian_diff: float, radius: float, speed: float) -> bool:
        """
        Constant-radius arc turn, odom-tracked by yaw delta (reuses the same
        completion criterion as _do_odom_turn - angle, not arc length).
        Call every tick. Latches start yaw on first call.

        radian_diff: signed segment angle target - positive CCW, negative CW.
        radius: arc radius in meters (sign-agnostic - direction comes from radian_diff).
        speed: linear speed along the arc (m/s).

        Drives both linear.x and angular.z so the robot's lidar traces a
        circular arc of the given radius, holding ~constant standoff to the
        wall through the turn (vs. _do_odom_turn's in-place pivot).

        Returns True when turn complete, False while turning.
        Caller must set self._odom_turn_active = False before starting a new turn.
        """
        # _odom_yaw is raw wheel-odom yaw (the node subscribes /paint_bot/odom). The
        # old _angle_diff(now,start) completion test aliased once the near-spin
        # overshot +/-180deg; the running integrator below fixes that.
        yaw = self._odom_yaw

        if not self._odom_turn_active:
            self._odom_turn_start_yaw = yaw      # repurposed as prev-yaw for the integrator
            self._arc_accum           = 0.0      # running integrated yaw for this arc
            self._odom_turn_active    = True
            self.get_logger().info(
                f'[ARC-TURN] start_yaw={math.degrees(yaw):.1f}deg '
                f'target={math.degrees(radian_diff):.1f}deg '
                f'radius={radius:.3f}m dir={"CCW" if radian_diff > 0 else "CW"}')

        # RUNNING INTEGRATOR: sum per-tick unwrapped yaw changes. Robust to overshoot
        # and to |sweep|>180deg - a single now-minus-start diff aliases once past +/-180deg.
        self._arc_accum += self._angle_diff(yaw, self._odom_turn_start_yaw)
        self._odom_turn_start_yaw = yaw
        remaining = radian_diff - self._arc_accum

        if abs(remaining) < self._corner_tol:
            self.publish_stop()
            self._odom_turn_active = False
            self.get_logger().info(
                f'[ARC-TURN] done - accum={math.degrees(self._arc_accum):.1f}deg '
                f'final_yaw={math.degrees(yaw):.1f}deg')
            return True

        abs_radius = max(abs(radius), 1e-3)  # guard against div-by-zero
        kp = self._corner_turn_kp
        # angular speed: magnitude only, capped by MAX so near-spins (|radius|~=0) don't
        # whip; sign applied when published. ramp down near completion.
        # Near the target (within ~8deg) drop the floor so the arc CREEPS the last few
        # degrees instead of blowing past the completion tolerance in one tick (the 0.10
        # floor at lin~=0.08 swept ~2.9deg/tick -> overshoot + reverse-arc correction that
        # pushed the landing outward). A little overshoot is fine.
        ang_floor = (self._arc_finish_ang_floor
                     if abs(remaining) < math.radians(8.0) else 0.10)
        ang_spd = max(ang_floor, min(speed / abs_radius, abs(remaining) * kp,
                                     self._arc_max_ang))
        # linear: magnitude |ang_spd|*|radius|, direction = SIGN OF RADIUS only (forward
        # for +radius; slight reverse for the near-spin OUT arc). Must NOT inherit the
        # turn direction, or a CW arc with +radius would drive backward.
        lin_spd = ang_spd * radius

        cmd = Twist()
        cmd.linear.x  = lin_spd
        cmd.angular.z = math.copysign(ang_spd, remaining)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'[ARC-TURN] accum={math.degrees(self._arc_accum):.1f}deg '
            f'remaining={math.degrees(remaining):.1f}deg '
            f'lin={lin_spd:+.3f} ang={cmd.angular.z:+.3f}',
            throttle_duration_sec=0.5)
        return False


    def _do_odom_turn(self, radian_diff: float, speed: float) -> bool:
        """
        Reusable odom-tracked yaw turn utility.
        Call every tick. Latches start yaw on first call.
        radian_diff: signed - positive CCW, negative CW.
        Returns True when turn complete, False while turning.
        Caller must set self._odom_turn_active = False before starting a new turn.
        """
        odom_yaw = self._odom_yaw

        if not self._odom_turn_active:
            self._odom_turn_start_yaw = odom_yaw
            self._odom_turn_active    = True
            self.get_logger().info(
                f'[TURN] start_yaw={math.degrees(odom_yaw):.1f}deg '
                f'target={math.degrees(radian_diff):.1f}deg '
                f'dir={"CCW" if radian_diff > 0 else "CW"}')

        delta     = self._angle_diff(odom_yaw, self._odom_turn_start_yaw)
        remaining = radian_diff - delta

        if abs(remaining) < self._corner_tol:
            self.publish_stop()
            self._odom_turn_active = False
            self.get_logger().info(
                f'[TURN] done - delta={math.degrees(delta):.1f}deg '
                f'final_yaw={math.degrees(odom_yaw):.1f}deg')
            return True

        kp  = self._corner_turn_kp
        spd = max(0.10, min(speed, abs(remaining) * kp))
        cmd = Twist()
        cmd.angular.z = math.copysign(spd, remaining)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'[TURN] delta={math.degrees(delta):.1f}deg '
            f'remaining={math.degrees(remaining):.1f}deg spd={spd:.2f}',
            throttle_duration_sec=0.5)
        return False


    def _do_odom_crawl(self, distance: float, speed: float) -> bool:
        """
        Reusable odom-tracked forward crawl utility.
        Call every tick. Latches start position on first call.
        Returns True when travelled >= distance, False while crawling.
        Caller must set self._odom_crawl_active = False before starting a new crawl.
        """
        if not self._odom_crawl_active:
            self._odom_crawl_start_x = self._odom_x
            self._odom_crawl_start_y = self._odom_y
            self._odom_crawl_active  = True
            self.get_logger().info(
                f'[ODOM CRAWL] start ({self._odom_crawl_start_x:.3f}, '
                f'{self._odom_crawl_start_y:.3f}) target={distance:.3f}m spd={speed:.2f}')

        dx        = self._odom_x - self._odom_crawl_start_x
        dy        = self._odom_y - self._odom_crawl_start_y
        travelled = math.sqrt(dx * dx + dy * dy)

        if travelled >= distance:
            self.publish_stop()
            self._odom_crawl_active = False
            self.get_logger().info(f'[ODOM CRAWL] done - travelled={travelled:.3f}m')
            return True, travelled

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'[ODOM CRAWL] forward {travelled:.3f}/{distance:.3f}m',
            throttle_duration_sec=0.5)
        return False, travelled


    def _do_corner(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # Do not republish completion events from a generic done latch.
        # TURN and REACQUIRE publish their own one-shot MotionStatus events at
        # the exact moment they complete. Re-publishing here can leak stale
        # corner_turn_complete=True into the next TURN command.
        if self._corner_phase == 'TURN' and self._corner_turn_done:
            self.publish_stop()
            return

        if self._corner_phase == 'REACQUIRE' and self._corner_reacquire_done:
            self.publish_stop()
            return

        if self._corner_done:
            self.publish_stop()
            return

        if self._corner_type == 'INTERNAL':
            self._do_internal_corner(now)
        elif self._corner_type == 'EXTERNAL':
            if self._corner_phase == 'REACQUIRE':
                self._do_external_reacquire(now)
            else:
                self._do_external_turn(now)
        elif self._corner_type == 'BUMPOUT':
            self._do_bump_arc(now)

    def _do_bump_arc(self, now: float):
        """One bump-out arc per CORNER_LOGIC item, selected by self._corner_phase:

          ARC1 : OUT arc  - CCW, +phi, axis radius R1-SPRAY_D (negative = near-spin).
          ARC2 : BACK arc - CW,  -phi, axis radius R2+SPRAY_D (positive).

        _reset_corner_state has already loaded this arc's signed swing into
        (_corner_turn_angle, _corner_turn_dir) and its TIP radius into
        _corner_turn_radius. We convert tip->axis radius here (the wheels drive the
        turn axis, the schedule radii are sprayhead-tip radii) and run one
        _do_odom_arc_turn. On completion signal corner_turn_complete so the FSM
        advances the queue (ARC1->ARC2, ARC2->WALL_ALIGN).
        """
        SPRAY_D = 0.60   # turn-axis -> sprayhead tip (matches bumpout planner SPRAY_D_ORIGIN)

        # Wait while FSM is doing the inter-segment paint stroke (same handshake as
        # the internal/external segmented turns).
        if self._corner_waiting_for_paint:
            self.publish_stop()
            return

        # ARC1 and ARC2 use the same segmented-paint handshake. Each arc is split
        # into bump_turn_segments swatches; every segment paints, including the final
        # one. Arc completion is reported only after the final paint returns.
        n_seg = max(1, int(self._bump_segments))
        # Signed swing of ONE segment: full arc swing / n_seg. _corner_turn_angle is the
        # magnitude for this arc (ARC1 or ARC2), _corner_turn_dir its sign.
        seg_angle = (self._corner_turn_angle / n_seg) * self._corner_turn_dir

        # tip radius -> axis radius. OUT (CCW, +): R1 - SPRAY_D (can be < 0 -> near-spin,
        # handled by the signed-lin_spd fix in _do_odom_arc_turn). BACK (CW, -): R2 + SPRAY_D.
        if self._corner_turn_dir > 0:
            axis_radius = self._corner_turn_radius - SPRAY_D
        else:
            axis_radius = self._corner_turn_radius + SPRAY_D

        turn_done = self._do_odom_arc_turn(seg_angle, axis_radius, self._corner_speed)
        if not turn_done:
            return

        self._corner_seg_index += 1
        self.get_logger().info(
            f'[BUMP] {self._corner_phase} seg {self._corner_seg_index}/{n_seg} done - '
            f'seg_swing={math.degrees(seg_angle):.1f}deg axis_R={axis_radius:+.3f}m')

        # FINAL segment -> request the final paint stroke and HOLD this same arc.
        # Do NOT publish corner_turn_complete here. _state_cb emits it only after
        # PAINT_EXECUTE returns, so the FSM cannot carry a stale completion into
        # the next bump arc / WALL_ALIGN.
        if self._corner_seg_index >= n_seg:
            self.publish_stop()
            self._corner_waiting_for_paint = True
            self._odom_turn_active = False
            self.get_logger().info(
                f'[BUMP] {self._corner_phase} final segment done -> final paint stroke')
            self.publish_motion_status(
                step_complete=True,
                corner_turn_complete=False)
            return

        # INTERMEDIATE segment -> request a paint stroke and hold until FSM resumes.
        self._corner_waiting_for_paint = True
        self._odom_turn_active = False
        self.publish_stop()
        self.get_logger().info(
            f'[BUMP] {self._corner_phase} seg {self._corner_seg_index} done -> paint stroke')
        self.publish_motion_status(step_complete=True)

    def _do_internal_corner(self, now: float):

        self.get_logger().info(
            f'[CORNER-DBG] waiting_for_paint={self._corner_waiting_for_paint} '
            f'turn_active={self._odom_turn_active} '
            f'seg={self._corner_seg_index}/{self._corner_segments} '
            f'done={self._corner_done} '
            f'odom_yaw={math.degrees(self._odom_yaw):.1f}deg')

        if self._corner_waiting_for_paint:
            self.publish_stop()
            return

        seg_angle = (self._corner_turn_angle / self._corner_segments) * self._corner_turn_dir
        # Absolute targets: the base keeps rotating a little while the gantry strokes between
        # segments, so a purely relative second segment would over-turn by that drift. Aim each
        # segment at origin + (index+1)*seg_angle instead.
        if not self._odom_turn_active:
            if self._int_origin_yaw is None:
                self._int_origin_yaw = self._odom_yaw
            expected = self._int_origin_yaw + self._corner_seg_index * seg_angle
            drift = self._angle_diff(self._odom_yaw, expected)
            self._int_seg_target = seg_angle - drift
            if abs(drift) > math.radians(2.0):
                self.get_logger().info(
                    f'[CORNER-INT] seg {self._corner_seg_index + 1}: drift {math.degrees(drift):+.1f}deg '
                    f'since last segment, compensating -> turn {math.degrees(self._int_seg_target):.1f}deg')
        turn_done = self._do_odom_turn(self._int_seg_target, self._corner_speed)

        if turn_done:
            self._corner_seg_index += 1
            self.get_logger().info(
                f'[CORNER-INT] seg {self._corner_seg_index}/{self._corner_segments} done')

            if self._corner_seg_index >= self._corner_segments:
                self._corner_done = True
                self.get_logger().info('[CORNER-INT] all segments done -> paint then WALL_ALIGN')
                self.publish_motion_status(
                    step_complete=True,
                    corner_turn_complete=True
                )
            else:
                self._corner_waiting_for_paint = True
                self.publish_motion_status(step_complete=True)
                self.get_logger().info('[CORNER-INT] seg done -> paint stroke')    

   
    def _do_external_turn(self, now: float):
            """Segmented external corner turn.

            Each segment uses either an arc or in-place rotation.
            Intermediate segments request a paint stroke.
            Final segment reports turn complete.
            """

            seg_angle = (
                self._corner_turn_angle /
                self._corner_segments
            ) * self._corner_turn_dir

            if self._ext_phase is None:
                self._ext_phase = 'ROTATION'
                self._odom_turn_active = False
                self.get_logger().info(
                    f'[CORNER-EXT] entering ROTATION - '
                    f'dir={"CCW" if self._corner_turn_dir > 0 else "CW"} '
                    f'angle={math.degrees(self._corner_turn_angle):.1f}deg '
                    f'segments={self._corner_segments}'
                )

            # Wait while FSM is doing the inter-segment paint.
            if self._corner_waiting_for_paint:
                self.publish_stop()
                return

            if self._external_turn_mode == 'arc':
                arc_R = (self._corner_turn_radius
                         if self._corner_turn_radius > 0.0
                         else self._external_arc_radius)   # fallback only if env sent 0
                turn_done = self._do_odom_arc_turn(
                    seg_angle,
                    arc_R,
                    self._corner_speed
                )
            else:
                turn_done = self._do_odom_turn(
                    seg_angle,
                    self._corner_speed
                )
                    
            if not turn_done:
                return

            self._corner_seg_index += 1

            self.get_logger().info(
                f'[CORNER-EXT] seg '
                f'{self._corner_seg_index}/{self._corner_segments} done'
            )

            # --------------------------------------------------
            # FINAL segment
            # --------------------------------------------------
            if self._corner_seg_index >= self._corner_segments:
                self._ext_phase = None
                self._corner_turn_done = True
                self._corner_done = True

                self.publish_stop()

                self.get_logger().info(
                    '[CORNER-EXT] all segments done -> '
                    'paint then TURN complete'
                )

                self.publish_motion_status(
                    step_complete=True,
                    corner_turn_complete=True
                )
                return

            # --------------------------------------------------
            # INTERMEDIATE segment -> paint
            # --------------------------------------------------
            self._corner_waiting_for_paint = True
            self._odom_turn_active = False

            self.get_logger().info(
                f'[CORNER-EXT] seg {self._corner_seg_index} done '
                f'-> paint stroke'
            )

            self.publish_motion_status(
                step_complete=True
            )

    # def _do_external_turn(self, now: float):
    #     """External corner ROTATION only - segmented yaw turn.
    #     Signals corner_turn_complete when all segments done (-> FSM advances to REACQUIRE)."""

    #                 # arc mode = one continuous 90deg wrap; spin mode = split into segments
    #     if self._external_turn_mode == 'arc':
    #             seg_angle = self._corner_turn_angle * self._corner_turn_dir
    #     else:
    #             seg_angle = (self._corner_turn_angle / self._corner_segments) * self._corner_turn_dir


    #     if self._ext_phase is None:
    #         self._ext_phase        = 'ROTATION'
    #         self._odom_turn_active = False
    #         self.get_logger().info(
    #             f'[CORNER-EXT] entering ROTATION - '
    #             f'dir={"CCW" if self._corner_turn_dir > 0 else "CW"} '
    #             f'angle={math.degrees(self._corner_turn_angle):.1f}deg')

    #     # swatch pause between segments
    #     # if self._corner_pause_start is not None:
    #     #     elapsed = now - self._corner_pause_start
    #     #     if elapsed < self._corner_pause_s:
    #     #         self.publish_stop()
    #     #         return
    #     #     else:
    #     #         self._corner_pause_start = None
    #     #         self._odom_turn_active   = False
    #     #         self.get_logger().info(
    #     #             f'[CORNER-EXT] swatch done -> seg {self._corner_seg_index + 1}/'
    #     #             f'{self._corner_segments}')

    #     if self._external_turn_mode == 'arc':
    #         # Use the REACTIVE radius env chose (carried in CornerParams.turn_radius ->
    #         # _corner_turn_radius at _reset_corner_state), not the fixed param. env
    #         # picks Re = clamp(0.904 + v_axis_x, 0.6, 0.8) from where the vertex landed,
    #         # so a fresh corner can want 0.73, 0.6, etc. Fall back to the param only if
    #         # no radius was latched (<=0).
    #         arc_R = self._corner_turn_radius if self._corner_turn_radius > 0.0 \
    #                 else self._external_arc_radius
    #         turn_done = self._do_odom_arc_turn(seg_angle, arc_R, self._corner_speed)
    #     else:
    #             turn_done = self._do_odom_turn(seg_angle, self._corner_speed)
            
    #     if turn_done:
    #         self._corner_seg_index += 1
    #         self.get_logger().info(
    #             f'[CORNER-EXT] seg {self._corner_seg_index}/{self._corner_segments} done')

    #         if self._corner_seg_index >= self._corner_segments:
    #             # rotation complete -> signal TURN item complete once.
    #             # Keep a local done latch so the next control tick does not
    #             # start a fresh rotation before FSM switches to REACQUIRE.
    #             self._ext_phase = None
    #             self._corner_turn_done = True
    #             self.publish_stop()
    #             self.get_logger().info('[CORNER-EXT] rotation complete -> TURN item done')
    #             self.publish_motion_status(corner_turn_complete=True)

    #         else:
    #             if self._corner_segments > 1:
    #                 self._corner_pause_start = now
    #                 self.get_logger().info('[CORNER-EXT] starting swatch pause')
    #                  # paint one swatch after the 1st segment only

    #                 if self._corner_seg_index == 1:
    #                     self.get_logger().info(
    #                         f'[DBG-TURN] seg {self._corner_seg_index} done -> swatch pause, '
    #                         f'firing step_complete for paint')
    #                     self.publish_motion_status(step_complete=True) 
    #             else:
    #                 self._odom_turn_active = False
    #     return

    def _do_external_reacquire(self, now: float):
        """External corner REACQUIRE - forward crawl until the right wall is
        stably back, or the crawl limit is hit. Completes as soon as the wall is
        reacquired (no forced minimum)."""
        crawl_done, travelled = self._do_odom_crawl(
            self._external_reacquire_dist,
            self._external_reacquire_speed)

        wall_fresh = False
        if self.wall_pose is not None and self.wall_pose.valid:
            try:
                age = (self.get_clock().now() -
                       rclpy.time.Time.from_msg(self.wall_pose.header.stamp)
                       ).nanoseconds * 1e-9
                wall_fresh = age < 0.5
            except Exception:
                wall_fresh = False

        if wall_fresh:
            self._right_seen_count += 1
        else:
            self._right_seen_count = 0

        wall_at_target = (
            self.wall_pose is not None and
            self.wall_pose.valid and
            abs(self.wall_pose.distance - self.target_dist) < 0.3
        )

        right_wall_back = (
            wall_fresh and wall_at_target and self._right_seen_count >= 20
        )

        # complete as soon as wall is stably back (no 0.5m minimum)
        if right_wall_back:
            self.publish_stop()
            self._corner_reacquire_done = True
            self._corner_done = True
            self.get_logger().info(
                f'[CORNER-EXT] wall reacquired - travelled={travelled:.3f}m '
                f'count={self._right_seen_count}')
            self.publish_motion_status(corner_reacquired=True)
            return

        # crawl limit reached without reacquire -> complete anyway (ALIGN will fix)
        if crawl_done:
            self.publish_stop()
            self._corner_reacquire_done = True
            self._corner_done = True
            self.get_logger().warn(
                f'[CORNER-EXT] reacquire limit {travelled:.3f}m - completing, ALIGN to recover')
            self.publish_motion_status(corner_reacquired=True)
            return
            
    @staticmethod
    def _wrap_angle(angle: float) -> float:
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = WallMotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()