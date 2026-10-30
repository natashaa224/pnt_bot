# VRai
# 04/21/26  (drop 1 — queue-driven normal mode)
from enum import Enum, auto
from painting_robot_msgs.msg import CollisionWarning, EnvStatus

SAFETY_THRESH=0.60 #  for turn recovery safety margins

class RobotState(Enum):
    IDLE            = auto()
    SYSTEM_CHECK    = auto()
    WALL_ALIGN      = auto()
    WALL_FOLLOW     = auto()
    STEP_REALIGN    = auto()
    CORNER_LOGIC    = auto()
    CORNER_APPROACH = auto()
    FAULT           = auto()
    PAINT_EXECUTE   = auto()
    HOLD            = auto()   # recoverable pause (operator PAUSE / safety SAFETY_HOLD)
    # ── Sentinel ──────────────────────────────────────────────────────────────
    # NOT a real state. A handler returns this to mean "this queue item is done —
    # FSM, advance the queue and route to the next item's state." The executor in
    # state_machine_node intercepts it before any transition; it is never entered.
    ITEM_COMPLETE   = auto()


class StateHandler:
    def __init__(self, node):
        self.node = node

    def on_enter(self):
        pass

    def on_exit(self):
        pass

    def evaluate(self, context) -> 'RobotState':
        raise NotImplementedError

    def post_paint(self, context) -> 'RobotState':
        """Where to go after a paint stroke triggered by THIS handler's item.
        In queue mode, one-shot items return ITEM_COMPLETE here — the paint was
        the item's work, so finishing paint finishes the item."""
        return RobotState.ITEM_COMPLETE


class IdleHandler(StateHandler):
    def on_enter(self):
        self.node.get_logger().info('[FSM] Entering IDLE')

    def evaluate(self, context) -> RobotState:
        if context.get('start_requested', False):
            return RobotState.SYSTEM_CHECK
        return RobotState.IDLE


class SystemCheckHandler(StateHandler):
    def on_enter(self):
        self.node.get_logger().info('[FSM] Entering SYSTEM_CHECK')
        self.node.fsm_context['homing_complete'] = False

    def evaluate(self, context) -> RobotState:
        # TEST BYPASS: homing node not running in sim test → skip the homing gate.
        if context.get('homing_complete', False) or getattr(self.node, '_skip_homing', False):
            # Homing done — load the first plan and start the queue.
            self.node.queue.start()
            first = self.node.queue.current_item()
            return first.state if first else RobotState.WALL_ALIGN
        return RobotState.SYSTEM_CHECK


# class WallAlignHandler(StateHandler):
#     """Queue item: align to wall, then paint one strip, then item complete.
#     One-shot per item — align -> paint -> ITEM_COMPLETE."""

#     def on_enter(self):
#         self.node.get_logger().info('[FSM] Entering WALL_ALIGN')
#         self._entry_ticks = 5           # 0.5s settle before trusting wall_status

#     def evaluate(self, context) -> RobotState:
#         if self._entry_ticks > 0:
#             self._entry_ticks -= 1
#             return RobotState.WALL_ALIGN
#         if context.get('wall_lost', False):
#             return RobotState.FAULT

#         item = self.node.queue.current_item()

#         wall_status = context.get('wall_status')
#         if wall_status is not None and wall_status.aligned:
#             if item and item.paints:
#                 context['_paint_caller'] = RobotState.WALL_ALIGN
#                 return RobotState.PAINT_EXECUTE
#             # no paint on this item — item done
#             return RobotState.ITEM_COMPLETE
#         return RobotState.WALL_ALIGN

#     def post_paint(self, context) -> RobotState:
#         # paint was the last work of this item
#         return RobotState.ITEM_COMPLETE

class WallAlignHandler(StateHandler):
    def on_enter(self):
        self.node.get_logger().info('[FSM] Entering WALL_ALIGN')
        self._entry_ticks = 5

    def evaluate(self, context) -> RobotState:
        if self._entry_ticks > 0:
            self._entry_ticks -= 1
            return RobotState.WALL_ALIGN
        # wall_lost is reported from fsm_loop (not here); just hold if set.
        if context.get('wall_lost', False):
            return RobotState.WALL_ALIGN

        wall_status = context.get('wall_status')

        # No valid wall to align to (e.g. landed at the next external corner after
        # an internal turn, right wall gone). Don't sit forever: report WALL_LOST so
        # safety raises SAFETY_HOLD -> FSM HOLD, where env's external-corner search
        # runs and (on a real corner) auto-resumes. Message-staleness never trips
        # here because wall_detection keeps publishing "NOT DETECTED", so VALIDITY
        # is the right trigger, not the fsm_loop timeout. Placeholder for a future
        # no_wall_align in wall_motion — for now we just hold and let recovery run.
        if wall_status is None or not getattr(wall_status, 'wall_valid', True):
            self.node.get_logger().warn(
                '[FSM] WALL_ALIGN — no valid wall to align to, reporting WALL_LOST')
            self.node.report_fault('WALL_LOST')
            return RobotState.WALL_ALIGN
        self.node.clear_fault('WALL_LOST')

        # Mid-recipe alignment: after an external corner turn the robot is not yet
        # back in its wall lane when an INTERNAL corner is already latched ahead
        # (external -> internal back-to-back, e.g. the end of a bump-out). The
        # following internal approach + turn restores the lane distance, so the lane
        # distance check does not apply to this intermediate align - holding here
        # on a ~0.1m error would fault the run before the internal turn can fix it.
        item_now = self.node.queue.current_item()
        mid_recipe = (
            item_now is not None
            and getattr(item_now, 'label', '') == 'corner_align+paint'
            and context.get('latched_corner_type') == EnvStatus.INTERNAL
        )
        if mid_recipe:
            self.node.clear_fault('WALL_DIST')

        # Post-EXTERNAL-corner recovery: no internal corner follows (that case is mid_recipe
        # above), so the robot is simply back on a straight wall but possibly too far out.
        # Open a recovery window instead of faulting: align on ANGLE only, skip the stroke while
        # the distance error is above dist_tolerance, and let WALL_FOLLOW (strong gains) close it.
        # NOTE: env retires the corner latch ("post-turn WALL_ALIGN ... retiring spent latch") as soon
        # as this step is entered, so latched_corner_type is already NONE here - do not test it.
        # The queue label identifies the step; the INTERNAL-ahead case is mid_recipe above.
        post_ext = (
            item_now is not None
            and getattr(item_now, 'label', '') in ('corner_align+paint', 'bump_align', 'corner_align')
            and not mid_recipe
            and self.node.post_corner_dist_tol > self.node.dist_tolerance
        )
        if post_ext and not context.get('_dist_recovery', False):
            context['_dist_recovery'] = True
            context['_dist_recovery_steps'] = 0
        recovery = bool(context.get('_dist_recovery', False))
        dist_err = abs(wall_status.distance_error_corrected) if wall_status is not None else 0.0
        tol_eff = self.node.post_corner_dist_tol if recovery else self.node.dist_tolerance

        # Distance-error exceedance: REPORT it (safety_node decides HOLD vs
        # escalate) and HOLD in place — do NOT self-transition to FAULT, or
        # safety stays NOMINAL and RESET can't recover the FSM.
        if not mid_recipe and wall_status is not None and dist_err > tol_eff:
            self.node.get_logger().error(
                f'[FSM] WALL_ALIGN dist error '
                f'{wall_status.distance_error_corrected:+.3f}m > tol '
                f'{tol_eff:.3f}m — reporting WALL_DIST')
            self.node.report_fault('WALL_DIST')
            return RobotState.WALL_ALIGN
        self.node.clear_fault('WALL_DIST')

        item = self.node.queue.current_item()

        # In recovery the detector's `aligned` also demands the distance, so use the angle alone.
        angle_ok = (recovery and wall_status is not None
                    and abs(wall_status.angle_error) <= self.node.post_corner_angle_tol)
        if wall_status is not None and (wall_status.aligned or angle_ok):
            if recovery and dist_err > self.node.dist_tolerance:
                self.node.get_logger().warn(
                    f'[FSM] post-corner recovery: distance error {wall_status.distance_error_corrected:+.3f}m '
                    f'> {self.node.dist_tolerance:.2f}m - stroke SKIPPED, wall-follow will close it')
                return RobotState.ITEM_COMPLETE
            # No stroke between a post-turn re-align and the following wall-follow: the corner's own
            # per-segment strokes already covered the turn, and the next strip is painted after the
            # first follow step. This covers the corner recipe's own align item AND the first plain
            # 'align+paint' item of the normal block that follows it.
            label = getattr(item, 'label', '') if item else ''
            if label in ('corner_align+paint', 'corner_align', 'bump_align'):
                context['_skip_next_align_stroke'] = True
                self.node.get_logger().info(
                    f'[FSM] post-turn re-align done ({label}) - no stroke, straight to wall-follow')
                return RobotState.ITEM_COMPLETE
            if label == 'align+paint' and context.pop('_skip_next_align_stroke', False):
                self.node.get_logger().info(
                    '[FSM] first align after a corner - stroke suppressed, straight to wall-follow')
                return RobotState.ITEM_COMPLETE
            if item and item.paints:
                # Skip a stroke that would repaint the same wall (robot has not
                # translated/rotated enough since the previous stroke).
                if self.node.stroke_is_redundant():
                    return RobotState.ITEM_COMPLETE
                context['_paint_caller'] = RobotState.WALL_ALIGN
                return RobotState.PAINT_EXECUTE
            return RobotState.ITEM_COMPLETE
        return RobotState.WALL_ALIGN

    def post_paint(self, context) -> RobotState:
        return RobotState.ITEM_COMPLETE

class WallFollowHandler(StateHandler):
    """Queue item: step forward one strip, paint it, item complete.
    One-shot per item — step -> paint -> ITEM_COMPLETE.

    Also owns the hard safety stop: if dist_error ever exceeds dist_tolerance,
    no amount of strong-gain correction is going to fix it on its own, so we
    FAULT immediately rather than keep painting through a bad strip."""

    def on_enter(self):
        self.node.get_logger().info('[FSM] Entering WALL_FOLLOW')

    def evaluate(self, context) -> RobotState:
        # wall_lost is reported from fsm_loop; just hold if set.
        if context.get('wall_lost', False):
            return RobotState.WALL_FOLLOW

        wall_status = context.get('wall_status')
        # Post-external-corner recovery window (see WallAlignHandler): accept a bigger distance
        # error for a few strips, skip strokes until it is back inside dist_tolerance.
        recovery = bool(context.get('_dist_recovery', False))
        dist_err = abs(wall_status.distance_error_corrected) if wall_status is not None else 0.0
        # Hysteresis: leave recovery only well inside the tolerance. Leaving at exactly dist_tolerance made a
        # reading of 0.099 m end it and the next 0.101 m reading fault (seen on the robot after a bump-out).
        exit_thr = self.node.dist_tolerance * self.node.post_corner_exit_frac
        if (recovery and wall_status is not None and getattr(wall_status, 'wall_valid', True)
                and dist_err <= exit_thr):
            self.node.get_logger().info(
                f'[FSM] post-corner recovery complete - distance {dist_err:.3f}m <= {exit_thr:.3f}m')
            context['_dist_recovery'] = False
            recovery = False
        tol_eff = self.node.post_corner_dist_tol if recovery else self.node.dist_tolerance
        # Distance-error exceedance: report + hold (see WallAlignHandler).
        if wall_status is not None and dist_err > tol_eff:
            self.node.get_logger().error(
                f'[FSM] WALL_FOLLOW dist error '
                f'{wall_status.distance_error_corrected:+.3f}m > tol '
                f'{tol_eff:.3f}m — reporting WALL_DIST')
            self.node.report_fault('WALL_DIST')
            return RobotState.WALL_FOLLOW
        self.node.clear_fault('WALL_DIST')

        if context.get('corner_preempt', False) or context.get('_corner_active', False):
            context['step_complete'] = False
            context['_dist_recovery'] = False        # a new corner recipe takes over
            return RobotState.ITEM_COMPLETE

        item = self.node.queue.current_item()

        if context.get('step_complete', False):
            context['step_complete'] = False
            if recovery:
                context['_dist_recovery_steps'] = context.get('_dist_recovery_steps', 0) + 1
                if context['_dist_recovery_steps'] >= self.node.post_corner_max_steps and dist_err > self.node.dist_tolerance:
                    self.node.get_logger().error(
                        f'[FSM] post-corner recovery did not close the distance error in '
                        f'{self.node.post_corner_max_steps} strips - reporting WALL_DIST')
                    context['_dist_recovery'] = False
                    self.node.report_fault('WALL_DIST')
                    return RobotState.WALL_FOLLOW
                if dist_err > self.node.dist_tolerance:
                    self.node.get_logger().warn(
                        f'[FSM] post-corner recovery: distance error {wall_status.distance_error_corrected:+.3f}m '
                        f'> {self.node.dist_tolerance:.2f}m - stroke SKIPPED')
                    return RobotState.ITEM_COMPLETE
            if item and item.paints:
                context['_paint_caller'] = RobotState.WALL_FOLLOW
                return RobotState.PAINT_EXECUTE
            return RobotState.ITEM_COMPLETE
        return RobotState.WALL_FOLLOW

    def post_paint(self, context) -> RobotState:
        return RobotState.ITEM_COMPLETE

class StepRealignHandler(StateHandler):
    """Queue item: reposition to next block (forward step + angle/dist correction).
    No paint. step_complete -> ITEM_COMPLETE.

    NOTE (post-external cleanup): this is _do_step_realign(realign=True) —
    same wall_motion code as WALL_FOLLOW with realign flag. Slated to fuse into
    WALL_FOLLOW as a phase once external corners prove out. Kept separate for now."""

    def on_enter(self):
        self.node.get_logger().info('[FSM] Entering STEP_REALIGN')
        self.node.fsm_context['step_complete'] = False

    def evaluate(self, context) -> RobotState:
        # wall_lost is reported from fsm_loop; just hold if set (no self-FAULT).
        if context.get('wall_lost', False):
            return RobotState.STEP_REALIGN

        if context.get('corner_preempt', False) or context.get('_corner_active', False):
                    context['step_complete'] = False
                    return RobotState.ITEM_COMPLETE
        
        if context.get('step_complete', False):
            context['step_complete'] = False
            return RobotState.ITEM_COMPLETE
        return RobotState.STEP_REALIGN



class PaintExecuteHandler(StateHandler):
    """Runs one stroke, then routes back to the caller item's handler via
    post_paint. Paint interleaves WITHIN a queue item — never advances the queue."""

    def on_enter(self):
        self.node.get_logger().info('[FSM] Entering PAINT_EXECUTE')
        self.node.fsm_context['stroke_complete'] = False
        self.node.note_stroke_started()

    def evaluate(self, context) -> RobotState:
        if context.get('stroke_complete', False):
            context['stroke_complete'] = False
            caller = context.get('_paint_caller', RobotState.WALL_FOLLOW)
            context['_paint_caller'] = None
            handler = self.node.handlers.get(caller)
            if handler and hasattr(handler, 'post_paint'):
                return handler.post_paint(context)
            return RobotState.ITEM_COMPLETE
        return RobotState.PAINT_EXECUTE

class FaultHandler(StateHandler):
    def on_enter(self):
        self.node.get_logger().error('[FSM] Entering FAULT — stopping robot')
        self.node.queue.flush()
        self.node.reset_stroke_gate()

    def evaluate(self, context) -> RobotState:
        return RobotState.FAULT


class HoldHandler(StateHandler):
    """Recoverable pause. Unlike FAULT, HOLD preserves everything — the motion
    queue, transient state, the saved return point — so the job can resume where
    it left off. Entering HOLD publishes /robot_state='HOLD', which every base/
    actuator driver (wall_motion, paint_executor) treats as a stop: one zero
    command, then silence. This handler is passive — it just holds; the exit
    (RESUME / safety NOMINAL) is driven from the FSM, not from here. Queue is NOT
    flushed on entry (that's the whole point)."""

    def on_enter(self):
        self.node.get_logger().warn('[FSM] Entering HOLD — motion paused, state preserved')
        self.node.reset_stroke_gate()

    def evaluate(self, context) -> RobotState:
        # Stay held. Resume is an external event (operator RESUME or safety
        # NOMINAL), handled in state_machine_node, which transitions us out.
        return RobotState.HOLD


# ─────────────────────────────────────────────────────────────────────────────
# Corner handlers — UNCHANGED from main branch for drop 1.
# Not queue-driven yet. External-corner drop rewrites these to emit ITEM_COMPLETE
# and split into TURN / ADVANCE phases via item.phase.
# ─────────────────────────────────────────────────────────────────────────────

class CornerApproachHandler(StateHandler):
    """
    Chunked crawl toward a corner.

    INTERNAL:
      - crawl in chunks up to move_step_size
      - paint completed crawl if distance >= MIN_PRETURN_PAINT_M
      - final shortened crawl follows the same paint rule
      - after final crawl/paint, return here
      - verify live front wall is at the internal trigger
      - ITEM_COMPLETE -> queue advances to TURN

    EXTERNAL:
      - retains existing CORNER_PREEMPT behaviour for now
    """

    MIN_PRETURN_PAINT_M = 0.15
    REARM_MIN_REMAINING = 0.02

    # LiDAR-frame INTERNAL trigger.
    # Current geometry: 0.904m turn-axis distance - 0.195m lidar offset.
    INTERNAL_TRIGGER_M = 0.709

    # Allow a small live LiDAR margin around the target.
    INTERNAL_FRONT_TOL_M = 0.02

    def on_enter(self):
        self.node.get_logger().info(
            '[FSM] Entering CORNER_APPROACH (CRAWL)'
        )

    def evaluate(self, context) -> RobotState:

        if context.get('wall_lost', False):
            return RobotState.FAULT

        item = self.node.queue.current_item()
        past = context.get('sprayhead_past_vertex', False)

        crawl_dist = context.get('distance_since_stroke', 0.0)
        remaining = context.get('crawl_remaining', -1.0)

        corner = context.get('corner_detected')

        is_internal = (
            corner is not None
            and corner.corner_type == EnvStatus.INTERNAL
        )

        # -------------------------------------------------------------
        # EXTERNAL only:
        # retain current PREEMPT behaviour.
        #
        # INTERNAL never enters this path anymore because env no longer
        # publishes CORNER_PREEMPT for INTERNAL.
        # -------------------------------------------------------------
        if (
            not is_internal
            and context.get('corner_preempt', False)
        ):
            context['step_complete'] = False

            self.node.get_logger().info(
                '[FSM] EXTERNAL CORNER_APPROACH preempt -> turn'
            )

            return RobotState.ITEM_COMPLETE

        # -------------------------------------------------------------
        # INTERNAL final crawl has already finished.
        #
        # We get here either:
        #   1. next tick after a short final crawl that did not paint
        #   2. after PAINT_EXECUTE returns from painting a >=0.15m
        #      final crawl
        #
        # Robot is stopped. Check the actual LiDAR front distance.
        # -------------------------------------------------------------
        if (
            is_internal
            and context.get('_internal_final_crawl_done', False)
        ):
            front = context.get('front_wall_distance', 0.0)

            if (
                front > 0.0
                and front <= (
                    self.INTERNAL_TRIGGER_M
                    + self.INTERNAL_FRONT_TOL_M
                )
            ):
                context['_internal_final_crawl_done'] = False
                context['step_complete'] = False

                self.node.get_logger().info(
                    f'[FSM] INTERNAL final approach confirmed - '
                    f'front={front:.3f}m <= '
                    f'{self.INTERNAL_TRIGGER_M + self.INTERNAL_FRONT_TOL_M:.3f}m '
                    f'-> TURN'
                )

                return RobotState.ITEM_COMPLETE

            # Odom says the commanded final crawl is finished, but the live
            # front wall does not yet agree. Stay stopped; do NOT re-arm crawl.
            self.node.get_logger().info(
                f'[FSM] INTERNAL final crawl done - '
                f'waiting for front trigger '
                f'(front={front:.3f}m, '
                f'target<={self.INTERNAL_TRIGGER_M + self.INTERNAL_FRONT_TOL_M:.3f}m)',
                throttle_duration_sec=1.0
            )

            return RobotState.CORNER_APPROACH

        # -------------------------------------------------------------
        # Current crawl command completed normally.
        # -------------------------------------------------------------
        if context.get('step_complete', False):

            context['step_complete'] = False

            # The final INTERNAL crawl is the one for which essentially
            # no odom distance remains to the trigger.
            final_internal_crawl = (
                is_internal
                and remaining >= 0.0
                and remaining <= self.REARM_MIN_REMAINING
            )

            if final_internal_crawl:
                context['_internal_final_crawl_done'] = True

                self.node.get_logger().info(
                    f'[FSM] INTERNAL final crawl complete - '
                    f'crawl={crawl_dist:.3f}m '
                    f'remaining={remaining:.3f}m'
                )

            # ---------------------------------------------------------
            # Paint ANY completed crawl >= 0.15m.
            #
            # This includes the final shortened crawl.
     
            if (
                item
                and item.paints
                and not past
                and crawl_dist >= self.MIN_PRETURN_PAINT_M
            ):
                self.node.get_logger().info(
                    f'[FSM] CORNER_APPROACH crawl complete - '
                    f'{crawl_dist:.3f}m >= '
                    f'{self.MIN_PRETURN_PAINT_M:.3f}m -> paint'
                )

                context['_paint_caller'] = RobotState.CORNER_APPROACH

                return RobotState.PAINT_EXECUTE

            # ---------------------------------------------------------
            # No paint.
            #
            # If this was the final INTERNAL crawl, do NOT issue another
            # movement. Return here and next tick the block above performs
            # the live front-wall confirmation.
            # ---------------------------------------------------------
            if final_internal_crawl:
                return RobotState.CORNER_APPROACH

            # ---------------------------------------------------------
            # Ordinary non-final crawl:
            # more distance remains, so issue another CORNER_APPROACH
            # command to wall_motion.
            # ---------------------------------------------------------
            if remaining > self.REARM_MIN_REMAINING:
                context['_rearm_corner_approach'] = True

        return RobotState.CORNER_APPROACH

    def post_paint(self, context) -> RobotState:
        # Every CORNER_APPROACH paint returns here first.
        #
        # If it was an ordinary strip, evaluate() will re-arm more crawl.
        # If it was the final strip, _internal_final_crawl_done is already
        # True and evaluate() will perform the front-wall confirmation,
        # then ITEM_COMPLETE -> TURN.
        return RobotState.CORNER_APPROACH


class CornerLogicHandler(StateHandler):
    """Shared state for two external-recipe phases, dispatched on item.phase:

      TURN    : in-place CW pivot (wall_motion runs segmented turn, paint per
                segment). Completes on wall_status.corner_turn_complete.
      ADVANCE : crawl forward past the vertex, paint per strip, watching for the
                new right wall to reacquire. Completes on corner_reacquired
                (or env-driven advance_complete / buffer timeout).

    Both emit ITEM_COMPLETE so the queue drives what's next:
      TURN complete    → queue advances → WALL_ALIGN/ALIGN
      ADVANCE complete → queue advances → drains → normal_block refills
    """

    def _phase(self):
        item = self.node.queue.current_item()
        return item.phase if item else 'TURN'

    def on_enter(self):
        phase = self._phase()
        corner = self.node.fsm_context.get('corner_detected')
        if phase == 'TURN':
            turn_dir = 'CCW' if (corner and corner.corner_radian_diff > 0) else 'CW'
            angle = abs(corner.corner_angle_deg) if corner else 90.0
            self.node.get_logger().info(
                f'[FSM] Entering CORNER_LOGIC/TURN — {turn_dir} {angle:.0f}°')
        elif phase in ('ARC1', 'ARC2'):
            self.node.get_logger().info(f'[FSM] Entering CORNER_LOGIC/{phase} (bump arc)')
        else:
            self.node.get_logger().info('[FSM] Entering CORNER_LOGIC/ADVANCE')

    def on_exit(self):
        self.node.get_logger().info(f'[FSM] CORNER_LOGIC/{self._phase()} exit')

    def evaluate(self, context) -> RobotState:
        # wall_lost is excluded in corner states (fsm_loop); hold if ever set.
        if context.get('wall_lost', False):
            return RobotState.CORNER_LOGIC

        item = self.node.queue.current_item()
        phase = self._phase()
        wall_status = context.get('wall_status')

        # ── TURN / ARC1 / ARC2 ──────────────────────────────────────────────────
        # All three are rotation items that complete on corner_turn_complete.
        # TURN  : single pivot (internal/external).
        # ARC1  : bump OUT arc  (paints — riser swatch).
        # ARC2  : bump BACK arc (no paint — item.paints is False, so no stroke).
        # Paint is gated by item.paints, so ARC2 never paints even on step_complete.
        if  phase in ('TURN', 'ARC1', 'ARC2', 'ARC'):    
            # paint one segment when wall_motion reports a segment step done
            if context.get('step_complete', False):
                context['step_complete'] = False
                # External corner, FINAL segment (step_complete arrives together with corner_turn_complete):
                # the robot stands just past the vertex and the nozzle has no wall in front of it yet,
                # so this stroke would spray air. Skip it; the earlier segment strokes covered the turn.
                if (item and getattr(item, 'label', '') == 'corner_arc'
                        and context.get('corner_turn_complete', False)):
                    self.node.get_logger().info(
                        '[FSM] external corner final segment - no wall in front of the nozzle, stroke suppressed')
                elif item and item.paints:
                    # Stroke gate (same rule as WALL_ALIGN): the turn-entry step_complete arrives with a
                    # 0 m crawl while the robot is still where the approach stroke already painted -
                    # a second stroke there is a duplicate. Segment-end strokes pass (robot turned >= 9 deg).
                    if not self.node.stroke_is_redundant():
                        context['_paint_caller'] = RobotState.CORNER_LOGIC
                        return RobotState.PAINT_EXECUTE
            if context.get('corner_turn_complete', False):
                context['corner_turn_complete'] = False
                self.node.get_logger().info(f'[FSM] {phase} complete')
                return RobotState.ITEM_COMPLETE     # → next arc, or WALL_ALIGN/ALIGN
            return RobotState.CORNER_LOGIC

        # ── ADVANCE ───────────────────────────────────────────────────────────
        # crawl forward, paint per strip, until the new right wall is reacquired
        if context.get('step_complete', False):
            context['step_complete'] = False
            if item and item.paints:
                context['_paint_caller'] = RobotState.CORNER_LOGIC
                return RobotState.PAINT_EXECUTE
            
        # if context.get('corner_reacquired', False):
        #     context['corner_reacquired'] = False
        #     self.node.get_logger().info('[FSM] REaquire reacquired new wall')
        #     # clear corner context now the sequence is finishing
        #     self.node._publish_corner_params(None)
        #     context['corner_preempt']  = False
        #     context['corner_detected'] = None
        #     context['_corner_active']  = False
        #     return RobotState.ITEM_COMPLETE         # → drains → normal_block

        if context.get('corner_reacquired', False):
            context['corner_reacquired'] = False
            ws = context.get('wall_status')
            if ws is not None and ws.wall_valid and ws.distance_error < -0.149:
                # Sprayhead collision risk — hard stop. Report (safety grants
                # FAULT tier, needs RESET) and hold; do NOT self-transition.
                self.node.get_logger().error(
                    f'[FSM] REACQUIRE wall too close '
                    f'(err={ws.distance_error:.3f} → ~{0.549 + ws.distance_error:.3f}m < SAFETY_THRESH) '
                    f'— sprayhead risk → reporting REACQUIRE_CLOSE')
                self.node.report_fault('REACQUIRE_CLOSE')
                return RobotState.CORNER_LOGIC
            self.node.get_logger().info('[FSM] REaquire reacquired new wall')
            self.node._publish_corner_params(None)
            context['corner_preempt']  = False
            context['corner_detected'] = None
            context['_corner_active']  = False
            return RobotState.ITEM_COMPLETE   

        return RobotState.CORNER_LOGIC

    # def post_paint(self, context) -> RobotState:
    #     # After a paint stroke during TURN or ADVANCE, return to this state to
    #     # let the next segment / crawl step run. Completion is re-checked in
    #     # evaluate on the next tick.
    #     return RobotState.CORNER_LOGIC

    def post_paint(self, context):
        if context.get('corner_turn_complete', False):
            return RobotState.ITEM_COMPLETE   
        return RobotState.CORNER_LOGIC