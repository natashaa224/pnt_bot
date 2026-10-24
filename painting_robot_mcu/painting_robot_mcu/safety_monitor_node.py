#!/usr/bin/env python3
"""
safety_monitor_node.py

Single safety aggregator. Detectors own their topics + debounce; this node
applies TIER POLICY, latches, and drives the FSM + PMC (via bridge).

Unidirectional fault model:
  Every fault - PMC DI, SBC sensor, or FSM-detected - enters through ONE door
  (_raise, via a detector callback or /fault_report) and exits through ONE door
  (/safety_command). Nodes with their own context (the FSM, perception) DETECT
  and REPORT on /fault_report; they do NOT self-transition. This node is the only
  place tiers, latches, flush policy and FSM transitions are decided.

Tiers (each does the previous + more):
  SAFETY_HOLD  sensor/obstacle hold, preserve all state.
         - SRC_OBSTACLE : auto-resume when clear, else escalate to FAULT at
                          ESCALATE_S.
  FAULT  controlled stop; flush motion queue + gantry limits; RESET+START.
  ESTOP  critical; flush everything incl limits; relay-drop (pending firmware).
         cliff / physical e-stop / collision-critical / operator Back.

  Operator PAUSE/RESUME and START are job control - owned by the FSM, NOT here.
  safety_node handles only ESTOP/RESET from the operator (STOP dropped).

Inputs:
  /start_cmd        std_msgs/String  START|PAUSE|RESUME|ESTOP|RESET|TEST_STROKE
  /fault_report     std_msgs/String  FSM-reported fault code (e.g. "WALL_LOST")
  /cliff_status     CliffStatus      cliff_detected -> ESTOP
  /ultrasound_status UltrasoundStatus any valid < US_MIN_MM -> SAFETY_HOLD(obstacle)
  /collision_warning CollisionWarning warn -> SAFETY_HOLD(obstacle); critical -> ESTOP
  /bms_status       BmsStatus        low V / fault / invalid -> FAULT
  /di_status_ros    DiStatus         estop / emg / ceiling / paint (PMC faults)
Staleness: any enabled input going silent > STALE_S -> FAULT (blind sensor).

Outputs:
  /safety_command   SafetyCommand    -> FSM (tier + flush policy)
  /safety_mode      SafetyMode       -> bridge -> PMC
  /safety_status    std_msgs/Bool    -> FSM healthy gate (True = NOMINAL)
  /safety_reason    std_msgs/String  -> HMI / logging

Recovery: FAULT/ESTOP latch. RESET clears (incl limits). SAFETY_HOLD auto-resumes.

NOTE - MCU status removed: the PMC MCU-status bitmask is no longer decoded
(stray-bit false FAULT during bench, comms_test_handoff.md). All PMC faults are
now DI-derived through /di_status_ros.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String
from painting_robot_msgs.msg import (
    SafetyCommand, SafetyMode, CliffStatus, UltrasoundStatus,
    CollisionWarning, BmsStatus, DiStatus,
)


# -- tier ordinals (match msg constants) --------------------------------------
NOMINAL, SAFETY_HOLD, FAULT, ESTOP = 0, 1, 2, 3
SRC_NONE, SRC_OBSTACLE, SRC_OPERATOR, SRC_SENSOR, SRC_PMC, SRC_FSM = 0, 1, 2, 3, 4, 5

# -- Universal fault reasons - single source of truth for reason strings -------
# Every _raise() uses one of these. HMI / logging keep reading /safety_reason
# as-is. This is the full set across SBC + PMC + FSM; add here as faults grow.
R_ESTOP            = 'estop'              # PMC hardware e-stop (DI)
R_EMG_LIMITS       = 'emg_limits'         # PMC EMG limit chain (DI)
R_LOWER_LIMIT      = 'lower_limit'        # observational only - NOT raised (see _di_cb)
R_PAINT_EMG        = 'paint_emg'          # paint level EMG (DI)
R_SSR_STATUS       = 'ssr_status'         # SSR feedback (DI/PIO) - gated until PIO exists
R_CEILING          = 'ceiling'            # ceiling detect (DI)
R_CLIFF            = 'cliff'
R_ULTRASOUND       = 'ultrasound_near'
R_COLLISION_WARN   = 'collision_warn'
R_COLLISION_CRIT   = 'collision_critical'
R_OBSTACLE_PERSIST = 'obstacle_persist'
R_BMS_COMMS        = 'bms_comms_lost'
R_BMS_FAULT        = 'bms_fault'
R_BATTERY_LOW      = 'battery_low'
R_WALL_LOST        = 'wall_lost'          # FSM-reported (perception/navigation)
R_WALL_DIST        = 'wall_dist'          # FSM-reported: dist error > tolerance
R_OPERATOR_ESTOP   = 'operator_estop'
R_OPERATOR_FAULT   = 'operator_fault'     # joy/HMI X-ladder step 2
R_OPERATOR_HOLD    = 'operator_hold'      # joy/HMI X-ladder step 1

# -- Reporter-requested faults on /fault_report: code -> granted tier ----------
# Only the FSM reports here today, only WALL_LOST. This dict is the start of the
# future policy table (the "allowed tier per code" lookup); _gate() is the clamp
# seam. Unknown codes default to FAULT.
FSM_FAULT_TIERS = {
    # wall_lost is a brief, recoverable condition (lidar dropout, doorway
    # mis-read) - HOLD and auto-resume if the wall comes back, escalate to FAULT
    # only if it persists past ESCALATE_S (same path as a persistent obstacle).
    R_WALL_LOST: SAFETY_HOLD,
    # dist error > tolerance — recoverable by re-align. HOLD, escalate to FAULT
    # if it persists (operator re-aligns, or RESET).
    R_WALL_DIST: SAFETY_HOLD,
}


class SafetyMonitorNode(Node):

    CHECK_RATE_HZ = 10.0
    ESCALATE_S    = 15.0      # obstacle-pause -> fault if not clear (tunable)
    STALE_S       = 10.0      # detector silence -> stale -> fault (must exceed slowest
                              # publisher period; BMS polls at 1Hz + jitter)
    US_MIN_MM     = 400       # ultrasound obstacle threshold (< 40 cm)
    BATT_MIN_V    = 44.0      # pack low-voltage fault (15S LiFePO4, tune)

    # -- Per-source enables - flip False to mute a flaky/mispositioned sensor --
    # BENCH flags. Disabled source: its callback ignored AND excluded from the
    # staleness watchdog (won't false-fault on a muted sensor).
    EN_CLIFF      = False
    EN_ULTRASOUND = False     # mispositioned - muted for bench
    EN_COLLISION  = False     # env_monitoring collision - muted for bench
    EN_BMS        = True
    EN_DI         = False
    EN_FAULT_RPT  = True      # FSM /fault_report (WALL_LOST etc.)

    # -- DI sub-gates - individual PMC DI faults, enabled as the hardware lands -
    EN_DI_SSR     = False     # SSR feedback input - no PIO yet, keep muted

    def __init__(self):
        super().__init__('safety_monitor_node')

        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        latched  = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=5)

        # -- Latched safety state ----------------------------------------------
        self._tier     = NOMINAL
        self._source   = SRC_NONE
        self._reason   = 'startup'
        self._latched  = False        # FAULT/ESTOP/operator-pause need RESET/START

        # obstacle-pause escalation clock
        self._obstacle_since = None

        # -- Live condition cache (per input) ----------------------------------
        self._cond = {}               # name -> (tier, source, reason)
        self._last_seen = {}          # topic -> ros time (staleness)

        # operator intent
        self._operator = None         # last operator_cmd

        # -- BYPASS ------------------------------------------------------------
        # Bench/sim only. When true, the node forces tier=NOMINAL every loop
        # regardless of any input, so /safety_status is True and the FSM can run
        # with detectors absent. This DISABLES ALL SAFETY. It exists so the FSM
        # no longer has to fabricate its own safety_ok - safety authority stays
        # here, one source of truth. REVERT for production. Warns continuously.
        self.declare_parameter('bypass', False)
        self._bypass = self.get_parameter('bypass').value
        # ESCALATE_S as a param (default = class const) so the wall_lost->FAULT
        # window can be widened from the launch file while iterating on corner
        # recovery. Revert to default for production.
        self.declare_parameter('escalate_s', self.ESCALATE_S)
        self.ESCALATE_S = float(self.get_parameter('escalate_s').value)
        self._bypass_warn_last = None
        if self._bypass:
            self.get_logger().error(
                '[SAFETY] *** BYPASS ON - ALL SAFETY DISABLED, forcing NOMINAL *** '
                '(bench/sim only - REVERT for production)')

        # -- Subs --------------------------------------------------------------
        self.create_subscription(String,          '/start_cmd',         self._op_cb,    reliable)
        self.create_subscription(String,          '/fault_report',      self._fault_cb, reliable)
        self.create_subscription(CliffStatus,      '/cliff_status',      self._cliff_cb, reliable)
        self.create_subscription(UltrasoundStatus, '/ultrasound_status', self._us_cb,    reliable)
        self.create_subscription(CollisionWarning, '/collision_warning', self._coll_cb,  reliable)
        self.create_subscription(BmsStatus,        '/bms_status',        self._bms_cb,   reliable)
        self.create_subscription(DiStatus,         '/di_status_ros',     self._di_cb,    reliable)

        # -- Pubs --------------------------------------------------------------
        self._cmd_pub    = self.create_publisher(SafetyCommand, '/safety_command', reliable)
        self._mode_pub   = self.create_publisher(SafetyMode,    '/safety_mode',    latched)
        self._status_pub = self.create_publisher(Bool,          '/safety_status',  reliable)
        self._reason_pub = self.create_publisher(String,        '/safety_reason',  reliable)

        self.create_timer(1.0 / self.CHECK_RATE_HZ, self._loop)

        # start safe
        self._emit_mode(SafetyMode.SAFETY_HOLD, 'startup')
        self.get_logger().info('safety_monitor_node started - holding until inputs healthy')

    # -- helpers ----------------------------------------------------------------
    def _now(self):
        return self.get_clock().now()

    def _mark(self, topic):
        self._last_seen[topic] = self._now()

    def _set_cond(self, name, tier, source, reason):
        self._cond[name] = (tier, source, reason)

    def _clear_cond(self, name):
        self._cond.pop(name, None)

    # -- Policy gate --------------------------------------------------------------
    def _gate(self, tier, source, reason):
        """Single policy chokepoint - every _raise() passes through here.

        Pass-thru for now. This is the seam where future policy goes: clamp a
        reporter-requested tier against an allowed-tier table, veto a source,
        downgrade, etc. Because every detector callback AND /fault_report funnel
        through _raise -> _gate, any rule added here applies everywhere at once.
        """
        return tier, source, reason

    # -- Operator ----------------------------------------------------------------
    def _op_cb(self, msg):
        # safety_node handles only safety verbs from the operator: STOP, ESTOP, RESET.
        # START and PAUSE/RESUME are job control -> go to the FSM directly, not here.
        cmd = msg.data.strip().upper()
        self._operator = cmd
        # Operator escalation ladder (joy X / HMI E), stepping HOLD->FAULT->ESTOP.
        # The HMI reads the live tier and sends the next step; this node just maps
        # each verb to its tier. All operator-sourced, distinct reasons.
        if cmd == 'ESTOP':
            self._raise(ESTOP, SRC_OPERATOR, R_OPERATOR_ESTOP, latch=True)
        elif cmd == 'FAULT':
            self._raise(FAULT, SRC_OPERATOR, R_OPERATOR_FAULT, latch=True)
        elif cmd == 'HOLD':
            self._raise(SAFETY_HOLD, SRC_OPERATOR, R_OPERATOR_HOLD, latch=False)
        elif cmd == 'RESET':
            self._on_reset()
        elif cmd == 'RESUME':
            # RESUME ends an operator HOLD. The hold is a safety-node condition,
            # so the FSM transitioning out of HOLD is not enough - we must clear
            # it HERE or the tier stays SAFETY_HOLD forever and actuators stay
            # inhibited. Clear only the operator hold; a still-live sensor hold
            # (obstacle/wall_lost) re-raises on its own next callback and RESUME
            # won't falsely clear it.
            self._on_resume()
        # START / PAUSE intentionally ignored - owned by FSM.

    def _on_resume(self):
        # Clear the operator HOLD condition. Only valid while actually holding.
        if self._tier != SAFETY_HOLD:
            self.get_logger().info('[SAFETY] RESUME ignored - not in SAFETY_HOLD')
            return
        self._clear_cond(R_OPERATOR_HOLD)
        # If a sensor/obstacle hold is still live it stays; otherwise we drop to
        # NOMINAL so the FSM's RESUME can actually run. No queue/limit flush -
        # RESUME continues the job, it doesn't abandon it.
        live = self._active_tier()
        if live[0] == NOMINAL:
            self.get_logger().info('[SAFETY] RESUME - operator hold cleared -> NOMINAL')
            self._cond.clear()
            self._obstacle_since = None
            self._publish_command(NOMINAL, SRC_NONE, 'resume',
                                  flush_queue=False, flush_limits=False)
            self._go_nominal()
        else:
            self.get_logger().warn(
                f'[SAFETY] RESUME - operator hold cleared, but still held by {live[2]}')

    # -- FSM / reporter fault channel ---------------------------------------------
    def _fault_cb(self, msg):
        """Non-sensor nodes (the FSM today; perception/odom later) report faults
        here as a reason code. They DETECT; this node DECIDES the tier. The tier
        comes from FSM_FAULT_TIERS, then through _gate (via _raise) - the reporter
        never commands a tier directly, preserving single-authority.
        """
        if not self.EN_FAULT_RPT:
            return
        code = msg.data.strip().lower()
        if not code:
            return
        self._mark('/fault_report')

        # -- Clear convention ------------------------------------------------
        # A reporter can explicitly clear a recoverable condition it raised by
        # publishing 'clear:<code>' (e.g. 'CLEAR:WALL_LOST'). Only recoverable
        # (non-latched, SAFETY_HOLD-tier) reporter faults may be cleared this way -
        # a latched FAULT/ESTOP still requires RESET. This is how the FSM tells us
        # "the wall_lost condition is resolved" (e.g. env found the next corner and
        # we're resuming) so we drop the hold instead of waiting out the 15s
        # escalation. Single-authority preserved: the reporter clears its OWN
        # reported condition; it does not command a tier.
        if code.startswith('clear:'):
            target = code.split('clear:', 1)[1].strip()
            if not target:
                return
            granted = FSM_FAULT_TIERS.get(target, FAULT)
            if granted >= FAULT:
                self.get_logger().warn(
                    f'[SAFETY] ignoring clear:{target} - {target} is a latched '
                    f'FAULT/ESTOP, needs RESET')
                return
            self._clear_cond(target)
            # reset the HOLD escalation clock if no recoverable hold remains
            if self._active_tier()[0] != SAFETY_HOLD:
                self._obstacle_since = None
            self.get_logger().info(f'[SAFETY] cleared reporter condition {target}')
            return

        tier = FSM_FAULT_TIERS.get(code, FAULT)   # default: treat unknown as FAULT
        self._raise(tier, SRC_FSM, code, latch=(tier >= FAULT))

    def _on_reset(self):
        if self._tier in (SAFETY_HOLD, FAULT, ESTOP):
            # operator-sourced conditions are commands, not persistent states -
            # RESET always clears them. Only a still-live sensor/PMC/FSM fault blocks.
            # (SAFETY_HOLD included so RESET can release an operator/sensor HOLD; a
            # still-present sensor cause simply re-raises on its next callback.)
            for r in (R_OPERATOR_ESTOP, R_OPERATOR_FAULT, R_OPERATOR_HOLD, 'operator_stop'):
                self._clear_cond(r)
            # Recoverable-origin faults that RESET must always escape:
            #  - 'hold_escalated': a HOLD that aged out to FAULT (obstacle_persist
            #    / wall_dist / wall_lost). The robot is parked at a bad spot; the
            #    operator RESETs to get to IDLE and teleop away. Keeping it would
            #    reject RESET forever while the cause persists (deadlock).
            #  - any SRC_FSM reporter condition (wall_dist/wall_lost/corner_abort/
            #    reacquire_close): the FSM re-reports these every tick while the
            #    geometry is bad, so they'd re-block RESET immediately. Clearing
            #    them lets RESET reach IDLE; they simply re-raise if the robot is
            #    put back into a painting state without first fixing the geometry.
            self._clear_cond('hold_escalated')
            for name, (t, src, rsn) in list(self._cond.items()):
                if src == SRC_FSM:
                    self._clear_cond(name)
            # refresh staleness - a topic that has come back fresh must not block
            # its own reset.
            now = self._now()
            for r in [k for k in list(self._cond) if k.startswith('stale:')]:
                topic = r.split('stale:', 1)[1]
                last = self._last_seen.get(topic)
                if last is not None and (now - last).nanoseconds * 1e-9 <= self.STALE_S:
                    self._clear_cond(r)
            # accept only if no critical condition still live
            live = self._active_tier()
            if live[0] in (FAULT, ESTOP):
                self.get_logger().warn(f'[SAFETY] RESET rejected - still active: {live[2]}')
                return
            was_hold = (self._tier == SAFETY_HOLD)
            self.get_logger().info(
                f'[SAFETY] RESET accepted - clearing{"" if was_hold else " (incl limits)"}')
            self._cond.clear()
            self._latched = False
            self._obstacle_since = None
            # A HOLD reset must NOT flush limits (nothing was faulted). A FAULT/
            # ESTOP reset flushes queue + limits as before.
            self._publish_command(NOMINAL, SRC_NONE, 'reset',
                                  flush_queue=not was_hold, flush_limits=not was_hold)
            self._go_nominal()
        else:
            self.get_logger().info('[SAFETY] RESET ignored - not in HOLD/FAULT/ESTOP')

    # -- Detector callbacks ------------------------------------------------------
    def _cliff_cb(self, msg):
        if not self.EN_CLIFF:
            return
        self._mark('/cliff_status')
        if msg.cliff_detected:
            self._raise(ESTOP, SRC_SENSOR, R_CLIFF, latch=True)
        else:
            self._clear_cond(R_CLIFF)

    def _us_cb(self, msg):
        if not self.EN_ULTRASOUND:
            return
        self._mark('/ultrasound_status')
        near = any(v and d < self.US_MIN_MM for d, v in zip(msg.distances_mm, msg.valid))
        if near:
            self._raise(SAFETY_HOLD, SRC_OBSTACLE, R_ULTRASOUND, latch=False)
        else:
            self._clear_cond(R_ULTRASOUND)

    def _coll_cb(self, msg):
        if not self.EN_COLLISION:
            return
        self._mark('/collision_warning')
        # CollisionWarning.status: 0 clear, 1 warn, 2 critical (per env_monitoring)
        st = getattr(msg, 'status', 0)
        if st >= 2:
            self._raise(ESTOP, SRC_SENSOR, R_COLLISION_CRIT, latch=True)
            self._clear_cond(R_COLLISION_WARN)
        elif st == 1:
            self._raise(SAFETY_HOLD, SRC_OBSTACLE, R_COLLISION_WARN, latch=False)
        else:
            self._clear_cond(R_COLLISION_WARN)

    def _bms_cb(self, msg):
        if not self.EN_BMS:
            return
        self._mark('/bms_status')
        if not msg.valid:
            self._raise(FAULT, SRC_SENSOR, R_BMS_COMMS, latch=True)
        elif msg.any_fault:
            self._raise(FAULT, SRC_SENSOR, R_BMS_FAULT, latch=True)
        elif msg.voltage < self.BATT_MIN_V:
            self._raise(FAULT, SRC_SENSOR, R_BATTERY_LOW, latch=True)
        else:
            for n in (R_BMS_COMMS, R_BMS_FAULT, R_BATTERY_LOW):
                self._clear_cond(n)

    def _di_cb(self, msg):
        if not self.EN_DI:
            return
        self._mark('/di_status_ros')
        # Hardware EMG chain owns the actual trip; safety_node OBSERVES and reflects
        # it into ROS tier/state. PMC DI faults -> universal reason codes.

        # -- ESTOP-class ------------------------------------------------------
        if msg.estop:
            self._raise(ESTOP, SRC_PMC, R_ESTOP, latch=True)
        else:
            self._clear_cond(R_ESTOP)

        if msg.ceiling_detect:
            self._raise(ESTOP, SRC_PMC, R_CEILING, latch=True)
        else:
            self._clear_cond(R_CEILING)

        if msg.emg_limits:
            self._raise(ESTOP, SRC_PMC, R_EMG_LIMITS, latch=True)
        else:
            self._clear_cond(R_EMG_LIMITS)

        # -- FAULT-class ------------------------------------------------------
        # paint level EMG - soft FAULT (not in the hardware EMG chain)
        if not msg.paint_level_ok:
            self._raise(FAULT, SRC_PMC, R_PAINT_EMG, latch=True)
        else:
            self._clear_cond(R_PAINT_EMG)

        # SSR feedback - distinct signal from relay_status (contactor DO). Gated
        # off until the PIO input exists; when enabled, a bad read is a FAULT.
        if self.EN_DI_SSR:
            if not getattr(msg, 'ssr_status', True):
                self._raise(FAULT, SRC_PMC, R_SSR_STATUS, latch=True)
            else:
                self._clear_cond(R_SSR_STATUS)

        # -- Observational - decoded, logged by FSM, NOT raised as a tier ------
        # lower_limit: per decision, observational debug only (not a fault yet).
        # upper_limit: part of the EMG chain, already covered by emg_limits.
        # relay_status: contactor DO feedback - not a DI fault here (and there is
        #   no DI bit mapped for it), so the old "safety_relay_open" ESTOP check
        #   is removed to avoid a permanent false trip when the bit decodes False.

    # -- Raise / recompute -------------------------------------------------------
    def _raise(self, tier, source, reason, latch):
        """Register a live condition. Single entry point - runs the policy gate,
        then caches the condition. Highest active tier wins in _loop.
        (`latch` is advisory for callers; the actual latch happens in _apply when
        a FAULT/ESTOP tier is applied.)"""
        tier, source, reason = self._gate(tier, source, reason)
        self._set_cond(reason, tier, source, reason)
        # arm the persist->FAULT escalation clock for any recoverable HOLD cause
        # (obstacle OR wall_lost) - all share the "hold briefly, fault if stuck".
        if reason in (R_ULTRASOUND, R_COLLISION_WARN, R_WALL_LOST, R_WALL_DIST):
            if self._obstacle_since is None:
                self._obstacle_since = self._now()

    def _active_tier(self):
        """Return (tier, source, reason) of the highest active condition."""
        best = (NOMINAL, SRC_NONE, 'nominal')
        for (tier, source, reason) in self._cond.values():
            if tier > best[0]:
                best = (tier, source, reason)
        return best

    # -- Main loop ----------------------------------------------------------------
    def _loop(self):
        # BYPASS: force NOMINAL at the source, ahead of staleness/active-tier, so
        # no condition (sensor, PMC, FSM, stale) can win. Outputs stay internally
        # consistent - status True, mode NOMINAL, command NOMINAL - because they
        # all derive from self._tier set here. Nags continuously while live.
        if self._bypass:
            self._bypass_nag()
            self._apply(NOMINAL, SRC_NONE, 'bypass')
            self._publish_status()
            return

        self._check_staleness()
        tier, source, reason = self._active_tier()

        # HOLD escalation: any recoverable SAFETY_HOLD (obstacle OR wall_lost)
        # still active after ESCALATE_S -> FAULT. The persisted reason is carried
        # into the fault reason so /safety_reason shows what got stuck.
        if tier == SAFETY_HOLD:
            if self._obstacle_since is not None:
                elapsed = (self._now() - self._obstacle_since).nanoseconds * 1e-9
                if elapsed >= self.ESCALATE_S:
                    self.get_logger().error(
                        f'[SAFETY] {reason} persisted {elapsed:.0f}s - escalating to FAULT')
                    esc_reason = f'{R_OBSTACLE_PERSIST}:{reason}'
                    self._set_cond('hold_escalated', FAULT, SRC_SENSOR, esc_reason)
                    tier, source, reason = FAULT, SRC_SENSOR, esc_reason
        else:
            # not holding -> reset the escalation clock
            self._obstacle_since = None

        # latch: once FAULT/ESTOP entered, hold until RESET even if source clears
        if self._latched and tier < self._tier:
            tier, source, reason = self._tier, self._source, self._reason

        self._apply(tier, source, reason)
        self._publish_status()

    def _check_staleness(self):
        now = self._now()
        # only watch enabled sources - a muted sensor must not stale-fault
        watched = []
        if self.EN_CLIFF:      watched.append('/cliff_status')
        if self.EN_ULTRASOUND: watched.append('/ultrasound_status')
        if self.EN_BMS:        watched.append('/bms_status')
        if self.EN_DI:         watched.append('/di_status_ros')
        for topic in watched:
            last = self._last_seen.get(topic)
            if last is None:
                continue    # never seen yet - don't fault on cold start
            if (now - last).nanoseconds * 1e-9 > self.STALE_S:
                self._set_cond(f'stale:{topic}', FAULT, SRC_SENSOR, f'stale{topic}')
            else:
                self._clear_cond(f'stale:{topic}')

    def _apply(self, tier, source, reason):
        if tier == self._tier and reason == self._reason:
            return    # no change
        self._tier, self._source, self._reason = tier, source, reason

        if tier in (FAULT, ESTOP):
            self._latched = True

        flush_queue  = tier in (FAULT, ESTOP)
        flush_limits = tier in (FAULT, ESTOP)   # both flush limits per design

        self._publish_command(tier, source, reason, flush_queue, flush_limits)

        mode = {NOMINAL: SafetyMode.NOMINAL, SAFETY_HOLD: SafetyMode.SAFETY_HOLD,
                FAULT: SafetyMode.FAULT, ESTOP: SafetyMode.ESTOP}[tier]
        self._emit_mode(mode, reason)

        name = ["NOMINAL", "SAFETY_HOLD", "FAULT", "ESTOP"][tier]
        line = f'[SAFETY] -> {name} ({reason})'
        if tier == NOMINAL:
            self.get_logger().info(line)
        elif tier == SAFETY_HOLD:
            self.get_logger().warning(line)
        else:
            self.get_logger().error(line)

    def _go_nominal(self):
        self._tier, self._source, self._reason = NOMINAL, SRC_NONE, 'nominal'
        self._emit_mode(SafetyMode.NOMINAL, 'recovered')

    def _bypass_nag(self, period_s=3.0):
        """Loud, recurring reminder that safety is OFF. The normal-path _apply
        logs once per tier change; bypass holds one tier forever, so without this
        the fact that safety is disabled scrolls off and is forgotten."""
        now = self._now()
        if (self._bypass_warn_last is None
                or (now - self._bypass_warn_last).nanoseconds * 1e-9 >= period_s):
            self.get_logger().error('[SAFETY] *** BYPASS ACTIVE - all safety inputs ignored ***')
            self._bypass_warn_last = now

    # -- Publishers --------------------------------------------------------------
    def _publish_command(self, tier, source, reason, flush_queue, flush_limits):
        m = SafetyCommand()
        m.header.stamp = self._now().to_msg()
        m.tier = tier
        m.source = source
        m.reason = reason
        m.flush_queue = bool(flush_queue)
        m.flush_limits = bool(flush_limits)
        self._cmd_pub.publish(m)

    def _emit_mode(self, mode, reason):
        m = SafetyMode()
        m.mode = mode
        m.paint_inhibit = False
        m.reason = reason
        self._mode_pub.publish(m)

    def _publish_status(self):
        b = Bool(); b.data = (self._tier == NOMINAL)
        self._status_pub.publish(b)
        r = String()
        names = ['NOMINAL', 'SAFETY_HOLD', 'FAULT', 'ESTOP']
        r.data = names[self._tier] + ('' if self._tier == NOMINAL else f': {self._reason}')
        self._reason_pub.publish(r)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
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