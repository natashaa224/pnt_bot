#!/usr/bin/env python3
"""
safety_logic_harness.py

Pure-Python test of the safety_monitor decision logic — no ROS, no serial.
Reimplements ONLY the tier/latch/escalation/priority state machine (kept in
lockstep with safety_monitor_node.py) and drives it through scripted scenarios,
asserting the resulting tier at each step.

This validates the LOGIC (latch held correctly, highest tier wins, escalation
timing, staleness, auto-resume vs manual) deterministically — the combinations
that are hard to stage repeatably on real hardware.

Run:  python3 safety_logic_harness.py
"""

# ── tier / source ordinals (mirror the msgs) ──────────────────────────────────
NOMINAL, PAUSE, FAULT, ESTOP = 0, 1, 2, 3
SRC_NONE, SRC_OBSTACLE, SRC_OPERATOR, SRC_SENSOR, SRC_PMC = 0, 1, 2, 3, 4
TIER_NAME = ['NOMINAL', 'PAUSE', 'FAULT', 'ESTOP']


class SafetyLogic:
    """
    Mirror of safety_monitor_node's decision core, with an injectable clock so
    escalation/staleness can be tested deterministically. Kept intentionally
    identical in behaviour to the node; if the node changes, update both.
    """
    ESCALATE_S = 15.0
    STALE_S    = 1.0

    def __init__(self):
        self.t          = 0.0          # injectable monotonic clock (s)
        self.cond       = {}           # reason -> (tier, source)
        self.tier       = NOMINAL
        self.source     = SRC_NONE
        self.reason     = 'nominal'
        self.latched    = False
        self.obstacle_since = None
        self.last_seen  = {}           # topic -> t
        self.last_cmd   = None         # (tier, flush_queue, flush_limits)

    # ── condition management ─────────────────────────────────────────────────
    def raise_cond(self, reason, tier, source):
        self.cond[reason] = (tier, source)
        if source == SRC_OBSTACLE and self.obstacle_since is None:
            self.obstacle_since = self.t

    def clear_cond(self, reason):
        self.cond.pop(reason, None)

    def mark(self, topic):
        self.last_seen[topic] = self.t

    def _active(self):
        best = (NOMINAL, SRC_NONE, 'nominal')
        for reason, (tier, source) in self.cond.items():
            if tier > best[0]:
                best = (tier, source, reason)
        return best

    # ── operator ─────────────────────────────────────────────────────────────
    def operator(self, cmd):
        if cmd == 'ESTOP':
            self.raise_cond('operator_estop', ESTOP, SRC_OPERATOR)
        elif cmd == 'STOP':
            self.raise_cond('operator_stop', FAULT, SRC_OPERATOR)
        elif cmd == 'PAUSE':
            self.raise_cond('operator_pause', PAUSE, SRC_OPERATOR)
        elif cmd == 'START':
            # clears operator-pause only
            if self.tier == PAUSE and self.source == SRC_OPERATOR:
                self.clear_cond('operator_pause')
                self.latched = False
        elif cmd == 'RESET':
            if self.tier in (FAULT, ESTOP):
                # operator-sourced conditions are commands, not persistent states —
                # RESET always clears them. Only a still-live sensor/PMC fault blocks.
                for r in ('operator_estop', 'operator_stop', 'operator_pause'):
                    self.clear_cond(r)
                # refresh staleness against current clock — a topic that has come
                # back fresh must not block its own reset.
                for r in [k for k in self.cond if k.startswith('stale:')]:
                    topic = r.split('stale:', 1)[1]
                    last = self.last_seen.get(topic)
                    if last is not None and (self.t - last) <= self.STALE_S:
                        self.clear_cond(r)
                live = self._active()
                if live[0] in (FAULT, ESTOP):
                    return  # rejected, a real sensor/pmc fault is still live
                self.cond.clear()
                self.latched = False
                self.obstacle_since = None

    # ── staleness ────────────────────────────────────────────────────────────
    def _check_staleness(self, watched):
        for topic in watched:
            last = self.last_seen.get(topic)
            if last is None:
                continue
            if self.t - last > self.STALE_S:
                self.raise_cond(f'stale:{topic}', FAULT, SRC_SENSOR)
            else:
                self.clear_cond(f'stale:{topic}')

    # ── main evaluation (call after mutating conditions / advancing clock) ────
    def evaluate(self, watched=()):
        self._check_staleness(watched)
        tier, source, reason = self._active()

        # obstacle escalation
        if tier == PAUSE and source == SRC_OBSTACLE:
            if self.obstacle_since is not None and (self.t - self.obstacle_since) >= self.ESCALATE_S:
                self.cond['obstacle_persist'] = (FAULT, SRC_SENSOR)
                tier, source, reason = FAULT, SRC_SENSOR, 'obstacle_persist'
        else:
            if not (tier == PAUSE and source == SRC_OBSTACLE):
                self.obstacle_since = None

        # latch
        if self.latched and tier < self.tier:
            tier, source, reason = self.tier, self.source, self.reason

        # commit
        if tier in (FAULT, ESTOP):
            self.latched = True
        elif tier == PAUSE and source == SRC_OPERATOR:
            self.latched = True

        self.tier, self.source, self.reason = tier, source, reason
        self.last_cmd = (tier, tier in (FAULT, ESTOP), tier in (FAULT, ESTOP))
        return tier


# ══════════════════════════════════════════════════════════════════════════════
# Scenarios
# ══════════════════════════════════════════════════════════════════════════════
_pass = 0
_fail = 0

def check(desc, got, want):
    global _pass, _fail
    ok = got == want
    _pass += ok
    _fail += (not ok)
    tag = 'ok ' if ok else 'FAIL'
    print(f'  [{tag}] {desc}: got {TIER_NAME[got]} want {TIER_NAME[want]}')


def s_cliff_latch():
    print('cliff -> ESTOP, latch holds after clear, RESET recovers')
    s = SafetyLogic()
    s.raise_cond('cliff', ESTOP, SRC_SENSOR); check('cliff fires', s.evaluate(), ESTOP)
    s.clear_cond('cliff');                    check('cliff clears (latched)', s.evaluate(), ESTOP)
    s.operator('RESET');                      check('reset clears', s.evaluate(), NOMINAL)


def s_obstacle_escalate():
    print('ultrasound obstacle -> PAUSE, persists 15s -> escalate FAULT')
    s = SafetyLogic()
    s.t = 0.0
    s.raise_cond('ultrasound_near', PAUSE, SRC_OBSTACLE); check('obstacle', s.evaluate(), PAUSE)
    s.t = 14.0;  check('14s still pause', s.evaluate(), PAUSE)
    s.t = 16.0;  check('16s escalates', s.evaluate(), FAULT)


def s_obstacle_autoresume():
    print('ultrasound obstacle -> PAUSE, clears at 10s -> auto NOMINAL')
    s = SafetyLogic()
    s.t = 0.0
    s.raise_cond('ultrasound_near', PAUSE, SRC_OBSTACLE); check('obstacle', s.evaluate(), PAUSE)
    s.t = 10.0; s.clear_cond('ultrasound_near');          check('clears -> nominal', s.evaluate(), NOMINAL)


def s_priority():
    print('battery FAULT + cliff ESTOP simultaneously -> ESTOP wins')
    s = SafetyLogic()
    s.raise_cond('battery_low', FAULT, SRC_SENSOR)
    s.raise_cond('cliff', ESTOP, SRC_SENSOR)
    check('estop wins', s.evaluate(), ESTOP)
    s.clear_cond('cliff'); check('cliff clears, still latched ESTOP', s.evaluate(), ESTOP)
    # even after reset, battery still live -> reset rejected
    s.operator('RESET'); check('reset rejected (batt live)', s.evaluate(), ESTOP)
    s.clear_cond('battery_low'); s.operator('RESET'); check('reset ok after batt clears', s.evaluate(), NOMINAL)


def s_operator_pause():
    print('operator PAUSE latches, obstacle-clear does not resume, START resumes')
    s = SafetyLogic()
    s.operator('PAUSE'); check('operator pause', s.evaluate(), PAUSE)
    # unrelated obstacle clearing must not auto-resume an operator pause
    s.t = 20.0;          check('still paused at 20s', s.evaluate(), PAUSE)
    s.operator('START'); check('start resumes', s.evaluate(), NOMINAL)


def s_staleness():
    print('bms goes stale -> FAULT; fresh again + reset -> NOMINAL')
    s = SafetyLogic()
    watched = ('/bms_status',)
    s.t = 0.0; s.mark('/bms_status'); check('fresh', s.evaluate(watched), NOMINAL)
    s.t = 2.0;                        check('stale -> fault', s.evaluate(watched), FAULT)
    s.t = 2.1; s.mark('/bms_status')  # fresh again, but latched
    s.operator('RESET');              check('reset after fresh', s.evaluate(watched), NOMINAL)


def s_fault_then_estop():
    print('FAULT then ESTOP escalates; RESET from ESTOP clears both')
    s = SafetyLogic()
    s.raise_cond('battery_low', FAULT, SRC_SENSOR); check('fault', s.evaluate(), FAULT)
    s.raise_cond('cliff', ESTOP, SRC_SENSOR);       check('escalates to estop', s.evaluate(), ESTOP)
    s.clear_cond('cliff'); s.clear_cond('battery_low')
    check('sources clear, latched estop', s.evaluate(), ESTOP)
    s.operator('RESET');                            check('reset clears', s.evaluate(), NOMINAL)


def s_estop_no_autoresume():
    print('operator ESTOP never auto-clears; needs RESET')
    s = SafetyLogic()
    s.operator('ESTOP'); check('estop', s.evaluate(), ESTOP)
    s.operator('START'); check('start ignored in estop', s.evaluate(), ESTOP)
    s.operator('RESET'); check('reset clears', s.evaluate(), NOMINAL)


if __name__ == '__main__':
    for sc in (s_cliff_latch, s_obstacle_escalate, s_obstacle_autoresume,
               s_priority, s_operator_pause, s_staleness,
               s_fault_then_estop, s_estop_no_autoresume):
        sc()
        print()
    print(f'== {_pass} passed, {_fail} failed ==')
