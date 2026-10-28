#!/usr/bin/env python3
"""
logging_node.py

Consumer-only telemetry logger. Subscribes to existing topics, maintains two
stores, publishes a live stats topic. Owns no sensor, no display.

Two data lifetimes:
  - per-run  : one dir per job, sealed on end, INCOMPLETE if crashed
  - lifetime : single YAML of cumulative counters, atomic-written,
               reconciled by replaying run event logs on boot

Run boundaries (from RobotCommand.state edges):
  IDLE -> SYSTEM_CHECK          : open_run()
  <active> -> IDLE  / -> FAULT  : seal_run(reason)
  crash / Ctrl-C (no seal)      : detected + marked INCOMPLETE on next boot,
                                  lifetime reconciled from its events.jsonl

Stroke      = /stroke_complete True (head_mode is never populated by the bridge)
Tip cycle   = SprayCmd.valve_open rising edge
Coverage    = per stroke: painted height (paint_on zones of the latest StripMap,
              taught homing range) * LATERAL_STRIP_M. Ignored until homed.
Errors      = online aggregate (n/sum/sumsq/max) of WallStatus dist/angle error

Publishes:
  /job_stats   (painting_robot_msgs/JobStats)   ~1 Hz

Subscribes:
  /robot_command      (painting_robot_msgs/RobotCommand)
  /paint_head_status  (painting_robot_msgs/PaintHeadStatus)
  /spray_cmd          (painting_robot_msgs/SprayCmd)
  /strip_map          (painting_robot_msgs/StripMap)
  /stroke_complete    (std_msgs/Bool)
  /wall_status        (painting_robot_msgs/WallStatus)
"""

import os
import json
import math
import time
import datetime as dt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from painting_robot_msgs.msg import (
    RobotCommand, PaintHeadStatus, SprayCmd, StripMap, WallStatus, JobStats,
)

# ── Constants ─────────────────────────────────────────────────────────────────
LATERAL_STRIP_M   = 0.203        # 8 inch strip width
STATE_SYSCHECK    = 'SYSTEM_CHECK'
STATE_IDLE        = 'IDLE'
STATE_FAULT       = 'FAULT'
FLUSH_PERIOD_S    = 5.0          # periodic stats.json flush
STATS_PERIOD_S    = 1.0          # /job_stats publish period


# ── Online aggregate — mean / rms / max without storing samples ───────────────
class OnlineStat:
    def __init__(self):
        self.n = 0
        self.sum = 0.0
        self.sumsq = 0.0
        self.max = 0.0

    def add(self, x):
        x = abs(float(x))
        self.n += 1
        self.sum += x
        self.sumsq += x * x
        if x > self.max:
            self.max = x

    @property
    def mean(self):
        return self.sum / self.n if self.n else 0.0

    @property
    def rms(self):
        return math.sqrt(self.sumsq / self.n) if self.n else 0.0

    def as_dict(self):
        return {'n': self.n, 'mean': self.mean, 'rms': self.rms, 'max': self.max}


def atomic_write(path, text):
    """Write via temp + os.replace so a crash mid-write can't corrupt the file."""
    tmp = f'{path}.tmp'
    with open(tmp, 'w') as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


class LoggingNode(Node):

    def __init__(self):
        super().__init__('logging_node')

        self.declare_parameter('log_root', os.path.expanduser('~/paint_logs'))
        self.log_root = self.get_parameter('log_root').value
        self.lifetime_path = os.path.join(self.log_root, 'lifetime.yaml')
        os.makedirs(self.log_root, exist_ok=True)

        # ── Run state ─────────────────────────────────────────────────────────
        self.run_active   = False
        self.run_dir      = None
        self.run_number   = 0
        self.run_start_ts = 0.0
        self.events_fp    = None
        self._reset_run_accumulators()

        # ── Edge trackers ─────────────────────────────────────────────────────
        self._prev_state    = STATE_IDLE
        self._prev_valve    = False

        # ── Latest-value caches for /job_stats ────────────────────────────────
        self.fsm_state    = STATE_IDLE
        self.batt_soc     = 0.0
        self.batt_voltage = 0.0
        self.batt_valid   = False       # BMS node not built yet
        self.homed        = False       # taught homing limits present (PaintHeadStatus)
        self.cur_strip    = None        # latest StripMap seen while homed

        # ── Lifetime counters ─────────────────────────────────────────────────
        self.lifetime = self._load_lifetime()

        # Boot recovery: seal any run left INCOMPLETE, reconcile lifetime.
        self._recover_incomplete_runs()

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.create_subscription(RobotCommand,     '/robot_command',     self._cmd_cb,   10)
        self.create_subscription(PaintHeadStatus,  '/paint_head_status', self._head_cb,  10)
        self.create_subscription(SprayCmd,         '/spray_cmd',         self._spray_cb, 10)
        self.create_subscription(StripMap,         '/strip_map',         self._strip_cb, 10)
        self.create_subscription(Bool,             '/stroke_complete',   self._stroke_cb, 10)
        self.create_subscription(WallStatus,       '/wall_status',       self._wall_cb,  10)

        self._stats_pub = self.create_publisher(JobStats, '/job_stats', 10)

        self.create_timer(FLUSH_PERIOD_S, self._flush_tick)
        self.create_timer(STATS_PERIOD_S, self._publish_stats)

        self.get_logger().info(f'logging_node started — root={self.log_root}')

    # ──────────────────────────────────────────────────────────────────────────
    # Run accumulators
    # ──────────────────────────────────────────────────────────────────────────
    def _reset_run_accumulators(self):
        self.strokes        = 0
        self.tip_cycles     = 0
        self.coverage_m2    = 0.0
        self.strip_z_min    = math.inf
        self.strip_z_max    = -math.inf
        self.paint_start    = None
        self.paint_last     = None
        self.dist_err       = OnlineStat()
        self.angle_err      = OnlineStat()
        self.stroke_first_ts = None     # first / last stroke wall time, for active rate
        self.stroke_last_ts  = None

    # ──────────────────────────────────────────────────────────────────────────
    # Lifetime store
    # ──────────────────────────────────────────────────────────────────────────
    def _default_lifetime(self):
        return {
            'total_runs': 0,
            'total_strokes': 0,
            'tip_cycles_total': 0,
            'strokes_since_tip_change': 0,
            'runs_since_clean': 0,
            'total_runtime_s': 0.0,
            'total_coverage_m2': 0.0,
            'reconciled_runs': [],      # run dirs already folded in (prevent double count)
        }

    def _load_lifetime(self):
        try:
            import yaml
            with open(self.lifetime_path) as f:
                d = yaml.safe_load(f) or {}
            base = self._default_lifetime()
            base.update(d)
            return base
        except FileNotFoundError:
            return self._default_lifetime()
        except Exception as e:
            self.get_logger().error(f'lifetime load failed ({e}); starting fresh')
            return self._default_lifetime()

    def _save_lifetime(self):
        import yaml
        atomic_write(self.lifetime_path, yaml.safe_dump(self.lifetime, sort_keys=False))

    # ──────────────────────────────────────────────────────────────────────────
    # Boot recovery — replay unsealed runs
    # ──────────────────────────────────────────────────────────────────────────
    def _recover_incomplete_runs(self):
        if not os.path.isdir(self.log_root):
            return
        for date_dir in sorted(os.listdir(self.log_root)):
            dpath = os.path.join(self.log_root, date_dir)
            if not os.path.isdir(dpath):
                continue
            for run_name in sorted(os.listdir(dpath)):
                rpath = os.path.join(dpath, run_name)
                if not os.path.isdir(rpath):
                    continue
                meta_path = os.path.join(rpath, 'meta.json')
                if not os.path.isfile(meta_path):
                    continue
                try:
                    with open(meta_path) as f:
                        meta = json.load(f)
                except Exception:
                    continue
                if meta.get('status') == 'SEALED':
                    continue
                # unsealed -> reconcile from events, mark INCOMPLETE
                self.get_logger().warn(f'recovering incomplete run {run_name}')
                self._reconcile_from_events(rpath, meta)
                meta['status'] = 'INCOMPLETE'
                meta['end_reason'] = meta.get('end_reason', 'CRASH')
                try:
                    atomic_write(meta_path, json.dumps(meta, indent=2))
                except Exception as e:
                    self.get_logger().error(f'could not mark INCOMPLETE: {e}')
        self._save_lifetime()

    def _reconcile_from_events(self, run_path, meta):
        """events.jsonl is source of truth. Fold this run into lifetime once."""
        run_id = meta.get('run_id', run_path)
        if run_id in self.lifetime['reconciled_runs']:
            return
        ev_path = os.path.join(run_path, 'events.jsonl')
        strokes = tips = 0
        coverage = 0.0
        runtime = 0.0
        if os.path.isfile(ev_path):
            with open(ev_path) as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        ev = json.loads(line)
                    except json.JSONDecodeError:
                        continue    # torn last line from a crash — discard
                    t = ev.get('type')
                    if t == 'stroke':
                        strokes += 1
                    elif t == 'tip':
                        tips += 1
                    elif t == 'coverage':
                        coverage += ev.get('area', 0.0)
                    elif t == 'end':
                        runtime = ev.get('runtime_s', 0.0)
        self.lifetime['total_runs'] += 1
        self.lifetime['total_strokes'] += strokes
        self.lifetime['tip_cycles_total'] += tips
        self.lifetime['strokes_since_tip_change'] += strokes
        self.lifetime['runs_since_clean'] += 1
        self.lifetime['total_runtime_s'] += runtime
        self.lifetime['total_coverage_m2'] += coverage
        self.lifetime['reconciled_runs'].append(run_id)

    # ──────────────────────────────────────────────────────────────────────────
    # Run lifecycle
    # ──────────────────────────────────────────────────────────────────────────
    def _open_run(self):
        now = dt.datetime.now()
        date_dir = os.path.join(self.log_root, now.strftime('%Y-%m-%d'))
        os.makedirs(date_dir, exist_ok=True)
        n = 1
        for name in os.listdir(date_dir):
            if name.startswith('run_'):
                try:
                    n = max(n, int(name.split('_')[1]) + 1)
                except (ValueError, IndexError):
                    pass
        run_name = f'run_{n:02d}_{now.strftime("%H%M")}'
        self.run_dir = os.path.join(date_dir, run_name)
        os.makedirs(self.run_dir, exist_ok=True)

        self.run_number   = n
        self.run_start_ts = time.time()
        self.run_active   = True
        self._reset_run_accumulators()

        meta = {
            'run_id':     os.path.relpath(self.run_dir, self.log_root),
            'run_number': n,
            'start':      now.isoformat(timespec='seconds'),
            'status':     'OPEN',
            'end_reason': None,
        }
        atomic_write(os.path.join(self.run_dir, 'meta.json'), json.dumps(meta, indent=2))
        self.events_fp = open(os.path.join(self.run_dir, 'events.jsonl'), 'a')
        self._event('start', run_number=n)
        self.get_logger().info(f'[run] opened {run_name}')

    def _seal_run(self, reason):
        if not self.run_active:
            return
        runtime = time.time() - self.run_start_ts
        self._event('end', runtime_s=runtime, reason=reason)
        if self.events_fp:
            self.events_fp.flush()
            os.fsync(self.events_fp.fileno())
            self.events_fp.close()
            self.events_fp = None

        self._write_stats()

        # mark meta sealed
        meta_path = os.path.join(self.run_dir, 'meta.json')
        try:
            with open(meta_path) as f:
                meta = json.load(f)
        except Exception:
            meta = {'run_id': os.path.relpath(self.run_dir, self.log_root)}
        meta['status'] = 'SEALED'
        meta['end_reason'] = reason
        meta['end'] = dt.datetime.now().isoformat(timespec='seconds')
        meta['runtime_s'] = runtime
        atomic_write(meta_path, json.dumps(meta, indent=2))

        # reconcile lifetime from this run's events (single source of truth)
        self._reconcile_from_events(self.run_dir, meta)
        self._save_lifetime()

        self.get_logger().info(f'[run] sealed {meta.get("run_id")} ({reason})')
        self.run_active = False
        self.run_dir = None

    def _write_stats(self):
        if not self.run_dir:
            return
        paint = 0.0
        if self.paint_start is not None and self.paint_last is not None:
            paint = max(0.0, self.paint_start - self.paint_last)
        runtime = time.time() - self.run_start_ts
        active = 0.0
        if self.stroke_first_ts is not None and self.stroke_last_ts is not None:
            active = self.stroke_last_ts - self.stroke_first_ts
        stats = {
            'run_number':   self.run_number,
            'runtime_s':    runtime,
            'strokes':      self.strokes,
            'tip_cycles':   self.tip_cycles,
            'coverage_m2':  self.coverage_m2,
            'coverage_m2_per_hr':        (self.coverage_m2 / runtime * 3600.0) if runtime > 0 else 0.0,
            'strokes_per_hr':            (self.strokes / runtime * 3600.0) if runtime > 0 else 0.0,
            # first stroke -> last stroke; excludes homing / idle lead-in. Needs >=2 strokes.
            'coverage_m2_per_hr_active': (self.coverage_m2 / active * 3600.0) if active > 0 else 0.0,
            'paint_consumed': paint,
            'strip_z_min':  None if self.strip_z_min ==  math.inf else self.strip_z_min,
            'strip_z_max':  None if self.strip_z_max == -math.inf else self.strip_z_max,
            'distance_error': self.dist_err.as_dict(),
            'angle_error':    self.angle_err.as_dict(),
        }
        atomic_write(os.path.join(self.run_dir, 'stats.json'),
                     json.dumps(stats, indent=2))

    def _event(self, etype, **kw):
        if not self.events_fp:
            return
        rec = {'t': time.time(), 'type': etype}
        rec.update(kw)
        self.events_fp.write(json.dumps(rec) + '\n')
        self.events_fp.flush()          # each event durable as it happens

    # ──────────────────────────────────────────────────────────────────────────
    # Subscriptions
    # ──────────────────────────────────────────────────────────────────────────
    def _cmd_cb(self, msg):
        state = msg.state
        self.fsm_state = state
        if state != self._prev_state:
            # open on IDLE -> SYSTEM_CHECK
            if state == STATE_SYSCHECK and self._prev_state == STATE_IDLE:
                self._open_run()
            # seal on entering IDLE or FAULT from an active run
            elif self.run_active and state in (STATE_IDLE, STATE_FAULT):
                self._seal_run(reason=state)
            self._prev_state = state

    def _head_cb(self, msg):
        # Homed = taught limits valid (same test paint_planner_node uses).
        self.homed = bool(msg.limit_lower_z
                          and msg.z_soft_upper_mm > msg.z_soft_lower_mm)

        if self.run_active:
            pl = float(msg.paint_level)
            if self.paint_start is None:
                self.paint_start = pl
            self.paint_last = pl

    def _spray_cb(self, msg):
        if msg.valve_open and not self._prev_valve and self.run_active:
            self.tip_cycles += 1
            self._event('tip')
        self._prev_valve = bool(msg.valve_open)

    def _strip_cb(self, msg):
        # Cache only. Before homing the planner publishes a fallback range
        # (launch-file guess), which must never reach coverage or the envelope.
        if not self.homed:
            return
        self.cur_strip = msg
        if not self.run_active:
            return
        self.strip_z_min = min(self.strip_z_min, float(msg.strip_z_min))
        self.strip_z_max = max(self.strip_z_max, float(msg.strip_z_max))

    @staticmethod
    def _painted_span(strip):
        """Height (m) of the strip's paint_on zones; whole span if no zones."""
        zs, ze, on = list(strip.z_start), list(strip.z_end), list(strip.paint_on)
        if not zs or not (len(zs) == len(ze) == len(on)):
            return max(0.0, float(strip.strip_z_max) - float(strip.strip_z_min))
        return sum(max(0.0, e - s) for s, e, p in zip(zs, ze, on) if p)

    def _stroke_cb(self, msg):
        if not msg.data or not self.run_active:
            return
        strip = self.cur_strip
        if strip is None:
            self.get_logger().warn('stroke with no homed StripMap - coverage not counted')
            area, zmin, zmax = 0.0, None, None
        else:
            area = self._painted_span(strip) * LATERAL_STRIP_M
            zmin, zmax = float(strip.strip_z_min), float(strip.strip_z_max)
        now = time.time()
        if self.stroke_first_ts is None:
            self.stroke_first_ts = now
        self.stroke_last_ts = now
        self.strokes += 1
        self.coverage_m2 += area
        self._event('stroke')
        self._event('coverage', area=area, z_min=zmin, z_max=zmax)

    def _wall_cb(self, msg):
        if not self.run_active:
            return
        self.dist_err.add(msg.distance_error)
        self.angle_err.add(msg.angle_error)

    # ──────────────────────────────────────────────────────────────────────────
    # Timers
    # ──────────────────────────────────────────────────────────────────────────
    def _flush_tick(self):
        if self.run_active:
            self._write_stats()

    def _publish_stats(self):
        m = JobStats()
        m.header.stamp = self.get_clock().now().to_msg()

        m.run_active = self.run_active
        m.run_number = self.run_number
        m.runtime_s  = (time.time() - self.run_start_ts) if self.run_active else 0.0
        m.strokes    = self.strokes
        m.coverage_m2 = self.coverage_m2
        if self.paint_start is not None and self.paint_last is not None:
            m.paint_consumed = max(0.0, self.paint_start - self.paint_last)
        m.dist_error_mean  = self.dist_err.mean
        m.angle_error_mean = self.angle_err.mean
        m.strip_z_min = 0.0 if self.strip_z_min ==  math.inf else self.strip_z_min
        m.strip_z_max = 0.0 if self.strip_z_max == -math.inf else self.strip_z_max
        m.fsm_state = self.fsm_state

        m.battery_soc     = self.batt_soc
        m.battery_voltage = self.batt_voltage
        m.battery_valid   = self.batt_valid

        lt = self.lifetime
        m.total_runs               = lt['total_runs']
        m.total_strokes            = lt['total_strokes']
        m.tip_cycles_total         = lt['tip_cycles_total']
        m.strokes_since_tip_change = lt['strokes_since_tip_change']
        m.runs_since_clean         = lt['runs_since_clean']
        m.total_runtime_hr         = lt['total_runtime_s'] / 3600.0
        m.total_coverage_m2        = lt['total_coverage_m2']

        self._stats_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = LoggingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # best-effort clean seal on Ctrl-C
        if node.run_active:
            node._seal_run(reason='CTRL_C')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
