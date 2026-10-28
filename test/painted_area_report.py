#!/usr/bin/env python3
"""Painted-area report for a real_bot_bench run: how much wall was actually painted until the robot stopped.

Run it in a second terminal while `ros2 launch painting_robot_bringup real_bot_bench.launch.py` is up
(same ROS_DOMAIN_ID), BEFORE pressing START:

    python3 painted_area_report.py                 # report at the first stop (HOLD / FAULT / ESTOP / back to IDLE / stall)
    python3 painted_area_report.py --keep-running  # log every stop as an event, report on Ctrl-C
    python3 painted_area_report.py --out ~/reports --swath 0.203

It only listens (no publishers, never commands the robot). Output goes to <out>/run_<time>/ :
report.md, summary.json, strokes.csv and (if matplotlib is installed) plan_view.png.

HOW THE AREA IS COMPUTED  (one "stroke" = one gantry up/down sweep: the Z axis starts moving, then stops)
 1. Height swept   = the gantry's TOTAL TRAVEL from homing: taught upper limit - taught lower limit
                     (/paint_head_status.z_soft_upper_mm - z_soft_lower_mm, valid once homed; the last valid value
                     is kept, so an ESTOP that clears the limits does not lose it). A stroke that swept at least
                     --full-frac (90 %) of that travel counts as the full travel; a stroke cut short by a stop
                     counts only the part it really swept (max-min of z_actual_pos_mm while the gantry was moving).
                     If homing was never seen, the measured sweep is used (or --gantry-travel if you give it).
 2. Width painted  = --swath (0.203 m, the wall-follow step) + the distance the robot moved along the wall during
                     the stroke (corner/bump strokes happen while the robot is rotating or creeping).
 3. No wall check. Every gantry sweep counts as a stroke. The lidar is only used to place the stroke on the
                     map (for overlap removal): if no fresh wall reading exists the nominal wall distance
                     (--target-dist) is assumed. Whether a wall was really seen is only logged (wall_samples).
 4. Where on the wall: contact point = robot axis (odom) + right-normal * (lidar_right + lidar_dist), segment of the
                     stroke width along the robot heading. Segments are rasterised on a 1 cm grid in plan view; each
                     cell remembers the Z range already painted, so overlapping strokes are counted ONCE.
                     unique area = painted once; duplicate area = on-wall area - unique area.
 5. Stop detection: /robot_state containing HOLD / FAULT / ESTOP, /safety_command tier>=1, any /fault_report code,
                     robot back in IDLE after it started, or no robot motion, no gantry motion and no spray for --stall-s seconds.
                     A stroke still open at the stop is closed at that moment with what it had swept.
 Production rate = unique area / time from START to the stop.
 Cross-check: paint_level (0..1 sensor) at START vs. at the stop.
Limits: wheel-odometry positions (drift over long runs), 1 cm grid on diagonal walls is +-40 % per stroke edge but
unbiased in total, wall tangent = robot heading (a few degrees off while aligning).
"""
import argparse
import csv
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from painting_robot_msgs.msg import WallStatus, SafetyCommand, SprayCmd, PaintHeadStatus

STOP_WORDS = ('HOLD', 'FAULT', 'ESTOP', 'E_STOP', 'E-STOP')
IDLE_WORDS = ('IDLE', 'DONE', 'COMPLETE', 'FINISH')
TIER_NAMES = {1: 'SAFETY_HOLD', 2: 'FAULT', 3: 'ESTOP'}
CELL = 0.01          # plan-view grid cell, m
SAMPLE = 0.005       # sampling step along a stroke segment, m


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AreaReport(Node):
    def __init__(self, a):
        super().__init__('painted_area_report')
        self.a = a
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=20)
        self.create_subscription(Odometry, a.odom_topic, self.odom_cb, be)
        self.create_subscription(WallStatus, '/wall_status', self.wall_cb, be)
        self.create_subscription(PaintHeadStatus, '/paint_head_status', self.head_cb, be)
        self.create_subscription(SprayCmd, '/spray_cmd', self.spray_cb, 10)
        self.create_subscription(String, '/robot_state', self.state_cb, 10)
        self.create_subscription(String, '/start_cmd', self.start_cb, 10)
        self.create_subscription(SafetyCommand, '/safety_command', self.safety_cb, 10)
        self.create_subscription(String, '/fault_report', self.fault_cb, 10)
        self.create_timer(0.2, self.tick)

        self.pose = None            # (x, y, yaw)
        self.wall = None            # (t, valid, lidar_dist)
        self.z_mm = None
        self.travel_m = None        # gantry total travel from homing (taught upper - lower), m
        self.z_soft = (None, None)
        self.paint_level = None
        self.state = 'UNKNOWN'
        self.started = False
        self.t_start = None         # monotonic
        self.t_stop = None
        self.stop_deadline = None
        self.finished = False
        self.events = []            # (t_rel, kind, detail)
        self.stop_reason = None
        self.level_start = None
        self.level_stop = None

        self.path_len = 0.0
        self.last_pose = None
        self.last_motion_t = time.monotonic()
        self.last_spray_t = time.monotonic()
        self.z_prev = None
        self.last_z_move_t = 0.0    # monotonic time of the last gantry movement
        self.spray_on = False

        self.stroke = None          # dict while the gantry is sweeping
        self.strokes = []           # finished stroke dicts
        self.grid = {}              # (ix, iy) -> [zlo, zhi]
        self.segments = []          # (x0, y0, x1, y1, on_wall, unique_frac)

    # ------------------------------------------------------------------ helpers
    def trel(self):
        return 0.0 if self.t_start is None else time.monotonic() - self.t_start

    def log_event(self, kind, detail):
        self.events.append((round(self.trel(), 2), kind, detail))
        self.get_logger().info(f'[{self.trel():7.1f}s] {kind}: {detail}')

    def contact(self):
        """Wall contact point (assumed nominal distance if no wall is seen) for the current pose, or None."""
        if self.pose is None:
            return None
        t, valid, dist = self.wall if self.wall is not None else (0.0, False, self.a.target_dist)
        seen = valid and time.monotonic() - t <= self.a.wall_fresh_s
        if not seen:
            dist = self.a.target_dist                # no wall reading: assume the nominal wall distance
        x, y, yaw = self.pose
        nx, ny = math.sin(yaw), -math.cos(yaw)           # right-hand normal
        reach = dist + self.a.lidar_right - self.a.head_right
        d_axis = self.a.lidar_right + dist
        return (x + nx * d_axis, y + ny * d_axis, yaw, reach, seen)

    # ------------------------------------------------------------------ callbacks
    def odom_cb(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.last_pose is not None:
            d = math.hypot(self.pose[0] - self.last_pose[0], self.pose[1] - self.last_pose[1])
            if d > 0.002 or abs(math.remainder(self.pose[2] - self.last_pose[2], 2 * math.pi)) > 0.01:
                self.last_motion_t = time.monotonic()
            if self.started and not self.finished:
                self.path_len += d
        self.last_pose = self.pose
        if self.stroke is not None:
            self.stroke['n'] += 1
            c = self.contact()
            if c:
                self.stroke['pts'].append(c)

    def wall_cb(self, m):
        # distance_error = lidar_dist - target; target is a launch parameter (default 0.77 m)
        self.wall = (time.monotonic(), bool(m.wall_valid), self.a.target_dist + float(m.distance_error))

    def head_cb(self, m):
        self.z_mm = float(m.z_actual_pos_mm)
        self.paint_level = float(m.paint_level)
        lo, hi = float(m.z_soft_lower_mm), float(m.z_soft_upper_mm)
        if hi - lo > 50.0:                          # homed: taught limits valid; keep the last valid pair
            self.z_soft = (lo, hi)
            t = (hi - lo) / 1000.0
            if self.travel_m is None or abs(t - self.travel_m) > 0.001:
                self.log_event('homing', f'gantry travel = {t:.3f} m (lower {lo:.1f} mm, upper {hi:.1f} mm)')
            self.travel_m = t
        if self.started and self.level_start is None:
            self.level_start = self.paint_level
        now = time.monotonic()
        moving = self.z_prev is not None and abs(self.z_mm - self.z_prev) >= self.a.z_move_mm
        if moving:
            self.last_z_move_t = now
            if self.stroke is None and self.started and not self.finished:
                z0 = self.z_prev
                self.stroke = dict(t0=self.trel(), zlo=min(z0, self.z_mm), zhi=max(z0, self.z_mm), pts=[], n=0,
                                   spray=False, pose0=self.pose)
                c = self.contact()
                self.stroke['n'] += 1
                if c:
                    self.stroke['pts'].append(c)
        self.z_prev = self.z_mm
        if self.stroke is not None:
            s = self.stroke
            s['zlo'] = min(s['zlo'], self.z_mm)
            s['zhi'] = max(s['zhi'], self.z_mm)

    def spray_cb(self, m):
        self.spray_on = bool(m.valve_open)
        if self.spray_on:
            self.last_spray_t = time.monotonic()
            if self.stroke is not None:
                self.stroke['spray'] = True      # informational only: a gantry sweep is a stroke either way

    def start_cb(self, m):
        if 'START' in m.data.upper() and not self.started:
            self.begin('/start_cmd ' + m.data)

    def state_cb(self, m):
        new = m.data.upper()
        old, self.state = self.state, new
        if new == old:
            return
        self.log_event('state', f'{old} -> {new}')
        if not self.started:
            if new not in ('UNKNOWN',) + IDLE_WORDS and not any(w in new for w in STOP_WORDS) and new != 'SYSTEM_CHECK':
                self.begin('state ' + new)
            return
        if any(w in new for w in STOP_WORDS):
            self.on_stop('STATE', new)
        elif any(w == new for w in IDLE_WORDS):
            self.on_stop('IDLE', f'robot returned to {new} after running')

    def safety_cb(self, m):
        if int(m.tier) >= 1 and self.started:
            self.on_stop(TIER_NAMES.get(int(m.tier), f'tier{m.tier}'), f'safety_command reason="{m.reason}"')

    def fault_cb(self, m):
        if m.data.startswith('CLEAR:'):
            return
        self.log_event('fault_report', m.data)
        if self.started:
            self.on_stop('FAULT', f'fault_report {m.data}')

    # ------------------------------------------------------------------ run control
    def begin(self, why):
        if self.started:
            return
        self.started = True
        self.t_start = time.monotonic()
        self.last_motion_t = self.last_spray_t = time.monotonic()
        self.level_start = self.paint_level
        self.events.append((0.0, 'START', why))
        self.get_logger().info(f'run started ({why}) - logging painted area')

    def on_stop(self, kind, detail):
        self.log_event('STOP', f'{kind}: {detail}')
        if self.stop_reason is None:
            self.stop_reason = (round(self.trel(), 2), kind, detail)
            self.t_stop = time.monotonic()
            if not self.a.keep_running:
                self.stop_deadline = time.monotonic() + self.a.grace_s   # let a closing stroke / fault text arrive

    def tick(self):
        if not self.started or self.finished:
            return
        now = time.monotonic()
        if self.stroke is not None and now - self.last_z_move_t > self.a.z_idle_s:
            self.close_stroke('gantry stopped')
        if (self.stop_reason is None and self.state not in ('HOLD',)
                and now - self.last_motion_t > self.a.stall_s
                and now - max(self.last_spray_t, self.last_z_move_t) > self.a.stall_s):
            self.on_stop('STALL', f'no robot motion, no gantry motion and no spray for {self.a.stall_s:.0f} s in state {self.state}')
        if self.stop_deadline is not None and now >= self.stop_deadline:
            self.finish()

    # ------------------------------------------------------------------ stroke -> area
    def close_stroke(self, why):
        s, self.stroke = self.stroke, None
        pts = s['pts']
        swept = max(0.0, (s['zhi'] - s['zlo']) / 1000.0)
        travel = self.travel_m if self.travel_m else (self.a.gantry_travel or None)
        if travel and swept >= self.a.full_frac * travel:
            height = travel                          # full stroke = the homed total travel
        else:
            height = min(swept, travel) if travel else swept
        rec = dict(idx=len(self.strokes) + 1, t_start=round(s['t0'], 2), t_end=round(self.trel(), 2), end_reason=why,
                   z_lo_m=round(s['zlo'] / 1000.0, 3), z_hi_m=round(s['zhi'] / 1000.0, 3), swept_m=round(swept, 3), height_m=round(height, 3),
                   spray_on=bool(s.get('spray')), wall_samples=f'{sum(1 for p in pts if p[4])}/{s["n"]}', on_wall=False, reach_m=None, width_m=0.0, nominal_m2=0.0, unique_m2=0.0, x=None, y=None)
        if height < self.a.min_height:
            rec['note'] = 'no Z sweep'
        elif not pts:
            rec['note'] = 'no robot pose'
        else:
            reach = sum(p[3] for p in pts) / len(pts)
            rec['reach_m'] = round(reach, 3)
            if True:
                travel = sum(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) for i in range(1, len(pts)))
                width = self.a.swath + travel
                yaw = math.atan2(sum(math.sin(p[2]) for p in pts), sum(math.cos(p[2]) for p in pts))
                cx = sum(p[0] for p in pts) / len(pts)
                cy = sum(p[1] for p in pts) / len(pts)
                tx, ty = math.cos(yaw), math.sin(yaw)
                x0, y0, x1, y1 = cx - tx * width / 2, cy - ty * width / 2, cx + tx * width / 2, cy + ty * width / 2
                frac = self.add_to_grid(x0, y0, x1, y1, s['zlo'] / 1000.0, s['zlo'] / 1000.0 + height)
                nominal = width * height
                rec.update(on_wall=True, width_m=round(width, 3), nominal_m2=round(nominal, 4),
                           unique_m2=round(nominal * frac, 4), x=round(cx, 3), y=round(cy, 3))
                self.segments.append((x0, y0, x1, y1, True, frac))
        if not rec['on_wall'] and pts:
            c = pts[len(pts) // 2]
            self.segments.append((c[0], c[1], c[0], c[1], False, 0.0))
        self.strokes.append(rec)

    def add_to_grid(self, x0, y0, x1, y1, zlo, zhi):
        """Rasterise one segment; return the fraction of it (by cell-height) not painted before."""
        L = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(L / SAMPLE))
        cells = {(int(math.floor((x0 + (x1 - x0) * i / n) / CELL)), int(math.floor((y0 + (y1 - y0) * i / n) / CELL)))
                 for i in range(n + 1)}
        h = max(1e-6, zhi - zlo)
        gain = 0.0
        for c in cells:
            old = self.grid.get(c)
            if old is None:
                self.grid[c] = [zlo, zhi]
                gain += h
            else:
                lo, hi = min(old[0], zlo), max(old[1], zhi)
                gain += max(0.0, (hi - lo) - (old[1] - old[0])) if (zlo <= old[1] and zhi >= old[0]) else h
                self.grid[c] = [lo, hi]
        return min(1.0, gain / (h * len(cells)))

    # ------------------------------------------------------------------ report
    def finish(self):
        if self.finished:
            return
        if self.stroke is not None:
            self.close_stroke('robot stopped mid-stroke')
        self.finished = True
        self.level_stop = self.paint_level
        if self.stop_reason is None:
            self.stop_reason = (round(self.trel(), 2), 'MANUAL', 'script ended (Ctrl-C) without a stop event')
        self.write_report()

    def write_report(self):
        t_end = self.t_stop - self.t_start if self.t_stop else self.trel()
        t_end = max(t_end, 1e-6)
        on = [s for s in self.strokes if s['on_wall']]
        off = [s for s in self.strokes if not s['on_wall']]
        nominal = sum(s['nominal_m2'] for s in on)
        unique = sum(s['unique_m2'] for s in on)
        dup = nominal - unique
        res = dict(stop_time_s=round(t_end, 1), stop_reason=self.stop_reason, strokes_total=len(self.strokes),
                   strokes_on_wall=len(on), strokes_off_wall=len(off), on_wall_area_m2=round(nominal, 3),
                   unique_painted_area_m2=round(unique, 3), duplicate_area_m2=round(dup, 3),
                   duplicate_pct=round(100 * dup / nominal, 1) if nominal > 0 else 0.0,
                   production_rate_m2_per_h=round(unique / t_end * 3600, 1), path_length_m=round(self.path_len, 2),
                   gantry_travel_m=self.travel_m, paint_level_start=self.level_start, paint_level_stop=self.level_stop,
                   events=self.events)
        out = os.path.join(os.path.expanduser(self.a.out), time.strftime('run_%Y%m%d_%H%M%S'))
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, 'summary.json'), 'w') as f:
            json.dump(res, f, indent=2)
        if self.strokes:
            with open(os.path.join(out, 'strokes.csv'), 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list({k: 0 for s in self.strokes for k in s}))
                w.writeheader()
                w.writerows(self.strokes)
        lv = ''
        if self.level_start is not None and self.level_stop is not None:
            lv = f'\n- Paint level sensor: {self.level_start:.2f} -> {self.level_stop:.2f} (cross-check only)'
        md = [f'# Painted-area report ({time.strftime("%Y-%m-%d %H:%M:%S")})', '',
              f'**Stopped after {t_end:.0f} s ({t_end / 60:.1f} min)** - {self.stop_reason[1]}: {self.stop_reason[2]}', '',
              '## Result',
              f'- **Unique painted wall area: {unique:.2f} m^2**',
              f'- On-wall paint commanded: {nominal:.2f} m^2, of which duplicate (overlap): {dup:.2f} m^2 ({res["duplicate_pct"]} %)',
              f'- Strokes: {len(self.strokes)} total, {len(on)} on a wall, {len(off)} not counted (no Z sweep / no pose)',
              f'- Production rate: {res["production_rate_m2_per_h"]} m^2/h (unique area / time since START)',
              f'- Gantry total travel (from homing): ' + (f'{self.travel_m:.3f} m' if self.travel_m else 'not seen - measured sweep used'),
              f'- Robot path length: {self.path_len:.2f} m' + lv, '',
              '## Method', 'See the docstring of painted_area_report.py: homed gantry travel x (swath + robot travel) per stroke, '
              'every gantry sweep counted, overlaps removed on a 1 cm plan-view grid.', '',
              '## Events', '| t (s) | event | detail |', '|---|---|---|']
        md += [f'| {t} | {k} | {d} |' for t, k, d in self.events]
        if off:
            md += ['', '## Strokes not counted', '| # | t | reason |', '|---|---|---|']
            md += [f'| {s["idx"]} | {s["t_start"]} | {s.get("note", "")} |' for s in off]
        with open(os.path.join(out, 'report.md'), 'w') as f:
            f.write('\n'.join(md) + '\n')
        self.plot(out)
        print('\n'.join(md[:12]))
        print(f'\nFiles written to {out}')

    def plot(self, out):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except Exception:
            return
        fig, ax = plt.subplots(figsize=(7, 7))
        for x0, y0, x1, y1, on, frac in self.segments:
            if on:
                ax.plot([x0, x1], [y0, y1], '-', color='tab:green' if frac > 0.5 else 'tab:orange', lw=3)
            else:
                ax.plot([x0], [y0], 'x', color='tab:red')
        if self.pose:
            ax.plot([self.pose[0]], [self.pose[1]], 'k*', ms=14, label='robot at stop')
        ax.set_aspect('equal')
        ax.grid(alpha=.3)
        ax.set_title('Plan view (odom frame): green = new paint, orange = mostly overlap, x = not counted')
        ax.legend(loc='best')
        fig.savefig(os.path.join(out, 'plan_view.png'), dpi=120, bbox_inches='tight')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='~/painting_reports')
    ap.add_argument('--odom-topic', default='/paint_bot/odom_filtered')
    ap.add_argument('--swath', type=float, default=0.203, help='paint width per stroke along the wall, m')
    ap.add_argument('--target-dist', type=float, default=0.77, help='wall_status target lidar-to-wall distance, m')
    ap.add_argument('--lidar-right', type=float, default=0.147, help='lidar offset to the right of the turn axis, m')
    ap.add_argument('--head-right', type=float, default=0.312, help='paint head offset to the right of the turn axis, m')
    ap.add_argument('--gantry-travel', type=float, default=0.0, help='gantry total travel in m, used only if homing limits are never seen')
    ap.add_argument('--full-frac', type=float, default=0.9, help='a stroke sweeping at least this fraction of the travel counts as the full travel')
    ap.add_argument('--z-move-mm', type=float, default=2.0, help='Z change between two status samples that counts as the gantry moving, mm')
    ap.add_argument('--z-idle-s', type=float, default=0.6, help='gantry still for this long ends a stroke, s')
    ap.add_argument('--min-height', type=float, default=0.05, help='min measured Z sweep to count a stroke, m')
    ap.add_argument('--wall-fresh-s', type=float, default=1.5)
    ap.add_argument('--stall-s', type=float, default=60.0, help='no robot motion, no gantry motion and no spray for this long = stopped')
    ap.add_argument('--grace-s', type=float, default=1.5, help='keep listening this long after the first stop event')
    ap.add_argument('--keep-running', action='store_true', help='do not exit at the first stop; report on Ctrl-C')
    a = ap.parse_args()
    rclpy.init()
    node = AreaReport(a)
    print('Waiting for START... (Ctrl-C writes the report with what was logged so far)')
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    if not node.finished:
        if not node.started:
            print('Run never started - nothing to report.')
        else:
            node.finish()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
