"""
Stanley wall-following simulation for RevolutioNICE paint bot — v3.

GOAL OF THIS FILE
──────────────────
This sim exists to validate controller LOGIC before it gets ported into the
real `_do_step_realign()` in wall_motion_node.py. To make that port safe,
every variable the CONTROLLER touches is named and computed exactly the way
the real node computes it — same units, same frame, same order of operations.
The only code in this file that is NOT part of the real system is the
physics engine (kinematics + drift + noise), which exists purely to feed the
controller believable inputs. That physics code is kept in its own section
and never reuses a controller variable name for a different meaning.

v3 changes from v2 (in response to review):
  1. Split what used to be an overloaded `theta` into two explicit values:
       - `heading`      : the robot's true physical yaw, standard convention,
                           only used by the PHYSICS engine to move x/y.
       - `angle_error`   : the detector-convention value (CW+, nose-toward-
                           wall) that the CONTROLLER reads — exactly the
                           field name and sign convention wall_status
                           publishes on the real robot.
     These used to be the same variable, which is what forced a sign flip to
     be hidden inside the integration line. Now the only place a sign
     conversion happens is one named function, ROBOT_YAW_RATE_FROM_OMEGA(),
     called once per tick, clearly commented as a PHYSICS fact (how the
     bot's real yaw responds to a cmd.angular.z command), not a controller
     convention. The controller code itself never sees `heading` and the
     physics code never sees `angle_error` directly — they only meet through
     that one named conversion + the lidar geometry function.
  2. `TARGET_DIST_M` is relabeled for what it actually is: the LIDAR-frame
     calibration target — the raw lidar range wall_detection_node compares
     against to produce `dist_error` (matches `self.target_dist` in the real
     node). `dist_error` in this sim is now computed the same way:
     `dist_error = raw_lidar_reading - target_dist`, exactly mirroring the
     real detector, and `dist_error_corrected` is derived FROM that raw
     `dist_error` using the exact real formula — not re-derived independently
     from geometry. (Numerically this matches v2 at all angles, since the
     offset terms vanish at angle_error=0 where target_dist is calibrated —
     but the derivation now matches the real code line-for-line instead of
     "coincidentally agreeing".)

WHAT CARRIES OVER FROM v2 UNCHANGED
────────────────────────────────────
Closed-loop WALL_FOLLOW with two explicit gain pairs (weak/maintenance,
strong/acquisition+recovery), per-tick escalation when the corrected error
exceeds a threshold, Model A (uniform) / Model B (front-loaded) drift, sensor
noise, and the 10-cycle + injected-error escalation test.

Run:  python3 stanley_sim.py
"""

import math
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

rng = np.random.default_rng(42)

# Save relative to this script's own location, not the process cwd — so it
# works the same whether you run `python3 stanley_sim.py` from this folder
# or from anywhere else.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "outputs")
os.makedirs(OUTPUT_DIR, exist_ok=True)
OUTPUT_PNG = os.path.join(OUTPUT_DIR, "stanley_unifsim.png")

# ═══════════════════════════════════════════════════════════════════════════
#  SECTION 1 — CONTROLLER-FACING CONSTANTS
#  Every name here has a 1:1 counterpart in the real wall_motion_node.
# ═══════════════════════════════════════════════════════════════════════════

LIDAR_OFFSET_X = 0.195     # m — self.lidar_offset_x — lidar forward of turn axis
LIDAR_OFFSET_Y = -0.147    # m — self.lidar_offset_y — lidar right of turn axis (negative)

TARGET_DIST_M = 0.770      # m — self.target_dist — LIDAR-FRAME calibration target:
                            # the raw lidar range read when the robot is exactly
                            # centred (angle_error = 0). NOT a centre-distance value
                            # directly — it's what dist_error is measured against,
                            # same as wall_detection_node does it.

FOLLOW_SPEED = 0.120        # m/s — self.follow_speed
MAX_ANG      = 0.400        # rad/s — self.max_ang hard limit
STEP_SIZE_M  = 0.203        # m — self.move_step_size

# Two explicit gain pairs — same Stanley formula, different magnitude.
STEP_REALIGN_DIST_KP    = 2.5   # self.step_realign_dist_kp  (strong)
STEP_REALIGN_HEADING_KP = 1.0   # self.step_realign_heading_kp
FOLLOW_DIST_KP    = 1         # self.follow_dist_kp  (weak / maintenance)
FOLLOW_HEADING_KP = 0.5         # self.follow_heading_kp

# Escalation: mid-step, if |dist_error_corrected| exceeds this during
# WALL_FOLLOW, that control tick uses the strong (STEP_REALIGN) gain pair.
ESCALATION_THRESHOLD_M = 0.045   # 45 mm — self.escalation_threshold (proposed)


# ═══════════════════════════════════════════════════════════════════════════
#  SECTION 2 — CONTROLLER (this is the block that gets ported verbatim)
#  Inputs/outputs use ONLY real-robot-convention variables:
#    dist_error, angle_error  → in                     (detector convention)
#    dist_error_corrected, psi_cmd, heading_error, omega → out (ROS convention)
#  No physics, no simulation-only concept appears anywhere in this section.
# ═══════════════════════════════════════════════════════════════════════════

def compute_dist_error(raw_lidar_reading_m, target_dist_m=TARGET_DIST_M):
    """
    Mirrors wall_detection_node: dist_error is measured directly against the
    raw lidar reading, in the lidar frame, BEFORE any centre-offset
    correction. This is the value that arrives at _do_step_realign() as its
    `dist_error` argument on the real robot.
    """
    return raw_lidar_reading_m - target_dist_m


def stanley_correction(dist_error, angle_error, kd, kt,
                        lidar_offset_x=LIDAR_OFFSET_X, lidar_offset_y=LIDAR_OFFSET_Y,
                        max_ang=MAX_ANG):
    """
    Exact formula from the real _do_step_realign() Stanley branch — byte-for-
    byte the same as what's deployed, just with (kd, kt) as arguments instead
    of hardcoded self.step_realign_* so the same function serves both gain
    pairs.

        dist_error_corrected = dist_error
                                + lidar_offset_x * sin(angle_error)
                                + lidar_offset_y * (1 - cos(angle_error))
        psi_cmd       = atan(kd * dist_error_corrected)
        heading_error = angle_error - psi_cmd        (verified vs real logs)
        omega         = kt * heading_error            (cmd.angular.z, ROS CCW+)

    Convention (verified against detector + ROS, same as real code's docstring):
        dist_error  > 0  = too far from right wall
        angle_error > 0  = nose toward right wall (CW, detector convention)
        angular.z   > 0  = CCW = away from right wall (ROS convention)
    """
    dist_error_corrected = (
        dist_error
        + lidar_offset_x * math.sin(angle_error)
        + lidar_offset_y * (1.0 - math.cos(angle_error))
    )
    psi_cmd = math.atan(kd * dist_error_corrected)
    heading_error = angle_error - psi_cmd
    omega = kt * heading_error
    omega = max(-max_ang, min(max_ang, omega))
    return omega, dist_error_corrected, psi_cmd, heading_error


def select_gains(dist_error_corrected, first_step_after_align):
    """
    Gain-scheduling / escalation policy (the part v2 adds on top of the
    existing controller): strong gains on the first WALL_FOLLOW step after a
    WALL_ALIGN (acquisition), or any tick where the corrected error exceeds
    the escalation threshold (recovery); weak/maintenance gains otherwise.
    Returns (kd, kt, used_strong).
    """
    use_strong = first_step_after_align or (abs(dist_error_corrected) > ESCALATION_THRESHOLD_M)
    if use_strong:
        return STEP_REALIGN_DIST_KP, STEP_REALIGN_HEADING_KP, True
    return FOLLOW_DIST_KP, FOLLOW_HEADING_KP, False


# ═══════════════════════════════════════════════════════════════════════════
#  SECTION 3 — PHYSICS ENGINE (simulation-only — nothing here is ported)
#  Owns `heading` (true yaw, standard convention) and world (x, y).
#  Talks to the controller ONLY through:
#    - lidar_geometry()      : heading, true y  →  raw_lidar_reading, true d_center
#    - ROBOT_YAW_RATE_FROM_OMEGA() : omega (controller output) → d(heading)/dt
# ═══════════════════════════════════════════════════════════════════════════

# The controller's `omega` is cmd.angular.z, ROS CCW+, and by the VERIFIED
# real convention CCW = away from the (right-side) wall. This sim's `heading`
# is standard yaw (CCW+ = turns toward +y). We place the wall at y=0 with the
# robot at y>0, i.e. the wall is in the -y direction from the robot — so a
# standard-convention CCW turn (heading increasing) points the nose toward
# +y, i.e. AWAY from the wall. That already matches the real convention with
# NO flip needed — this constant exists only to make that fact visible and
# named, instead of leaving readers to rediscover it by tracing signs.
ROBOT_YAW_RATE_FROM_OMEGA = +1.0   # d(heading)/dt = ROBOT_YAW_RATE_FROM_OMEGA * omega


def lidar_geometry(true_y, heading, x_L=LIDAR_OFFSET_X, y_L=LIDAR_OFFSET_Y):
    """
    True (noise-free) geometry. Wall at y=0, robot at y>0 (wall is in the
    -y / right-hand direction from the robot, matching the real right-wall
    setup — see ROBOT_YAW_RATE_FROM_OMEGA comment above).

    angle_error (detector convention, CW+/nose-toward-wall) is the NEGATIVE
    of standard-convention heading, because turning the nose toward the wall
    (which is in -y) is a CW/negative-heading rotation in standard convention.
    This conversion happens HERE, once, at the sensor boundary — nowhere else.
    """
    true_d_center = true_y                      # distance from wall, wall at y=0
    angle_error = -heading
    raw_lidar_reading = (
        true_d_center
        - x_L * math.sin(angle_error)
        - y_L * (1.0 - math.cos(angle_error))
    )
    return raw_lidar_reading, true_d_center, angle_error


def drift_increment(model, t_in_step, step_dur, dt, magnitude, b_fraction):
    """
    Lateral (y) displacement applied this tick due to unmodeled mechanical /
    transient drift. Drift always pushes the robot AWAY from the wall
    (increasing y further, since wall is at y=0 and robot at y>0), matching
    what was observed on the real robot.
    """
    if model == "A":
        # uniform: spread evenly over the whole step duration
        return +(magnitude / step_dur) * dt
    elif model == "B":
        # front-loaded: dumped into the first `b_fraction` of the step
        active_dur = step_dur * b_fraction
        if t_in_step <= active_dur:
            return +(magnitude / active_dur) * dt
        return 0.0
    else:
        raise ValueError(f"unknown drift model {model!r}")


# ═══════════════════════════════════════════════════════════════════════════
#  SECTION 4 — SIM CONFIG (drift / noise / test schedule — not controller state)
# ═══════════════════════════════════════════════════════════════════════════

DIST_NOISE_STD_M    = 0.0015                 # 1.5 mm — matches observed rest-jitter
ANGLE_NOISE_STD_RAD = math.radians(0.15)      # small angle jitter

DRIFT_PER_STEP_M = 0.001   # 10 mm/step total injected drift magnitude
DRIFT_B_FRACTION = 0.15    # Model B: fraction of step duration carrying the drift

INIT_DIST_ERROR_M    = +0.000    # initial dist_error (lidar-frame) at cycle start
INIT_ANGLE_ERROR_DEG = +3.0

N_CYCLES        = 10       # steady-state cycles before the injected-error test
N_EXTRA_CYCLES  = 4        # cycles run after the large-error injection
INJECT_ERROR_M  = 0.070    # 70 mm sudden lateral error injected after cycle N_CYCLES
DT              = 0.05     # control timestep (s)


# ═══════════════════════════════════════════════════════════════════════════
#  SECTION 5 — ONE CLOSED-LOOP STEP (glues controller + physics together)
# ═══════════════════════════════════════════════════════════════════════════

def closed_loop_step(x0, true_y0, heading0, drift_model, first_step_after_align=False,
                      add_noise=True, inject_error_m=0.0):
    """
    Drive one strip width (STEP_SIZE_M) forward. Every tick:
      1. PHYSICS reports true state → lidar_geometry() → raw_lidar_reading, angle_error
      2. sensor noise added (simulation-only)
      3. CONTROLLER: compute_dist_error() → select_gains() → stanley_correction()
      4. PHYSICS integrates x, true_y (+ drift), heading (from omega via
         ROBOT_YAW_RATE_FROM_OMEGA)
    """
    x, true_y, heading = x0, true_y0 + inject_error_m, heading0
    traj = [(x, true_y, heading)]
    gain_log = []      # 'strong' / 'weak' per tick
    err_log = []        # dist_error_corrected (mm) per tick, controller's noisy estimate
    escalated_this_step = False

    step_dur = STEP_SIZE_M / FOLLOW_SPEED
    t = 0.0
    max_ticks = int(20 * step_dur / DT) + 200
    ticks = 0

    while (x - x0) < STEP_SIZE_M:
        ticks += 1
        if ticks > max_ticks:
            print(f"  [WARN] step did not terminate within {max_ticks} ticks — "
                  f"controller unstable at this gain/drift combo; breaking.")
            break

        # 1) physics → true geometry
        raw_lidar_true, true_d_center, angle_error_true = lidar_geometry(true_y, heading)

        # 2) sensor noise (sim-only)
        raw_lidar_meas = raw_lidar_true + (rng.normal(0.0, DIST_NOISE_STD_M) if add_noise else 0.0)
        angle_error_meas = angle_error_true + (rng.normal(0.0, ANGLE_NOISE_STD_RAD) if add_noise else 0.0)

        # 3) controller (real-code-identical path)
        dist_error = compute_dist_error(raw_lidar_meas)
        # need dist_error_corrected to pick gains, so do a cheap pre-pass with
        # weak gains just to get the corrected-error estimate (kd doesn't
        # affect dist_error_corrected itself — it's computed before kd is used)
        dist_error_corrected_preview = (
            dist_error
            + LIDAR_OFFSET_X * math.sin(angle_error_meas)
            + LIDAR_OFFSET_Y * (1.0 - math.cos(angle_error_meas))
        )
        kd, kt, used_strong = select_gains(dist_error_corrected_preview, first_step_after_align)
        omega, dist_error_corrected, psi_cmd, heading_error = stanley_correction(
            dist_error, angle_error_meas, kd, kt)

        if used_strong and not first_step_after_align:
            escalated_this_step = True

        # 4) physics integrates true state
        x += FOLLOW_SPEED * math.cos(heading) * DT
        true_y += FOLLOW_SPEED * math.sin(heading) * DT
        true_y += drift_increment(drift_model, t, step_dur, DT, DRIFT_PER_STEP_M, DRIFT_B_FRACTION)
        heading += ROBOT_YAW_RATE_FROM_OMEGA * omega * DT
        t += DT

        traj.append((x, true_y, heading))
        gain_log.append('strong' if used_strong else 'weak')
        err_log.append(dist_error_corrected * 1000.0)

    return traj, x, true_y, heading, gain_log, err_log, escalated_this_step


# ═══════════════════════════════════════════════════════════════════════════
#  SECTION 6 — RUN / REPORT / PLOT
# ═══════════════════════════════════════════════════════════════════════════

def run_sim(drift_model, label):
    true_y0 = TARGET_DIST_M + INIT_DIST_ERROR_M
    x0 = 0.0
    heading0 = -math.radians(INIT_ANGLE_ERROR_DEG)   # angle_error = -heading

    x, true_y, heading = x0, true_y0, heading0
    all_traj = [(x, true_y, heading)]
    cycle_rows = []      # (cycle, d_center_start_mm, d_center_end_mm, angle_error_end_deg, escalated)
    all_gain_log, all_err_log = [], []

    print("=" * 72)
    print(f"  Closed-loop WALL_FOLLOW Stanley sim — drift model {drift_model} ({label})")
    print(f"  weak  gains: kd={FOLLOW_DIST_KP}  kt={FOLLOW_HEADING_KP}")
    print(f"  strong gains: kd={STEP_REALIGN_DIST_KP}  kt={STEP_REALIGN_HEADING_KP}")
    print(f"  escalation threshold: {ESCALATION_THRESHOLD_M*1000:.0f} mm")
    print(f"  drift magnitude: {DRIFT_PER_STEP_M*1000:.0f} mm/step "
          f"({'uniform' if drift_model=='A' else f'front-loaded in first {DRIFT_B_FRACTION*100:.0f}% of step'})")
    print("=" * 72)

    total_cycles = N_CYCLES + N_EXTRA_CYCLES

    for cyc in range(1, total_cycles + 1):
        first_step = (cyc == 1)
        inject = INJECT_ERROR_M if cyc == N_CYCLES + 1 else 0.0

        _, d_cen_start, _ = lidar_geometry(true_y, heading)

        traj, x, true_y, heading, gain_log, err_log, escalated = closed_loop_step(
            x, true_y, heading, drift_model, first_step_after_align=first_step,
            inject_error_m=inject)
        all_traj.extend(traj)
        all_gain_log.extend(gain_log)
        all_err_log.extend(err_log)

        _, d_cen_end, angle_err_end = lidar_geometry(true_y, heading)

        cycle_rows.append((cyc, d_cen_start*1000, d_cen_end*1000,
                            math.degrees(angle_err_end), escalated))

        tag = f" <<< {INJECT_ERROR_M*1000:.0f}mm error injected at step start" if inject else ""
        esc_tag = "  [ESCALATED to strong gains mid-step]" if escalated else ""
        print(f"Cycle {cyc:2d}  centre {d_cen_start*1000:7.1f} → {d_cen_end*1000:7.1f} mm  "
              f"(err {d_cen_end*1000 - TARGET_DIST_M*1000:+6.1f} mm)  "
              f"angle_err_end {math.degrees(angle_err_end):+5.1f}°{esc_tag}{tag}")

    print()
    steady_err = [r[2] - TARGET_DIST_M*1000 for r in cycle_rows[:N_CYCLES]]
    print(f"  steady-state (cycles 1-{N_CYCLES}) end-of-step centre error: "
          f"mean={np.mean(steady_err):+.1f}mm  std={np.std(steady_err):.1f}mm  "
          f"max|.|={np.max(np.abs(steady_err)):.1f}mm")

    post_inject = [r for r in cycle_rows if r[0] > N_CYCLES]
    esc_fired = any(r[4] for r in post_inject)
    final_err = post_inject[-1][2] - TARGET_DIST_M*1000
    print(f"  after injected {INJECT_ERROR_M*1000:.0f}mm error: escalation fired = {esc_fired}  "
          f"final centre error after {N_EXTRA_CYCLES} recovery cycles = {final_err:+.1f}mm")
    print()

    return dict(traj=np.array(all_traj), cycle_rows=cycle_rows,
                gain_log=all_gain_log, err_log=all_err_log, label=label)


def run():
    result_a = run_sim("A", "uniform drift")
    result_b = run_sim("B", "front-loaded / step-input transient drift")

    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(
        "Closed-loop Stanley WALL_FOLLOW — Model A (uniform) vs Model B (front-loaded) drift\n"
        f"weak(kd={FOLLOW_DIST_KP},kt={FOLLOW_HEADING_KP})  "
        f"strong(kd={STEP_REALIGN_DIST_KP},kt={STEP_REALIGN_HEADING_KP})  "
        f"escalation@{ESCALATION_THRESHOLD_M*1000:.0f}mm  "
        f"drift={DRIFT_PER_STEP_M*1000:.0f}mm/step  inject={INJECT_ERROR_M*1000:.0f}mm@cycle{N_CYCLES+1}",
        fontsize=10)
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.35, wspace=0.28)
    colors = {"A": "#1f77b4", "B": "#d62728"}

    ax0 = fig.add_subplot(gs[0, 0])
    for res, key in [(result_a, "A"), (result_b, "B")]:
        rows = res['cycle_rows']
        cycs = [r[0] for r in rows]
        end_err = [r[2] - TARGET_DIST_M*1000 for r in rows]
        ax0.plot(cycs, end_err, 'o-', color=colors[key], label=f"Model {key}")
        esc_c = [r[0] for r in rows if r[4]]
        esc_e = [r[2] - TARGET_DIST_M*1000 for r in rows if r[4]]
        if esc_c:
            ax0.scatter(esc_c, esc_e, marker='*', s=160, color=colors[key],
                       edgecolor='k', zorder=5, label=f"Model {key} escalated")
    ax0.axhline(0, color='gray', lw=1, ls='--')
    ax0.axvline(N_CYCLES + 0.5, color='black', lw=1, ls=':', label='error injected')
    ax0.axhspan(-ESCALATION_THRESHOLD_M*1000, ESCALATION_THRESHOLD_M*1000, color='green', alpha=0.05)
    ax0.set_xlabel("Cycle"); ax0.set_ylabel("End-of-step centre dist error (mm)")
    ax0.set_title("Centre distance error per cycle (primary tracked parameter)")
    ax0.legend(fontsize=7); ax0.grid(True, alpha=0.3)

    ax1 = fig.add_subplot(gs[0, 1])
    gl = result_b['gain_log']
    strong_mask = np.array([1 if g == 'strong' else 0 for g in gl])
    ax1.fill_between(range(len(strong_mask)), 0, strong_mask, step='pre',
                     color='#d62728', alpha=0.6, label='strong gains active')
    ax1.set_ylim(-0.1, 1.1)
    ax1.set_xlabel("control tick"); ax1.set_ylabel("strong-gain flag")
    ax1.set_title("Gain schedule over time — Model B\n(step1 acquisition + escalation events)")
    ax1.legend(fontsize=8); ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(gs[1, 0])
    arr = result_b['traj']
    xs, ys = arr[:, 0], arr[:, 1]
    ax2.plot(xs, ys, lw=1.0, color=colors["B"])
    ax2.axhline(TARGET_DIST_M, color='red', lw=1.2, ls='--', label='target standoff')
    ax2.set_xlabel("Along-wall x (m)"); ax2.set_ylabel("Distance from wall (m)")
    ax2.set_title("Top-view trajectory — Model B (front-loaded drift)")
    ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(result_a['err_log'], color=colors["A"], lw=0.8, alpha=0.8, label='Model A')
    ax3.plot(result_b['err_log'], color=colors["B"], lw=0.8, alpha=0.8, label='Model B')
    ax3.axhline(0, color='gray', lw=1, ls='--')
    ax3.axhline(ESCALATION_THRESHOLD_M*1000, color='k', lw=0.8, ls=':')
    ax3.axhline(-ESCALATION_THRESHOLD_M*1000, color='k', lw=0.8, ls=':')
    ax3.set_xlabel("control tick (all cycles concatenated)")
    ax3.set_ylabel("dist_error_corrected (mm)")
    ax3.set_title("Per-tick centre error, controller's noisy estimate")
    ax3.legend(fontsize=8); ax3.grid(True, alpha=0.3)

    plt.savefig(OUTPUT_PNG, dpi=150, bbox_inches='tight')
    print(f"Plot saved → {OUTPUT_PNG}")


if __name__ == '__main__':
    run()