"""
Stanley wall-following simulation for RevolutioNICE paint bot.

Coordinate frame (clean, unambiguous):
  - Wall is at y = 0
  - Robot is on the negative-y side (y < 0)
  - positive dist_error = robot too far from wall (|y| > TARGET_DIST_M)
  - positive theta = nose angled toward wall (y increasing = toward y=0)
  - positive angular.z = CCW = away from wall in this frame
  - Stanley: psi_cmd > 0 when too far → heading_error drives omega negative → turns toward wall

Run:  python3 stanley_sim.py
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ─────────────────────────────────────────────
#  ROBOT / SENSOR PARAMETERS
# ─────────────────────────────────────────────
LIDAR_OFFSET_X   = 0.195   # m — lidar forward of turn axis
LIDAR_OFFSET_Y   = -0.147  # m — lidar lateral from turn axis (+left, -right in ROS frame)
                            # negative because lidar is to the RIGHT of turn axis
TARGET_DIST_M    = 0.770   # desired standoff from wall (m)
FOLLOW_SPEED     = 0.120   # m/s
MAX_ANG          = 0.400   # rad/s hard limit
STEP_SIZE_M      = 0.203   # one strip width along wall (m)

# ─────────────────────────────────────────────
#  CONTROLLER GAINS
# ─────────────────────────────────────────────
STANLEY_DIST_KP    = 2.5   # outer: dist_error → psi_cmd
STANLEY_HEADING_KP = 1.0   # inner: heading_error → omega  (k_theta)

# ─────────────────────────────────────────────
#  INITIAL CONDITIONS
# ─────────────────────────────────────────────
INIT_DIST_ERROR_M    = +0.090  # + = too far, − = too close
INIT_ANGLE_ERROR_DEG = +3.0    # + = nose toward wall

# ─────────────────────────────────────────────
#  DISTURBANCE
# ─────────────────────────────────────────────
DRIFT_PER_STEP_M = +0.003   # lateral drift away from wall per WALL_FOLLOW step

# ─────────────────────────────────────────────
#  SIM SETTINGS
# ─────────────────────────────────────────────
N_CYCLES = 8
DT       = 0.05   # control timestep (s)


# ─────────────────────────────────────────────────────────────────────────────
#  GEOMETRY
#
#  Wall at y=0.  Robot at y = -(TARGET_DIST_M + dist_error).
#  d_center = -robot_y  (always positive, = distance from wall)
#  d_lidar  = d_center - x_L * sin(theta)
#             nose toward wall (theta>0) → lidar closer → d_lidar < d_center  ✓
#  dist_error = d_center - TARGET_DIST_M
#             positive = too far, negative = too close                          ✓
# ─────────────────────────────────────────────────────────────────────────────

def lidar_reading(robot_y, theta,
                  x_L=LIDAR_OFFSET_X, y_L=LIDAR_OFFSET_Y):
    """
    Geometric lidar distance given robot lateral position and heading.
    x_L > 0 : lidar ahead of turn axis  → closer to wall when nose-in
    y_L < 0 : lidar to the right        → closer to wall when parallel
    The static y_L offset is baked into target_dist (calibrated in lidar frame),
    so we only model the angle-induced delta: y_L*(1 - cos(theta)).
    At theta=0: both terms zero → d_lidar = d_center ✓
    """
    d_center = -robot_y
    d_lidar  = (d_center
                - x_L * math.sin(theta)
                - y_L * (1.0 - math.cos(theta)))
    return d_lidar, d_center


def dist_error_corrected(d_lidar, theta,
                         x_L=LIDAR_OFFSET_X, y_L=LIDAR_OFFSET_Y,
                         target=TARGET_DIST_M):
    """
    Reconstruct centre-referenced dist_error from lidar reading + heading.
    Inverse of lidar_reading angle-induced terms.
    d_center = d_lidar + x_L*sin(theta) + y_L*(1 - cos(theta))
    """
    d_center = (d_lidar
                + x_L * math.sin(theta)
                + y_L * (1.0 - math.cos(theta)))
    return d_center - target   # + = too far, − = too close


def stanley_omega(dist_err, angle_err_rad,
                  kd=STANLEY_DIST_KP, kt=STANLEY_HEADING_KP,
                  max_ang=MAX_ANG):
    """
    psi_cmd     = atan(kd * dist_err)        desired heading toward wall
    heading_err = psi_cmd - angle_err_rad    how far actual heading is from desired
    omega       = kt * heading_err          negative because CCW is away from wall

    Sign trace (too far, nose parallel):
      dist_err > 0  → psi_cmd > 0  (want to angle toward wall)
      angle_err = 0 → heading_err = psi_cmd > 0
      omega = -kt * heading_err < 0  → CW → toward wall  ✓
    """
    psi_cmd     = math.atan(kd * dist_err)
    heading_err = psi_cmd - angle_err_rad
    omega       = kt * heading_err
    return float(np.clip(omega, -max_ang, max_ang)), psi_cmd, heading_err


# ─────────────────────────────────────────────────────────────────────────────
#  PHASE SIMULATORS
# ─────────────────────────────────────────────────────────────────────────────

def step_realign(x0, y0, theta0):
    """
    STEP_REALIGN: Stanley correction.
    Termination on lateral progress along wall (x-axis) >= STEP_SIZE_M.
    """
    x, y, theta = x0, y0, theta0
    traj = [(x, y, theta)]
    while (x - x0) < STEP_SIZE_M:
        d_lid, _ = lidar_reading(y, theta)
        d_err    = dist_error_corrected(d_lid, theta)
        omega, _, _ = stanley_omega(d_err, theta)
        x     += FOLLOW_SPEED * math.cos(theta) * DT
        y     += FOLLOW_SPEED * math.sin(theta) * DT
        theta += omega * DT
        traj.append((x, y, theta))
    return traj, x, y, theta


def wall_follow(x0, y0, theta0, drift=DRIFT_PER_STEP_M):
    """
    WALL_FOLLOW: straight drive + lateral drift (mechanical bias).
    Drift applied as steady lateral displacement over the step duration.
    """
    x, y, theta = x0, y0, theta0
    traj = [(x, y, theta)]
    step_dur = STEP_SIZE_M / FOLLOW_SPEED
    while (x - x0) < STEP_SIZE_M:
        x += FOLLOW_SPEED * math.cos(theta) * DT
        y += FOLLOW_SPEED * math.sin(theta) * DT
        y -= (drift / step_dur) * DT   # drift away from wall (y more negative)
        traj.append((x, y, theta))
    return traj, x, y, theta


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────

def run():
    # Robot starts at y = -(TARGET_DIST_M + INIT_DIST_ERROR_M)
    y0     = -(TARGET_DIST_M + INIT_DIST_ERROR_M)
    x0     = 0.0
    theta0 = math.radians(INIT_ANGLE_ERROR_DEG)

    x, y, theta = x0, y0, theta0
    all_traj  = [(x, y, theta)]
    phase_log = []

    print("=" * 65)
    print("  Stanley Wall-Following Simulation")
    print(f"  Wall at y=0 | robot starts at y={y0:.3f} m")
    print(f"  Target dist : {TARGET_DIST_M*1000:.0f} mm")
    print(f"  LiDAR offset: x={LIDAR_OFFSET_X*1000:.0f}mm y={LIDAR_OFFSET_Y*1000:.0f}mm")
    print(f"  Init dist err: {INIT_DIST_ERROR_M*1000:+.0f} mm  "
          f"Init angle err: {INIT_ANGLE_ERROR_DEG:+.1f}°")
    print(f"  Gains  kd={STANLEY_DIST_KP}  kt={STANLEY_HEADING_KP}")
    print(f"  Drift per step: {DRIFT_PER_STEP_M*1000:+.1f} mm")
    print("=" * 65)

    # Sanity check
    d_lid, d_cen = lidar_reading(y0, theta0)
    d_err        = dist_error_corrected(d_lid, theta0)
    offset_contrib = (d_lid - d_cen) * 1000
    print(f"\n[SANITY] At start:")
    print(f"  True centre dist from wall : {d_cen*1000:.1f} mm  "
          f"(expected {TARGET_DIST_M*1000 + INIT_DIST_ERROR_M*1000:.0f} mm) "
          f"{'✓' if abs(d_cen - TARGET_DIST_M - INIT_DIST_ERROR_M) < 1e-6 else '✗'}")
    print(f"  Expected lidar reading     : {d_lid*1000:.1f} mm  "
          f"(offset contribution {offset_contrib:+.1f} mm at {INIT_ANGLE_ERROR_DEG:+.1f}°)")
    print(f"  dist_error after correction: {d_err*1000:+.1f} mm  "
          f"(expected {INIT_DIST_ERROR_M*1000:+.0f} mm) "
          f"{'✓' if abs(d_err - INIT_DIST_ERROR_M) < 1e-6 else '✗'}\n")

    for cyc in range(1, N_CYCLES + 1):

        # WALL_FOLLOW
        d_lid, _ = lidar_reading(y, theta)
        de_s = dist_error_corrected(d_lid, theta)
        ae_s = theta

        traj, x, y, theta = wall_follow(x, y, theta)
        all_traj.extend(traj)

        d_lid, d_cen = lidar_reading(y, theta)
        de_e = dist_error_corrected(d_lid, theta)
        ae_e = theta

        phase_log.append((cyc, "FOLLOW", de_s*1000, de_e*1000,
                          math.degrees(ae_s), math.degrees(ae_e)))
        print(f"Cycle {cyc}  FOLLOW   "
              f"dist {de_s*1000:+6.1f}→{de_e*1000:+6.1f} mm  "
              f"angle {math.degrees(ae_s):+5.1f}→{math.degrees(ae_e):+5.1f}°  "
              f"[lidar≈{d_lid*1000:.1f} mm | centre≈{d_cen*1000:.1f} mm | "
              f"offset {(d_lid-d_cen)*1000:+.1f} mm]")

        # STEP_REALIGN
        d_lid, _ = lidar_reading(y, theta)
        de_s = dist_error_corrected(d_lid, theta)
        ae_s = theta

        traj, x, y, theta = step_realign(x, y, theta)
        all_traj.extend(traj)

        d_lid, d_cen = lidar_reading(y, theta)
        de_e = dist_error_corrected(d_lid, theta)
        ae_e = theta

        phase_log.append((cyc, "REALIGN", de_s*1000, de_e*1000,
                          math.degrees(ae_s), math.degrees(ae_e)))
        print(f"Cycle {cyc}  REALIGN  "
              f"dist {de_s*1000:+6.1f}→{de_e*1000:+6.1f} mm  "
              f"angle {math.degrees(ae_s):+5.1f}→{math.degrees(ae_e):+5.1f}°")
        print()

    print("=" * 65)

    # ── PLOT ──────────────────────────────────────────────────────────────
    arr  = np.array(all_traj)
    xs = arr[:, 0]
    ys = -arr[:, 1]          # convert raw y → distance from wall (-robot_y)

    fw_rows = [r for r in phase_log if r[1] == "FOLLOW"]
    re_rows = [r for r in phase_log if r[1] == "REALIGN"]
    cycs    = [r[0] for r in fw_rows]

    fig = plt.figure(figsize=(14, 9))
    fig.suptitle(
        f"Stanley STEP_REALIGN Sim  |  "
        f"kd={STANLEY_DIST_KP}  kt={STANLEY_HEADING_KP}  "
        f"lidar_x={LIDAR_OFFSET_X*1000:.0f}mm lidar_y={LIDAR_OFFSET_Y*1000:.0f}mm  "
        f"drift={DRIFT_PER_STEP_M*1000:+.1f} mm/step",
        fontsize=11)
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.38, wspace=0.32)

    # trajectory
    ax0 = fig.add_subplot(gs[0, :])
    ax0.plot(xs, ys, color='steelblue', lw=1.2, label='robot path (turn axis)')
    ax0.axhline(TARGET_DIST_M, color='red', lw=1.5, ls='--',
                label=f'target standoff ({TARGET_DIST_M*1000:.0f} mm)')
    for i in range(N_CYCLES):
        xv = (i + 0.5) * STEP_SIZE_M * 2
        ax0.axvspan(i * STEP_SIZE_M * 2, i * STEP_SIZE_M * 2 + STEP_SIZE_M,
                    alpha=0.06, color='orange', label='FOLLOW' if i == 0 else None)
        ax0.axvspan(i * STEP_SIZE_M * 2 + STEP_SIZE_M,
                    (i + 1) * STEP_SIZE_M * 2,
                    alpha=0.06, color='green', label='REALIGN' if i == 0 else None)
    ax0.set_xlabel("Along-wall distance x (m)")
    ax0.set_ylabel("Distance from wall (m)")
    ax0.set_title("Top-view trajectory")
    ax0.legend(fontsize=8, ncol=4)
    ax0.grid(True, alpha=0.3)

    # dist error
    ax1 = fig.add_subplot(gs[1, 0])
    ax1.plot(cycs, [r[2] for r in fw_rows],  'o--', color='#e07b39', label='FOLLOW start')
    ax1.plot(cycs, [r[3] for r in fw_rows],  's-',  color='#e07b39', label='FOLLOW end')
    ax1.plot(cycs, [r[2] for r in re_rows],  'o--', color='#4caf50', label='REALIGN start')
    ax1.plot(cycs, [r[3] for r in re_rows],  's-',  color='#4caf50', label='REALIGN end')
    ax1.axhline(0, color='red', lw=1, ls='--')
    ax1.set_xlabel("Cycle"); ax1.set_ylabel("dist_error (mm)")
    ax1.set_title("Distance error per cycle")
    ax1.legend(fontsize=8); ax1.grid(True, alpha=0.3)

    # angle error
    ax2 = fig.add_subplot(gs[1, 1])
    ax2.plot(cycs, [r[4] for r in fw_rows],  'o--', color='#e07b39', label='FOLLOW start')
    ax2.plot(cycs, [r[5] for r in fw_rows],  's-',  color='#e07b39', label='FOLLOW end')
    ax2.plot(cycs, [r[4] for r in re_rows],  'o--', color='#4caf50', label='REALIGN start')
    ax2.plot(cycs, [r[5] for r in re_rows],  's-',  color='#4caf50', label='REALIGN end')
    ax2.axhline(0, color='red', lw=1, ls='--')
    ax2.set_xlabel("Cycle"); ax2.set_ylabel("angle_error (deg)")
    ax2.set_title("Angle error per cycle")
    ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)

    plt.savefig('./test/misc/outputs/stanley_sim_xy.png', dpi=150, bbox_inches='tight')
    print("Plot saved → stanley_sim.png")


if __name__ == '__main__':
    run()

    plt.show()
    print("Plot saved → stanley_sim.png")
