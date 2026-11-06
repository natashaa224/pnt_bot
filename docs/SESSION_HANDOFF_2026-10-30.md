# Painting robot – session handoff, update 2026-10-30 (read this first)

Continues `docs/SESSION_HANDOFF_2026-10-27.md` (system overview, sim/bump work, calibration, post-corner recovery, internal-corner fix). Read that for background; this file covers everything done **after** it, plus the current state and open issues.
`docs/SESSION_HANDOFF.md` belongs to another session – do not overwrite it.

Repo root = `src/`, branch `main`, remote `origin`. GitHub token is in `src/token.txt` (gitignored; never commit). Commits end with `Co-Authored-By: Claude Sonnet 5.5 <noreply@anthropic.com>`.

## 0. How the user works (read before acting)
- Tests on a real robot (Raspberry Pi CM4, `pi@rpicm4`) and pastes ROS launch logs; diagnose from the log, then fix.
- The user asks "commit and push" explicitly; do not push unasked. Use `git add` on specific files (an earlier `git add -A` swept in their own edit to `painting_robot_hmi/.../logging_node.py`).
- They copy individual files to the Pi by hand (e.g. "I just copy states.py") – always say exactly which files changed.
- They want simple-words explanations and exact changed lines when asked.
- **Two code bases exist on the Pi**: Python = `/home/pi/sid_painting_robot-main` (this repo) and a C++ port = `/home/pi/painting_robot_cpp_ws` (`src/painting_robot_fsm/src/state_machine_node.cpp`). The C++ source is NOT on the dev PC. Check which one a pasted log came from: C++ prints `paints=1`, `(no driver)`; Python prints `paints=True` and tracebacks mention `/home/pi/sid_painting_robot-main`. Fixes made here only apply to the Python tree; the C++ port needs them ported separately.
- Always check the log's message text against the current source to detect a stale file on the Pi (e.g. the old `post-corner recovery complete - distance back inside tolerance` text means hysteresis is missing).

## 1. Changes since 2026-10-27 (all committed and pushed)

| Commit | What / why |
|---|---|
| `abf0f80` | Strokes suppressed where the nozzle has no wall: (a) first re-align after a corner (`corner_align+paint`, `corner_align`, `bump_align`) goes straight to wall-follow, flag `_skip_next_align_stroke` skips the next `align+paint` stroke; (b) final segment of an external corner turn (`label == 'corner_arc'` and turn complete). Recovery now also covers `bump_align`. Added `test/painted_area_report.py`. |
| `c8471ff` | Stroke gate (`stroke_is_redundant()`) also applied to corner-turn strokes (removed duplicate stroke at turn entry caused by a stale zero-length crawl `step_complete`). Report counts every gantry sweep as a stroke. (Also swept in user's `logging_node.py` edit.) |
| `1567819` | **Recovery hysteresis**: post-corner distance recovery ends only when error ≤ `dist_tolerance × post_corner_exit_frac` (0.6 → 0.06 m). Why: it used to end at exactly 0.100 m, a 0.003 m wobble then triggered `WALL_FOLLOW dist error +0.101 > tol 0.100 → WALL_DIST` → HOLD → FAULT. Log text is now `post-corner recovery complete - distance 0.0xx m <= 0.060 m`. Files: `painting_robot_fsm/states.py`, `state_machine_node.py` (param `post_corner_exit_frac`). |

Stroke gate rule (unchanged): allow a stroke if robot moved ≥ 0.05 m or rotated ≥ 9° since the last stroke (`stroke_gate_mode: pose`). Log line `[FSM] stroke gate: robot moved X m, turned Y deg since the last stroke - painting`.

Expected log messages that confirm the fixes are live:
- `[FSM] external corner final segment - no wall in front of the nozzle, stroke suppressed`
- `[FSM] post-turn re-align done (corner_align+paint) - no stroke, straight to wall-follow`
- `[FSM] first align after a corner - stroke suppressed, straight to wall-follow`
- `[CORNER-INT] seg N: drift … compensating` (internal corner, absolute heading targets)
- `[STRIP] … psi_cmd=+10.0deg` (10° approach cap)

## 2. Painted-area report tool: `test/painted_area_report.py`
Run alongside `real_bot_bench.launch.py` (ROS must be sourced, `painting_robot_msgs` built). Writes `~/painting_reports/run_<time>/{report.md,summary.json,strokes.csv,plan_view.png}`. Ends on any stop (FAULT, HOLD, stall, Ctrl-C).
- Stroke = gantry Z sweep (Z moving ≥ `--z-move-mm` 2.0, idle `--z-idle-s` 0.6). **No wall check** (user's explicit decision).
- Stroke height = homed gantry travel (z_soft_upper − z_soft_lower from `/paint_head_status` after teach-homing; ≥ `--full-frac` 0.9 counts as full; `--gantry-travel` is the fallback).
- Area = height × (swath 0.203 m + robot travel during the stroke); 1 cm plan-view grid dedup so overlaps are not double counted.
- Defaults: `--target-dist 0.77 --lidar-right 0.147 --head-right 0.312 --swath 0.203 --min-height 0.05 --stall-s 60 --grace-s 1.5`.
- Other test scripts: `test/rotation_test.py`, `test/straight_drive_test.py` (odometry calibration).

## 3. Latest real-robot log findings (2026-10-30, Python tree)
- Alignment, external-corner turn (2×45°), stroke suppression and the 10° cap all worked; wall-follow after the corner recovered the error from −0.07 m to −0.01 m.
- **The stop was NOT a wall fault.** At `[JOY] Direct Z_HOME sent` (joy_teleop_node, mid-stroke) the actuator un-homed (`teach-homing armed`, `executor: homed cleared`). Next stroke: `[executor] Z not homed — cannot start stroke` and the FSM waits in PAINT_EXECUTE forever (no timeout). Cause is a joystick Z_HOME button press; remedy is to re-teach homing (jog down + A, jog up + Y).
- That log still had the old `post-corner recovery complete - distance back inside tolerance` text → the Pi's `states.py` lacked commit `1567819`.
- Previous log (C++ port) faulted with `WALL_FOLLOW dist error +0.100 > tol 0.100` – same hysteresis bug, in the C++ tree.

## 4. Open issues / next steps (priority order)
1. **FSM hangs on "Z not homed"**: executor refuses the stroke but the FSM never times out. Add a HOLD/FAULT (e.g. `Z_NOT_HOMED`) or a PAINT_EXECUTE watchdog. Offered to the user, not yet done.
2. **Port to the C++ tree** (`/home/pi/painting_robot_cpp_ws`): hysteresis exit (`dist_err <= dist_tolerance * 0.6`), plus confirm stroke-suppression and corner stroke gate (they appeared in the C++ log, so already ported). Need the user to provide `state_machine_node.cpp` (+ states file).
3. **Base still moving during strokes**: motor ramp-down ~1.5 s causes drift. Proposed: FSM waits for odom speed < ~0.01 m/s and 0.02 rad/s (timeout ~2 s) before PAINT_EXECUTE, or raise decel. Not done.
4. **Wrist settle timeouts**: every stroke warns `wrist settle timeout (0.26s) — actual=… target=±45°` (wrist 20–60° short). Raise timeout/investigate wrist speed. In C++ log the wrist shows `(no driver)`.
5. Bump-out / external arcs are open-loop with *relative* segments → leave ~0.10 m residual distance error after the corner. Idea: absolute-heading targets (like internal corner) and/or wall feedback in the arc. Not done.
6. `ext_trigger_turnaxis` ≈ 0.0 experiment (`env_monitoring_params.yaml`): log shows `trig_turnaxis=0.000` in the latest run, 0.104 earlier – check which value is deployed.
7. `wall_motion_node` stays silent in "No valid wall pose" without faulting; repo yaml wheel_dia 0.203 vs measured 0.200 on the robot.
8. Internal-corner absolute-heading fix verified on the robot (turn ≈ 90°); not yet re-run in sim.

## 5. Deploy checklist for the Pi (Python tree)
Files most recently changed: `painting_robot_fsm/painting_robot_fsm/states.py`, `.../state_machine_node.py`, `painting_robot_navigation/painting_robot_navigation/wall_motion_node.py`, `test/*.py`. With `--symlink-install` copying the .py files is enough; otherwise `colcon build --symlink-install --packages-select painting_robot_fsm painting_robot_navigation`, then `source install/setup.bash`. Verify: `ros2 param get /state_machine_node post_corner_exit_frac` → 0.6, `ros2 param get /wall_motion_node max_approach_angle_deg` → 10.0.

## 6. Commit history (newest last)
`60de02c` → `5a5ff53` → `db7c3c5` → `f2a9c9e` → `4028d99` → `2266e8a` (recovery, 10° cap, internal segments, calibration) → `abf0f80` (stroke suppression, report script) → `c8471ff` (corner stroke gate) → `078b19f` (another contributor, README workflow diagram) → `1567819` (recovery hysteresis).
