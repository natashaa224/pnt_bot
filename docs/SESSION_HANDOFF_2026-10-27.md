# Painting robot – session handoff (what was done, and why)

Written for other Claude sessions / engineers picking this workspace up. Repo root = `src/` (remote `origin` = sid_painting_robot, branch `main`). ROS 2 Humble, colcon `--symlink-install`, Gazebo classic sim, real bench launch `painting_robot_bringup/real_bot_bench.launch.py` (backend `sbc`, fake_paint False, homing gate on).
Token for GitHub pushes is in `src/token.txt` (gitignored) – use it via `git -c http.extraheader="Authorization: Basic <b64 x-access-token:TOKEN>" push origin main`; never commit it.

## 1. System overview (just enough to navigate)
- **FSM** (`painting_robot_fsm`): queue-driven recipes – `align+paint`, `follow`, `corner_approach`, `corner_arc`, `corner_align+paint`, `corner_turn`, `corner_align`, `bump_*`. Safety HOLD → FAULT after 15 s (`obstacle_persist:wall_dist`).
- **Perception** (`painting_robot_perception/env_monitoring_v2.py`): corner detection priority BUMPOUT > SUCKIN > INTERNAL > EXTERNAL.
- **Navigation** (`painting_robot_navigation/wall_motion_node.py`): wall-follow (Stanley: `psi_cmd = atan(kd*dist_err)`, STRONG kd = `step_realign_dist_kp` 1.8), odom-tracked turns/crawls, bump arcs (`maneuver_planner`).
- **Painting/actuation** (`painting_robot_painting/actuator_bridge_node.py`, `leadshine_driver.py`): odometry `counts_per_m = cpr*gear/(pi*wheel_dia)`, `dtheta=(dl-dr)/track`.
- Geometry: gantry base 0.312 m right of axis, lidar at (0.195, -0.147). External-corner final lidar→wall distance ≈ `s + R - 0.147` (s = `ext_trigger_turnaxis` 0.104, R = 0.8).
- Worlds: `paint_bot_description/worlds/` (`pillar_corner_removed`, `bump_box_1p2`, `bump_room_10x8` = 10×8 m room, spawn 9.093,-7.3 yaw 1.5708).

## 2. Changes made (chronological) and the reason for each

### Corner / bump-out handling (sim)
| Change | Why |
|---|---|
| `bumpout_max_depth` raised to **1.20 m** (env_monitoring_v2) with comment that it is verified on the depths tested | Earlier cap (detector-side switch at 0.8 m; env2 cap lower) rejected deep bump-outs as "not executing as compound bump". Verified in sim on 1.2 m box. |
| Shallow bump-outs (SUCKIN, depth ≤ `SUCKIN_MAX_DEPTH` 0.6 in geometry_utils) handled as **external corner** (`suckin_as_external` True → `_latch_external`) | Shallow bumps are too small for the compound bump maneuver; treating them as an external corner worked. |
| `bump_defer_internal_m` 1.5 – defer INTERNAL latch right after a bump sighting; `internal_max_backup` 0.60 | A bump's inner wall looked like an INTERNAL corner and triggered a bogus latch. |
| Reverse limit raised, new worlds (`bump_box_1p2`, `bump_room_10x8`, extended pillar world) | To exercise deeper bumps / gapped pillars. 10×8 lap ran with **0 faults**. |

### Duplicate-paint / stroke gate
- Added `stroke_gate_mode` (`pose` default | `head`) + `stroke_min_robot_travel_m` 0.05 + `stroke_min_rotation_deg` 9.0 + `_stroke_redundant_pose()` in `state_machine_node.py`: a stroke is allowed only if the robot moved ≥0.05 m **or** rotated ≥~9° since the last stroke (raw odometry, wrapped heading, reset on HOLD/FAULT). Why: stop duplicate strokes at the same spot while still painting during pure rotation at corners. Compared with the older "head" method in sim: 0 disagreements. Docs/figures: `docs/production_report_bump_room.md`, `docs/bump_room_fault_report.md`.

### Real-robot calibration (done with `test/straight_drive_test.py`, `test/rotation_test.py`)
- Measured wheel dia **0.200 m**, `base_gear_ratio` **20.0**, `base_counts_per_rev` 10000 (`painting_robot_bringup/config/actuator_bridge_params.yaml` ~lines 35-38). Straight test: tape 1.96 m vs 2.0 commanded, odom 1.99. (An earlier inferred gear ratio 17.9/16.9 was wrong – based on unverified yaml values.)
- `track_width_m` **locked at 0.625** (rotation 360° test gave ~354–368° across runs, within noise; 8° error on a 90° turn was investigated and attributed to ramp/overshoot, not track width).
- Pi build gotchas fixed: missing `meshes` dir (mkdir), stale build/install of `painting_robot_msgs` (rm -rf), rplidar_ros empty.

### Real-robot post-external-corner faults (WALL_DIST)
Symptom: after an external turn the robot sits 0.07–0.18 m too far from the wall (mean ≈ +0.10 m; sim ≈ −0.02), then FSM faults with `obstacle_persist:wall_dist`.
- `states.py` `WallAlignHandler`/`WallFollowHandler` + `state_machine_node.py` params `post_corner_dist_tol_m` 0.20, `post_corner_max_steps` 8, `post_corner_angle_tol_rad` 0.035: **recovery mode** after `corner_align+paint` – align on angle only, skip strokes while distance error > normal tolerance, close the gap over ≤8 strips, fault `WALL_DIST` only if it does not close. Recovery is label-based (latched corner type is already cleared by then) and only ends when `wall_valid` and error ≤ tol (a lost wall reads error 0.0 – earlier bug).
- `wall_motion_node.py` param `max_approach_angle_deg` 10: caps `psi_cmd`. Why: STRONG kd gave atan(1.8×0.175)=17° approach into the wall, lidar lost it ("No valid wall pose") and robot stopped. 0.17 m error now closes in ~4–5 strips; 0.05 m error unaffected (5.1°).
- **Not yet done / suggested:** try `ext_trigger_turnaxis` ≈ 0.0 in `env_monitoring_params.yaml` (hypothesis: arc starts ~0.10 m too late on the real robot). Untested.

### Internal-corner over-turn (latest real-robot log)
Symptom: INTERNAL 90° turn (2 segments × 45° with a paint stroke between) ended at ~110°, robot faced away, `WALL_LOST` → HOLD.
- Cause: base keeps rotating on its motor ramp-down while the gantry strokes (yaw drifted −143° → −124.7° during the stroke); segment 2 was a *relative* 45° from the drifted pose.
- Fix (`_do_internal_corner` in `wall_motion_node.py`): each segment now targets an **absolute** heading (`origin + idx*seg_angle`), logging `[CORNER-INT] seg N: drift …deg … compensating`. `_int_origin_yaw` reset in corner reset. Compile-checked only, **not yet run in sim or on the robot**.

## 3. Known open issues (not fixed)
1. **Gantry strokes while the base is still moving**: stop is sent at stroke start but `drive_base` ramps 0.1→0 over ~1.5 s (Leadshine accel/decel settings `accel=150 decel=120`). Proposed: FSM waits until odom speed < ~0.01 m/s & 0.02 rad/s (timeout ~2 s) before PAINT_EXECUTE (+~1 s per stroke), or raise decel.
2. **Wrist settle timeouts**: wrist reaches only 35.7°/9°/−17°/41° of ±45° in 0.46 s – strokes complete before the wrist is in position. Raise timeout or investigate wrist speed.
3. First turn sits still ~1.8 s before moving (stiction/ramp-up).
4. `wall_motion_node` sits silently in "No valid wall pose" without timing out to a fault.
5. Wall-follow cannot remove small steady offsets (~+0.04 m) outside STRONG mode.
6. `ext_trigger_turnaxis` experiment above.

## 4. Deploying to the real robot
`git pull` on the Pi, then `colcon build --symlink-install --packages-select painting_robot_fsm painting_robot_navigation` (plus perception/bringup if configs changed), `source install/setup.bash`. Verify: `ros2 param get /wall_motion_node max_approach_angle_deg` → 10.0; `ros2 param get /state_machine_node post_corner_dist_tol_m` → 0.2. Check the launched install path is from the workspace you just built.

## 5. Sim tips
- User's stack uses ROS domain 29. `pkill -f` can kill your own shell (exit 144) – kill by PID. Use wall-clock timestamps when mixing sim/wall clocks in analysis scripts.
- Commits end with `Co-Authored-By: Claude Sonnet 5.5 <noreply@anthropic.com>`.

## 6. Commit history
`60de02c` initial → `5a5ff53` shallow bump-outs + 10×8 world → `db7c3c5` 1.2 m depth → `f2a9c9e` depth docs → `4028d99` stroke gate mode + reports → `2266e8a` post-corner recovery, 10° approach cap, drift-free internal segments, calibration scripts.
