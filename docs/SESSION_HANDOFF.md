# Session handoff — painting robot bump-out / corner work

Read this first if you are a new Claude session continuing this project.
Written 2026-10-23 at the end of a long session. Everything below was verified from
simulation logs unless marked **UNTESTED** or **NOT DONE**.

## Environment

- Workspace: `/home/ira_linux/claude/oct22/bumpout_painting_seq_Sent_from_vijeth`
  (colcon workspace, sources in `src/`). **Not a git repo** — no history; changes are
  only recorded in this file.
- ROS 2 Humble + Gazebo classic. Build: `colcon build --symlink-install` from the
  workspace root (symlink-install, so Python and launch edits take effect without a
  rebuild; message changes need a rebuild).
- `export ROS_DOMAIN_ID=37` was added to `~/.bashrc` (line ~122) because another
  person uses ROS 2 on the same network. New shells pick it up; old ones need
  `source ~/.bashrc`. All terminals and tools must share the ID.
- The user runs things in their own terminal and reports back by pasting logs.
  Terminal HMI: `ros2 run painting_robot_hmi hmi_keyboard_node` (key `A` = start).

## How to run the simulation

```bash
bash /tmp/claude-1000/killall.sh; cd /home/ira_linux/claude/oct22/bumpout_painting_seq_Sent_from_vijeth && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch painting_robot_bringup corner_bringup.launch.py world:=$(ros2 pkg prefix paint_bot_description)/share/paint_bot_description/worlds/just_bump_out.world spawn_x:=4.618 spawn_y:=-8.0 spawn_z:=0.1 spawn_yaw:=1.5708
```

Start the robot (second terminal; the first publish is sometimes missed, so repeat):

```bash
ros2 topic pub -t 3 -r 2 /start_cmd std_msgs/msg/String "{data: START}"
```

Swap `just_bump_out.world` for `sketch_layout.world` or `pillar_corner_removed.world`
to try the other layouts.

`/tmp` is cleared on reboot. `killall.sh` is just a script that force-kills the stack;
recreate it if missing:

```bash
#!/bin/bash
for p in $(ps -eo pid,args | grep -E "painting_robot_ws|bumpout_painting_seq|ros2 launch|gzserver|gzclient|robot_state_publisher|controller_manager/spawner|ekf_node" | grep -vE "grep|killall" | awk '{print $1}'); do kill -9 $p 2>/dev/null; done
```

**Pitfalls**
- Ctrl-C on `ros2 launch` often leaves nodes behind. Duplicate nodes publish
  conflicting `/safety_reason` (the HMI flickers NOMINAL/FAULT). Check with
  `ros2 node list | sort | uniq -c | awk '$1>1'`; empty output = clean.
- Never use `pkill -f <pattern>` from inside Claude's own shell when the pattern
  appears in the command line — it kills the shell (exit 144). Use the script above.
- With the default launch the log shows `paint_axis_controller ... Failed to activate`
  and `Not existing: zAxis_joint/position`. This is expected and harmless (the
  gantry is not in the sim model by default).
- Warnings `Parameter file path is not a file: .../common_params.yaml` and
  `.../painting_robot_painting/config/actuator_bridge_params.yaml` are harmless.

## Current state of the code (all changes made this session)

### 1. `painting_robot_bringup/launch/corner_bringup.launch.py`
- New launch arg **`world`** (default `bump_070.world`, the original).
- Spawn args `spawn_x spawn_y spawn_z spawn_yaw` are now actually wired into
  `spawn_entity` (defaults 2.50, 3.08, 0.1, −3.1415 = the old hardcoded values).
- Still hardcoded for sim: `attach_gantry:=false`, `fake_paint: True`,
  `skip_homing_gate: True`. A gantry experiment (below) was **reverted**.

### 2. Worlds in `paint_bot_description/worlds/` (origin = top-left of the sketch, X right, Y down/negative; wall thickness 0.15, height 3.0)
- `sketch_layout.world` — full sketch: top-left notch, top wall, step, long wall, pillar.
- `just_bump_out.world` — long wall (x=5.6, inner face x=5.525) + pillar only.
- `pillar_corner_removed.world` — sketch_layout without the small jog in the top-right
  corner; top wall extended to x=5.675 and long wall extended up to y=+0.075 so they
  meet in a plain internal corner.
- Pillar: 0.46 × 0.9 m centred at (5.055, −5.575). It is **free-standing**: 0.24 m clear
  gap to the long wall inner face, so it protrudes **0.70 m** from the wall (this is
  the "bump-out depth 0.70" in the logs). Robot spawn (4.618, −8.0), yaw 90° (north).
- `docs/sketch_layout_dimensions.png` still shows the OLD 0.165 m gap;
  `docs/just_bump_out_dimensions.png` is correct.

### 3. Internal-corner reverse crawl (fixes a WALL_DIST fault)
Problem: after the external turn at the pillar's end the robot is already closer to the
front wall than the internal trigger (0.709 m lidar-frame). `crawl_total` went negative,
was clamped to 0, and the in-place 90° pivot left the robot ~0.12–0.21 m too close to
the wall, tripping `WALL_DIST` → HOLD → FAULT (`obstacle_persist:wall_dist`).
Fix:
- `painting_robot_msgs/msg/EnvStatus.msg`: new field `float32 crawl_backup`.
- `painting_robot_perception/.../env_monitoring_v2.py`: new param
  `internal_max_backup` (0.30 m); publishes `crawl_backup` = overshoot while approaching
  an INTERNAL corner (closed loop on odom; shrinks as the robot reverses). Above the
  cap it only warns.
- `painting_robot_navigation/.../wall_motion_node.py`: in `CORNER_APPROACH`, reverses
  straight at `crawl_speed` until `crawl_backup ≤ 0.012 m` (`[BACKUP] reversing ...`
  log lines), then falls through to the normal zero-length final crawl. No stroke is
  issued while reversing. FSM needed no change for this.
- Verified in sim: `crawl_total=-0.116/-0.120/-0.137` → reverse → `front≈0.705–0.715`
  → turn → right-wall distance ≈ 0.755–0.760 (target 0.77), run continues.

### 4. Mid-recipe WALL_DIST skip
Even with the reverse, the `WALL_ALIGN` step right after the external turn can see a
~0.1 m lane error (it measured +0.101 vs tolerance 0.100 once → fault). The internal
approach+turn fixes the lane afterwards, so:
- `painting_robot_fsm/.../states.py` (`WallAlignHandler`): skip (and clear) the
  `WALL_DIST` check when the current queue item label is `corner_align+paint` AND an
  INTERNAL corner is already latched.
- `painting_robot_fsm/.../state_machine_node.py`: stores
  `fsm_context['latched_corner_type']` from `EnvStatus`.
- All other `WALL_ALIGN` steps still enforce `dist_tolerance` (0.1 in
  `common_alignment_params.yaml`; the node's own default is 0.07).

### 5. Gantry-in-simulation experiment — REVERTED (kept here as a recipe)
What was learned/built, in case it is wanted again:
- The sim robot has no gantry because the launch passes `attach_gantry:=false` to
  xacro; that also makes `paint_axis_controller` fail to activate. With
  `attach_gantry:=true` the `zAxis_joint` exists, the controller activates, and
  trajectories on `/paint_axis_controller/joint_trajectory` move it (0→0.6→0 m verified).
- Real painting flow (see `real_bot_bench.launch.py`, `paint_executor_node.py`):
  after each ~0.2 m travel step the FSM enters `PAINT_EXECUTE`; the executor sends a Z
  stroke (up to `strip_z_max` 1.2 m, direction alternates each stroke), opens spray,
  flicks the wrist, and publishes `/stroke_complete`. Bench launch uses
  `fake_paint: False`, `skip_homing_gate: False`, `hardware_backend: sbc`.
- To get real strokes in sim you needed: `attach_gantry:=true`, `fake_paint:=false`,
  and homing (executor refuses with `Z not homed` otherwise). It was done with a new
  `sim_auto_home` bridge param (SimBackend `auto_home()` marking Z homed over 0–1.2 m
  on `Z_HOME`) plus launch args `fake_paint`, `skip_homing_gate`, `sim_auto_home`,
  `attach_gantry`, and pointing `painting_params` at
  `painting_robot_bringup/config/actuator_bridge_params.yaml`. 4 real strokes completed
  (~4 s each). All of this was removed again at the user's request.
- Second attempt (after the stroke gate, section 6): re-added the same launch args/bridge
  change with defaults gantry-on. 9 real strokes ran (up/down, no `Z not homed`), but
  the run then faulted: the bump-out latched with depth 0.658 (instead of ≈0.70), the
  arcs left the robot with no valid right wall (pillar end already behind it) →
  `WALL_LOST` → HOLD → FAULT. One run only; not confirmed as repeatable. Suspected
  cause (unverified): the 3 kg gantry shifting the robot's centre of mass changes what
  the lidar sees. The user then asked to drop gantry movement again, so everything was
  reverted a second time. The stroke gate (section 6) was kept.
- Observed once with the gantry attached: a single run misclassified the pillar as an
  INTERNAL corner (see open issue B). A/B runs with gantry on/off both classified it
  correctly, so the gantry is not proven to be the cause.

### 6. Redundant-stroke gate (removes duplicate gantry strokes)
User requirement: no stroke when the robot has not moved (translation or rotation)
since the previous stroke. Implemented in the FSM:
- `state_machine_node.py`: new params `stroke_min_head_travel_m` (0.05),
  `spray_head_offset_x/y` (0.0 / -0.312, gantry position in the robot frame from the
  URDF). Spray-head position = **raw wheel odom** `/paint_bot/odom` pose + that
  offset (new `raw_odom_cb`; NOT `/paint_bot/odom_filtered`, whose EKF output lags at
  turn ends and gave 0.16 m phantom movement). Helpers: `spray_head_xy()`,
  `note_stroke_started()`, `reset_stroke_gate()`, `stroke_is_redundant()`.
- `states.py`: `WallAlignHandler` returns `ITEM_COMPLETE` instead of `PAINT_EXECUTE`
  when the head moved < 0.05 m since the last stroke (logs
  `[FSM] stroke SKIPPED - spray head moved only X m`; otherwise logs
  `stroke gate: ... painting`). `PaintExecuteHandler.on_enter` records the head
  position; `HoldHandler`/`FaultHandler` entry and IDLE reset the gate so the first
  stroke after a stop/resume always paints.
- Only WALL_ALIGN-caller strokes are gated (corner-segment strokes are never gated:
  skipping them would break the segmented-turn resume logic in `_transition_to`).
- Verified on `just_bump_out.world`: 23 → 20 strokes, exactly three skipped, all true
  non-moves: (a) the `align+paint` right after the second bump-out arc / `bump_align`
  (moved 0.000 m), (b) `corner_align+paint` after the external turn (0.016 m),
  (c) the `align+paint` right after the internal turn / `corner_align` (0.004 m).
  Normal strip strokes show ≈0.22 m movement and still paint. No HOLD/FAULT.
- NOT removed by design: in-place pivot strokes (internal turn 2×45°, bump arc 1) —
  the head sweeps ~0.24 m of wall per 45° segment, so they exceed the 0.05 m gate.

## Real robot caveats
- `actuator_bridge_node.py` was edited and then restored; net effect on the real
  (sbc) backend is none.
- The reverse crawl (3) and the WALL_ALIGN skip (4) live in shared nodes and DO run on
  the real robot. **UNTESTED on hardware.** The lidar cannot see behind the robot; the
  reverse is capped at 0.30 m. Disable the reverse with `internal_max_backup: 0.0`
  (it then only emits a throttled warning). `EnvStatus` changed, so every machine
  running these nodes must be rebuilt.

## Open issues / next steps (none of these are implemented)

**A. Duplicate gantry strokes at corners — DONE (see section 6).** Remaining judgment
call: the in-place pivot stroke pairs still paint (head moves ~0.24 m per segment). If
the user wants fewer strokes there, raise `stroke_min_head_travel_m` or extend the gate
to corner-segment strokes (needs care with the segmented-turn resume logic). UNTESTED
on the real robot (needs `/paint_bot/odom` published by the sbc actuator bridge, which
`wall_motion_node` already depends on).
An idea the user asked for and then cancelled: a top-view plot of stroke positions; the
script was deleted, nothing remains.

**B. Intermittent pillar misclassification.** One user run latched `INTERNAL`
(front_wall≈1.12 m) at the pillar's near face instead of `BUMPOUT`, so the robot
pivoted away from the wall → `WALL_LOST` → HOLD → FAULT. Detector
(`geometry_utils.detect_motion_events`) only returns BUMPOUT when it finds the
INTERNAL+EXTERNAL pair in the same scan; `env_monitoring_v2._detect_and_latch`
latches the first event immediately. Proposed fix: require the same event type on ~3
consecutive scans before latching (and don't latch INTERNAL if a BUMPOUT shows up in
that window). Costs ~0.3–0.5 s before latching a real corner.

## Last verified behaviour
- `just_bump_out.world`, default launch: START → wall-follow → bump-out latched
  (depth ≈ 0.70) → arcs → external corner at pillar's end → internal latch with
  `[BACKUP]` → turn → back on the wall at ~0.755 m, no HOLD/FAULT.
- `pillar_corner_removed.world` (user run): pillar sequence OK, internal corner at the
  top of the long wall OK (crawl positive, no reverse needed), then the notch
  bump-out latched (depth 0.561) before the user stopped it.
