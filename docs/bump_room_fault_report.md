# bump_room_10x8 first run - fault report

World: `bump_room_10x8.world`, spawn (9.093, -7.3) facing north, wall on the robot's right.

## Result
FAULT `obstacle_persist:wall_dist` at the FIRST feature (east wall, 0.5 m flush bump-out).
The other 5 features were never reached.

## Timeline (ROS clock, s after start of the run)
| t | event |
|---|---|
| 0 | START, wall follow north along the east wall |
| +10 | INTERNAL candidate deferred (bump seen 0.22 m ago) - new guard worked |
| +10 | BUMPOUT LATCH depth=0.496 m, R1=0.752 R2=0.300 phi=58.1 deg |
| +44 | ARC1 + ARC2 complete, robot now 0.77 m from the bump FACE (wall dist OK) |
| +45 | detector reports SUCKIN: "detected but execution disabled this build - ignoring" (repeats every scan) |
| +45..+64 | robot keeps wall-following straight; the bump face ends, the long wall is now 0.5 m further away |
| +64 | safety HOLD (wall_dist): right lidar -> wall 1.28 m, target 0.77 +/- 0.10 |
| +79 | HOLD persisted 15 s -> FAULT obstacle_persist:wall_dist |

## Root cause
Bump-outs with depth <= 0.60 m (`SUCKIN_CUTOFF` in `maneuver_planner.py`) are classified by the
detector as SUCKIN once the robot is past the bump face (bump end = external edge, wall return =
internal corner, seen as one compound feature). `env_monitoring_v2` has no handler for SUCKIN
("execution disabled this build"), so nothing latches. The deeper 0.70 m pillar and 1.2 m block
were instead seen as separate EXTERNAL + INTERNAL events, which the existing recipes handle.
With nothing latched, the robot never returns to the wall line, the wall distance grows to 0.77 + 0.5 m,
and the safety monitor faults after 15 s.

## Not the cause
- The new bump_defer_internal guard (it correctly deferred the pillar-like INTERNAL).
- The 1.2 m depth limit (this bump is 0.5 m).

## Fix options
1. When SUCKIN is detected and not enabled, downgrade it to its EXTERNAL part so the existing
   external -> internal recipe returns the robot to the wall (smallest change, same path the 1.2 m block took).
2. Implement a real SUCKIN recipe from the planner's `suckin_radii` (new motion, needs sim sweep).
