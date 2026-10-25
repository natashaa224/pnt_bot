# Production report - bump_room_10x8, one full lap (sim)

Launch: `corner_bringup.launch.py world:=bump_room_10x8.world spawn 9.093,-7.3, yaw 1.5708`
Code: commit 5a5ff53. Sim only, `fake_paint=True` (see caveats).

## Result
- Lap completed, **0 faults, 0 safety HOLDs** (back at the start after 34.4 m of path).
- Sim time 834.6 s (13.9 min); wall-clock 854.8 s (real-time factor ~0.98).

## Time breakdown (sim, after START)
| State | s | % |
|---|---|---|
| Corner turns / bump arcs (CORNER_LOGIC) | 290.3 | 34.8 |
| Painting (PAINT_EXECUTE, fake 2 s stroke) | 263.7 | 31.6 |
| Corner approach / crawl / reverse | 121.2 | 14.5 |
| Wall follow | 110.7 | 13.3 |
| Wall align | 48.6 | 5.8 |

## Strokes and area (stroke = 0.203 m wide x 1.2 m gantry travel = 0.2436 m^2)
| | count | area |
|---|---|---|
| Strokes commanded | 126 | 30.7 m^2 |
| Skipped by the stroke gate (no movement) | 19 | - |
| Strokes with a wall within 1.0 m on the right | 88 | 21.4 m^2 |
| Strokes with no wall within 1.0 m (21 at 1.0-1.5 m, 6 at 1.5-3 m, 11 at open air) | 38 | 9.3 m^2 |
| **Unique painted wall area** | - | **20.6 m^2** (17.16 m of wall x 1.2 m) |
| Overlapping (duplicate) paint | 8 pairs | 0.85 m^2 (4.7 % of on-wall paint) |

Unique painted length per surface (wall length available): east wall 3.9 m (8), north wall 2.6 m (10),
west wall 2.0 m (8), south wall 1.6 m (10); bump faces 4.8 m, pillar faces 2.2 m.

## Production rate
- Unique painted area / time = 20.6 m^2 / 834.6 s = **0.0247 m^2/s = 88.8 m^2/h**
- Commanded area / time = 30.7 m^2 / 834.6 s = 132.4 m^2/h (includes strokes that would miss the wall)
- Average travel speed 0.041 m/s over the lap.

## Duplicate paint (overlapping strokes on the same surface)
| Strokes | Surface | Overlap (of 0.203 m) |
|---|---|---|
| 40/41 | north pillar | 0.165 m (81 %) |
| 54/56 | north bump 1.0 | 0.169 m (83 %) |
| 117/118 | south bump 0.8 | 0.107 m |
| 117/119 | south bump 0.8 | 0.084 m |
| 103/104 | south pillar | 0.083 m |
| 5/6 | east bump 0.5 | 0.058 m |
| 55/56 | north bump 1.0 | 0.022 m |
| 73/74 | west wall | 0.013 m |
All eight are at corners / bump arcs, where the head sweeps through a turn and consecutive swatches land close together.
Straight wall-follow strokes are spaced 0.218 m (0.015 m gap, no overlap). The stroke gate already removed 19 strokes at zero movement.

## Caveats
- The gantry is not homed or attached in this sim: every stroke logs "Z not homed - cannot start stroke" and completion is faked
  after ~2 s. Areas are *commanded* paint, not measured paint, and stroke time on the real gantry will differ.
- Spray geometry is modelled, not measured: head 0.312 m right of the turn axis, spraying along the robot's right-hand side, reach 1.0 m.
  The 1.0 m reach is my assumption; it decides the on-wall / off-wall split (38 strokes).
- Positions come from wheel odometry mapped to world coordinates from the spawn pose (no ground truth).
- One lap, one run.
