#!/usr/bin/env python3
"""
maneuver_planner.py — maneuver geometry planner.

Boundary in the architecture:
    detector / env2  ->  feature description {type, depth, vertices, ...}
                                |
                                v
                       plan_maneuver(...)
                                |
                                v
    FSM / wall_motion executes the returned steps -> cmd_vel

This module does NOT detect features and does NOT execute motion.

All trigger distances are TURN-AXIS frame. env2 owns LiDAR offsets and converts
the returned trigger into the LiDAR-frame quantity it needs.

BUMPOUT:
    Uses a compact analytical two-arc schedule derived from the offline
    trajectory / clearance study:

        R1 = clamp(1.0 - 0.5*d, 0.60, 0.85)
        R2 = 0.30
        trigger_axis = 0.90

        phi = acos(1 - d / (R1 + R2))

    ARC1 uses +phi with R1; ARC2 uses -phi with R2.

    env2 owns the maximum accepted BUMPOUT depth. The planner itself does not
    impose a BUMPOUT depth cutoff or switch a deep bump into separate corners.

Trigger types:
  FRONT_WALL        : turn-axis distance to wall B / V1 at which to fire
  ODOM_PAST_VERTEX  : turn-axis distance travelled past a latched vertex
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional


# ── locked robot / task geometry ──────────────────────────────────────────────
SPRAY_STANDOFF    = 0.304
SPRAY_D_ORIGIN    = 0.600
AXIS_STANDOFF     = SPRAY_STANDOFF + SPRAY_D_ORIGIN      # 0.904

R_INTERNAL_SPIN   = 0.0
R_EXTERNAL_DEF    = 0.80

SUCKIN_CUTOFF     = 0.60
SUCKIN_R1_FLOOR   = 0.60

EXT_TRIGGER_PAST  = AXIS_STANDOFF - R_EXTERNAL_DEF       # 0.104 for R=0.8
INT_TRIGGER_FRONT = AXIS_STANDOFF                         # 0.904
SUCKIN_START_PAST_V1 = 0.30


# ── BUMPOUT analytical schedule ───────────────────────────────────────────────
BUMP_TRIGGER_AXIS = 0.90

BUMP_R1_A         = 1.00
BUMP_R1_B         = 0.50
BUMP_R1_MIN       = 0.60
BUMP_R1_MAX       = 0.85

BUMP_R2           = 0.30


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ── BUMPOUT geometry ──────────────────────────────────────────────────────────
def bumpout_geometry(d):
    """
    Return the deterministic two-arc BUMPOUT geometry:

        R1            tip radius for ARC1 / OUT
        R2            tip radius for ARC2 / BACK
        phi           equal swing magnitude in radians
        trigger_axis  positive turn-axis distance before V1

    The schedule was selected from the offline clearance / standoff study so
    runtime planning has no optimizer, table lookup, or discrete mode switching.

    env2 owns the operational maximum bump depth.
    """
    d = float(d)

    if d <= 0.0:
        raise ValueError("BUMPOUT depth must be > 0")

    R1 = _clamp(
        BUMP_R1_A - BUMP_R1_B * d,
        BUMP_R1_MIN,
        BUMP_R1_MAX,
    )
    R2 = BUMP_R2

    arg = 1.0 - d / (R1 + R2)

    # Clamp only for numerical robustness. For the accepted env2 operating
    # range this should already lie inside [-1, +1].
    phi = math.acos(_clamp(arg, -1.0, 1.0))

    return R1, R2, phi, BUMP_TRIGGER_AXIS


# ── legacy/simple schedules still used by SUCKIN ─────────────────────────────
def suckin_radii(d):
    """Compound suck-in tip radii, R1 floored at 0.60 m."""
    R1 = max(_clamp(0.34 + 0.42*d, 0.25, 0.95), SUCKIN_R1_FLOOR)
    R2 = _clamp(0.30 + 0.10*d, 0.25, 0.95)
    return R1, R2


def external_radius(d):
    """External radius helper retained for suck-in offline/reference branch."""
    return _clamp(d, 0.60, 0.80)


# ── planner result types ──────────────────────────────────────────────────────
@dataclass
class Step:
    maneuver: str
    trigger_type: str
    trigger_axis: float

    R_axis: Optional[float] = None

    # Compound-curve tip radii.
    R1: Optional[float] = None
    R2: Optional[float] = None

    # Equal swing magnitude for compound ARC1/ARC2.
    # ARC1 uses +phi for BUMPOUT; ARC2 uses -phi.
    phi: Optional[float] = None

    turn_dir: str = "CW"
    note: str = ""


@dataclass
class Plan:
    feature: str
    depth: Optional[float]
    steps: List[Step] = field(default_factory=list)
    feasible: bool = True
    reason: str = ""


# ── main planner ──────────────────────────────────────────────────────────────
def plan_maneuver(feature_type, depth=None, turn_dir=None, radius=None):
    """
    Return a Plan for a detected feature.

    INTERNAL:
        Existing in-place spin path.

    EXTERNAL:
        Existing external-arc path. env2 may provide a radius; this branch is
        otherwise unchanged.

    BUMP / BUMPOUT:
        env2 decides whether the detected depth is permitted.
        Planner returns one deterministic compound two-arc geometry.
        No internal BUMPOUT depth cutoff is applied here.

    SUCKIN:
        Existing behaviour retained unchanged.
    """
    t = feature_type.upper()

    if t == "INTERNAL":
        return Plan(
            "INTERNAL",
            depth,
            [
                Step(
                    "internal_spin",
                    "FRONT_WALL",
                    INT_TRIGGER_FRONT,
                    R_axis=R_INTERNAL_SPIN,
                    turn_dir=turn_dir or "CCW",
                    note="in-place spin",
                )
            ],
        )

    if t == "EXTERNAL":
        Re = (
            R_EXTERNAL_DEF
            if radius is None
            else _clamp(radius, 0.60, 0.80)
        )

        return Plan(
            "EXTERNAL",
            depth,
            [
                Step(
                    "external_arc",
                    "ODOM_PAST_VERTEX",
                    AXIS_STANDOFF - Re,
                    R_axis=Re,
                    turn_dir=turn_dir or "CW",
                    note=f"arc R={Re:.2f}",
                )
            ],
        )

    if t in ("BUMP", "BUMPOUT"):
        if depth is None:
            raise ValueError("BUMP needs depth")

        R1, R2, phi, trigger_axis = bumpout_geometry(float(depth))

        return Plan(
            "BUMP",
            depth,
            [
                Step(
                    "bump_compound",
                    "FRONT_WALL",
                    trigger_axis,
                    R1=R1,
                    R2=R2,
                    phi=phi,
                    turn_dir="CCW",
                    note=(
                        f"analytical bump: "
                        f"R1={R1:.2f} "
                        f"R2={R2:.2f} "
                        f"phi={math.degrees(phi):.1f}deg "
                        f"trigger={trigger_axis:.3f}"
                    ),
                )
            ],
        )

    if t == "SUCKIN":
        if depth is None:
            raise ValueError("SUCKIN needs depth")

        if depth <= SUCKIN_CUTOFF:
            R1, R2 = suckin_radii(depth)

            return Plan(
                "SUCKIN",
                depth,
                [
                    Step(
                        "suckin_compound",
                        "ODOM_PAST_VERTEX",
                        SUCKIN_START_PAST_V1,
                        R1=R1,
                        R2=R2,
                        note="compound external->internal, R1 floored 0.60",
                    )
                ],
            )

        # Retained as offline/reference behaviour for now.
        Re = external_radius(depth)

        return Plan(
            "SUCKIN",
            depth,
            [
                Step(
                    "external_arc",
                    "ODOM_PAST_VERTEX",
                    AXIS_STANDOFF-Re,
                    R_axis=Re,
                    turn_dir="CW",
                    note=f"[OFFLINE/REF] round V1 off baseline, Re={Re:.2f}",
                ),
                Step(
                    "internal_spin",
                    "FRONT_WALL",
                    INT_TRIGGER_FRONT,
                    R_axis=0.0,
                    turn_dir="CCW",
                    note="[OFFLINE/REF] round V2 back onto baseline",
                ),
            ],
            feasible=False,
            reason="deep suck-in handling not yet converted to runtime compound planner",
        )

    raise ValueError(f"unknown feature_type {feature_type}")


def _fmt(plan):
    out = [
        f"Plan: {plan.feature}"
        + (f" d={plan.depth}" if plan.depth is not None else "")
        + ("" if plan.feasible else f"  [INFEASIBLE: {plan.reason}]")
    ]

    for i, s in enumerate(plan.steps):
        if s.R_axis is not None:
            radius_txt = f"R={s.R_axis:.3f}"
        else:
            radius_txt = f"R1={s.R1:.3f} R2={s.R2:.3f}"

        phi_txt = ""
        if s.phi is not None:
            phi_txt = f" phi={math.degrees(s.phi):.1f}deg"

        out.append(
            f"  step {i}: {s.maneuver:16s} "
            f"{radius_txt:20s} "
            f"{s.turn_dir:3s} "
            f"{s.trigger_type:17s} "
            f"trig_axis={s.trigger_axis:.3f}"
            f"{phi_txt}  # {s.note}"
        )

    return "\n".join(out)


if __name__ == "__main__":
    print(_fmt(plan_maneuver("INTERNAL"))); print()

    print(_fmt(plan_maneuver("EXTERNAL"))); print()
    print(_fmt(plan_maneuver("EXTERNAL", radius=0.70))); print()

    for d in (0.40, 0.50, 0.60, 0.70, 0.80):
        print(_fmt(plan_maneuver("BUMP", depth=d)))
        print()

    print(_fmt(plan_maneuver("SUCKIN", depth=0.40))); print()
    print(_fmt(plan_maneuver("SUCKIN", depth=0.70))); print()
