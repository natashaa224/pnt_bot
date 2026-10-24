# VRai
# motion_queue.py
#
# Motion queue engine for the paint-bot FSM.
#
# Design (locked in session):
#   - FSM owns state flow; MotionQueue owns the *plan* (what sequence, where in it).
#   - Coarse items: one item == one FSM state execution. Sub-sequencing (turn
#     segments, reacquire) lives inside wall_motion, NOT here.
#   - The queue is a STREAM: when it drains, it refills with the default recipe
#     (normal painting block). Corner recipes splice in on demand (later drop).
#   - Paint interleaves WITHIN an item, driven by item.paints — the queue only
#     sequences motion items, never paint. Paint stays FSM-driven via _paint_caller.
#
# Drop 1 scope: normal-block recipe only. `phase` is carried but inert
# (nothing dispatches on it yet — becomes load-bearing when CORNER_LOGIC splits
# into TURN/ADVANCE in the external-corner drop).

from dataclasses import dataclass, field


@dataclass
class MotionItem:
    """One coarse motion step in the plan.

    state  : the RobotState whose handler executes this item.
    phase  : disambiguates items that share a state (e.g. TURN vs ADVANCE both
             run under CORNER_LOGIC). Inert in drop 1 — populated, not dispatched.
    paints : whether paint interleaves during this item. Drives _paint_caller.
    label  : human-readable tag for logging / HMI queue display.
    """
    state:  object            # RobotState enum member
    phase:  str = ''
    paints: bool = False
    label:  str = ''


class MotionQueue:
    """Ordered plan with a pointer. Held by state_machine_node.

    The queue is never mutated by handlers — they only read current_item() and
    report completion via the ITEM_COMPLETE sentinel. state_machine_node calls
    advance() on sentinel and flush() on safety.

    Stream semantics: advance() past the end refills with default_recipe() so the
    normal paint loop runs forever. splice() inserts a one-shot recipe (corners)
    ahead of the refill.
    """

    def __init__(self, robot_state_cls, logger=None,
                 strips_per_block: int = 4):
        self._S = robot_state_cls          # RobotState class (avoids circular import)
        self._log = logger
        self._strips_per_block = strips_per_block

        self._items: list[MotionItem] = []
        self._ptr: int = 0
        self._active: bool = False         # True once a plan is loaded

    # ── Recipe builders ──────────────────────────────────────────────────────
    # All builders return list[MotionItem]. This is what lets recipes be data:
    # a bump is just build_internal + build_external + build_external + build_internal.


    def build_normal_block(self) -> list[MotionItem]:
        """Normal painting cycle: WALL_ALIGN(paint) + WALL_FOLLOW(no paint),
        repeating. Align checks/corrects heading and paints in place;
        follow just translates 0.203m to the next strip position."""
        S = self._S
        return [
            MotionItem(S.WALL_ALIGN,  phase='ALIGN',  paints=True,  label='align+paint'),
            MotionItem(S.WALL_FOLLOW, phase='FOLLOW', paints=False, label='follow'),
        ]



    def build_bumpout(self, corner_params=None, paints: bool = True) -> list[MotionItem]:
        """Bump-out (shallow, d < 0.8 m): approach → OUT arc → BACK arc → align.

        paints : master gate for this recipe's paint interleave. Set False to test
                the trajectory alone (no actuator) — arcs complete on odom/turn
                signals, never on a stroke, so the maneuver runs paint-independent.
                ARC2 never paints regardless (BACK arc does not paint face B).
        """
        S = self._S
        return [
            MotionItem(S.CORNER_APPROACH, phase='CRAWL', paints=paints,
                    label='bump_approach'),
            MotionItem(S.CORNER_LOGIC,    phase='ARC1',  paints=paints,
                    label='bump_out_arc'),
            MotionItem(S.CORNER_LOGIC,    phase='ARC2',  paints=paints,
                    label='bump_back_arc'),
            MotionItem(S.WALL_ALIGN,      phase='ALIGN', paints=False,
                    label='bump_align'),
        ]

    def build_external(self, corner_params=None) -> list[MotionItem]:
        """External (convex) corner — ARC around the vertex (not in-place).
        Same 3-item shape as build_internal; the pivot-vs-arc difference is the
        turn_radius in corner_params (0 = internal pivot, real = external arc),
        consumed by wall_motion. Phase stays 'TURN' so CornerLogicHandler's
        rotation path (completes on corner_turn_complete) handles it unchanged."""
        S = self._S
        return [
            MotionItem(S.CORNER_APPROACH, phase='CRAWL', paints=True,
                    label='corner_approach'),
            MotionItem(S.CORNER_LOGIC,    phase='TURN',  paints=True,
                    label='corner_arc'),
            MotionItem(S.WALL_ALIGN,      phase='ALIGN', paints=True,
                    label='corner_align+paint'),
        ]


    # def build_external(self, corner_params=None) -> list[MotionItem]:
    #     """External (convex) corner sequence — in-place CW pivot.

    #     4 items. There is no RESUME item: when ADVANCE completes, the queue
    #     splices a fresh normal_block, so re-entry is the same WALL_ALIGN→paint→
    #     WALL_FOLLOW loop every block uses.

    #     CORNER_APPROACH/CRAWL : crawl along old wall, paint per strip, until env
    #                             fires corner_preempt (lidar→vertex range gate).
    #     CORNER_LOGIC/TURN     : in-place CW pivot, segmented, paint per segment,
    #                             until corner_turn_complete.
    #     WALL_ALIGN/ALIGN      : re-align to the new wall. No paint.
    #     CORNER_LOGIC/ADVANCE  : crawl forward watching for right wall to reacquire;
    #                             on complete → normal_block spliced in.
    #     """
    #     S = self._S
    #     return [
    #         MotionItem(S.CORNER_APPROACH, phase='CRAWL',   paints=True,
    #                    label='corner_approach'),
    #         MotionItem(S.CORNER_LOGIC,    phase='TURN',    paints=True,
    #                    label='corner_turn'),
    #         MotionItem(S.CORNER_LOGIC,    phase='REACQUIRE', paints=False,
    #                    label='corner_reacquire'),
    #         MotionItem(S.WALL_ALIGN,      phase='ALIGN',   paints=False,
    #                    label='corner_align'),
    #     ]

    def build_internal(self, corner_params=None) -> list[MotionItem]:
        """Internal corner: approach → pivot in place → align. No REACQUIRE
        (the followed wall is immediately reachable after the in-place turn)."""
        S = self._S
        return [
            MotionItem(S.CORNER_APPROACH, phase='CRAWL', paints=True,
                       label='corner_approach'),
            MotionItem(S.CORNER_LOGIC,    phase='TURN',  paints=True,
                       label='corner_turn'),
            MotionItem(S.WALL_ALIGN,      phase='ALIGN', paints=False,
                       label='corner_align'),
        ]    

    def default_recipe(self) -> list[MotionItem]:
        """What to enqueue when the plan drains. Normal painting for now."""
        return self.build_normal_block()

    # ── Stream control ───────────────────────────────────────────────────────

    def start(self):
        """Load the first plan. Call once when FSM leaves IDLE into painting."""
        self._items = self.default_recipe()
        self._ptr = 0
        self._active = True
        self._info(f'[QUEUE] started — {self._describe()}')

    def current_item(self) -> MotionItem | None:
        if not self._active or self._ptr >= len(self._items):
            return None
        return self._items[self._ptr]

    def advance(self) -> MotionItem | None:
        """Move to next item. Refill with default recipe if the plan drains.
        Returns the new current item (never None while active)."""
        if not self._active:
            return None
        self._ptr += 1
        if self._ptr >= len(self._items):
            self._items = self.default_recipe()
            self._ptr = 0
            self._info(f'[QUEUE] drained → refilled — {self._describe()}')
        item = self._items[self._ptr]
        self._info(f'[QUEUE] advance → [{self._ptr}] {item.label} '
                   f'(state={item.state.name} paints={item.paints})')
        return item

    # def splice_bumpout(self, corner_params=None):
    #     """Splice the bump-out recipe. Same insert-after-current logic as the
    #     other corner recipes."""
    #     S = self._S
    #     cur = self.current_item()
    #     if cur is not None and cur.state in (S.CORNER_APPROACH, S.CORNER_LOGIC):
    #         self._info('[QUEUE] splice_bumpout ignored — already in corner recipe')
    #         return
    #     self._items = self._items[:self._ptr + 1] + self.build_bumpout(corner_params)
    #     self._active = True
    #     self._info(f'[QUEUE] splice_bumpout — {self._describe()}')

    def splice_bumpout(self, corner_params=None, paints: bool = True):
        S = self._S
        cur = self.current_item()
        if cur is not None and cur.state in (S.CORNER_APPROACH, S.CORNER_LOGIC):
            self._info('[QUEUE] splice_bumpout ignored — already in corner recipe')
            return
        self._items = self._items[:self._ptr + 1] + self.build_bumpout(corner_params, paints)
        self._active = True
        self._info(f'[QUEUE] splice_bumpout(paints={paints}) — {self._describe()}')    

    def splice_internal(self, corner_params=None):
        """Splice the internal-corner recipe (approach → turn → align, no
        REACQUIRE). Same insert-after-current logic as external."""
        S = self._S
        cur = self.current_item()
        if cur is not None and cur.state in (S.CORNER_APPROACH, S.CORNER_LOGIC):
            self._info('[QUEUE] splice_internal ignored — already in corner recipe')
            return
        self._items = self._items[:self._ptr + 1] + self.build_internal(corner_params)
        self._active = True
        self._info(f'[QUEUE] splice_internal — {self._describe()}')
            

    def splice_external(self, corner_params=None):
        """Splice the external-corner recipe so the NEXT advance() lands on
        CORNER_APPROACH.

        The item live at splice time (a normal WALL_FOLLOW/STEP_REALIGN) is
        usually mid-step; it will finish its stroke and call advance(). So the
        recipe must be inserted AFTER the current item — at ptr+1 — otherwise the
        pending advance() skips straight over CORNER_APPROACH into the turn.

        Everything after the current item (rest of the normal block) is dropped.
        After the recipe's last item (ADVANCE) completes, advance() drains →
        default_recipe() refills a fresh normal_block automatically.
        """
        S = self._S
        cur = self.current_item()
        if cur is not None and cur.state in (S.CORNER_APPROACH, S.CORNER_LOGIC):
            self._info('[QUEUE] splice_external ignored — already in corner recipe')
            return
        recipe = self.build_external(corner_params)
        # keep items up to AND INCLUDING the current one, then the recipe
        self._items = self._items[:self._ptr + 1] + recipe
        self._active = True
        self._info(f'[QUEUE] splice_external — {self._describe()}')

    def splice(self, recipe_items: list[MotionItem]):
        """Generic splice: insert after current item. Kept for future recipes
        (internal corner, bump). Inserts AFTER current, preserving it."""
        head = self._items[:self._ptr + 1]
        self._items = head + recipe_items
        self._active = True
        self._info(f'[QUEUE] spliced {len(recipe_items)} items — {self._describe()}')

    def flush(self):
        """Clear the plan. Called on STOP / safety fault."""
        self._items = []
        self._ptr = 0
        self._active = False
        self._info('[QUEUE] flushed')

    def is_active(self) -> bool:
        return self._active and self._ptr < len(self._items)

    # ── Introspection (HMI / logging) ────────────────────────────────────────

    def upcoming_labels(self) -> list[str]:
        """Labels from current item onward — for /motion_queue HMI display."""
        if not self._active:
            return []
        return [it.label for it in self._items[self._ptr:]]

    def _describe(self) -> str:
        return (f'ptr={self._ptr}/{len(self._items)} '
                f'plan=[{", ".join(it.label for it in self._items)}]')

    def _info(self, msg: str):
        if self._log is not None:
            self._log.info(msg)
