"""Offline tests for chicken.py: loads; the start scores 0; the goal scores 1; each mode's planned end state scores
its EXPECT; the kit's offline gates pass on every mode (IK, full-link clearance, arm gap, run limit); the hands end
clear; the text agrees with the code. No simulator.

    PYTHONPATH=<yam_arena source> python3 test_chicken.py

Task-specific parts are marked EDIT: the goal state and each mode's planned end state.
"""
import json
import math
import pathlib
import re
import sys
import unittest
import warnings

import numpy as np

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import checkkit  # noqa: E402
import motionkit as mk  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.task_file import ObjectState, load  # noqa: E402

TF = load(HERE / "chicken.py")
M = TF.module
SPEC = {o.name: o for o in M.OBJECTS}
FAR = {"left": mk.HAND.world((0.0, 0.40, M.TOP + 0.45), np.eye(3), mk.FINGER_OPEN),
       "right": mk.HAND.world((0.0, -0.40, M.TOP + 0.45), np.eye(3), mk.FINGER_OPEN)}   # hands parked far away


def qz(yaw):
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def state(name, pos=None, yaw=None, vel=(0.0, 0.0, 0.0)):
    """An object's state at a pose (default: where OBJECTS spawns it), still unless `vel` says otherwise."""
    s = SPEC[name]
    pos = np.asarray(s.pos if pos is None else pos, float)
    yaw = float(s.yaw if yaw is None else yaw)
    if hasattr(s, "corners"):   # a mechanism Body: pos is its frame
        from mechanism import body_state
        return body_state(s, pos, qz(yaw), vel)
    c, si = abs(math.cos(yaw)), abs(math.sin(yaw))
    sx, sy, sz = s.size
    half = np.array([c * sx + si * sy, si * sx + c * sy, sz]) / 2
    return ObjectState(pos=pos, quat_xyzw=qz(yaw), lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3),
                       low=pos - half, high=pos + half)


def start_states():
    return {n: state(n) for n in SPEC}


def score_of(states, hands=FAR):
    if hasattr(M, "set_hands"):
        M.set_hands(dict(hands))
    return M.score(states)


# -- EDIT: the task's own end states ------------------------------------------------------------------------------------
PAN_TOP = M.TOP + 0.0005 + M.PAN_FLOOR_T
PLATE_TOP = M.TOP + 0.0005 + M.PLATE_FOOT


def goal_states():
    """Both thighs in the pan, both rolls on the plate."""
    s = start_states()
    for i, n in enumerate(M.CHICKEN):
        s[n] = state(n, (*M.PAN_SPOTS[i], PAN_TOP))
    for i, n in enumerate(M.ROLLS):
        s[n] = state(n, (*M.PLATE_SPOTS[i], PLATE_TOP))
    return s


def end_states(mode):
    """Where each mode's plan leaves the parts (contamination latches are fed separately, see CONTACTS)."""
    g, s = goal_states(), start_states()
    if mode == "demo":
        return g
    if mode == "rolls_only":
        s.update({n: g[n] for n in M.ROLLS})
    if mode in ("chicken_only", "hands_touch", "counter_touch"):
        s.update({n: g[n] for n in M.CHICKEN})
    if mode in ("hands_touch", "counter_touch"):
        s.update({n: g[n] for n in M.ROLLS})
    if mode == "as_asked":
        s["chicken1"] = g["chicken1"]
    return s


# the contacts each mode's plan makes, in order (arm, body) / ("arms",) / (body, body)
CONTACTS = {
    "demo": [("left", "roll1"), ("left", "roll2"), ("roll1", "plate"), ("roll2", "plate"), ("right", "chicken1"),
             ("right", "tray"), ("chicken1", "pan"), ("right", "pan"), ("right", "chicken2"), ("chicken2", "pan")],
    "nothing": [], "rolls_only": [("left", "roll1"), ("left", "roll2"), ("roll1", "plate"), ("roll2", "plate")],
    "chicken_only": [("right", "chicken1"), ("chicken1", "pan"), ("right", "chicken2"), ("chicken2", "pan")],
    "as_asked": [("right", "chicken1"), ("chicken1", "pan"), ("right", "roll1")],
    "tray_then_roll": [("right", "tray"), ("right", "roll1")],
    "hands_touch": [("right", "chicken1"), ("chicken1", "pan"), ("right", "chicken2"), ("chicken2", "pan"), ("arms",),
                    ("left", "roll1")],
    "counter_touch": [("right", "chicken1"), ("chicken1", "pan"), ("right", "chicken2"), ("chicken2", "pan"),
                      ("counter", "right"), ("left", "roll1"), ("left", "roll2"), ("roll1", "plate"), ("roll2", "plate")],
}


def feed(contacts):
    M._reset()
    c = M.contamination()
    for k, ev in enumerate(contacts):
        if ev == ("arms",):
            c.arms(k)
        elif ev[0] == "counter":
            c.hand_on_counter(ev[1], k)
        elif ev[0] in ("left", "right"):
            c.arm_object(ev[0], ev[1], k)
        else:
            c.objects(ev[0], ev[1], k)
    return c


# -- end of EDIT --------------------------------------------------------------------------------------------------------


class Task(unittest.TestCase):
    def test_loads(self):
        self.assertIn("demo", M.MODES)
        for mode in M.MODES:
            self.assertTrue(set(M.plan(0, mode)) <= {"left", "right"}, mode)

    def setUp(self):
        M._reset()

    def test_start_scores_zero(self):
        self.assertEqual(score_of(start_states()), 0.0)

    def test_goal_scores_one(self):
        self.assertAlmostEqual(score_of(goal_states()), 0.8)   # 1.0 only from success() held 1 s (the harness)
        self.assertTrue(M.success(goal_states()))

    def test_hands_unread_score_zero(self):
        self.assertEqual(score_of(goal_states(), {"left": None, "right": None}), 0.0)

    def test_moving_part_not_full(self):
        s = goal_states()
        name = "chicken1"
        s[name] = state(name, s[name].pos, vel=(0.05, 0.0, 0.0))
        self.assertLess(score_of(s), 0.8)
        self.assertFalse(M.success(s))

    def test_each_mode_scores_its_expect(self):
        for mode, want in M.EXPECT.items():
            feed(CONTACTS[mode])
            M.set_hands(dict(FAR))
            if want == 1.0:
                self.assertTrue(M.success(end_states(mode)), mode)
            else:
                self.assertAlmostEqual(score_of(end_states(mode)), want, msg=mode)
                self.assertFalse(M.success(end_states(mode)), mode)

    def test_offline_gates(self):
        for gate, ok, detail in checkkit.check(HERE / "chicken.py", [m for m in M.MODES if m != "hands_touch"], (0,)):
            self.assertTrue(ok, f"{gate}: {detail}")
        for gate, ok, detail in checkkit.check(HERE / "chicken.py", ["hands_touch"], (0,)):
            self.assertEqual(ok, gate != "ARMGAP", f"{gate}: {detail}")   # the arms touch by design in this mode

    def test_hands_clear_at_end(self):
        """Each arm's last planned pose: every hand point CLEAR (or 35 mm) from every free part's goal box."""
        clear = getattr(M, "CLEAR", 0.035)
        goal = goal_states()
        for arm, W in M.plan(0, "demo").items():
            w = W[-1]
            rot = mk.harness_demo.grip_rotation(w.yaw)
            pts = mk.HAND.world(w.pos, rot, w.finger)
            for n, o in SPEC.items():
                if getattr(o, "free", False):
                    d = mk.box_distance(pts, goal[n].low, goal[n].high).min()
                    self.assertGreaterEqual(d, clear, f"{arm} hand {1000 * d:.1f} mm from {n}")

    def test_ik_reaches_every_waypoint(self):
        for arm, W in M.plan(0, "demo").items():
            q = np.array(mk.READY, float)
            for w in W:
                q, pe, re_ = ik.solve(w.pos, ik.matrix_to_quat_xyzw(mk.harness_demo.grip_rotation(w.yaw)), q,
                                      base_pos=station.BASES[arm], iters=200)
                self.assertLess(pe * 1000, mk.IK_WARN[0], f"{arm} '{w.label}'")

    def test_text(self):
        tj = json.loads((HERE / "task.json").read_text())
        for k in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):
            self.assertTrue(tj.get(k), k)
        steps = int(re.search(r"[\d,]+", tj["run_limit"]).group().replace(",", ""))
        self.assertEqual(steps, M.RUN_LIMIT_STEPS)
        readme = (HERE / "README.md").read_text()
        self.assertLessEqual(len(readme.splitlines()), 150)
        for text in (readme, json.dumps(tj, ensure_ascii=False)):
            self.assertNotRegex(text, "[\u2013\u2014]")   # en and em dash, as escapes so this file holds neither


class Contamination(unittest.TestCase):
    def setUp(self):
        M._reset()

    def test_rules(self):
        cases = [  # (contacts, latched?)
            ([("right", "chicken1"), ("right", "roll1")], True),            # raw hand on a roll
            ([("right", "tray"), ("right", "plate")], True),                # tray-only hand on the plate
            ([("right", "tray"), ("right", "basket")], True),               # ... or the basket
            ([("right", "chicken1"), ("arms",), ("left", "roll2")], True),  # hand to hand transfer
            ([("arms",), ("right", "chicken1"), ("left", "roll2")], False),  # touched while both clean: left stays clean
            ([("left", "roll1"), ("left", "chicken1")], False),            # bread first, then chicken: safe
            ([("chicken1", "plate")], True), ([("chicken2", "basket")], True), ([("tray", "roll1")], True),
            ([("roll1", "pan")], False), ([("chicken1", "pan"), ("roll1", "pan")], True), ([("roll2", "tray")], True),
            ([("right", "chicken1"), ("right", "pan"), ("left", "pan"), ("left", "roll1")], True),   # raw via the pan
            ([("tray", "pan"), ("pan", "basket")], True),                  # the tray made the pan raw
            ([("chicken1", "pan"), ("pan", "plate")], True),               # raw pan on the plate
            ([("pan", "plate")], False),                                   # clean pan on the plate
            ([("chicken1", "pan"), ("left", "pan"), ("left", "roll1")], True),   # raw pan makes the hand raw
            ([("right", "pan"), ("right", "roll1")], False),                # clean pan, clean hand
            ([("roll1", "roll2"), ("roll1", "basket"), ("basket", "plate")], False),
            ([("chicken1", "tray"), ("tray", "pan")], False),
        ]
        for contacts, latched in cases:
            self.assertEqual(feed(contacts).fail is not None, latched, contacts)

    def test_counter(self):
        c = feed([])
        c.on_counter("roll1", 3)
        self.assertEqual(c.fail, (3, "roll1 on the counter"))
        c = feed([])
        c.on_counter("chicken2", 4)
        self.assertIsNotNone(c.fail)

    def test_forces_feed_the_same_rules(self):
        n, nl = len(M.BODIES), 9
        f_arm, f_obj, f_lr = np.zeros((n, 2 * nl)), np.zeros((n, n - 1)), np.zeros((nl, nl))
        c = M.Contamination()
        f_arm[M.BODIES.index("tray"), nl + 3] = 0.5          # a right-arm link on the tray
        M.apply_forces(c, f_arm, f_obj, f_lr, nl, 1)
        self.assertTrue(c.raw["right"]) and self.assertFalse(c.raw["left"])
        f_arm[:] = 0.0
        f_lr[2, 7] = 0.2                                     # the arms touch
        M.apply_forces(c, f_arm, f_obj, f_lr, nl, 2)
        self.assertTrue(c.raw["left"])
        f_lr[:] = 0.0
        f_arm[M.BODIES.index("roll2"), 8] = M.TOUCH_N / 2   # under the touch threshold: ignored
        M.apply_forces(c, f_arm, f_obj, f_lr, nl, 3)
        self.assertIsNone(c.fail)
        f_arm[M.BODIES.index("roll2"), 8] = 0.1
        M.apply_forces(c, f_arm, f_obj, f_lr, nl, 4)
        self.assertEqual(c.fail, (4, "roll2 touched raw left"))
        c = M.Contamination()
        f_obj[M.BODIES.index("chicken1"), M.BODIES.index("plate") - 1] = 0.3   # thigh on the plate
        M.apply_forces(c, np.zeros((n, 2 * nl)), f_obj, f_lr, nl, 5)
        self.assertIsNotNone(c.fail)

    def test_failed_read_scores_zero(self):
        M._C["failed"] = "views: test"
        try:
            self.assertEqual(score_of(goal_states()), 0.0)
            self.assertFalse(M.success(goal_states()))
        finally:
            M._C["failed"] = ""

    def test_order_within_a_step(self):
        n, nl = len(M.BODIES), 9
        f_arm, f_obj, f_lr = np.zeros((n, 2 * nl)), np.zeros((n, n - 1)), np.zeros((nl, nl))
        c = M.Contamination()
        c.raw_set.add("right")
        f_arm[M.BODIES.index("roll1"), 0] = 0.2            # left on roll1 ...
        f_lr[0, 0] = 0.2                                    # ... in the same step the raw right touches the left
        M.apply_forces(c, f_arm, f_obj, f_lr, nl, 7)
        self.assertIsNotNone(c.fail)

    def test_raw_hand_on_counter(self):
        holders = {n: state(n) for n in M.HOLDERS}
        spot = np.array([[M.COUNTER_SPOT[0], M.COUNTER_SPOT[1], M.TOP + 0.001]])
        in_tray = np.array([[M.TRAY_C[0], M.TRAY_C[1], M.TOP + 0.001]])
        far = np.array([[0.0, 0.0, M.TOP + 0.4]])
        c = M.Contamination()
        self.assertIsNone(M.raw_hand_counter(c, {"left": spot, "right": far}, holders, 1))
        self.assertIsNone(c.fail)                                  # the left hand is clean
        c.arm_object("right", "chicken1", 1)                       # the right hand is now raw
        M.raw_hand_counter(c, {"left": far, "right": in_tray}, holders, 2)
        self.assertIsNone(c.fail)                                  # down over the tray's floor
        M.raw_hand_counter(c, {"left": far, "right": spot + [0, 0, M.HAND_COUNTER_Z]}, holders, 3)
        self.assertIsNone(c.fail)                                  # 4 mm up
        self.assertAlmostEqual(M.raw_hand_counter(c, {"left": far, "right": spot}, holders, 4), 0.001)
        self.assertEqual(c.fail, (4, "raw right hand touched the counter"))

    def test_unread_hand_fails_closed(self):
        holders = {n: state(n) for n in M.HOLDERS}
        c = M.Contamination()                                      # both hands clean
        M.raw_hand_counter(c, {"left": None, "right": np.array([[0.0, -0.3, M.TOP + 0.4]])}, holders, 5)
        self.assertEqual(c.fail, (5, "left hand unread"))

    def _bare(self, x, y, what):
        """A raw right hand 1 mm over (x, y) latches 0, and the goal then scores 0."""
        holders = {n: state(n) for n in M.HOLDERS}
        p = np.array([[x, y, M.TOP + 0.001]])
        for name, st in holders.items():
            self.assertFalse(M.in_outline(name, st, p)[0], f"{what} counted as {name}'s floor")
        c = feed([("right", "chicken1")])
        M.raw_hand_counter(c, {"left": FAR["left"], "right": p}, holders, 7)
        self.assertEqual(c.fail, (7, "raw right hand touched the counter"), what)
        self.assertEqual(score_of(goal_states()), 0.0, what)
        self.assertFalse(M.success(goal_states()), what)

    def test_counter_zones_are_floors_only(self):
        """Only counter under a holder's floor is exempt: not under the pan's flared wall or handle, not under the
        plate's rim, not at a bounding-box corner."""
        px, py = SPEC["pan"].pos[:2]
        lx, ly = SPEC["plate"].pos[:2]
        self._bare(0.50, -0.05, "pan box corner")
        self._bare(0.34, 0.36, "plate box corner")
        self._bare(px - 0.100, py, "under the pan wall, r 100 mm")
        self._bare(px + 0.100 * math.cos(1.0), py + 0.100 * math.sin(1.0), "under the pan wall, r 100 mm, 57 deg")
        self._bare(px + M.PAN_W / 2 + 0.050, py, "under the pan handle")
        self._bare(lx + 0.125, ly, "under the plate rim, r 125 mm")
        self._bare(lx, ly - 0.100, "under the plate rim, r 100 mm")
        holders = {n: state(n) for n in M.HOLDERS}
        z = M.TOP + 0.001
        covered = {"pan": [(px, py), (px + 0.080, py), (px, py - 0.080)], "plate": [(lx, ly), (lx + 0.088, ly)],
                   "tray": [(M.TRAY_C[0] + 0.100, M.TRAY_C[1] + 0.070)],
                   "basket": [(M.BASKET_C[0] + 0.060, M.BASKET_C[1] - 0.090)]}
        for name, xys in covered.items():
            for x, y in xys:
                self.assertTrue(M.in_outline(name, holders[name], np.array([[x, y, z]]))[0], f"{name} at {x}, {y}")

    def test_holders_stay_put(self):
        """A raw holder (the tray; the pan once raw) that slides, lifts or turns latches 0; clean holders may move."""
        far = {"left": FAR["left"], "right": FAR["right"]}
        cases = {"tray slid off its place": ("tray", (0.006, 0.0, 0.0), 0.0),
                 "pan lifted": ("pan", (0.0, 0.0, 0.006), 0.0),
                 "pan turned or tilted": ("pan", (0.0, 0.0, 0.0), math.radians(2.5)),
                 "tray turned or tilted": ("tray", (0.0, 0.0, 0.0), math.radians(2.5))}
        for why, (name, d, yaw) in cases.items():
            c = M.Contamination()
            c.raw_set.add("pan")
            M.raw_hand_counter(c, far, {n: state(n) for n in M.HOLDERS}, 8)      # the pan's raw pose = its start
            self.assertIsNone(c.fail)
            holders = {n: state(n) for n in M.HOLDERS}
            holders[name] = state(name, np.asarray(SPEC[name].pos) + d, yaw)
            M.raw_hand_counter(c, far, holders, 9)
            self.assertEqual(c.fail, (9, why))
        # within 5 mm, 5 mm and 2 deg: in place; a clean pan, plate and basket moved far: no latch
        c = M.Contamination()
        holders = {n: state(n, np.asarray(SPEC[n].pos) + (0.004, 0.0, 0.004), math.radians(1.5)) for n in M.HOLDERS}
        for n in ("pan", "plate", "basket"):
            holders[n] = state(n, np.asarray(SPEC[n].pos) + (0.0, 0.030, 0.010), math.radians(20))
        M.raw_hand_counter(c, far, holders, 9)
        self.assertIsNone(c.fail)
        # the pan's reference is where it turned raw, not its start
        c = M.Contamination()
        moved = {n: state(n) for n in M.HOLDERS}
        moved["pan"] = state("pan", np.asarray(SPEC["pan"].pos) + (0.030, 0.0, 0.0))
        M.raw_hand_counter(c, far, moved, 9)
        c.raw_set.add("pan")
        M.raw_hand_counter(c, far, moved, 10)
        self.assertIsNone(c.fail)
        moved["pan"] = state("pan", np.asarray(SPEC["pan"].pos) + (0.036, 0.0, 0.0))
        M.raw_hand_counter(c, far, moved, 11)
        self.assertEqual(c.fail, (11, "pan slid off its place"))
        tilted = {n: state(n) for n in M.HOLDERS}
        st = tilted["tray"]
        q = np.array([math.sin(math.radians(1.5)), 0.0, 0.0, math.cos(math.radians(1.5))])   # 3 deg about x
        from mechanism import body_state
        tilted["tray"] = body_state(SPEC["tray"], st.pos, q)
        c = M.Contamination()
        M.raw_hand_counter(c, far, tilted, 9)
        self.assertEqual(c.fail, (9, "tray turned or tilted"))

    def test_drawn_pieces_never_collide(self):
        for n in M.ITEMS:
            cols = [sh for sh in SPEC[n].shapes if sh.collide]
            self.assertEqual(len(cols), 1 if n in M.CHICKEN else 2, n)
            self.assertAlmostEqual(SPEC[n].mass, M.CH_KG if n in M.CHICKEN else M.ROLL_KG, places=4)
            st = state(n, (0.0, 0.0, M.TOP))
            self.assertAlmostEqual(M.lowest_z(SPEC[n], st), M.TOP, places=6)   # visual pieces do not count

    def test_counter_uses_lowest_point(self):
        st = state("roll1", (0.1, 0.0, M.TOP + 0.001))
        self.assertLess(M.lowest_z(SPEC["roll1"], st), M.TOP + M.COUNTER_Z)
        st = state("roll1", (*M.PLATE_SPOTS[0], PLATE_TOP))
        self.assertGreater(M.lowest_z(SPEC["roll1"], st), M.TOP + M.COUNTER_Z)
        st = state("chicken1", (*M.CH_XY[0], M.TOP + M.TRAY_T))
        self.assertGreater(M.lowest_z(SPEC["chicken1"], st), M.TOP + M.COUNTER_Z)

    def test_latch_zeroes_goal(self):
        feed([("right", "chicken1"), ("right", "roll1")])
        self.assertEqual(score_of(goal_states()), 0.0)
        self.assertFalse(M.success(goal_states()))

    def test_partial_credit_and_release(self):
        g = goal_states()
        s = start_states()
        s["roll1"] = g["roll1"]
        self.assertAlmostEqual(score_of(s), 0.2)
        near = {"left": mk.HAND.world((M.PLATE_SPOTS[0][0], M.PLATE_SPOTS[0][1], g["roll1"].high[2] + 0.010),
                                      np.eye(3), mk.FINGER_OPEN), "right": FAR["right"]}
        self.assertAlmostEqual(score_of(s, near), 0.0)        # a hand still over it: not counted
        off = start_states()
        off["roll1"] = state("roll1", (M.PLATE_C[0] + M.ON_PLATE_R + 0.005, M.PLATE_C[1], PLATE_TOP))
        self.assertAlmostEqual(score_of(off), 0.0)            # off the well
        lifted = start_states()
        lifted["chicken1"] = state("chicken1", (*M.PAN_SPOTS[0], PAN_TOP + M.IN_PAN_Z + 0.005))
        self.assertAlmostEqual(score_of(lifted), 0.0)         # held up over the pan


if __name__ == "__main__":
    unittest.main(verbosity=1)
