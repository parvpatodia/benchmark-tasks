"""Offline tests for cookies.py: loads; the start scores 0; the goal scores 1; each mode's planned end state scores
its EXPECT; the kit's offline gates pass on every mode (IK, full-link clearance, arm gap, run limit); the hands end
clear; the text agrees with the code. No simulator.

    PYTHONPATH=<yam_arena source> python3 test_cookies.py

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

TF = load(HERE / "cookies.py")
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


def clear_latches():
    for v in M.SPOILED.values():
        v.clear()
    for v in M._Z.values():
        v.clear()
    M.CARRIED.clear()
    M.UNREAD[0] = False


def start_states():
    clear_latches()
    return {n: state(n) for n in SPEC}


def score_of(states, hands=FAR):
    if hasattr(M, "set_hands"):
        M.set_hands(dict(hands))
    return M.score(states)


# -- EDIT: the task's own end states ------------------------------------------------------------------------------------
SLOTS = [(M.RACK_XY[0], M.RACK_XY[1] + d) for d in (0.090, 0.0, -0.090)]


def on_rack_state(i, xy=None, dz=0.0, yaw=0.0):
    x, y = SLOTS[i] if xy is None else xy
    return state(f"cookie_{i}", (x, y, M.RACK_TOP + dz), yaw)


def goal_states():
    """Every part where a full-score trial leaves it."""
    clear_latches()
    s = start_states()
    for i in range(3):
        s[f"cookie_{i}"] = on_rack_state(i)
    M.CARRIED.update(M.NAMES)          # each was seen carried on the blade on its way
    return s


def end_states(mode):
    """Where each mode's plan leaves the parts (the planned end, as the README's mode table says), with the latches
    the scorer would hold: touch presses the middle cookie, drop drops every cookie, shove leaves them on the sheet."""
    if mode == "demo":
        return goal_states()
    clear_latches()
    if mode in ("nothing", "shove"):
        return start_states()
    s = goal_states()
    for n in (["cookie_1"] if mode == "touch" else M.NAMES):
        M.SPOILED[n].add("touched" if mode == "touch" else "dropped")
    return s
# -- end of EDIT --------------------------------------------------------------------------------------------------------


class Task(unittest.TestCase):
    def test_loads(self):
        self.assertIn("demo", M.MODES)
        for mode in M.MODES:
            self.assertTrue(set(M.plan(0, mode)) <= {"left", "right"}, mode)

    def test_start_scores_zero(self):
        self.assertEqual(score_of(start_states()), 0.0)

    def test_goal_scores_one(self):
        self.assertEqual(score_of(goal_states()), 1.0)
        self.assertTrue(M.success(goal_states()))

    def test_hands_unread_not_full(self):
        """Unread hands are never clear, so no full score; each cookie's credit stays (it is judged on the cookie)."""
        self.assertEqual(score_of(goal_states(), {"left": None, "right": None}), 0.75)

    def test_moving_part_not_full(self):
        s = goal_states()
        name = next(n for n, o in SPEC.items() if getattr(o, "free", False))
        s[name] = state(name, s[name].pos, vel=(0.05, 0.0, 0.0))
        self.assertLess(score_of(s), 1.0)

    def test_each_mode_scores_its_expect(self):
        for mode, want in M.EXPECT.items():
            self.assertAlmostEqual(score_of(end_states(mode)), want, msg=mode)

    def test_offline_gates(self):
        for gate, ok, detail in checkkit.check(HERE / "cookies.py", M.MODES, (0,)):
            self.assertTrue(ok, f"{gate}: {detail}")

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


class Cookies(unittest.TestCase):
    """The scorer's own rules, each with the nearest failing case."""

    def test_partial_credit(self):
        s = goal_states()
        s["cookie_2"] = state("cookie_2")            # left on the sheet
        self.assertAlmostEqual(score_of(s), 0.5)
        s = goal_states()
        s["turner"] = state("turner", (0.30, -0.20, M.TOP + 0.10))   # still held over the sheet
        self.assertAlmostEqual(score_of(s), 0.75)

    def test_touch_latches(self):
        s = goal_states()
        c = s["cookie_1"]
        rim = np.asarray(c.pos) + [M.R + 0.002, 0.0, M.THICK / 2]
        M.note_step(s, {"left": None, "right": rim[None, :]})
        self.assertIn("touched", M.SPOILED["cookie_1"])
        self.assertAlmostEqual(score_of(s), 0.5)    # 2 good, no full score
        clear_latches()
        far = np.asarray(c.pos) + [M.R + M.TOUCH + 0.002, 0.0, M.THICK / 2]
        M.note_step(s, {"left": None, "right": far[None, :]})
        self.assertEqual(M.SPOILED["cookie_1"], set())

    def test_drop_latches(self):
        s = goal_states()
        s["cookie_0"] = state("cookie_0", s["cookie_0"].pos, vel=(0.0, 0.0, -M.DROP_V - 0.01))
        M.note_step(s, {})
        self.assertIn("dropped", M.SPOILED["cookie_0"])
        clear_latches()
        s["cookie_0"] = state("cookie_0", s["cookie_0"].pos, vel=(0.0, 0.0, -M.DROP_V + 0.05))
        M.note_step(s, {})
        self.assertEqual(M.SPOILED["cookie_0"], set())
        clear_latches()
        for n in M._Z.values():
            n.clear()
        p = np.asarray(s["cookie_0"].pos, float)
        for dz in (0.0, -0.009, -M.DROP_H - 0.001):                         # 19 mm in two steps, speed unsampled
            M.note_step({**s, "cookie_0": state("cookie_0", p + [0, 0, dz])}, {})
        self.assertIn("dropped", M.SPOILED["cookie_0"])
        clear_latches()
        for n in M._Z.values():
            n.clear()
        for dz in (0.0, -0.009, -0.017):                                    # the slowest set-down passes
            M.note_step({**s, "cookie_0": state("cookie_0", p + [0, 0, dz])}, {})
        self.assertEqual(M.SPOILED["cookie_0"], set())

    def test_stacked_overhanging_tilted_do_not_count(self):
        rack = state("rack")
        self.assertTrue(M.on_rack(on_rack_state(1), rack))
        self.assertFalse(M.on_rack(on_rack_state(1, dz=M.THICK), rack))           # on another cookie
        edge = M.RACK_XY[0] + M.RACK[0] / 2 - M.R + M.RACK_TOL + 0.002
        self.assertFalse(M.on_rack(on_rack_state(1, xy=(edge, M.RACK_XY[1])), rack))   # overhangs the frame
        self.assertTrue(M.on_rack(on_rack_state(1, xy=(edge - 0.004, M.RACK_XY[1])), rack))
        self.assertFalse(M.on_rack(on_rack_state(1, dz=-0.05), rack))           # on the table beside it
        tilted = on_rack_state(1)
        tilted = ObjectState(pos=tilted.pos, quat_xyzw=np.array([math.sin(math.radians(6)), 0, 0,
                                                                 math.cos(math.radians(6))]),
                             lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=tilted.low, high=tilted.high)
        self.assertFalse(M.on_rack(tilted, rack))                               # 12 deg, leaning

    def test_turner_and_sheet_rules(self):
        s = goal_states()
        s["turner"] = state("turner", (M.REST_XY[0] + 0.025, M.REST_XY[1], M.TOP + M.OFF))
        self.assertAlmostEqual(score_of(s), 0.75)
        s = goal_states()
        s["sheet"] = state("sheet", (M.SHEET_XY[0] + 0.035, M.SHEET_XY[1], M.TOP))
        self.assertAlmostEqual(score_of(s), 0.75)

    def test_hand_on_turner_not_full(self):
        s = goal_states()
        on_handle = mk.HAND.world((M.REST_XY[0], M.REST_XY[1], M.TOP + M.OFF), mk.down(0).rot, -0.011)
        self.assertAlmostEqual(score_of(s, {"left": FAR["left"], "right": on_handle}), 0.75)

    def test_must_be_carried_on_the_blade(self):
        s = goal_states()
        M.CARRIED.discard("cookie_2")             # got there without riding the blade (tipped, pushed)
        self.assertAlmostEqual(score_of(s), 0.5)
        clear_latches()
        s = start_states()                        # a cookie seen on the blade, 10 mm over the mat
        t = s["turner"] = state("turner", (M.COOKIE_X - 0.080, M.COOKIE_Y[1], M.MAT_TOP + 0.020 + M.OFF))
        inside, gap = M.over_blade(state("cookie_1", (M.COOKIE_X, M.COOKIE_Y[1], M.MAT_TOP + 0.0207)), t)
        self.assertTrue(inside)
        s["cookie_1"] = state("cookie_1", (M.COOKIE_X, M.COOKIE_Y[1], M.MAT_TOP + 0.020 + 0.00065))
        M.note_step(s, FAR)
        self.assertIn("cookie_1", M.CARRIED)
        clear_latches()
        s["cookie_1"] = state("cookie_1", (M.COOKIE_X, M.COOKIE_Y[1], M.MAT_TOP + 0.005))   # under it, on the mat
        M.note_step(s, FAR)
        self.assertNotIn("cookie_1", M.CARRIED)

    def test_step_hook_reads_the_turner(self):
        """The simulator's step hook must hand note_step the turner too, or no cookie can ever be carried."""
        import types
        import yam_arena
        asked = []
        fake = types.ModuleType("yam_arena.scene")      # WHY: the real module needs Isaac Lab, absent offline
        fake.object_states = lambda env, names: asked.append(list(names)) or start_states()
        old = sys.modules.get("yam_arena.scene"), getattr(yam_arena, "scene", None)
        sys.modules["yam_arena.scene"], yam_arena.scene = fake, fake
        try:
            M.read_hands(object())
        finally:
            if old[0] is None:
                sys.modules.pop("yam_arena.scene", None)
            else:
                sys.modules["yam_arena.scene"] = old[0]
            if old[1] is None:
                del yam_arena.scene
            else:
                yam_arena.scene = old[1]
        self.assertEqual(len(asked), 1)
        self.assertIn("turner", asked[0])
        self.assertTrue(set(M.NAMES) <= set(asked[0]))

    def test_left_on_the_blade_does_not_count(self):
        s = goal_states()
        x, y = SLOTS[1]
        s["turner"] = state("turner", (x - 0.080, y, M.RACK_TOP + M.OFF))     # blade lying on the wires
        s["cookie_1"] = state("cookie_1", (x, y, M.RACK_TOP + 0.00065))       # the cookie still on the blade
        self.assertTrue(M.on_rack(s["cookie_1"], s["rack"]))
        self.assertAlmostEqual(score_of(s), 0.5)

    def test_rack_moved_and_unread_hands_not_full(self):
        s = goal_states()
        s["rack"] = state("rack", (M.RACK_XY[0], M.RACK_XY[1] - 0.035, M.TOP))
        moved = {**s, **{f"cookie_{i}": state(f"cookie_{i}", np.asarray(s[f"cookie_{i}"].pos) + [0, -0.035, 0])
                         for i in range(3)}}
        self.assertAlmostEqual(score_of(moved), 0.75)
        s = goal_states()
        M.note_step(s, {"left": None, "right": FAR["right"]})
        self.assertAlmostEqual(score_of(s), 0.75)

    def test_cookie_numbers(self):
        self.assertAlmostEqual(M.COOKIE_KG, 0.0329, places=3)
        readme = (HERE / "README.md").read_text()
        for txt in ("%.0f g" % (M.COOKIE_KG * 1000), "%d mm across" % round(2000 * M.R), "%d mm thick" % round(1000 * M.THICK)):
            self.assertIn(txt, readme)


if __name__ == "__main__":
    unittest.main(verbosity=1)
