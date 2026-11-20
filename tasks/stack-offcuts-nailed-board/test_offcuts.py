"""Offline tests for offcuts.py: loads; the start scores 0; the goal scores 1; each mode's planned end state scores
its EXPECT; the kit's offline gates pass on every mode (IK, full-link clearance, arm gap, run limit); the hands end
clear; the text agrees with the code. No simulator.

    PYTHONPATH=<yam_arena source> python3 test_offcuts.py

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

TF = load(HERE / "offcuts.py")
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
def on_deck(k, y=None):
    return state(k, (M.SLOT_X, M.SLOT_Y[k] if y is None else y, M.DECK_TOP + M.STOCK / 2), 0.0)


def in_tote():
    return state("n", (M.TOTE_XY[0], M.TOTE_XY[1], M.TOP + M.TOTE_FLOOR + M.STOCK / 2), math.pi / 2)


def goal_states():
    """Every part where a full-score trial leaves it."""
    s = start_states()
    for k in M.CLEAN:
        s[k] = on_deck(k)
    s["n"] = in_tote()
    return s


def end_states(mode):
    """Where each mode's plan leaves the parts (the planned end, as the README's mode table says)."""
    s = start_states()
    if mode == "demo":
        return goal_states()
    if mode == "stack_all":
        for k in (*M.CLEAN, "n"):
            s[k] = on_deck(k)
    elif mode == "no_tote":
        for k in M.CLEAN:
            s[k] = on_deck(k)
    elif mode == "one_arm":
        s["c"], s["n"] = on_deck("c"), in_tote()
    return s
# -- end of EDIT --------------------------------------------------------------------------------------------------------


def qrpy(roll=0.0, pitch=0.0, yaw=0.0):
    from scipy.spatial.transform import Rotation
    return Rotation.from_euler("xyz", [roll, pitch, yaw]).as_quat()


def posed(name, pos, quat, vel=(0.0, 0.0, 0.0)):
    """A board at any pose (tilted too), its world box from its corners."""
    s = SPEC[name]
    pos, quat = np.asarray(pos, float), np.asarray(quat, float)
    if hasattr(s, "corners"):
        from mechanism import body_state
        return body_state(s, pos, quat, vel)
    tmp = ObjectState(pos=pos, quat_xyzw=quat, lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3), low=pos, high=pos)
    c = M.board_corners(name, tmp)
    return ObjectState(pos=pos, quat_xyzw=quat, lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3),
                       low=c.min(axis=0), high=c.max(axis=0))


def hand_at(p):
    return mk.HAND.world(p, mk.down(0.0).rot, mk.FINGER_OPEN)


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
        # WHY not 0: partial credit is for where the boards lie; only the full score needs the hands read and clear
        self.assertEqual(score_of(goal_states(), {"left": None, "right": None}), 0.8)

    def test_moving_part_not_full(self):
        s = goal_states()
        name = next(n for n, o in SPEC.items() if getattr(o, "free", False))
        s[name] = state(name, s[name].pos, vel=(0.05, 0.0, 0.0))
        self.assertLess(score_of(s), 1.0)

    def test_each_mode_scores_its_expect(self):
        for mode, want in M.EXPECT.items():
            self.assertAlmostEqual(score_of(end_states(mode)), want, msg=mode)

    def test_offline_gates(self):
        for gate, ok, detail in checkkit.check(HERE / "offcuts.py", M.MODES, (0,)):
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
        self.assertLessEqual(len(readme.splitlines()), 200)
        for text in (readme, json.dumps(tj, ensure_ascii=False)):
            self.assertNotRegex(text, "[\u2013\u2014]")   # en and em dash, as escapes so this file holds neither


class Rules(unittest.TestCase):
    """The scorer's rules, one outcome at a time (README "Scoring")."""

    def test_nailed_on_deck_zero(self):
        s = goal_states()
        s["n"] = state("n", (M.SLOT_X, M.SLOT_Y["n"], M.DECK_TOP + M.STOCK / 2), 0.0)
        self.assertEqual(score_of(s), 0.0)

    def test_nailed_on_top_of_stack_zero(self):
        s = goal_states()
        s["n"] = state("n", (M.SLOT_X, M.SLOT_Y["c"], M.DECK_TOP + 1.5 * M.STOCK), 0.0)
        self.assertEqual(score_of(s), 0.0)

    def test_nailed_leaning_on_deck_edge_zero(self):
        """One end on the bench in front of the deck, the other up on the deck's near slat."""
        s = goal_states()
        ang = math.asin((M.DECK_TOP - M.TOP) / 0.20)
        x = M.DECK_X[0] + 0.02 - 0.5 * M.BOARDS["n"][0] * math.cos(ang)
        s["n"] = posed("n", (x, -0.10, M.TOP + 0.5 * (M.DECK_TOP - M.TOP) + M.STOCK / 2), qrpy(pitch=-ang))
        self.assertTrue(M.nailed_on_pallet(s))
        self.assertEqual(score_of(s), 0.0)

    def test_nailed_held_over_pallet_zero(self):
        s = goal_states()
        s["n"] = state("n", (M.SLOT_X, 0.0, M.DECK_TOP + 0.10), 0.0)
        self.assertEqual(score_of(s), 0.0)

    def test_nailed_on_bench_beside_deck_not_on_pallet(self):
        s = goal_states()
        s["n"] = state("n", (M.SLOT_X, M.DECK_Y[0] - M.STOCK / 2 - 0.001, M.TOP + M.STOCK / 2), 0.0)
        self.assertFalse(M.nailed_on_pallet(s))
        self.assertAlmostEqual(score_of(s), 0.45)

    def test_tilted_board_not_stacked(self):
        s = goal_states()
        s["a"] = posed("a", s["a"].pos + np.array([0, 0, 0.02]), qrpy(roll=math.radians(15)))
        self.assertNotIn("a", M.stacked(s))

    def test_hovering_board_not_stacked(self):
        s = goal_states()
        s["a"] = state("a", s["a"].pos + np.array([0, 0, 0.02]), 0.0)
        self.assertNotIn("a", M.stacked(s))

    def test_board_on_board_stacked(self):
        s = goal_states()
        s["b"] = state("b", (M.SLOT_X, M.SLOT_Y["a"], M.DECK_TOP + 1.5 * M.STOCK + 0.002), math.pi / 2 * 0.1)
        self.assertEqual(M.stacked(s), {"a", "b", "c"})
        self.assertEqual(score_of(s), 1.0)

    def test_board_off_deck_not_stacked(self):
        s = goal_states()
        s["c"] = state("c", (M.DECK_X[1] + 0.005, M.SLOT_Y["c"], M.DECK_TOP + M.STOCK / 2), 0.0)
        self.assertNotIn("c", M.stacked(s))

    def test_nailed_standing_in_tote_no_credit(self):
        s = goal_states()
        L = M.BOARDS["n"][0]
        s["n"] = posed("n", (M.TOTE_XY[0], 0.0, M.TOP + M.TOTE_FLOOR + L / 2), qrpy(pitch=math.pi / 2))
        self.assertFalse(M.nailed_in_tote(s))
        self.assertAlmostEqual(score_of(s), 0.45)

    def test_nailed_against_tote_wall_counts(self):
        s = goal_states()
        x = M.TOTE_XY[0] - M.TOTE_IN[0] / 2 + M.STOCK / 2 - 0.001   # pressed 1 mm into the near wall
        s["n"] = state("n", (x, 0.0, M.TOP + M.TOTE_FLOOR + M.STOCK / 2), math.pi / 2)
        self.assertTrue(M.nailed_in_tote(s))

    def test_nailed_across_rim_no_credit(self):
        s = goal_states()
        s["n"] = posed("n", (M.TOTE_XY[0], 0.0, M.TOP + M.TOTE_H + M.STOCK / 2), qrpy(yaw=0.0))
        self.assertFalse(M.nailed_in_tote(s))

    def test_tote_tipped_no_credit(self):
        s = goal_states()
        from mechanism import body_state
        s["tote"] = body_state(SPEC["tote"], s["tote"].pos, qrpy(roll=math.radians(30)))
        self.assertFalse(M.nailed_in_tote(s))

    def test_moving_board_in_tote_no_credit(self):
        s = goal_states()
        s["n"] = posed("n", s["n"].pos, s["n"].quat_xyzw, vel=(0.05, 0, 0))
        self.assertFalse(M.nailed_in_tote(s))

    def test_hand_near_board_not_full(self):
        s = goal_states()
        near = hand_at((M.SLOT_X, M.SLOT_Y["a"], M.DECK_TOP + M.STOCK + 0.013 + 0.020))
        self.assertEqual(score_of(s, {"left": near, "right": FAR["right"]}), 0.8)
        far = hand_at((M.SLOT_X, M.SLOT_Y["a"], M.DECK_TOP + M.STOCK + 0.013 + 0.040))
        self.assertEqual(score_of(s, {"left": far, "right": FAR["right"]}), 1.0)

    def test_all_in_tote_partial_only(self):
        """Dumping everything in the tote earns the tote credit only (the clean boards are not stacked)."""
        s = goal_states()
        for k in M.CLEAN:
            s[k] = state(k, (M.TOTE_XY[0], 0.0, M.TOP + M.TOTE_FLOOR + 1.5 * M.STOCK), math.pi / 2)
        self.assertAlmostEqual(score_of(s), 0.35)


    def test_hand_on_nail_not_full(self):
        s = goal_states()
        head = s["n"].to_world((M.NAIL_X, 0.0, M.STOCK / 2 + M.NAIL_PROUD))
        hand = hand_at(head + np.array([0.0, 0.0, 0.010]))
        self.assertLess(score_of(s, {"left": FAR["left"], "right": hand}), 1.0)

    def test_hand_on_tote_wall_not_full(self):
        s = goal_states()
        wall = np.array([M.TOTE_XY[0] - M.TOTE_IN[0] / 2, 0.10, M.TOP + M.TOTE_H + 0.013 + 0.010])
        self.assertLess(score_of(s, {"left": hand_at(wall), "right": FAR["right"]}), 1.0)

    def test_tote_lifted_no_credit(self):
        s = goal_states()
        from mechanism import body_state
        s["tote"] = body_state(SPEC["tote"], s["tote"].pos + np.array([0, 0, 0.05]), s["tote"].quat_xyzw)
        s["n"] = posed("n", s["n"].pos + np.array([0, 0, 0.05]), s["n"].quat_xyzw)
        self.assertFalse(M.nailed_in_tote(s))

    def test_pressed_board_not_stacked(self):
        s = goal_states()
        s["a"] = state("a", s["a"].pos, 0.0, vel=(0.02, 0.0, 0.0))
        self.assertNotIn("a", M.stacked(s))

class Scene(unittest.TestCase):
    def test_nail_position_and_height(self):
        self.assertAlmostEqual(M.BOARDS["n"][0] / 2 - M.NAIL_X, 0.050)
        n = SPEC["n"]
        lo, hi = zip(*(sh.bounds() for sh in n.shapes))
        self.assertAlmostEqual(float(np.max(np.array(hi)[:, 2])) - M.STOCK / 2, M.NAIL_PROUD, places=6)
        self.assertLess(M.NAIL_IN, M.STOCK)   # the point stays inside the wood

    def test_nail_inside_tote_rim(self):
        top = M.TOTE_FLOOR + M.STOCK + M.NAIL_PROUD
        self.assertLess(top, M.TOTE_H)

    def test_letters_on_sign(self):
        L = M.TOTE_IN[1] + 2 * M.TOTE_WALL
        for p0, p1 in M.text_layout():
            for a, b in M.stroke_outline(p0, p1):
                self.assertTrue(0.0 <= a <= L and 0.0 <= b <= M.SIGN_LEN, (a, b))

    def test_sign_faces_top_camera(self):
        """The sign's normal against the line to the top camera (both variants): the letters' projected height in the
        320 x 176 policy frame, as the README states it."""
        o, u, v, n = M.sign_frame()
        R, p = np.eye(3), np.array([*M.TOTE_XY, M.TOP])
        centre = p + o + (L := M.TOTE_IN[1] / 2 + M.TOTE_WALL) * u + M.SIGN_LEN / 2 * v
        for cam in station.TOP_CAMS:
            c = station.top_camera(cam)
            d = np.asarray(c.pos) - centre
            dist = float(np.linalg.norm(d))
            cos = float(n @ d) / dist
            px = c.lens.fy * 176 / 360 * M.LETTER_H * cos / dist
            self.assertGreater(cos, 0.85, cam)
            self.assertGreater(px, 11.0, cam)

    def test_sign_in_frame(self):
        """Every corner of the sign projects at least 15 px inside both top cameras' 640 x 360 frames (README rows)."""
        from scipy.spatial.transform import Rotation
        o, u, v, n = M.sign_frame()
        L = M.TOTE_IN[1] + 2 * M.TOTE_WALL
        p = np.array([*M.TOTE_XY, M.TOP])
        for cam in station.TOP_CAMS:
            c = station.top_camera(cam)
            R, P, k = Rotation.from_quat(c.rot_xyzw).as_matrix(), np.asarray(c.pos), c.lens
            for a in (0.0, L):
                for b in (0.0, M.SIGN_LEN):
                    q = R.T @ (p + o + a * u + b * v - P)
                    row, col = k.cy - k.fy * q[1] / -q[2], k.cx + k.fx * q[0] / -q[2]
                    self.assertTrue(15 <= row <= k.height - 15 and 15 <= col <= k.width - 15, (cam, row, col))

    def test_slats_fit_deck(self):
        x = [lo[0] for n, lo, hi, c in M.pallet_boxes() if n.startswith("slat")]
        hi = [hi[0] for n, lo, hi, c in M.pallet_boxes() if n.startswith("slat")]
        for i in range(len(x) - 1):
            self.assertGreater(x[i + 1] - hi[i], 0.010)

    def test_numbers_in_text(self):
        readme = (HERE / "README.md").read_text()
        tj = (HERE / "task.json").read_text()
        o, u, v, n = M.sign_frame()
        c = np.array([*M.TOTE_XY, M.TOP]) + o + (M.TOTE_IN[1] / 2 + M.TOTE_WALL) * u + M.SIGN_LEN / 2 * v
        px = {}
        for cam in station.TOP_CAMS:
            cc = station.top_camera(cam)
            d = np.asarray(cc.pos) - c
            px[cam] = (round(cc.lens.fy * 176 / 360 * M.LETTER_H * float(n @ d) / float(np.linalg.norm(d)) ** 2),
                       math.degrees(math.acos(float(n @ d) / float(np.linalg.norm(d)))))
        self.assertIn(f"{px['calibrated'][0]} px tall", readme)
        self.assertIn(f"({px['demo'][0]} px from the lower demo", readme)
        self.assertLess(max(a for _, a in px.values()), 29.0)
        for t in (readme, tj):
            self.assertIn(f"{round(1000 * (M.DECK_TOP - M.TOP))}", t)
            self.assertIn(f"{round(1000 * M.NAIL_PROUD)} mm", t)
            self.assertIn(f"{round(1000 * M.LETTER_H)} mm", t)
        self.assertIn(f"{M.TOTE.mass:.2f} kg", tj)
        self.assertIn(f"deck outline x {round(100 * M.DECK_X[0])} to {round(100 * M.DECK_X[1])}", tj)
        self.assertIn(f"deck outline x {round(1000 * M.DECK_X[0])} to {round(1000 * M.DECK_X[1])}", readme)
        self.assertIn(f"centred at ({round(100 * M.TOTE_XY[0])}, 0)", tj)
        self.assertIn(f"centred at ({round(1000 * M.TOTE_XY[0])}, 0)", readme)
        for k, (L, x, y, yw) in M.BOARDS.items():
            self.assertIn(f"{k} {round(100 * L)} cm at ({100 * x:g}, {100 * y:+g}), {yw:+g} deg", tj)
            self.assertIn(f"({round(1000 * x)}, {round(1000 * y):+d}) yaw {yw:+g} deg", readme)

    def test_boards_apart_at_start(self):
        st = start_states()
        names = list(M.BOARDS)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                ca, cb = M.board_corners(a, st[a]), M.board_corners(b, st[b])
                sep = (ca.min(0) > cb.max(0)) | (cb.min(0) > ca.max(0))
                self.assertTrue(sep[:2].any(), (a, b))

if __name__ == "__main__":
    unittest.main(verbosity=1)
