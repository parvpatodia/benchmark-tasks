"""Offline tests for contour_gauge.py: loads; the start scores 0; the goal scores 1; each mode's planned end state scores
its EXPECT; the kit's offline gates pass on every mode (IK, full-link clearance, arm gap, run limit); the hands end
clear; the text agrees with the code. No simulator.

    PYTHONPATH=<yam_arena source> python3 test_contour_gauge.py

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

TF = load(HERE / "contour_gauge.py")
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
from mechanism import MechanismState  # noqa: E402

Z_SHEET = M.TOP + M.SHEET_T


def copy_pushes(D, dy=0.0):
    """Each finger's push after a square copy with the frame front D from the wall and the gauge dy off centre."""
    out = []
    for y in M.YF:
        t = M.outline_t(y + dy - M.FING_W / 2, y + dy + M.FING_W / 2)
        out.append(0.0 if t is None else min(M.TRAVEL, max(0.0, M.PROTRUDE - D + t)))
    return np.array(out)


def gauge(x, y, z, pushes, yaw=0.0, vel=(0.0, 0.0, 0.0), quat=None):
    p = np.array([x, y, z], float)
    q = qz(yaw) if quat is None else np.asarray(quat, float)
    joints = {f"j{i:02d}": -float(v) for i, v in enumerate(pushes)}
    return MechanismState(pos=p, quat_xyzw=q, lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3), low=p, high=p,
                          joints=joints, links={})


def start_states():
    return {"gauge": gauge(*M.GAUGE_START, M.TOP, np.zeros(M.N_FING)), "sheet": state("sheet")}


def goal_states(D=M.D_COPY, dy=0.0):
    s = start_states()
    s["gauge"] = gauge(M.X_WALL - D - M.FX0 - M.SLIDE, dy, Z_SHEET, copy_pushes(D, dy))
    return s


def end_states(mode):
    if mode == "demo":
        return goal_states()
    s = start_states()
    if mode == "no_press":
        s["gauge"] = gauge(M.X_MARK, 0.0, Z_SHEET, np.zeros(M.N_FING))
    elif mode == "off_line":
        s["gauge"] = gauge(M.X_MARK, 0.012, Z_SHEET, copy_pushes(M.D_COPY))
    elif mode == "one_arm":
        pass   # what it reaches is measured in the simulator (README); offline the plan's end is not modelled
    return s
# # -- end of EDIT --------------------------------------------------------------------------------------------------------


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

    def test_hands_unread_score_zero(self):
        self.assertLess(score_of(goal_states(), {"left": None, "right": None}), 1.0)

    def test_moving_part_not_full(self):
        s = goal_states()
        g = s["gauge"]
        s["gauge"] = gauge(*g.pos, copy_pushes(M.D_COPY), vel=(0.05, 0.0, 0.0))
        self.assertLess(score_of(s), 1.0)

    def test_copy_window(self):
        """Any frame depth from 27 to 37 mm makes a full-score copy when laid down from that same depth."""
        for D in (0.027, 0.030, 0.032, 0.035, 0.037):
            self.assertEqual(score_of(goal_states(D)), 1.0, D)

    def test_copy_fakes_score_zero(self):
        """Fingers straight, all pushed alike, or the casing copied as a flat block or a plain bevel: no copy."""
        flat_block = np.array([M.PROTRUDE - 0.032 + (0.019 if p > 0.009 else 0.0) for p in copy_pushes(0.032)])
        bevel = np.clip(np.linspace(0.0, 0.03, M.N_FING), 0, M.TRAVEL)
        for s in (np.zeros(M.N_FING), np.full(M.N_FING, 0.01), flat_block, bevel):
            self.assertFalse(M.copy_fit(s)[1] <= M.FIT_MAX)
            g = goal_states()
            g["gauge"] = gauge(*g["gauge"].pos, s)
            self.assertEqual(score_of(g), 0.0)

    def test_copy_taken_off_centre_counts(self):
        """A copy taken 2 mm to one side and laid 2 mm to the same side is the same cut."""
        self.assertEqual(score_of(goal_states(dy=0.002)), 1.0)

    def test_copy_then_depth_mismatch(self):
        """A copy taken at 32 mm but laid as if from 28 mm puts every tip 4 mm off: 0.75."""
        s = goal_states(0.028)
        s["gauge"] = gauge(*s["gauge"].pos, copy_pushes(0.032))
        self.assertEqual(score_of(s), 0.75)

    def test_tiers(self):
        s = start_states()
        s["gauge"] = gauge(*M.GAUGE_START, M.TOP, copy_pushes(M.D_COPY))
        self.assertEqual(score_of(s), 0.25)                       # copied, still on the floor
        g = goal_states()["gauge"]
        tilt = (math.sin(math.radians(1.25)), 0.0, 0.0, math.cos(math.radians(1.25)))   # 2.5 deg
        s = goal_states()
        s["gauge"] = gauge(*g.pos, copy_pushes(M.D_COPY), quat=(math.sin(math.radians(4)), 0, 0,
                                                                 math.cos(math.radians(4))))
        self.assertEqual(score_of(s), 0.25)                       # tilted 8 deg: not flat
        s["gauge"] = gauge(*g.pos, copy_pushes(M.D_COPY), quat=tilt)
        self.assertGreaterEqual(score_of(s), 0.75)                # 2.5 deg is still flat
        s = goal_states()
        s["gauge"] = gauge(g.pos[0], g.pos[1], g.pos[2] + 0.004, copy_pushes(M.D_COPY))
        self.assertEqual(score_of(s), 0.25)                       # 4 mm over the sheet: not resting on it
        s = goal_states()
        s["gauge"] = gauge(*g.pos, copy_pushes(M.D_COPY), yaw=math.pi)
        self.assertEqual(score_of(s), 0.5)                        # turned end for end: tips off the cut
        s = goal_states()
        s["gauge"] = gauge(g.pos[0] - 0.005, g.pos[1], g.pos[2], copy_pushes(M.D_COPY))
        self.assertEqual(score_of(s), 0.75)                       # 5 mm short of the line
        s["gauge"] = gauge(g.pos[0] - 0.008, g.pos[1], g.pos[2], copy_pushes(M.D_COPY))
        self.assertEqual(score_of(s), 0.5)                        # 8 mm short

    def test_hand_on_gauge_caps(self):
        s = goal_states()
        g = s["gauge"]
        near = dict(FAR)
        near["left"] = mk.HAND.world((g.pos[0], g.pos[1] + M.YB, g.pos[2] + M.TCP_H), np.eye(3), mk.FINGER_OPEN)
        self.assertEqual(score_of(s, near), 0.75)

    def test_sheet_moved_with_gauge_still_scores(self):
        """The cut is judged in the sheet's own frame: sheet and gauge moved together 20 mm still score 1."""
        s = goal_states()
        sh = SPEC["sheet"]
        from mechanism import body_state
        s["sheet"] = body_state(sh, np.asarray(sh.pos) + (0.0, 0.02, 0.0), qz(0.0))
        g = s["gauge"]
        s["gauge"] = gauge(g.pos[0], g.pos[1] + 0.02, g.pos[2], copy_pushes(M.D_COPY))
        self.assertEqual(score_of(s), 1.0)

    def test_pads_clear_fingers(self):
        """At each grip the open and closed pads keep 3 mm from every finger, set anywhere in its travel."""
        g = np.array([*M.GAUGE_START, M.TOP])
        for arm, W in M.plan(0, "demo").items():
            q = np.array(mk.READY, float)
            for w in W:
                q, _pe, _re = ik.solve(w.pos, ik.matrix_to_quat_xyzw(mk.harness_demo.grip_rotation(w.yaw)), q,
                                       base_pos=station.BASES[arm], iters=200)
                if w.label.startswith("pick") and ("close" in w.label or "in" in w.label.split(": ")[-1]):
                    d = mk.box_distance(mk.HAND.from_q(arm, q, w.finger) - g, *M.FINGER_ENV).min()
                    self.assertGreaterEqual(d, 0.003, f"{arm} {w.label} {1000 * d:.1f} mm")

    def test_small_sideways_misregister_is_graded(self):
        """A lay 2 mm sideways of the copy is a 2 mm error (1.0); 5 mm is 0.75, not a jump to the step height."""
        for dy, want in ((0.002, 1.0), (-0.002, 1.0), (0.005, 0.75)):
            s = goal_states()
            g = s["gauge"]
            s["gauge"] = gauge(g.pos[0], g.pos[1] + dy, g.pos[2], copy_pushes(M.D_COPY))
            self.assertEqual(score_of(s), want, dy)

    def test_face_down_gauge(self):
        """Copied and laid face down marks the same cut: 1.0. Copied face up and laid face down (mirrored): 0.5."""
        s = goal_states()
        g = s["gauge"]
        flip = (1.0, 0.0, 0.0, 0.0)   # half turn about x
        s["gauge"] = gauge(g.pos[0], g.pos[1], g.pos[2] + M.FRAME_H, copy_pushes(M.D_COPY)[::-1], quat=flip)
        self.assertEqual(score_of(s), 1.0)
        s["gauge"] = gauge(g.pos[0], g.pos[1], g.pos[2] + M.FRAME_H, copy_pushes(M.D_COPY), quat=flip)
        self.assertEqual(score_of(s), 0.5)

    def test_slip_model(self):
        """A finger holds until F_HOLD and then slides with its target F_HOLD / k behind it."""
        t = M.slip([-0.001, -0.010], [0.0, 0.0])
        lag = M.F_HOLD / M.K_FING
        self.assertAlmostEqual(float(t[0]), -0.001 + lag, places=9)
        self.assertAlmostEqual(float(t[1]), -0.010 + lag, places=9)
        self.assertTrue(np.all(np.abs(M.K_FING * (t - np.array([-0.001, -0.010]))) <= M.F_HOLD + 1e-9))

    def test_profile_resolved(self):
        """Every feature of the casing is at least one finger wide, so the copy can show it."""
        for u0, u1, _t in M.PROFILE:
            self.assertGreaterEqual(u1 - u0, M.PITCH)

    def test_each_mode_scores_its_expect(self):
        for mode, want in M.EXPECT.items():
            self.assertAlmostEqual(score_of(end_states(mode)), want, msg=mode)

    def test_offline_gates(self):
        for gate, ok, detail in checkkit.check(HERE / "contour_gauge.py", M.MODES, (0,)):
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


if __name__ == "__main__":
    unittest.main(verbosity=1)
