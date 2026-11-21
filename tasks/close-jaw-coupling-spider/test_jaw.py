"""Offline tests for jaw.py: loads; the start scores 0; the goal scores 1; each mode's planned end state scores
its EXPECT; the kit's offline gates pass on every mode (IK, full-link clearance, arm gap, run limit); the hands end
clear; the text agrees with the code. No simulator.

    PYTHONPATH=<yam_arena source> python3 test_jaw.py

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

TF = load(HERE / "jaw.py")
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
    """The trial score if this state is held to the end: --scorer task_score gives 1.0 when success() held 1 s,
    else score() at the end."""
    if hasattr(M, "set_hands"):
        M.set_hands(dict(hands))
    return 1.0 if M.success(states) else M.score(states)


# -- EDIT: the task's own end states ------------------------------------------------------------------------------------
from mechanism import MechanismState  # noqa: E402


def mech_state(spec, joints=None, spin=0.0):
    """A mechanism at rest with the given joint positions; `spin` (rad/s) sets every moving link turning."""
    joints = {j.name: 0.0 for j in spec.joints} | dict(joints or {})
    pos = np.asarray(spec.pos, float)
    links = {ln.name: ObjectState(pos=pos, quat_xyzw=qz(spec.yaw), lin_vel=np.zeros(3),
                                  ang_vel=np.array([spin, 0.0, 0.0]), low=pos, high=pos) for ln in spec.links}
    half = np.asarray(spec.size, float) / 2
    return MechanismState(pos=pos, quat_xyzw=qz(spec.yaw), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                          low=pos - half, high=pos + half, joints=joints, links=links)


def start_states():
    s = {n: state(n) for n in SPEC if n not in ("drive", "driven")}
    s["drive"], s["driven"] = mech_state(M.DRIVE), mech_state(M.DRIVEN)
    return s


SEAT = (M.SEAT_X, 0.0, M.AXIS_Z)    # spider centre when seated, world [ARITH in jaw.py]


def with_hub(slide, spider=SEAT, turn=M.TURN0):
    s = start_states()
    if spider is not None:
        s["spider"] = state("spider", spider, 0.0)
    s["driven"] = mech_state(M.DRIVEN, {"hub_slide": slide, "shaft_turn": turn})
    return s


def goal_states():
    """Spider seated against the fixed hub, loose hub slid home (faces G apart), shaft turned to mesh."""
    return with_hub(M.SLIDE)


def end_states(mode):
    """Where each mode's plan leaves the parts (the README's mode table)."""
    if mode == "demo":
        return goal_states()
    if mode in ("no_turn", "part_turn"):        # jaw tips stop on the leg ends: hub face at the spider's near face
        return with_hub(M.SLIDE - M.SP_W, turn=0.0)
    if mode == "weak_push":                     # turned, but the tips jam where they meet the spider (RUN: 35.6 mm)
        return with_hub(M.SLIDE - M.SP_W)
    if mode == "no_spider":                     # hub home, spider still in its holder
        return with_hub(M.SLIDE, spider=None)
    return start_states()                       # nothing; knock (the spider never leaves its holder)
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

    def test_full_needs_the_hold(self):
        """score() alone tops out at 0.75: a final state reached on the last step is not the held 1.0."""
        M.set_hands(dict(FAR)) if hasattr(M, "set_hands") else None
        self.assertEqual(M.score(goal_states()), 0.75)
        self.assertEqual(M.SUCCESS_HOLD_S, 1.0)

    def test_hands_unread_score_zero(self):
        self.assertEqual(score_of(goal_states(), {"left": None, "right": None}), 0.0)

    def test_moving_part_not_full(self):
        s = goal_states()
        s["spider"] = state("spider", SEAT, 0.0, vel=(0.05, 0.0, 0.0))
        self.assertLess(score_of(s), 1.0)
        s = goal_states()
        s["driven"] = mech_state(M.DRIVEN, {"hub_slide": M.SLIDE}, spin=2.0)
        self.assertLess(score_of(s), 1.0)

    def test_stage_edges(self):
        """Each stage's threshold, on both sides."""
        self.assertEqual(score_of(with_hub(M.SLIDE, spider=(M.SEAT_X, 0.0, M.AXIS_Z - 0.003))), 0.0)  # dropped 3 mm
        self.assertEqual(score_of(with_hub(M.SLIDE, spider=(M.SEAT_X - M.DIR * 0.003, 0.0, M.AXIS_Z))), 0.0)  # off face
        shy = M.SLIDE - M.SP_W + M.MESH_IN
        self.assertEqual(score_of(with_hub(shy - 0.0005)), 0.25)
        self.assertEqual(score_of(with_hub(shy + 0.0005)), 0.5)
        self.assertEqual(score_of(with_hub(M.SLIDE - M.LINE_TOL - 0.0003)), 0.5)
        self.assertEqual(score_of(with_hub(M.SLIDE - M.LINE_TOL + 0.0003)), 1.0)
        near = {"left": mk.HAND.world((M.SEAT_X, 0.0, M.AXIS_Z + M.R_OD + 0.020), np.eye(3), 0.0), "right": FAR["right"]}
        self.assertEqual(score_of(goal_states(), near), 0.0)    # a hand on the coupling: spider not clear

    def test_fingers_clear_fixed_jaws(self):
        """CHECK_SKIP hides the plan's squeezed fingers; here the fingers at the leg ends (not squeezed past them)
        keep off both hubs while the spider comes down and seats."""
        R = checkkit._rz(M.DRIVE.yaw)
        parts = []
        for mech in (M.DRIVE, M.DRIVEN):
            R = checkkit._rz(mech.yaw)
            for link in mech.all_links():
                for sh in link.shapes:
                    if sh.collide:
                        parts.append(checkkit.Part(mech.name, [checkkit.shape_points(sh) @ R.T + np.asarray(mech.pos)]))
        for x in (M.SLOT_X, M.SEAT_X + M.DIR * M.SEAT_PUSH):
            pts = mk.HAND.world((x, 0.0, M.AXIS_Z), M.down(0.0).rot, -M.SP_LEG)
            least = min(float(p.signed(pts).min()) for p in parts)
            self.assertGreater(least, 0.0, f"fingers {1000 * least:.2f} mm into a hub at x {x:.4f}")

    def test_each_mode_scores_its_expect(self):
        for mode, want in M.EXPECT.items():
            self.assertAlmostEqual(score_of(end_states(mode)), want, msg=mode)

    def test_offline_gates(self):
        for gate, ok, detail in checkkit.check(HERE / "jaw.py", M.MODES, (0,)):
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
