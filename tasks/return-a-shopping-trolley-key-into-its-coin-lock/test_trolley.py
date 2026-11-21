"""Offline tests for trolley.py: loads; the start scores 0; the goal scores 0.75 from score() and is a success; every
scoring rule against the code, each mode's planned end state against EXPECT, the closed shortcuts (a finger cannot
reach the plunger, the coin slide cannot come out without the key, the key cannot reach its own lock), the latch
geometry, the kit's offline gates, and the text against the code. No simulator.

    PYTHONPATH=<yam_arena source> python3 test_trolley.py
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
from mechanism import MechanismState, body_state  # noqa: E402
from yam_arena.task_file import ObjectState, load  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

TF = load(HERE / "trolley.py")
M = TF.module
FAR = {"left": mk.HAND.world((0.0, 0.40, M.TOP + 0.45), np.eye(3), mk.FINGER_OPEN),
       "right": mk.HAND.world((0.0, -0.40, M.TOP + 0.45), np.eye(3), mk.FINGER_OPEN)}
Z3 = np.zeros(3)


def quat(r):
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return np.array([x, y, z, w])


def ry(deg):
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(deg))


def link(pos, rot=np.eye(3), vel=Z3):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=quat(rot), lin_vel=np.asarray(vel, float), ang_vel=Z3, low=p, high=p)


def front(key_rot=np.eye(3), key_pos=None, vel=Z3):
    p = np.asarray(M.FRONT.pos if key_pos is None else key_pos, float)
    o = np.asarray(M.FRONT.pos, float)
    return MechanismState(pos=o, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=Z3, ang_vel=Z3, low=o, high=o,
                          joints={}, links={"ckey": link(p, key_rot, vel)})


def key_at(depth, dy=0.0, dz=0.0, turn=90.0):
    """The front mechanism with its key turned `turn` deg about y and its tip `depth` past the near mouth."""
    r = ry(turn)
    tip = np.array([M.XN + M.FX - depth, M.YL + M.SLOT_Y + dy, M.TOP + M.H + M.SLOT_Z + dz])
    return front(r, tip - r @ np.asarray(M.KEY_TIP))


def near(slide=0.0, plunger=0.0, pin=0.0):
    o = np.asarray(M.NEAR.pos, float)
    return MechanismState(pos=o, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=Z3, ang_vel=Z3, low=o, high=o,
                          joints={"slide_j": slide, "plunger_j": plunger, "pin_j": pin},
                          links={"plunger": link(o), "slide": link(o + (0, slide, 0))})


def coin(pos=None, vel=Z3, flat=True):
    r = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.pi / 2) if flat else np.eye(3)
    return body_state(M.QUARTER, M.QUARTER.pos if pos is None else pos, quat(r), vel)


ON_MAT = (M.MAT_XY[0], M.MAT_XY[1], M.TOP + M.MAT[2] + M.COIN_T / 2)


def states(fr=None, nr=None, q=None):
    mat = ObjectState(pos=np.asarray(M.MAT_BOX.pos, float), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=Z3,
                      ang_vel=Z3, low=np.asarray(M.MAT_BOX.pos) - np.asarray(M.MAT) / 2,
                      high=np.asarray(M.MAT_BOX.pos) + np.asarray(M.MAT) / 2)
    return {"front": fr or front(), "near": nr or near(), "near_chain": near(), "quarter": q or coin(flat=False),
            "mat": mat}


HOME = M.HOME_DEPTH
OUT = -M.PUSH_TRAVEL


def goal():
    return states(key_at(HOME), near(OUT, -M.RELEASE_TRAVEL), coin(ON_MAT))


def sc(s, hands=FAR):
    M.set_hands(dict(hands))
    return M.score(s)


def end_states(mode):
    if mode == "demo":
        return goal()
    if mode in ("nothing", "pry_coin", "no_key_coin"):
        return states()
    if mode == "partway":
        return states(key_at(M.PLUNGER_IN))
    s = states(key_at(HOME), near(OUT, -M.RELEASE_TRAVEL),
               coin(np.asarray(M.COIN_START) + (0, -M.PUSH_TRAVEL, 0), flat=False))
    if mode == "drop_coin":
        s["quarter"] = coin((M.MAT_XY[0], M.MAT_XY[1] - 0.09, M.TOP + M.COIN_T / 2))
    if mode == "hold_coin":
        s["quarter"] = coin((M.MAT_XY[0], M.MAT_XY[1], M.TOP + M.MAT[2] + M.COIN_R + 0.03), flat=False)
    return s


class Scoring(unittest.TestCase):
    def test_start_zero(self):
        self.assertEqual(sc(states()), 0.0)

    def test_goal_success_but_score_capped(self):
        self.assertEqual(sc(goal()), 0.75)
        self.assertTrue(M.success(goal()))

    def test_hands_unread_zero(self):
        self.assertEqual(sc(goal(), {"left": None, "right": None}), 0.0)

    def test_entered_and_partway(self):
        self.assertEqual(sc(states(key_at(M.ENTER - 0.001))), 0.0)
        self.assertEqual(sc(states(key_at(M.ENTER + 0.001))), 0.25)
        self.assertEqual(sc(states(key_at(HOME))), 0.25)           # home depth, slide still in: 0.25

    def test_key_out_of_line(self):
        self.assertEqual(sc(states(key_at(HOME, dy=M.LINE_Y + 0.001), near(OUT))), 0.0)
        self.assertEqual(sc(states(key_at(HOME, dz=M.LINE_Z + 0.001), near(OUT))), 0.0)
        self.assertEqual(sc(states(key_at(HOME, turn=90 - M.LINE_DEG - 2), near(OUT))), 0.0)
        self.assertEqual(sc(states(key_at(HOME, turn=-90), near(OUT))), 0.0)   # key backwards: its back end in
        self.assertEqual(sc(states(front(), near(OUT))), 0.0)                 # hanging key

    def test_home_needs_depth_and_slide(self):
        self.assertEqual(sc(states(key_at(HOME), near(OUT))), 0.5)
        self.assertEqual(sc(states(key_at(HOME - M.HOME_TOL - 0.0005), near(OUT))), 0.25)   # short of home
        self.assertEqual(sc(states(key_at(M.ENTER - 0.001), near(OUT))), 0.0)

    def test_retention_rule(self):
        """0.5 and up need the lock to hold the key: home and the pin down in its hole, not the hand holding it."""
        self.assertEqual(sc(states(key_at(HOME), near(OUT, pin=0.0))), 0.5)
        self.assertEqual(sc(states(key_at(HOME), near(OUT, pin=M.PIN_DOWN + 0.0002))), 0.25)   # pin riding on the key
        s = goal()
        s["near"] = near(OUT, -M.RELEASE_TRAVEL, pin=0.002)
        self.assertEqual(sc(s), 0.25)
        self.assertFalse(M.success(s))
        self.assertEqual(sc(states(key_at(HOME), near(-M.SLIDE_OUT + 0.001))), 0.25)
        self.assertEqual(sc(states(None, near(OUT), coin(ON_MAT))), 0.0)        # slide out, quarter on mat, no key

    def test_quarter_rules(self):
        self.assertEqual(sc(states(key_at(HOME), near(OUT), coin(ON_MAT))), 0.75)
        off = (M.MAT_XY[0] + M.MAT[0] / 2 - M.MAT_IN + 0.001, M.MAT_XY[1], ON_MAT[2])
        self.assertEqual(sc(states(key_at(HOME), near(OUT), coin(off))), 0.5)
        up = (ON_MAT[0], ON_MAT[1], ON_MAT[2] + M.MAT_REST + 0.001)
        self.assertEqual(sc(states(key_at(HOME), near(OUT), coin(up))), 0.5)
        on_edge = (ON_MAT[0], ON_MAT[1], M.TOP + M.MAT[2] + M.COIN_R)
        self.assertEqual(sc(states(key_at(HOME), near(OUT), coin(on_edge, flat=False))), 0.75)

    def test_full_needs_still_and_clear(self):
        s = goal()
        s["quarter"] = coin(ON_MAT, vel=(0.02, 0, 0))
        self.assertFalse(M.success(s))
        s = goal()
        s["front"] = key_at(HOME)
        s["front"].links["ckey"] = link(s["front"].links["ckey"].pos, ry(90), (0.02, 0, 0))
        self.assertFalse(M.success(s))
        tip, _ = M.key_points(goal()["front"])
        for p, ok in ((tip + (0.0, -(M.CLEAR - 0.002), 0.0), False), (tip + (0.0, -(M.CLEAR + 0.01), 0.0), True)):
            M.set_hands(dict(FAR, right=np.array([p])))
            self.assertEqual(M.success(goal()), ok)
            self.assertEqual(M.score(goal()), 0.75)
        M.set_hands(dict(FAR, right=np.array([np.asarray(ON_MAT) + (0.0, 0.0, M.CLEAR - 0.002)])))
        self.assertFalse(M.success(goal()))

    def test_each_mode_expect(self):
        for m, want in M.EXPECT.items():
            s = end_states(m)
            if want == 1.0:     # score() stops at 0.75; 1.0 only from success() held SUCCESS_HOLD_S
                self.assertEqual(sc(s), 0.75, m)
                self.assertTrue(M.success(s), m)
            else:
                self.assertAlmostEqual(sc(s), want, msg=m)
                self.assertFalse(M.success(s), m)
        self.assertEqual(set(M.EXPECT), set(M.MODES))


class Construct(unittest.TestCase):
    def test_latch_geometry(self):
        overlap = min(M.T_X[1], M.L_X[1]) - max(M.T_X[0], M.L_X[0])
        self.assertAlmostEqual(overlap, M.RELEASE_TRAVEL, places=6)   # T clears L at the release travel
        self.assertLess(M.RELEASE_TRAVEL, M.PLUNGER_STOP)            # reachable before the plunger's stop
        self.assertLessEqual(M.L_Y[0] - M.PUSH_TRAVEL, M.T_Y[0])     # the rail lies across T with the slide out
        self.assertLess(M.L_Y[1], M.SLD_Y1)                          # the rail stays inside the casing
        self.assertAlmostEqual(M.HOME_DEPTH, M.PLUNGER_IN + M.RELEASE_TRAVEL)

    def test_retention_geometry(self):
        """Offline stand-in for 'the key stays in with the fingers open under its chain's pull': at home the key's hole
        wall meets the pin's upright face; that face is vertical (a pull along the key pushes it sideways, never up);
        the pin overlaps the key's thickness wherever the key lies in the slot; and the chain's pull (well under 1 N,
        along +x) has no part that could lift the sprung pin."""
        tip = M.FX - HOME
        self.assertAlmostEqual(tip + M.APER_D0, M.PIN_X - M.PIN_HX, places=9)
        p = checkkit.shape_points(M.pin_shapes()[0])
        x0 = p[:, 0].min()
        face = p[np.isclose(p[:, 0], x0)]
        self.assertAlmostEqual(np.ptp(face[:, 0]), 0.0)                      # upright -x face
        self.assertLess(face[:, 2].min(), M.Z_KEY0 + M.KEY_T - 0.001)        # key on the slot floor: 1 mm overlap
        self.assertGreater(face[:, 2].max(), M.Z_KEY1)                       # and the face spans a key at the top
        self.assertGreaterEqual(M.PIN_Z0, M.Z_KEY0)
        hi = p[np.isclose(p[:, 0], p[:, 0].max())][:, 2].min()
        self.assertGreater(hi, M.Z_KEY1)                                     # +x ramp starts above the slot: it lifts
        self.assertGreater(M.PIN_Z0 + M.PIN_RISE, M.Z_KEY1)                  # the pin lifts clear of any key
        chain_n = M.CHAIN_N * M.CHAIN_KG * 9.81 + M.KEY_KG * 9.81
        self.assertLess(chain_n, 1.0)
        self.assertGreater(M.PIN_K * M.PIN_PRE, 0.0)

    def test_not_home_not_retained(self):
        """Short of home the pin's footprint lies on solid key, so the pin stands 1.25 mm or more up: over PIN_DOWN."""
        x0, x1 = M.PIN_X - M.PIN_HX, M.PIN_X + M.PIN_HX
        for d in np.arange(M.ENTER, HOME - 0.0002, 0.0005):
            tip = M.FX - d
            in_hole = tip + M.APER_D0 <= x0 + 1e-9 and tip + M.APER_D1 >= x1 - 1e-9
            self.assertFalse(in_hole, d)
        self.assertGreater(M.Z_KEY0 + M.KEY_T - M.PIN_Z0, M.PIN_DOWN)        # riding: pin_j above PIN_DOWN
        tip = M.FX - HOME
        self.assertTrue(tip + M.APER_D0 <= x0 + 1e-9 and tip + M.APER_D1 >= x1)   # home: the pin drops in
        self.assertGreaterEqual(M.FX - (x1 - M.APER_D1), M.PLUNGER_IN + M.PLUNGER_STOP)   # to the plunger's stop

    def test_key_fits_slot_with_play(self):
        self.assertGreater(M.SLOT_W - M.KEY_W, 0.002)
        self.assertGreaterEqual(M.SLOT_T - M.KEY_T, 0.0015 - 1e-9)

    def test_finger_cannot_reach_plunger(self):
        """A single open finger, tip first, fits the slot only as deep as its cross-section stays inside it."""
        p = mk.HAND.local(mk.FINGER_OPEN)
        one = p[p[:, 0] > 0.02]
        tip = one[:, 2].max()
        depth = 0.0
        for d in np.arange(0.0, 0.03, 0.0005):
            s = one[one[:, 2] >= tip - d]
            a, b = np.ptp(s[:, 0]), np.ptp(s[:, 1])
            if not ((a <= M.SLOT_T and b <= M.SLOT_W) or (b <= M.SLOT_T and a <= M.SLOT_W)):
                break
            depth = d
        pushed = depth - M.PLUNGER_IN          # how far a finger could press the plunger
        self.assertLess(pushed, M.RELEASE_TRAVEL - 0.001, f"finger fits {1000 * depth:.1f} mm deep")

    def test_coin_hidden_until_released(self):
        top = M.COIN_Z + M.COIN_R
        self.assertLess(top, M.Z_SLD1)                     # under the casing's cover
        self.assertGreater(M.COIN_Y0 - M.COIN_R, -M.LOCK_L / 2)   # wholly inside the casing
        out_edge = M.COIN_Y0 - M.PUSH_TRAVEL + M.COIN_R
        self.assertLess(out_edge, -M.LOCK_L / 2)           # wholly out once released

    def test_key_cannot_reach_its_own_lock(self):
        """The chain is shorter than the way from its anchor round to the front lock's own slot: the patent's 'cannot
        be inserted into its own coin lock'."""
        a = np.array(M.ANCHOR)
        slot_back = np.array([M.FX + M.KEY_L - M.HOME_DEPTH, M.SLOT_Y, M.SLOT_Z])
        way = abs(a[0] - (-M.LOCK_W / 2)) + M.LOCK_W + (slot_back[0] - M.FX) + abs(slot_back[2] - a[2])
        self.assertGreater(way, 0.0)
        reach_to_near = np.linalg.norm((np.array([M.XN, 0, 0]) + slot_back) - (np.array([M.XF, 0, 0]) + a))
        self.assertLess(reach_to_near, M.CHAIN_N * M.CHAIN_P)    # the near slot is within the chain's reach

    def test_key_hangs_free(self):
        tip_z = M.TOP + M.H + M.KEY_TOP - M.KEY_L
        self.assertGreater(tip_z - M.TOP, 0.05)

    def test_quarter_and_lock_numbers(self):
        self.assertEqual((M.COIN_D, M.COIN_T, M.COIN_KG), (0.02426, 0.00175, 0.00567))
        self.assertEqual((M.LOCK_L, M.LOCK_W, M.LOCK_H), (0.090, 0.070, 0.068))
        self.assertAlmostEqual(M.NEST, 0.1778, places=4)
        src = (HERE / "src" / "usmint_quarter.txt").read_text()
        for s in ("24.26 mm", "1.75 mm", "5.670 g"):
            self.assertIn(s, src)
        self.assertIn("Size:90*70*68mm", (HERE / "src" / "malltek_coinlock.txt").read_text())
        self.assertIn('Nesting Distance 7"', (HERE / "src" / "unarco_M60TN.txt").read_text())


class Plans(unittest.TestCase):
    def test_loads(self):
        for m in M.MODES:
            self.assertTrue(set(M.plan(0, m)) <= {"left", "right"}, m)

    def test_offline_gates(self):
        for gate, ok, detail in checkkit.check(HERE / "trolley.py", M.MODES, (0,)):
            self.assertTrue(ok, f"{gate}: {detail}")

    def test_hands_clear_at_end(self):
        for m in ("demo",):
            W = M.plan(0, m)["right"]
            w = W[-1]
            pts = mk.HAND.world(w.pos, mk.harness_demo.grip_rotation(w.yaw), w.finger)
            g = goal()
            M.set_hands({"left": FAR["left"], "right": pts})
            self.assertTrue(M.hands_clear(g))


class Text(unittest.TestCase):
    def test_text(self):
        tj = json.loads((HERE / "task.json").read_text())
        for k in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):
            self.assertTrue(tj.get(k), k)
        self.assertIn("media", tj)
        steps = int(re.search(r"[\d,]+", tj["run_limit"]).group().replace(",", ""))
        self.assertEqual(steps, M.RUN_LIMIT_STEPS)
        readme = (HERE / "README.md").read_text()
        self.assertLessEqual(len(readme.splitlines()), 150)
        self.assertLessEqual(len(tj["fidelity"]), 500)
        for text in (readme, json.dumps(tj, ensure_ascii=False), (HERE / "trolley.py").read_text()):
            self.assertNotRegex(text, "[–—]")

    def test_numbers_in_text_match_code(self):
        tj = json.loads((HERE / "task.json").read_text())
        both = tj["setup"] + tj["scoring"] + (HERE / "README.md").read_text()
        mm = lambda v: f"{round(v * 1000, 1):g} mm"  # noqa: E731
        for v in (M.ENTER, M.SLIDE_OUT, M.PUSH_TRAVEL, M.MAT_IN, M.MAT_REST, M.CLEAR,
                  M.CHAIN_N * M.CHAIN_P, M.LINE_Y, M.LINE_Z):
            self.assertIn(mm(v), both, mm(v))
        self.assertIn(f"{M.LINE_DEG:g} degrees", both)
        self.assertIn(f"{M.SUCCESS_HOLD_S:g} s", tj["scoring"])


if __name__ == "__main__":
    unittest.main(verbosity=1)
