"""Offline tests for trolley.py: loads; the start scores 0; the goal scores 0.75 from score() and is a success; every
scoring rule against the code, each mode's planned end state against EXPECT, the closed shortcuts (a finger cannot
reach the plunger, the coin slide cannot come out without the key, the near lock's own key scores 0), the latch
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

    def test_key_behind_lock_zero(self):
        """A key held in line with the slot but behind the near lock (tip past its back face) is not in the slot."""
        self.assertEqual(sc(states(key_at(M.MAX_IN - 0.0005))), 0.25)
        for depth in (M.MAX_IN + 0.001, M.LOCK_W + 0.002, 0.13, 0.20):
            self.assertEqual(sc(states(key_at(depth))), 0.0, depth)

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
        self.assertEqual(sc(states(key_at(HOME), near(OUT), coin(on_edge, flat=False))), 0.5)   # on edge: not laid

    def test_held_quarter_is_not_on_the_mat(self):
        """A flat quarter still in the gripper just over the mat scores 0.5, not 0.75 (hands 10 mm off it for 0.75)."""
        hover = (ON_MAT[0], ON_MAT[1], ON_MAT[2] + 0.0029)
        top = np.asarray(hover) + (0.0, 0.0, M.COIN_T / 2)
        for gap, want in ((M.COIN_FREE - 0.002, 0.5), (M.COIN_FREE + 0.002, 0.75)):
            hands = dict(FAR, right=np.array([top + (0.0, 0.0, gap)]))
            self.assertEqual(sc(states(key_at(HOME), near(OUT), coin(hover)), hands), want, gap)

    def test_quarter_tilt_limit(self):
        """The quarter counts only lying within COIN_FLAT_DEG of flat, its lowest rim point on the mat."""
        for tilt, want in ((M.COIN_FLAT_DEG - 3, 0.75), (M.COIN_FLAT_DEG + 3, 0.5)):
            r = ry(90 - tilt)                                   # body x (face normal) tilted `tilt` from vertical
            a = math.radians(tilt)
            z = M.TOP + M.MAT[2] + M.COIN_R * math.sin(a) + M.COIN_T / 2 * math.cos(a)
            q = body_state(M.QUARTER, (ON_MAT[0], ON_MAT[1], z), quat(r), Z3)
            self.assertAlmostEqual(M.coin_low_z(q), M.TOP + M.MAT[2], places=6)
            self.assertEqual(sc(states(key_at(HOME), near(OUT), q)), want, tilt)

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
        """Short of home the pin's footprint lies on solid key, so the pin stands 2.5 mm or more up: over PIN_DOWN."""
        x0, x1 = M.PIN_X - M.PIN_HX, M.PIN_X + M.PIN_HX
        for d in np.arange(M.ENTER, HOME - 0.0002, 0.0005):
            tip = M.FX - d
            in_hole = tip + M.APER_D0 <= x0 + 1e-9 and tip + M.APER_D1 >= x1 - 1e-9
            self.assertFalse(in_hole, d)
        self.assertGreater(M.Z_KEY0 + M.KEY_T - M.PIN_Z0, M.PIN_DOWN)        # riding: pin_j above PIN_DOWN
        tip = M.FX - HOME
        self.assertTrue(tip + M.APER_D0 <= x0 + 1e-9 and tip + M.APER_D1 >= x1)   # home: the pin drops in
        self.assertGreaterEqual(M.FX - (x1 - M.APER_D1), M.PLUNGER_IN + M.PLUNGER_STOP)   # to the plunger's stop

    def test_off_centre_key_retained(self):
        """A key pushed home anywhere in the slot's play is caught: the pin's footprint lies inside the key's hole for
        a key at either side of the slot, and such a key with the slide out and the pin down scores 0.5."""
        play = (M.SLOT_W - M.KEY_W) / 2
        self.assertGreater(play, 0.002)
        for dy in (-play, 0.0, play):
            hole = (M.SLOT_Y + dy - M.APER_W / 2, M.SLOT_Y + dy + M.APER_W / 2)
            pin = (M.SLOT_Y - M.PIN_HY, M.SLOT_Y + M.PIN_HY)
            self.assertLessEqual(hole[0], pin[0], dy)
            self.assertGreaterEqual(hole[1], pin[1], dy)
            self.assertEqual(sc(states(key_at(HOME, dy=dy), near(OUT))), 0.5, dy)
        self.assertGreaterEqual((M.KEY_W - M.APER_W) / 2, 0.002)            # the key's rails beside the hole stay 2 mm+

    def test_basket_wall_under_handle(self):
        """The near basket's rear wall stands between the near handle and the hanging key, under the handle, and below
        the level key's path to the slot."""
        shapes = M.basket_shapes()
        top = max(float(sh.bounds()[1][2]) for sh in shapes)
        self.assertLess(top, M.Z_KEY0 - M.KEY_W / 2 - 0.02)                 # 20 mm under a key turned on edge
        key_x = M.XF - M.XN + M.ANCHOR[0]                                   # hanging key's line, near lock frame
        self.assertGreater(M.BASKET_X - M.WIRE_R, M.TUBE_R)
        self.assertLess(M.BASKET_X + M.WIRE_R, key_x - M.KEY_T - 0.05)
        self.assertGreater(M.H + M.BASKET_Z0, 0.0)                          # off the bench

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

    def test_own_key_scores_zero(self):
        """The near lock's own key hangs on its own chain. Its chain reaches round to its own slot (a real lock stops
        this; EP0427819B1 says the key 'cannot be inserted into its own coin lock'), but only the front lock's key
        counts: the near key home in the near slot with the slide out and the quarter on the mat scores 0."""
        a = np.array(M.ANCHOR)
        back = np.array([M.FX + M.KEY_L - M.HOME_DEPTH + M.KEY_EYE, M.SLOT_Y, M.SLOT_Z])
        way = abs(a[0] - (-M.LOCK_W / 2)) + M.LOCK_W + (back[0] - M.FX) + abs(back[2] - a[2])
        self.assertLess(way, M.CHAIN_N * M.CHAIN_P)              # it can reach: scoring, not geometry, closes it
        self.assertIn("nkey", [ln.name for ln in M._own_links])
        s = states(None, near(OUT, -M.RELEASE_TRAVEL), coin(ON_MAT))
        r = ry(90)
        tip = np.array([M.XN + M.FX - HOME, M.YL + M.SLOT_Y, M.TOP + M.H + M.SLOT_Z])
        s["near_chain"].links["nkey"] = link(tip - r @ np.asarray(M.KEY_TIP), r)
        self.assertEqual(sc(s), 0.0)
        self.assertFalse(M.success(s))

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
        self.assertIn("Size:90*70*68mm", (HERE / "src" / "supermarket_fixture_coinlock.txt").read_text())
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
        for v in (M.ENTER, M.MAX_IN, M.SLIDE_OUT, M.PUSH_TRAVEL, M.MAT_IN, M.MAT_REST, M.CLEAR,
                  M.CHAIN_N * M.CHAIN_P, M.LINE_Y, M.LINE_Z):
            self.assertIn(mm(v), both, mm(v))
        self.assertIn(f"{M.LINE_DEG:g} degrees", both)
        self.assertIn(f"{M.SUCCESS_HOLD_S:g} s", tj["scoring"])


def scad_params(path=HERE / "cad" / "coinlock.scad"):
    """(block-1 SIM numbers as written, every top-level `NAME = expr;` evaluated in file order) from the .scad."""
    sim, env, block = {}, {"sqrt": math.sqrt, "pow": math.pow}, 0
    for line in path.read_text().splitlines():
        b = re.match(r"// ---- (\d)\.", line)
        if b:
            block = int(b.group(1))
        m = re.match(r"^([A-Z][A-Z0-9_]*)\s*=\s*([^;]+);", line)
        if m:
            env[m.group(1)] = eval(m.group(2), {"__builtins__": {}}, env)  # noqa: S307  WHY: our own file
            if block == 1 and re.fullmatch(r"-?[\d.]+", m.group(2).strip()):
                sim[m.group(1)] = float(m.group(2))
    return sim, env


def box_of(shape):
    """(low, high) corners of a sim box shape, in mm."""
    c, z = np.asarray(shape.center) * 1000, np.asarray(shape.size) * 1000 / 2
    return c - z, c + z


class Cad(unittest.TestCase):
    """The printable lock in cad/coinlock.scad has the sim's dimensions: every SIM number equals the trolley.py
    constant, the derived contact geometry equals the sim's shapes, and the bought springs work in their pockets."""
    SIM, P = scad_params()
    VECTORS = {"T_X": ("T_X0", "T_X1"), "T_Y": ("T_Y0", "T_Y1"), "L_X": ("L_X0", "L_X1"), "L_Y": ("L_Y0", "L_Y1"),
               "HOLE": ("HOLE_X0", "HOLE_X1", "HOLE_Y0", "HOLE_Y1"), "ANCHOR": ("ANCHOR_X", "ANCHOR_Y", "ANCHOR_Z")}

    def test_sim_numbers_equal_scene_constants(self):
        done = set()
        for vec, names in self.VECTORS.items():
            for name, v in zip(names, getattr(M, vec)):
                self.assertAlmostEqual(self.SIM[name], v * 1000, places=6, msg=name)
                done.add(name)
        for name in ("TOOTH_TOP", "LUG_TOP"):
            done.add(name)
        for name, v in self.SIM.items():
            if name not in done:
                self.assertTrue(hasattr(M, name), f"{name} is not a trolley.py constant")
                self.assertAlmostEqual(v, getattr(M, name) * 1000, places=6, msg=name)
        self.assertEqual(len(self.SIM), 69)

    def test_parts_match_sim_shapes(self):
        p = self.P
        body, tooth = (box_of(s) for s in M.plunger_shapes())
        np.testing.assert_allclose(body[0], (p["PL_X0"], p["SY0"] + 0.5, p["Z_KEY0"] + 0.5), atol=1e-6)
        np.testing.assert_allclose(body[1], (p["PL_X1"], p["SY1"] - 0.5, p["Z_KEY1"] - 0.5), atol=1e-6)
        np.testing.assert_allclose(tooth[1], (p["T_X1"], p["T_Y1"], p["TOOTH_TOP"]), atol=1e-6)
        sl = M.slide_shapes()
        np.testing.assert_allclose(box_of(sl[-1])[1], (p["L_X1"], p["L_Y1"], p["LUG_TOP"]), atol=1e-6)
        wall = box_of(sl[4])                                     # the coin pocket's -x wall
        np.testing.assert_allclose(wall[0], (p["W_LO"] - p["WALL_T"], p["CY0"], p["SLIDE_FLOOR"]), atol=1e-6)
        np.testing.assert_allclose(wall[1], (p["W_LO"], p["CY1"], p["SLIDE_FLOOR"] + p["WALL_H"]), atol=1e-6)
        pts = np.asarray(M.pin_shapes()[0].points) * 1000
        self.assertAlmostEqual(pts[:, 2].max(), p["PIN_TOP"], places=6)
        self.assertAlmostEqual(pts[:, 2].min(), p["PIN_Z0"], places=6)
        self.assertAlmostEqual(box_of(M.casing_shapes()[3])[1][0], p["BEHIND_X1"], places=6)

    def test_real_stops_are_sim_limits(self):
        p = self.P
        # T's slot in the cover: its +x end holds the plunger at rest (joint 0), its -x end at PLUNGER_STOP
        self.assertIn("bx([T_X0 - PLUNGER_STOP, T_Y0 - CLR, Z_KEY1 - 1], [T_X1, T_Y1 + CLR, Z_COV1 + 1]);",
                      (HERE / "cad" / "coinlock.scad").read_text())
        # the slide's post meets the cap's stop block after PUSH_TRAVEL
        self.assertAlmostEqual(p["POST_Y0"] - p["STOP_Y1"], M.PUSH_TRAVEL * 1000)
        # the pin's head sits on the bore's shoulder at the sim's rest and meets the bore top at PIN_RISE
        self.assertAlmostEqual(p["PIN_TOP"], (M.PIN_Z0 + 0.006) * 1000)
        # nothing added reaches the key: the arm ends behind the homed key's deepest tip (plunger stop, 14 mm)
        self.assertLess(p["ARM_X1"] + p["CLR"], p["FX"] - (M.PLUNGER_IN + M.PLUNGER_STOP) * 1000)
        self.assertGreaterEqual(p["T_Y1"], p["ARM_Y0"])

    def test_springs_fit_and_stay_in_range(self):
        p = self.P
        # plunger spring VD-006C: preload PLUNGER_PRE at rest, PLUNGER_PRE + PLUNGER_STOP at the stop
        self.assertLessEqual(p["PLUNGER_PRE"] + p["PLUNGER_STOP"], p["PSPR_SN"])
        self.assertGreaterEqual(p["PSPR_L0"] - p["PLUNGER_PRE"] - p["PLUNGER_STOP"], p["PSPR_LN"])
        self.assertGreaterEqual(p["CHEEK_GAP"], p["PSPR_DH"])
        self.assertGreater(p["PL_X0"] - p["PLUNGER_STOP"] - p["CHEEK_L"], p["PSPR_X0"])
        # pin spring VD-006: preload PIN_PRE at rest, PIN_PRE + PIN_RISE fully lifted
        self.assertLessEqual(p["PIN_PRE"] + p["PIN_RISE"], p["PINSPR_SN"])
        self.assertGreaterEqual(p["PIN_BORE_D"], p["PINSPR_DH"])
        self.assertAlmostEqual(p["PIN_SPR_Z1"] - p["PIN_SPR_Z0"], p["PINSPR_L0"] - p["PIN_PRE"])
        # slide spring Century ST4-15 cut to length, on the 3 mm rod: short of solid with the slide in, and within 15%
        # of the sim's slide spring force at both slide positions (in: q = 0, out: q = -PUSH_TRAVEL)
        self.assertLessEqual(p["SSPR_DEFL_IN"], p["SSPR_SN"])
        self.assertGreater(p["SSPR_DEFL_IN"] - p["PUSH_TRAVEL"], 0)    # still pushes at the out stop
        for q in (0.0, -M.PUSH_TRAVEL):
            sim = M.SLIDE_K * (q + M.PUSH_TRAVEL + M.SLIDE_PRE)
            real = p["SSPR_R"] * (p["SSPR_DEFL_IN"] + q * 1000)
            self.assertLess(abs(real / sim - 1), 0.15, (q, real, sim))
        self.assertLessEqual(p["ROD_D"], p["SSPR_DI"])
        self.assertGreater(p["STOP_Y0"], -p["HY"])                       # the out-stop block is inside the cap
        self.assertGreater(p["ROD_Y0"], -p["HY"])
        self.assertLess(p["SPR_Z"] + p["ROD_D"] / 2, p["STOP_Z0"])      # the rod passes under the stop block
        self.assertGreater(p["POST_TOP"], p["STOP_Z0"])                  # the post does not
        self.assertGreater(p["SPR_Z"] - p["SSPR_DE"] / 2, p["SLIDE_FLOOR"] + p["COIN_D"])   # spring over the coin

    def test_files_exported(self):
        for part in ("floor_plate", "clamp_lower", "key_plate", "cover_plate", "shell", "cap", "plunger", "slide",
                     "pin", "key"):
            f = HERE / "cad" / "stl" / f"{part}.stl"
            self.assertTrue(f.exists() and f.stat().st_size > 1000, part)
            self.assertRegex((HERE / "cad" / "make.sh").read_text(), rf"\b{part}\b")
        self.assertTrue((HERE / "cad" / "key.dxf").exists())
        self.assertTrue((HERE / "cad" / "drawing.svg").exists())


if __name__ == "__main__":
    unittest.main(verbosity=1)
