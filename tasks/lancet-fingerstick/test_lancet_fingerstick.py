"""Offline tests for lancet_fingerstick.py (no simulator): the site, the press and firing model, the scorer, the lazy
routes, and both arms' demo replayed through the model on each film seed by a kinematic stand-in for the sim (a gripped
lancet follows the TCP; the press is the arm's compliance in series with the shield's spring; an unheld hand tips
past its tipping force). Run with yam_arena on PYTHONPATH."""
import json
import math
import pathlib
import types
import unittest
import warnings
import zlib

import numpy as np

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parent
from yam_arena.task_file import load  # noqa: E402

M = load(HERE / "lancet_fingerstick.py").module
CLI = (0, 2, 4)
N_SEEDS = 20                     # inspect-robots --seed 0..19 for the many-seed replay
N_PAIRS = 20                     # seed pairs (a, a + 20) for the open-loop replay
SIM_SEEDS = (26385, 84277, 5410, 55059, 126748, 5673, 58369, 100825)   # --seed nearest each layout corner (README)
DT = 0.05
K_ARM = 500.0                    # N/m at the TCP in z [EST, as the task's PRESS_DZ comment]
W = M.HAND_KG * 9.81
GRIP_READ = -(M.L_T / 2 - 0.0003)   # the finger's reading closed on the 10 mm side: 0.3 mm in [RUN, sim trace]


def trial_seed(cli):
    return zlib.crc32(f"{cli}:None:0".encode())


def q_from_R(r):
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def st(pos, quat=(0.0, 0.0, 0.0, 1.0)):
    return types.SimpleNamespace(pos=np.asarray(pos, float), quat_xyzw=np.asarray(quat, float),
                                 lin_vel=np.zeros(3))


def hand_link(seed, theta=0.0):
    """The hand link's frame (world pos, quat): the mechanism's frame turned theta about the pivot's axis."""
    hx, hy, hz, yaw = M.layout(seed)["hand"]
    Ry = ry(theta)
    piv = np.array([M.PIVOT_X, 0.0, M.REST_F_Z])
    Rw = rz(yaw)
    pos = np.array([hx, hy, hz]) + Rw @ (piv - Ry @ piv)
    return pos, q_from_R(Rw @ Ry)


class Kin:
    """A kinematic stand-in for the sim, both arms."""
    GRAB = 0.004

    def __init__(self, seed):
        self.seed = seed
        self.l = np.array(M.L_START, float)
        self.lyaw, self.tab, self.q, self.theta = 0.0, 0.0, 0.0, 0.0
        self.r = [np.array([0.323, -0.305, 1.008]), M.OPEN_F, 0.0]
        self.lf = [np.array(M.PARK_L, float), M.OPEN_F, 0.0]
        self.held, self.off = False, None
        self.trace = []

    def _lancet_quat(self):
        return q_from_R(rz(self.lyaw))

    def _left_holds(self):
        g, _ = M.stub_world(self.seed)
        g = g + [0, 0, M.STUB_TCP]
        p, f, _ = self.lf
        return bool(-f <= M.STUB_R + 0.0005 and math.hypot(*(p[:2] - g[:2])) <= 0.004 and p[2] <= g[2] + 0.002)

    def _press(self, cmd_face):
        """The shield's travel and the face's true place for a commanded face point, the hand tipping if unheld."""
        for theta in ((0.0, M.TILT_STOP) if not self._left_holds() else (0.0,)):
            hp, hq = hand_link(self.seed, theta)
            Rh = M._rot3(hq)
            pm = Rh.T @ (cmd_face - hp)
            # how far the commanded face is inside the skin, along the lancet's axis (vertical)
            depth = self._depth(cmd_face, hp, Rh)
            if depth <= 0.0:
                self.theta = theta
                return 0.0, 0.0
            f = (depth + M.F_PRE / M.K_SHIELD) / (1 / M.K_SHIELD + 1 / K_ARM)
            q = min(M.SHIELD_TRAVEL, max(0.0, (f - M.F_PRE) / M.K_SHIELD))
            if f < M.F_PRE:
                q = 0.0
            lever = M.PIVOT_X - float(pm[0])
            if theta == 0.0 and not self._left_holds() and lever > 0 and M.shield_force(q) * lever > W * (
                    M.COM_X - M.PIVOT_X):
                continue   # it tips: try again with the hand on its stop
            self.theta = theta
            return q, depth - q
        return 0.0, 0.0

    def _depth(self, face, hp, Rh):
        """How far (m) a commanded face point sits under the hand's top surface straight above it (0 if none)."""
        def inside(z):
            p = np.array([face[0], face[1], z])
            s = M.site_of(Rh.T @ (p - hp))
            return s is not None and s[4] < 0.0 and "palm" not in s[0] and "forearm" not in s[0]
        zs = [face[2] + 0.0005 * i for i in range(101)]
        top = None
        for z in zs:
            if inside(z):
                top = z
        if top is None:
            return 0.0
        lo, hi = top, top + 0.0005
        for _ in range(30):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if inside(mid) else (lo, mid)
        return hi - face[2]

    def step(self):
        p, f, yaw = self.r
        grip = self.l + [0, 0, M.tcp_over_lancet()]
        closed = -f <= M.L_T / 2 + 0.0005
        if not self.held and closed and np.linalg.norm(p - grip) <= self.GRAB:
            self.held, self.off = True, self.lyaw - yaw
        elif self.held and not closed:
            self.held = False
        self.q = 0.0
        if self.held:
            cmd = p - [0, 0, M.tcp_over_lancet()]
            self.lyaw = yaw + self.off
            if M.in_socket(cmd) and self.tab < math.radians(100):
                self.tab = min(math.radians(100), max(self.tab, self.lyaw))
            self.q, back = self._press(cmd.copy()) if not M.in_socket(cmd) else (0.0, 0.0)
            self.l = cmd + [0, 0, back]
        elif not M.in_socket(self.l) and not M.in_sharps(self.l) and self.l[2] > M.TOP + 0.0005:
            ox, oy = M.SH_OPEN[0] / 2, M.SH_OPEN[1] / 2
            if abs(self.l[0] - M.SHARPS[0]) <= ox and abs(self.l[1] - M.SHARPS[1]) <= oy:
                self.l = np.array([self.l[0], self.l[1], M.TOP + M.SH_WALL])
            else:
                self.l = np.array([self.l[0], self.l[1], M.TOP])
        hp, hq = hand_link(self.seed, self.theta)
        # a finger closed on the housing stops at it: report what the sim reads, not the command
        right = (self.r[0].copy(), min(self.r[1], GRIP_READ) if self.held else self.r[1])
        left = (self.lf[0].copy(), self.lf[1])
        M.advance(M._M, DT, self.l.copy(), self._lancet_quat(), self.q, self.tab, self.theta, hp, hq, right, left)
        M._CLOCK["dt"] = DT
        M._CLOCK["steps"] += 1
        M._M["steps"] += 1
        self.trace.append(self.objects())

    def objects(self):
        hx, hy, hz, yaw = M.layout(self.seed)["hand"]
        return {"hand": st((hx, hy, hz), q_from_R(rz(yaw))), "lancet": st(self.l, self._lancet_quat()),
                "bench": st((0.0, 0.0, M.TOP))}

    def run(self, right, left, tail=2.0):
        segs = {"r": list(right), "l": list(left)}
        state = {"r": [0, 0.0, None], "l": [0, 0.0, None]}
        t, end = 0.0, max(sum(w.duration for w in right), sum(w.duration for w in left)) + tail
        while t < end - 1e-9:
            t += DT
            for k, arm in (("r", self.r), ("l", self.lf)):
                i, t0, start = state[k]
                ws = segs[k]
                while i < len(ws) and t > t0 + ws[i].duration + 1e-9:
                    t0 += ws[i].duration
                    i += 1
                    start = None
                if i >= len(ws):
                    continue
                w = ws[i]
                if start is None:
                    start = (arm[0].copy(), arm[1], arm[2])
                a = min(1.0, (t - t0) / w.duration)
                arm[0] = start[0] + a * (np.asarray(w.pos) - start[0])
                arm[1] = start[1] + a * (w.finger - start[1])
                arm[2] = start[2] + a * (w.yaw - start[2])
                state[k] = [i, t0, start]
            self.step()
            M.update(self.trace[-1])
        return M.score(self.trace[-1])


def replay(seed, plan_seed=None, edit=None):
    M.reset_memory(seed)
    right, left = M.plan(0.0, seed if plan_seed is None else plan_seed)
    if edit:
        right, left = edit(right, left)
    k = Kin(seed)
    return k.run(right, left), k


def site_point(name, s, ang):
    """A point on a finger's distal pad (mechanism frame): s back from its apex, ang round from the pad's middle line."""
    v, u0, _, rd, _ = M.FINGERS[name]
    return M.hand_to_mech((u0 + s, v + rd * math.sin(ang), rd * math.cos(ang)))


def fresh():
    M.reset_memory(0)
    M._M["tab_off"] = M._M["out"] = True
    return M._M


def press_at(m, p_mech, f, theta=0.0, l_quat=(0.0, 0.0, 0.0, 1.0), seed=0, right=None):
    """One model step with the shield's face at p_mech (hand frame at rest) under force f, the lancet held."""
    hp, hq = hand_link(seed, theta)
    face = hp + M._rot3(hq) @ np.asarray(p_mech, float)
    q = max(0.0, (f - M.F_PRE) / M.K_SHIELD) if f > M.F_PRE else 0.0
    l_pos = face - M._rot3(l_quat) @ np.array([0.0, 0.0, q])
    tcp = M.grip_point(l_pos, l_quat)
    M.advance(m, DT, l_pos, l_quat, q, M.TAB_BREAK + 0.1, theta, hp, hq,
              right if right is not None else (tcp, GRIP_READ), None)


GOOD = site_point("ring", M.SITE_AIM, -math.radians(30))


class Model(unittest.TestCase):
    def test_shield_force(self):
        self.assertEqual(M.shield_force(0.0), 0.0)
        self.assertAlmostEqual(M.shield_force(0.001), M.F_PRE + M.K_SHIELD * 0.001)
        q_fire = (M.F_FIRE - M.F_PRE) / M.K_SHIELD
        self.assertLess(q_fire, M.SHIELD_TRAVEL)    # it fires before the shield reaches its stop

    def test_demo_site_is_valid_and_faces_up(self):
        s = M.site_of(np.array([M.SITE_AIM, 0.0, M.FINGERS["ring"][3]]))
        self.assertEqual(s[0], "ring")
        self.assertEqual(M.site_ok(s), (True, ""))
        self.assertAlmostEqual(float(s[3] @ np.array([0.0, 0.0, -1.0])), 1.0, places=9)   # inward normal is down

    def test_site_rules(self):
        cases = [(("ring", 0.008, 0.0), "middle of the ring's pad"),
                 (("ring", 0.008, math.radians(70)), "too far round"),
                 (("ring", 0.020, -0.5), "off the pad"),
                 (("little", 0.008, -0.5), "little finger"),
                 (("index", 0.008, -0.5), "not the side of a middle or ring"),
                 (("middle", 0.008, -0.5), None),
                 (("ring", 0.008, 0.5), None)]   # the other side of the pad is a site too (BD shades both)
        for (name, s, ang), why in cases:
            ok, msg = M.site_ok(M.site_of(site_point(name, s, ang)))
            if why is None:
                self.assertTrue(ok, msg)
            else:
                self.assertFalse(ok)
                self.assertIn(why, msg)
        self.assertIn("not the fingertip", M.site_of(site_point("ring", 0.040, -0.5))[0])
        self.assertIsNone(M.site_of(np.array([0.0, 0.0, 0.05])))

    def test_lifted_with_tab_on_latches(self):
        M.reset_memory(0)
        lp = np.array(M.L_START) + [0, 0, M.LIFT_OUT + 0.002]
        hp, hq = hand_link(0)
        M.advance(M._M, DT, lp, (0, 0, 0, 1), 0.0, math.radians(30), 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)),
                                                                                  GRIP_READ))
        self.assertIn("tab still on", M._M["fault"])

    def test_twisted_then_lifted_is_fine(self):
        M.reset_memory(0)
        hp, hq = hand_link(0)
        lp = np.array(M.L_START)
        M.advance(M._M, DT, lp, (0, 0, 0, 1), 0.0, M.TAB_BREAK + 0.01, 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)),
                                                                                    GRIP_READ))
        lp = lp + [0, 0, M.LIFT_OUT + 0.002]
        M.advance(M._M, DT, lp, (0, 0, 0, 1), 0.0, 0.0, 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)), GRIP_READ))
        self.assertIsNone(M._M["fault"])
        self.assertTrue(M._M["tab_off"] and M._M["out"])

    def test_good_press_fires_clean(self):
        m = fresh()
        for f in (2.5, 5.0, 8.5):
            press_at(m, GOOD, f)
        self.assertTrue(m["fired"])
        self.assertTrue(m["positioned"])
        self.assertIsNone(m["fault"])

    def test_placed_needs_a_firm_press(self):
        """A light touch on a good site is not "placed": the rung needs a press over the hand's tipping force."""
        m = fresh()
        press_at(m, GOOD, M.F_FIRM - 0.5)
        self.assertFalse(m["positioned"])
        press_at(m, GOOD, M.F_FIRM + 0.1)
        self.assertTrue(m["positioned"])
        self.assertGreater(M.F_FIRM, W * (M.COM_X - M.PIVOT_X) / (M.PIVOT_X - M.SITE_S[0]))

    def test_no_fire_under_the_fire_force(self):
        m = fresh()
        press_at(m, GOOD, M.F_FIRE - 0.2)
        self.assertFalse(m["fired"])

    def test_slide_latches(self):
        m = fresh()
        press_at(m, GOOD, 3.0)
        press_at(m, GOOD + np.array([M.SLIDE_MAX + 0.0005, 0.0, 0.0]), 4.0)
        self.assertIn("slid", m["fault"])

    def test_small_creep_and_lift_reset_are_fine(self):
        m = fresh()
        press_at(m, GOOD, 3.0)
        press_at(m, GOOD + np.array([0.0005, 0.0, 0.0]), 4.0)
        press_at(m, GOOD + np.array([0.003, 0.0, 0.0]), 0.0)   # lifted off: a new placement may start elsewhere
        press_at(m, GOOD + np.array([0.003, 0.0, 0.0]), 3.0)
        press_at(m, GOOD + np.array([0.003, 0.0, 0.0]), 9.0)
        self.assertIsNone(m["fault"])
        self.assertTrue(m["fired"])

    def test_tilted_lancet_latches(self):
        m = fresh()
        tq = (math.sin(math.radians(10)), 0.0, 0.0, math.cos(math.radians(10)))   # 20 deg about x
        press_at(m, GOOD, 3.0, l_quat=tq)
        press_at(m, GOOD, 9.0, l_quat=tq)
        self.assertIn("tilted", m["fault"])

    def test_unsteadied_wrist_latches(self):
        m = fresh()
        theta = -math.asin((M.WRIST_LIFT_MAX + 0.001) / (M.STUB_X - M.PIVOT_X))
        press_at(m, GOOD, 3.0, theta=theta)
        press_at(m, GOOD, 9.0, theta=theta)
        self.assertIn("not steadied", m["fault"])

    def test_fire_on_little_finger_latches(self):
        m = fresh()
        p = site_point("little", 0.008, -math.radians(30))
        press_at(m, p, 3.0)
        press_at(m, p, 9.0)
        self.assertIn("little finger", m["fault"])

    def test_fire_off_the_hand_latches(self):
        m = fresh()
        p = np.array([-0.05, 0.0, -M.HZ])   # on the table before the fingertips
        press_at(m, p, 3.0)
        press_at(m, p, 9.0)
        self.assertIn("not on the hand", m["fault"])

    def test_drop_outside_latches_and_sharps_does_not(self):
        m = fresh()
        hp, hq = hand_link(0)
        on_table = np.array([0.25, -0.05, M.TOP])
        for _ in range(int(M.DROP_S / DT) + 2):
            M.advance(m, DT, on_table, (0, 0, 0, 1), 0.0, 1.0, 0.0, hp, hq, (np.array([0.2, -0.2, 1.0]), M.OPEN_F))
        self.assertIn("dropped", m["fault"])
        m = fresh()
        inside = np.array([M.SHARPS[0], M.SHARPS[1], M.TOP + M.SH_WALL])
        self.assertTrue(M.in_sharps(inside))
        for _ in range(40):
            M.advance(m, DT, inside, (0, 0, 0, 1), 0.0, 1.0, 0.0, hp, hq, (np.array([0.2, -0.2, 1.0]), M.OPEN_F))
        self.assertIsNone(m["fault"])

    def test_grasp_across_the_width_is_held(self):
        """A gripper closed across the housing's 13 mm width holds it as one across its 10 mm thickness does: carried
        in the air for 2 s, neither is a drop. Open past the width, or closed on nothing, it is not held."""
        lp = np.array([0.20, -0.10, M.TOP + 0.10])
        hp, hq = hand_link(0)
        for gap in (M.L_T / 2 - 0.0003, M.L_W / 2 - 0.0003, M.L_W / 2 + 0.0004):
            m = fresh()
            for _ in range(int(2.0 / DT)):
                M.advance(m, DT, lp, (0, 0, 0, 1), 0.0, 1.0, 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)), -gap))
            self.assertIsNone(m["fault"], f"half-gap {1000 * gap:.1f} mm")
        g = M.grip_point(lp, (0, 0, 0, 1))
        for gap in (M.L_W / 2 + 0.001, 0.001):   # open past the width; closed on nothing (the demo's 1 mm command)
            self.assertFalse(M.held_by((g, -gap), lp, (0, 0, 0, 1)), f"half-gap {1000 * gap:.1f} mm")

    def test_out_of_the_stand_bounds(self):
        """Out of the stand: more than 4 mm across or 10 mm up from where it stood."""
        p0 = np.array(M.L_START)
        self.assertTrue(M.in_socket(p0 + [M.OUT_XY - 0.0005, 0, M.LIFT_OUT - 0.0005]))
        self.assertFalse(M.in_socket(p0 + [M.OUT_XY + 0.0005, 0, 0]))
        self.assertFalse(M.in_socket(p0 + [0, 0, M.LIFT_OUT + 0.0005]))

    def test_put_back_in_the_stand_and_let_go_is_a_drop(self):
        """Once out, the lancet stays out: put back in the stand and let go, it is a drop (task.json scoring)."""
        m = fresh()
        m["out"] = False
        hp, hq = hand_link(0)
        lp = np.array(M.L_START)
        M.advance(m, DT, lp + [0, 0, M.LIFT_OUT + 0.002], (0, 0, 0, 1), 0.0, 1.0, 0.0, hp, hq,
                  (M.grip_point(lp + [0, 0, M.LIFT_OUT + 0.002], (0, 0, 0, 1)), GRIP_READ))
        self.assertTrue(m["out"])
        for _ in range(int(M.DROP_S / DT) + 2):
            M.advance(m, DT, lp, (0, 0, 0, 1), 0.0, 1.0, 0.0, hp, hq, (np.array([0.2, -0.2, 1.0]), M.OPEN_F))
        self.assertIn("dropped", m["fault"])

    def test_press_needs_the_wrist_held(self):
        """Over the whole site the hand tips under less than the fire force: a press that fires needs the other hand."""
        for s in M.SITE_S:
            tip = W * (M.COM_X - M.PIVOT_X) / (M.PIVOT_X - s)
            self.assertLess(tip, 0.6 * M.F_FIRE)
        # and the demo's hold covers what the press needs at its peak (10 N [EST]) with margin
        need = (10.0 * (M.PIVOT_X - M.SITE_AIM) - W * (M.COM_X - M.PIVOT_X)) / (M.STUB_X - M.PIVOT_X)
        self.assertLess(need, 0.5 * K_ARM * M.HOLD_DZ)


class Scoring(unittest.TestCase):
    def test_model_must_step(self):
        M.reset_memory(0)
        M._CLOCK["steps"] = 10
        self.assertIn("not stepping", M.failure(Kin(0).objects(), M._M))

    def test_base_dragged_latches(self):
        M.reset_memory(0)
        o = Kin(0).objects()
        o["hand"] = st(o["hand"].pos + [M.BASE_MOVED + 0.001, 0, 0])
        self.assertIn("dragged", M.failure(o, M._M))

    def test_ladder_along_the_demo(self):
        M.reset_memory(trial_seed(2))
        right, left = M.plan(0.0, trial_seed(2))
        k = Kin(trial_seed(2))
        k.run(right, left)
        seen = []
        M.reset_memory(trial_seed(2))
        k = Kin(trial_seed(2))
        orig = k.step

        def rec():
            orig()
            M.update(k.trace[-1])
            seen.append(M._MEM["now"]["stage"])
        k.step = rec
        k.run(right, left)
        order = [s for i, s in enumerate(seen) if i == 0 or s != seen[i - 1]]
        self.assertEqual(order, [0, 1, 2, 3, 4])


class Demo(unittest.TestCase):
    def test_demo_succeeds_on_every_film_seed(self):
        for cli in CLI:
            sc, k = replay(trial_seed(cli))
            self.assertEqual(sc, 1.0, f"--seed {cli}: {M._MEM['fail']} stage {M._MEM['now']['stage']}")
            self.assertTrue(M._M["fired"])
            self.assertLess(M._M["fire"]["slide"], M.SLIDE_MAX)

    def test_demo_succeeds_at_every_layout_corner(self):
        for c in M.corners():
            sc, _ = replay(c)
            self.assertEqual(sc, 1.0, f"corner {c}: {M._MEM['fail']} stage {M._MEM['now']['stage']}")

    def test_demo_succeeds_on_many_seeds(self):
        for cli in list(range(N_SEEDS)) + list(SIM_SEEDS):
            sc, _ = replay(trial_seed(cli))
            self.assertEqual(sc, 1.0, f"--seed {cli}: {M._MEM['fail']} stage {M._MEM['now']['stage']}")

    def test_layout_is_continuous_and_fills_its_box(self):
        """Each seed draws its own layout: inside the bounds, all distinct, every octant of the box reached, each axis
        spread to within 5% of its bounds, and the same seed always gives the same layout."""
        lays = [M.shift_turn(trial_seed(c)) for c in range(2000)]
        lim = (M.SHIFT_MAX, M.SHIFT_MAX, M.TURN_MAX)
        for lay in lays:
            for v, b in zip(lay, lim):
                self.assertLessEqual(abs(v), b)
        self.assertEqual(len(set(lays)), len(lays))
        self.assertEqual(len({tuple(v > 0 for v in lay) for lay in lays}), 8)
        for i, b in enumerate(lim):
            self.assertGreater(max(lay[i] for lay in lays), 0.95 * b)
            self.assertLess(min(lay[i] for lay in lays), -0.95 * b)
        self.assertEqual(M.shift_turn(trial_seed(7)), M.shift_turn(trial_seed(7)))
        self.assertEqual(M.shift_turn(None), (0.0, 0.0, 0.0))

    def test_turn_is_about_the_palm(self):
        """The turn leaves the point TURN_C behind the apex where the shift alone puts it."""
        for c in M.corners():
            hx, hy, _, yaw = M.layout(c)["hand"]
            p = np.array([hx, hy]) + rz(yaw)[:2, :2] @ np.array([M.TURN_C, 0.0])
            np.testing.assert_allclose(p, np.array(M.HAND0) + np.array(c[:2]) + [M.TURN_C, 0.0], atol=1e-12)

    def test_open_loop_plan_fails_other_layouts(self):
        """A plan made for one seed's layout, replayed on another seed's, never reaches 1.0 (README Seeds)."""
        for a in range(N_PAIRS):
            b = a + N_PAIRS
            sc, _ = replay(trial_seed(a), plan_seed=trial_seed(b))
            self.assertLess(sc, 1.0, f"plan of --seed {b} replayed on --seed {a}")
        for c in M.corners():
            sc, _ = replay(c, plan_seed=(0.0, 0.0, 0.0))   # WHY a tuple: plan_seed=None means "the trial's own seed"
            self.assertLess(sc, 1.0, f"the unshifted plan replayed on corner {c}")

    def test_hand_clear_of_the_stand_and_the_container(self):
        """At every corner the trainer's base plate keeps 20 mm from the stand and the sharps container."""
        x0, x1 = M.PIVOT_X - 0.030, M.STUB_X + 0.040
        for c in M.corners():
            hx, hy, _, yaw = M.layout(c)["hand"]
            pts = [np.array([hx, hy]) + rz(yaw)[:2, :2] @ np.array([x, y]) for x in (x0, x1) for y in (-0.030, 0.030)]
            for (cx, cy), (hxx, hyy) in ((M.STAND, (M.STAND_HALF, M.STAND_HALF)),
                                         (M.SHARPS, (M.SH_L / 2, M.SH_W / 2))):
                # the plate's corners all lie beyond one of the fixture's faces (x or y), by 20 mm
                gaps = [(cx - hxx) - max(p[0] for p in pts), min(p[0] for p in pts) - (cx + hxx),
                        (cy - hyy) - max(p[1] for p in pts), min(p[1] for p in pts) - (cy + hyy)]
                self.assertGreater(max(gaps), 0.020, f"corner {c}")

    def test_demo_fits_the_run(self):
        r, lf = M.plan(0.05, 0)
        self.assertLess(max(sum(w.duration for w in r), sum(w.duration for w in lf)) + 2.0, M.RUN_LIMIT_STEPS / 20)


def _no_left(right, left):
    return right, [M._w(sum(w.duration for w in right), M.PARK_L, M.OPEN_F, "stay parked", 0.0)]


def _no_twist(right, left):
    return [M._w(w.duration, w.pos, w.finger, w.label, 0.0 if w.yaw == M.TWIST else w.yaw) for w in right], left


def _retarget(dx=0.0, dy=0.0):
    """Move the right arm's press waypoints by (dx, dy) in the hand's frame."""
    def edit(right, left):
        out = []
        press = {"over the fingertip", "steady over the fingertip", "just over the site", "wait for the wrist to be held",
                 "tip on the site", "press until it fires", "hold the press", "lift off", "up"}
        for w in right:
            p = w.pos
            if w.label in press:
                p = (p[0] + dx, p[1] + dy, p[2])
            out.append(M._w(w.duration, p, w.finger, w.label, w.yaw))
        return out, left
    return edit


def _drop_on_table(right, left):
    out = []
    for w in right:
        if w.label in ("over the sharps container", "steady over the opening", "let go", "up clear"):
            p = (0.25, -0.08, w.pos[2])
            out.append(M._w(w.duration, p, w.finger, w.label, w.yaw))
        else:
            out.append(w)
    return out, left


def _keep_holding(right, left):
    return [w if w.label not in ("let go", "up clear") else M._w(w.duration, w.pos, M.GRIP_L, w.label, w.yaw)
            for w in right], left


class LazyRoutes(unittest.TestCase):
    """Each shortcut a policy might try, and the score it gets (the README's Lazy routes table)."""
    S = trial_seed(2)

    def _run(self, edit):
        sc, _ = replay(self.S, edit=edit)
        return sc, M._MEM["fail"]

    def test_no_steadying_hand(self):
        sc, why = self._run(_no_left)
        self.assertEqual(sc, 0.0)
        self.assertIn("not steadied", why)

    def test_pull_without_twisting(self):
        sc, why = self._run(_no_twist)
        self.assertEqual(sc, 0.0)
        self.assertIn("tab still on", why)

    def test_press_on_the_little_finger(self):
        v = M.FINGERS["little"][0] - M.FINGERS["ring"][0]
        y, z = M._axis_yz(M.FINGERS["little"][0])
        du = M.FINGERS["little"][1] - M.FINGERS["ring"][1]
        rl = M.FINGERS["little"][3]
        right, left = M.plan(0.0, self.S)

        def edit(r, l_):
            out = []
            for w in r:
                p = w.pos
                if w.label in ("tip on the site", "press until it fires", "hold the press", "just over the site",
                               "wait for the wrist to be held", "over the fingertip", "steady over the fingertip"):
                    _, _, _, yaw = M.layout(self.S)["hand"]
                    off = rz(yaw) @ np.array([du, y, z + rl - M.FINGERS["ring"][3]])
                    p = (p[0] + off[0], p[1] + off[1], p[2] + off[2])
                out.append(M._w(w.duration, p, w.finger, w.label, w.yaw))
            return out, l_
        self.assertNotEqual(v, 0.0)
        sc, why = self._run(edit)
        self.assertEqual(sc, 0.0)
        self.assertIn("little finger", why)

    def test_press_on_the_pad_middle(self):
        """A vertical lancet on the pad's middle line meets skin turned 30 deg: off the side and not square."""
        _, _, _, yaw = M.layout(self.S)["hand"]
        r = M.FINGERS["ring"][3]
        top = M.hand_to_mech((M.FINGERS["ring"][1] + M.SITE_AIM, M.FINGERS["ring"][0], r))
        off = rz(yaw) @ np.array([0.0, top[1], 0.0])
        sc, why = self._run(_retarget(off[0], off[1]))
        self.assertEqual(sc, 0.0)
        self.assertTrue("middle of the ring's pad" in why or "tilted" in why, why)

    def test_press_past_the_pad(self):
        _, _, _, yaw = M.layout(self.S)["hand"]
        off = rz(yaw) @ np.array([0.030, 0.0, 0.0])
        sc, why = self._run(_retarget(off[0], off[1]))
        self.assertEqual(sc, 0.0)
        self.assertIn("not the fingertip", why)

    def test_drop_on_the_table(self):
        sc, why = self._run(_drop_on_table)
        self.assertEqual(sc, 0.0)
        self.assertIn("dropped", why)

    def test_keep_holding_over_the_container(self):
        sc, why = self._run(_keep_holding)
        self.assertEqual(sc, 0.6)
        self.assertIsNone(why)

    def test_one_arm_sim_stems(self):
        """sim/one_arm.py, appended by sim/mk.sh to the one-arm copies (o00, j00; both scored 0 in PhysX, README Sim
        results), keeps the left arm parked: in the replay both its press and its jab score 0, the hand not steadied."""
        src = (HERE / "sim" / "one_arm.py").read_text()
        for jab in (False, True):
            ns = dict(vars(M))
            exec(compile(src.replace("_JAB = False", f"_JAB = {jab}"), "one_arm.py", "exec"), ns)
            d = ns["demo"]()
            self.assertTrue(all(w.pos == M.PARK_L for w in d["left"]))
            press = [w for w in d["right"] if w.label == "press until it fires"][0]
            self.assertEqual(press.duration, ns["_JAB_S"] if jab else 1.5)
            M.reset_memory(None)
            k = Kin(None)
            self.assertEqual(k.run(d["right"], d["left"]), 0.0)
            self.assertIn("not steadied", M._MEM["fail"])

    def test_do_nothing(self):
        M.reset_memory(self.S)
        k = Kin(self.S)
        self.assertEqual(k.run([M._w(5.0, (0.323, -0.305, 1.008), M.OPEN_F, "idle", 0.0)],
                               [M._w(5.0, M.PARK_L, M.OPEN_F, "idle", 0.0)]), 0.0)


class Geometry(unittest.TestCase):
    def test_hand_clear_of_the_table(self):
        lo = min(s.bounds()[0][2] for s in M.HAND_SHAPES)
        self.assertGreater(M.HZ + lo, 0.010)

    def test_ring_ulnar_pad_faces_up(self):
        n = M.R_ROLL @ np.array([0.0, -math.sin(math.radians(30)), math.cos(math.radians(30))])
        self.assertAlmostEqual(float(n[2]), 1.0, places=9)

    def test_pads_clear_the_socket_rim_and_the_little_finger(self):
        self.assertGreater(M.PAD_TIP_L, M.CUP_D + 0.005)
        # at the press the pad tips are over the little finger's top and the housing's side is clear of it
        r = M.FINGERS["ring"][3]
        y, z = M._axis_yz(M.FINGERS["little"][0])
        little_top = z + M.FINGERS["little"][3]
        self.assertGreater(r + M.PAD_TIP_L, little_top + 0.005)
        self.assertGreater(abs(y) - M.FINGERS["little"][3] - M.L_W / 2, 0.002)
        self.assertGreater(M.FINGERS["little"][1] - M.FINGERS["ring"][1], M.SITE_S[1] + M.L_T / 2 + 0.003)

    def test_shield_reaches_skin_before_housing(self):
        """At the fire travel the housing's front still stands off the skin (the shield takes the whole press)."""
        q_fire = (M.F_FIRE - M.F_PRE) / M.K_SHIELD
        self.assertGreater(M.SHIELD_OUT - q_fire, 0.001)

    def test_lancet_fits_the_opening_and_clears_the_lid(self):
        self.assertLess(math.hypot(M.L_W, M.L_T), M.SH_OPEN[1] + 0.001)
        face = M.RELEASE_Z - M.tcp_over_lancet()
        self.assertGreater(face, M.TOP + M.SH_H + 0.005)

    def test_forces_under_the_finger_budget(self):
        """Every working force at or under 40 N a YAM finger, with margin (G16)."""
        grip = 5000.0 * (M.L_T / 2 + M.GRIP_L)  # N a finger: 4 mm into the housing at 5000 N/m
        self.assertLessEqual(M.F_FIRE_MAX_REAL * 2.5, 40.0)    # the press, through the housing: 2.7x under 40 N
        twist = M.T_TAB_MAX_REAL / M.L_W        # N: the couple on the housing's 13 mm faces at the twist limit
        self.assertLessEqual((grip + twist) * 1.6, 40.0)       # 23.8 N: 1.7x under 40 N
        # the 8 N press is 0.59 of the grip's friction, at the 0.34 the sim showed (it slid at 7.9 N on 11.7 N a finger)
        self.assertLessEqual(M.F_FIRE / (2 * 0.34 * grip), 0.65)


class ReadmeNumbers(unittest.TestCase):
    """The README's numbers come from the code."""
    TXT = " ".join((HERE / "README.md").read_text().split())

    def test_seed_table(self):
        """Every row of the README's seed table is the layout the code draws for that --seed."""
        rows = [ln for ln in (HERE / "README.md").read_text().splitlines() if ln.startswith("| ") and ln.count("|") == 5
                and ln.split("|")[1].strip().isdigit()]
        self.assertGreaterEqual(len(rows), len(CLI) + 8)
        for ln in rows:
            cli = int(ln.split("|")[1])
            dx, dy, yaw = M.shift_turn(trial_seed(cli))
            self.assertEqual(ln.split("|")[2:4], [f" {1000 * dx:.1f}, {1000 * dy:.1f} ", f" {math.degrees(yaw):.2f} "],
                             ln)
        for cli in CLI:
            self.assertTrue(any(ln.startswith(f"| {cli} |") for ln in rows))

    def test_model_numbers(self):
        t = self.TXT
        q_fire = (M.F_FIRE - M.F_PRE) / M.K_SHIELD
        for s in (f"fires at {M.F_FIRE:g} N", f"{1000 * q_fire:.2f} mm of travel",
                  f"breaks at {math.degrees(M.TAB_BREAK):.0f} deg", f"{100 * M.T_TAB:g} N-cm",
                  f"{1000 * M.SITE_S[0]:.0f} to {1000 * M.SITE_S[1]:.0f} mm back from the apex",
                  f"{math.degrees(M.SITE_ANG[0]):.0f} to {math.degrees(M.SITE_ANG[1]):.0f} deg",
                  f"within {math.degrees(M.TILT_MAX):.0f} deg", f"more than {1000 * M.SLIDE_MAX:.1f} mm",
                  f"wrist rises more than {1000 * M.WRIST_LIFT_MAX:.0f} mm", f"loose for {M.DROP_S:g} s",
                  f"{abs(math.degrees(M.ROLL)):.0f} deg", f"{M.HAND_KG:.3f} kg",
                  f"{W * (M.COM_X - M.PIVOT_X) / (M.PIVOT_X - M.SITE_S[1]):.1f} N",
                  f"{W * (M.COM_X - M.PIVOT_X) / (M.PIVOT_X - M.SITE_S[0]):.1f} N",
                  f"{1000 * M.BASE_MOVED:.0f} mm", f"{M.RUN_LIMIT_STEPS:,} steps",
                  f"more than {1000 * M.OUT_XY:.0f} mm across or {1000 * M.LIFT_OUT:.0f} mm up",
                  f"half-gap {1000 * M.HELD_GAP[0]:.1f} to {1000 * M.HELD_GAP[1]:.1f} mm",
                  f"{1000 * M.L_T:.0f} mm) to 0.5 mm outside its wider side ({1000 * M.L_W:.0f} mm)",
                  f"each uniform within +-{1000 * M.SHIFT_MAX:.0f} mm",
                  f"turned uniform within +-{math.degrees(M.TURN_MAX):.0f} deg about a point {1000 * M.TURN_C:.0f} mm",
                  f"({M.HAND0[0]:g}, {M.HAND0[1]:g})"):
            self.assertIn(s, t)

    def test_cradle_numbers(self):
        """The rig's sizes under Real trials are the model's: the hinge pin at the pivot, the wrist block as the stop."""
        t = self.TXT
        pin, top_w = M.HZ + M.REST_F_Z, M.HZ + M.REST_W_Z
        for s in (f"{1000 * M.PIVOT_X:.0f} mm behind the ring's apex and {1000 * pin:.1f} mm over the table",
                  f"its top {1000 * top_w:.1f} mm over the table, centred {1000 * M.STUB_X:.0f} mm behind the apex",
                  f"{1000 * M.STUB_YZ[0]:.0f} mm to the thumb side", f"{1000 * (M.STUB_X - M.PIVOT_X):.0f} mm behind the pin",
                  f"{1000 * (M.STUB_X + 0.040 - M.PIVOT_X + 0.030):.0f} by 60 by 5 mm base plate ({1000 * (M.PIVOT_X - 0.030):.0f} "
                  f"to", f"{1000 * (M.STUB_X + 0.040):.0f} mm behind the apex), ballasted to {M.BASE_KG:g} kg",
                  f"centre of mass {1000 * (M.COM_X - M.PIVOT_X):.0f} mm behind the pivot, a moment of "
                  f"{W * (M.COM_X - M.PIVOT_X):.3f} N m", f"{1000 * 2 * M.STUB_R:.0f} mm forearm stub",
                  f"{1000 * (M.L_W + 2 * M.CUP_CLEAR):.1f} by {1000 * (M.L_T + 2 * M.CUP_CLEAR):.1f} mm, "
                  f"{1000 * M.CUP_D:.0f} mm deep, {1000 * M.CUP_CLEAR:.1f} mm each side",
                  f"{1000 * (M.STAND_H + M.TAB_H):.0f} mm over the table, is where the shield's face rests",
                  f"rim {1000 * (M.STAND_H + M.TAB_H + M.CUP_D):.0f} mm over the table",
                  f"a {1000 * 2 * M.STAND_HALF:.0f} mm square block {1000 * M.STAND_H:.0f} mm high"):
            self.assertIn(s, t)
        self.assertGreater(M.STUB_YZ[0], 0.0)   # +y in the rolled frame is the thumb side

    def test_tipping_force_sits_with_its_place(self):
        """The README pairs each tipping force with where on the site it acts (smallest near the apex)."""
        near = W * (M.COM_X - M.PIVOT_X) / (M.PIVOT_X - M.SITE_S[0])
        far = W * (M.COM_X - M.PIVOT_X) / (M.PIVOT_X - M.SITE_S[1])
        self.assertLess(near, far)
        self.assertIn(f"{near:.1f} N ({1000 * M.SITE_S[0]:.0f} mm from the apex) to {far:.1f} N "
                      f"({1000 * M.SITE_S[1]:.0f} mm back", self.TXT)

    def test_instruction_is_outcomes_not_choreography(self):
        """Under 1000 characters, and no gripper assignment or grasp (the #826 rejection)."""
        ins = " ".join(M.__doc__.split())
        self.assertLess(len(ins), 1000)
        for w in ("other hand", "left", "right gripper", "your hand", "grip "):
            self.assertNotIn(w, ins.lower())
        self.assertEqual(json.loads((HERE / "task.json").read_text())["instruction"], ins)


def real_square_tilt(s, theta, r, lean_apex, lean_middle):
    """README Real trials, Square: the lancet's angle off the skin's normal (deg) from what the trial reads. s (mm back
    from the apex) and theta (deg round from the pad's middle line toward the side facing up) from the ink patch on the
    template; r the fingertip's radius (check 3b); lean_apex and lean_middle the housing top's lean (deg) toward the
    apex in the along-finger view and toward the pad's middle line in the across-finger view."""
    beta = math.asin((r - s) / r) if s < r else 0.0
    gamma = math.radians(30.0) - math.radians(theta)
    n = np.array([math.tan(beta) / math.cos(gamma), math.tan(gamma), 1.0])
    d = np.array([math.tan(math.radians(lean_apex)), math.tan(math.radians(lean_middle)), 1.0])
    return math.degrees(math.acos(float(n @ d) / (np.linalg.norm(n) * np.linalg.norm(d))))


class RealTrial(unittest.TestCase):
    def test_real_square_check_matches_the_scorer(self):
        """The README's two-camera squareness reading gives the scorer's own tilt (tilt_of against site_of's normal)
        over the whole site, both fingers, lancets leaning up to 25 deg any way."""
        rng = np.random.default_rng(3)
        for _ in range(400):
            name = ("ring", "middle")[int(rng.integers(2))]
            v, u0, _, r, _ = M.FINGERS[name]
            s = float(rng.uniform(*M.SITE_S))
            theta = float(rng.uniform(15.0, 45.0))
            rho = math.sqrt(r * r - (r - s) ** 2) if s < r else r
            a = -math.radians(theta)
            p = M.hand_to_mech((u0 + s, v + rho * math.sin(a), rho * math.cos(a)))
            site = M.site_of(p)
            self.assertEqual(site[0], name)
            self.assertAlmostEqual(site[4], 0.0, places=9)
            tip = math.radians(float(rng.uniform(0.0, 25.0)))
            az = float(rng.uniform(-math.pi, math.pi))
            d = np.array([math.sin(tip) * math.cos(az), math.sin(tip) * math.sin(az), math.cos(tip)])
            k = np.cross([0.0, 0.0, 1.0], d)
            ang = math.asin(min(1.0, float(np.linalg.norm(k))))
            q = (0.0, 0.0, 0.0, 1.0) if ang < 1e-12 else tuple(
                list(k / np.linalg.norm(k) * math.sin(ang / 2)) + [math.cos(ang / 2)])
            code = math.degrees(M.tilt_of(q, (0.0, 0.0, 0.0, 1.0), site[3]))
            lean_apex = math.degrees(math.atan2(-d[0], d[2]))      # the apex lies toward -x
            lean_middle = math.degrees(math.atan2(d[1], d[2]))     # the pad's middle line lies toward +y
            self.assertAlmostEqual(real_square_tilt(1000 * s, theta, 1000 * r, lean_apex, lean_middle), code, places=6)

    def test_rig_scad_is_current(self):
        """rig.scad is what tools/rig_scad.py writes from the task's numbers now."""
        import importlib.util
        spec = importlib.util.spec_from_file_location("rig_scad", HERE / "tools" / "rig_scad.py")
        rs = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(rs)
        txt = (HERE / "rig.scad").read_text()
        self.assertEqual(txt, rs.text(), "rerun python3 tools/rig_scad.py")
        for a, b in ("{}", "()", "[]"):
            self.assertEqual(txt.count(a), txt.count(b))
        n = rs.numbers()
        self.assertEqual(n["pin"], (round(1000 * M.PIVOT_X, 3), round(1000 * (M.HZ + M.REST_F_Z), 3)))
        self.assertEqual(n["wrist_top"], round(1000 * (M.HZ + M.REST_W_Z), 3))
        self.assertEqual(n["stub_lever"], round(1000 * (M.STUB_X - M.PIVOT_X), 3))
        # the shield's face rests on the post at the model's height, and the cup's rim is the model's socket wall
        face = round(1000 * (M.STAND_H + M.TAB_H), 3)
        self.assertAlmostEqual(face + 0.5, 1000 * (M.L_START[2] - M.TOP), places=6)
        self.assertIn(f"cylinder(d = {rs.POST_D}, h = {face})", txt)
        self.assertIn(f"cube([{round(1000 * (M.L_W + 2 * M.CUP_CLEAR), 3)}, {round(1000 * (M.L_T + 2 * M.CUP_CLEAR), 3)}",
                      txt)
        rim = face + round(1000 * M.CUP_D, 3)
        self.assertIn(f"h = {round(rim - (1000 * M.STAND_H - rs.CUP_SINK), 3)}", txt)
        for s in ("tools/rig_scad.py", "rig.scad", f"{n['pin'][1]:.1f} mm over the table"):
            self.assertIn(s, ReadmeNumbers.TXT)


if __name__ == "__main__":
    unittest.main()
