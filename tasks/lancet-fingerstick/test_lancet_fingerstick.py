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
DT = 0.05
K_ARM = 500.0                    # N/m at the TCP in z [EST, as the task's PRESS_DZ comment]
W = M.HAND_KG * 9.81


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
        right = (self.r[0].copy(), self.r[1])
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


def site_point(name, s, phi):
    """A point on a finger's distal pad (mechanism frame): s back from its apex, phi from the pad's middle line."""
    v, u0, _, rd, _ = M.FINGERS[name]
    return M.hand_to_mech((u0 + s, v + rd * math.sin(phi), rd * math.cos(phi)))


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
              right if right is not None else (tcp, M.GRIP_L), None)


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
        for (name, s, phi), why in cases:
            ok, msg = M.site_ok(M.site_of(site_point(name, s, phi)))
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
                                                                                  M.GRIP_L))
        self.assertIn("tab still on", M._M["fault"])

    def test_twisted_then_lifted_is_fine(self):
        M.reset_memory(0)
        hp, hq = hand_link(0)
        lp = np.array(M.L_START)
        M.advance(M._M, DT, lp, (0, 0, 0, 1), 0.0, M.TAB_BREAK + 0.01, 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)),
                                                                                    M.GRIP_L))
        lp = lp + [0, 0, M.LIFT_OUT + 0.002]
        M.advance(M._M, DT, lp, (0, 0, 0, 1), 0.0, 0.0, 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)), M.GRIP_L))
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

    def test_film_seeds_differ(self):
        self.assertEqual(sorted(M.layout_index(trial_seed(c)) for c in CLI), [0, 1, 2])

    def test_open_loop_plan_fails_other_layouts(self):
        for cli in CLI:
            for other in CLI:
                if other == cli:
                    continue
                sc, _ = replay(trial_seed(cli), plan_seed=trial_seed(other))
                self.assertLess(sc, 1.0, f"plan of --seed {other} replayed on --seed {cli}")

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
        for cli in CLI:
            i = M.layout_index(trial_seed(cli))
            dx, dy, yaw = M.LAYOUTS[i]
            self.assertIn(f"| {cli} | {i} | {1000 * dx:.0f}, {1000 * dy:.0f} | {math.degrees(yaw):.0f} |", self.TXT)

    def test_model_numbers(self):
        t = self.TXT
        q_fire = (M.F_FIRE - M.F_PRE) / M.K_SHIELD
        for s in (f"fires at {M.F_FIRE:g} N", f"{1000 * q_fire:.2f} mm of travel",
                  f"breaks at {math.degrees(M.TAB_BREAK):.0f} deg", f"{100 * M.T_TAB:g} N-cm",
                  f"{1000 * M.SITE_S[0]:.0f} to {1000 * M.SITE_S[1]:.0f} mm back from the apex",
                  f"{math.degrees(M.SITE_PHI[0]):.0f} to {math.degrees(M.SITE_PHI[1]):.0f} deg",
                  f"within {math.degrees(M.TILT_MAX):.0f} deg", f"more than {1000 * M.SLIDE_MAX:.1f} mm",
                  f"wrist rises more than {1000 * M.WRIST_LIFT_MAX:.0f} mm", f"loose for {M.DROP_S:g} s",
                  f"{abs(math.degrees(M.ROLL)):.0f} deg", f"{M.HAND_KG:.3f} kg",
                  f"{W * (M.COM_X - M.PIVOT_X) / (M.PIVOT_X - M.SITE_S[1]):.1f} N",
                  f"{W * (M.COM_X - M.PIVOT_X) / (M.PIVOT_X - M.SITE_S[0]):.1f} N",
                  f"{1000 * M.BASE_MOVED:.0f} mm", f"{M.RUN_LIMIT_STEPS:,} steps"):
            self.assertIn(s, t)


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


if __name__ == "__main__":
    unittest.main()
