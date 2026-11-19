"""Offline tests for lancet_fingerstick.py (no simulator): the site, the press and firing model, the scorer, the lazy
routes, and both arms' demo replayed through the model on each film seed by a kinematic stand-in for the sim (a gripped
lancet follows the TCP; the press is the arm's compliance in series with the shield's spring and the ring finger's
slide; an unheld finger gives way down its slide). Run with yam_arena on PYTHONPATH."""
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


def _tool(name):
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, HERE / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


YS = _tool("yieldsweep")
FIRE_MARGIN = 1.0                # N: every allowed press on the unheld finger at F_FIRE pushes it down by this much
#                                  more than it takes to give it DROP_MAX
CLI = (0, 2, 4)
N_SEEDS = 20                     # inspect-robots --seed 0..19 for the many-seed replay
N_PAIRS = 20                     # seed pairs (a, a + 20) for the open-loop replay
SIM_SEEDS = (26385, 84277, 5410, 55059, 126748, 5673, 58369, 100825)   # --seed nearest each layout corner (README)
DT = 0.05
K_ARM = 500.0                    # N/m at the TCP in z [EST, as the task's PRESS_DZ comment]
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


def ring_link(seed, drop=0.0):
    """The ring link's frame (world pos, quat): the mechanism's frame, lowered drop down the slide."""
    hx, hy, hz, yaw = M.layout(seed)["hand"]
    return np.array([hx, hy, hz - drop]), q_from_R(rz(yaw))


def hold_grip(seed):
    """The left TCP closed on the ring finger at the demo's hold point (world)."""
    g, _ = M.ring_world(seed, M.HOLD_AT)
    return g + [0, 0, M.HOLD_TCP]


GRIP_R_READ = -(M.FINGERS["ring"][4] - 0.0005)   # a finger closed on the ring finger reads it 0.5 mm in [EST]


class Kin:
    """A kinematic stand-in for the sim, both arms."""
    GRAB = 0.004

    def __init__(self, seed):
        self.seed = seed
        self.l = np.array(M.L_START, float)
        self.lyaw, self.tab, self.q, self.drop = 0.0, 0.0, 0.0, 0.0
        self.r = [np.array([0.323, -0.305, 1.008]), M.OPEN_F, 0.0]
        self.lf = [np.array(M.PARK_L, float), M.OPEN_F, 0.0]
        self.held, self.off = False, None
        self.trace = []

    def _lancet_quat(self):
        return q_from_R(rz(self.lyaw))

    def _left_on_finger(self):
        """(the left closed round the ring finger, its pull up on it (N), its TCP and reading as the sim gives them)."""
        p, f, _ = self.lf
        hp, hq = ring_link(self.seed, 0.0)
        loc = M._rot3(hq).T @ (p - hp)
        rp = M.FINGERS["ring"][4]
        on = (-f <= rp + 0.0005 and M.HOLD_SEG[0] <= loc[0] <= M.HOLD_SEG[1] and abs(loc[1]) <= 0.004
              and -0.003 <= loc[2] - M.HOLD_TCP <= M.HOLD_UP + 0.005)
        if not on:
            return False, 0.0, p.copy(), f
        tcp = hp + M._rot3(hq) @ np.array([loc[0], loc[1], M.HOLD_TCP])
        return True, K_ARM * max(0.0, float(loc[2]) - M.HOLD_TCP), tcp, min(f, GRIP_R_READ)

    def _press(self, cmd_face):
        """The shield's travel, the finger's drop and the housing's rise over the command, for a commanded face point:
        the arm (K_ARM), the shield and the finger's slide in series; a finger held by the left also has the left
        arm's stiffness and its pull behind it."""
        hp, hq = ring_link(self.seed, 0.0)
        depth = self._depth(cmd_face, hp, M._rot3(hq))
        if depth <= 0.0:
            return 0.0, 0.0, 0.0
        on, pull, _, _ = self._left_on_finger()
        k_fin = M.K_RING + (K_ARM if on else 0.0)
        pre = M.RING_PRELOAD + pull

        def parts(f):
            q = 0.0 if f <= M.F_PRE else min(M.SHIELD_TRAVEL, (f - M.F_PRE) / M.K_SHIELD)
            drop = min(M.DROP_STOP, max(0.0, (f - pre) / k_fin))
            return q, drop
        lo, hi = 0.0, 200.0
        for _ in range(60):
            f = (lo + hi) / 2
            q, drop = parts(f)
            lo, hi = (f, hi) if f / K_ARM + q + drop < depth else (lo, f)
        q, drop = parts(lo)
        return q, drop, depth - drop - q

    def _depth(self, face, hp, Rh):
        """How far (m) a commanded face point sits under the trainer's top surface straight above it (0 if none)."""
        def inside(z):
            p = np.array([face[0], face[1], z])
            s = M.site_of(Rh.T @ (p - hp))
            return s is not None and s[4] < 0.0 and "palm" not in s[0]
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
        self.q, self.drop = 0.0, 0.0
        if self.held:
            cmd = p - [0, 0, M.tcp_over_lancet()]
            self.lyaw = yaw + self.off
            if M.in_socket(cmd) and abs(self.tab) < M.TAB_STOP:
                if abs(self.lyaw) > abs(self.tab):
                    self.tab = max(-M.TAB_STOP, min(M.TAB_STOP, self.lyaw))
            if not M.in_socket(cmd):
                self.q, self.drop, back = self._press(cmd.copy())
                self.l = cmd + [0, 0, back]
            else:
                # in the stand the shield rests on the tab cap: a TCP commanded under where the lancet stands
                # compresses the shield (the arm's compliance in series), the housing stays where it stood
                depth = max(0.0, M.L_START[2] - cmd[2])
                f = (depth + M.F_PRE / M.K_SHIELD) / (1 / M.K_SHIELD + 1 / K_ARM) if depth > 0 else 0.0
                self.q = min(M.SHIELD_TRAVEL, max(0.0, (f - M.F_PRE) / M.K_SHIELD)) if f > M.F_PRE else 0.0
                self.l = np.array([cmd[0], cmd[1], max(cmd[2], M.L_START[2])])
        elif not M.in_socket(self.l) and not M.in_sharps(self.l) and self.l[2] > M.TOP + 0.0005:
            ox, oy = M.SH_OPEN[0] / 2, M.SH_OPEN[1] / 2
            if abs(self.l[0] - M.SHARPS[0]) <= ox and abs(self.l[1] - M.SHARPS[1]) <= oy:
                self.l = np.array([self.l[0], self.l[1], M.TOP + M.SH_WALL])
            else:
                self.l = np.array([self.l[0], self.l[1], M.TOP])
        hp, hq = ring_link(self.seed, self.drop)
        # a finger closed on the housing stops at it: report what the sim reads, not the command
        right = (self.r[0].copy(), min(self.r[1], GRIP_READ) if self.held else self.r[1])
        _, _, ltcp, lread = self._left_on_finger()
        left = (ltcp, lread)
        M.advance(M._M, DT, self.l.copy(), self._lancet_quat(), self.q, self.tab, -self.drop, hp, hq, right, left)
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


def press_at(m, p_mech, f, drop=0.0, l_quat=(0.0, 0.0, 0.0, 1.0), seed=0, right=None, held=True):
    """One model step with the shield's face at p_mech (ring frame at rest) under force f, the lancet held, the ring
    finger dropped `drop` down its slide and held by the left (held) or not."""
    hp, hq = ring_link(seed, drop)
    face = hp + M._rot3(hq) @ np.asarray(p_mech, float)
    q = max(0.0, (f - M.F_PRE) / M.K_SHIELD) if f > M.F_PRE else 0.0
    l_pos = face - M._rot3(l_quat) @ np.array([0.0, 0.0, q])
    tcp = M.grip_point(l_pos, l_quat)
    left = (hold_grip(seed) - [0, 0, drop], GRIP_R_READ) if held else None
    M.advance(m, DT, l_pos, l_quat, q, M.TAB_BREAK + 0.1, -drop, hp, hq,
              right if right is not None else (tcp, GRIP_READ), left)


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
        cases = [(("ring", 0.010, 0.0), "middle of the ring's pad"),
                 (("ring", 0.010, -math.radians(70)), "too far round"),
                 (("ring", 0.020, -0.5), "off the pad"),
                 (("ring", 0.006, -0.5), "off the pad"),          # under 8 mm: on the cap, not scored
                 (("index", 0.010, -0.5), "not the side of the ring fingertip"),
                 (("ring", 0.010, -0.5), None),
                 (("ring", 0.010, 0.5), "faces down")]          # BD shades it, but the task scores the side facing up
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
        hp, hq = ring_link(0)
        M.advance(M._M, DT, lp, (0, 0, 0, 1), 0.0, math.radians(30), 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)),
                                                                                  GRIP_READ))
        self.assertIn("tab still on", M._M["fault"])

    def test_twisted_then_lifted_is_fine(self):
        M.reset_memory(0)
        hp, hq = ring_link(0)
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

    def test_placed_needs_the_finger_held(self):
        """A touch on a good site is "placed" only with a gripper closed on the ring finger."""
        m = fresh()
        press_at(m, GOOD, M.F_TOUCH + 0.5, held=False)
        self.assertFalse(m["positioned"])
        press_at(m, GOOD, M.F_TOUCH + 0.5)
        self.assertTrue(m["positioned"])

    def test_fire_without_holding_the_finger_latches(self):
        """Fired with the finger still (say, a light finger under a fast jab) but no gripper on it: 0."""
        m = fresh()
        press_at(m, GOOD, 3.0, held=False)
        press_at(m, GOOD, 9.0, held=False)
        self.assertIn("no gripper holding the ring finger", m["fault"])

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

    def test_finger_that_gives_latches(self):
        m = fresh()
        press_at(m, GOOD, 3.0, drop=M.DROP_MAX + 0.001)
        press_at(m, GOOD, 9.0, drop=M.DROP_MAX + 0.001)
        self.assertIn("not held still", m["fault"])

    def test_jab_window_runs_past_the_click(self):
        """B1: a jab that fires before the finger moves, which then gives way within 0.5 s of the click, scores 0; the
        finger moving after the window (the lancet long lifted) does not count."""
        m = fresh()
        press_at(m, GOOD, 3.0)
        press_at(m, GOOD, 9.0)
        self.assertTrue(m["fired"])
        self.assertIsNone(m["fault"])
        for _ in range(int(0.4 / DT)):
            press_at(m, GOOD + [0, 0, 0.02], 0.0, drop=0.0)
        press_at(m, GOOD + [0, 0, 0.02], 0.0, drop=0.005)
        self.assertIn("within 0.5 s of the click", m["fault"])
        m = fresh()
        press_at(m, GOOD, 3.0)
        press_at(m, GOOD, 9.0)
        for _ in range(int(0.6 / DT)):
            press_at(m, GOOD + [0, 0, 0.02], 0.0)
        press_at(m, GOOD + [0, 0, 0.02], 0.0, drop=0.005)
        self.assertIsNone(m["fault"])

    def test_window_starts_at_first_touch_whatever_the_force(self):
        """The finger moved under a touch, before any firm press, counts too (the window starts at first touch)."""
        m = fresh()
        press_at(m, GOOD, M.F_TOUCH + 0.1)
        press_at(m, GOOD, 0.0, drop=0.004)    # lifted off, but the finger was pushed down meanwhile
        self.assertIn("not held still", m["fault"])

    def test_tab_breaks_either_way(self):
        for sign in (1, -1):
            M.reset_memory(0)
            hp, hq = ring_link(0)
            lp = np.array(M.L_START)
            M.advance(M._M, DT, lp, (0, 0, 0, 1), 0.0, sign * (M.TAB_BREAK + 0.01), 0.0, hp, hq,
                      (M.grip_point(lp, (0, 0, 0, 1)), GRIP_READ))
            self.assertTrue(M._M["tab_off"], sign)
            M.reset_memory(0)
            M.advance(M._M, DT, lp, (0, 0, 0, 1), 0.0, sign * (M.TAB_BREAK - 0.01), 0.0, hp, hq,
                      (M.grip_point(lp, (0, 0, 0, 1)), GRIP_READ))
            self.assertFalse(M._M["tab_off"], sign)

    def test_finger_held_rule(self):
        """Held: a gripper closed on the ring finger between its distal joint and its base. Closed on the index finger,
        over the palm, open, or closed on nothing, it is not."""
        hp, hq = ring_link(0)
        R = M._rot3(hq)
        on = lambda x, y=0.0, z=M.HOLD_TCP: hp + R @ np.array([x, y, z])   # noqa: E731
        self.assertTrue(M.finger_held((hold_grip(0), GRIP_R_READ), hp, hq))
        self.assertTrue(M.finger_held((on(0.040), GRIP_R_READ), hp, hq))
        iy, iz = M._axis_yz(M.FINGERS["index"][0])
        for tcp, gap, why in ((on(M.HOLD_AT, iy, iz + M.HOLD_TCP), -(M.FINGERS["index"][4] - 0.0005), "index"),
                              (on(0.140, 0.020, M.PALM_W + 0.005), GRIP_R_READ, "palm"),
                              (hold_grip(0), M.OPEN_R, "open"), (hold_grip(0), -0.001, "on nothing"),
                              (on(0.012), GRIP_R_READ, "on the pad, past the distal joint")):
            self.assertFalse(M.finger_held((tcp, gap), hp, hq), why)

    def test_square_is_against_the_local_normal_at_every_site(self):
        """The square rule reads the lancet against the skin's inward normal where it touches, at every allowed site
        (site_of's normal, not the vertical): a lancet along that normal is square everywhere, a vertical one is off by
        the normal's own lean, and leaning 0.5 deg past TILT_MAX off the normal is not square."""
        for s, a, p, n in YS.sites():
            site = M.site_of(p)
            self.assertEqual(site[0], "ring")
            if min([abs(abs(a) - b) for b in M.SITE_ANG] + [abs(s - b) for b in M.SITE_S]) > 1e-6:   # WHY: on a bound, rounding puts it either side
                self.assertEqual(M.site_ok(site), (True, ""), (s, a))
            down = np.array([0.0, 0.0, -1.0])
            lean = math.acos(float(n @ down))
            self.assertAlmostEqual(M.tilt_of((0, 0, 0, 1), (0, 0, 0, 1), site[3]), lean, places=9)
            for extra, square in ((0.0, True), (M.TILT_MAX + math.radians(0.5), False)):
                ax = np.cross(n, [0.0, 1.0, 0.0])
                ax /= np.linalg.norm(ax)
                c, sn = math.cos(extra), math.sin(extra)
                d = c * n + sn * np.cross(ax, n)       # n turned `extra` about ax
                k = np.cross(down, d)
                ang = math.acos(max(-1.0, min(1.0, float(down @ d))))
                q = (0.0, 0.0, 0.0, 1.0) if np.linalg.norm(k) < 1e-12 else tuple(
                    list(k / np.linalg.norm(k) * math.sin(ang / 2)) + [math.cos(ang / 2)])
                self.assertEqual(M.tilt_of(q, (0, 0, 0, 1), site[3]) <= M.TILT_MAX, square, (s, a, extra))

    def test_fire_off_the_hand_latches(self):
        m = fresh()
        p = np.array([-0.05, 0.0, -M.HZ])   # on the table before the fingertip
        press_at(m, p, 3.0)
        press_at(m, p, 9.0)
        self.assertIn("not on the hand", m["fault"])

    def test_drop_outside_latches_and_sharps_does_not(self):
        m = fresh()
        hp, hq = ring_link(0)
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
        hp, hq = ring_link(0)
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
        hp, hq = ring_link(0)
        lp = np.array(M.L_START)
        M.advance(m, DT, lp + [0, 0, M.LIFT_OUT + 0.002], (0, 0, 0, 1), 0.0, 1.0, 0.0, hp, hq,
                  (M.grip_point(lp + [0, 0, M.LIFT_OUT + 0.002], (0, 0, 0, 1)), GRIP_READ))
        self.assertTrue(m["out"])
        for _ in range(int(M.DROP_S / DT) + 2):
            M.advance(m, DT, lp, (0, 0, 0, 1), 0.0, 1.0, 0.0, hp, hq, (np.array([0.2, -0.2, 1.0]), M.OPEN_F))
        self.assertIn("dropped", m["fault"])

    def test_unheld_finger_gives_before_fire(self):
        """tools/yieldsweep.py over the whole site (the ring, 8 to 15 mm, 15 to 45 deg round on the side facing up),
        every lancet direction within the square rule's 10 deg of the local normal, and the contact force's direction
        set by friction 0 to 1.0 in either the shield's face or its rim contact: at F_FIRE every press pushes the
        unheld finger down by at least FIRE_MARGIN more than it takes to give it DROP_MAX, so a slow one-handed press
        never fires on a still finger (a jab is the B1 window's: test_jab_window_runs_past_the_click)."""
        for mu in YS.MUS:
            k, where = YS.sweep(mu)
            self.assertGreaterEqual(M.F_FIRE * k - YS.give_force(), FIRE_MARGIN, f"mu {mu} at {where}")
        # and the demo's hold covers what its press needs at its peak (10 N [EST]) with margin
        self.assertGreater(M.RING_PRELOAD + K_ARM * M.HOLD_UP, 10.0 + 2.0)

    def test_the_sweep_can_fail(self):
        """The check can fail: a spring five times as strong (5 N to spare) would hold the finger still under some
        allowed presses at 8 N, with friction 1.0."""
        k, _ = YS.sweep(1.0)
        self.assertLess(M.F_FIRE * k - (5.0 + M.K_RING * M.DROP_MAX), 0.0)

    def test_the_side_facing_down(self):
        """WHY the site is the side facing up only: there the skin's normal is 45 to 75 deg from vertical, past what a
        top-down gripper's lancet can meet square (this test), and friction can even lift the finger (k under 0)."""
        for s, a, p, n in YS.sites(side=1):
            self.assertGreater(math.degrees(math.acos(float(-n[2]))), 44.9)
        k, _ = YS.sweep(1.0, side=1)
        self.assertLess(k, 0.0)

    def test_fired_in_the_stand_latches(self):
        """Tab off and still in the stand, a press of F_FIRE on the shield fires it into the post: 0."""
        M.reset_memory(0)
        lp = np.array(M.L_START)
        hp, hq = ring_link(0)
        q = (M.F_FIRE + 0.1 - M.F_PRE) / M.K_SHIELD
        M.advance(M._M, DT, lp, (0, 0, 0, 1), q, M.TAB_BREAK + 0.1, 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)),
                                                                                   GRIP_READ))
        self.assertTrue(M._M["fired"])
        self.assertIn("fired in the stand", M._M["fault"])
        M.reset_memory(0)   # tab still on: it does not fire
        M.advance(M._M, DT, lp, (0, 0, 0, 1), q, 0.1, 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)), GRIP_READ))
        self.assertFalse(M._M["fired"])
        self.assertIsNone(M._M["fault"])

    def test_slipped_or_deep_grasp_in_the_container_is_still_held(self):
        """A gripper closed on the housing with its TCP 25 mm off the demo's grip point (slipped along the pads, or
        taken deep or high) still holds it: lowered into the container and kept there it scores 0.6, not 0.8. The
        old rule (TCP within 20 mm of the grip point) read it as let go."""
        inside = np.array([M.SHARPS[0], M.SHARPS[1], M.TOP + 0.150])
        q0 = (0.0, 0.0, 0.0, 1.0)
        g = M.grip_point(inside, q0)
        for off in (np.array([0.0, 0.0, -0.025]), np.array([0.0, 0.0, 0.020]), np.array([0.009, 0.0, -0.018])):
            tcp = g + off
            self.assertGreater(np.linalg.norm(tcp - g), 0.020)   # the old rule: let go
            m = fresh()
            m["fired"] = True
            hp, hq = ring_link(0)
            for _ in range(int(2.0 / DT)):
                M.advance(m, DT, inside, q0, 0.0, 1.0, 0.0, hp, hq, (tcp, GRIP_READ), None)
                M._CLOCK["dt"] = DT
                M._CLOCK["steps"] += 1
                m["steps"] += 1
            objs = Kin(0).objects()
            objs["lancet"] = st(inside, q0)
            self.assertTrue(M.held_by((tcp, GRIP_READ), inside, q0))
            self.assertEqual(M.stage(objs, m), 3, off)
        # closed tight on nothing, beside the lancet: not let go either; open, it is
        self.assertFalse(M.let_go((g + [0.0, 0.0, -0.030], -0.001), inside, q0))
        self.assertTrue(M.let_go((g, M.OPEN_F), inside, q0))
        self.assertTrue(M.let_go((g + [0.0, 0.0, 0.10], -0.001), inside, q0))


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
            u = M.TURN_C * np.array([math.cos(M.HAND_YAW), math.sin(M.HAND_YAW)])
            np.testing.assert_allclose(p, np.array(M.HAND0) + np.array(c[:2]) + u, atol=1e-12)

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
        for c in M.corners():
            hx, hy, _, yaw = M.layout(c)["hand"]
            # the plate's corners, and the index fingertip and the ring's apex (they overhang the plate's front)
            pts = [np.array([hx, hy]) + rz(yaw)[:2, :2] @ np.array([x, y]) for x in M.PLATE_X for y in M.PLATE_Y]
            pts += [np.array([hx, hy]) + rz(yaw)[:2, :2] @ np.array([b[0], b[1]])
                    for sh in M.HAND_SHAPES + M.RING_SHAPES for b in sh.bounds()]
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
        press = {"over the fingertip", "steady over the fingertip", "just over the site", "steady just over the site",
                 "tip on the site", "press until it fires", "hold the press", "lift off"}
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

    def test_no_holding_hand(self):
        sc, why = self._run(_no_left)
        self.assertEqual(sc, 0.0)
        self.assertIn("not held still", why)

    def test_pull_without_twisting(self):
        sc, why = self._run(_no_twist)
        self.assertEqual(sc, 0.0)
        self.assertIn("tab still on", why)

    def test_press_on_the_index_fingertip(self):
        """The same press moved onto the index fingertip's top (fixed to the base, so it does not give): 0."""
        y, z = M._axis_yz(M.FINGERS["index"][0])
        du = M.FINGERS["index"][1] - M.FINGERS["ring"][1]
        d_r = M.FINGERS["index"][3] - M.FINGERS["ring"][3]
        _, _, _, yaw = M.layout(self.S)["hand"]
        off = rz(yaw) @ np.array([du, y, z + d_r])
        moved = {"tip on the site", "press until it fires", "hold the press", "just over the site",
                 "steady just over the site", "over the fingertip", "steady over the fingertip"}

        def edit(r, l_):
            return [M._w(w.duration, np.asarray(w.pos) + off if w.label in moved else w.pos, w.finger, w.label, w.yaw)
                    for w in r], l_
        sc, why = self._run(edit)
        self.assertEqual(sc, 0.0)
        self.assertIn("not the side of the ring fingertip", why)

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

    def test_sim_stems(self):
        """sim/stems.py, appended by sim/mk.sh to the stem copies (README Sim results), replayed offline on --seed 0's
        layout: each scores what the README expects, for the reason it gives."""
        src = (HERE / "sim" / "stems.py").read_text()
        want = {"o": (0.0, ("not held still",)), "j": (0.0, ("not held still",)),
                "x": (0.0, ("not held still", "no gripper holding the ring finger")),
                "w": (0.0, ("not held still", "slid")), "f": (0.0, ("fired in the stand",)), "k": (0.6, (None,)),
                "c": (1.0, (None,)), "i": (0.0, ("not held still", "no gripper holding the ring finger"))}
        seed = trial_seed(0)
        for stem, (score, why) in want.items():
            ns = dict(vars(M))
            ns["run_seed"] = lambda: seed
            exec(compile(src.replace('_STEM = "none"', f'_STEM = "{stem}"'), "stems.py", "exec"), ns)
            d = ns["demo"]()
            if stem in "ojxw":
                self.assertTrue(all(w.pos == M.PARK_L for w in d["left"]))
            press = [w for w in d["right"] if w.label == "press until it fires"][0]
            tip = [w for w in d["right"] if w.label == "tip on the site"][0]
            if stem == "j":
                self.assertEqual(press.duration, ns["_JAB_S"])
            if stem == "x":   # 30 mm under contact in one control step
                self.assertEqual(press.duration, DT)
                self.assertAlmostEqual(tip.pos[2] - press.pos[2], 0.030, places=9)
            if stem == "c":
                self.assertTrue(any(w.yaw == -M.TWIST for w in d["right"]))
            M.reset_memory(seed)
            k = Kin(seed)
            got = k.run(d["right"], d["left"])
            self.assertEqual(got, score, f"stem {stem}: {M._MEM['fail']}")
            fail = M._MEM["fail"]
            self.assertTrue(any((w is None and fail is None) or (w and fail and w in fail) for w in why),
                            f"stem {stem}: {fail}")
            if stem == "w":   # pressed 8.5 mm back, on the site, driven toward the knuckle
                f = [w for w in d["right"] if w.label == "press until it fires"][0]
                t = [w for w in d["right"] if w.label == "tip on the site"][0]
                yaw = M.layout(seed)["hand"][3]
                along = (np.asarray(f.pos) - np.asarray(t.pos))[:2] @ np.array([math.cos(yaw), math.sin(yaw)])
                self.assertAlmostEqual(along, M.PRESS_DZ * math.tan(ns["_TILT"]), places=6)
            if stem == "c":
                self.assertLess(M._M["tab"], -M.TAB_BREAK)
            if stem == "k":
                self.assertTrue(M.in_sharps(k.l))
                self.assertTrue(M.held_by(M._M["right"], k.l, k._lancet_quat()))

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

    def test_pads_clear_the_socket_rim(self):
        self.assertGreater(M.PAD_TIP_L, M.CUP_D + 0.005)

    def test_slide_parts_fit(self):
        """The carriage stays 2 mm or more inside the holding pads (closed, 7.3 mm off the finger's axis), ends 3 mm
        before the palm block, and lands on the guide block after DROP_STOP; the hold lies on the finger in front of it."""
        self.assertGreaterEqual(-GRIP_R_READ - M.CAR_HALF_Y, 0.002)
        self.assertGreaterEqual(M.hand_to_mech((M.PALM_U[0], M.FINGERS["ring"][0], 0.0))[0] - M.CAR_X[1], 0.002)
        self.assertAlmostEqual(M.CAR_Z[0] - M.GUIDE_TOP, M.DROP_STOP, places=12)
        self.assertLess(M.CAR_Z[1], 0.0)
        self.assertTrue(M.CAR_X[0] < M.PIN_X[0] < M.PIN_X[1] < M.CAR_X[1])
        self.assertTrue(M.HOLD_SEG[0] < M.HOLD_AT < M.HOLD_SEG[1])

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
        self.assertLessEqual(M.F_FIRE_MAX_REAL * 2.5, 40.0)
        twist = M.T_TAB_MAX_REAL / M.L_W
        self.assertLessEqual((grip + twist) * 1.6, 40.0)
        self.assertLessEqual(M.F_FIRE / (2 * 0.34 * grip), 0.65)
        hold = 5000.0 * (M.FINGERS["ring"][4] + M.GRIP_R)   # the left's 5 mm into the ring finger
        self.assertLessEqual(hold * 1.5, 40.0)


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
        rv, ru0, ru1, rd, rp = M.FINGERS["ring"]
        for s in (f"fires at {M.F_FIRE:g} N on the shield, {1000 * q_fire:.2f} mm of travel",
                  f"breaks at {math.degrees(M.TAB_BREAK):.0f} deg of turn either way",
                  f"up to {math.degrees(M.TAB_STOP):.0f} deg either way", f"{100 * M.T_TAB:g} N-cm",
                  f"{1000 * M.SITE_S[0]:.0f} to {1000 * M.SITE_S[1]:.0f} mm back from the apex and "
                  f"{math.degrees(M.SITE_ANG[0]):.0f} to {math.degrees(M.SITE_ANG[1])
                                                      :.0f} deg round from the pad's middle line",
                  f"within {math.degrees(M.TILT_MAX):.0f} deg of the skin's inward normal",
                  f"more than {1000 * M.SLIDE_MAX:.1f} mm on the skin is a slide",
                  f"pushed more than {1000 * M.DROP_MAX:.0f} mm down its slide",
                  f"until {M.STEADY_AFTER:g} s after the click", f"loose for {M.DROP_S:g} s",
                  f"rolled {abs(math.degrees(M.ROLL)):.0f} deg", f"the ring finger ({1000 * M.RING_KG:.0f} g)",
                  f"{1000 * M.KNUCKLE_X:.0f} mm behind its apex", f"A spring ({M.K_RING:g} N/m)",
                  f"with {M.RING_PRELOAD:g} N to spare over its weight; it stops {1000 * M.DROP_STOP:.0f} mm down",
                  f"gives {1000 * M.DROP_MAX:.0f} mm at {YS.give_force():.2f} N straight down",
                  f"half-gap {1000 * M.HOLD_GAP[0]:.1f} to {1000 * M.HOLD_GAP[1]:.1f} mm and its TCP within "
                  f"{1000 * M.HOLD_R:.0f} mm of the ring finger's axis between its distal joint "
                  f"({1000 * M.HOLD_SEG[0]:.1f} mm back from the apex) and its base ({1000 * M.HOLD_SEG[1]:.1f} mm back)",
                  f"{1000 * M.HOLD_AT:.1f} mm back from its apex, {1000 * (M.HOLD_AT - M.SITE_AIM):.1f} mm behind the press",
                  f"aims {1000 * M.HOLD_UP:.0f} mm over the finger",
                  f"pads {1000 * rp:.1f} to 14.2 mm either side",
                  f"{1000 * M.SITE_AIM:g} mm back from the ring's apex",
                  f"within {1000 * M.HELD_R:.0f} mm of the lancet's axis, anywhere from {-1000 * M.HELD_AX[0]:.0f} mm "
                  f"beyond its face to {1000 * M.HELD_AX[1]:.0f} mm up it",
                  f"half-gap over {1000 * M.HELD_GAP[1]:.1f} mm, or its TCP more than {1000 * M.RELEASE_R:.0f} mm",
                  f"half-gap {1000 * M.HELD_GAP[0]:.1f} to {1000 * M.HELD_GAP[1]:.1f} mm",
                  f"{1000 * M.BASE_MOVED:.0f} mm; more is a fault", f"{M.RUN_LIMIT_STEPS:,} steps",
                  f"more than {1000 * M.OUT_XY:.0f} mm across or {1000 * M.LIFT_OUT:.0f} mm up",
                  f"each uniform within +-{1000 * M.SHIFT_MAX:.0f} mm",
                  f"turned uniform within +-{math.degrees(M.TURN_MAX):.0f} deg about a point {1000 * M.TURN_C:.0f} mm",
                  f"({M.HAND0[0]:g}, {M.HAND0[1]:g})", f"waits clear at ({M.WAIT_R[0]:.2f}, {M.WAIT_R[1]:.2f})",
                  f"{abs(math.degrees(M.HAND_YAW)):.0f} deg right of +x"):
            self.assertIn(s, t)
        self.assertAlmostEqual(rp, 0.0078)

    def test_run_numbers(self):
        """The plans' lengths the README gives, and the run limit's headroom over the slowest (a stem)."""
        r, lf = M.plan(0.0, trial_seed(0))
        a, b = sum(w.duration for w in r), sum(w.duration for w in lf)
        self.assertIn(f"take {a:.1f} s (right) and {b:.1f} s (left)", self.TXT)
        ik = _tool("ikcheck")
        slow = max((max(sum(w.duration for w in x) for x in ik.stem_plan(k, trial_seed(0))), k) for k in ik.STEMS)
        self.assertIn(f"the slowest stem, {slow[1]}00, {slow[0]:.1f} s", self.TXT)
        self.assertGreater(M.RUN_LIMIT_STEPS / 20, slow[0] + 15.0)
        for f in ("film.sh", "dream.sh"):
            self.assertIn(f"--max-steps {M.RUN_LIMIT_STEPS}", (HERE / "sim" / f).read_text())
        self.assertIn(f"--max-steps {M.RUN_LIMIT_STEPS}", self.TXT)

    def test_yield_table(self):
        """The README's yield table and the sweep's other numbers are the sweep's."""
        t = self.TXT
        g = YS.give_force()
        for mu, txt in zip(YS.MUS, ("0", "0.25", "0.5", "1.0")):
            k, w = YS.sweep(mu)
            self.assertIn(f"| {txt} | {k:.4f} | {g / k:.2f} | {M.F_FIRE * k - g:+.2f} N |", t)
        k, w = YS.sweep(1.0)
        self.assertEqual(w, (8.0, -15.0))
        self.assertIn("The worst site is 8.0 mm back, 15 deg round", t)
        self.assertIn(f"would be {YS.sweep(1.0, tilt=math.radians(15))[0]:.4f}", t)
        self.assertIn(f"the least k is {YS.sweep(1.0, side=1)[0]:.3f}", t)
        self.assertIn(f"gives the unheld finger way by {g / k:.2f} N or less", t)

    def test_real_check_numbers(self):
        """Checks 1 and R and the touch reading come from the sweep."""
        t = self.TXT
        g = YS.give_force()
        k, _ = YS.sweep(1.0)
        f_min = math.ceil(10 * (g + FIRE_MARGIN) / k) / 10
        self.assertIn(f"the lowest at least {f_min:.1f} N", t)
        self.assertIn(f"{k:.4f}, times {f_min:.1f} N, less {g:.2f} N)", t)
        self.assertIn(f"1.0 to 1.4 N (the sim's {g:.2f} N plus or minus 0.2 N)", t)
        self.assertIn(f"by {M.F_FIRE * k - 1.4:.2f} N at 8 N ({k:.4f} x 8 - 1.4)", t)
        lo, hi = YS.k_range(1.0)
        self.assertIn(f"{lo:.2f} to {hi:.2f} times it", t)
        self.assertIn(f"{round(1000 * M.F_TOUCH / M.GRAVITY):.0f} g ({M.F_TOUCH:.1f} N)", t)

    def test_rig_numbers(self):
        """The rig's sizes under Real trials are rig_scad.numbers(), which come from the model."""
        t = self.TXT
        n = _tool("rig_scad").numbers()
        cx0, cx1, cw, cb, ct = n["carriage"]
        gx0, gx1, gw, gt, gu = n["guide"]
        sp, pin = n["spring"], n["pin"]
        x0, x1, y0, y1, _ = n["plate"]
        bx, by, bz = n["base_size"]
        for s in (f"a plate {x1 - x0:g} by {y1 - y0:g} by 5 mm (x {x0:g} to {x1:g})",
                  f"{bx} by {by} by {bz} mm, so it needs a bed of at least {bx + 10} by {by + 10} mm and {bz + 5} mm",
                  f"{gx1 - gx0:g} by {gw:g} mm (x {gx0:g} to {gx1:g}), its top at z {gt:g}",
                  f"(z {gu:g} to {gt:g}) carries the pins'", f"a carriage {cx1 - cx0:g} by {cw:g} mm, z {cb:g} to {ct:g}",
                  f"{pin['length']:g} mm long, are pressed 8 mm into the carriage at x {n['pins_x'][0]:g} and "
                  f"{n['pins_x'][1]:g}", f"top after {n['drop_stop']:g} mm: the lower stop",
                  f"each {sp['rate_each']:g} N/mm, free length {sp['free']:.1f} mm, {sp['rest']:g} mm long at rest "
                  f"({sp['force_rest_each']:.3f} N each, {2 * sp['force_rest_each']:.2f} N in all",
                  f"{sp['bottom']:g} mm at the lower stop", f"axis {n['ring_axis_z']:g} mm over the table",
                  f"{n['cup_pocket'][0]:g} by {n['cup_pocket'][1]:g}, {n['cup_pocket'][2]:g} deep",
                  f"stops it at +-{n['tab_stop_deg']:g} deg"):
            self.assertIn(s, t)
        self.assertAlmostEqual(2 * sp["force_rest_each"], M.RING_PRELOAD + M.RING_KG * M.GRAVITY, places=2)
        self.assertGreater(sp["bottom"], 0.0)
        self.assertGreater(pin["bottom"] - 1000 * M.DROP_STOP, n["seat_top"])   # the pins clear the seats at the stop

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
    template; r the fingertip's radius (check 3b); lean_apex and lean_middle the housing top's lean (deg, signed)
    toward the apex, seen by the camera looking across the fingers, and toward the pad's middle line, seen by the
    camera looking along them."""
    beta = math.asin((r - s) / r) if s < r else 0.0
    gamma = math.radians(30.0) - math.radians(theta)
    n = np.array([math.tan(beta) / math.cos(gamma), math.tan(gamma), 1.0])
    d = np.array([math.tan(math.radians(lean_apex)), math.tan(math.radians(lean_middle)), 1.0])
    return math.degrees(math.acos(float(n @ d) / (np.linalg.norm(n) * np.linalg.norm(d))))


class RealTrial(unittest.TestCase):
    def test_real_square_check_matches_the_scorer(self):
        """The README's two-camera squareness reading gives the scorer's own tilt (tilt_of against site_of's normal)
        over the whole site, lancets leaning up to 25 deg any way."""
        rng = np.random.default_rng(3)
        for _ in range(400):
            name = "ring"
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

    def test_real_square_check_both_sides_of_theta(self):
        """g = 30 - theta is signed: near theta 15 and 45 deg (the normal leaning toward, then away from, the pad's
        middle line) and with leans of both signs, the reading equals the scorer's tilt; a vertical lancet reads
        |30 - theta| at both."""
        for name in ("ring",):
            v, u0, _, r, _ = M.FINGERS[name]
            for theta in (15.5, 44.5):   # WHY not 15 and 45 exactly: rounding puts a site on the bound either way
                p = M.hand_to_mech((u0 + 0.011, v + r * math.sin(-math.radians(theta)), r * math.cos(math.radians(theta))))
                site = M.site_of(p)
                self.assertEqual(M.site_ok(site), (True, ""))
                for la, lm in ((0.0, 0.0), (8.0, -12.0), (-8.0, 12.0), (0.0, 15.0), (0.0, -15.0)):
                    d = np.array([-math.tan(math.radians(la)), math.tan(math.radians(lm)), 1.0])
                    d /= np.linalg.norm(d)
                    k = np.cross([0.0, 0.0, 1.0], d)
                    ang = math.asin(min(1.0, float(np.linalg.norm(k))))
                    q = (0.0, 0.0, 0.0, 1.0) if ang < 1e-12 else tuple(
                        list(k / np.linalg.norm(k) * math.sin(ang / 2)) + [math.cos(ang / 2)])
                    code = math.degrees(M.tilt_of(q, (0.0, 0.0, 0.0, 1.0), site[3]))
                    self.assertAlmostEqual(real_square_tilt(11.0, theta, 1000 * r, la, lm), code, places=6)
                    if la == lm == 0.0:
                        self.assertAlmostEqual(code, abs(30.0 - theta), places=6)

    def test_rig_scad_is_current(self):
        """rig.scad is what tools/rig_scad.py writes from the task's numbers now, and its key sizes are the model's."""
        rs = _tool("rig_scad")
        txt = (HERE / "rig.scad").read_text()
        self.assertEqual(txt, rs.text(), "rerun python3 tools/rig_scad.py")
        for a, b in ("{}", "()", "[]"):
            self.assertEqual(txt.count(a), txt.count(b))
        n = rs.numbers()
        cx0, cx1, cw, cb, ct = n["carriage"]
        self.assertIn(f"translate([{cx0}, {-cw / 2}, {cb}]) cube([{round(cx1 - cx0, 3)}, {cw}, {round(ct - cb, 3)}])", txt)
        self.assertEqual(n["drop_stop"], round(1000 * M.DROP_STOP, 3))
        self.assertIn(f"capsule([26.6, 0.0, {n['ring_axis_z']}], [99.0, 0.0, {n['ring_axis_z']}], 7.8);", txt)
        # the shield's face rests on the post at the model's height, and the cup's pocket is the model's socket
        face = round(1000 * (M.STAND_H + M.TAB_H), 3)
        self.assertAlmostEqual(face + 0.5, 1000 * (M.L_START[2] - M.TOP), places=6)
        self.assertIn(f"cylinder(d = {rs.POST_D}, h = {face})", txt)
        self.assertIn(f"cube([{round(1000 * (M.L_W + 2 * M.CUP_CLEAR), 3)}, {round(1000 * (M.L_T + 2 * M.CUP_CLEAR), 3)}",
                      txt)
        self.assertIn(f"rotate_extrude(angle = {2 * math.degrees(M.TAB_STOP):g})", txt)
        # the springs: two in parallel are the model's K_RING, and at rest they carry the preload plus the weight
        sp = n["spring"]
        self.assertAlmostEqual(2 * sp["rate_each"] * 1000, M.K_RING, places=6)
        self.assertAlmostEqual(sp["comp"], 1000 * M.ring_target(), places=3)
        for s in ("tools/rig_scad.py", "rig.scad", "OpenSCAD is not installed"):
            self.assertIn(s, ReadmeNumbers.TXT)


if __name__ == "__main__":
    unittest.main()
