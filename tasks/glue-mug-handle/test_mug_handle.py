"""Offline tests for mug_handle.py (no simulator): the glue model, the seating, the scorer, and the demo replayed through
the model on each film seed by a kinematic stand-in for the sim (a gripped part follows the TCP; a handle cannot pass
into the faces). Run with yam_arena on PYTHONPATH."""
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

M = load(HERE / "mug_handle.py").module
CLI = (0, 2, 4)
IDENT = (0.0, 0.0, 0.0, 1.0)
DT = 0.05
FAR = (np.array([0.0, 0.40, 1.10]), -0.04)   # a hand far off and open


def trial_seed(cli):
    return zlib.crc32(f"{cli}:None:0".encode())


def q_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def st(pos, quat=IDENT, v=(0.0, 0.0, 0.0)):
    return types.SimpleNamespace(pos=np.asarray(pos, float), quat_xyzw=np.asarray(quat, float),
                                 lin_vel=np.asarray(v, float))


def mug_pose(seed):
    x, y, z, _ = M.layout(seed)["mug"]
    return np.array([x, y, z]), IDENT


def seated_handle(seed):
    """The handle's frame with both broken ends on their faces."""
    p, q = mug_pose(seed)
    return np.array([p[0], p[1], p[2] + M.R_MUG + M.STUB_H]), q_yaw(0.0)


class Kin:
    """A kinematic stand-in for the sim: the TCP and finger move linearly to each waypoint; a part closed on by the
    fingers (the TCP within GRAB of its grip point, the jaw square to it within 10 deg) follows the TCP; the handle
    stops on the faces or the pad; the tube's walls give the squeeze of the finger target."""
    GRAB = 0.004

    def __init__(self, seed):
        self.seed = seed
        lay = M.layout(seed)
        hx, hy, hz, hyaw = lay["handle"]
        self.h = [np.array([hx, hy, hz - 0.0005]), hyaw]
        self.t = np.array(M.TUBE_START) - np.array([0.0, 0.0, 0.0005])
        self.mug = mug_pose(seed)
        self.tcp, self.f, self.yaw = np.array([0.2, -0.1, M.TOP + 0.25]), M.OPEN_F, M.YAW0
        self.held, self.off = None, None
        self.left = FAR
        self.trace, self.holds = [], []

    def _grip_h(self):
        return self.h[0] + np.array([0.0, 0.0, M.TCP_OVER_H])

    def _grip_t(self):
        return self.t + np.array([0.0, 0.0, M.TCP_OVER_T])

    def _closed_on(self, half):
        return -self.f < half

    def _angle_ok(self, part_yaw):
        d = (self.yaw - part_yaw - M.YAW0 + math.pi / 2) % math.pi - math.pi / 2
        return abs(d) <= math.radians(10)

    def step(self):
        if self.held is None:
            if self._closed_on(M.SEC_W / 2) and np.linalg.norm(self.tcp - self._grip_h()) <= self.GRAB \
                    and self._angle_ok(self.h[1]):
                self.held, self.off = "handle", (self._grip_h() - self.tcp, self.h[1] - self.yaw)
            elif self._closed_on(M.TUBE_W / 2) and np.linalg.norm(self.tcp - self._grip_t()) <= self.GRAB \
                    and self._angle_ok(0.0):
                self.held, self.off = "tube", self._grip_t() - self.tcp
        elif self.held == "handle" and not self._closed_on(M.SEC_W / 2):
            self.held = None
        elif self.held == "tube" and not self._closed_on(M.TUBE_W / 2):
            self.held = None
        if self.held == "handle":
            self.h[0] = self.tcp + self.off[0] - np.array([0.0, 0.0, M.TCP_OVER_H])
            self.h[1] = self.yaw + self.off[1]
        elif self.held == "tube":
            self.t = self.tcp + self.off - np.array([0.0, 0.0, M.TCP_OVER_T])
        # the handle rests on the faces when over them; held and pushed down, it drives both sprung stubs in
        sh, _ = seated_handle(self.seed)
        stubs = (0.0, 0.0)
        if math.hypot(*(self.h[0][:2] - sh[:2])) < 0.004 and self.h[0][2] < sh[2]:
            tr = max(-M.STUB_TRAVEL, self.h[0][2] - sh[2]) if self.held == "handle" else 0.0
            self.h[0][2], stubs = sh[2] + tr, (tr, tr)
        floor = M.TOP + M.PAD_H
        if self.held != "handle" and self.h[0][2] > floor and math.hypot(*(self.h[0][:2] - sh[:2])) > 0.004:
            self.h[0][2] = floor
        squeeze = M.wall_force_at(self.f) if self.held == "tube" else 0.0
        travel = max(0.0, (squeeze - M.WALL_PRE) / M.K_WALL) if squeeze > 0 else 0.0
        M.advance(M._M, DT, self.t, IDENT, -travel, travel, *self.mug, self.h[0], q_yaw(self.h[1]), self.tcp, self.f,
                  stubs=stubs, left=self.left)
        M._CLOCK["dt"] = DT
        M._CLOCK["steps"] += 1
        M._M["steps"] += 1
        self.trace.append(self.objects())
        self.holds.append(self.held)

    def objects(self):
        return {"mug": st(self.mug[0], self.mug[1]), "handle": st(self.h[0], q_yaw(self.h[1])),
                "tube": st(self.t), "bench": st((0.0, 0.0, M.TOP))}

    def run(self, ws, tail=2.0):
        for w in ws:
            p0, f0, y0 = self.tcp.copy(), self.f, self.yaw
            n = max(1, int(round(w.duration / DT)))
            for i in range(n):
                a = (i + 1) / n
                self.tcp = p0 + a * (np.asarray(w.pos) - p0)
                self.f = f0 + a * (w.finger - f0)
                self.yaw = y0 + a * (w.yaw - y0)
                self.step()
                M.update(self.trace[-1])
        for _ in range(int(tail / DT)):
            self.step()
            M.update(self.trace[-1])
        return M.score(self.trace[-1])


def replay(seed, plan_seed=None):
    M.reset_memory(seed)
    ws, _ = M.plan(0.0, seed if plan_seed is None else plan_seed)
    k = Kin(seed)
    return k.run(ws), k


class Model(unittest.TestCase):
    def setUp(self):
        M.reset_memory(trial_seed(2))
        self.mug = mug_pose(trial_seed(2))
        self.face = M.faces_w(*self.mug)

    def _tube_at(self, tip):
        return np.asarray(tip) - np.array([0.0, 0.0, M.NOZZLE_Z])

    def _squeeze(self, tip, force, secs, quat=IDENT, h=None):
        h = h if h is not None else (np.array([M.PAD[0], M.PAD[1], M.TOP + M.PAD_H]), IDENT)
        travel = (force - M.WALL_PRE) / M.K_WALL
        for _ in range(int(round(secs / DT))):
            M.advance(M._M, DT, self._tube_at(tip), quat, -travel, travel, *self.mug, h[0], h[1], None, None)

    def _fit(self, park=True):
        M.advance(M._M, DT, M.TUBE_START, IDENT, 0.0, 0.0, *self.mug, *seated_handle(trial_seed(2)), None, None)
        self.assertTrue(M._M["fitted"])
        if park:
            M.advance(M._M, DT, M.TUBE_START, IDENT, 0.0, 0.0, *self.mug, np.array(M.HANDLE_START), IDENT, *FAR)
            self.assertTrue(M._M["parked"])

    def _joined(self):
        """Fitted, parked, 20 uL on each face, the handle seated with the tube back."""
        self._fit()
        self._squeeze(self.face[0] + [0, 0, 0.001], 8.0, 2.0)
        self._squeeze(self.face[1] + [0, 0, 0.001], 8.0, 2.0)
        self._seat()
        self.assertTrue(M._M["joined"])

    def _seat(self, stubs=(0.0, 0.0), secs=DT, hand=(None, None), left=None, dz=0.0, tube=None, dx=0.0):
        p, q = seated_handle(trial_seed(2))
        for _ in range(int(round(secs / DT))):
            M.advance(M._M, DT, M.TUBE_START if tube is None else tube, IDENT, 0.0, 0.0, *self.mug,
                      p + [dx, 0, stubs[0] + dz], q, *hand, stubs=stubs, left=left)

    def test_no_flow_under_yield(self):
        self._fit()
        self._squeeze(self.face[0] + [0, 0, 0.001], M.F_YIELD - 0.1, 3.0)
        self.assertEqual(M._M["out"], 0.0)

    def test_flow_lands_on_the_face_under_the_nozzle(self):
        self._fit()
        self._squeeze(self.face[1] + [0, 0, 0.001], 8.0, 2.0)
        self.assertAlmostEqual(M._M["glue"][1], 20.0, places=6)
        self.assertEqual(M._M["glue"][0], 0.0)
        self.assertEqual(M._M["stray"], 0.0)

    def test_glue_off_the_face_is_stray(self):
        self._fit()
        self._squeeze(self.face[0] + [M.SEC_W / 2, 0, 0.001], 8.0, 0.2)   # over the face's edge
        self.assertGreater(M._M["stray"], M.STRAY_MAX)
        self.assertIn("off the broken faces", M.failure({"mug": st(self.mug[0]), "handle": st((0, 0, M.TOP)),
                                                         "tube": st(M.TUBE_START)}, M._M))

    def test_tilted_nozzle_is_stray(self):
        self._fit()
        tilt = (math.sin(math.radians(15)), 0.0, 0.0, math.cos(math.radians(15)))   # 30 deg about x
        self._squeeze(self.face[0] + [0, 0, 0.001], 8.0, 0.5, quat=tilt)
        self.assertEqual(M._M["glue"], [0.0, 0.0])

    def test_glue_before_dry_fit_latches(self):
        self._squeeze(self.face[0] + [0, 0, 0.001], 8.0, 1.0)
        self.assertTrue(M._M["glued_early"])

    def test_seating_rules(self):
        p, q = seated_handle(trial_seed(2))
        self.assertEqual(M.seated(*self.mug, p, q), (True, True))
        self.assertEqual(M.seated(*self.mug, p + [M.SEAT_XY * 1.2, 0, 0], q), (False, False))
        self.assertEqual(M.seated(*self.mug, p + [0, 0, M.SEAT_Z[1] * 1.2], q), (False, False))   # hovering
        self.assertEqual(M.seated(*self.mug, p + [0, 0, M.SEAT_Z[0] * 1.2], q), (False, False))
        self.assertEqual(M.seated(*self.mug, p + [0, 0, -0.001], q, stubs=(-0.001, -0.001)), (True, True))
        self.assertEqual(M.seated(*self.mug, p, q_yaw(math.radians(5))), (False, False))
        self.assertEqual(M.seated(*self.mug, p, q_yaw(math.pi)), (True, True))   # turned end for end still fits

    def test_overfull_face_squeezes_out(self):
        self._fit()
        self._squeeze(self.face[0] + [0, 0, 0.001], 8.0, 3.5)          # 35 uL
        self._squeeze(self.face[1] + [0, 0, 0.001], 8.0, 2.0)
        M.advance(M._M, DT, M.TUBE_START, IDENT, 0.0, 0.0, *self.mug, *seated_handle(trial_seed(2)), None, None)
        self.assertTrue(M._M["squeezed_out"][0])
        self.assertFalse(M._M["joined"])

    def test_open_time(self):
        self._fit()
        self._squeeze(self.face[0] + [0, 0, 0.001], 8.0, 2.0)
        self._squeeze(self.face[1] + [0, 0, 0.001], 8.0, 2.0)
        M._M["t"] += M.OPEN_TIME
        M.advance(M._M, DT, M.TUBE_START, IDENT, 0.0, 0.0, *self.mug, *seated_handle(trial_seed(2)), None, None)
        self.assertFalse(M._M["joined"])

    def test_glue_before_parking_latches(self):
        self._fit(park=False)
        self._squeeze(self.face[0] + [0, 0, 0.001], 8.0, 1.0)
        self.assertEqual(M._M["glued_early"], "glue put on before the handle was set back on the pad")

    def test_nozzle_resting_on_the_face_counts(self):
        self._fit()
        self._squeeze(self.face[0] + [0, 0, -0.0004], 8.0, 1.0)
        self.assertGreater(M._M["glue"][0], 9.0)
        self.assertEqual(M._M["stray"], 0.0)

    def test_tube_must_be_back_before_joining(self):
        self._fit()
        self._squeeze(self.face[0] + [0, 0, 0.001], 8.0, 2.0)
        self._squeeze(self.face[1] + [0, 0, 0.001], 8.0, 2.0)
        self._seat(tube=np.array(M.TUBE_START) + [0.0, 0.05, 0.05])
        self.assertFalse(M._M["joined"])
        self.assertEqual(M._M["order"], "handle seated on its glue before the tube was back in its holder")

    def test_lifting_a_joined_handle_breaks_the_joint(self):
        self._joined()
        tr = -0.0016
        self._seat(stubs=(tr, tr), secs=1.0)
        self.assertTrue(M._M["pressed"])
        self._seat(dz=M.LIFT_OFF + 0.001, secs=DT)                  # lifted off before let go
        self.assertFalse(M._M["joined"])
        self.assertFalse(M._M["pressed"])
        self._seat(secs=DT, hand=FAR)                               # set back and let go unpressed
        self.assertTrue(M._M["joined"])
        self.assertFalse(M._M["let_go"])

    def test_a_press_rebound_does_not_break_the_joint(self):
        """Released from a press, the stubs' springs throw the handle up to 1.3 mm (run 9): unseated for a moment but
        still on its stubs, the joint and the press stand, and it is let go once seated again."""
        self._joined()
        tr = -0.0016
        self._seat(stubs=(tr, tr), secs=1.0)
        self.assertTrue(M._M["pressed"])
        self._seat(dz=0.0013, secs=3 * DT)                          # thrown up by the stubs, not seated
        self.assertFalse(all(M._M["seat"]))
        self.assertTrue(M._M["joined"])
        self.assertTrue(M._M["pressed"])
        self._seat(secs=DT, hand=FAR)                               # back down, let go
        self.assertTrue(M._M["let_go"])

    def test_lift_limit_both_sides(self):
        """Just under LIFT_OFF the joint stands; slid so the ends' centres are over SEAT_XY off, it breaks."""
        self._joined()
        tr = -0.0016
        self._seat(stubs=(tr, tr), secs=1.0)
        self.assertTrue(M._M["pressed"])
        self._seat(dz=M.LIFT_OFF - 0.0003, secs=3 * DT)
        self.assertTrue(M._M["pressed"])
        self._seat(dx=M.SEAT_XY + 0.0005, secs=DT)                 # slid across the faces before let go
        self.assertFalse(M._M["joined"])
        self.assertFalse(M._M["pressed"])

    def test_a_tilted_end_gaping_at_one_edge_is_not_seated(self):
        p, _ = seated_handle(trial_seed(2))
        a = math.radians(3)                                         # under the 4 deg square limit
        tilt = (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))
        lift = np.array([0, 0, 0.0004])                             # the centre within SEAT_Z
        self.assertEqual(M.seated(*self.mug, p + lift, IDENT), (True, True))
        self.assertEqual(M.seated(*self.mug, p + lift, tilt), (False, False))

    def test_let_go_needs_a_press_on_both_stubs(self):
        self._joined()
        self._seat(secs=0.5, hand=FAR)                              # let go unpressed
        self.assertFalse(M._M["let_go"])
        tr = -(M.F_PRESS - M.STUB_PRE) / M.K_STUB - 0.0001
        self._seat(stubs=(tr, 0.0), secs=1.0)                       # one stub only
        self.assertFalse(M._M["pressed"])
        self._seat(stubs=(tr, tr), secs=M.PRESS_S - 2 * DT)         # both, but not for PRESS_S
        self.assertFalse(M._M["pressed"])
        self._seat(stubs=(tr, tr), secs=3 * DT)
        self.assertTrue(M._M["pressed"])
        self._seat(secs=DT, hand=FAR)
        self.assertTrue(M._M["let_go"])

    def test_left_hand_holding_blocks_let_go(self):
        self._joined()
        tr = -0.0016
        self._seat(stubs=(tr, tr), secs=1.0)
        grip = M.grip_w(*seated_handle(trial_seed(2)))
        self._seat(secs=0.5, hand=FAR, left=(grip, -0.004))         # the left hand closed on the handle
        self.assertFalse(M._M["let_go"])
        self._seat(secs=DT, hand=FAR, left=FAR)
        self.assertTrue(M._M["let_go"])

    def test_moved_is_measured_against_the_mug(self):
        self._joined()
        self._seat(stubs=(-0.0016, -0.0016), secs=1.0)
        self._seat(secs=DT, hand=FAR, left=FAR)
        self.assertTrue(M._M["let_go"])
        p, q = seated_handle(trial_seed(2))
        shift = np.array([0.0, 0.003, 0.0])                         # the mug and the handle slide together
        M.advance(M._M, DT, M.TUBE_START, IDENT, 0.0, 0.0, self.mug[0] + shift, self.mug[1], p + shift, q, *FAR,
                  left=FAR)
        self.assertFalse(M._M["moved"])
        M.advance(M._M, DT, M.TUBE_START, IDENT, 0.0, 0.0, self.mug[0], self.mug[1], p + [0.0012, 0, 0], q, *FAR,
                  left=FAR)
        self.assertTrue(M._M["moved"])

    def test_bead_shows_from_the_window_floor(self):
        proud = M.bead_rise(M.V_LO, False) - (M.BEAD_DROP - M.BEAD_T)
        self.assertGreater(proud, 0.001)
        self.assertLess(M.bead_rise(0.8 * M.V_HI, False), M.bead_rise(M.V_HI, False))   # still rising inside the window

    def test_force_inverse(self):
        for f in (2.0, 4.0, 8.0, 12.0):
            self.assertAlmostEqual(M.wall_force_at(M.finger_for_force(f)), f, places=9)
        self.assertLess(M.wall_force_at(-(M.TUBE_W / 2 - M.HOLD_PEN)), M.F_YIELD)   # the holding grip does not dispense


class Demo(unittest.TestCase):
    def test_demo_succeeds_on_every_film_seed(self):
        for cli in CLI:
            s, k = replay(trial_seed(cli))
            self.assertEqual(s, 1.0, (cli, M._MEM["fail"], M._M["glue"], M._M["stray"], M._MEM["now"]))
            for g in M._M["glue"]:
                self.assertTrue(M.V_LO + 4 <= g <= M.V_HI - 4, (cli, M._M["glue"]))   # margin for the sim

    def test_open_loop_plan_fails_other_layouts(self):
        """Q2: the plan for one layout, replayed on another, does not succeed (the handle and the mug move by seed)."""
        for cli in CLI:
            for other in CLI:
                if other != cli:
                    s, _ = replay(trial_seed(cli), plan_seed=trial_seed(other))
                    self.assertLess(s, 0.5, (cli, other, s))

    def test_skipping_the_dry_fit_scores_zero(self):
        seed = trial_seed(0)
        M.reset_memory(seed)
        ws, _ = M.plan(0.0, seed)
        start = next(i for i, w in enumerate(ws) if w.label == "over the tube")
        lay = M.layout(seed)
        ws = list(ws[start:])
        k = Kin(seed)
        k.h = [np.array([M.PAD[0], M.PAD[1], M.TOP + M.PAD_H]), 0.0]   # as if set square on the pad
        self.assertEqual(k.run(ws), 0.0)
        self.assertEqual(M._MEM["fail"], "glue put on before the dry fit")
        del lay

    def test_demo_fits_the_run(self):
        for cli in CLI:
            ws, _ = M.plan(0.0, trial_seed(cli))
            self.assertLess(sum(w.duration for w in ws) / DT, M.RUN_LIMIT_STEPS - 400)

    def test_film_seeds_differ(self):
        self.assertEqual(sorted(M.layout_index(trial_seed(c)) for c in CLI), [0, 1, 2])


class Scoring(unittest.TestCase):
    def test_ladder_along_the_demo(self):
        """The score climbs 0.2, 0.4, 0.6, 0.8, 1.0 along the demo and never falls."""
        seed = trial_seed(2)
        M.reset_memory(seed)
        ws, _ = M.plan(0.0, seed)
        k = Kin(seed)
        seen = []
        for w in ws:
            k.run([w], tail=0.0)
            seen.append(M.score(k.trace[-1]))
        k.run([], tail=2.0)
        seen.append(M.score(k.trace[-1]))
        self.assertEqual(sorted(set(seen)), [0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
        self.assertEqual(seen, sorted(seen))

    def test_moving_after_let_go_latches(self):
        s, k = replay(trial_seed(4))
        self.assertEqual(s, 1.0)
        k.h[0] = k.h[0] + np.array([0.0015, 0.0, 0.0])
        k.run([], tail=0.2)
        self.assertEqual(M._MEM["fail"], "handle moved after it was let go on its glue")

    def test_model_must_step(self):
        s, k = replay(trial_seed(4))
        M._CLOCK["steps"] += 10
        M._MEM["last"] = None
        self.assertEqual(M.score(k.objects()), 0.0)

    def test_tube_left_out_drops_below_the_seated_rung(self):
        """Stage 3 is the state now: with the tube out of its holder at the end the run scores the glued rung."""
        s, k = replay(trial_seed(4))
        o = k.objects()
        o["tube"] = st(np.array(M.TUBE_START) + [0.03, 0, 0])
        M._MEM["last"] = None
        self.assertEqual(M.score(o), 0.4)

    def test_mug_turned_or_slid_in_its_cradle_fails(self):
        M.reset_memory(trial_seed(2))
        mp, _ = mug_pose(trial_seed(2))
        base = {"handle": st((0, 0, M.TOP)), "tube": st(M.TUBE_START)}
        self.assertIsNone(M.failure({**base, "mug": st(mp)}, M._M))
        a = math.radians(6)
        self.assertEqual(M.failure({**base, "mug": st(mp, (0.0, math.sin(a / 2), 0.0, math.cos(a / 2)))}, M._M),
                         "mug turned in its cradle")
        self.assertEqual(M.failure({**base, "mug": st(mp + [0, M.IN_CRADLE * 1.2, 0])}, M._M),
                         "mug moved in its cradle")


class Geometry(unittest.TestCase):
    def test_handle_ends_flat_at_z0(self):
        for sh in M.handle_shapes():
            self.assertGreaterEqual(min(p[2] for p in sh.points), -1e-12)
        y0, z0 = M.arch_point(0.0, 0.0)
        y1, z1 = M.arch_point(math.pi, 0.0)
        self.assertAlmostEqual(y0, M.STUB_Y[0])
        self.assertAlmostEqual(y1, M.STUB_Y[1])
        self.assertAlmostEqual(z0, 0.0)

    def test_handle_reach_matches_handle_out(self):
        top = max(p[2] for sh in M.handle_shapes() for p in sh.points)
        self.assertAlmostEqual(top + M.STUB_H, M.HANDLE_OUT, places=4)

    def test_mug_does_not_roll(self):
        """The whole mug's centre of mass (body, stubs, beads) lies under its axis, so it rests stubs up."""
        m = z = 0.0
        for lk in (M.MUG.base, *M.MUG.links):
            c = lk.com if lk.com is not None else (0.0, 0.0, 0.0)   # the beads' collider is on the axis
            m, z = m + lk.mass, z + lk.mass * c[2]
        self.assertLess(z / m, -0.001)

    def test_mug_body_collider_is_a_prism(self):
        """PhysX's cylinder collider spun the untouched mug in place on the rails' edges (run 7): the body is a prism."""
        body = M.MUG.base.shapes[0]
        self.assertEqual(body.kind, "hull")
        self.assertEqual(len(body.points), 2 * M.MUG_SIDES)
        for x, y, z in body.points:
            self.assertAlmostEqual(math.hypot(x, z), M.R_MUG, places=9)
            self.assertAlmostEqual(abs(y), M.L_MUG / 2, places=9)
        # PhysX cooks at most 64 hull vertices by default, so more would be reduced to some other polytope
        self.assertLessEqual(len(body.points), 64)
        # its flats sit inside the true radius by under 0.2 mm, so the rest height ZC barely moves
        self.assertLess(M.R_MUG * (1 - math.cos(math.pi / M.MUG_SIDES)), 2e-4)

    def test_mug_rests_on_rails(self):
        self.assertAlmostEqual(math.hypot(M.RAIL_IN, M.ZC - M.RAIL_H), M.R_MUG, places=9)

    def test_nozzle_reaches_face_clear_of_mug(self):
        """While dispensing, the tube's shoulder and walls stay over the stub's top (no collision with the mug)."""
        tip_z = M.Z_FACE + M.NOZ_OVER
        frame = tip_z - M.NOZZLE_Z
        self.assertGreater(frame + M.SHOULDER_Z, M.Z_FACE + 0.010)

    def test_grip_pads_straddle_the_arch(self):
        """The pad tips sit under the arch's section at its top, above the faces when seated."""
        tips = M.TCP_OVER_H - M.TIP
        self.assertLess(tips, M.ARCH_B - M.SEC_D / 2)
        self.assertGreater(tips, 0.010)


SEAT_GAUGE = M.SEAT_Z[1]


class ReadmeNumbers(unittest.TestCase):
    """The README's numbers come from the code."""
    TXT = " ".join((HERE / "README.md").read_text().split())

    def test_seed_table(self):
        for cli in CLI:
            i = M.layout_index(trial_seed(cli))
            dx, dy, yaw, mdy = M.LAYOUTS[i]
            row = (f"| {cli} | {i} | {1000 * dx:.0f}, {1000 * dy:.0f} | {math.degrees(yaw):.0f} | {1000 * mdy:.0f} |")
            self.assertIn(row, self.TXT)

    def test_model_numbers(self):
        t = self.TXT
        self.assertIn(f"{M.V_LO:g} to {M.V_HI:g} uL", t)
        self.assertIn(f"within {1000 * M.DEP_H:.0f} mm over a {1000 * M.SEC_W:.0f} by {1000 * M.SEC_D:.0f} mm face", t)
        self.assertIn(f"{1000 * 2 * M.ARCH_A:.0f} mm apart", t)
        self.assertIn(f"two walls {1000 * M.TUBE_W:.0f} mm apart that slide in up to {1000 * M.SQUEEZE_MAX:.0f} mm", t)
        self.assertIn(f"({M.K_WALL / 1000:g} N/mm, {M.WALL_PRE:g} N preload)", t)
        self.assertIn(f"mean squeeze is {M.F_YIELD:g} N or less", t)
        self.assertIn(f"{M.FLOW_G:g} uL/s per N", t)
        self.assertIn(f"({1000 * M.DEP_IN:.1f} mm inside its edges, up to {1000 * M.DEP_H:.0f} mm above, or resting", t)
        self.assertIn(f"within {math.degrees(math.acos(M.UPRIGHT)):.0f} deg", t)
        self.assertIn(f"within {1000 * M.SEAT_XY:g} mm of its face across and along the mug, every corner of the end "
                      f"{-1000 * M.SEAT_Z[0]:.1f} mm into to {1000 * M.SEAT_Z[1]:.1f} mm over the face in height", t)
        self.assertIn(f"slide {1000 * M.IN_CRADLE:.0f} mm along its cradle and turn "
                      f"{math.degrees(math.acos(M.MUG_UPRIGHT)):.0f} deg", t)
        self.assertIn(f"centre of mass is set {1000 * M.MUG_COM_DROP:.0f} mm below its", t)
        self.assertIn(f"{M.V_LO:g} to {M.V_HI:g} uL on the face is {M.V_LO / 96:.2f} to {M.V_HI / 96:.2f} mm thick", t)
        self.assertIn(f"({M.K_STUB / 1000:g} N/mm, {M.STUB_PRE:g} N preload, {1000 * M.STUB_TRAVEL:g} mm travel)", t)
        self.assertIn(f"held at {M.F_PRESS:g} N or more for {M.PRESS_S:g} s", t)
        self.assertIn(f"press each with {M.F_PRESS:g} N", t)
        self.assertIn(f"({2 * M.F_PRESS:g} N) over the rest weight", t)
        self.assertIn(f"a {1000 * SEAT_GAUGE:.1f} mm feeler gauge", t)
        self.assertIn(f"TCP {1000 * M.HAND_CLEAR:.0f} mm from the handle's grip, or {1000 * M.FAR_CLEAR:.0f} mm away", t)
        self.assertIn(f"mug's radius ({1000 * M.R_MUG:.0f} mm", t)
        proud = 1000 * (M.bead_rise(M.V_LO, False) - (M.BEAD_DROP - M.BEAD_T))
        self.assertIn(f"{proud:.1f} mm proud at {M.V_LO:g} uL", t)
        self.assertIn(f"within {math.degrees(math.acos(M.SEAT_TILT)):.0f} deg", t)
        self.assertIn(f"moving {1000 * M.MOVED:.0f} mm in the mug's frame is a fault", t)
        self.assertIn(f"({M.V_AIM:.0f} uL a face)", t)
        self.assertIn(f"within {M.OPEN_TIME / 60:.0f} minutes of the first glue", t)
        area = 1e6 * M.SEC_W * M.SEC_D
        self.assertIn(f"on a {area:.0f} mm2 face, a layer {M.V_LO / area:.2f} to {M.V_HI / area:.2f} mm thick", t)


if __name__ == "__main__":
    unittest.main()
