"""Offline tests for dowel_joint.py: geometry, the measurement, every rung's bounds, each stem's expected end state,
the demo's reach, and the text against the code. No simulator: yam_arena on PYTHONPATH (pure Python parts only).

    PYTHONPATH=<yam_arena source> python3 test_dowel_joint.py
"""
import json
import math
import pathlib
import sys
import unittest
import warnings

import numpy as np

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from yam_arena.task_file import load  # noqa: E402

TF = load(HERE / "dowel_joint.py")
M = TF.module
Q0 = (0.0, 0.0, 0.0, 1.0)


def qyaw(deg):
    return M._quat(math.radians(deg))


def qtilt(deg):  # turned about x (board's long end lifts)
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class S:
    """A minimal object state."""

    def __init__(self, pos, quat=Q0, vel=(0.0, 0.0, 0.0)):
        self.pos, self.quat_xyzw, self.lin_vel = np.asarray(pos, float), np.asarray(quat, float), np.asarray(vel, float)


A0 = np.array([M.X_LINE, M.SEAM_Y - M.BOARD_L / 2, M.Z_C])


def state(gap=0.0, dx=0.0, dz=0.0, yaw=0.0, a_quat=Q0, b_quat=None, vel=0.0):
    """The dowel board at its seat; the hole board placed so its end centre sits gap along, dx across, dz up (in the
    dowel board's frame, which here is the world's) and turned yaw deg about its end centre."""
    bq = qyaw(yaw) if b_quat is None else b_quat
    end = A0 + np.array([dx, M.BOARD_L / 2 + gap, dz])
    centre = end - M._rot3(bq) @ np.array([0.0, -M.BOARD_L / 2, 0.0])
    return {"dowel_board": S(A0, a_quat), "hole_board": S(centre, bq, (0.0, vel, 0.0))}


def clear_hands():
    M._M["pads"] = [np.array([0.35, 0.10, M.TOP + 0.20])] * 8


def score_of(objects, pads="clear", held=0.0):
    M.reset_memory()
    if pads == "clear":
        clear_hands()
    elif pads is not None:
        M._M["pads"] = pads
    M._CLOCK["dt"] = 0.05
    s = M.score(objects)
    if held:
        for _ in range(int(round(held / 0.05))):
            M._MEM["last"] = None
            s = M.score(objects)
    return s


class Geometry(unittest.TestCase):
    def test_clearance_and_capture(self):
        self.assertAlmostEqual(M.R_HOLE - M.R_DOWEL, M.CLR)
        self.assertAlmostEqual(M.CAPTURE, M.CLR + M.DOWEL_CH)
        self.assertAlmostEqual(M.CAPTURE, 0.0012)

    def test_dowel_fits_hole(self):
        """The 24-sided dowel's corners (on R_DOWEL) sit CLR inside the 16-sided hole's flats in every direction."""
        rv = M.R_HOLE / math.cos(math.pi / M.HOLE_SIDES)
        for k in range(M.DOWEL_SIDES):
            a = 2 * math.pi * k / M.DOWEL_SIDES
            # distance to the hole polygon along a: the flat's apothem over cos of the angle to the flat's normal
            seg = 2 * math.pi / M.HOLE_SIDES
            off = (a % seg) - seg / 2
            self.assertGreaterEqual(M.R_HOLE / math.cos(off) - M.R_DOWEL, M.CLR - 1e-12)
        self.assertGreater(rv, M.R_HOLE)

    def test_hole_walls_fill_the_cut_and_leave_the_bore(self):
        """Each hole's 16 wedges: convex quads whose inner edges are the bore's flats and outer edges the square cut."""
        w = M.hole_wedges(0.0, 650.0)
        self.assertEqual(len(w), M.HOLE_SIDES)
        for s in w:
            pts = np.array(s.points)[:4][:, [0, 2]]
            cross = []
            for i in range(4):
                u, v = pts[(i + 1) % 4] - pts[i], pts[(i + 2) % 4] - pts[(i + 1) % 4]
                cross.append(u[0] * v[1] - u[1] * v[0])
            self.assertTrue(all(c > 0 for c in cross) or all(c < 0 for c in cross))
            self.assertLessEqual(np.abs(pts).max(), M.CELL / math.cos(math.pi / 4) + 1e-9)
            mid_inner = (pts[0] + pts[1]) / 2
            self.assertAlmostEqual(float(np.linalg.norm(mid_inner)), M.R_HOLE, places=9)

    def test_board_masses(self):
        for o in TF.objects:
            self.assertAlmostEqual(o.mass, M.BOARD_KG, places=9)

    def test_wall_thickness(self):
        self.assertGreaterEqual(M.BOARD_W / 2 - M.PITCH - M.R_DOWEL, 0.007 - 1e-9)
        self.assertGreaterEqual(M.BOARD_T / 2 - M.CELL, 0.003 - 1e-9)


class Measure(unittest.TestCase):
    def test_seated(self):
        m = M.measure(*[v for o in state().values() for v in (o.pos, o.quat_xyzw)])
        self.assertAlmostEqual(m["gaps"][0], 0.0, places=9)
        self.assertAlmostEqual(m["gaps"][1], 0.0, places=9)
        for d in m["depth"]:
            self.assertAlmostEqual(d, M.DOWEL_OUT, places=9)
        self.assertEqual(M.rung(m), 3)

    def test_start_gap(self):
        lay = M.layout(0)
        a, b = lay["dowel_board"], lay["hole_board"]
        m = M.measure(a[:3], M._quat(a[3]), b[:3], M._quat(b[3]))
        self.assertAlmostEqual(m["gaps"][0], 0.510, places=6)

    def test_yaw_corner_gaps(self):
        o = state(gap=0.005, yaw=2.0)
        m = M.measure(o["dowel_board"].pos, o["dowel_board"].quat_xyzw, o["hole_board"].pos, o["hole_board"].quat_xyzw)
        self.assertAlmostEqual(abs(m["gaps"][1] - m["gaps"][0]), M.BOARD_W * math.sin(math.radians(2.0)), places=6)


class Rungs(unittest.TestCase):
    def test_nothing_moved(self):
        lay = M.layout(0)
        a, b = lay["dowel_board"], lay["hole_board"]
        self.assertEqual(score_of({"dowel_board": S(a[:3], M._quat(a[3])), "hole_board": S(b[:3])}), 0.0)

    def test_full_and_hold(self):
        self.assertEqual(score_of(state()), 0.9)
        self.assertEqual(score_of(state(), held=1.05), 1.0)

    def test_seated_hand_on(self):
        self.assertEqual(score_of(state(), pads=None), 0.75)
        on = [A0 + np.array([0.0, 0.05, 0.02])] * 8
        self.assertEqual(score_of(state(), pads=on), 0.75)

    def test_moving_board_not_done(self):
        self.assertEqual(score_of(state(vel=0.006)), 0.75)

    def test_rung_limits(self):
        self.assertEqual(score_of(state(gap=M.SEAT_GAP - 1e-6)), 0.9)
        self.assertEqual(score_of(state(gap=M.SEAT_GAP + 1e-5)), 0.5)
        self.assertEqual(score_of(state(gap=M.HALF_GAP - 1e-6)), 0.5)
        self.assertEqual(score_of(state(gap=M.HALF_GAP + 1e-5)), 0.25)
        self.assertEqual(score_of(state(gap=M.DOWEL_OUT - M.TIP_IN - 1e-6)), 0.25)
        self.assertEqual(score_of(state(gap=M.DOWEL_OUT - M.TIP_IN + 1e-5)), 0.0)
        self.assertEqual(score_of(state(dx=M.FLUSH - 1e-6)), 0.9)
        self.assertEqual(score_of(state(dx=M.FLUSH + 1e-5)), 0.5)

    def test_stacked_never_scores(self):
        """A board on top of the other (19 mm up) at any gap, and the level bound at its edge."""
        for gap in (-0.100, -0.040, -0.010, 0.0, 0.005, 0.015):
            self.assertEqual(score_of(state(gap=gap, dz=M.BOARD_T)), 0.0, gap)
        self.assertEqual(score_of(state(dz=M.LEVEL - 1e-6)), 0.9)
        self.assertEqual(score_of(state(dz=M.LEVEL + 1e-5)), 0.0)

    def test_overlap_never_scores(self):
        for gap in (-0.150, -0.020, -0.003, M.OVERLAP - 1e-5):
            self.assertEqual(score_of(state(gap=gap)), 0.0, gap)
        self.assertEqual(score_of(state(gap=M.OVERLAP + 1e-5)), 0.9)

    def test_one_dowel_missed(self):
        """Hole board one pitch to the side: seam closed, two dowels in holes, one beside the board: 0."""
        self.assertEqual(score_of(state(dx=M.PITCH)), 0.0)
        self.assertEqual(score_of(state(gap=0.005, dx=M.PITCH)), 0.0)

    def test_skew_on_chamfers(self):
        """Turned 5 deg, the outer dowels stop on the end face (gap about the tips' 20 mm): 0."""
        self.assertEqual(score_of(state(gap=M.DOWEL_OUT - 0.0005, yaw=5.0)), 0.0)

    def test_tilted_board(self):
        self.assertEqual(score_of(state(b_quat=qtilt(M.FLAT_DEG + 0.5))), 0.0)
        self.assertEqual(score_of(state(a_quat=qtilt(M.FLAT_DEG + 0.5))), 0.0)

    def test_half_in(self):
        self.assertEqual(score_of(state(gap=0.008)), 0.5)

    def test_final_state_only(self):
        """Seated then pulled apart: the end state scores."""
        score_of(state())
        M._MEM["last"] = None
        self.assertEqual(M.score(state(gap=0.030)), 0.0)


class Stems(unittest.TestCase):
    # The design's expectation per stem; the measured scores on the final commit go in the README. relay: reported,
    # not gated (None), as nothing in the sim stops a relay from seating
    EXPECT = {"demo": 1.0, "nothing": 0.0, "half": 0.5, "offset": 0.0, "skew": 0.0, "stack": 0.0, "relay": None,
              "left_only": 0.0, "hold": 0.75}

    def test_every_stem_has_an_expectation(self):
        for v in self.EXPECT:
            M.plan(0, v)  # each variant builds
        txt = (HERE / "sim" / "mk.sh").read_text()
        for v in ("nothing", "half", "offset", "skew", "left_only", "relay", "stack", "hold"):
            self.assertIn(v, txt)

    def test_hold_keeps_the_grip(self):
        """s_hold: the demo to the seat, but neither hand lets go, so the pads stay on the boards: 0.75."""
        try:
            demo, hold = M.plan(0, "demo"), M.plan(0, "hold")
            for arm in ("left", "right"):
                labels = [w.label for w in hold[arm]]
                self.assertNotIn("let go", labels)
                self.assertEqual(hold[arm][-1].finger, M.GRIP)
                self.assertEqual(labels, [w.label for w in demo[arm]][:len(labels)])
            self.assertTrue(M._SERVO["on"])
        finally:
            M.reset_memory()

    def test_half_commands_8mm(self):
        W = M.plan(0, "half")["left"]
        push = [w for w in W if w.label == "push the joint closed slowly"][0]
        seat = M.grip_point("hole_board", *M.seat_centres()["hole_board"], 0.0)
        self.assertAlmostEqual(push.pos[1] - seat[1], (0.008 - M.LAG) / 2, places=9)   # stops LAG wider: 8 mm

    def test_left_only_out_of_reach(self):
        """The left arm alone cannot reach the dowel board at its start nor carry its own board to it."""
        from yam_arena import ik, station
        from yam_arena.demo import grip_rotation
        lay = M.layout(0)
        a = lay["dowel_board"]
        targets = [M.grip_point("dowel_board", a[0], a[1], a[3]),
                   M.grip_point("hole_board", a[0], a[1] + M.BOARD_L, 0.0)]
        for x, y in targets:
            _, err, _ = ik.solve(np.array([x, y, M.Z_GRIP]), ik.matrix_to_quat_xyzw(grip_rotation(M.YAW_L)), None,
                                 base_pos=station.BASES["left"], iters=400)
            self.assertGreater(err, 0.020, (x, y))

    def test_open_loop_wrong_layout_misses_the_grip(self):
        """A plan for one seed's layout puts a finger on the dowel board for another (12 mm shift > 11.5 mm open
        margin)."""
        margin = -M.OPEN - M.BOARD_W / 2
        self.assertLess(margin, abs(M.LAYOUTS[1][0]))


class Placement(unittest.TestCase):
    """The seeded layout is placed: mechanism's reset runs on_place for a task of loose Bodies only (2026-11-18: it ran
    only beside a Mechanism, so every trial started in layout 0 while the demo planned for the seed's layout)."""

    def test_reset_places_a_body_only_task(self):
        import types
        mech = sys.modules["mechanism"]
        calls = []

        class Base:
            step_dt = 0.05
            sim = types.SimpleNamespace(render=lambda: None)

        class Emb:
            def __init__(self):
                self.task = types.SimpleNamespace(objects=M.OBJECTS)
                self._env = types.SimpleNamespace(step=lambda a: calls.append("step"))
                self._base, self._ready = Base(), None

            def reset(self, scene, *, seed=None):
                return "first"

            def step(self, action):
                return None

            def close(self):
                return None

            def _task_step(self, *a, **k):
                return None

            def _observation(self):
                return "placed"

        fake = types.SimpleNamespace(YamArenaEmbodiment=Emb, SETTLE_S=0.1, RESET_RENDERS=1)
        mech._patch_embodiment(fake)
        place = mech._PLACE[:]
        mech._PLACE[:] = [lambda env: calls.append(("place", mech.run_seed()))]
        try:
            obs = Emb().reset(None, seed=2566101560)
        finally:
            mech._PLACE[:] = place
        self.assertEqual(obs, "placed")
        self.assertIn(("place", 2566101560), calls)
        self.assertEqual(M.layout_index(2566101560), 2)
        self.assertIn(M.place_layout, place)

    def test_placed_pose_matches_the_plan_for_every_layout(self):
        """place_layout writes, for each layout, the start pose the demo plans its grips for; the batch 4 traces
        (step 10, after the reset settled) show the same poses [RUN]."""
        seen = {0: (350.0, -330.0, 0.0), 1: (361.932, -330.005, 2.977), 2: (338.069, -329.995, -2.976)}
        seeds = {0: 377265627, 1: 1415158438, 2: 2566101560}
        mech = sys.modules["mechanism"]
        write = M._write_root
        try:
            for lay, seed in seeds.items():
                got = {}
                M._write_root = lambda base, name, xyz, yaw: got.__setitem__(name, (*xyz, yaw))
                mech._SEED[0] = seed
                M.place_layout(types_ns(unwrapped=None))
                self.assertEqual(M.layout_index(seed), lay)
                want = M.layout(seed)
                for name in ("dowel_board", "hole_board"):
                    self.assertEqual(got[name], want[name], (lay, name))
                a = got["dowel_board"]
                self.assertAlmostEqual(1000 * a[0], seen[lay][0], delta=0.1)
                self.assertAlmostEqual(1000 * a[1], seen[lay][1], delta=0.1)
                self.assertAlmostEqual(math.degrees(a[3]), seen[lay][2], delta=0.05)
                for arm, name in (("right", "dowel_board"), ("left", "hole_board")):
                    w = [x for x in M.plan(seed)[arm] if x.label == "grip across its width"][0]
                    x, y, _, yaw = got[name]
                    self.assertAlmostEqual(w.pos[0], M.grip_point(name, x, y, yaw)[0], places=9)
                    self.assertAlmostEqual(w.pos[1], M.grip_point(name, x, y, yaw)[1], places=9)
                    base_yaw = M.YAW_R if arm == "right" else M.YAW_L
                    self.assertAlmostEqual(w.yaw - base_yaw, yaw, places=9)
        finally:
            M._write_root = write
            mech._SEED[0] = None


def types_ns(**k):
    import types
    return types.SimpleNamespace(**k)


class Tries(unittest.TestCase):
    class P:
        def __init__(self, w):
            self.waypoints = w

    def run_to(self, t, gap, dx=0.0):
        p = M.plan(0)
        plans = {a: self.P(list(w)) for a, w in p.items()}
        st = LineUp().seat(gap=gap, b_dx=dx)
        M._M["boards"], M._M["tcps"] = tuple(st[:4]), {"right": st[4], "left": st[5]}
        M.servo(plans, t)
        return p, plans

    def check_time(self, n):
        return [t for t, k, l in M._SERVO["events"] if l == f"check {n}"][0]

    def test_tries_in_the_plan(self):
        W = M.plan(0)["left"]
        labels = [w.label for w in W]
        self.assertEqual(sum(lb.startswith("check") for lb in labels), 1 + M.RETRIES)
        seat = M.grip_point("hole_board", *M.seat_centres()["hole_board"], 0.0)
        for lb, g in (("push to the holes", M.PROBE_GAP - M.LAG), ("retry 1 back off", M.RETRY_GAP + M.LAG),
                      ("retry 2 line up at the mouths", M.MOUTH_GAP - M.LAG)):
            self.assertAlmostEqual(W[labels.index(lb)].pos[1] - seat[1], g / 2, places=9)
        self.assertLess(labels.index(f"check {1 + M.RETRIES}"), labels.index("push the joint closed slowly"))
        self.assertGreater(M.PROBE_GAP, 0.0)
        self.assertLess(M.PROBE_GAP + M.ENTERED, M.DOWEL_OUT)   # a try that went in is ENTERED deep at PROBE_GAP

    def test_in_at_the_first_check_holds_the_retries(self):
        try:
            M.plan(0)
            t = self.check_time(1) + 0.01
            p, plans = self.run_to(t, gap=M.PROBE_GAP)
            self.assertTrue(M._SERVO["entered"])
            W = plans["left"].waypoints
            hold = [w for w in W if w.label == "check 1"][0]
            for w in W:
                if w.label.startswith("retry") or w.label in ("check 2", "check 3"):
                    self.assertEqual((w.pos, w.yaw), (hold.pos, hold.yaw), w.label)
            n = len(M._SERVO["log"])
            M.servo(plans, 99.0)                      # later passes and checks are skipped; the square passes run
            self.assertEqual([r["kind"] for r in M._SERVO["log"][n:]], ["square", "square"])
            fin = [w for w in W if w.label == "push the joint closed slowly"][0]
            self.assertEqual(fin, [w for w in p["left"] if w.label == "push the joint closed slowly"][0])
        finally:
            M.reset_memory()

    def test_square_passes_follow_the_tries(self):
        for v in ("demo", "relay"):
            labels = [w.label for w in M.plan(0, v)["left"]]
            last = max(i for i, lb in enumerate(labels) if lb.startswith("check"))
            self.assertEqual(labels.index(M.SQUARE[0]), last + 1, v)
            self.assertEqual(labels.index(M.SQUARE[1]), labels.index("push the joint closed slowly") + 1, v)
            self.assertLess(labels.index(M.SQUARE[1]), labels.index("let go"), v)
        M.reset_memory()

    def test_square_turns_the_hole_board_once_in(self):
        """The batch 5 layout 2 failure: in at check 1 with the boards 0.37 deg apart; the square pass turns the hole
        board's hand that much the other way, and leaves the dowel board's."""
        try:
            M.plan(0)
            ts = [t for t, k, l in M._SERVO["events"] if k == "square"]
            self.assertEqual(len(ts), 2)
            p = M.plan(0)
            plans = {a: self.P(list(w)) for a, w in p.items()}
            st = LineUp().seat(gap=M.PROBE_GAP, yb=math.radians(0.37))
            M._M["boards"], M._M["tcps"] = tuple(st[:4]), {"right": st[4], "left": st[5]}
            M._SERVO["entered"] = True
            M.servo(plans, ts[0] + 0.01)
            rec = M._SERVO["log"][-1]
            self.assertEqual(rec["kind"], "square")
            self.assertNotIn("right", rec["cmd"])
            self.assertAlmostEqual(rec["cmd"]["left"][1], -0.37, places=2)
        finally:
            M.reset_memory()

    def test_square_does_nothing_until_in(self):
        try:
            M.plan(0)
            ts = [t for t, k, l in M._SERVO["events"] if k == "square"]
            p, plans = self.run_to(ts[0] + 0.01, gap=M.DOWEL_OUT - 0.001, dx=0.0003)
            self.assertFalse(M._SERVO["entered"])
            self.assertEqual(M._SERVO["log"][-1]["kind"], "square")
            self.assertEqual(M._SERVO["log"][-1]["cmd"], {})
        finally:
            M.reset_memory()

    def test_stalled_on_the_chamfers_retries(self):
        """The batch 4 failure: ends 0.3 mm off with the tips 1 mm in (seam gap 19 mm): not in, so the retry runs."""
        try:
            M.plan(0)
            t = self.check_time(1) + 0.01
            p, plans = self.run_to(t, gap=M.DOWEL_OUT - 0.001, dx=0.0003)
            self.assertFalse(M._SERVO["entered"])
            back = [w for w in plans["left"].waypoints if w.label == "retry 1 back off"][0]
            seat = M.grip_point("hole_board", *M.seat_centres()["hole_board"], 0.0)
            self.assertGreater(back.pos[1] - seat[1], M.RETRY_GAP / 2 - 0.007)   # still backs off (line-up may shift x)
            r1 = [t for t, k, l in M._SERVO["events"] if l == "retry 1 line up at the mouths"][0]
            M.servo(plans, r1 + 0.01)
            self.assertEqual(M._SERVO["log"][-1]["kind"], "line")
            self.assertNotIn("right", M._SERVO["log"][-1]["cmd"])   # the pass before a push turns only the hole board
        finally:
            M.reset_memory()


class LineUp(unittest.TestCase):
    """The demo's line-up passes, on a kinematic stand-in: each board turns and shifts with its hand about the TCP."""
    BIAS = math.radians(0.7)   # [RUN: the 2026-11-18 p_fit trace, both boards about 0.7 deg off after the grip]

    def seat(self, gap=M.PRE_GAP, ya=0.0, yb=0.0, b_dx=0.0):
        """Both boards in line at a seam gap with their grips (= TCPs) where the plan puts them, then each turned about
        its own grip: (a_pos, a_quat, b_pos, b_quat, tcp_a, tcp_b)."""
        ca = np.array([M.X_LINE, M.SEAM_Y - M.BOARD_L / 2 - gap / 2])
        cb = np.array([M.X_LINE + b_dx, M.SEAM_Y + M.BOARD_L / 2 + gap / 2])
        ga, gb = ca + [0.0, -M.GRIP_DY], cb + [0.0, M.GRIP_DY]
        ca, cb = M._turn_about(ca, ga, ya), M._turn_about(cb, gb, yb)
        return ([*ca, M.Z_C], M._quat(ya), [*cb, M.Z_C], M._quat(yb), np.array([*ga, M.Z_GRIP]),
                np.array([*gb, M.Z_GRIP]))

    def apply(self, st, d):
        """The kinematic answer to a pass: each board turns with its hand about the TCP; the left one also shifts."""
        pa, qa, pb, qb, ta, tb = st
        da = d.get("right", (0.0, 0.0))[1]
        dxb, db = d["left"]
        ya, yb = M._yaw_of(qa) + da, M._yaw_of(qb) + db
        ca = M._turn_about(pa, ta, da)
        cb = M._turn_about(pb, tb, db) + [dxb, 0.0]
        tb2 = tb + [dxb, 0.0, 0.0]
        return ([*ca, M.Z_C], M._quat(ya), [*cb, M.Z_C], M._quat(yb), ta, tb2)

    def test_the_probe_miss(self):
        """Both boards turned 0.7 deg the same way about their grips, 115 mm from the seam each at the 30 mm gap: the
        ends 2.8 mm apart sideways, past the 1.2 mm capture, with no yaw between them (the probe: 1.9 mm just after the
        grip, 2.45 mm at the end face, yaws 0.35 to 1.2 deg, all the same sign)."""
        m = M.measure(*self.seat(ya=self.BIAS, yb=self.BIAS)[:4])
        lever = M.BOARD_L / 2 + M.GRIP_DY + M.PRE_GAP / 2
        self.assertAlmostEqual(abs(m["dx"]), 2 * lever * math.sin(self.BIAS), delta=2e-5)
        self.assertGreater(min(m["off"]), M.CAPTURE)
        self.assertLess(abs(m["yaw"]), 1e-9)

    def test_one_pass_lines_up(self):
        for ya, yb, dx in ((self.BIAS, self.BIAS, 0.0), (self.BIAS, -0.3 * self.BIAS, 0.003), (-0.01, 0.02, -0.004)):
            st = self.seat(ya=ya, yb=yb, b_dx=dx)
            st2 = self.apply(st, M.line_up(*st))
            m = M.measure(*st2[:4])
            self.assertLess(abs(m["dx"]), 1e-6, (ya, yb, dx))
            self.assertLess(abs(m["yaw"]), 1e-9)
            self.assertLess(abs(M._yaw_of(st2[1])), 1e-9)
            self.assertLess(max(m["off"]), 1e-6)

    def test_keeps_a_stems_offset(self):
        def ends(st):
            pa, qa, pb, qb = st[:4]
            end_a = np.asarray(pa) + M._rot3(qa) @ [0.0, M.BOARD_L / 2, 0.0]
            end_b = np.asarray(pb) + M._rot3(qb) @ [0.0, -M.BOARD_L / 2, 0.0]
            return end_b[0] - end_a[0]
        for b_dx, b_yaw in ((M.PITCH, 0.0), M.OFF, (0.0, M.SKEW)):
            want = ends(self.seat(yb=b_yaw, b_dx=b_dx))          # the stem's own end offset, no grip error
            st = self.seat(ya=self.BIAS, yb=b_yaw + self.BIAS, b_dx=b_dx)
            st2 = self.apply(st, M.line_up(*st, b_dx=want, b_yaw=b_yaw))
            qb = st2[3]
            self.assertAlmostEqual(ends(st2), want, delta=1e-6)
            self.assertAlmostEqual(M._yaw_of(qb), b_yaw, delta=1e-9)

    def test_clipped(self):
        st = self.seat(yb=math.radians(10.0), b_dx=0.020)
        d = M.line_up(*st)
        self.assertLessEqual(abs(d["left"][0]), M.LINE_UP_MAX[0])
        self.assertLessEqual(abs(d["left"][1]), M.LINE_UP_MAX[1])

    def test_relay_follows_the_loose_board(self):
        """The right hand has let go: only the left moves, and turns its board to the dowel board's yaw."""
        st = self.seat(gap=0.0, ya=self.BIAS, yb=0.0)
        d = M.line_up(*st, turn_a=False)
        self.assertNotIn("right", d)
        st2 = self.apply(st, d)
        m = M.measure(*st2[:4])
        self.assertLess(abs(m["yaw"]), 1e-9)
        self.assertLess(abs(m["dx"]), 2e-5)

    def test_off_probe_inside_capture_skew_outside(self):
        """Where the dowel tips meet the hole board's end plane (gap = the 20 mm they stand out): the off probe's
        tips within 0.6 mm of their holes, inside the 1.2 mm capture; the skew stem's all 6 mm out."""
        def offs(variant):
            w = [x for x in M.plan(0, variant)["left"] if x.label == "push the joint closed slowly"][0]
            yaw = w.yaw - M.YAW_L
            cx, cy = w.pos[0] + M.GRIP_DY * math.sin(yaw), w.pos[1] - M.GRIP_DY * math.cos(yaw)  # grip_point inverted
            cy = M.SEAM_Y + M.DOWEL_OUT + M.BOARD_L / 2 * math.cos(yaw)   # the end 20 mm off the dowel board's end
            return M.measure([M.X_LINE, M.SEAM_Y - M.BOARD_L / 2, M.Z_C], Q0, [cx, cy, M.Z_C], M._quat(yaw))["off"]
        o = offs("off")
        self.assertLess(max(o), 0.0006)
        self.assertGreater(max(o), 0.0004)
        self.assertGreater(min(offs("skew")), 0.006)

    def test_plan_turns_the_hole_board_about_its_end(self):
        W = M.plan(0, "off")["left"]
        w = [x for x in W if x.label == "push the joint closed slowly"][0]
        yaw = w.yaw - M.YAW_L
        gx = w.pos[0]
        cx = gx + M.GRIP_DY * math.sin(yaw)                  # grip_point inverted for the hole board
        self.assertAlmostEqual(cx + M.BOARD_L / 2 * math.sin(yaw), M.X_LINE + M.OFF[0], places=9)

    def test_shift_after_leaves_the_playing_segment(self):
        W = M.plan(0)["left"]
        t = M.label_ends(W, M.LINE_UP)[0] - 0.1
        W2 = M.shift_after(W, t, 0.002, 0.01)
        i = [w.label for w in W].index(M.LINE_UP[0])
        self.assertEqual(W2[:i + 1], W[:i + 1])
        for a, b in zip(W[i + 1:], W2[i + 1:]):
            self.assertAlmostEqual(b.pos[0] - a.pos[0], 0.002)
            self.assertAlmostEqual(b.yaw - a.yaw, 0.01)
            self.assertEqual((a.pos[1:], a.duration, a.finger), (b.pos[1:], b.duration, b.finger))

    def test_passes_before_the_push(self):
        for v in M.SERVO_VARIANTS:
            M.plan(0, v)
            W = M.plan(0, v)["left"]
            times = M._SERVO["times"]
            self.assertEqual(len(times), 1 + len(M.SHUFFLE_GAPS), v)
            self.assertLess(times[-1], M.label_ends(W, ("push the joint closed slowly",))[0] - 4.0)
            self.assertEqual(times, sorted(times))
            self.assertTrue(M._SERVO["on"])
        for v in ("nothing", "left_only", "stack"):
            M.plan(0, v)
            self.assertFalse(M._SERVO["on"], v)

    def test_servo_runs_each_pass_once(self):
        class P:
            def __init__(self, w):
                self.waypoints = w
        p = M.plan(0)
        plans = {a: P(list(w)) for a, w in p.items()}
        st = self.seat(ya=self.BIAS, yb=self.BIAS)
        M._M["boards"], M._M["tcps"] = tuple(st[:4]), {"right": st[4], "left": st[5]}
        try:
            t0 = M._SERVO["times"][0]
            M.servo(plans, t0 - 0.05)
            self.assertEqual(M._SERVO["done"], set())
            M.servo(plans, t0 + 0.01)
            once = [w.yaw for w in plans["left"].waypoints]
            M.servo(plans, t0 + 0.02)
            self.assertEqual([w.yaw for w in plans["left"].waypoints], once)
            self.assertEqual(M._SERVO["done"], {0})
            self.assertAlmostEqual(once[-1] - p["left"][-1].yaw, -self.BIAS, places=9)
        finally:
            M.reset_memory()

    def test_every_later_pass_follows_a_slide(self):
        """Between one pass and the next both hands back off past their friction band (SHUFFLE / 2, past the 3.3 mm band) and
        come in again, so the correction lands while the board slides; the last come-ins are commanded LAG closer than
        the 22 mm the boards should stop at (tips 2 mm short of the mouths)."""
        p = M.plan(0)
        for arm, W in p.items():
            labels = [w.label for w in W]
            for k in range(1, len(M.LINE_UP)):
                i = labels.index(M.LINE_UP[k])
                back, come = W[i - 2], W[i - 1]
                self.assertAlmostEqual(abs(back.pos[1] - come.pos[1]), (M.SHUFFLE + M.LAG) / 2, places=9)
                self.assertGreater(M.SHUFFLE / 2, M.LAG / 2 + 0.002)
                self.assertEqual(come.pos, W[i].pos)
            last = W[labels.index(M.LINE_UP[-1])]
            seat_g = M.grip_point(("hole_board" if arm == "left" else "dowel_board"),
                                  *M.seat_centres()["hole_board" if arm == "left" else "dowel_board"], 0.0)
            self.assertAlmostEqual(abs(last.pos[1] - seat_g[1]), (M.MOUTH_GAP - M.LAG) / 2, places=9)
        self.assertGreaterEqual(M.MOUTH_GAP - M.DOWEL_OUT, 0.002 - 1e-12)

    def test_last_pass_leaves_the_dowel_board(self):
        class P:
            def __init__(self, w):
                self.waypoints = w
        p = M.plan(0)
        plans = {a: P(list(w)) for a, w in p.items()}
        st = self.seat(gap=M.MOUTH_GAP, ya=self.BIAS, yb=self.BIAS)
        M._M["boards"], M._M["tcps"] = tuple(st[:4]), {"right": st[4], "left": st[5]}
        try:
            M.servo(plans, M._SERVO["times"][-1] + 0.01)    # every pass due at once: all run, the last one relative
            self.assertEqual(len(M._SERVO["done"]), len(M.LINE_UP))
            self.assertNotIn("right", M._SERVO["log"][-1]["cmd"])
            self.assertIn("right", M._SERVO["log"][0]["cmd"])
        finally:
            M.reset_memory()


class Demo(unittest.TestCase):
    def test_fits_run_limit(self):
        for s in range(3):
            p = M.plan(s)
            self.assertLess(max(sum(w.duration for w in W) for W in p.values()) + 1.0, M.RUN_LIMIT_STEPS / 20)
        for v in Stems.EXPECT:   # every stem too, the relay's let-go well inside the limit
            p = M.plan(0, v)
            for W in p.values():
                letgo = M.label_ends(W, ("let go",))
                self.assertLess((letgo[0] if letgo else 0.0) + 2.0, M.RUN_LIMIT_STEPS / 20, v)

    def test_hands_apart(self):
        """The TCPs stay at least 120 mm apart in y through the demo (G11)."""
        from yam_arena import station
        from yam_arena.demo import ArmPlan
        p = M.plan(0)
        L, R = ArmPlan(p["left"], station.BASES["left"]), ArmPlan(p["right"], station.BASES["right"])
        t = 0.0
        while t < min(L.duration, R.duration):
            self.assertGreater(L.target(t)[0][1] - R.target(t)[0][1], 0.120)
            t += 0.1

    def test_ik(self):
        from yam_arena import station
        from yam_arena.demo import ArmPlan
        for s in range(3):
            for arm, W in M.plan(s).items():
                p = ArmPlan(W, station.BASES[arm])
                t = 0.0
                while t <= p.duration:
                    p.action7(t)
                    t += 0.1
                self.assertLess(p.worst_ik_mm, 0.5)
                self.assertLess(p.worst_ik_deg, 0.5)

    def test_press_from_stiffness(self):
        self.assertAlmostEqual(M.PRESS, M.F_PRESS / M.K_Y)


class Text(unittest.TestCase):
    def test_task_json_numbers_from_code(self):
        d = json.loads((HERE / "task.json").read_text())
        mm = lambda v: f"{v * 1000:.2f}".rstrip("0").rstrip(".")  # noqa: E731
        for v in (M.SEAT_GAP, M.FLUSH, M.LEVEL, M.HALF_GAP, M.TIP_IN, M.CAPTURE, M.PAD_CLEAR, M.OVERLAP):
            self.assertIn(f"{mm(v)} mm", d["scoring"])
        self.assertIn(f"{mm(M.CLR)} mm", d["fidelity"])
        self.assertIn(f"{M.RUN_LIMIT_STEPS:,} steps", d["run_limit"])
        self.assertEqual(d["instruction"], " ".join(M.__doc__.split()))
        self.assertIn("move both hands clear", d["instruction"])   # 0.9 and 1.0 need the pads PAD_CLEAR off
        self.assertNotIn("no dry-fit friction", d["fidelity"])

    def test_readme_numbers_from_code(self):
        """The README's scoring and demo numbers match the code."""
        r = " ".join((HERE / "README.md").read_text().split())
        mm = lambda v: f"{v * 1000:.2f}".rstrip("0").rstrip(".")  # noqa: E731
        g = lambda v: f"{v:g}"  # noqa: E731
        want = [f"at least {mm(M.OVERLAP)} mm", f"height step within {mm(M.LEVEL)} mm",
                f"within {g(M.FLAT_DEG)} deg of flat", f"at least {mm(M.TIP_IN)} mm into its own hole",
                f"within {mm(M.CAPTURE)} mm of its axis", f"at most {mm(M.HALF_GAP)} mm",
                f"at most {mm(M.SEAT_GAP)} mm", f"flush within {mm(M.FLUSH)} mm",
                f"at least {mm(M.PAD_CLEAR)} mm from both boards", f"under {mm(M.STILL)} mm/s",
                f"held {g(M.SUCCESS_HOLD_S)} s", f"only {mm(-M.OPEN - M.BOARD_W / 2)} mm off it",
                f"slide both to a {mm(M.PRE_GAP)} mm gap", f"backing the gap off {mm(M.SHUFFLE)} mm "
                f"({mm(M.SHUFFLE / 2)} mm a hand)",
                "coming in to " + ", ".join(mm(v) for v in M.SHUFFLE_GAPS[:-1]) + f" and {mm(M.SHUFFLE_GAPS[-1])} mm",
                f"tips {mm(M.MOUTH_GAP - M.DOWEL_OUT)} mm short of the holes",
                f"at most {mm(M.LINE_UP_MAX[0])} mm and {g(round(math.degrees(M.LINE_UP_MAX[1]), 6))} deg",
                f"commanded {mm(M.LAG)} mm closer", f"toward a {mm(M.PROBE_GAP)} mm gap",
                f"at least {mm(M.ENTERED)} mm into its own hole", f"back off to a {mm(M.RETRY_GAP)} mm gap "
                f"(commanded {mm(M.LAG)} mm wider)", f"up to {M.RETRIES} retries",
                f"{M.BENCH_MU[0]:.2f} static", f"at {M.SLIDE_N:.2f} N",
                f"{2000 * M.SLIDE_N / M.K_Y:.1f} mm [ARITH: 2 x {M.SLIDE_N:.2f} N / {g(M.K_Y)} N/m]",
                f"{g(M.TABLE_MU[0])} and {g(M.TABLE_MU[1])}"]
        for w in want:
            self.assertTrue(w in r, w)
        for v in Stems.EXPECT:
            self.assertIn("`t00`" if v == "demo" else f"`s_{v}`", r)
        # WHY: before the sim runs each placeholder appears once; after finish fills them, none remain
        counts = {k: r.count(k) for k in ("STEMS_PENDING", "SEEDS_PENDING", "FILM_PENDING", "DREAM_PENDING")}
        self.assertIn(set(counts.values()), ({1}, {0}), counts)


if __name__ == "__main__":
    unittest.main()
