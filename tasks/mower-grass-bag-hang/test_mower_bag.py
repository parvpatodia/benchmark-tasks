"""Offline tests for mower_bag.py: geometry against the sources, the finite-rod hook check, every rung's bounds, each
stem's expected end state, the two-arm need, the demo's timing and clearances, and the text against the code. No
simulator: yam_arena on PYTHONPATH (pure Python parts only).

    PYTHONPATH=<yam_arena source> python3 test_mower_bag.py
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
from yam_arena.task_file import load  # noqa: E402

TF = load(HERE / "mower_bag.py")
M = TF.module
Q0 = (0.0, 0.0, 0.0, 1.0)
FAR = [np.array([0.0, 0.30, M.TOP + 0.30]), np.array([0.0, -0.30, M.TOP + 0.30])]   # both hands parked away


class S:
    """A minimal object state."""

    def __init__(self, pos, quat=Q0, vel=(0.0, 0.0, 0.0), spin=(0.0, 0.0, 0.0)):
        self.pos, self.quat_xyzw = np.asarray(pos, float), np.asarray(quat, float)
        self.lin_vel, self.ang_vel = np.asarray(vel, float), np.asarray(spin, float)


class MS(S):
    def __init__(self, pos, joints, links):
        super().__init__(pos)
        self.joints, self.links = joints, links


def qy(rad):
    return (0.0, math.sin(rad / 2), 0.0, math.cos(rad / 2))


def qz(rad):
    return (0.0, 0.0, math.sin(rad / 2), math.cos(rad / 2))


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def mower(roll=0.0, theta=None, flap_spin=0.0):
    """The mower rolled `roll`, its flap at `theta` (default: resting on the hung bag)."""
    th = M.REST if theta is None else theta
    pm = np.array([M.X_M + roll, 0.0, M.TOP])
    h = np.array([M.HINGE[0], 0.0, M.HINGE[1]])
    pf = pm + h - M._ry(th) @ h
    links = {"mower": S(pm), "flap": S(pf, qy(th), spin=(0.0, flap_spin, 0.0))}
    return MS(pm, {"roll": roll, "flap": th}, links)


def hung(roll=0.0, d=(0.0, 0.0, 0.0), yaw=0.0, vel=0.0):
    """The bag hung as the scorer expects, moved d (world) and turned yaw about its rod centre."""
    p, q, _, _ = M.hung_pose(roll)
    return S(p + np.asarray(d, float), qmul(qz(yaw), q), (vel, 0.0, 0.0))


def objs(bag=None, **kw):
    return {"mower": mower(**kw), "bag": hung() if bag is None else bag}


def score_once(o, pads=FAR, dt=0.05):
    M.reset_memory()
    M._M["pads"] = pads
    M._CLOCK["dt"] = dt
    return M.score(o)


def score_held(make, pads=FAR, steps=21, dt=0.05):
    """Score after `steps` control steps of the same end state (fresh dicts, so each counts as a step)."""
    M.reset_memory()
    M._M["pads"] = pads
    M._CLOCK["dt"] = dt
    s = None
    for _ in range(steps):
        s = M.score(make())
    return s


def bar_world(bag):
    return bag.pos + M._rot3(bag.quat_xyzw) @ np.array([M.HANDLE_X, 0.0, M.BAR_Z])


class Sources(unittest.TestCase):
    def test_manual_numbers(self):
        self.assertEqual(M.MOWER_KG, 13.2)                                  # "29 lbs (13.2 kg)"
        self.assertAlmostEqual(2 * M.WHEEL_R, 0.14)                          # "5.5 in. (14 cm)"
        self.assertLess(abs(M.BAG_L * M.BAG_W * M.BAG_H * 1000 - 40.0), 1.0)  # "10.6 gal (40 l)"

    def test_masses(self):
        self.assertAlmostEqual(M.BAG.mass, M.BAG_KG, places=6)
        self.assertAlmostEqual(M.ROD_KG, 7850 * math.pi * M.ROD_R ** 2 * M.ROD_L, places=3)
        self.assertAlmostEqual(sum(link.mass for link in M.MOWER.links), M.MOWER_KG, places=6)

    def test_handle_over_centre_of_mass(self):
        m = sum(s.mass for s in M.BAG.shapes)
        cx = sum(s.mass * s.center[0] for s in M.BAG.shapes) / m
        self.assertLess(abs(cx - M.HANDLE_X), 1e-9)

    def test_mower_fits_the_bench(self):
        hi = max(s.bounds()[1][0] for s in M.MOWER.links[0].shapes if s.collide) + M.X_M
        self.assertLessEqual(hi, 0.36 + 1.10 / 2 + 0.001)                  # station table's far edge
        lo_wheel = min(s.bounds()[0][2] for s in M.MOWER.links[0].shapes if s.kind == "cylinder" and s.collide)
        self.assertGreater(lo_wheel, 0.0)                                     # the slide carries it, wheels clear


class Geometry(unittest.TestCase):
    def test_slot_takes_the_rod_with_play(self):
        width = M.SLOT_X[1] - M.SLOT_X[0]
        self.assertAlmostEqual(width, 0.021)
        self.assertAlmostEqual((width - 2 * M.ROD_R) / 2, 0.0055)          # 5.5 mm a side, README

    def test_lift_clears_the_lips(self):
        self.assertAlmostEqual(M.LIFT_Z - M.ROD_R - (M.FLOOR_Z + M.LIP_H), 0.012)

    def test_rod_spans_both_hooks_with_margin(self):
        reach = M.ROD_L / 2 - (M.HOOK_Y + M.HOOK_W / 2)
        self.assertAlmostEqual(reach, 0.062)                                 # 62 mm past each hook's outer side

    def test_hung_bag_rests_on_its_tail(self):
        _, _, tilt, which = M.hung_pose()
        self.assertEqual(which, "bench")
        self.assertTrue(math.radians(3.0) < tilt < math.radians(6.0))

    def test_flap_rests_on_the_cover(self):
        self.assertIsNotNone(M.REST)
        self.assertTrue(math.radians(45) < M.REST < math.radians(70))
        p, q, _, _ = M.hung_pose()
        gap, _, on = M.flap_edge_gap(M.REST, p, q, np.eye(3), np.array([M.X_M, 0.0, M.TOP]))
        self.assertTrue(on)
        self.assertLess(abs(gap), 0.0005)


class TwoArms(unittest.TestCase):
    def test_closed_flap_lip_sits_inside_the_hung_bag(self):
        """The real blocker: closed, the flap's lip lies inside the space the hung bag's front fills, so the bag cannot
        hang with the flap shut. The rod alone is not blocked: a 15 mm channel between the plate and the hooks' lips
        takes the 10 mm rod, and the rod's side arms pass outside the flap."""
        hx, hz = M.HINGE
        lip = M.flap_shapes()[1]
        lo, hi = lip.bounds()
        pts = np.array([(x, y, z) for x in np.linspace(lo[0], hi[0], 5) for y in np.linspace(lo[1], hi[1], 5)
                        for z in np.linspace(lo[2], hi[2], 3)])
        p, q, _, _ = M.hung_pose()
        Rb = M._rot3(q)
        inside = []
        for s in M.BAG_SHAPES[3:5]:                                          # the cover and the bag's body
            blo, bhi = s.bounds()
            b = (pts + np.array([M.X_M, 0.0, M.TOP]) - p) @ Rb
            inside.append(np.any(np.all((b > blo) & (b < bhi), axis=1)))
        self.assertTrue(any(inside))
        channel = (M.SLOT_X[0] - M.LIP_T) - hx
        self.assertAlmostEqual(channel, 0.015)
        self.assertGreater(channel, 2 * M.ROD_R)
        self.assertGreater(M.ROD_L / 2 - 0.015, M.FLAP_W / 2)                # the rod's side arms outside the flap
        self.assertLess(hz - M.FLAP_L, M.FLOOR_Z - 0.025)                    # the plate reaches 30 mm below the floor

    def test_flap_closes_from_every_angle(self):
        for deg in np.arange(0.0, math.degrees(M.FLAP_MAX) + 0.1, 1.0):
            th = math.radians(deg)
            spring = M.FLAP_K * (th + M.FLAP_PRE)                            # closing (toward 0)
            gravity = M.FLAP_KG * 9.81 * (M.FLAP_L / 2) * math.sin(th)      # closing: the CoM rises as it opens
            self.assertGreater(spring + gravity, 0.19)

    def test_one_arm_stem_has_no_left_plan(self):
        self.assertEqual(set(M.plan(0, "one_arm")), {"right"})
        self.assertEqual(set(M.plan(0, "lift")), {"right"})

    def test_lift_route_geometry(self):
        """s_lift, the one-hand route: tipped nose-down, the bag clears the bench; the rod goes in under the lip and
        rises between the plate and the hooks' lips."""
        Rt = M._ry(M.LIFT_TIP)                                               # nose down
        corners = [Rt @ np.array([x, 0.0, M.BOT_Z]) for x in (M.FRONT, M.FRONT - M.BAG_L)]
        self.assertGreater(M.LIFT_LOW + min(c[2] for c in corners), 0.0)
        self.assertLess(M.LIFT_LOW + M.ROD_R, M.HINGE[1] - M.FLAP_L)         # under the plate's foot and the lip
        self.assertGreater(M.LIFT_IN - M.ROD_R, M.HINGE[0])                   # inside the closed plate
        self.assertLess(M.LIFT_IN + M.ROD_R, M.SLOT_X[0] - M.LIP_T)          # outside the hooks' lips
        corner = Rt @ np.array([M.FRONT, 0.0, M.TOP_Z])
        self.assertGreater(M.LIFT_IN + corner[0], M.HINGE[0] - M.FLAP_T - M.FLAP_LIP)   # cover corner under the lip
        self.assertLess(M.LIFT_LOW + corner[2], M.HINGE[1] - M.FLAP_L)


class FiniteRod(unittest.TestCase):
    """The reviewer's point: the actual finite rod, never its axis extended past its ends."""

    def _hooks(self, bag):
        Rm, pm = np.eye(3), np.array([M.X_M, 0.0, M.TOP])
        c, u = M.rod_in_mower(bag.pos, bag.quat_xyzw, Rm, pm)
        return {s: M.hook_check(c, u, s)["ok"] for s in (1, -1)}, c, u

    def test_hung_rod_passes_both(self):
        self.assertEqual(self._hooks(hung())[0], {1: True, -1: True})

    def test_one_hook_stem_fails_the_far_hook_only(self):
        h, c, u = self._hooks(hung(d=(0.0, M.ONE_HOOK_DY, 0.0)))
        self.assertEqual(h, {1: False, -1: True})
        # the rod still crosses the left hook's inner side: its end is inside the hook, not short of it
        end = c[1] + M.ROD_L / 2
        self.assertTrue(M.HOOK_Y - M.HOOK_W / 2 < end < M.HOOK_Y + M.HOOK_W / 2)

    def test_extended_axis_would_have_passed(self):
        """The same bag, checked on the rod's axis without the |t| <= L/2 bound, passes: the bound is what fails it."""
        _, c, u = self._hooks(hung(d=(0.0, M.ONE_HOOK_DY, 0.0)))
        ok = True
        for ys in (M.HOOK_Y - M.HOOK_W / 2 - M.SPAN_MARGIN, M.HOOK_Y, M.HOOK_Y + M.HOOK_W / 2 + M.SPAN_MARGIN):
            p = c + (ys - c[1]) / u[1] * u
            ok = ok and M.SLOT_X[0] + M.ROD_R - M.SLOT_SLACK <= p[0] <= M.SLOT_X[1] - M.ROD_R + M.SLOT_SLACK
        self.assertTrue(ok)

    def test_span_margin_edge(self):
        far = M.HOOK_Y + M.HOOK_W / 2 + M.SPAN_MARGIN                        # 93 mm
        just_in = M.ROD_L / 2 - far - 0.001
        just_out = M.ROD_L / 2 - far + 0.001
        self.assertEqual(self._hooks(hung(d=(0.0, just_in, 0.0)))[0][-1], True)
        self.assertEqual(self._hooks(hung(d=(0.0, just_out, 0.0)))[0][-1], False)

    def test_rod_across_lip_tops_fails(self):
        bag = hung(d=(-M.LIP_T / 2 - M.ROD_R, 0.0, M.LIP_H))                 # centre over the lips' middle, on top
        self.assertEqual(self._hooks(bag)[0], {1: False, -1: False})

    def test_rod_lifted_off_the_floor_fails(self):
        self.assertEqual(self._hooks(hung(d=(0.0, 0.0, 0.001)))[0], {1: True, -1: True})
        self.assertEqual(self._hooks(hung(d=(0.0, 0.0, 0.002)))[0], {1: False, -1: False})
        self.assertEqual(self._hooks(hung(d=(0.0, 0.0, -0.0004)))[0], {1: True, -1: True})

    def test_rod_out_of_the_slot_fails(self):
        self.assertEqual(self._hooks(hung(d=(-0.004, 0.0, 0.0)))[0], {1: False, -1: False})   # over the lip
        self.assertEqual(self._hooks(hung(d=(0.014, 0.0, 0.0)))[0], {1: False, -1: False})    # into the face

    def test_slot_slack_edge(self):
        self.assertEqual(self._hooks(hung(d=(-0.0019, 0.0, 0.0)))[0], {1: True, -1: True})
        self.assertEqual(self._hooks(hung(d=(-0.0021, 0.0, 0.0)))[0], {1: False, -1: False})
        top = M.SLOT_X[1] - M.ROD_R - (M.SLOT_X[0] + M.ROD_R)                # across to the rear face
        self.assertEqual(self._hooks(hung(d=(top + 0.0019, 0.0, 0.0)))[0], {1: True, -1: True})
        self.assertEqual(self._hooks(hung(d=(top + 0.0021, 0.0, 0.0)))[0], {1: False, -1: False})

    def test_rod_turned_past_sixty_fails(self):
        u = np.array([math.sin(math.radians(61.0)), math.cos(math.radians(61.0)), 0.0])
        c = np.array([M.SLOT_X[0] + M.ROD_R, 0.0, M.FLOOR_Z + M.ROD_R])
        self.assertFalse(M.hook_check(c, u, 1)["ok"])

    def test_rod_turned_in_plan(self):
        self.assertEqual(self._hooks(hung(yaw=math.radians(1.0)))[0], {1: True, -1: True})
        self.assertEqual(self._hooks(hung(yaw=math.radians(8.0)))[0][1] and
                         self._hooks(hung(yaw=math.radians(8.0)))[0][-1], False)


class Ladder(unittest.TestCase):
    def test_full_fit_scores_one_after_a_second(self):
        self.assertEqual(score_once(objs()), 0.9)
        self.assertEqual(score_held(objs), 1.0)
        self.assertEqual(score_held(objs, steps=19), 0.9)                    # 0.95 s: not yet
        self.assertEqual(score_held(objs, steps=20), 1.0)                    # 20 steps of 0.05 s

    def test_nothing_done(self):
        bag = S((M.BAG_START[0], 0.0, M.Z_LYING))
        self.assertEqual(score_once(objs(bag=bag, theta=0.0)), 0.0)

    def test_bag_held(self):
        b = hung()
        self.assertEqual(score_held(objs, pads=[bar_world(b), FAR[0]]), 0.25)

    def test_pad_clearance_edge(self):
        b = hung()
        top = b.pos + M._rot3(b.quat_xyzw) @ np.array([M.HANDLE_X, 0.0, M.BAR_Z + M.BAR_R])
        near = [top + np.array([0.0, 0.0, M.PAD_CLEAR - 0.001]), FAR[0]]
        clear = [top + np.array([0.0, 0.0, M.PAD_CLEAR + 0.001]), FAR[0]]
        self.assertEqual(score_held(objs, pads=near), 0.25)
        self.assertEqual(score_held(objs, pads=clear), 1.0)

    def test_bag_moving(self):
        self.assertEqual(score_held(lambda: objs(bag=hung(vel=0.006))), 0.25)
        self.assertEqual(score_held(lambda: objs(bag=hung(vel=0.004))), 1.0)

    def test_flap_held_open(self):
        self.assertEqual(score_held(lambda: objs(theta=M.FLAP_UP)), 0.5)

    def test_flap_on_a_gripper(self):
        self.assertEqual(score_held(lambda: objs(theta=M.UNDER)), 0.5)

    def test_flap_gap_edge(self):
        p, q, _, _ = M.hung_pose()
        th = M.REST
        while M.flap_edge_gap(th, p, q, np.eye(3), np.array([M.X_M, 0.0, M.TOP]))[0] < M.FLAP_GAP - 0.0005:
            th += math.radians(0.02)
        self.assertEqual(score_held(lambda: objs(theta=th)), 1.0)
        th2 = th + math.radians(0.5)
        self.assertGreater(M.flap_edge_gap(th2, p, q, np.eye(3), np.array([M.X_M, 0.0, M.TOP]))[0], M.FLAP_GAP)
        self.assertEqual(score_held(lambda: objs(theta=th2)), 0.5)

    def test_hand_on_the_flap(self):
        hx, hz = M.HINGE   # a pad 10 mm off the flap's outer face, 30 mm from the hinge: on the flap, clear of the bag
        pad = np.array([M.X_M + hx, 0.0, M.TOP + hz]) + M._ry(M.REST) @ np.array([-M.FLAP_T - 0.010, 0.0, -0.030])
        b = hung()
        self.assertGreater(M.dist_to_shapes(pad, M._rot3(b.quat_xyzw), b.pos, M.BAG_SHAPES), M.PAD_CLEAR)
        self.assertEqual(score_held(objs, pads=[pad, FAR[0]]), 0.75)

    def test_flap_still_moving(self):
        self.assertEqual(score_held(lambda: objs(flap_spin=0.1)), 0.75)

    def test_flap_edge_dig(self):
        """The edge may sit up to 3 mm into the cover (contact offset); at 2 mm it scores, at 4 mm it does not."""
        p, q, _, _ = M.hung_pose()
        pm = np.array([M.X_M, 0.0, M.TOP])

        def at(gap):
            th = M.REST
            while M.flap_edge_gap(th, p, q, np.eye(3), pm)[0] > gap:
                th -= math.radians(0.01)
            return th
        self.assertEqual(score_held(lambda: objs(theta=at(-0.002))), 1.0)
        self.assertEqual(score_held(lambda: objs(theta=at(-0.0028))), 1.0)
        self.assertEqual(score_held(lambda: objs(theta=at(-0.004))), 0.5)

    def test_bag_turning(self):
        def spun(w):
            b = hung()
            b.ang_vel = np.array([0.0, w, 0.0])
            return objs(bag=b)
        self.assertEqual(score_held(lambda: spun(0.06)), 0.25)
        self.assertEqual(score_held(lambda: spun(0.04)), 1.0)

    def test_upside_down_bag_scores_nothing(self):
        p, q, _, _ = M.hung_pose()
        flip = qmul(q, (1.0, 0.0, 0.0, 0.0))                                 # turned 180 deg about its x
        self.assertEqual(score_held(lambda: objs(bag=S(p, flip))), 0.0)
        self.assertEqual(score_held(lambda: objs(bag=S(p, qmul(q, (math.sin(math.radians(20)), 0.0, 0.0,
                                                                     math.cos(math.radians(20))))))) > 0.0, False)

    def test_housing_near_the_bag_counts(self):
        """The whole gripper counts, not only its pads: housing 10 mm over the cover, pad faces 40 mm or more away."""
        b = hung()
        Rb = M._rot3(b.quat_xyzw)
        # TCP frame in B: approach along +y, fingers closing along z, pad axis along x
        Rh = np.column_stack([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        tip = M.PAD_TIP - M.TCP_OFFSET[2]
        tcp_b = np.array([-0.40, 0.10 - tip, M.TOP_Z + 0.010 + M.HAND_HOUSING[2] / 1000])
        R = Rb @ Rh
        tcp = b.pos + Rb @ tcp_b
        g = tcp - R @ np.array(M.TCP_OFFSET)
        from yam_arena import ik
        pts = M.hand_points(g, ik.matrix_to_quat_xyzw(R), 0.0)
        self.assertAlmostEqual(M.points_to_shapes(pts, Rb, b.pos, M.BAG_SHAPES), 0.010, places=4)
        old = [tcp + R @ np.array([0.0, 0.0, z - M.TCP_OFFSET[2]]) for z in (M.PAD_TIP, M.PAD_TOP)]
        self.assertGreater(min(M.dist_to_shapes(x, Rb, b.pos, M.BAG_SHAPES) for x in old), 0.040)
        self.assertEqual(score_held(objs, pads=np.vstack([pts, FAR])), 0.25)

    def test_nudge_keeps_full_score(self):
        for r in (0.005, 0.008, 0.010, 0.025, 0.059, -0.06):
            self.assertEqual(score_held(lambda: objs(roll=r, bag=hung(roll=r))), 1.0, r)

    def test_shove_caps(self):
        for r in (0.061, 0.075):
            self.assertEqual(score_held(lambda: objs(roll=r, bag=hung(roll=r))), 0.75, r)

    def test_roll_limit_is_the_bench_edge(self):
        self.assertAlmostEqual(M.BENCH_EDGE, 0.91)
        self.assertAlmostEqual(M.ROLL_FWD, 0.91 - (M.X_M + M.FRONT_WHEEL_X))
        self.assertAlmostEqual(M.ROLL_SAFE, 0.060)
        roll = [j for j in M.MOWER.joints if j.name == "roll"][0]
        self.assertAlmostEqual(roll.upper, M.ROLL_FWD)
        self.assertAlmostEqual(roll.friction, 3.9)

    def test_mower_moved_without_the_bag_fails_the_seat(self):
        # the bag stays where it was while the mower rolls 20 mm away: the rod is no longer in the slots
        self.assertEqual(score_once(objs(roll=0.020)), 0.0)

    def test_bag_on_the_ground(self):
        bag = S((M.X_M - 0.20, 0.0, M.Z_LYING))
        self.assertEqual(score_once(objs(bag=bag, theta=0.0)), 0.0)

    def test_unread_hands_count_as_touching(self):
        self.assertEqual(score_held(objs, pads=None), 0.25)

    def test_score_is_final_state_only(self):
        """A bad step earlier (hands on the bag, the mower off its mark) does not lower the end score."""
        M.reset_memory()
        M._CLOCK["dt"] = 0.05
        M._M["pads"] = [bar_world(hung())]
        M.score(objs(roll=0.06, bag=hung(roll=0.06)))
        M._M["pads"] = FAR
        s = None
        for _ in range(21):
            s = M.score(objs())
        self.assertEqual(s, 1.0)

    def test_every_rung_value(self):
        self.assertEqual(M.LADDER, {0: 0.0, 1: 0.25, 2: 0.5, 3: 0.75})


class Stems(unittest.TestCase):
    """Each stem's planned end state, scored offline (the sim run is the check that it happens)."""

    def test_tips_stem_sets_the_rod_on_the_lips(self):
        W = M.plan(0, "tips")["right"]
        last = [w for w in W if w.label == "keep holding"][0]
        rod_x = last.pos[0] - M.HANDLE_X
        rod_z = last.pos[2] - M.TOP - M.BAR_Z
        self.assertAlmostEqual(rod_x - M.X_M, M.SLOT_X[0] - M.LIP_T / 2, places=6)
        self.assertAlmostEqual(rod_z, M.FLOOR_Z + M.LIP_H + M.ROD_R, places=6)

    def test_pinch_stem_parks_under_the_flap(self):
        W = M.plan(0, "pinch")["right"]
        under = np.array([w for w in W if w.label == "stay there"][0].pos)
        hx, hz = M.HINGE
        r = math.hypot(under[0] - M.X_M - hx, under[2] - M.TOP - hz)
        self.assertAlmostEqual(r, M.UNDER_R, places=6)
        self.assertGreater(M.PINCH_LET, M.UNDER)                              # let go above it, so it lands on it
        t_under = sum(w.duration for w in W if w.label != "stay there")
        t_let = 0.0
        for w in M.plan(0, "pinch")["left"]:
            if w.label == "let go of the flap":
                break
            t_let += w.duration
        self.assertGreater(t_let, t_under)                                    # parked before the flap is let go
        # the whole closed gripper stays 30 mm clear of the bag: the score is 0.5, not 0.25
        w = [w for w in W if w.label == "stay there"][0]
        from yam_arena import ik
        g = under - w.yaw.R @ np.array(M.TCP_OFFSET)
        pts = M.hand_points(g, ik.matrix_to_quat_xyzw(w.yaw.R), w.finger)
        b = hung()
        # 38 mm as planned; the unload pass lifts the hand's later waypoints by about SEAT_PUSH, so more in the sim
        self.assertGreater(M.points_to_shapes(pts, M._rot3(b.quat_xyzw), b.pos, M.BAG_SHAPES), M.PAD_CLEAR + 0.005)

    def test_variant_times_fit_the_run_limit(self):
        limit = M.RUN_LIMIT_STEPS / 20.0
        for v in ("demo", "one_arm", "lift", "one_hook", "tips", "ground", "hold", "prop", "pinch", "slam", "nudge",
                  "shove"):
            for arm, W in M.plan(0, v).items():
                self.assertLess(sum(w.duration for w in W) + M.SUCCESS_HOLD_S, limit - 5.0, (v, arm))

    def test_demo_left_lets_go_after_the_right_is_clear(self):
        p = M.plan(0, "demo")
        t_right = sum(w.duration for w in p["right"])
        t, t_lower = 0.0, None
        for w in p["left"]:
            if w.label == "lower the flap" and t_lower is None:
                t_lower = t
            t += w.duration
        self.assertGreater(t_lower, t_right)

    def test_nudge_and_shove_push_while_holding(self):
        for v, dist in (("nudge", M.NUDGE), ("shove", M.SHOVE)):
            W = M.plan(0, v)["right"]
            labels = [w.label for w in W]
            i = labels.index("roll the mower")
            self.assertLess(i, labels.index("open round the bar"))
            self.assertAlmostEqual(W[i].pos[0] - W[i - 1].pos[0], dist + M.PUSH_LAG, places=9)
        self.assertTrue(0.005 <= M.NUDGE <= 0.010)
        self.assertGreater(M.SHOVE, M.ROLL_SAFE + 0.005)
        self.assertLess(M.SHOVE + M.PUSH_LAG, M.ROLL_FWD - 0.005)

    def test_left_hand_moves_with_the_mower(self):
        """s_nudge and s_shove: while the right hand rolls the mower, the left pinch moves with the flap, in step."""
        for v, roll in (("nudge", M.NUDGE), ("shove", M.SHOVE)):
            p = M.plan(0, v)
            t, t_r = 0.0, None
            for w in p["right"]:
                if w.label == "roll the mower":
                    t_r = t
                t += w.duration
            t, t_l, W = 0.0, None, p["left"]
            for k, w in enumerate(W):
                if w.label == "move with the mower":
                    t_l, i = t, k
                t += w.duration
            self.assertAlmostEqual(t_l, t_r, places=6)
            self.assertAlmostEqual(W[i].pos[0] - W[i - 1].pos[0], roll, places=9)
            demo = [w for w in M.plan(0, "demo")["left"] if w.label == "lower the flap"]
            mine = [w for w in W if w.label == "lower the flap"]
            self.assertAlmostEqual(mine[0].pos[0] - demo[0].pos[0], roll, places=9)

    def test_open_loop_grip_misses_some_seed(self):
        """Q2: the bag's seeded start moves the rod's ends past the slot's play, so one fixed plan cannot fit every
        seed. The bag is carried rigidly from where a seed-blind plan grips (the unseeded bar) to the seat."""
        Rm, pm = np.eye(3), np.array([M.X_M, 0.0, M.TOP])
        nominal = M.bar_tcp(M.BAG_START, 0.0, M.Z_LYING - M.TOP)
        fails = []
        for s in range(20):
            x, y, yaw = M.bag_layout(s)
            # the hand closes at `nominal`; the bar is captured in the closing plane, its offset along the bar and the
            # bag's yaw stay; carried with no turn to the seat
            bar = M.bar_tcp((x, y), yaw, M.Z_LYING - M.TOP)
            along = (bar - nominal)[1]
            p, q, _, _ = M.hung_pose()
            bag = S(p, q)
            seat_bar = bar_world(bag)
            rod = seat_bar + M._rz(yaw) @ (p - seat_bar) + np.array([0.0, along, 0.0])
            c, u = M.rod_in_mower(rod, qmul(qz(yaw), q), Rm, pm)
            fails.append(not all(M.hook_check(c, u, k)["ok"] for k in (1, -1)))
        self.assertTrue(any(fails))
        self.assertFalse(all(fails))


class Clearances(unittest.TestCase):
    def test_left_fingers_stay_off_the_cover_at_let_go(self):
        th = M.REST + M.LET_FLAP
        R = M.flap_rot(th).R
        g = M.pinch_point(th) - R @ np.array(M.TCP_OFFSET)
        from yam_arena import ik
        pts = M.hand_points(g, ik.matrix_to_quat_xyzw(R), M.PINCH_OPEN)
        b = hung()
        self.assertGreater(M.points_to_shapes(pts, M._rot3(b.quat_xyzw), b.pos, M.BAG_SHAPES), 0.010)

    def test_hands_apart_through_the_demo(self):
        p = M.plan(0, "demo")
        from yam_arena.demo import ArmPlan
        from yam_arena import station
        A = {arm: ArmPlan(W, station.BASES[arm]) for arm, W in p.items()}
        t, low = 0.0, 1e9
        while t < max(a.duration for a in A.values()):
            low = min(low, float(np.linalg.norm(A["left"].target(t)[0] - A["right"].target(t)[0])))
            t += 0.1
        self.assertGreater(low, 0.10)

    def test_left_pinch_clears_the_hook(self):
        tip_y = M.PINCH_Y - (M.PAD_TIP - M.TCP_OFFSET[2])
        self.assertAlmostEqual(tip_y - (M.HOOK_Y + M.HOOK_W / 2), 0.009, places=6)
        self.assertGreater(M.FLAP_W / 2 - (M.Y_BESIDE - (M.PAD_TIP - M.TCP_OFFSET[2])), -0.008)


class Sag(unittest.TestCase):
    """The demo's load compensation (mower_bag.commanded) against the joint-spring model of the arm."""

    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(HERE / "tools"))
        import sagcheck
        cls.S = sagcheck
        from yam_arena import station
        cls.base = np.asarray(station.BASES["right"], float)

    def _settled(self, w_plan, w_cmd):
        from yam_arena import ik
        from yam_arena.demo import grip_rotation
        R, Rc = grip_rotation(w_plan.yaw), grip_rotation(w_cmd.yaw)
        q0, _, _ = ik.solve(w_plan.pos, ik.matrix_to_quat_xyzw(R), None, base_pos=self.base, iters=300)
        qc, _, _ = ik.solve(w_cmd.pos, ik.matrix_to_quat_xyzw(Rc), q0, base_pos=self.base, iters=300)
        return (ik.tcp_pose(self.S.settle(q0, self.base), self.base)[0],
                ik.tcp_pose(self.S.settle(qc, self.base), self.base)[0])

    def test_model_sag_matches_the_traced_run(self):
        # the 704ea7f PhysX trace (t00, seed 0): TCP minus plan at "line up behind the hooks", before its pass, was
        # (-6.6, 11.9, -45.4) mm
        p = M.plan(1415158438, "demo")["right"]
        w = next(w for w in p if w.label == M.SERVO_LABELS[0])
        raw, _ = self._settled(w, w)
        err = 1000 * (raw - np.asarray(w.pos))
        for got, want in zip(err, (-6.6, 11.9, -45.4)):
            self.assertAlmostEqual(got, want, delta=3.0)

    def test_compensated_hand_lands_on_the_plan(self):
        p = M.plan(1415158438, "demo")
        c = M.commanded(p)
        n = 0
        for w, wc in zip(p["right"], c["right"]):
            if w.label in M.LOADED:
                n += 1
                _, fixed = self._settled(w, wc)
                self.assertLess(np.linalg.norm(fixed - np.asarray(w.pos)), 0.001, w.label)
            else:
                self.assertEqual(w, wc)
        self.assertEqual(n, 9)
        self.assertEqual(p["left"], c["left"])

    def test_height_pass_before_the_carry(self):
        labels = [w.label for w in M.plan(0, "demo")["right"]]
        i = labels.index(M.SERVO_LABELS[0])
        self.assertEqual(labels[i + 1:i + 3], [M.SERVO_LABELS[3], "carry over the slots"])
        self.assertIn(M.SERVO_LABELS[3], [lab for _, lab in M.servo_events(M.plan(0, "demo")["right"])])
        self.assertNotIn("lower into the hooks", M.LOADED)


class Text(unittest.TestCase):
    def setUp(self):
        self.task = json.loads((HERE / "task.json").read_text())
        self.readme = (HERE / "README.md").read_text()

    def test_scoring_text_numbers(self):
        s = self.task["scoring"]
        for n in (f"{M.SPAN_MARGIN * 1000:.0f} mm", f"{M.SEAT_ABOVE * 1000:.1f} mm", f"{M.SEAT_BELOW * 1000:.1f} mm",
                  f"{M.PAD_CLEAR * 1000:.0f} mm", f"{M.STILL * 1000:.0f} mm/s", f"{M.FLAP_GAP * 1000:.0f} mm",
                  f"{M.ROLL_SAFE * 1000:.0f} mm", f"{M.SLOT_SLACK * 1000:.0f} mm", f"{-M.FLAP_DIG * 1000:.0f} mm below",
                  f"{math.degrees(M.MAX_TURN):.0f} deg", f"{math.degrees(M.UPRIGHT):.0f} deg",
                  f"{M.BAG_SPIN:.2f} rad/s", "unread", "0.25", "0.5", "0.75", "0.9", "1.0",
                  f"{M.SUCCESS_HOLD_S:.0f} s"):
            self.assertIn(n, s, n)

    def test_run_limit(self):
        self.assertEqual(self.task["run_limit"], "1,200 steps (60 s at 20 Hz)")
        self.assertEqual(M.RUN_LIMIT_STEPS, 1200)

    def test_setup_numbers(self):
        s = self.task["setup"]
        for n in ("13.2 kg", f"{M.BAG_L * 1000:.0f} x {M.BAG_W * 1000:.0f} x {M.BAG_H * 1000:.0f} mm",
                  f"{M.ROD_L * 1000:.0f} mm", f"{2 * M.ROD_R * 1000:.0f} mm", f"{M.HOOK_Y * 2000:.0f} mm",
                  f"{M.FLOOR_Z * 1000:.0f} mm", f"{(M.SLOT_X[1] - M.SLOT_X[0]) * 1000:.0f} mm",
                  f"{M.LIP_H * 1000:.0f} mm", f"{M.FLAP_W * 1000:.0f} x {M.FLAP_L * 1000:.0f} mm",
                  f"{M.FLAP_PRE * M.FLAP_K:.2f} N m"):
            self.assertIn(n, s, n)

    def test_readme_rest_numbers(self):
        self.assertIn(f"{math.degrees(M.REST):.0f} deg", self.readme)
        self.assertIn(f"{math.degrees(M.hung_pose()[2]):.1f} deg", self.readme)

    def test_no_dashes_or_names(self):
        for f in HERE.rglob("*"):
            if f.is_file() and f.suffix in (".py", ".md", ".json", ".sh") and "arm" not in f.parts[-2:-1]:
                t = f.read_text()
                self.assertNotIn("\u2014", t, f.name)
                self.assertNotIn("\u2013", t, f.name)


if __name__ == "__main__":
    unittest.main(verbosity=1)
