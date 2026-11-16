"""Offline tests for nail_trim.py (no simulator): the cut model, the dot and the bleed, the scorer, the demo replayed
through the model on each film seed, and that no fixed cut length serves every claw and seed. Run with yam_arena on
PYTHONPATH."""
import math
import pathlib
import sys
import types
import unittest
import warnings
import zlib

import numpy as np

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parent
from yam_arena.task_file import load  # noqa: E402

M = load(HERE / "nail_trim.py").module
CLI = (0, 2, 4)
IDENT = (0.0, 0.0, 0.0, 1.0)
PAW_AT_REST = (M.T_POS[0], M.T_POS[1], M.TOP)
PAW_Q = M.T_QUAT                          # the paw link's orientation at rest (the trainer's)
HEAD = M.T_YAW + M.YAW_CUT                # the demo's clipper heading while cutting
CLIP_Q = (0.0, 0.0, math.sin(HEAD / 2), math.cos(HEAD / 2))


def trial_seed(cli):
    return zlib.crc32(f"{cli}:None:0".encode())


def _state(pos, lin_vel=(0.0, 0.0, 0.0)):
    return types.SimpleNamespace(pos=np.asarray(pos, float), lin_vel=np.asarray(lin_vel, float), quat_xyzw=IDENT)


def objects(clip=None):
    return {"clipper": _state(M.CLIP_START if clip is None else clip), "trainer": _state((0.0, 0.0, M.TOP))}


AWAY = [((0.17, -0.16, M.CLIP_START[2] + M.CLEAR_R + 0.02), M.OPEN_F), ((0.10, 0.30, M.TOP + 0.30), 0.0)]   # right lifted off, left idle


def ring_at(k, protrude):
    """The clipper's grip point that puts claw k's end ``protrude`` over the blade, the ring centred on it."""
    tip = M.claw_tips(PAW_AT_REST, PAW_Q, M._M["removed"])[k]
    return np.array([tip[0] - M.RING_X * math.cos(HEAD), tip[1] - M.RING_X * math.sin(HEAD),
                     tip[2] - protrude - M.Z_BLADE])


def M_quat_mul(a, b):
    """Hamilton product of two (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def squeeze(m, clip, paw=PAW_AT_REST):
    tips = M.claw_tips(paw, PAW_Q, m["removed"])
    for q in (0.0, 0.004, 2 * M.SQUEEZE_EACH, 0.0):      # 4 mm: the blade's edge in the claw, which pushes back
        M.advance(m, q, tips, clip, CLIP_Q)
        tips = M.claw_tips(paw, PAW_Q, m["removed"])


class Model(unittest.TestCase):
    def setUp(self):
        M.reset_memory(trial_seed(0))

    def test_cut_takes_what_stands_over_the_blade(self):
        squeeze(M._M, ring_at(1, 0.0015))
        self.assertAlmostEqual(M._M["removed"][1], 0.0015, places=9)
        self.assertEqual(M._M["removed"][0] + M._M["removed"][2] + M._M["removed"][3], 0.0)

    def test_no_cut_without_the_claw_through_the_ring(self):
        squeeze(M._M, ring_at(1, -0.001))                      # the end under the blade's plane
        off = ring_at(1, 0.0015) + np.array([0.004, 0.0, 0.0])   # the ring 4 mm off the claw's axis
        squeeze(M._M, off)
        self.assertEqual(sum(M._M["removed"]), 0.0)

    def test_one_cut_per_squeeze(self):
        clip = ring_at(0, 0.001)
        tips = M.claw_tips(PAW_AT_REST, PAW_Q, M._M["removed"])
        for q in (0.0, 0.004, 2 * M.SQUEEZE_EACH, 2 * M.SQUEEZE_EACH, M.Q_CUT, 2 * M.SQUEEZE_EACH):   # never reopened
            M.advance(M._M, q, tips, clip, CLIP_Q)
        self.assertEqual(M._M["cuts"][0], 1)

    def test_claw_resists_the_blade_only_while_it_cuts(self):
        """R_CUT while the blade's edge is inside a claw in the ring during a squeeze from open; 0 before the edge reaches
        it, after a finished cut, and with no claw in the ring (review of #2108: no tool force)."""
        clip = ring_at(0, 0.001)
        tips = M.claw_tips(PAW_AT_REST, PAW_Q, M._M["removed"])
        seen = []
        for q in np.arange(0.0, 2 * M.SQUEEZE_EACH + 1e-9, 0.0002):
            M.advance(M._M, float(q), tips, clip, CLIP_Q)
            seen.append(M._M["resist"])
        self.assertEqual(seen[0], 0.0)                                 # the edge is outside the claw at rest
        self.assertIn(M.R_CUT, seen)                                    # it resists in the claw
        self.assertEqual(seen[-1], 0.0)                                 # and not after the cut
        self.assertEqual(M._M["cuts"][0], 1)
        far = ring_at(0, 0.001) + np.array([0.0, 0.0, 0.02])            # ring 20 mm over the claw: nothing to cut
        M._M["armed"] = True
        self.assertEqual(max(M.resisting(M._M, q, tips, far, CLIP_Q)[0] for q in np.arange(0, 0.012, 0.0005)), 0.0)

    def test_resistance_is_felt_per_claw(self):
        """Review 2026-11-17: a half-closed blade that met claw 1 and was carried to claw 2 does not cut claw 2."""
        tips = M.claw_tips(PAW_AT_REST, PAW_Q, M._M["removed"])
        M.advance(M._M, 0.0, tips, ring_at(0, 0.001), CLIP_Q)
        M.advance(M._M, 0.004, tips, ring_at(0, 0.001), CLIP_Q)     # claw 1 pushes back
        M.advance(M._M, 0.0117, tips, ring_at(1, 0.001), CLIP_Q)    # finished over claw 2
        self.assertEqual(M._M["cuts"][:2], [0, 0])

    def test_snap_squeeze_past_the_claw_does_not_cut(self):
        """Review 2026-11-17 finding 1: the blade from 1.5 mm to 11.7 mm in one step never met the claw's push-back, and
        cut. Now it does not; reopened and squeezed through the claw, it cuts."""
        clip = ring_at(0, 0.001)
        tips = M.claw_tips(PAW_AT_REST, PAW_Q, M._M["removed"])
        for q in (0.0, 0.0015, 0.0117):
            M.advance(M._M, q, tips, clip, CLIP_Q)
        self.assertEqual(M._M["cuts"][0], 0)
        for q in (0.0, 0.004, 0.0117):
            M.advance(M._M, q, tips, clip, CLIP_Q)
        self.assertEqual(M._M["cuts"][0], 1)

    def test_gripper_can_squeeze_through_a_claw(self):
        """At every handle travel where the claw resists, the squeezing finger's drive (yam_arena GRIPPER, stiffness to
        the SQUEEZE_F target, capped at its effort) beats the spring plus R_CUT by 5 N or more."""
        from yam_arena.constants import GRIPPER
        lo, hi = -M.BLADE_EDGE0 - M.CLAW_HALF - M.IN_RING_R, -M.BLADE_EDGE0 + M.CLAW_HALF + M.IN_RING_R
        for q in np.arange(max(lo, 0.0), hi, 0.0001):
            x = q / 2                                                   # each handle's travel, centred squeeze
            face = M.HANDLE_Y + M.HANDLE_W / 2 - x                      # the handle's outer face, pad contact
            drive = min(GRIPPER["effort"], GRIPPER["stiffness"] * (face + M.SQUEEZE_F))
            need = M.SPRING_PRE + M.SPRING_K * x + M.R_CUT
            self.assertGreater(drive, need + 5.0, (q, drive, need))

    def test_dot_zone_and_bleed(self):
        q = M._M["quick"][2]
        squeeze(M._M, ring_at(2, q - M.DOT - 0.0002))          # just short of the dot zone
        self.assertFalse(M.dot_shows(M._M, 2))
        squeeze(M._M, ring_at(2, 0.0004))                      # into the zone
        self.assertTrue(M.dot_shows(M._M, 2))
        squeeze(M._M, ring_at(2, M.DOT))                       # past the quick
        self.assertTrue(M._M["bled"][2])
        self.assertFalse(M.dot_shows(M._M, 2))

    def test_flipped_clipper_does_not_cut(self):
        """Upside down over a claw, its ring 2 mm over the end: no cut (v1 review finding 5)."""
        tip = M.claw_tips(PAW_AT_REST, PAW_Q, M._M["removed"])[1]
        flip = (math.sin(HEAD / 2) * 0 + 1.0, 0.0, 0.0, 0.0)              # 180 deg about x
        grip = np.array([tip[0] - M.RING_X * math.cos(HEAD), tip[1] + M.RING_X * math.sin(HEAD), tip[2] + 0.002])
        inside, below = M.in_ring(tip, grip, flip)
        self.assertFalse(inside)
        for q in (0.0, 0.004, 2 * M.SQUEEZE_EACH, 0.0):
            M.advance(M._M, q, [tip] * 4, grip, flip)
        self.assertEqual(sum(M._M["removed"]), 0.0)

    def test_tilted_clipper_does_not_cut(self):
        clip = ring_at(2, 0.0015)
        for deg, cuts in ((10.0, True), (20.0, False)):
            a = math.radians(deg)
            ax, ay = -math.sin(HEAD), math.cos(HEAD)     # tilt about the clipper's own y axis (the hand's tilt)
            qt = np.array([ax * math.sin(a / 2), ay * math.sin(a / 2), 0.0, math.cos(a / 2)])
            q = M_quat_mul(qt, CLIP_Q)
            tip = M.claw_tips(PAW_AT_REST, PAW_Q, [0.0] * 4)[2]
            self.assertEqual(M.in_ring(tip, clip, q)[0], cuts, deg)

    def test_cut_carried_by_the_paw(self):
        """The cut is judged where the paw actually is: a paw pushed 3 mm up takes 3 mm more."""
        clip = ring_at(3, 0.001)
        tips = M.claw_tips((M.T_POS[0], M.T_POS[1], M.TOP + 0.003), PAW_Q, M._M["removed"])
        for q in (0.0, 0.004, 2 * M.SQUEEZE_EACH):
            M.advance(M._M, q, tips, clip, CLIP_Q)
        self.assertAlmostEqual(M._M["removed"][3], 0.004, places=9)


class Scoring(unittest.TestCase):
    def setUp(self):
        M.reset_memory(trial_seed(2))
        M._CLOCK.update({"dt": 0.05, "steps": 0})
        M._M["hands"] = AWAY

    def _tick(self, obj):
        M._M["steps"] += 1
        M._CLOCK["steps"] += 1
        return M.score(obj), M.success(obj)

    def test_ladder(self):
        self.assertEqual(self._tick(objects())[0], 0.0)
        for k in range(4):
            M._M["removed"][k] = M._M["quick"][k] - M.DOT / 2
            s, ok = self._tick(objects(clip=(0.2, 0.0, M.TOP + 0.13)))
            self.assertAlmostEqual(s, M.LADDER[k + 1])
            self.assertFalse(ok)
        s, ok = self._tick(objects())
        self.assertEqual((s, ok), (0.9, True))
        for _ in range(20):
            s, ok = self._tick(objects())
        self.assertEqual((s, ok), (1.0, True))

    def test_bleed_latches_zero(self):
        for k in range(4):
            M._M["removed"][k] = M._M["quick"][k] - M.DOT / 2
        M._M["bled"][1] = True
        self.assertEqual(self._tick(objects()), (0.0, False))
        M._M["bled"][1] = False                                   # latched: still 0
        self.assertEqual(self._tick(objects())[0], 0.0)

    def test_moving_clipper_is_not_down(self):
        for k in range(4):
            M._M["removed"][k] = M._M["quick"][k] - M.DOT / 2
        self.assertEqual(self._tick({"clipper": _state(M.CLIP_START, (0.1, 0, 0)),
                                     "trainer": _state((0, 0, M.TOP))})[0], M.LADDER[4])

    def test_held_clipper_is_not_down(self):
        """A hand still holding the clipper still on its mat is not success (v1 review finding 4, the #914 flag)."""
        for k in range(4):
            M._M["removed"][k] = M._M["quick"][k] - M.DOT / 2
        grip = (M.CLIP_START[0], M.CLIP_START[1], M.CLIP_START[2] + M.TCP_OVER_G)
        M._M["hands"] = [(grip, M.HOLD_F), AWAY[1]]
        for _ in range(30):
            s, ok = self._tick(objects())
        self.assertEqual((s, ok), (M.LADDER[4], False))
        M._M["hands"] = [((grip[0], grip[1], grip[2] + 0.04), M.OPEN_F), AWAY[1]]   # opened, 40 mm up: fingers in reach
        for _ in range(30):
            s, ok = self._tick(objects())
        self.assertEqual((s, ok), (M.LADDER[4], False))
        M._M["hands"] = [((grip[0], grip[1], grip[2] + M.CLEAR_R + 0.005), M.OPEN_F), AWAY[1]]
        for _ in range(30):
            s, ok = self._tick(objects())
        self.assertEqual((s, ok), (1.0, True))

    def test_open_pinch_end_to_end_is_not_let_go(self):
        """Review 2026-11-17 finding 2: pads 85 mm apart pinching a handle end to end, the TCP 31.8 mm from the grip
        point, scored 1.0 under v2's finger-opening rule."""
        for k in range(4):
            M._M["removed"][k] = M._M["quick"][k] - M.DOT / 2
        tcp = (M.CLIP_START[0] - 0.0275, M.CLIP_START[1] + M.HANDLE_Y, M.CLIP_START[2] + 0.004)
        M._M["hands"] = [(tcp, -0.0425), AWAY[1]]
        for _ in range(30):
            s, ok = self._tick(objects())
        self.assertEqual((s, ok), (M.LADDER[4], False))

    def test_clipper_on_its_edge_is_not_down(self):
        """Review 2026-11-17 finding 8: on its edge the clipper's frame is 14.5 mm up, inside ON_MAT."""
        for k in range(4):
            M._M["removed"][k] = M._M["quick"][k] - M.DOT / 2
        o = _state((M.CLIP_START[0], M.CLIP_START[1], M.CLIP_START[2] + 0.0145))
        o.quat_xyzw = np.array([math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)])
        for _ in range(30):
            s, ok = self._tick({"clipper": o, "trainer": _state((0.0, 0.0, M.TOP))})
        self.assertEqual((s, ok), (M.LADDER[4], False))

    def test_left_hand_holding_counts_too(self):
        for k in range(4):
            M._M["removed"][k] = M._M["quick"][k] - M.DOT / 2
        M._M["hands"] = [AWAY[0], ((M.CLIP_START[0], M.CLIP_START[1], M.CLIP_START[2] + 0.01), M.HOLD_F)]
        self.assertFalse(self._tick(objects())[1])

    def test_hands_unread_is_not_success(self):
        for k in range(4):
            M._M["removed"][k] = M._M["quick"][k] - M.DOT / 2
        M._M["hands"] = None
        self.assertFalse(self._tick(objects())[1])

    def test_model_must_step(self):
        for _ in range(7):
            M._CLOCK["steps"] += 1
        self.assertEqual(M.score(objects()), 0.0)


class Demo(unittest.TestCase):
    def _replay(self, cli):
        """The demo's planned ring heights through the model: each squeeze cuts what it planned."""
        seed = trial_seed(cli)
        M.reset_memory(seed)
        R, info = M.plan(0.0, seed)
        for w in R:
            if w.label.startswith("squeeze"):
                grip = np.asarray(w.pos, float) - np.array([0.0, 0.0, M.TCP_OVER_G + M.CLIP_SAG])
                squeeze(M._M, grip)
        return M._M, info

    def test_demo_reaches_every_dot_on_every_seed(self):
        for cli in CLI:
            m, info = self._replay(cli)
            self.assertEqual([M.dot_shows(m, k) for k in range(4)], [True] * 4, (cli, m["removed"], m["quick"]))
            self.assertFalse(any(m["bled"]))
            for k, c in m["log"]:
                self.assertLessEqual(c, 2.01, (cli, k, c))   # every cut 2 mm or less (VCA: "1–2 mm"; not scored)

    def test_demo_fits_the_run(self):
        for cli in CLI:
            R, _ = M.plan(0.0, trial_seed(cli))
            self.assertLess(sum(w.duration for w in R) * 20, M.RUN_LIMIT_STEPS - 400)

    def test_demo_lets_go_and_uses_one_arm(self):
        R, _ = M.plan(0.0, trial_seed(0))
        self.assertEqual(set(M.demo().keys()), {"right"})
        last = R[-1]
        self.assertEqual(last.finger, M.OPEN_F)
        grip = np.asarray(M.CLIP_START) + np.array([0.0, 0.0, M.TCP_OVER_G])
        self.assertGreater(np.linalg.norm(np.asarray(last.pos) - grip), M.CLEAR_R + 0.01)

    def test_clear_radius_beyond_fingers_reach(self):
        """CLEAR_R exceeds the clipper's extent plus the fingers' reach from the TCP (tools/reach.py)."""
        try:
            sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent / "tools"))
            import reach
        except ImportError as e:
            self.skipTest(f"yam_arena not importable: {e}")
        self.assertGreater(M.CLEAR_R, reach.clipper_extent() + reach.finger_reach() + 0.01)

    def test_film_seeds_differ(self):
        self.assertEqual(len({M.quicks(trial_seed(c)) for c in CLI}), 3)


class HiddenQuick(unittest.TestCase):
    def test_best_blind_script_rarely_succeeds(self):
        """Q2: the best script that cuts a fixed length per claw without looking (any lengths, 0.1 mm steps, chosen
        knowing the draw) shows all four dots on under 5% of 2,000 seeds; each claw's quick is drawn on its own."""
        draws = [M.quicks(s) for s in range(2000)]
        best = 1.0
        for k in range(4):
            share = max(sum(0.0 < d[k] - r / 10 <= M.DOT * 1000 for d in draws) / len(draws) for r in range(0, 121))
            best *= share
        self.assertLess(best, 0.05, best)
        self.assertLess(len({d for d in draws}), len(draws))      # repeats exist, but
        self.assertGreater(len({d for d in draws}), 1500)          # most seeds are different trials

    def test_blind_script_earns_little(self):
        """The best single cut length for every claw, blind, putting the clipper down after (so four dots earn 1.0):
        its expected score stays at about 0.2 (README states it; review 2026-11-17 finding 3)."""
        draws = [M.quicks(s) for s in range(4000)]
        best = 0.0
        for r10 in range(0, 90):
            r, tot = r10 / 10, 0.0
            for d in draws:
                if any(r >= q for q in d):
                    continue
                n = sum(0 < q - r <= M.DOT * 1000 for q in d)
                tot += 1.0 if n == 4 else M.LADDER[n]
            best = max(best, tot / len(draws))
        self.assertLess(best, 0.22, best)
        self.assertIn(f"{best:.2f}", ReadmeNumbers.TXT)

    def test_quick_range(self):
        for s in range(500):
            for q in M.quicks(s):
                self.assertTrue(4.5 <= q <= 8.5)
                self.assertLess(q / 1000, M.CUT_MAX)


class Geometry(unittest.TestCase):
    def test_trainer_frame_round_trip(self):
        for p in ((0.245, -0.021), (0.37, 0.0), M.MAT):
            self.assertTrue(np.allclose(M.to_local(*M.to_world(*p)), p))
        x, y = M.to_world(M.CLAW_X, 0.0)
        self.assertTrue(np.allclose(M.claw_tips(PAW_AT_REST, PAW_Q, [0.0] * 4)[1][:2],
                                    M.to_world(M.CLAW_X, M.CLAW_Y[1])))

    def test_demo_ring_centred_on_each_claw(self):
        """The planned grip point puts the ring's hole on the claw it cuts (the clipper's heading as planned)."""
        R, _ = M.plan(0.0, trial_seed(0))
        tips = M.claw_tips(PAW_AT_REST, PAW_Q, [0.0] * 4)
        for w in R:
            if w.label.startswith("squeeze"):
                k = int(R[R.index(w) - 3].label.split()[1].rstrip(":")) - 1
                q = (0.0, 0.0, math.sin(w.yaw / 2), math.cos(w.yaw / 2))
                inside, _ = M.in_ring(tips[k], np.asarray(w.pos) - [0, 0, M.TCP_OVER_G], q)
                self.assertTrue(inside, (k, w.pos))

    def test_dot_hidden_inside_the_tip_until_it_shows(self):
        """The dot (radius DOT_R) sits 3 mm under the tip's end; the tip there is wider than the dot on both sides."""
        f = (M.Z_TIP - (M.Z_TIP - 0.0030 + 0.0002)) / (M.Z_TIP - (M.Z_TOE + 0.003))
        half_y = (M.CLAW_W[0] + (M.CLAW_ROOT[0] - M.CLAW_W[0]) * f) / 2
        self.assertGreater(half_y - M.DOT_R, 0.0002)
        self.assertLess(2 * M.DOT_R, M.CLAW_W[0])             # and it fits on the narrowest cut face

    def test_claw_end_fits_the_ring(self):
        self.assertLess(max(M.CLAW_ROOT) / 2, M.RING_IN)
        self.assertAlmostEqual(2 * M.RING_IN, 0.009)

    def test_remaining_claw_keeps_the_ring_off_the_toes(self):
        deepest = max(M.QUICK_MM) / 1000
        ring_bottom = M.Z_TIP - deepest - M.RING_T      # the ring's underside at the deepest cut
        self.assertGreater(ring_bottom - M.Z_TOE, 0.002)

    def test_clipper_holds_without_cutting(self):
        """The pads 3 mm into each handle push 15 N, under the spring's 20 N preload, so the blade stays open; the
        demo's squeeze beats the spring at its stop, within the fingers' 50 N."""
        press = 5000.0 * (M.HANDLE_Y + M.HANDLE_W / 2 - abs(M.HOLD_F))
        self.assertLess(press, M.SPRING_PRE - 3.0)
        self.assertGreater(50.0, M.SPRING_PRE + M.SPRING_K * M.SQUEEZE_EACH)
        face = M.HANDLE_Y + M.HANDLE_W / 2 - M.SQUEEZE_EACH          # a handle's outer face at its stop
        push = 5000.0 * (face - abs(M.SQUEEZE_F))                      # the demo's squeeze, each pad, at the stop
        self.assertGreater(push, M.SPRING_PRE + M.SPRING_K * M.SQUEEZE_EACH + 5.0)
        self.assertLessEqual(push, 50.0)
        self.assertGreater(press, 10.0)


class ReadmeNumbers(unittest.TestCase):
    """The README's numbers come from the code."""
    TXT = " ".join((HERE / "README.md").read_text().split())

    def test_seed_table(self):
        for cli in CLI:
            row = f"| {cli} | " + " | ".join(f"{q:.1f}" for q in M.quicks(trial_seed(cli))) + " |"
            self.assertIn(row, self.TXT)

    def test_claw_row_and_parts(self):
        xs = ", ".join(f"{1000 * M.to_world(M.CLAW_X, y)[0]:.0f}" for y in M.CLAW_Y)
        self.assertIn(f"at x {xs} mm, y {1000 * M.to_world(M.CLAW_X, 0.0)[1]:.0f} mm", self.TXT)
        self.assertIn(f"{2 * M.RING_IN * 1000:.0f} mm ring", self.TXT)
        self.assertIn(f"({M.SPRING_PRE:.0f} N preload each)", self.TXT)
        self.assertIn(f"up to {M.SQUEEZE_EACH * 1000:.0f} mm toward the other", self.TXT)
        self.assertIn(f"{M.DOT * 1000:.0f} mm dot zone", self.TXT)
        self.assertIn(f"past {M.Q_CUT * 1000:.1f} mm", self.TXT)
        self.assertIn(f"within {M.IN_RING_R * 1000:.2f} mm of the hole's centre", self.TXT)
        self.assertIn(f"{2 * M.DOT_R * 1000:.1f} mm", self.TXT)


if __name__ == "__main__":
    unittest.main()
