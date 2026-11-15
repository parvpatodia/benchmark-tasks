"""Offline tests for solder_joint.py (no simulator): the heat model, the scorer, the demo replayed through the model
on each seed, and that no open-loop timing serves every seed. Run with yam_arena on PYTHONPATH."""
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

M = load(HERE / "solder_joint.py").module
CLI = (0, 2, 4)


def trial_seed(cli):
    return zlib.crc32(f"{cli}:None:0".encode())


def _q(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _lerp(ws, t):
    t0, prev = 0.0, None
    for w in ws:
        p = np.asarray(w.pos, float)
        prev = p if prev is None else prev
        if t <= t0 + w.duration:
            return prev + ((t - t0) / w.duration if w.duration else 1.0) * (p - prev)
        t0 += w.duration
        prev = p
    return prev


def state(torch_g, wire_tip_pos, face_q=0.0):
    """A states dict like yam_arena's, from the torch's grip and the wire's tip."""
    wpos = np.asarray(wire_tip_pos, float) - np.asarray(M.TIP_LOCAL)
    return {"torch": types.SimpleNamespace(pos=np.asarray(torch_g, float), quat_xyzw=_q(M.TORCH_YAW)),
            "wire": types.SimpleNamespace(pos=wpos, quat_xyzw=(0.0, 0.0, 0.0, 1.0)),
            "bench": types.SimpleNamespace(joints={"face": face_q, "bead": 0.0, "burn": 0.0})}


def replay(left, right, idx, dt=0.05, held_from=None):
    """Step the model along both plans as a stiff arm would: the torch rides the left TCP once gripped, the wire's tip
    rides the right TCP once lifted, and the face follows the wire's tip down when pushed. Returns the last score and
    the model."""
    M.reset_memory()
    M._M["idx"] = idx
    held_from = 4.0 if held_from is None else held_from
    end = min(sum(w.duration for w in left), sum(w.duration for w in right))
    t, s, best = 0.0, 0.0, 0.0
    while t <= end + 1e-9:
        g = (_lerp(left, t) - np.array([0.0, 0.0, M.TCP_OVER_G + M.TORCH_SAG])) if t >= held_from else np.asarray(M.G_REST)
        tip = _lerp(right, t) - np.array([0.0, 0.0, M.TCP_OVER_TIP])
        target = -M._M["fed"]
        face_top = M.TOP + M.STOP_TOP
        over = math.hypot(tip[0] - M.JX, tip[1]) <= M.STOP_R + M.WIRE_R
        q = max(-M.FED_MAX - 0.001, min(target, tip[2] - face_top)) if over else target
        M.advance(M._M, dt, M.flame_tip(g, _q(M.TORCH_YAW)), tip, q)
        M._M["steps"] += 1
        M._CLOCK.update({"dt": dt, "steps": M._CLOCK["steps"] + 1})
        st = state(g, tip, q)
        s = M.score(st)
        best = max(best, s)
        t += dt
        if M.success(st) and M._MEM["held"] >= M.HOLD_S:
            break
    return s, M._M, M._MEM


class Numbers(unittest.TestCase):
    def test_sources(self):
        self.assertAlmostEqual(M.FULL, 0.320 * 0.0254)
        self.assertAlmostEqual(2 * M.TUBE_R, 0.625 * 0.0254)
        self.assertAlmostEqual(M.CUP_D, 0.500 * 0.0254)
        self.assertEqual((M.SOLIDUS, M.LIQUIDUS, M.BURN_T), (452.0, 464.0, 600.0))
        self.assertAlmostEqual(2 * M.WIRE_R, 0.125 * 0.0254)

    def test_quotes_in_src(self):
        """Every quote in the text is in a saved source (CDA in reading order: pdftotext without -layout)."""
        src = HERE.parent / "src"
        if not src.exists():
            self.skipTest("src/ not shipped")
        flat = lambda f: " ".join((src / f).read_text().split())  # noqa: E731
        cda, onet = flat("soldering_brazing_ads-1_flow.txt"), flat("onet_47-2152.00.txt")
        ctx = json.loads((HERE / "task.json").read_text())["context"]
        quotes = [q for i, q in enumerate(ctx.split('"')) if i % 2]
        self.assertGreaterEqual(len(quotes), 5)
        for q in quotes:
            self.assertTrue(q in cda or q in onet, q)
        self.assertIn("to the base of the cup to aid capillary", cda)
        self.assertIn("Using 1/8-inch diameter (No. 9) Wire Solder", flat("soldering_brazing_ads-1.txt"))
        self.assertIn("Employment (2025) 510,600 employees", onet)
        h = flat("harris_95-5_spec.txt")
        self.assertIn("Solidus 452", h)
        self.assertIn("Liquidus 464", h)
        self.assertRegex(flat("superior_flux_142-5.txt"), r"Soldering Temperature Range 95\W*C . 315\W*C /200\W*F. 600")

    def test_heat_index_seeds(self):
        self.assertEqual([M.heat_index(trial_seed(c)) for c in CLI], [1, 0, 2])

    def test_flame_tip_at_mark_in_heat_pose_and_far_at_rest(self):
        self.assertLess(np.linalg.norm(M.flame_tip(M.G_HEAT, _q(M.TORCH_YAW)) - np.asarray(M.MARK)), 1e-9)
        self.assertLess(M.weight(M.flame_tip(M.G_REST, _q(M.TORCH_YAW))), 1e-3)   # under 1 F at rest
        self.assertLess(M.weight(M.flame_tip((M.AWAY_G[0], M.AWAY_G[1], M.AWAY_G[2]), _q(M.TORCH_YAW))), 1e-3)

    def test_geometry(self):
        self.assertLess(M.STOP_TOP, M.Z_MOUTH)
        self.assertGreater(M.HOLE, 2 * M.WIRE_R)
        land = M.R_WIRE + M.WIRE_DRIFT - M.WIRE_R                 # the wire's inner edge where it lands (measured drift)
        self.assertLess(land, M.TUBE_R)                            # aimed into the top tube: it rests against it
        self.assertGreater(land, M.TUBE_R - 0.0005)                # by at most 0.5 mm
        self.assertLessEqual(M.TUBE_R + M.EDGE_R, M.STOP_R)             # the edge zone lies on the face
        self.assertLess(M.FED_MAX, M.CUP_D)
        # the torch's handle sits on its rest at the start
        self.assertAlmostEqual(M.G_REST[2] - M.HANDLE_W / 2 - (M.TOP + M.REST_TOP), 0.0005, places=6)
        self.assertEqual(M.BENCH.joints[0].name, "face")


class Model(unittest.TestCase):
    def setUp(self):
        M.reset_memory()

    def run_const(self, idx, w_tip, secs, wire=None, face_q=0.0):
        M._M["idx"] = idx
        for _ in range(int(round(secs / 0.05))):
            M.advance(M._M, 0.05, w_tip, (0, 0, 1) if wire is None else wire, face_q)

    def test_open_loop_cannot_serve_every_seed(self):
        """Flame held on its mark from t = 0: the slowest seed melts after the fastest has burnt its flux, so no single
        time to feed and take the flame off works; and no fixed flame distance keeps every seed in the window."""
        tm = [-M.TAU * math.log(1 - (M.LIQUIDUS - M.T_ROOM) / (t - M.T_ROOM)) for t in M.T_SS]
        tb = [-M.TAU * math.log(1 - (M.BURN_T - M.T_ROOM) / (t - M.T_ROOM)) for t in M.T_SS if t > M.BURN_T]
        self.assertGreater(max(tm), min(tb))
        w_hi = (M.BURN_T - M.T_ROOM) / (max(M.T_SS) - M.T_ROOM)        # the hottest seed stays under the burn
        w_lo = (M.LIQUIDUS - M.T_ROOM) / (min(M.T_SS) - M.T_ROOM)      # the coolest seed reaches the liquidus
        self.assertLess(w_hi, w_lo)

    def test_burn_latches(self):
        self.run_const(0, M.MARK, 25.0)
        self.assertTrue(M._M["burnt"])

    def test_cold_wire_does_not_feed(self):
        tip = (M.W_XY[0], M.W_XY[1], M.TOP + M.STOP_TOP - 0.002)
        self.run_const(2, (9, 9, 9), 5.0, wire=tip, face_q=-0.002)
        self.assertEqual(M._M["fed"], 0.0)
        self.assertTrue(M._M["pressed"])

    def test_resting_wire_without_push_does_not_feed(self):
        M._M["T"] = 500.0
        tip = (M.W_XY[0], M.W_XY[1], M.TOP + M.STOP_TOP)
        M.advance(M._M, 0.05, M.MARK, tip, 0.0)
        self.assertTrue(M._M["on"])
        self.assertFalse(M._M["pressed"])
        self.assertEqual(M._M["fed"], 0.0)

    def test_hot_pressed_wire_feeds_at_rate(self):
        M._M["T"], M._M["idx"] = 500.0, 0
        tip = (M.W_XY[0], M.W_XY[1], M.TOP + M.STOP_TOP - 0.002)
        M.advance(M._M, 0.05, M.MARK, tip, -0.002)
        self.assertAlmostEqual(M._M["fed"], M.FEED_RATE * 0.05)

    def test_wire_off_the_face_does_not_feed(self):
        M._M["T"] = 500.0
        tip = (M.JX + 0.03, 0.0, M.TOP + M.STOP_TOP - 0.002)
        M.advance(M._M, 0.05, M.MARK, tip, -0.002)
        self.assertEqual(M._M["fed"], 0.0)

    def test_edge_zone(self):
        """On the joint's edge only with the wire's inner edge within EDGE_R of the top tube."""
        z = M.TOP + M.STOP_TOP
        for r_in, want in ((M.TUBE_R + 0.0005, True), (M.CUP_R - 0.0001, True), (M.CUP_R + 0.0003, False), (M.TUBE_R + 0.0015, False)):
            r = r_in + M.WIRE_R
            self.assertEqual(M.on_joint((M.JX + r * M.U[0], r * M.U[1], z), 0.0), want, r_in)
        r = M.TUBE_R + M.WIRE_R
        self.assertFalse(M.on_joint((M.JX + r * M.U[0], r * M.U[1], z + 0.001), 0.0))   # 1 mm over the face

    def test_wire_low_counts_tilt(self):
        a = math.radians(10)
        q = (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))
        self.assertAlmostEqual(M.wire_low((0, 0, 0), q)[2], M.wire_tip((0, 0, 0), q)[2] - M.WIRE_R * math.sin(a))

    def test_flame_on_the_edge_or_the_wire_latches(self):
        edge = (M.JX + M.CUP_R, 0.0, M.TOP + M.Z_MOUTH + 0.005)
        M.advance(M._M, 0.05, edge, (0, 0, 1), 0.0)
        self.assertTrue(M._M["flamed"])
        M.reset_memory()
        w = (M.JX + 0.05, 0.05, M.TOP + 0.10)
        M.advance(M._M, 0.05, (w[0] + 0.005, w[1], w[2]), w, 0.0)
        self.assertTrue(M._M["flamed"])
        M.reset_memory()
        M.advance(M._M, 0.05, M.MARK, (0, 0, 1), 0.0)              # on the mark: 12.7 mm under the edge
        self.assertFalse(M._M["flamed"])

    def test_stuck_when_left_on_while_setting(self):
        M._M.update({"T": M.SOLIDUS + 0.1, "fed": M.FULL})
        tip = (M.W_XY[0], M.W_XY[1], M.TOP + M.STOP_TOP - M.FULL)
        M.advance(M._M, 0.05, (9, 9, 9), tip, -M.FULL)
        self.assertTrue(M._M["stuck"])


class Scoring(unittest.TestCase):
    def setUp(self):
        M.reset_memory()
        M._CLOCK.update({"dt": 0.05, "steps": 0})

    def judge(self, fed, T, g=None, tip=None):
        M.reset_memory()
        M._M.update({"fed": fed, "T": T})
        M._CLOCK.update({"dt": 0.05})
        g = M.AWAY_G if g is None else g
        tip = (M.W_XY[0], M.W_XY[1] - 0.06, M.TOP + 0.12) if tip is None else tip
        return M.score(state(g, tip))

    def test_ladder(self):
        self.assertEqual(self.judge(0.0, 70), 0.0)
        self.assertEqual(self.judge(0.001, 70), 0.25)
        self.assertEqual(self.judge(M.FULL / 2, 70), 0.5)
        self.assertEqual(self.judge(M.FULL, 500), 0.6)                          # full, not set
        self.assertEqual(self.judge(M.FULL, 400, g=M.G_HEAT), 0.6)             # set, flame still on
        self.assertEqual(self.judge(M.FULL, 400, tip=(M.W_XY[0], M.W_XY[1], M.TOP + M.Z_MOUTH + 0.01)), 0.6)
        self.assertEqual(self.judge(M.FULL, 400), 0.8)                          # not yet held 1 s

    def test_failures_score_zero(self):
        M.reset_memory()
        M._M.update({"fed": M.FULL, "T": 400, "burnt": True})
        self.assertEqual(M.score(state(M.AWAY_G, (0.3, -0.1, M.TOP + 0.1))), 0.0)
        M.reset_memory()
        M._M.update({"fed": M.FULL, "T": 400, "stuck": True})
        self.assertEqual(M.score(state(M.AWAY_G, (0.3, -0.1, M.TOP + 0.1))), 0.0)
        M.reset_memory()
        M._M.update({"fed": M.FULL, "T": 400})
        self.assertEqual(M.score(state((0.1, 0.2, M.TOP - 0.05), (0.3, -0.1, M.TOP + 0.1))), 0.0)  # torch lost

    def test_flamed_scores_zero(self):
        M.reset_memory()
        M._M.update({"fed": M.FULL, "T": 400, "flamed": True})
        self.assertEqual(M.score(state(M.AWAY_G, (0.3, -0.1, M.TOP + 0.1))), 0.0)

    def test_dropped_tool_is_not_away(self):
        """A torch or a wire let go on the table is not success, however far from the joint."""
        self.assertEqual(self.judge(M.FULL, 400, g=(0.10, 0.25, M.TOP + 0.012)), 0.6)
        self.assertEqual(self.judge(M.FULL, 400, tip=(0.32, -0.15, M.TOP + 0.012 + M.TIP_LOCAL[2])), 0.6)
        self.assertEqual(self.judge(M.FULL, 400, g=(0.22, 0.03, M.TOP + 0.041)), 0.6)      # lying on the vise
        rest_tip = np.asarray(M.WIRE_START) + np.asarray(M.TIP_LOCAL)
        self.assertEqual(self.judge(M.FULL, 400, g=M.G_REST, tip=rest_tip), 0.8)           # both back on their stands

    def test_not_stepping_fails(self):
        M.reset_memory()
        M._M.update({"fed": M.FULL, "T": 400})
        M._CLOCK.update({"dt": 0.05, "steps": 10})
        self.assertEqual(M.score(state(M.AWAY_G, (0.3, -0.1, M.TOP + 0.1))), 0.0)


class Demo(unittest.TestCase):
    def test_demo_scores_one_on_every_seed(self):
        for cli in CLI:
            seed = trial_seed(cli)
            L, R, info = M.plan(0.0, seed)
            s, m, mem = replay(L, R, M.heat_index(seed), held_from=info["held_from"])
            self.assertEqual(s, 1.0, f"cli {cli}: {mem['fail']} fed {m['fed']:.4f} T {m['T']:.0f} Tmax {m['T_max']:.0f}")
            self.assertLess(m["T_max"], M.BURN_T - 20, f"cli {cli} margin")
            self.assertGreaterEqual(m["fed"], M.FULL)

    def test_demo_keeps_the_flame_off_the_edge_and_the_wire(self):
        """Along the plan the inner cone stays 2 mm or more outside FLAME_CLEAR of the joint's edge and the wire's tip."""
        for cli in CLI:
            L, R, info = M.plan(0.0, trial_seed(cli))
            end, t, worst = min(sum(w.duration for w in L), sum(w.duration for w in R)), 0.0, 1.0
            while t <= end:
                g = (_lerp(L, t) - np.array([0.0, 0.0, M.TCP_OVER_G + M.TORCH_SAG])) if t >= info["held_from"] else np.asarray(M.G_REST)
                ft = M.flame_tip(g, _q(M.TORCH_YAW))
                tip = _lerp(R, t) - np.array([0.0, 0.0, M.TCP_OVER_TIP])
                worst = min(worst, M.edge_dist(ft), float(np.linalg.norm(ft - tip)))
                t += 0.05
            self.assertGreater(worst, M.FLAME_CLEAR + 0.002, f"cli {cli}")

    def test_demo_with_start_delays(self):
        for d in (0.05, 0.10):
            seed = trial_seed(0)
            L, R, info = M.plan(d, seed)
            s, m, mem = replay(L, R, M.heat_index(seed), held_from=info["held_from"])
            self.assertEqual(s, 1.0, f"delay {d}: {mem['fail']}")

    def test_plan_for_one_seed_fails_another(self):
        """The demo is timed for its seed; the same script run on another seed burns or leaves the joint unfilled."""
        hot, cool = trial_seed(2), trial_seed(4)
        L, R, info = M.plan(0.0, cool)
        s, m, mem = replay(L, R, M.heat_index(hot), held_from=info["held_from"])
        self.assertEqual(s, 0.0)
        self.assertTrue(m["burnt"])
        L, R, info = M.plan(0.0, hot)
        s, m, mem = replay(L, R, M.heat_index(cool), held_from=info["held_from"])
        self.assertLess(s, 0.5)

    def test_task_loads(self):
        self.assertEqual([o.name for o in M.OBJECTS], ["bench", "torch", "wire"])
        self.assertTrue(M.__doc__.strip().startswith("A vertical 1/2 in copper"))


class TaskJson(unittest.TestCase):
    def test_numbers_in_text_come_from_code(self):
        p = HERE / "task.json"
        if not p.exists():
            self.skipTest("task.json not generated yet")
        d = json.loads(p.read_text())
        self.assertEqual(d["instruction"], " ".join(M.__doc__.split()))
        sc = d["scoring"]
        for v in ("464", "452", "600", "8.1 mm", "8 cm", "3 cm", "0.25", "0.5", "0.6", "0.8", "1 s"):
            self.assertIn(v, sc)

    def test_readme_numbers_match_code(self):
        """The hand-written README numbers: the seeds' heat rates, the falloff, the melt and burn times."""
        rd = (HERE / "README.md").read_text()
        tm = [-M.TAU * math.log(1 - (M.LIQUIDUS - M.T_ROOM) / (t - M.T_ROOM)) for t in M.T_SS]
        tb = -M.TAU * math.log(1 - (M.BURN_T - M.T_ROOM) / (M.T_SS[0] - M.T_ROOM))
        for v in [f"{t:,.0f} F" for t in M.T_SS] + [f"sigma = {M.SIGMA * 1000:.0f} mm", f"tau = {M.TAU:.0f} s",
                                                     f"{tb:.1f} s", f"{max(tm):.1f} s"] + [f"{t:.0f} s" for t in tm]:
            self.assertIn(v, rd)


if __name__ == "__main__":
    unittest.main()
