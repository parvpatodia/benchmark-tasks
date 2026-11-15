"""Checks of flare_tool.py's geometry, forming model, scoring, text and demo, without the simulator. Needs yam_arena on
PYTHONPATH:

    PYTHONPATH=<yam-isaac-lab-arena checkout> python -m pytest -q -p no:cacheprovider
"""

import math
import sys
import warnings
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load

warnings.simplefilter("ignore")
HERE = Path(__file__).resolve().parent
TASK = load(HERE / "flare_tool.py")
M = TASK.module
sys.path.insert(0, str(HERE / "tools"))
import mechanism  # noqa: E402  (the task file put HERE on sys.path)
from geom import JOINTED, pair_gap, pieces  # noqa: E402

D = math.pi / 180
IN = 0.0254
TH_SHUT = M.TH_SHUT
TUBE_IN = (M.TUBE_X, M.HY, M.Z_BAR - M.TUBE_L)
S_END = M.HY - M.PARK_Y


# -- states -----------------------------------------------------------------------------------------------------------
def q_up():
    return np.array([0.0, 0.0, 0.0, 1.0])


def tube(top=None, xy=(M.TUBE_X, M.HY), pos=None, tilt=0.0):
    if pos is None:
        z = (M.Z_BAR if top is None else top) - M.TUBE_L
        pos = (xy[0], xy[1], z)
    q = np.array([math.sin(tilt * D / 2), 0.0, 0.0, math.cos(tilt * D / 2)])
    return mechanism.body_state(M.TUBE, pos, q)


def tool(slide=S_END, feed=0.0):
    z = np.zeros(3)
    return mechanism.MechanismState(pos=z, quat_xyzw=q_up(), lin_vel=z, ang_vel=z, low=z, high=z,
                                    joints={"hinge": TH_SHUT, "slide": slide, "feed": feed, "cone_z": 0.0,
                                            "clamp_turn": 0.0, "clamp_x": 0.0}, links={})


def far_hands():
    v = np.array([(x, y, z) for x in (1.9, 2.0) for y in (1.9, 2.0) for z in (1.9, 2.0)])
    return [(v, np.eye(3), v.mean(axis=0), 0.1)]


def near_hands():
    c = np.array([M.BX, M.HY, M.Z_BAR + 0.02])
    v = c + np.array([(x, y, z) for x in (-1e-3, 1e-3) for y in (-1e-3, 1e-3) for z in (-1e-3, 1e-3)])
    return [(v, np.eye(3), c, 0.002)]


def feed(m, turns, top=M.Z_BAR, xy=(M.TUBE_X, M.HY), cone=(M.BX, M.HY), clamp_act=None, slip=False, step=0.05):
    """Turn the handle ``turns`` (clockwise +) in small steps through the model. ``slip``: the tube moves down by
    the cone's push past its end (an unclamped bar does not hold it). Returns the tube's final top."""
    n = max(1, int(abs(turns) / step))
    for _ in range(n):
        ca = M.clamp_joint(m) if clamp_act is None else clamp_act
        M.advance(m, turns / n, 0.0, top, xy, True, cone, ca)
        if slip and m["pushing"]:
            top = min(top, m["tip_z"])
    return top


def clamped(seed=None):
    m = M._fresh_model(seed)
    M.advance(m, 0.0, 0.86, M.Z_BAR, (M.TUBE_X, M.HY), True, (M.BX, M.HY), 0.0005)   # 0.86 turn: tight
    return m


def formed(seed=None, iron=2.5, back=0.0):
    m = clamped(seed)
    feed(m, (M.tip_start(seed) + M.SEAT_D) / M.PITCH + iron, clamp_act=0.0005)
    if back:
        feed(m, -back, clamp_act=0.0005)
    return m


def run(seq, m, dt=0.05):
    """Score a sequence of (objects, hands) with the model state ``m``."""
    M.reset_memory()
    M._M.update(m)
    M._CLOCK["dt"] = dt
    s = 0.0
    for st, hands in seq:
        st = dict(st)
        M._HANDS["hands"] = hands
        M.success(st)
        s = M.score(st)
    return s


def done(top=M.Z_BAR, slide=S_END, feed_rad=0.0):
    return {"tool": tool(slide, feed_rad), "tube": tube(top)}


# -- text, sizes ----------------------------------------------------------------------------------------------------
def test_objects_and_joints():
    assert [o.name for o in M.OBJECTS] == ["tool", "tube"]
    assert [j.name for j in M.TOOL.joints] == list(M.JOINTS)
    assert abs(M.TUBE.mass - M.TUBE_KG) < 1e-9


def test_sizes_match_the_sources():
    assert M.TUBE_OD == pytest.approx(0.375 * IN) and M.TUBE_WALL == pytest.approx(0.030 * IN)   # CDA, 1/4 Type L
    assert M.TUBE_WALL <= 0.035 * IN                                                           # EC42865 maximum
    assert M.BAR_L == pytest.approx(7.5 * IN)
    assert M.SIZES_IN == (0.125, 0.1875, 0.25, 0.3125, 0.375, 0.5, 0.625, 0.75)               # 458R openings
    y = M.HINGE_Y
    for yc, s in zip(M.HOLE_Y, M.SIZES_IN, strict=True):
        assert yc - s * IN / 2 > y                                                              # openings in order
        y = yc + s * IN / 2
    assert y < M.HINGE_Y + M.BAR_L


def test_the_instruction_names_every_step():
    doc = " ".join(M.__doc__.split())
    for s in ("3/8 opening", "closed flaring bar", "even with the top of the bar", "arrow on the 3/8 line",
              "tighten the clamp screw", "clockwise", "clutch has released", "two more full turns",
              "back until the cone's tip is at least 2 mm over", "move both grippers away"):
        assert s in doc, s


# -- geometry ---------------------------------------------------------------------------------------------------------
def _gaps(q, tube_pose):
    P = pieces(q, tube_pose)
    out = {}
    names = list(P)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if frozenset((a, b)) not in JOINTED:
                out[(a, b)] = pair_gap(P[a], P[b], near=0.01)
    return out


def test_nothing_starts_inside_anything_but_the_spring_loaded_half():
    g = _gaps({"hinge": TH_SHUT}, M.TUBE_START)                     # the bar holding the tube: everything clear
    bad = {k: v for k, v in g.items() if v < -3e-5}
    assert not bad, bad
    assert g[("base", "tube")] == pytest.approx(0.0, abs=3e-5)        # both half-holes touch the tube
    assert g[("front_half", "tube")] == pytest.approx(0.0, abs=3e-5)
    g0 = _gaps({"hinge": 0.0}, M.TUBE_START)                         # the joints at 0, as the reset puts them
    bad = {k: v for k, v in g0.items() if v < -3e-5 and k != ("front_half", "tube")}
    assert not bad, bad
    assert -M.PRESS_FIT - 3e-5 <= g0[("front_half", "tube")] < -0.5 * M.PRESS_FIT   # the spring's 0.1 mm press fit
    assert M.TUBE_START[2] > M.TOP + 0.005                           # the tube hangs clear of the table
    assert M.TUBE_START[2] + M.TUBE_L - M.Z_BAR == pytest.approx(M.PROUD)


def test_no_collision_hull_is_under_1_mm_thick():
    """PhysX may inflate a convex hull thinner than its default minimum thickness (1 mm); a thin slice of a half-hole
    would then stand into the opening and spring the bar open on the tube."""
    from scipy.spatial import ConvexHull
    links = [M.TOOL.base, *M.TOOL.links]
    for link in links:
        for s in link.shapes:
            if s.kind == "hull" and s.collide:
                P = np.array(s.points)
                w = min(np.ptp(P @ e[:3]) for e in ConvexHull(P).equations)
                assert w > 0.0011, (link.name, w)


def test_the_hinge_stop_keeps_the_bare_faces_apart():
    P = pieces({"hinge": M.TOOL.joints[0].lower}, None)
    assert pair_gap(P["front_half"], P["base"], near=0.002) > 0.0


def test_the_cone_pushes_a_loose_tube_but_not_a_clamped_one():
    for tight in (False, True):
        m = clamped(1) if tight else M._fresh_model(1)
        m["tip0"] = 0.0                                   # the cone's tip on the tube's end at once
        thrust = M.advance(m, 0.01, 0.0, M.Z_BAR + 0.0001, (M.TUBE_X, M.HY), True, (M.BX, M.HY),
                           0.0005 if tight else 0.0)
        assert m["pushing"]
        assert m["tight"] == tight
        assert thrust == (0.0 if tight else M.THRUST)


def test_the_closed_bar_grips_the_tube():
    P = pieces({"hinge": TH_SHUT}, TUBE_IN)
    assert pair_gap(P["front_half"], P["tube"]) == pytest.approx(0.0, abs=3e-5)
    assert pair_gap(P["base"], P["tube"]) == pytest.approx(0.0, abs=3e-5)


def test_the_yoke_slides_over_the_set_tube_and_the_clamp_meets_the_bar_after_a_third_of_a_turn():
    for f in np.linspace(0, 1, 11):
        g = _gaps({"hinge": TH_SHUT, "slide": f * S_END}, TUBE_IN)
        for k in (("base", "tube"), ("front_half", "tube")):      # the bar's grip on the tube
            g.pop(k)
        assert min(g.values()) > 0.0003, (f, {k: v for k, v in g.items() if v < 0.0003})
    P = pieces({"hinge": TH_SHUT, "slide": S_END}, TUBE_IN)
    g0 = pair_gap(P["clamp_tip"], P["front_half"])
    assert 0.0003 < g0 < 0.0006                    # CLAMP_GAP less the 0.1 mm the held half stands open
    contact = g0 / M.CLAMP_PITCH
    presses = (M.T_END - (180.0 - M.TBAR_START) + (M.PRESSES - 1) * 180.0) / 360.0
    assert presses - contact >= M.CLAMP_TIGHT + 0.1, (presses, contact)


def test_the_t_bar_swings_clear_of_the_bar_and_the_tube():
    for a in range(-180, 181, 20):
        P = pieces({"hinge": TH_SHUT, "slide": S_END, "clamp_turn": -(a - M.TBAR_START) * D}, TUBE_IN)
        assert min(pair_gap(P["clamp"], P[k], near=0.01) for k in ("front_half", "base", "tube")) > 0.002, a


# -- the forming model --------------------------------------------------------------------------------------------------
def test_the_seed_sets_the_cone_start():
    assert [M.tip_start(s) for s in (0, 1, 2, 3)] == pytest.approx([0.002, 0.005, 0.008, 0.002])
    assert M.FWD_TURNS >= (M.TIP_START[1] + M.SEAT_D) / M.PITCH + M.IRON_TURNS     # the demo irons every seed 2 turns


def test_a_set_clamped_tube_flares_and_the_clutch_stops_the_cone():
    for seed in (0, 1, 2):
        m = formed(seed, iron=2.5)
        assert M.flare_good(m) and not M.flare_scrap(m)
        assert m["tip_z"] == pytest.approx(M.Z_BAR - M.SEAT_D)
        assert m["iron"] >= M.IRON_TURNS
        before = m["tip_z"]
        feed(m, 3.0, clamp_act=0.0005)                                   # more turns: the clutch slips
        assert m["tip_z"] == pytest.approx(before) and m["iron"] >= M.IRON_TURNS + 3.0 - 1e-6


def test_turning_back_lifts_the_cone_at_once():
    m = formed(2, iron=2.5)
    feed(m, -1.0, clamp_act=0.0005)
    assert m["tip_z"] == pytest.approx(M.Z_BAR - M.SEAT_D + M.PITCH)
    feed(m, -(M.BACK_TURNS - 1.0), clamp_act=0.0005)
    assert M.retracted(m) and M.flare_good(m)


def test_the_cone_pushes_the_tube_only_until_the_flare_is_formed():
    m = clamped(0)
    pushes = []
    for _ in range(120):
        M.advance(m, 0.05, 0.0, M.Z_BAR, (M.TUBE_X, M.HY), True, (M.BX, M.HY), 0.0005)
        pushes.append(m["pushing"])
    assert any(pushes) and not pushes[-1] and m["formed"]


@pytest.mark.parametrize("top,ok", [(M.Z_BAR + 0.0004, True), (M.Z_BAR - 0.0004, True), (M.Z_BAR + 0.0007, False),
                                    (M.Z_BAR - 0.0007, False), (M.Z_BAR + 0.003, False)])
def test_the_end_must_be_even_with_the_top(top, ok):
    m = clamped(1)
    feed(m, 5.0, top=top, clamp_act=0.0005)
    assert m["formed"] and M.flare_good(m) is ok and M.flare_scrap(m) is (not ok)


def test_an_unclamped_tube_slips_and_scraps():
    m = M._fresh_model(1)
    top = feed(m, 5.0, slip=True, clamp_act=0.0)
    assert top < M.Z_BAR - M.FLARE_OK and M.flare_scrap(m)
    assert not m["tight"]


def test_a_tube_held_even_but_not_clamped_when_the_cone_meets_it_scraps():
    m = M._fresh_model(1)                                   # the end held even (no slip), the clamp loose
    feed(m, 5.0, clamp_act=0.0)
    assert m["formed"] and abs(m["p"]) <= M.FLARE_OK and M.flare_scrap(m)
    m = M._fresh_model(1)                                   # clamped only after the cone met the tube: still scrap
    feed(m, (M.tip_start(1) + 0.0002) / M.PITCH, clamp_act=0.0)
    assert m["touched"]
    M.advance(m, 0.0, 0.86, M.Z_BAR, (M.TUBE_X, M.HY), True, (M.BX, M.HY), 0.0005)
    feed(m, 4.0, clamp_act=0.0005)
    assert m["formed"] and M.flare_scrap(m) and not M.flare_good(m)


def test_a_cone_brought_over_the_tube_from_the_side_scraps():
    m = clamped(1)
    far = (M.BX, M.HY + M.TUBE_OD / 2 + 0.005)              # the yoke off the tube: the cone comes down beside it
    feed(m, (M.tip_start(1) + 0.001) / M.PITCH, cone=far, clamp_act=0.0005)
    assert not m["touched"] and m["tip_z"] < M.Z_BAR - M.STRIKE_D
    feed(m, 4.0, clamp_act=0.0005)                          # then slid over the tube
    assert m["struck"] and M.flare_scrap(m)


def test_a_yoke_slide_scripted_for_one_seed_ends_off_the_line_on_the_others():
    _, r0, _ = M.plan(0.0, 0)
    grip = next(w for w in r0 if w.label == "close on the wing").pos[1]
    slid = next(w for w in r0 if w.label == "slide the yoke to the 3/8 line").pos[1]
    for seed in (3, 6):
        hub = M.PARK_Y + M.park_off(seed)
        _, rs, _ = M.plan(0.0, seed)
        assert abs(next(w for w in rs if w.label == "close on the wing").pos[1] - hub) < 1e-9
        # the wing lies along y and the fingers close along x, so a seed-0 grip holds the wing (it is within its
        # length) without centring it: the hub keeps its offset from the fingers and ends that far off the line
        assert abs(grip - hub) < M.WING_L / 2 - 0.01
        assert abs(slid + (hub - grip) - M.HY) > 3 * M.CENTRE_TOL


def test_turns_with_the_clamp_loosened_do_not_iron():
    m = formed(1, iron=0.0)
    feed(m, 3.0, clamp_act=M.clamp_joint(m))                # the tip not pressing the bar: the clamp is not tight
    assert m["formed"] and not m["tight"] and m["iron"] == 0.0


def test_the_cone_off_the_tube_axis_scraps():
    m = clamped(1)
    feed(m, 5.0, cone=(M.BX, M.HY + 0.0013), clamp_act=0.0005)
    assert M.flare_scrap(m)
    m = clamped(1)
    feed(m, 5.0, cone=(M.BX, M.HY + 0.0007), clamp_act=0.0005)
    assert M.flare_good(m)


def test_the_cone_away_from_the_tube_forms_nothing():
    m = clamped(1)
    feed(m, 6.0, cone=(M.BX, M.HY + 0.02), clamp_act=0.0005)
    assert not m["formed"] and not M.flare_scrap(m)


def test_a_loose_clamp_is_not_tight():
    m = M._fresh_model()
    M.advance(m, 0.0, 0.3, M.Z_BAR, (M.TUBE_X, M.HY), True, (M.BX, M.HY), 0.00045)
    assert not m["tight"]
    M.advance(m, 0.0, 0.6, M.Z_BAR, (M.TUBE_X, M.HY), True, (M.BX, M.HY), 0.0005)
    assert m["tight"]


# -- scoring -------------------------------------------------------------------------------------------------------
HOLD = int(M.HOLD_S / 0.05) + 2      # steps at 20 Hz to hold success 1 s


def test_done_clear_and_still_succeeds_once_held_1_s():
    m = formed(1, back=M.BACK_TURNS)
    assert run([(done(), far_hands())] * HOLD, m) == 1.0
    assert run([(done(), far_hands())] * 3, m) == 0.8                      # not yet held 1 s
    M.reset_memory()
    M._M.update(m)
    M._CLOCK["dt"] = 0.05
    M._HANDS["hands"] = far_hands()
    M.update(done())
    st = done()
    assert M.success(st)


def test_partial_credit_ladder():
    h = far_hands()
    assert run([(done(), near_hands())] * HOLD, formed(1, back=M.BACK_TURNS)) == 0.8
    assert run([(done(), h)] * HOLD, formed(1)) == 0.6                        # ironed, cone still down
    assert run([(done(), h)] * HOLD, formed(1, iron=1.0)) == 0.5               # formed, ironed 1 turn
    assert run([(done(), h)] * HOLD, clamped(1)) == 0.25                       # set even and clamped
    assert run([(done(), h)] * HOLD, M._fresh_model(1)) == 0.0                 # set even, not clamped
    assert run([(done(top=M.Z_BAR + 0.002), h)] * HOLD, clamped(1)) == 0.0     # clamped, end 2 mm proud
    tiers = [M.LADDER[k] for k in sorted(M.LADDER)] + [0.8, 1.0]
    assert tiers == sorted(tiers) and len(set(tiers)) == len(tiers)


def test_a_scrap_flare_latches_zero():
    m = clamped(1)
    feed(m, 5.0, top=M.Z_BAR + 0.001, clamp_act=0.0005)
    assert run([(done(), far_hands())] * HOLD, m) == 0.0


def test_a_lost_tube_latches_zero():
    lost = {"tool": tool(0.0), "tube": tube(pos=(0.2, -0.3, M.TOP - 0.05))}
    m = formed(1, back=M.BACK_TURNS)
    assert run([(lost, far_hands()), (done(), far_hands()), (done(), far_hands())], m) == 0.0


def test_unread_hands_never_score_clear():
    assert run([(done(), None)] * HOLD, formed(1, back=M.BACK_TURNS)) == 0.8


def test_a_moving_handle_is_not_still():
    m = formed(1, back=M.BACK_TURNS)
    seq = [(done(feed_rad=0.0), far_hands()), (done(feed_rad=0.2), far_hands())] * HOLD
    assert run(seq, m) == 0.8


def test_a_model_error_latches_zero():
    m = formed(1, back=M.BACK_TURNS)
    m["error"] = "step RuntimeError()"
    assert run([(done(), far_hands())] * HOLD, m) == 0.0


# -- the demo ------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("delay", [0.0, 0.05, 0.10])
def test_demo_tracks_with_ik_under_0_1_mm(delay):
    left, right, _ = M.plan(delay)
    for arm, ws in (("left", left), ("right", right)):
        p = ArmPlan(ws, station.BASES[arm])
        t = 0.0
        while t <= p.duration + 1e-9:
            p.action7(t)
            t += 0.05
        assert p.worst_ik_mm < 0.1 and p.worst_ik_deg < 0.1, (arm, p.worst_ik_mm, p.worst_ik_deg)


def test_demo_turns_cover_every_seed_and_fit_the_run_limit():
    left, right, info = M.plan()
    assert info["fwd"] == pytest.approx(M.FWD_TURNS) and info["back"] == pytest.approx(M.BACK_TURNS)
    dur = max(sum(w.duration for w in ws) for ws in (left, right))
    assert (dur + 1.0) * 20 < M.RUN_LIMIT_STEPS


def test_task_json_is_generated_from_the_code():
    import json
    import subprocess
    d = json.loads((HERE / "task.json").read_text())
    assert d["instruction"] == " ".join(M.__doc__.split())
    before = (HERE / "task.json").read_text()
    subprocess.run([sys.executable, "-W", "ignore", str(HERE / "tools" / "gen_taskjson.py")], check=True,
                   capture_output=True)
    assert (HERE / "task.json").read_text() == before, "task.json text is stale: rerun tools/gen_taskjson.py"
    assert len(d["fidelity"]) <= 500


def test_a_fixed_turn_count_fails_some_seed():
    for turns in (4.0, 5.5):
        results = []
        for seed in (0, 1, 2):
            m = clamped(seed)
            feed(m, turns, clamp_act=0.0005)
            results.append(M.flare_good(m) and m["iron"] >= M.IRON_TURNS)
        assert not all(results), turns


def test_the_clamp_must_be_on_the_3_8_line_for_stage_1():
    h = far_hands()
    assert run([(done(), h)] * HOLD, clamped(1)) == 0.25
    assert run([(done(slide=S_END + 0.0015), h)] * HOLD, clamped(1)) == 0.0


def test_a_stalled_model_latches_zero():
    m = formed(1, back=M.BACK_TURNS)
    M.reset_memory()
    M._M.update(m)
    M._CLOCK["dt"] = 0.05
    M._HANDS["hands"] = far_hands()
    M._HANDS["steps"] = M._M["steps"] + 10
    st = done()
    M.success(st)
    assert M.score(st) == 0.0
