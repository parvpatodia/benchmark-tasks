"""Checks of server_rail.py's scoring, layout and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_server_rail.py
"""

import importlib.util
import math
import sys
import warnings
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.analysis import fk_link_poses_world, world_hulls
from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import ArmPlan
from yam_arena.task_file import Box, ObjectState, load

HERE = Path(__file__).resolve().parent
TASK = load(HERE / "server_rail.py")
M = TASK.module
sys.path.insert(0, str(HERE))
import mechanism  # noqa: E402
from mechanism import MechanismState  # noqa: E402


def _spec(name):
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


MESHES = _spec("make_meshes")
H = M.RAIL_L / 2


# -- states ------------------------------------------------------------------------------------------------------------
def _quat(rot):
    m = rot
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    q[3] = (m[k, j] - m[j, k]) / s
    return q


def rail(nose_u, v=0.0, dz=0.0, yaw=0.0, pitch=0.0, vel=0.0, spin=0.0, flipped=False):
    """The inner member with its tapered end at nose_u (slide frame), its centre v across and dz above riding height,
    turned yaw degrees flat and pitch degrees nose-up off the channel's line."""
    y, p = math.radians(yaw), math.radians(pitch)
    au = np.array([-math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), math.sin(p)])  # slide frame
    a = np.array([-au[1], au[0], au[2]])                                                 # world
    if flipped:
        a = -a
    up = np.array([0.0, 0.0, 1.0])
    c2 = up - a * float(up @ a)
    c2 /= np.linalg.norm(c2)
    rot = np.column_stack([a, np.cross(c2, a), c2])
    cu = nose_u - au[0] * H
    centre = np.array(M.world(cu, v, M.RAIL_ZC + dz))
    return ObjectState(pos=centre, quat_xyzw=_quat(rot), lin_vel=np.array([0.0, vel, 0.0]),
                       ang_vel=np.array([0.0, 0.0, spin]), low=centre, high=centre)


def rail_start():
    q = np.array([0.0, 0.0, math.sin(-math.pi / 4), math.cos(-math.pi / 4)])
    c = np.array([M.RAIL_START[0], M.RAIL_START[1], M.TOP + M.RAIL_T / 2])
    return ObjectState(pos=c, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=c, high=c)


def retainer(q, vel=0.0):
    z = np.zeros(3)
    cage = ObjectState(pos=np.array([0.0, q, 0.0]), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.array([0, vel, 0]),
                       ang_vel=z, low=z, high=z)
    return MechanismState(pos=np.array(M.world(0, 0, 0)), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=z, ang_vel=z,
                          low=z, high=z, joints={"retainer": q}, links={"cage": cage})


def objs(r, q, cage_v=0.0):
    return {"inner_rail": r, "retainer": retainer(q, cage_v)}


def q_after(q0, nose_u):
    """The cage's travel after rolling from first contact (cage at q0) until the tapered end is at nose_u."""
    return q0 - ((M.U_FE0 + q0) - nose_u) / 2


QF = M.Q_FRONT
HOME_Q = q_after(QF, M.U_HOME)


@pytest.fixture(autouse=True)
def fresh():
    M.reset_memory()
    yield
    M.reset_memory()


def play(states):
    """Score a sequence of states as the trial would, returning the last (score, success)."""
    out = None
    for s in states:
        out = (M.score(s), M.success(s))
    return out


def correct_run(end_nose=M.U_HOME, **kw):
    """Retainer pushed forward, then the member slid in to end_nose."""
    seq = [objs(rail_start(), 0.0), objs(rail_start(), QF), objs(rail(M.SLIDE_L + 0.01), QF)]
    for u in np.linspace(M.U_ENTRY + 0.001, end_nose, 12):
        seq.append(objs(rail(u), q_after(QF, min(u, M.U_ENTRY))))
    last = seq[-1]
    seq[-1] = objs(rail(end_nose, **kw), last["retainer"].joints["retainer"])
    return seq


# Thresholds are written as literals: they are the README's numbers, so a changed constant must fail a test.
# -- scoring -----------------------------------------------------------------------------------------------------------
def test_start_scores_zero():
    assert M.score(objs(rail_start(), 0.0)) == 0.0
    assert not M.success(objs(rail_start(), 0.0))


def test_retainer_forward_threshold_both_sides():
    assert M.score(objs(rail_start(), QF)) == 0.2
    assert M.score(objs(rail_start(), QF - 0.0149)) == 0.2
    assert M.score(objs(rail_start(), QF - 0.0151)) == 0.0


def test_correct_run_succeeds_and_score_caps_below_one():
    s, ok = play(correct_run())
    assert ok and s == 0.9
    assert s < 1.0


def test_score_never_one_on_a_grid():
    for u in np.linspace(M.U_HOME - 0.002, M.SLIDE_L, 40):
        M.reset_memory()
        M.score(objs(rail(M.U_ENTRY + 0.0005), QF))
        for qq in (QF, HOME_Q):
            assert M.score(objs(rail(u), qq)) < 1.0


def test_progress_tiers_ordered():
    M.reset_memory()
    first = play(correct_run(end_nose=M.U_ENTRY))[0]
    M.reset_memory()
    mid = play(correct_run(end_nose=(M.U_ENTRY + M.U_HOME) / 2))[0]
    M.reset_memory()
    near = play(correct_run(end_nose=M.U_HOME + M.HOME_TOL + 0.0005))[0]
    assert 0.3 <= first < mid < near <= 0.7
    assert abs(mid - 0.5) < 0.01


def test_home_threshold_both_sides():
    assert play(correct_run(end_nose=M.U_HOME + 0.0019)) == (0.9, True)
    M.reset_memory()
    s, ok = play(correct_run(end_nose=M.U_HOME + 0.0021))
    assert not ok and 0.69 < s <= 0.7


@pytest.mark.parametrize("kw", [dict(vel=0.02), dict(vel=-0.02), dict(spin=0.6), dict(spin=-0.6)])
def test_home_but_rail_moving(kw):
    s, ok = play(correct_run(**kw))
    assert s == 0.75 and not ok


def test_home_but_retainer_moving():
    seq = correct_run()
    last = seq[-1]
    seq[-1] = objs(last["inner_rail"], last["retainer"].joints["retainer"], cage_v=0.02)
    s, ok = play(seq)
    assert s == 0.75 and not ok


def test_still_just_under_thresholds():
    s, ok = play(correct_run(vel=0.009, spin=0.45))
    assert ok and s == 0.9


@pytest.mark.parametrize("name,inside,outside", [
    ("v", 0.0013, 0.0017),
    ("dz", 0.0013, 0.0017),
    ("yaw", 1.4, 1.6),
    ("pitch", 1.4, 1.6),
])
@pytest.mark.parametrize("sign", [1, -1])
def test_alignment_thresholds_both_signs(name, inside, outside, sign):
    M.reset_memory()
    assert play(correct_run(**{name: sign * inside})) == (0.9, True)
    M.reset_memory()
    s, ok = play(correct_run(**{name: sign * outside}))
    assert not ok and s < 0.3


def test_flipped_member_is_not_home():
    s, ok = play(correct_run(flipped=True))
    assert not ok and s < 0.3


def test_contact_threshold_both_sides():
    fe = M.U_ENTRY
    assert M.score(objs(rail(fe + 0.0023), QF)) == 0.2
    assert M.score(objs(rail(fe + 0.0017), QF)) >= 0.3


def test_wrong_order_scores_zero_for_good():
    """Member run into the retainer while it is back: 0 now, and still 0 after pulling out, pushing the retainer
    forward and going home."""
    seq = [objs(rail_start(), 0.0), objs(rail(M.SLIDE_L + 0.01), 0.0)]
    for u in np.linspace(M.SLIDE_L - 0.005, M.U_FE0, 8):
        seq.append(objs(rail(u), 0.0))
    seq.append(objs(rail(M.U_FE0 - 0.02), M.Q_BACK))     # stalled against the back stop
    assert play(seq) == (0.0, False)
    seq2 = [objs(rail(M.SLIDE_L + 0.01), M.Q_BACK + 0.01), objs(rail_start(), QF)] + correct_run()[2:]
    assert play(seq2) == (0.0, False)


def test_wrong_order_partly_forward_is_wrong():
    q = QF - M.FORWARD_K - 0.001
    seq = [objs(rail(M.SLIDE_L + 0.01), q), objs(rail(M.U_FE0 + q + 0.001), q)]
    assert play(seq) == (0.0, False)


def test_right_then_wrong_contact_is_wrong():
    seq = [objs(rail(M.SLIDE_L + 0.01), QF), objs(rail(M.U_ENTRY), QF), objs(rail(M.SLIDE_L + 0.01), QF),
           objs(rail(M.SLIDE_L + 0.01), 0.0), objs(rail(M.U_FE0), 0.0)]
    assert play(seq) == (0.0, False)


def test_home_without_a_recorded_first_contact_is_wrong():
    """A home state seen with no earlier step is a contact with the retainer back (it rolled back): not credited."""
    assert play([objs(rail(M.U_HOME), HOME_Q)]) == (0.0, False)


def test_engaged_implies_ok():
    """Why done() and score() need no separate mutant for `ok`: any engaged state has met the retainer, so order() has
    set ok or bad, whatever came before."""
    for q in (0.0, QF - 0.02, QF):
        for u in (M.U_FE0 + q + 0.0019, M.U_FE0 + q - 0.02, M.U_HOME + 0.001):
            M.reset_memory()
            s = objs(rail(u), q)
            assert M.engaged(s)
            ok, bad = M.order(s)
            assert ok != bad


def test_second_contact_with_retainer_just_short_is_wrong():
    """Met forward (12 mm short), pulled out, retainer moved to 16 mm short (not forward, yet within the rolling
    allowance from the first touch), met again: a miss, from the first-touch check alone."""
    q1, q2 = QF - 0.012, QF - 0.016
    play([objs(rail_start(), q1), objs(rail(M.SLIDE_L + 0.01), q1), objs(rail(M.U_FE0 + q1 + 0.001), q1)])
    assert M._MEM["ok"] and not M._MEM["bad"]
    play([objs(rail(M.SLIDE_L + 0.01), q1), objs(rail(M.SLIDE_L + 0.01), q2), objs(rail(M.U_FE0 + q2 + 0.001), q2)])
    assert M._MEM["bad"]


def test_order_memory_is_idempotent():
    s = objs(rail(M.U_FE0 + 0.001), 0.0)
    a = M.order(s)
    b = M.order(s)
    assert a == b == (False, True)


def test_reset_clears_memory_and_is_registered():
    play([objs(rail(M.U_FE0), 0.0)])
    assert M._MEM["bad"]
    assert M.reset_memory in mechanism._RESET
    for fn in mechanism._RESET:
        fn()
    assert not M._MEM["bad"] and not M._MEM["ok"]


def test_member_on_the_lips_or_dropped_low_earns_nothing_extra():
    on_lips = rail(M.U_HOME, dz=M.LIP_Z1 + M.RAIL_T / 2 - M.RAIL_ZC)
    assert M.score(objs(on_lips, QF)) == 0.2
    M.reset_memory()
    low = rail(0.200, dz=M.WEB_T + M.RAIL_T / 2 - M.RAIL_ZC)    # resting on the web ahead of the rollers
    assert M.score(objs(low, QF)) == 0.2


def test_member_pulled_out_keeps_only_the_first_step():
    seq = correct_run(end_nose=0.1)
    play(seq)
    assert M.score(objs(rail_start(), HOME_Q)) == 0.2


def _on_table(nose_u, yaw=0.0):
    return rail(nose_u, yaw=yaw, dz=M.RAIL_T / 2 - M.RAIL_ZC)


def _nose_at(nose_u, z, pitch):
    """The member pitched nose-down with its tapered end's centre at height z (slide frame)."""
    r = rail(nose_u, pitch=pitch)
    n = M.rail_pose(objs(r, 0.0))[1]
    return rail(nose_u, pitch=pitch, dz=z - n[2])


@pytest.mark.parametrize("name,state", [
    ("behind the slide on the table", lambda: _on_table(-0.020, yaw=180.0)),
    ("nose dipped through the top opening behind the cage", lambda: _nose_at(0.050, 0.018, -40.0)),
    ("nose hovering at the top opening", lambda: _nose_at(0.100, 0.02375, -60.0)),
])
def test_ends_outside_the_cavity_do_not_meet_the_retainer(name, state):
    """Found in review: these set the miss with the retainer at its start, so a later perfect run scored 0."""
    assert not M.touching(objs(state(), 0.0))
    play([objs(rail_start(), 0.0), objs(state(), 0.0)])
    assert play(correct_run()) == (0.9, True)


@pytest.mark.parametrize("sign", [1, -1])
def test_entry_angle_gate_both_sides(sign):
    """The tapered end held at the channel's centre at riding height, the member turned about it."""
    def turned(**kw):
        n = M.rail_pose(objs(rail(M.U_FE0 - 0.01, **kw), 0.0))[1]
        return objs(rail(M.U_FE0 - 0.01, v=-n[1], dz=M.RAIL_ZC - n[2], **kw), 0.0)
    for key in ("yaw", "pitch"):
        assert M.touching(turned(**{key: sign * 14.0}))
        assert not M.touching(turned(**{key: sign * 16.0}))


def test_cavity_bounds_both_sides():
    def at(u, v, z):
        r = rail(u)
        n = M.rail_pose(objs(r, 0.0))[1]
        return objs(rail(u, v=v, dz=z - n[2]), 0.1)  # retainer front edge past every u used here
    assert M.touching(at(0.001, 0.0, 0.011)) and not M.touching(at(-0.001, 0.0, 0.011))
    assert M.touching(at(0.1, 0.0159, 0.011)) and not M.touching(at(0.1, 0.0161, 0.011))
    assert M.touching(at(0.1, -0.0159, 0.011)) and not M.touching(at(0.1, -0.0161, 0.011))
    assert M.touching(at(0.1, 0.0, 0.0214)) and not M.touching(at(0.1, 0.0, 0.0216))


def test_retainer_driven_back_while_touching_is_wrong():
    """Found in review: the nose held against the front rollers while the retainer is driven back, then pushed in."""
    seq = [objs(rail_start(), 0.0), objs(rail_start(), QF), objs(rail(M.SLIDE_L + 0.01), QF)]
    for q in np.linspace(QF, 0.0, 30):
        seq.append(objs(rail(M.U_FE0 + q + 0.001), q))
    for u in np.linspace(M.U_FE0 + 0.001, M.U_FE0 + 2 * M.Q_BACK, 10):
        seq.append(objs(rail(u), q_after(0.0, u)))
    assert play(seq) == (0.0, False)


def test_rolling_lag_threshold_both_sides():
    """An end gone d further in since the first touch allows the retainer down to q_first - d/2 - 5 mm."""
    for q0 in (QF, QF - 0.010):
        e0 = M.U_FE0 + q0 + 0.001
        for q, bad in ((q0 - 0.050 - 0.0049, False), (q0 - 0.050 - 0.0051, True)):
            M.reset_memory()
            play([objs(rail_start(), q0), objs(rail(M.SLIDE_L + 0.01), q0), objs(rail(e0), q0), objs(rail(e0 - 0.1), q)])
            assert M._MEM["bad"] == bad, (q0, q)


@pytest.mark.parametrize("flipped", [False, True])
def test_shove_then_retry_is_wrong(flipped):
    """Found in the second review: an end held against the front rollers' faces shoves the forward retainer back as far
    as it goes (not half); pulled out, the retainer pushed forward again and the member put in right must not pay."""
    seq = [objs(rail_start(), QF), objs(rail(M.SLIDE_L + 0.01), QF)]
    for d in np.linspace(0.0, 0.035, 8):
        q = QF - d
        seq.append(objs(rail(M.U_FE0 + q, flipped=flipped), q))
    seq += [objs(rail_start(), QF - 0.035), objs(rail_start(), QF)]
    play(seq)
    assert M._MEM["bad"]
    assert play(correct_run()[1:]) == (0.0, False)


def test_meeting_reaches_the_cage_front_bar():
    """The order check starts at the cage's front bar, 157 mm from the back end plus the retainer's travel, 2 mm early;
    being in the riding position still waits for the front rollers (147.5 mm)."""
    for q in (0.0, QF):
        assert M.meets(objs(rail(0.157 + q + 0.0019), q)) and not M.meets(objs(rail(0.157 + q + 0.0021), q))
        assert not M.touching(objs(rail(0.157 + q), q))


def test_high_member_striking_the_bar_with_the_retainer_back_is_wrong():
    """Found in the third review: carried 5 mm high, the member passes under the lips and strikes the cage's front bar
    before the rollers; with the retainer back, that strike shoves it to its back stop. Pulled out and redone in order,
    it must not pay."""
    bar = M.CAGE_BACK0 + M.CAGE_L
    seq = [objs(rail_start(), 0.0)]
    seq += [objs(rail(u, dz=0.005), 0.0) for u in np.linspace(M.SLIDE_L + 0.01, bar, 15)]
    seq += [objs(rail(bar + q, dz=0.005), q) for q in np.linspace(0.0, M.Q_BACK, 10)]
    seq += [objs(rail_start(), M.Q_BACK), objs(rail_start(), QF)] + correct_run()[2:]
    assert play(seq) == (0.0, False)


def test_bar_shove_of_a_forward_retainer_trips_after_5_mm():
    """Before the end reaches the rollers nothing rolls, so a shove at the bar may move the retainer only 5 mm."""
    bar = M.CAGE_BACK0 + M.CAGE_L
    for d, bad in ((0.0049, False), (0.0051, True)):
        M.reset_memory()
        play([objs(rail_start(), QF), objs(rail(M.SLIDE_L + 0.01, dz=0.005), QF), objs(rail(bar + QF, dz=0.005), QF),
              objs(rail(bar + QF - d, dz=0.005), QF - d)])
        assert M._MEM["bad"] == bad, d


def test_square_end_first_strike_is_wrong():
    """Put in backwards with the retainer back, the square end strikes it: remembered, as with the tapered end."""
    back = rail(M.U_FE0 - 0.005, flipped=True)
    play([objs(rail_start(), 0.0), objs(back, 0.0)])
    assert M._MEM["bad"]
    assert play([objs(rail_start(), QF)] + correct_run()[2:]) == (0.0, False)


def test_misaligned_partial_insert_never_scores_below_the_first_step():
    """Found in review: right order, member partly in but held 1.6 mm high or 1.6 deg off scored 0, below 0.2."""
    play(correct_run(end_nose=0.137))
    q = q_after(QF, 0.137)
    assert M.score(objs(rail(0.137), q)) > 0.3
    assert M.score(objs(rail(0.137, dz=0.0016), q)) == 0.2
    assert M.score(objs(rail(0.137, yaw=1.6), q)) == 0.2


# -- the mechanism's numbers ---------------------------------------------------------------------------------------------
def test_stroke_arithmetic():
    # from the front stop the cage ends 10 mm before its back stop; 20 mm short of the front stop it ends at the stop
    assert HOME_Q - M.Q_BACK == pytest.approx(0.010, abs=1e-6)
    assert q_after(QF - M.K_MAX, M.U_HOME) == pytest.approx(M.Q_BACK, abs=1e-9)
    # from the start (90 mm back) the member stalls when the cage reaches its back stop
    stall = M.U_FE0 + 2 * M.Q_BACK
    assert stall - M.U_HOME == pytest.approx(0.070, abs=0.0005)
    assert M.FORWARD_K < M.K_MAX


def test_layout_clearances():
    b = M.CAGE_BACK0
    # cage and rollers clear of the stop block at the back stop, and of the guide block at the front stop
    assert b + M.Q_BACK > M.STOP_U[1] + 0.005
    assert b + M.STATIONS[0] - M.ROLL_R + M.Q_BACK > M.STOP_U[1] + 0.005
    assert M.U_ENTRY < M.SHELF_U[0] - 0.002
    assert b + M.CAGE_L + M.Q_FRONT <= M.SLIDE_L - 0.002
    # heights: lower rollers on the web carry the member at riding height; upper rollers dip into its path and are
    # pushed up within their travel; the cage's frame and the lips clear it
    assert M.WEB_T + 2 * M.ROLL_R == pytest.approx(M.RAIL_Z0)
    lift = M.PRELOAD_DIP
    assert 0 < lift < 0.0015
    assert M.CAGE_Z0 > M.RAIL_Z0 + M.RAIL_T + 0.0015
    assert M.CAGE_Z1 < M.LIP_Z0
    assert M.RAIL_Z0 + M.RAIL_T + lift + 2 * M.ROLL_R < M.LIP_Z0
    assert M.RAIL_W / 2 > M.LIP_IN                       # cannot be dropped in from above
    assert 0.0005 < M.WALL_IN - M.RAIL_W / 2 < 0.002
    assert M.RAIL_Z0 - (M.RAIL_Z0 - 0.0005) == pytest.approx(0.0005)  # guide block 0.5 mm under the member
    for j in M.RETAINER.joints:
        if j.kind == "prismatic":
            assert math.isfinite(j.lower) and math.isfinite(j.upper)
        else:
            assert j.lower == -math.inf and j.upper == math.inf


def test_preload_and_drag_forces():
    """Spring forces at the working points, and the ratios the order depends on [ARITH from the task's constants]."""
    f_top = M.TOP_PRELOAD_K * (M.PRELOAD_DIP - 0.0002 - M.TOP_PRELOAD_T)  # carrier pushed from -0.2 mm to +0.8 mm
    shoe = M.SHOE_K * (-0.0005 - M.SHOE_T)
    drag = 1.2 * shoe
    taper = math.atan(MESHES.NOSE_DW / MESHES.NOSE_L)
    climb = f_top * math.tan(taper)
    stall = 1.2 * f_top
    assert 39.0 < f_top < 42.0 and drag == pytest.approx(7.2)
    assert climb < drag / 2            # an upper roller climbing the taper does not shove the cage back
    assert stall > 2.5 * drag          # a stalled front station resists far more than sliding the retainer


def test_mechanism_sizes_positive_and_shapes_clear_of_fixed_boxes():
    boxes = {o.name: o for o in M.OBJECTS if isinstance(o, Box) and not isinstance(o, mechanism.Mechanism)}
    for link in M.RETAINER.all_links():
        for s in link.shapes:
            assert all(v > 0 for v in s.size)
    # every moving shape at both cage stops is outside every fixed box, except the rollers and shoe resting on the web
    for q in (M.Q_BACK, 0.0, M.Q_FRONT):
        for link in M.RETAINER.links:
            for s in link.shapes:
                lo, hi = s.bounds()
                lo = lo + np.array([q, 0, 0])
                hi = hi + np.array([q, 0, 0])
                for name, bx in boxes.items():
                    c = np.asarray(bx.pos)
                    half = np.asarray(bx.size) / 2
                    # the fixed box in the slide frame (u along world y, v = -x)
                    blo = np.array([c[1] - half[1] - M.Y0, M.X_C - c[0] - half[0], c[2] - half[2] - M.TOP])
                    bhi = np.array([c[1] + half[1] - M.Y0, M.X_C - c[0] + half[0], c[2] + half[2] - M.TOP])
                    gap = np.max(np.maximum(blo - hi, lo - bhi))
                    if name == "web" and (link.name.startswith("low_roller") or link.name == "shoe"):
                        assert gap > -0.0006, (link.name, name, q, gap)
                    else:
                        assert gap > 0.0004, (link.name, name, q, gap)


def test_mesh_matches_the_task():
    pts = np.array(MESHES.rail_points())
    assert pts[:, 0].max() - pts[:, 0].min() == pytest.approx(M.RAIL_L)
    assert pts[:, 1].max() - pts[:, 1].min() == pytest.approx(M.RAIL_W)
    assert pts[:, 2].max() - pts[:, 2].min() == pytest.approx(M.RAIL_T)
    assert abs(pts.mean(axis=0)[1]) < 1e-9 and abs((pts[:, 0].max() + pts[:, 0].min()) / 2) < 1e-9
    assert MESHES.NOSE_L == M.NOSE_L
    assert (HERE / f"{M.RAIL_ASSET}.usda").read_text() == MESHES.usda()
    assert math.degrees(math.atan(MESHES.NOSE_DW / MESHES.NOSE_L)) < 40  # a ramp the rollers roll up at friction 1.2


def test_task_file_loads():
    assert TASK.scene == "station"
    assert "inner_rail" in TASK.free_objects
    assert TASK.success_hold_s == 1.0
    assert TASK.score_over == "end"


# -- demo --------------------------------------------------------------------------------------------------------------
def _plan(arm, delay=0.0):
    return ArmPlan(M.demo(delay)[arm], station.BASES[arm])


def _run(plan, every=0.05):
    t = 0.0
    while t <= plan.duration + 1e-9:
        plan.action7(t)
        t += every
    return plan


@pytest.mark.parametrize("delay", [0.0, 0.05, 0.1])
def test_demo_reaches(delay):
    for arm in ("left", "right"):
        p = _run(_plan(arm, delay))
        assert p.worst_ik_mm < 2.0, (arm, p.worst_ik_mm)
        assert p.worst_ik_deg < 2.0, (arm, p.worst_ik_deg)


def test_demo_fits_the_run_limit_and_takes_turns():
    d = M.demo(0.1)
    total = max(sum(w.duration for w in d[a]) for a in d)
    assert total + 1.5 < 2400 / 20
    t_right_done = sum(w.duration for w in d["right"][:7])   # lifted out of the channel
    t_left_near = sum(w.duration for w in d["left"][:7])     # over the mouth's line
    assert t_right_done < t_left_near
    labels = [w.label for w in d["left"]]
    slide = d["left"][labels.index("slide in")]
    nose = slide.pos[1] - M.Y0 - (M.RAIL_L - M.GRIP_FROM_END)
    assert nose >= M.U_HOME + 0.005                          # let go short of home: never seated while gripped


def _fingers(arm, wp):
    plan = ArmPlan([wp.__class__(3.0, wp.pos, wp.finger, wp.label, wp.yaw)], station.BASES[arm])
    a, _ = plan.action7(3.0)
    q = {f"joint{i + 1}": float(a[i]) for i in range(6)}
    q["joint7"] = q["joint8"] = wp.finger
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return world_hulls(fk_link_poses_world(q, station.BASES[arm]), ["gripper", "tip_left", "tip_right"])


def _gap(hulls, box):
    """A lower bound on the clearance between the hulls and an axis-aligned box (separating axes: the box's and each
    hull's face normals). Negative means they may overlap."""
    c, half = np.asarray(box.pos, float), np.asarray(box.size, float) / 2
    corners = np.array([[c[0] + sx * half[0], c[1] + sy * half[1], c[2] + sz * half[2]]
                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    worst = np.inf
    for _, h in hulls:
        normals = [np.eye(3)[k] for k in range(3)] + [n for n in h.planes[:, :3] if np.all(np.isfinite(n))]
        best = -np.inf
        for n in normals:
            a, b = h.vertices @ n, corners @ n
            best = max(best, b.min() - a.max(), a.min() - b.max())
        worst = min(worst, best)
    return worst


SLIDE_BOXES = [o for o in M.OBJECTS if isinstance(o, Box) and not isinstance(o, mechanism.Mechanism)]


@pytest.mark.parametrize("arm,label", [
    ("right", "push the retainer forward"), ("right", "hold at the stop"), ("right", "tip into the channel"),
    ("left", "slide in"), ("left", "let go"), ("left", "push home"), ("left", "hold"), ("left", "down to the end"),
])
def test_fingers_clear_the_slide(arm, label):
    wp = next(w for w in M.demo()[arm] if w.label == label)
    if label in ("push home", "hold"):
        # the end push aims past home; the member's end, at home, stops the tips first: check that pose
        wp = wp.__class__(wp.duration, (wp.pos[0], M.Y0 + M.U_HOME + M.RAIL_L + 0.0077, wp.pos[2]), wp.finger,
                          wp.label, wp.yaw)
    hulls = _fingers(arm, wp)
    for bx in SLIDE_BOXES:
        g = _gap(hulls, bx)
        assert g > 0.0025, (arm, label, bx.name, g)


def test_end_push_is_stopped_by_the_end_and_covers_it():
    """The end push's target lies past the end at home (so the end, not the target, stops the tips), its start lies
    behind the end of a member stopped short, and the tips span the end face whether it rides or lies on the table."""
    left = M.demo()["left"]
    push = next(w for w in left if w.label == "push home")
    start = next(w for w in left if w.label == "down to the end")
    end_home = M.Y0 + M.U_HOME + M.RAIL_L
    assert push.pos[1] - 0.0077 < end_home - 0.03
    assert start.pos[1] - 0.0077 > M.Y0 + M.PUSH_FROM_NOSE + M.RAIL_L
    tip_lo = push.pos[2] - M.TIP - M.TOP
    assert 0.002 <= tip_lo <= 0.0 + M.RAIL_T - 0.004            # covers an end lying on the table (0 to 8 mm)
    assert tip_lo + M.TIP >= M.RAIL_Z0 + M.RAIL_T              # and one riding level (7 to 15 mm)


def test_pusher_tip_reaches_the_back_bar():
    wp = next(w for w in M.demo()["right"] if w.label == "tip into the channel")
    hulls = _fingers("right", wp)
    v = np.vstack([h.vertices for n, h in hulls if n.startswith("tip")])
    assert v[:, 2].min() - M.TOP < M.CAGE_Z0 - 0.002            # tip end under the back bar's underside
    assert v[:, 2].min() - M.TOP > M.WEB_T + 0.005               # and well above the web
    u = v[:, 1] - M.Y0
    near = v[:, 2] - M.TOP < M.CAGE_Z1
    assert u[near].max() < M.CAGE_BACK0 - 0.001                 # behind the bar before the push


def test_grasp_clears_the_table():
    wp = next(w for w in M.demo()["left"] if w.label == "grip")
    v = np.vstack([h.vertices for _, h in _fingers("left", wp)])
    assert v[:, 2].min() > M.TOP + 0.0002
    assert wp.finger < FINGER_CLOSED


def test_member_carried_over_or_beside_the_slide_does_not_meet_the_retainer():
    """Carried over the lips, or lying on the table beside the slide, with its tapered end past the retainer's front
    rollers in u: not inside the channel, so not a contact. A correct run afterwards still succeeds."""
    over = rail(0.10, dz=M.LIP_Z1 + 0.004 - M.RAIL_ZC)
    beside = rail(0.10, v=0.05, dz=M.RAIL_T / 2 - M.RAIL_ZC)
    assert play([objs(over, 0.0), objs(beside, 0.0)]) == (0.0, False)
    assert not M._MEM["bad"]
    assert play(correct_run()) == (0.9, True)
