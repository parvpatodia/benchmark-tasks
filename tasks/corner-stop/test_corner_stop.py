"""Checks of corner_stop.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_corner_stop.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("corner_stop.py"))
M = TASK.module
HALF = np.array(M.BLOCK_SIZE) / 2


def _rot(yaw, tilt):
    """The spin about the vertical times a tilt about the block's own x."""
    cz, sz, cx, sx = math.cos(yaw), math.sin(yaw), math.cos(tilt), math.sin(tilt)
    return np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]]) @ np.array(
        [[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])


def block_state(x, y, yaw_deg=0.0, bottom_z=None, tilt_deg=0.0, moving=False):
    """The block with its centre at x, y, turned ``yaw_deg``, tilted ``tilt_deg``, its lowest corner at ``bottom_z``.
    A box's bounding box from its turned corners is exact."""
    bottom_z = M.TOP if bottom_z is None else bottom_z
    psi, tau = math.radians(yaw_deg), math.radians(tilt_deg)
    rot = _rot(psi, tau)
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * HALF @ rot.T
    centre = np.array([x, y, bottom_z - corners[:, 2].min()])
    world = corners + centre
    sz, cz, sx, cx = math.sin(psi / 2), math.cos(psi / 2), math.sin(tau / 2), math.cos(tau / 2)
    return ObjectState(
        pos=centre,
        quat_xyzw=np.array([cz * sx, sz * sx, sz * cx, cz * cx]),  # the spin about z times the tilt about x
        lin_vel=np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3),
        ang_vel=np.zeros(3),
        low=world.min(axis=0),
        high=world.max(axis=0),
    )


def pressed(yaw_deg=0.0, gap_mm=0.0, **kw):
    """The block turned ``yaw_deg`` with its nearest corner ``gap_mm`` from each stop face."""
    xy = (np.array([[i, j] for i in (-1, 1) for j in (-1, 1)]) * HALF[:2]) @ _rot(math.radians(yaw_deg), 0)[:2, :2].T
    g = gap_mm / 1000
    return block_state(M.STOP_X - g - xy[:, 0].max(), M.STOP_Y + g - xy[:, 1].min(), yaw_deg, **kw)


def scene(block):
    return {"block": block}


def test_the_quaternion_matches_the_rotation():
    b = block_state(0.3, -0.3, yaw_deg=25, tilt_deg=3)
    assert np.allclose(M._rotation(b.quat_xyzw), _rot(math.radians(25), math.radians(3)))
    assert math.degrees(M._long_axis_yaw(b)) == pytest.approx(25, abs=1e-9)
    assert M._tilt_deg(b) == pytest.approx(3, abs=1e-9)


@pytest.mark.parametrize("roll_deg", [90, 180, -90])
def test_rolled_onto_another_long_face_still_counts(roll_deg):
    s = scene(pressed(tilt_deg=roll_deg))  # same 60 x 40 footprint, same height
    assert M.success(s)


def test_standing_on_its_end_scores_zero():
    r = _rot(0.0, 0.0) @ np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])  # 60 mm axis vertical
    q = np.array([0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)])
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * HALF @ r.T
    centre = np.array([M.HOME[0], M.HOME[1], M.TOP + 0.03])
    b = ObjectState(pos=centre, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                    low=(corners + centre).min(axis=0), high=(corners + centre).max(axis=0))
    assert np.allclose(M._rotation(q), r) and M.score(scene(b)) == 0.0


def test_the_start_scores_zero():
    start = block_state(M.BLOCK.pos[0], M.BLOCK.pos[1], yaw_deg=20)
    assert not M.success(scene(start)) and M.score(scene(start)) == 0.0


@pytest.mark.parametrize("yaw_deg", [0, 90, 180, -90])
def test_square_in_the_corner_is_success_either_way_round(yaw_deg):
    s = scene(pressed(yaw_deg))
    assert M.success(s) and M.score(s) == 0.5  # 1.0 comes from success() holding for 1 s, not from score()


@pytest.mark.parametrize(("gap_mm", "expected"), [(0.8, 1.0), (2.5, 0.5), (8.0, 0.25), (12.0, 0.0)])
def test_the_largest_gap_sets_the_score(gap_mm, expected):
    s = scene(pressed(gap_mm=gap_mm))
    assert (1.0 if M.success(s) else M.score(s)) == expected


def test_slightly_turned_against_the_corner_gets_half():
    s = scene(pressed(1.5))  # touching at one corner; the 60 mm edge opens 60 sin(1.5 deg) = 1.6 mm at the other
    assert not M.success(s) and M.score(s) == 0.5


def test_turned_4_degrees_gets_a_quarter():
    assert M.score(scene(pressed(4.0))) == 0.25  # 60 sin(4 deg) = 4.2 mm at the far end of the side stop


def test_the_gaps_are_measured_at_both_corners():
    far, side = M._gaps_mm(pressed(1.5))
    assert far == pytest.approx(40 * math.sin(math.radians(1.5)), abs=1e-6)
    assert side == pytest.approx(60 * math.sin(math.radians(1.5)), abs=1e-6)


@pytest.mark.parametrize(("gap_mm", "ok"), [(-0.4, True), (-0.6, False)])
def test_a_little_contact_overlap_is_allowed(gap_mm, ok):
    assert M.success(scene(pressed(gap_mm=gap_mm))) == ok


def test_pressed_harder_into_the_stops_still_gets_half():
    s = scene(pressed(gap_mm=-2.0))  # still being pushed: not registered, but not scored 0
    assert not M.success(s) and M.score(s) == 0.5


def test_on_top_of_a_rail_scores_zero():
    assert M.score(scene(pressed(bottom_z=M.TOP + M.RAIL_H))) == 0.0


def test_held_just_above_the_table_is_not_registered():
    assert not M.success(scene(pressed(bottom_z=M.TOP + 0.002)))


def test_tilted_is_not_registered():
    assert not M.success(scene(pressed(tilt_deg=3)))


def test_a_moving_block_is_not_yet_registered():
    assert not M.success(scene(pressed(moving=True)))


def test_off_the_table_scores_zero():
    assert M.score(scene(block_state(0.3, -0.7, bottom_z=M.TOP - 0.3))) == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
