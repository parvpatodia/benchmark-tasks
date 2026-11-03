"""Checks of pin_fit.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_pin_fit.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("pin_fit.py"))
M = TASK.module
HX, HY = M.HOLE_XY


def _rot_y(t):
    return np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])


def _rot_z(s):
    return np.array([[math.cos(s), -math.sin(s), 0.0], [math.sin(s), math.cos(s), 0.0], [0.0, 0.0, 1.0]])


def pin_state(name, x, y, bottom_z=None, tilt_deg=0.0, spin_deg=0.0, moving=False):
    """A pin with its centre at x, y, spun ``spin_deg`` on its own axis, then tilted ``tilt_deg`` about world y, with
    its lowest point at ``bottom_z``. Its bounding box is built the way Arena 0.3.1 builds it (``get_aabb_w``): the 8
    corners of the pin's local box, a square post, turned with the pin."""
    r = M.PIN_DIAMETER_M[name] / 2
    bottom_z = M.TOP if bottom_z is None else bottom_z
    t, s = math.radians(tilt_deg), math.radians(spin_deg)
    centre = np.array([x, y, bottom_z + M.PIN_H / 2 * abs(math.cos(t)) + r * abs(math.sin(t))])
    rot = _rot_y(t) @ _rot_z(s)
    corners = np.array([[i * r, j * r, k * M.PIN_H / 2] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    world = corners @ rot.T + centre
    sy, cy, sz, cz = math.sin(t / 2), math.cos(t / 2), math.sin(s / 2), math.cos(s / 2)
    return ObjectState(
        pos=centre,
        quat_xyzw=np.array([sy * sz, sy * cz, cy * sz, cy * cz]),  # the tilt about y times the spin about z
        lin_vel=np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3),
        ang_vel=np.zeros(3),
        low=world.min(axis=0),
        high=world.max(axis=0),
    )


def scene(**pins):
    """Every pin in its row, except those given as name=ObjectState."""
    return {name: pins.get(name, pin_state(name, M.PIN_X[name], M.PIN_ROW_Y)) for name in M.PIN_DIAMETER_M}


def fit(**kw):
    """The fitting pin, centred over the hole."""
    return pin_state(M.FITS, HX, HY, **kw)


def test_the_start_scores_zero():
    assert not M.success(scene()) and M.score(scene()) == 0.0


def test_the_fitting_pin_on_the_floor_is_success():
    s = scene(pin_b=fit())
    assert M.success(s) and M.score(s) == 0.5  # 1.0 comes from success() holding for 1 s, not from score()


@pytest.mark.parametrize(("above_mm", "seated"), [(2.5, True), (3.5, False)])
def test_seated_means_within_3_mm_of_the_floor(above_mm, seated):
    s = scene(pin_b=fit(bottom_z=M.TOP + above_mm / 1000))
    assert M.success(s) == seated
    assert M.score(s) == 0.5 or seated  # not fully down, but in the hole


@pytest.mark.parametrize(("above_mm", "expected"), [(10.0, 0.5), (22.0, 0.5), (24.0, 0.0)])
def test_entered_means_2_mm_below_the_block_top(above_mm, expected):
    """1 mm in is not yet entered, and a pin held there over the open hole is not standing on anything."""
    assert M.score(scene(pin_b=fit(bottom_z=M.TOP + above_mm / 1000))) == expected


@pytest.mark.parametrize(("dx_mm", "expected"), [(45.0, 0.25), (60.0, 0.0)])
def test_upright_on_the_table_near_the_hole_scores_a_quarter(dx_mm, expected):
    assert M.score(scene(pin_b=pin_state(M.FITS, HX + dx_mm / 1000, HY))) == expected


def test_upright_on_the_block_top_scores_a_quarter():
    on_wall = pin_state(M.FITS, HX + M.HOLE / 2 + M.WALL / 2, HY, bottom_z=M.TOP + M.DEPTH)
    assert M.score(scene(pin_b=on_wall)) == 0.25


@pytest.mark.parametrize(("dx_mm", "dy_mm"), [(45.0, 0.0), (0.0, 0.0), (0.5, 0.5)])
def test_held_at_block_top_height_off_the_block_scores_zero(dx_mm, dy_mm):
    """Beside the block (it ends 31 mm out) or over the open hole, nothing is under the pin."""
    held = pin_state(M.FITS, HX + dx_mm / 1000, HY + dy_mm / 1000, bottom_z=M.TOP + M.DEPTH)
    assert M.score(scene(pin_b=held)) == 0.0


def test_a_lying_pin_next_to_the_hole_scores_zero():
    assert M.score(scene(pin_b=pin_state(M.FITS, HX + 0.045, HY, tilt_deg=90))) == 0.0


def test_a_wrong_pin_scores_nothing():
    wrong = pin_state("pin_c", HX, HY, bottom_z=M.TOP + M.DEPTH)  # 23 mm: rests on the hole's rim
    assert M.score(scene(pin_c=wrong)) == 0.0


def test_a_pin_off_the_table_zeroes_the_trial():
    s = scene(pin_b=fit(), pin_a=pin_state("pin_a", 0.18, 0.60, bottom_z=M.TOP - 0.3, tilt_deg=90))
    assert not M.success(s) and M.score(s) == 0.0


def test_leaning_into_a_corner_of_the_hole_is_still_seated():
    assert M.success(scene(pin_b=fit(tilt_deg=6.5)))  # the square hole allows about 6.8 degrees toward a corner


def test_too_tilted_is_not_seated():
    s = scene(pin_b=fit(tilt_deg=8))
    assert not M.success(s) and M.score(s) == 0.5


def test_a_pin_seated_upside_down_counts():
    assert M.success(scene(pin_b=fit(tilt_deg=180)))


def test_a_pin_wedged_part_way_in_has_entered():
    t = math.radians(20)  # its centre is outside the hole's footprint, its lower end inside it
    wedged = pin_state(M.FITS, HX + M.PIN_H / 2 * math.sin(t), HY, bottom_z=M.TOP + 0.016, tilt_deg=20)
    assert M.score(scene(pin_b=wedged)) == 0.5


def test_a_pin_held_in_the_air_near_the_hole_scores_zero():
    assert M.score(scene(pin_b=pin_state(M.FITS, HX + 0.045, HY, bottom_z=M.TOP + 0.10))) == 0.0


def test_distance_is_measured_from_the_lower_end():
    assert M.distance(scene(pin_b=fit())) == pytest.approx(0.0, abs=1e-9)
    t = math.radians(20)
    wedged = pin_state(M.FITS, HX + M.PIN_H / 2 * math.sin(t), HY, bottom_z=M.TOP + 0.016, tilt_deg=20)
    assert M.distance(scene(pin_b=wedged)) < 0.02  # its centre is 14 mm off the hole's axis, its lower end is not


def test_a_moving_pin_is_not_yet_seated():
    assert not M.success(scene(pin_b=fit(moving=True)))


def test_a_tilted_spun_pin_is_scored_by_its_real_bottom():
    t, s = math.radians(4.0), math.radians(45.0)
    pin = fit(bottom_z=M.TOP + 0.0032, tilt_deg=4, spin_deg=45)  # 3.2 mm up: not seated
    r = M.PIN_DIAMETER_M[M.FITS] / 2
    box_error = (M.TOP + 0.0032) - float(pin.low[2])  # how much lower Arena's box puts the bottom
    assert box_error == pytest.approx(r * (math.cos(s) + math.sin(s) - 1) * math.sin(t), abs=1e-9)
    assert float(pin.low[2]) <= M.TOP + M.SEATED_MM / 1000  # the box alone says seated
    assert not M.success(scene(pin_b=pin))


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
