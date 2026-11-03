"""On the station table: three bare steel pins, 80 mm tall and 20, 23 and 26 mm across, stand upright in a row in front
of the left arm. A fixed aluminium fixture block (62 x 62 mm, 25 mm tall) stands in front of the right arm, with a
square hole 22 mm across and 25 mm deep at its centre; the hole's floor is the table. Only a pin narrower than the hole
goes in. The pins are loose; the fixture is fixed. The left arm can't reach the fixture and the right arm can't reach
the pins, so a pin has to be set down between the arms for the right arm to take.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
STEEL_KG_M3 = 7850.0

PIN_H = 0.08
PIN_ROW_Y = 0.305  # in line with the left arm's base
# WHY: the pin that fits sits in the middle of the row, so "take the nearest" or "take the farthest" never scores.
PIN_DIAMETER_M = {"pin_a": 0.026, "pin_b": 0.020, "pin_c": 0.023}
PIN_X = {"pin_a": 0.18, "pin_b": 0.26, "pin_c": 0.34}
FITS = "pin_b"
PINS = [
    Cylinder(name, pos=(PIN_X[name], PIN_ROW_Y, TOP + PIN_H / 2 + 0.001), radius=d / 2, height=PIN_H,
             mass=round(STEEL_KG_M3 * math.pi * (d / 2) ** 2 * PIN_H, 3), color=(0.62, 0.63, 0.66))
    for name, d in PIN_DIAMETER_M.items()
]

HOLE = 0.022  # square hole, side (m)
WALL = 0.020
DEPTH = 0.025
HOLE_XY = (0.30, -0.305)  # in line with the right arm's base
_hx, _hy = HOLE_XY
_off = HOLE / 2 + WALL / 2
ALU = (0.78, 0.79, 0.80)
_z = TOP + DEPTH / 2
FIXTURE = [  # four walls around the hole; no mass, so fixed
    Box("wall_far", pos=(_hx + _off, _hy, _z), size=(WALL, HOLE + 2 * WALL, DEPTH), color=ALU),
    Box("wall_near", pos=(_hx - _off, _hy, _z), size=(WALL, HOLE + 2 * WALL, DEPTH), color=ALU),
    Box("wall_left", pos=(_hx, _hy + _off, _z), size=(HOLE, WALL, DEPTH), color=ALU),
    Box("wall_right", pos=(_hx, _hy - _off, _z), size=(HOLE, WALL, DEPTH), color=ALU),
]
OBJECTS = [*PINS, *FIXTURE]

SEATED_MM = 3.0  # the pin's bottom within this of the hole's floor
ENTERED_MM = 2.0  # the pin's bottom at least this far below the hole's top edge
NEAR_M = 0.05
# WHY: a pin with its bottom on the floor of the 22 mm square hole can lean about 4.7 degrees toward a wall and about
# 6.8 toward a corner, so 7 degrees accepts every seated pin.
MAX_TILT_SEATED_DEG = 7.0
MAX_TILT_UPRIGHT_DEG = 10.0
MAX_TILT_ENTERED_DEG = 30.0  # a pin jammed part way in leans up to acos(20/22) = 25 degrees; a lying pin has not entered
STANDING_MM = 3.0  # standing on a surface: the pin's bottom within this of it
OFF_TABLE_M = 0.02  # a pin whose bottom is this far below the table top has fallen off


# A pin's bottom comes from its centre and axis, not from ``low``: Arena builds ``low``/``high`` from the corners of the
# pin's local box (a square post), so a tilted pin that also spins on its own axis reads lower than it is.


def _axis_z(quat_xyzw) -> float:
    """The vertical component of the pin's axis (its local z) in the world frame."""
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return 1.0 - 2.0 * (x * x + y * y)


def _bottom(pin, radius) -> float:
    """World z of the pin's lowest point: its centre less half the height times cos(tilt) and the radius times
    sin(tilt). Exact for a cylinder, and spin-free."""
    c = min(1.0, abs(_axis_z(pin.quat_xyzw)))
    return float(pin.pos[2]) - (PIN_H / 2 * c + radius * math.sqrt(1.0 - c * c))


def _pin_bottom(objects, name) -> float:
    return _bottom(objects[name], PIN_DIAMETER_M[name] / 2)


def _tilt_deg(quat_xyzw) -> float:
    """Angle between the pin's axis and the vertical, either end up (a plain pin is the same both ways)."""
    return math.degrees(math.acos(min(1.0, abs(_axis_z(quat_xyzw)))))


def _bottom_end_xy(pin):
    """x, y of the centre of the pin's lower end."""
    x, y, z, w = (float(v) for v in pin.quat_xyzw)
    ax, ay, az = 2.0 * (x * z + w * y), 2.0 * (y * z - w * x), _axis_z(pin.quat_xyzw)
    sign = -1.0 if az >= 0 else 1.0  # the end that is lower
    return float(pin.pos[0]) + sign * ax * PIN_H / 2, float(pin.pos[1]) + sign * ay * PIN_H / 2


def _from_hole(pin):
    return float(pin.pos[0]) - _hx, float(pin.pos[1]) - _hy


def _in_hole_xy(pin) -> bool:
    """The centre of the pin's lower end is over the hole (a leaning pin's centre can be outside it)."""
    bx, by = _bottom_end_xy(pin)
    return abs(bx - _hx) <= HOLE / 2 and abs(by - _hy) <= HOLE / 2


def _seated(pin) -> bool:
    return (
        _in_hole_xy(pin)
        and _bottom(pin, PIN_DIAMETER_M[FITS] / 2) <= TOP + SEATED_MM / 1000
        and _tilt_deg(pin.quat_xyzw) <= MAX_TILT_SEATED_DEG
        and pin.resting()
    )


def _on_block_top(pin) -> bool:
    """The lower end's centre is over the block's top face: inside its 62 mm square but not over the open hole, where a
    20 mm pin has nothing under it."""
    bx, by = _bottom_end_xy(pin)
    out = max(abs(bx - _hx), abs(by - _hy))
    return HOLE / 2 - PIN_DIAMETER_M[FITS] / 2 < out <= HOLE / 2 + WALL


def _any_off_table(objects) -> bool:
    return any(_pin_bottom(objects, name) < TOP - OFF_TABLE_M for name in PIN_DIAMETER_M)


def success(objects):
    """The 20 mm pin stands in the hole, its bottom within 3 mm of the floor, tilted at most 7 degrees (either end up),
    and no pin is off the table; holds 1 s."""
    return _seated(objects[FITS]) and not _any_off_table(objects)


def distance(objects):
    """From the hole's floor centre to the fitting pin: its lower end's centre across, its lowest point up (m)."""
    bx, by = _bottom_end_xy(objects[FITS])
    dx, dy = bx - _hx, by - _hy
    return float(math.sqrt(dx * dx + dy * dy + (_pin_bottom(objects, FITS) - TOP) ** 2))


def score(objects):
    """Partial credit (success scores 1.0): 0.5 if the 20 mm pin has entered the hole but is not fully down; 0.25 if
    it stands upright, on the table or on the block, with its centre within 5 cm of the hole's centre. Any pin off the
    table: 0."""
    if _any_off_table(objects):
        return 0.0
    pin = objects[FITS]
    entered = _in_hole_xy(pin) and _tilt_deg(pin.quat_xyzw) <= MAX_TILT_ENTERED_DEG
    if entered and _pin_bottom(objects, FITS) <= TOP + DEPTH - ENTERED_MM / 1000:
        return 0.5
    dx, dy = _from_hole(pin)
    bottom = _pin_bottom(objects, FITS)
    on_table = abs(bottom - TOP) <= STANDING_MM / 1000
    on_block = abs(bottom - (TOP + DEPTH)) <= STANDING_MM / 1000 and _on_block_top(pin)
    standing = on_table or on_block
    upright = _tilt_deg(pin.quat_xyzw) <= MAX_TILT_UPRIGHT_DEG
    return 0.25 if standing and upright and math.hypot(dx, dy) <= NEAR_M else 0.0


GRIP_BELOW_TOP = 0.025  # pad tips sit 13 mm below the TCP, so they stay 17 mm above the fixture when the pin is in
GRIP_Z = TOP + PIN_H - GRIP_BELOW_TOP  # TCP height that grips a pin standing on the table (or on the hole's floor)
TRAVEL_Z = TOP + 0.14
RELAY_XY = (0.28, 0.0)  # on the midline, inside both arms' reach
RIGHT_WAIT = (0.28, -0.15, TRAVEL_Z)
# WHY: the pin hangs off-centre in the right gripper over the hole: the relay leaves it 3.6 mm short in x, which the
# fingers (closing along y) don't correct, and the arm lands a few mm short. Traced on the official stack (2026-11-03):
# over the hole the pin's centre was 6.0 mm short in x and 1.0 mm off in y, against 1 mm of clearance. Aim past it.
HOLE_AIM = (0.006, 0.001)


def demo():
    """The left arm takes the 20 mm pin and stands it on the midline, then moves back over its own side. The right arm
    waits clear of the midline until then, takes the pin, carries it over the hole, lowers it to 5 mm above the hole's
    top edge, pushes it down slowly to the floor and lets go."""
    pick = (PIN_X[FITS], PIN_ROW_Y, GRIP_Z)
    relay = (*RELAY_XY, GRIP_Z + 0.002)  # let go 2 mm above the table
    left = pick_place_waypoints(pick, relay, travel_z=TRAVEL_Z)
    left.append(Waypoint(1.5, (PIN_X[FITS], PIN_ROW_Y, TRAVEL_Z), FINGER_OPEN, "clear the midline"))
    handoff_s = sum(w.duration for w in left)

    above = GRIP_Z + DEPTH + 0.005  # pin bottom 5 mm above the hole's top edge
    right = [Waypoint(handoff_s, RIGHT_WAIT, FINGER_OPEN, "wait for the pin")]
    aim = (HOLE_XY[0] + HOLE_AIM[0], HOLE_XY[1] + HOLE_AIM[1])
    right += pick_place_waypoints((*RELAY_XY, GRIP_Z), (*aim, above), travel_z=TRAVEL_Z)[:5]  # up to the carry
    right += [
        Waypoint(1.5, (*aim, above), FINGER_CLOSED, "over the hole"),
        Waypoint(3.0, (*aim, GRIP_Z + 0.001), FINGER_CLOSED, "push in"),
        Waypoint(0.8, (*aim, GRIP_Z + 0.001), FINGER_OPEN, "release"),
        Waypoint(1.2, (*aim, TRAVEL_Z), FINGER_OPEN, "retreat"),
    ]
    return {"left": left, "right": right}
