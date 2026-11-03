"""On the station table: a loose aluminium workpiece block (60 x 40 x 40 mm) lies flat in front of the left arm, turned
20 degrees counterclockwise (seen from above). A fixed steel L-shaped stop stands in front of the right arm: a far stop
(a 120 mm rail across the table) and a side stop (a 100 mm rail running toward the arms along the outer edge), both 20
mm wide and 12 mm tall, meeting at a square inner corner. The block is registered when it lies flat, square to the stop,
touching both stop faces. The left arm can't reach the stop and the right arm can't reach the block's start, so the
block has to be set down between the arms for the right arm to take.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

BLOCK_SIZE = (0.060, 0.040, 0.040)
START_YAW = math.radians(20.0)
BLOCK = Box("block", pos=(0.22, 0.305, TOP + BLOCK_SIZE[2] / 2 + 0.001), size=BLOCK_SIZE, mass=0.259,
            yaw=START_YAW, color=(0.80, 0.81, 0.83))

# The stop's two inner faces: the far stop faces the arms at x = STOP_X; the side stop faces the midline at y = STOP_Y.
STOP_X = 0.37
STOP_Y = -0.345
RAIL = 0.020  # width
RAIL_H = 0.012  # WHY: low, so the gripper's pads clear it while they hold the block against it
STEEL = (0.35, 0.36, 0.38)
FAR_STOP = Box("far_stop", pos=(STOP_X + RAIL / 2, -0.305, TOP + RAIL_H / 2), size=(RAIL, 0.12, RAIL_H), color=STEEL)
SIDE_STOP = Box("side_stop", pos=(0.32, STOP_Y - RAIL / 2, TOP + RAIL_H / 2), size=(0.10, RAIL, RAIL_H), color=STEEL)
OBJECTS = [BLOCK, FAR_STOP, SIDE_STOP]

FULL = (1.0, 2.0)  # largest gap at each stop (mm), squareness (deg)
HALF = (3.0, 5.0)
QUARTER_GAP_MM = 10.0
MAX_TILT_DEG = 2.0
# WHY: sim contacts let a pressed block sit up to half a millimetre into a stop, so that still counts as touching. A
# block still being pushed hard can sink further; the partial tiers allow 5 mm so it isn't scored 0. Past a rail
# (20 mm wide) the gap is below -20 mm.
TOUCH_MM = 0.5
PRESSED_IN_MM = 5.0
OFF_TABLE_M = 0.02


def _rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(block) -> float:
    """How far the block is from lying flat on one of its four 60 x 40 mm faces (deg): its 60 mm axis off horizontal,
    or its roll about that axis off a multiple of 90 degrees, whichever is larger."""
    r = _rotation(block.quat_xyzw)
    long_off = math.degrees(math.asin(min(1.0, abs(r[2, 0]))))
    roll_off = math.degrees(math.acos(min(1.0, max(abs(r[2, 1]), abs(r[2, 2])))))
    return max(long_off, roll_off)


def _long_axis_yaw(block) -> float:
    """Direction of the block's 60 mm axis in the table plane (rad)."""
    r = _rotation(block.quat_xyzw)
    return math.atan2(r[1, 0], r[0, 0])


def _corners_xy(block) -> np.ndarray:
    """The four corners of the block's footprint, world x and y (m), from its centre and the direction of its 60 mm
    axis; the same on whichever long face it lies."""
    yaw = _long_axis_yaw(block)
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = BLOCK_SIZE[0] / 2, BLOCK_SIZE[1] / 2
    local = np.array([(hx, hy), (hx, -hy), (-hx, hy), (-hx, -hy)])
    rot = np.array([[c, -s], [s, c]])
    return local @ rot.T + np.asarray(block.pos[:2], dtype=float)


def _gaps_mm(block):
    """The largest gap along each stop, as a feeler gauge finds it (mm): the far stop against the edge facing it, at
    both of that edge's corners; the same for the side stop."""
    xy = _corners_xy(block)
    facing_far = np.sort(xy[:, 0])[-2:]  # the two corners nearest the far stop
    facing_side = np.sort(xy[:, 1])[:2]  # the two corners nearest the side stop
    return 1000.0 * (STOP_X - facing_far.min()), 1000.0 * (facing_side.max() - STOP_Y)


def _square_deg(block) -> float:
    """How far the block's edges are from parallel to the stop (deg); either side may face the far stop."""
    d = math.degrees(_long_axis_yaw(block)) % 90.0
    return min(d, 90.0 - d)


def _registered(block, gap_mm, square_deg, overlap_mm=TOUCH_MM) -> bool:
    far, side = _gaps_mm(block)
    return (
        -overlap_mm <= far <= gap_mm
        and -overlap_mm <= side <= gap_mm
        and _square_deg(block) <= square_deg
        and _tilt_deg(block) <= MAX_TILT_DEG
        # on the table: not on a rail, not held up (low is exact for a box)
        and TOP - 0.001 <= float(block.low[2]) <= TOP + 0.001
    )


def success(objects):
    """Flat on the table, every gap at both stops at most 1 mm, square within 2 degrees, still; holds 1 s."""
    block = objects["block"]
    return _registered(block, *FULL) and block.resting()


def distance(objects):
    """The larger of the two largest gaps (m)."""
    return max(0.0, *_gaps_mm(objects["block"])) / 1000.0


def score(objects):
    """Partial credit (success scores 1.0): 0.5 if every gap is at most 3 mm and it is square within 5 degrees; 0.25 if
    every gap is at most 10 mm. Off the table: 0."""
    block = objects["block"]
    if float(block.low[2]) < TOP - OFF_TABLE_M:
        return 0.0
    if _registered(block, *HALF, PRESSED_IN_MM):
        return 0.5
    return 0.25 if _registered(block, QUARTER_GAP_MM, 45.0, PRESSED_IN_MM) else 0.0


# TCP height that grips the block across its 40 mm sides: the pad tips, 13 mm below the TCP, stay 5 mm above the rails.
GRIP_Z = TOP + RAIL_H + 0.005 + PAD_TIP_PAST_TCP
TRAVEL_Z = TOP + 0.14
RELAY_XY = (0.26, 0.0)
RIGHT_WAIT = (0.26, -0.15, TRAVEL_Z)
HOME = (STOP_X - BLOCK_SIZE[0] / 2, STOP_Y + BLOCK_SIZE[1] / 2)  # the block's centre when registered
SET_OFF = 0.010  # set it down this far out from each stop, then slide it in
PRESS = 0.002  # commanded past contact, so the slide ends pressed against both stops
# WHY: low and out by the corner, the right arm lands the block short of where it aims. Traced on the official stack
# (2026-11-03): aiming at the set-down point, the block came down 16.7 mm short in x and 15.0 mm past it toward the side
# stop, onto the rail. Aim the set-down and the slide past by that much.
RIGHT_AIM = (0.0167, 0.0150)
# WHY: a block let go in the corner stands back up 1.5-3 mm off the side stop: slid in gripped, its bottom drags on the
# table (friction 1.2) while the grip tips its top into the stop (traces, 2026-11-03). So after letting go the right arm
# pushes the block's open side, 3 mm above the table, until it meets the side stop (see demo()).
PUSH_TIP_Z = 0.003  # pad tips this far above the table while pushing
PUSH_START = 0.015  # the pad comes down this far out from the block's open side when registered
PAD_PRESS = 0.005  # past contact, in case the fingers open less than OPEN_PAD_Y
OPEN_PAD_Y = 0.0475  # an open finger's inner pad face from the TCP (about 95 mm open, from the grip readings)


def demo():
    """The left arm grips the block across its 40 mm sides (fingers turned 20 degrees to match it), sets it down square
    on the midline and moves back over its own side. The right arm waits clear of the midline until then, takes the
    block, sets it down 10 mm out from each stop, slides it diagonally into the corner still gripped and lets go.
    Then it comes down with its fingers open, one beyond the side stop and one beside the block's open side, pushes
    the block against the side stop with that finger's inner pad, and backs off."""
    start = (BLOCK.pos[0], BLOCK.pos[1], GRIP_Z)
    left = pick_place_waypoints(start, (*RELAY_XY, GRIP_Z + 0.002), travel_z=TRAVEL_Z, yaw=START_YAW, place_yaw=0.0)
    left.append(Waypoint(1.5, (BLOCK.pos[0], BLOCK.pos[1], TRAVEL_Z), FINGER_OPEN, "clear the midline"))
    handoff_s = sum(w.duration for w in left)

    set_xy = (HOME[0] - SET_OFF + RIGHT_AIM[0], HOME[1] + SET_OFF + RIGHT_AIM[1])
    seat_xy = (HOME[0] + PRESS + RIGHT_AIM[0], HOME[1] - PRESS + RIGHT_AIM[1])
    right = [Waypoint(handoff_s, RIGHT_WAIT, FINGER_OPEN, "wait for the block")]
    low = GRIP_Z + 0.001  # the block 1 mm above the table
    right += pick_place_waypoints((*RELAY_XY, GRIP_Z), (*set_xy, low), travel_z=TRAVEL_Z)[:6]  # up to the set-down
    right += [
        Waypoint(2.5, (*seat_xy, low), FINGER_CLOSED, "slide into the corner"),
        Waypoint(0.8, (*seat_xy, low), FINGER_CLOSED, "hold"),
        Waypoint(0.6, (*seat_xy, low), FINGER_OPEN, "release"),
        Waypoint(1.0, (*seat_xy, TRAVEL_Z), FINGER_OPEN, "retreat"),
    ]
    # WHY: pushed with closed fingers the block tips instead of sliding, and the TCP rises as it pushes, so they seem
    # to meet the block high up (traces, 2026-11-03). The open +y finger's inner pad is flat and upright: if the block
    # starts to tip, its face leaves the pad except at the bottom, so it slides. The open -y finger comes down outside
    # the side stop.
    pad_y = OPEN_PAD_Y
    push_x = HOME[0]
    face_y = HOME[1] + BLOCK_SIZE[1] / 2 - PAD_PRESS  # the block's open side when registered, pushed past
    push_z = TOP + PUSH_TIP_Z + PAD_TIP_PAST_TCP
    start_y = HOME[1] + BLOCK_SIZE[1] / 2 + PUSH_START - pad_y
    right += [
        Waypoint(1.0, (push_x, start_y, TRAVEL_Z), FINGER_OPEN, "over the block, fingers open"),
        Waypoint(1.2, (push_x, start_y, push_z), FINGER_OPEN, "down either side of the block and the side stop"),
        Waypoint(2.5, (push_x, face_y - pad_y, push_z), FINGER_OPEN, "push it against the side stop with the pad"),
        # WHY: no hold. The pad leaves at once, so the 1 s that success() needs counts only the block lying there alone.
        Waypoint(0.6, (push_x, start_y, push_z), FINGER_OPEN, "back off"),
        Waypoint(1.0, (push_x, start_y, TRAVEL_Z), FINGER_OPEN, "up"),
    ]
    return {"left": left, "right": right}
