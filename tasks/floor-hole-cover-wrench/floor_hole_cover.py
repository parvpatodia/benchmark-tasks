"""A section of plywood floor deck stands on the bench. A square hole is cut through it, closed by a yellow plywood
cover whose two cleats sit down inside the hole. A combination wrench was dropped into the hole and lies on the floor
of the pocket under the cover. An open tool tote stands on the deck to the right. Get the wrench out of the hole and
put it in the tool tote.
"""
import dataclasses
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Shape, body, hull, on_place, on_reset, on_step, run_seed  # noqa: E402,F401
from yam_arena import ik, station  # noqa: E402,F401
from yam_arena.constants import READY, SHOULDER, WRIST  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 1200           # 60 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# Every number: [SOURCE] quoted in the README, [ARITH] derived, [EST] estimate, [CHOICE] design choice, [RUN] measured
# by a tool this session. World frame: x forward from the arm bases, y left, z up; heights in comments are above the
# bench top (TOP). Cover frame C: origin at the plate's centre on its underside, x along the handle, the cleats along
# y. Wrench frame W: origin under the middle of its length on its bottom face, x from the open end to the box end.

# -- the floor deck -------------------------------------------------------------------------------------------------------
PLY_T = 0.0183                   # 23/32 in plywood, the usual floor sheathing panel [ARITH: 23/32 in = 18.26 mm]
SLEEPER_T = 0.0381               # 2x4 sleepers laid flat, 1-1/2 in actual [ARITH: 1.5 in = 38.1 mm]
DECK_Z = PLY_T + SLEEPER_T       # deck top over the bench (the pocket's depth) [ARITH: 56.4 mm]
XO, YO = 0.270, 0.0              # the hole's centre (world x, y) [CHOICE: both arms reach it top-down]
OPEN = 0.250                     # the hole, square, side [CHOICE: a 222 mm wrench fits at every seeded pose]
DECK_X = (0.075, 0.465)          # deck footprint, world x [CHOICE: 30 mm past the seated cover front and back]
DECK_Y = (-0.500, 0.460)         # ... world y [CHOICE: room for the tote and a spot right of it (right) and the
#                                  set-aside cover (left); 190 mm inside the bench's edges]
PLY, PINE, YELLOW, STEEL_C, TOTE_C = (0.62, 0.50, 0.34), (0.80, 0.67, 0.46), (0.95, 0.78, 0.08), (0.76, 0.77, 0.79), \
    (0.15, 0.15, 0.17)
WOOD = (0.50, 0.45)              # plywood and pine friction, static / dynamic [EST]
STEEL = (0.40, 0.35)             # chrome-plated steel [EST]
MATS = {"wood": WOOD, "steel": STEEL}
RHO_PLY, RHO_PINE = 530.0, 450.0  # kg/m3 [EST: softwood plywood and pine]

# -- the cover ----------------------------------------------------------------------------------------------------------
PLATE = 0.330                    # square plate of PLY_T plywood [CHOICE: 40 mm onto the deck on every side]
CLEAT_W, CLEAT_H = 0.0381, 0.0191   # cleats: 1x2 strips laid flat [ARITH: 1-1/2 x 3/4 in actual]
CLEAT_L = 0.230                  # cleat length along C y [CHOICE: 10 mm of end play each end in the hole]
PLAY = 0.006                     # side play: each cleat's outer face this far inside the hole's wall when centred
#                                  [CHOICE]
CLEAT_X = OPEN / 2 - PLAY - CLEAT_W / 2   # cleat centres at C x = +-CLEAT_X [ARITH: 100 mm]
CHAMFER = 0.005                  # 45 deg chamfer planed on each cleat's bottom edges, a lead-in [CHOICE]
HANDLE_L, HANDLE_W, HANDLE_H = 0.200, 0.0381, 0.0381   # a 2x2 softwood offcut along C x [ARITH: 1-1/2 in actual]


def _box(lo, hi, color, material, mass=0.0):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Shape("box", tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, material=material,
                 mass=float(mass))


def _vol(lo, hi):
    return float(np.prod(np.asarray(hi, float) - np.asarray(lo, float)))


def cover_shapes():
    """The cover in C: plate, handle, two cleats (each a box and a chamfered foot). Masses from the volumes."""
    h = PLATE / 2
    lo, hi = (-h, -h, 0.0), (h, h, PLY_T)
    out = [_box(lo, hi, YELLOW, "wood", RHO_PLY * _vol(lo, hi))]
    lo, hi = (-HANDLE_L / 2, -HANDLE_W / 2, PLY_T), (HANDLE_L / 2, HANDLE_W / 2, PLY_T + HANDLE_H)
    out.append(_box(lo, hi, PINE, "wood", RHO_PINE * _vol(lo, hi)))
    for s in (-1.0, 1.0):
        x0, x1 = s * CLEAT_X - CLEAT_W / 2, s * CLEAT_X + CLEAT_W / 2
        y0, y1 = -CLEAT_L / 2, CLEAT_L / 2
        z_b, z_c = -CLEAT_H, -CLEAT_H + CHAMFER
        lo, hi = (x0, y0, z_c), (x1, y1, 0.0)
        out.append(_box(lo, hi, PINE, "wood", RHO_PINE * _vol(lo, hi)))
        c = CHAMFER
        ring_b = [(x0 + c, y0 + c, z_b), (x1 - c, y0 + c, z_b), (x1 - c, y1 - c, z_b), (x0 + c, y1 - c, z_b)]
        ring_t = [(x0, y0, z_c), (x1, y0, z_c), (x1, y1, z_c), (x0, y1, z_c)]
        foot_v = (CLEAT_W - c) * (CLEAT_L - c) * c   # [ARITH: frustum, near enough for the mass]
        out.append(hull(ring_b + ring_t, PINE, "wood", mass=RHO_PINE * foot_v))
    return out


COVER_SHAPES = cover_shapes()
COVER_KG = sum(s.mass for s in COVER_SHAPES)
COVER_COM = np.sum([s.mass * np.asarray(s.center, float) for s in COVER_SHAPES], axis=0) / COVER_KG
CLEAT_FEET = [s for s in COVER_SHAPES if s.kind == "hull"]

# -- the wrench (GEARWRENCH 81671, 14 mm 12 point long pattern combination wrench) ----------------------------------------
WRENCH_L = 0.222                 # [SOURCE: "Overall Length : 8.74 in" = 222.0 mm]
WRENCH_KG = 0.1451               # [SOURCE: "Weight (Catalog) : 0.32 lb" = 0.1451 kg]
OPEN_W, OPEN_T = 0.0289, 0.0070  # open end [SOURCE: "End A Width : 1.138 in", "End A Thickness : 0.276 in"]
BOX_W, BOX_T = 0.0215, 0.0093    # box end [SOURCE: "End B Width : 0.846 in", "End B Thickness : 0.366 in"]
OPEN_L, BOX_L = 0.034, 0.024     # each head's length along the wrench [EST]
BEAM_W, BEAM_T = 0.013, 0.0060   # the beam between the heads [EST: not on the maker's sheet]


def wrench_shapes():
    """The wrench in W, flat on its back (the box end's 15 deg offset is not modelled). Mass shared by volume."""
    a = -WRENCH_L / 2
    parts = [((a, -OPEN_W / 2, 0.0), (a + OPEN_L, OPEN_W / 2, OPEN_T)),
             ((a + OPEN_L, -BEAM_W / 2, 0.0), (-a - BOX_L, BEAM_W / 2, BEAM_T)),
             ((-a - BOX_L, -BOX_W / 2, 0.0), (-a, BOX_W / 2, BOX_T))]
    total = sum(_vol(lo, hi) for lo, hi in parts)
    return [_box(lo, hi, STEEL_C, "steel", WRENCH_KG * _vol(lo, hi) / total) for lo, hi in parts]


WRENCH_SHAPES = wrench_shapes()
WRENCH_COM = np.sum([s.mass * np.asarray(s.center, float) for s in WRENCH_SHAPES], axis=0) / WRENCH_KG

# -- the tote -----------------------------------------------------------------------------------------------------------
XT, YT = 0.215, -0.310           # tote centre on the deck [CHOICE: in the right arm's top-down reach]
TOTE_IN = (0.270, 0.160)         # inside length (x) and width (y) [EST: a small open parts tote]
TOTE_H = 0.065                   # wall height over the deck [EST]
TOTE_WALL, TOTE_FLOOR = 0.004, 0.004   # [EST: moulded polypropylene]
TOTE_FLOOR_Z = DECK_Z + TOTE_FLOOR     # the tote's inside floor over the bench [ARITH]


def fixed_boxes():
    """Every fixed part: (name, lo, hi, color), world frame. The deck is drawn solid (plywood on sleepers), cut
    through by the hole, whose floor is the bench."""
    z0, z1 = TOP, TOP + DECK_Z
    hx0, hx1, hy0, hy1 = XO - OPEN / 2, XO + OPEN / 2, YO - OPEN / 2, YO + OPEN / 2
    out = [("deck_front", (DECK_X[0], DECK_Y[0], z0), (hx0, DECK_Y[1], z1), PLY),
           ("deck_back", (hx1, DECK_Y[0], z0), (DECK_X[1], DECK_Y[1], z1), PLY),
           ("deck_right", (hx0, DECK_Y[0], z0), (hx1, hy0, z1), PLY),
           ("deck_left", (hx0, hy1, z0), (hx1, DECK_Y[1], z1), PLY)]
    ix, iy, w = TOTE_IN[0] / 2, TOTE_IN[1] / 2, TOTE_WALL
    zf, zr = TOP + DECK_Z, TOP + DECK_Z + TOTE_H
    out += [("tote_floor", (XT - ix - w, YT - iy - w, zf), (XT + ix + w, YT + iy + w, zf + TOTE_FLOOR), TOTE_C),
            ("tote_near", (XT - ix - w, YT - iy - w, zf), (XT - ix, YT + iy + w, zr), TOTE_C),
            ("tote_far", (XT + ix, YT - iy - w, zf), (XT + ix + w, YT + iy + w, zr), TOTE_C),
            ("tote_right", (XT - ix, YT - iy - w, zf), (XT + ix, YT - iy, zr), TOTE_C),
            ("tote_left", (XT - ix, YT + iy, zf), (XT + ix, YT + iy + w, zr), TOTE_C)]
    return out


def _fixed(name, lo, hi, color):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Box(name=name, pos=tuple(((lo + hi) / 2).tolist()), size=tuple((hi - lo).tolist()), color=color)


# -- the seeded start ---------------------------------------------------------------------------------------------------
SPAWN_GAP = 0.0005               # loose parts start this far over what they rest on and settle in the reset [CHOICE]


def layout(seed):
    """The start for a seed: the cover (centre dx, dy, yaw) seated in the hole, its handle along x or along y (a
    quarter turn) and turned up to +-1.5 deg within the cleats' play; the wrench (centre x, y, yaw) on the pocket's
    floor, within +-10 mm of the hole's centre and +-10 deg of lying along x [CHOICE]. Seed None: all nominal."""
    if seed is None:
        return {"cover": (XO, YO, 0.0), "wrench": (XO, YO, 0.0)}
    rng = np.random.default_rng(int(seed) % (2 ** 32))
    quarter = int(rng.integers(0, 2))
    cdx, cdy = rng.uniform(-0.0015, 0.0015, 2)
    cyaw = quarter * math.pi / 2 + math.radians(rng.uniform(-1.5, 1.5))
    wdx, wdy = rng.uniform(-0.010, 0.010, 2)
    wyaw = math.radians(rng.uniform(-10.0, 10.0))
    return {"cover": (XO + cdx, YO + cdy, cyaw), "wrench": (XO + wdx, YO + wdy, wyaw)}


COVER = body("cover", (XO, YO, TOP + DECK_Z + SPAWN_GAP), COVER_SHAPES, MATS, color=YELLOW)
WRENCH = body("wrench", (XO, YO, TOP + SPAWN_GAP), WRENCH_SHAPES, MATS, color=STEEL_C)
OBJECTS = [*(_fixed(*f) for f in fixed_boxes()), COVER, WRENCH]


# -- small maths ----------------------------------------------------------------------------------------------------------
def _rot3(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _quat_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _rz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _np(x):
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=np.float64)


def _torch_of(x):
    return getattr(x, "torch", x)


def _yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def _wrap(a, period):
    """a wrapped into (-period/2, period/2]."""
    return a - period * math.floor(a / period + 0.5)


@on_place
def place_loose(env):
    """The cover's and the wrench's seeded starts (before the reset settles)."""
    import torch
    base = env.unwrapped
    lay = layout(run_seed())
    o = _torch_of(base.scene.env_origins)[0]
    for name, z in (("cover", TOP + DECK_Z + SPAWN_GAP), ("wrench", TOP + SPAWN_GAP)):
        asset = base.scene[name]
        x, y, yaw = lay[name]
        dev = _torch_of(asset.data.root_link_pose_w).device
        pose = torch.tensor([[x, y, z, *_quat_yaw(yaw)]], dtype=torch.float32, device=dev)
        pose[:, :3] += o
        zero = torch.zeros((1, 6), dtype=torch.float32, device=dev)
        if hasattr(asset, "write_root_pose_to_sim_index"):  # Isaac Lab 3.0
            asset.write_root_pose_to_sim_index(root_pose=pose)
            asset.write_root_velocity_to_sim_index(root_velocity=zero)
        else:
            asset.write_root_state_to_sim(torch.cat([pose, zero], dim=1))
        asset.write_data_to_sim()


# -- what the score measures (each is also a real-trial check, README) ----------------------------------------------------
FLAT_TOL = 0.003                 # each plate corner's underside within 3 mm of the deck top [CHOICE: a 3 mm feeler
#                                  leaf (or a 3 mm hex key) must not go under any corner; a cleat on the deck lifts
#                                  its side 19 mm]
OUTLINE_TOL = 0.020              # each plate corner within 20 mm of a corner of the pencilled seated outline [CHOICE:
#                                  above the 14.8 mm the cleats' play allows (tested), and 20 mm of the 40 mm overlap
#                                  stays on the deck all round]
CLEAT_DOWN = 0.010               # each cleat's foot at least 10 mm below the deck top, inside the hole [CHOICE]
DROP = 0.020                     # a plate corner 20 mm or more below the deck top: the cover went off the deck (over
#                                  its edge or into the hole) [CHOICE: a seated or set-aside cover never goes below it]
REST_TOL = 0.003                 # the wrench's lowest point within 3 mm of the tote's floor: resting on it [CHOICE: a
#                                  3 mm feeler leaf must not pass under the wrench's lowest point]
WALL_TOL = 0.002                 # the wrench may press this far into a tote wall (4 mm thick) and still be inside
#                                  [CHOICE: contact pushes a resting part a fraction of a mm into what it touches]
CLEAR = 0.030                    # every point of both grippers (fingers and housing) at least 30 mm from the cover
#                                  and the wrench [CHOICE: a hand that is off the parts, read with a tape]
STILL = 0.005                    # m/s: the cover and the wrench still [CHOICE]
SPIN = 0.05                      # rad/s [CHOICE]


def world_points(state_pos, state_quat, pts):
    R, p = _rot3(state_quat), np.asarray(state_pos, float)
    return np.asarray(pts, float) @ R.T + p


PLATE_CORNERS_C = np.array([(sx * PLATE / 2, sy * PLATE / 2, 0.0) for sx in (-1, 1) for sy in (-1, 1)])
OUTLINE = np.array([(XO + sx * PLATE / 2, YO + sy * PLATE / 2) for sx in (-1, 1) for sy in (-1, 1)])
FEET_C = np.array([p for s in CLEAT_FEET for p in s.points[:4]])   # each foot's bottom ring


def _corners(shapes):
    out = []
    for s in shapes:
        lo, hi = s.bounds()
        out += [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    return np.array(out)


WRENCH_CORNERS_W = _corners(WRENCH_SHAPES)


def in_hole_xy(p):
    return abs(p[0] - XO) <= OPEN / 2 and abs(p[1] - YO) <= OPEN / 2


def cover_check(pos, quat):
    """The cover from its plate corners and its cleats' feet (never its body origin)."""
    deck = TOP + DECK_Z
    corners = world_points(pos, quat, PLATE_CORNERS_C)
    gaps = corners[:, 2] - deck
    flat = bool(np.all(np.abs(gaps) <= FLAT_TOL))
    off = [float(np.min(np.linalg.norm(OUTLINE - c[:2], axis=1))) for c in corners]
    placed = max(off) <= OUTLINE_TOL
    feet = world_points(pos, quat, FEET_C)
    cleats = bool(all(in_hole_xy(f) and f[2] <= deck - CLEAT_DOWN for f in feet))
    return {"flat": flat, "placed": placed, "cleats": cleats, "seated": flat and placed and cleats,
            "gap_max": float(np.max(np.abs(gaps))), "outline_off": max(off), "dropped": bool(gaps.min() <= -DROP)}


def wrench_check(pos, quat):
    pts = world_points(pos, quat, WRENCH_CORNERS_W)
    ix, iy = TOTE_IN[0] / 2 + WALL_TOL, TOTE_IN[1] / 2 + WALL_TOL
    inside = bool(np.all(np.abs(pts[:, 0] - XT) <= ix) and np.all(np.abs(pts[:, 1] - YT) <= iy))
    low = float(pts[:, 2].min() - (TOP + TOTE_FLOOR_Z))
    resting = inside and -0.002 <= low <= REST_TOL
    in_hole = bool(any(in_hole_xy(p) and p[2] < TOP + DECK_Z for p in pts))
    # off the floor: its lowest point DROP or more below the deck's top anywhere but over the hole (fallen off the
    # deck onto the bench, or further)
    k = int(np.argmin(pts[:, 2]))
    dropped = bool(pts[k, 2] <= TOP + DECK_Z - DROP and not in_hole_xy(pts[k]))
    return {"in_tote": inside, "low": low, "away": bool(resting), "in_hole": in_hole, "dropped": dropped}


def points_to_shapes(pts_world, R, p, shapes):
    """Least distance from any of the world points (rows) to the shape boxes (bounds) of a body posed (R, p)."""
    q = (np.asarray(pts_world, float) - p) @ R
    best = np.inf
    for s in shapes:
        lo, hi = s.bounds()
        d = np.linalg.norm(np.maximum(np.maximum(lo - q, q - hi), 0.0), axis=1)
        best = min(best, float(d.min()))
    return best


def measure(objects, pads):
    cv, wr = objects["cover"], objects["wrench"]
    c = cover_check(cv.pos, cv.quat_xyzw)
    w = wrench_check(wr.pos, wr.quat_xyzw)
    if pads is not None and len(pads):
        d_cover = points_to_shapes(pads, _rot3(cv.quat_xyzw), np.asarray(cv.pos, float), COVER_SHAPES)
        d_wrench = points_to_shapes(pads, _rot3(wr.quat_xyzw), np.asarray(wr.pos, float), WRENCH_SHAPES)
    else:
        d_cover = d_wrench = 0.0  # WHY: unread hands count as touching
    still = (float(np.linalg.norm(cv.lin_vel)) < STILL and float(np.linalg.norm(cv.ang_vel)) < SPIN
             and float(np.linalg.norm(wr.lin_vel)) < STILL and float(np.linalg.norm(wr.ang_vel)) < SPIN)
    return {**c, **{"w_" + k: v for k, v in w.items()}, "d_cover": d_cover, "d_wrench": d_wrench, "still": still}


def rung(m, latched):
    """0 to 4. 0: the cover or the wrench went off the deck at any time, or the wrench is still in the hole, or neither
    the hole is
    covered nor the wrench put away. 1: wrench put away, hole not covered. 2: hole covered, wrench out of the hole
    but not put away. 3: both. 4: both, no gripper within CLEAR of the cover or the wrench, both still."""
    if latched or m["w_in_hole"]:
        return 0
    if m["seated"] and m["w_away"]:
        clear = m["d_cover"] >= CLEAR and m["d_wrench"] >= CLEAR
        return 4 if clear and m["still"] else 3
    if m["seated"]:
        return 2
    if m["w_away"]:
        return 1
    return 0


LADDER = {0: 0.0, 1: 0.25, 2: 0.5, 3: 0.75}

# -- per-step memory: the hands (read from the sim), the latch and the 1 s hold ------------------------------------------
TCP_OFFSET = (0.0, 0.0, 0.134)   # yam_arena.constants: the TCP along the gripper's approach axis
PAD_TIP = 0.147                  # pad tips along it: 13 mm past the TCP (yam_arena.constants.PAD_TIP_PAST_TCP)
_M = {"pads": None, "tcps": None, "cover": None, "wrench": None, "error": None}
_MEM = {"last": None, "now": None, "held": 0.0, "latched": False}
_CLOCK = {"dt": None}


def reset_memory():
    _M.update({"pads": None, "tcps": None, "cover": None, "wrench": None, "error": None})
    _MEM.update({"last": None, "now": None, "held": 0.0, "latched": False})
    _CLOCK.update({"dt": None})


@on_reset
def _reset():
    reset_memory()


# The gripper as points (mm back from the pad tips along the approach axis): finger slices (from, to, thickness outward
# of the pad face, half width along the pad axis) and the housing beyond 83.5 mm, 51 x 33 mm half sizes [EST from the
# YAM gripper mesh]. One model: the scorer and every tool use gripper_points.
HAND_SLICES = ((0, 10, 5.4, 11), (10, 20, 7.1, 14.8), (20, 40, 12.6, 22.6), (40, 60, 20.2, 30.4), (60, 83.5, 26, 36))
HAND_HOUSING = (83.5, 200.0, 51.0, 33.0)


def gripper_points(finger):
    """Points of one gripper in its TCP frame (x closing, y pad axis, z approach), its fingers |finger| either side."""
    f = abs(float(finger))
    tip = PAD_TIP - TCP_OFFSET[2]
    pts = []
    for h0, h1, out, hw in HAND_SLICES:
        for h in np.linspace(h0, h1, 3) / 1000:
            for s in (-1.0, 1.0):
                for dx in np.linspace(0.0, out / 1000, 3):
                    for y in np.linspace(-hw / 1000, hw / 1000, 5):
                        pts.append((s * (f + dx), y, tip - h))
    h0, h1, hx, hy = HAND_HOUSING
    for h in np.linspace(h0, h1, 7) / 1000:
        for x in np.linspace(-hx / 1000, hx / 1000, 7):
            for y in np.linspace(-hy / 1000, hy / 1000, 5):
                pts.append((x, y, tip - h))
    return np.array(pts)


def hand_points(g_pos, g_quat, finger):
    """One hand's gripper points in the world (rows), from the gripper link's pose and its finger joint."""
    R, g = _rot3(g_quat), np.asarray(g_pos, float)
    tcp = g + R @ np.array(TCP_OFFSET)
    return tcp + gripper_points(finger) @ R.T


@on_step
def read_hands(env):
    """Both hands' gripper points after every control step; also the TCPs and the cover's and wrench's poses, which
    only the scripted demo's line-up passes read (never the score)."""
    try:
        base = env.unwrapped
        origin = _np(base.scene.env_origins)[0]
        pads, tcps = [], {}
        for side in ("left", "right"):
            rob = base.scene[f"{side}_robot"]
            g = _np(rob.data.body_link_pose_w)[0][list(rob.body_names).index("gripper")]
            q = _np(rob.data.joint_pos)[0]
            pads.append(hand_points(g[:3] - origin, g[3:7], q[list(rob.joint_names).index("joint7")]))
            tcps[side] = g[:3] - origin + _rot3(g[3:7]) @ np.array(TCP_OFFSET)
        _M["pads"], _M["tcps"] = np.vstack(pads), tcps
        _CLOCK["dt"] = float(base.step_dt)
    except Exception as e:  # noqa: BLE001  WHY: never stop the run; unread hands count as touching
        _M["error"], _M["pads"] = repr(e), None
    try:  # WHY apart: a failed read here must not blank the hands the score reads
        base = env.unwrapped
        origin = _np(base.scene.env_origins)[0]
        for name in ("cover", "wrench"):
            p = _np(base.scene[name].data.root_link_pose_w)[0]
            _M[name] = (p[:3] - origin, p[3:7])
    except Exception as e:  # noqa: BLE001
        _M["cover"] = _M["wrench"] = None
        _M["error"] = repr(e)


def update(objects):
    if objects is _MEM["last"]:
        return _MEM
    _MEM["last"] = objects
    m = measure(objects, _M["pads"])
    _MEM["latched"] = _MEM["latched"] or m["dropped"] or m["w_dropped"]
    r = rung(m, _MEM["latched"])
    done = r == 4
    _MEM["held"] = _MEM["held"] + (_CLOCK["dt"] or 0.0) if done else 0.0
    _MEM["now"] = {"m": m, "rung": r, "done": done}
    return _MEM


def success(objects):
    """Hole covered, wrench resting in the tote, hands off both, both still, held 1 s (ends the run)."""
    n = update(objects)["now"]
    return n["done"] and _MEM["held"] >= SUCCESS_HOLD_S - 1e-9


def score(objects):
    """From the last step (and the drop latch): 1.0 rung 4 held 1 s; 0.9 rung 4; 0.75 covered and put away with a
    hand on a part or a part moving; 0.5 covered, wrench out but not put away; 0.25 put away, hole not covered; 0."""
    n = update(objects)["now"]
    if n["done"]:
        return 1.0 if _MEM["held"] >= SUCCESS_HOLD_S - 1e-9 else 0.9
    return LADDER[n["rung"]]


def distance(objects):
    return 1.0 - score(objects)


# -- the demo -------------------------------------------------------------------------------------------------------------
# The stock Waypoint carries only a yaw for a straight-down grip. Here a waypoint's yaw may be a Rot: a float (the yaw)
# that also carries the full TCP rotation, and grip_rotation() (which yam_arena's ArmPlan calls for every waypoint)
# returns it. The load compensation (commanded) needs it: the weight tips the wrist, so the command is tipped the other
# way. This changes only the scripted demo policy, never the scene or the score.
class Rot(float):
    """A waypoint yaw that carries a full TCP rotation matrix R (columns: closing axis, pad axis, approach)."""

    def __new__(cls, R):
        R = np.asarray(R, float)
        obj = super().__new__(cls, math.atan2(R[1, 0], R[0, 0]))
        obj.R = R
        return obj

    def turned(self, dyaw):
        return Rot(_rz(dyaw) @ self.R)


def _patch_grip_rotation():
    import yam_arena.demo as d
    if getattr(d.grip_rotation, "_hc_patched", False):
        return
    stock = d.grip_rotation

    def grip_rotation(yaw=0.0):
        R = getattr(yaw, "R", None)
        return np.array(R) if R is not None else stock(float(yaw))

    grip_rotation._hc_patched = True
    d.grip_rotation = grip_rotation


_patch_grip_rotation()

OPEN_F = -0.0465                 # fingers fully open [CHOICE: just inside FINGER_OPEN -0.04695]
HANDLE_OPEN = -0.032             # fingers either side of the 38 mm handle before the grip: 13 mm clear each side
HANDLE_GRIP = -0.014             # 5 mm past each face of the handle [CHOICE]
WRENCH_OPEN = -0.024             # either side of the 13 mm beam: 17.5 mm clear each side [CHOICE]
WRENCH_GRIP = -0.002             # 4.5 mm past each face of the beam [CHOICE]
WRENCH_LETGO = -0.022            # opened off the beam inside the tote [CHOICE]
TIP_OVER_PLATE = 0.006           # pad tips this far over the plate when gripping the handle [CHOICE]
TIP_OVER_FLOOR = 0.0015          # pad tips this far over the wrench's bottom (the floor) when gripping it [CHOICE]
TIP = PAD_TIP - TCP_OFFSET[2]    # TCP to pad tips [ARITH: 13 mm]
GRIP_C = np.array([0.0, 0.0, PLY_T + TIP_OVER_PLATE + TIP])   # the TCP on the handle, in C [ARITH]
GRIP_W = np.array([WRENCH_COM[0], 0.0, TIP_OVER_FLOOR + TIP])  # the TCP on the beam, in W, over its centre of mass
CLEAT_LIFT = 0.012               # the cleats' feet this far over the deck while the cover is carried [CHOICE]
SET_PUSH = 0.004                 # a part set down is commanded this far past resting, so what is under it takes it
#                                  [CHOICE: about 1 N at the arm's stiffness, tools/sagcheck.py]
WRENCH_CARRY = TOTE_H + 0.025    # the wrench's bottom this far over the deck while carried: 25 mm over the tote's rim
PARK = (0.270, 0.280)            # where the cover is set aside, standing on its cleats on the deck [CHOICE: left arm]
DECK_SPOT = (0.215, -0.462)      # s_on_deck: the wrench set down on the deck here, along x, right of the tote
#                                  [CHOICE: the open fingers stay 18 mm off the tote's wall]
HELD_UP = 0.020                  # s_held: the wrench held this far over the tote's floor [CHOICE: the verdict's case]
DROP_UP = 0.060                  # s_drop: the wrench let go this far over the tote's floor [CHOICE]
OFF_EDGE = 0.010                 # s_slid_off: the cover's centre set this far past the deck's left edge [CHOICE: it
#                                  tips off onto the bench and stays on it, its far edge about 15 mm inside]
STEADY = 0.8                     # s over a part before going down to it [CHOICE]
VARIANT = "demo"                 # sim/mk.sh sets this in stem copies: see plan()
VARIANTS = ("demo", "nothing", "held", "drop", "slid_off", "aside", "on_deck", "touch", "back")


def _w(t, p, f, label, yaw):
    return Waypoint(float(t), (float(p[0]), float(p[1]), float(p[2])), float(f), label, yaw)


def cover_tcp(cx, cy, cyaw, plate_z):
    """The TCP on the handle for a cover centred at (cx, cy), turned cyaw, its plate's underside plate_z over the
    bench."""
    return np.array([cx, cy, TOP + plate_z]) + _rz(cyaw) @ GRIP_C


def wrench_tcp(wx, wy, wyaw, bottom_z):
    """The TCP on the beam for a wrench frame at (wx, wy), turned wyaw, its bottom bottom_z over the bench."""
    return np.array([wx, wy, TOP + bottom_z]) + _rz(wyaw) @ GRIP_W


YAW_CENTRE = {"left": -math.pi / 4, "right": math.pi / 4}   # [RUN: IK at the hole (x 0.27, y 0) puts the left wrist
#                                  (joint6) at -48 deg - yaw and refuses yaw +90 deg (joint6 -120 deg, its stop); the right
#                                  arm mirrors it]


def hand_yaw(part_yaw, arm):
    """The grip's yaw for a part turned part_yaw: the fingers close across the handle (or beam), which is symmetric
    under a half turn, so of the two yaws the one within 90 deg of the arm's YAW_CENTRE is used."""
    c = YAW_CENTRE[arm]
    return _wrap(part_yaw - c, math.pi) + c


SEAT_LABELS = ("line up over the hole", "line up again")
PICK_LABELS = {"line up on the handle": "lift the cover", "line up on the set-aside cover": "lift it again",
               "line up on the wrench": "lift out of the hole"}
TOTE_LABEL = "line up over the tote"
UNLOAD_LABELS = ("rest on the deck", "seat the cover", "rest on the tote floor", "rest on the deck spot")


def left_plan(start, variant, t_resume):
    """The cover hand: grip the handle, lift the cover clear of the hole, set it aside on the deck, let go; later
    (from t_resume) grip it again, carry it back over the hole, line it up, lower it until the cleats drop in, let go
    and draw away."""
    cx, cy, cyaw = start
    yh = hand_yaw(cyaw, "left")
    seat_z = DECK_Z
    lift_z = DECK_Z + CLEAT_H + CLEAT_LIFT
    park_z = DECK_Z + CLEAT_H
    g0 = cover_tcp(cx, cy, cyaw, seat_z)
    above = g0 + np.array([0.0, 0.0, 0.06])
    W = [_w(3.0, above, HANDLE_OPEN, "over the handle", yh),
         _w(STEADY, above, HANDLE_OPEN, "line up on the handle", yh),
         _w(1.5, g0, HANDLE_OPEN, "down round the handle", yh),
         _w(0.8, g0, HANDLE_GRIP, "grip the handle", yh),
         _w(1.5, cover_tcp(cx, cy, cyaw, lift_z), HANDLE_GRIP, "lift the cover", yh)]
    if variant == "slid_off":                               # set down hanging over the deck's left edge; it tips off
        ex = (PARK[0], DECK_Y[1] + OFF_EDGE)
        # WHY turned on the way: at the deck's left edge the left wrist reaches only yaws near 0 [RUN: IK, joint6 at
        # its stop for -90 deg]; the square cover may be turned a quarter turn in the hand
        ye = min((yh + k * math.pi / 2 for k in range(-2, 3)), key=abs)
        W += [_w(2.5, cover_tcp(*ex, cyaw, lift_z), HANDLE_GRIP, "carry over the edge", ye),
              _w(1.2, cover_tcp(*ex, cyaw, DECK_Z + 0.004), HANDLE_GRIP, "lower over the edge", ye),
              _w(0.6, cover_tcp(*ex, cyaw, DECK_Z + 0.004), HANDLE_OPEN, "let go over the edge", ye),
              _w(1.5, cover_tcp(*ex, cyaw, DECK_Z + 0.08), HANDLE_OPEN, "up and away", ye)]
        return W
    gp = cover_tcp(PARK[0], PARK[1], cyaw, park_z)
    W += [_w(2.5, cover_tcp(PARK[0], PARK[1], cyaw, lift_z), HANDLE_GRIP, "carry to the side", yh),
          _w(1.2, gp - np.array([0.0, 0.0, SET_PUSH]), HANDLE_GRIP, "set it down", yh),
          _w(0.5, gp - np.array([0.0, 0.0, SET_PUSH]), HANDLE_GRIP, "rest on the deck", yh),
          _w(0.6, gp, HANDLE_OPEN, "let go", yh),
          _w(1.0, gp + np.array([0.0, 0.0, 0.06]), HANDLE_OPEN, "up off the handle", yh)]
    if variant == "aside":                                  # the cover left set aside: the hole stays open
        return W + [_w(1.5, np.array([0.18, 0.30, TOP + 0.15]), HANDLE_OPEN, "away", 0.0),
                    _w(25.0, np.array([0.18, 0.30, TOP + 0.15]), HANDLE_OPEN, "rest away", 0.0)]
    t_now = sum(w.duration for w in W)
    W += [_w(max(0.5, t_resume - t_now), gp + np.array([0.0, 0.0, 0.06]), HANDLE_OPEN, "wait beside", yh),
          _w(0.6, gp + np.array([0.0, 0.0, 0.06]), HANDLE_OPEN, "line up on the set-aside cover", yh),
          _w(1.5, gp, HANDLE_OPEN, "down round the handle again", yh),
          _w(0.8, gp, HANDLE_GRIP, "grip it again", yh),
          _w(1.2, cover_tcp(PARK[0], PARK[1], cyaw, lift_z), HANDLE_GRIP, "lift it again", yh)]
    sx, sy = XO, YO
    W += [_w(2.5, cover_tcp(sx, sy, cyaw, lift_z), HANDLE_GRIP, "carry over the hole", yh),
          _w(0.8, cover_tcp(sx, sy, cyaw, lift_z), HANDLE_GRIP, SEAT_LABELS[0], yh),
          _w(1.2, cover_tcp(sx, sy, cyaw, DECK_Z + CHAMFER + 0.004), HANDLE_GRIP, "lower toward the hole", yh),
          _w(0.5, cover_tcp(sx, sy, cyaw, DECK_Z + CHAMFER + 0.004), HANDLE_GRIP, SEAT_LABELS[1], yh)]
    g_seat = cover_tcp(sx, sy, cyaw, seat_z)
    W += [_w(1.5, g_seat - np.array([0.0, 0.0, SET_PUSH]), HANDLE_GRIP, "lower the cleats in", yh),
          _w(0.6, g_seat - np.array([0.0, 0.0, SET_PUSH]), HANDLE_GRIP, "seat the cover", yh),
          _w(0.6, g_seat, HANDLE_OPEN, "let go of the cover", yh),
          _w(1.2, g_seat + np.array([0.0, 0.0, 0.07]), HANDLE_OPEN, "up off the cover", yh),
          _w(1.5, np.array([0.18, 0.30, TOP + 0.15]), HANDLE_OPEN, "away", 0.0),
          _w(2.5, np.array([0.18, 0.30, TOP + 0.15]), HANDLE_OPEN, "rest away", 0.0)]
    return W


def right_plan(start, variant, t_start):
    """The wrench hand: wait beside the deck until the hole is open (t_start), go down into the hole, grip the wrench
    over its centre of mass, lift it out, turn it along the tote, lower it onto the tote's floor, let go, draw
    away."""
    wx, wy, wyaw = start
    yw = hand_yaw(wyaw, "right")
    wait_p = np.array([0.16, -0.16, TOP + 0.15])   # [RUN: TOP+0.20 left joint4 3 deg off its stop]
    g0 = wrench_tcp(wx, wy, wyaw, 0.0)
    above = wrench_tcp(wx, wy, wyaw, DECK_Z + 0.03)
    carry = wrench_tcp(wx, wy, wyaw, DECK_Z + WRENCH_CARRY)
    W = [_w(3.0, wait_p, OPEN_F, "beside the deck", 0.0),
         _w(max(0.5, t_start - 3.0), wait_p, OPEN_F, "wait for the hole", 0.0),
         _w(2.0, above, WRENCH_OPEN, "over the wrench", yw),
         _w(STEADY, above, WRENCH_OPEN, "line up on the wrench", yw),
         _w(1.5, g0, WRENCH_OPEN, "down round the wrench", yw),
         _w(0.8, g0, WRENCH_GRIP, "grip the wrench", yw),
         _w(1.5, carry, WRENCH_GRIP, "lift out of the hole", yw)]
    if variant == "on_deck":                                # set down on the deck, not in the tote
        sx, sy = DECK_SPOT
        yd = 0.0
        gd = wrench_tcp(sx, sy, yd, DECK_Z)
        W += [_w(2.5, wrench_tcp(sx, sy, yd, DECK_Z + WRENCH_CARRY), WRENCH_GRIP, "carry to the deck spot", yd),
              _w(1.5, gd + np.array([0.0, 0.0, 0.002]), WRENCH_GRIP, "lower to the deck", yd),
              _w(0.6, gd, WRENCH_GRIP, "rest on the deck spot", yd),
              _w(0.6, gd, WRENCH_LETGO, "let go of the wrench", yd),
              # WHY 0.10: the hand then passes over the tote's right wall (rim 65 mm over the deck) on its way back
              _w(1.5, gd + np.array([0.0, 0.0, 0.10]), WRENCH_LETGO, "up off the wrench", yd),
              _w(1.5, wait_p, OPEN_F, "away", 0.0)]
        return W
    t_over = wrench_tcp(XT, YT, 0.0, DECK_Z + WRENCH_CARRY)
    floor = TOTE_FLOOR_Z
    W += [_w(2.5, t_over, WRENCH_GRIP, "carry to the tote", 0.0),
          _w(0.6, t_over, WRENCH_GRIP, TOTE_LABEL, 0.0)]
    if variant == "held":                                   # held inside the tote, above its floor, to the end
        g = wrench_tcp(XT, YT, 0.0, floor + HELD_UP)
        return W + [_w(1.5, g, WRENCH_GRIP, "hold above the floor", 0.0),
                    _w(25.0, g, WRENCH_GRIP, "keep holding", 0.0)]
    if variant == "drop":                                   # let go high over the floor
        g = wrench_tcp(XT, YT, 0.0, floor + DROP_UP)
        return W + [_w(1.2, g, WRENCH_GRIP, "drop height", 0.0),
                    _w(0.6, g, WRENCH_LETGO, "let go of the wrench", 0.0),
                    _w(1.5, g + np.array([0.0, 0.0, 0.03]), WRENCH_LETGO, "up out of the tote", 0.0),
                    _w(1.5, wait_p, OPEN_F, "away", 0.0)]
    g = wrench_tcp(XT, YT, 0.0, floor)
    # WHY no push past the floor for the wrench: the pad tips are only TIP_OVER_FLOOR over its bottom; the 1.4 N
    # wrench sags the unloaded hand about 1 mm onto the floor anyway (tools/sagcheck.py)
    W += [_w(1.5, g + np.array([0.0, 0.0, 0.002]), WRENCH_GRIP, "lower into the tote", 0.0),
          _w(0.6, g, WRENCH_GRIP, "rest on the tote floor", 0.0)]
    if variant == "touch":                                  # the wrench resting on the floor, still gripped
        return W + [_w(25.0, g, WRENCH_GRIP, "keep gripping", 0.0)]
    return W + [_w(0.6, g, WRENCH_LETGO, "let go of the wrench", 0.0),
                _w(1.5, g + np.array([0.0, 0.0, 0.10]), WRENCH_LETGO, "up out of the tote", 0.0),
                _w(1.5, wait_p, OPEN_F, "away", 0.0)]


def _t_until(W, label, inclusive=True):
    t = 0.0
    for w in W:
        if w.label == label:
            return t + (w.duration if inclusive else 0.0)
        t += w.duration
    raise KeyError(label)


# Waypoints in free air, no part within 7 mm: the IK check allows these 3 mm and 2 deg (tools/ikcheck.py), every other
# one 0.5 mm and 0.5 deg.
FREE_AIR = ("over the handle", "beside the deck", "wait for the hole", "away", "rest away", "up off the handle",
            "wait beside", "up off the cover", "over the wrench", "up out of the tote", "up and away",
            "up off the wrench")
HOLE_FREE_MARGIN = 0.5           # s between one hand leaving the hole's space and the other entering it [CHOICE]
HOLD_TAIL = ("keep holding", "keep gripping", "rest away")


def _plan(seed, v):
    if v == "nothing":
        return {}
    lay = layout(seed)
    probe = left_plan(lay["cover"], v, 0.0)
    t_open = _t_until(probe, "up off the handle") if v != "slid_off" else _t_until(probe, "let go over the edge")
    if v == "back":                                         # the cover lifted and put straight back, wrench left
        return {"left": left_plan(lay["cover"], v, t_open + 2.0)}
    right = right_plan(lay["wrench"], v, t_open + HOLE_FREE_MARGIN)
    if v == "slid_off":
        return {"left": probe, "right": right}
    t_out = _t_until(right, "lift out of the hole")
    return {"left": left_plan(lay["cover"], v, t_out + HOLE_FREE_MARGIN), "right": right}


SPLIT = ("carry to the side", "carry over the hole", "carry over the edge", "carry to the tote", "carry to the deck spot")
SPLIT_STEP = (0.05, math.radians(10.0))   # a loaded carry is cut into pieces of at most 50 mm and 10 deg [CHOICE: the
#                                  load compensation is exact only at waypoints; uncut, the hand strayed up to 3.9 mm
#                                  mid-carry under the cover (RUN tools/sagcheck.py)]


def split_carries(W):
    """Each SPLIT waypoint cut into equal straight pieces (position and yaw linear) of the same label."""
    out = []
    for w in W:
        if w.label in SPLIT and out:
            a, b = np.asarray(out[-1].pos, float), np.asarray(w.pos, float)
            ya, yb = float(out[-1].yaw), float(w.yaw)
            n = max(1, int(math.ceil(max(np.linalg.norm(b - a) / SPLIT_STEP[0], abs(yb - ya) / SPLIT_STEP[1]) - 1e-9)))
            for k in range(1, n + 1):
                out.append(_w(w.duration / n, a + (b - a) * k / n, w.finger, w.label, ya + (yb - ya) * k / n))
        else:
            out.append(w)
    return out


def plan(seed=None, variant=None):
    v = VARIANT if variant is None else variant
    p = {arm: split_carries(W) for arm, W in _plan(seed, v).items()}
    _SERVO.update({"on": True, "done": {a: set() for a in p}, "log": [],
                   "events": {a: servo_events(W) for a, W in p.items()}})
    return p


def demo():
    """Left hand: lift the cover off the hole, set it aside on the deck, later put it back with its cleats in the hole
    and let go. Right hand: once the hole is open, take the wrench out and set it on the tote's floor, let go. Loaded
    waypoints are commanded against the arm's sag under the part (commanded)."""
    return commanded(plan(run_seed()))


# -- the demo's load compensation ---------------------------------------------------------------------------------------
# WHY: the arm holds each joint on a PD spring (yam_arena.constants: 80 N m/rad at joints 1 to 3, 10 at joints 4 to 6)
# and its gravity compensation covers the arm alone, so a held part bends the hand off the plan (the mower-bag task's
# traced PhysX run sagged 45 mm under 1 kg). Each loaded waypoint is commanded at the joint angles the weight bends onto
# the planned pose: q_cmd = q - K^-1 J(q)^T w (J the TCP's 6x6 Jacobian, w the part's wrench at the TCP). Plans, tests
# and tools/clearance.py keep the planned poses (where the hand should end up).
ARM_K = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)   # N m/rad, joints 1 to 6
LOADED = {"left": frozenset({"lift the cover", "carry to the side", "lift it again", "carry over the hole",
                             *SEAT_LABELS, "lower toward the hole", "carry over the edge", "lower over the edge"}),
          "right": frozenset({"lift out of the hole", "carry to the tote", TOTE_LABEL, "lower into the tote",
                              "hold above the floor", "keep holding", "drop height", "carry to the deck spot",
                              "lower to the deck"})}
# WHY not the set-downs and after: what is under the part takes its weight there; commanded with the load, the hand
# would lift it off ("keep gripping" in s_touch rests on the floor, unloaded; "keep holding" in s_held is in the air).
PAYLOAD = {"left": (COVER_KG, COVER_COM - GRIP_C), "right": (WRENCH_KG, WRENCH_COM - GRIP_W)}


def tcp_jacobian(q, base):
    """6x6 Jacobian of the TCP (linear; angular, world frame) in the arm joints, by finite differences of ik.tcp_pose."""
    p0, R0 = ik.tcp_pose(q, base)
    J, e = np.zeros((6, 6)), 1e-6
    for i in range(6):
        dq = np.array(q, float)
        dq[i] += e
        p, R = ik.tcp_pose(dq, base)
        W = (R - R0) @ R0.T / e
        J[:3, i], J[3:, i] = (p - p0) / e, 0.5 * np.array([W[2, 1] - W[1, 2], W[0, 2] - W[2, 0], W[1, 0] - W[0, 1]])
    return J


def part_wrench(arm, R_tcp):
    """The held part's weight as a wrench at the TCP (force; moment about the TCP), the part turned with the hand."""
    from yam_arena.demo import grip_rotation
    mass, rel = PAYLOAD[arm]
    Rp = R_tcp @ grip_rotation(0.0).T
    r = Rp @ rel
    F = np.array([0.0, 0.0, -9.81 * mass])
    return np.r_[F, np.cross(r, F)]


def sag_dq(arm, q, base, R_tcp):
    """Joint deflection under the held part at joints q: K^-1 J^T w [ARITH, linear joint springs]."""
    return tcp_jacobian(q, base).T @ part_wrench(arm, R_tcp) / ARM_K


def commanded(plans):
    """The plans with each arm's LOADED waypoints moved against the part's sag (see above)."""
    from yam_arena.demo import grip_rotation
    out = dict(plans)
    for arm, Wp in plans.items():
        base, q, W = np.asarray(station.BASES[arm], float), np.array(READY, float), []
        for w in Wp:
            R = grip_rotation(w.yaw)
            q, _, _ = ik.solve(w.pos, ik.matrix_to_quat_xyzw(R), q, base_pos=base, iters=200)
            if w.label in LOADED[arm]:
                p, Rc = ik.tcp_pose(q - sag_dq(arm, q, base, R), base)
                w = dataclasses.replace(w, pos=(float(p[0]), float(p[1]), float(p[2])), yaw=Rot(Rc))
            W.append(w)
        out[arm] = W
    return out


# -- the demo's line-up passes: privileged, from the parts' true poses (the score never reads them) -----------------------
# WHY: the arm is compliant and the parts settle; the cleats leave 6 mm a side in the hole and the beam 17.5 mm either
# side of the open fingers. A person looks; the script reads the poses instead. Each pass moves later waypoints only, so
# the playing segment's goal stays put and the hand never jumps.
_SERVO = {"on": False, "done": {}, "log": [], "events": {}}
SERVO_MAX = (0.025, math.radians(8.0))   # the most one pass moves the hand (m, rad) [CHOICE]


def servo_events(W):
    labels = (*SEAT_LABELS, *PICK_LABELS, TOTE_LABEL, *UNLOAD_LABELS)
    out, t = [], 0.0
    for w in W:
        t += w.duration
        if w.label in labels:
            out.append((t - 0.1, w.label))
    return out


def shift_after(waypoints, t, d, dyaw, pivot, stop=None):
    """Waypoints after the one playing at plan time t, turned dyaw about the world point pivot (vertical axis) and then
    moved by d; with stop, only up to and including the first waypoint labelled stop."""
    end, i = 0.0, len(waypoints)
    for k, w in enumerate(waypoints):
        end += w.duration
        if t < end:
            i = k
            break
    out = list(waypoints[:i + 1])
    moving = True
    for w in waypoints[i + 1:]:
        if moving:
            p = np.asarray(w.pos, float)
            p = np.asarray(pivot) + _rz(dyaw) @ (p - np.asarray(pivot)) + np.asarray(d)
            yaw = w.yaw.turned(dyaw) if isinstance(w.yaw, Rot) else float(w.yaw) + dyaw
            w = dataclasses.replace(w, pos=(float(p[0]), float(p[1]), float(p[2])), yaw=yaw)
        out.append(w)
        if stop is not None and w.label == stop:
            moving = False
    return out


def pick_error(arm, label, pose, planned_tcp, planned_yaw):
    """(d, dyaw) from the planned grip to the grip on the part's true pose."""
    p, q = pose
    R = _rot3(q)
    yaw = _yaw_of(R)
    if label == "line up on the wrench":
        want = np.asarray(p, float) + R @ GRIP_W
    else:
        want = np.asarray(p, float) + R @ GRIP_C
    dyaw = _wrap(hand_yaw(yaw, arm) - float(planned_yaw), math.pi)
    d = want - np.asarray(planned_tcp, float)
    return d, dyaw


def place_error(label, pose, tcp):
    """(d, dyaw) moving the hand (about its TCP) so the held part lands where this line-up wants it (xy and yaw)."""
    p, q = pose
    R = _rot3(q)
    yaw = _yaw_of(R)
    if label == TOTE_LABEL:
        centre = np.asarray(p, float) + R @ np.array([WRENCH_COM[0], 0.0, 0.0])
        want, dyaw = np.array([XT, YT]), -_wrap(yaw, math.pi)
    else:
        centre = np.asarray(p, float)
        want, dyaw = np.array([XO, YO]), -_wrap(yaw, math.pi / 2)
    turned = tcp + _rz(dyaw) @ (centre - tcp)
    d = np.array([want[0] - turned[0], want[1] - turned[1], 0.0])
    return d, dyaw


def next_grip(waypoints, t):
    """The first waypoint after the one playing at plan time t that goes down round a part (the grip pose)."""
    end = 0.0
    for w in waypoints:
        end += w.duration
        if end > t and w.label.startswith("down round"):
            return w
    raise ValueError("no grip after t")


def servo(plans, t):
    if not _SERVO["on"] or _M.get("tcps") is None:
        return
    for arm, events in _SERVO["events"].items():
        if arm not in plans:
            continue
        for k, (tk, label) in enumerate(events):
            if k in _SERVO["done"][arm] or t < tk:
                continue
            _SERVO["done"][arm].add(k)
            tcp = np.asarray(_M["tcps"][arm], float)
            part = _M["wrench"] if (arm == "right") else _M["cover"]
            if part is None:
                continue
            stop, dyaw = None, 0.0
            if label in UNLOAD_LABELS:     # what is under the part holds it: aim the hand where it is
                d = tcp - np.asarray(plans[arm].target(t)[0], float)
            elif label in PICK_LABELS:     # the grip moves to the part; the carry targets stay where the part goes
                stop = PICK_LABELS[label]
                nxt = next_grip(plans[arm].waypoints, t)
                d, dyaw = pick_error(arm, label, part, nxt.pos, nxt.yaw)
                tcp = np.asarray(nxt.pos, float)
            else:
                d, dyaw = place_error(label, part, tcp)
            dyaw = float(np.clip(dyaw, -SERVO_MAX[1], SERVO_MAX[1]))
            d = np.clip(d, -SERVO_MAX[0], SERVO_MAX[0])
            plans[arm].waypoints = shift_after(plans[arm].waypoints, t, d, dyaw, tcp, stop)
            rec = {"arm": arm, "pass": k, "label": label, "t": round(t, 2), "d_mm": [round(1000 * v, 2) for v in d],
                   "dyaw_deg": round(math.degrees(dyaw), 3)}
            _SERVO["log"].append(rec)
            print("[hole cover line-up] " + str(rec), flush=True)


def _replan_demo_after_reset():
    """WHY: inspect-robots resets the policy before the embodiment, so yam_scripted would call demo() before the reset
    has set this trial's seed. This replans on the first action, after the reset, and runs the demo's line-up passes
    before each action. It changes only the scripted demo policy, never the scene or the score."""
    try:
        from yam_arena import scripted
    except ImportError:
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_hc_replan", False):
        return
    reset, act = cls.reset, cls.act
    unset = object()

    def patched_reset(self, scene):
        self._hc_scene, self._hc_seed = scene, unset
        reset(self, scene)

    def patched_act(self, observation):
        if getattr(self, "_hc_seed", None) is unset:
            self._hc_seed = run_seed()
            reset(self, self._hc_scene)
        servo(self._plans, float(observation.state_time) + self._dt)
        return act(self, observation)

    cls.reset, cls.act, cls._hc_replan = patched_reset, patched_act, True


_replan_demo_after_reset()
