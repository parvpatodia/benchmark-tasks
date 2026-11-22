"""Return a shopping trolley. Two nested trolleys are shown by their push handles: the returned (near) trolley's handle
and, 178 mm further on, the handle of the trolley in front. Each handle carries a coin lock. The near lock holds a US
quarter. The front lock's chain hangs free with its key at the end. Take the hanging key, turn it level, push it home
in the near lock's key slot, then take the quarter that the lock pushes out and lay it on the mat.

Basis: the coin lock mechanism of EP0427819B1 (src/ep0427819.txt): the coupling member (the key) goes in through an
opening in a side wall, a coin slide runs at right angles to it, and on full insertion the key releases the coin slide,
which a compression spring pushes out "so that the coin can be removed". Lock size and chain from a coin lock listing
(src/supermarket_fixture_coinlock.txt); nesting distance from a tight-nesting cart sheet (src/unarco_M60TN.txt); quarter from the
US Mint (src/usmint_quarter.txt).
"""
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, body, hull, mechanism, on_reset, on_step, run_seed  # noqa: E402,F401
from motionkit import (FINGER_OPEN, HAND, RELEASE, Plan, SagCorrector, as_grip, box_distance, down, finger_for, install,  # noqa: E402
                       level, mode, plans_out, tilted)
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 1600           # 80 s at 20 Hz
TOP = TABLE_TOP[SCENE]
MODES = ("demo", "nothing", "partway", "pry_coin", "key_only", "drop_coin", "hold_coin", "no_key_coin")
EXPECT = {"demo": 1.0, "nothing": 0.0, "partway": 0.25, "pry_coin": 0.0, "key_only": 0.5, "drop_coin": 0.5,
          "hold_coin": 0.5, "no_key_coin": 0.0}

# Every number: [SOURCE] from a saved file in src/, [ARITH] derived, [EST] estimate (measure on the bought part),
# [CHOICE] design choice, [RUN] measured in simulation. World: x forward from the arm bases, y left, z up; metres.
# Lock frame: world aligned, origin on the handle tube's axis under the lock's middle.
IN = 0.0254
# -- trolleys ------------------------------------------------------------------------------------------------------------
NEST = 7 * IN                    # handle to handle of two nested trolleys [SOURCE: unarco_M60TN.txt, nesting distance 7"]
TUBE_R = 0.0125                  # handle tube radius [ARITH: the lock fits 25-27 mm handles, SOURCE supermarket_fixture_coinlock.txt; 25 mm tube]
HANDLE_LEN = 0.30                # handle section between its side tubes [CHOICE: a real handle is wider; 300 mm keeps
#                                  the resting left arm and the working right elbow clear of its ends]
FRAME_RUN = 0.08                 # each side tube runs this far forward (+x) as it falls to the bench [EST]
H = 0.340                        # handle axis over the bench [CHOICE: the 959 mm real handle lowered into the arms'
#                                  reach; the key's tip still hangs 66.5 mm clear of the bench]
XN = 0.150                       # near handle axis, world x [CHOICE]
XF = XN + NEST                   # front handle axis [ARITH: 0.328]
YL = 0.050                       # both locks' middles, world y [CHOICE: the near handle's -y end clear of the right
#                                  arm's elbow when it works at the slot]
# -- coin lock (supermarket_fixture_coinlock.txt: "Size:90*70*68mm", 260 mm chain, coins 23-28 mm) ---------------------------------
LOCK_L, LOCK_W, LOCK_H = 0.090, 0.070, 0.068   # along the handle, across it, tall [SOURCE]
LOCK_Z0 = TUBE_R                 # the lock sits on the tube [CHOICE]
LOCK_Z1 = LOCK_Z0 + LOCK_H       # [ARITH: 0.0805]
FX = LOCK_W / 2                  # front face (toward the trolley in front), lock x [ARITH]
SLOT_Y, SLOT_Z = -0.030, 0.024   # key slot centre in the front face, lock frame [CHOICE: near the -y end, low (the
#                                  patent's key slot runs under the coin slide)]
SLOT_W, SLOT_T = 0.021, 0.0045   # slot opening across and tall [CHOICE: 3 and 0.75 mm play a side for a rigid key]
PLUNGER_IN = 0.008               # plunger face this far inside the mouth [EST]
RELEASE_TRAVEL = 0.0045          # plunger travel that frees the coin slide [EST]
PLUNGER_STOP = 0.006             # plunger travel to its stop [EST]
HOME_DEPTH = PLUNGER_IN + RELEASE_TRAVEL   # key tip depth when latched: the slide's rail holds the plunger [ARITH 12.5]
#                                            and the retention pin holds the key [ARITH, below]
PUSH_TRAVEL = 0.040              # coin slide travel out of the lock's -y end [EST: measure]
# Key retention, after EP0427819B1 (src/ep0427819.txt): "the spherical locking member 18 drops downwards under the action
# of gravity, so that it occupies the apertures 19 and 21" (19 in the casing wall over the key, 21 an "elongate hole" in
# the key), "whereas the coupling member 8 is firmly locked by the locking member 18". Here: a pin in the casing over the
# key's path, sprung down; its +x face slopes so the key coming in lifts it, its -x face is upright so a key pulled back
# stops on it once the pin has dropped into the key's hole.
APER_D0, APER_D1, APER_W = 0.006, 0.014, 0.009   # key's hole: from the tip, along the key; across [CHOICE]
PIN_HX, PIN_HY = 0.0025, 0.001   # pin half sizes along x and y [CHOICE: a 5 x 2 mm pin for the patent's 7 mm ball]
# WHY the hole is 9 mm across and the pin 2 mm: the key has (SLOT_W - KEY_W) / 2 = 3 mm of play either side in the slot
# [ARITH], and the pin drops in whenever the key's centre line is within APER_W / 2 - PIN_HY = 3.5 mm of the slot's
# [ARITH], so any key pushed home anywhere inside the slot is caught, not only a centred one
PIN_X = FX - HOME_DEPTH + APER_D0 + PIN_HX   # [ARITH: the hole's tip-side wall meets the pin's upright face at home]
PIN_Z0 = SLOT_Z - SLOT_T / 2 + 0.0005   # pin's low edge at rest, lock z [CHOICE: 2.5 mm
#                                  into a key lying on the slot floor. WHY: with 1.25 mm the 1 mm contact offsets let the
#                                  chain's pull ride the hole's wall up the pin's corner (RUN, hash 666022d6: pin 1.0 mm
#                                  up after release)]
PIN_RAMP = 0.0045                # its sloped bottom rises this much toward +x [CHOICE: 0.5 mm above the slot top]
PIN_RISE = 0.0045                # pin travel up [ARITH: lifts its low edge 0.5 mm clear of a key at the slot top]
PIN_K, PIN_PRE = 50.0, 0.004     # N/m, preload [EST: 0.2 to 0.4 N down]
PLUNGER_K, PLUNGER_PRE = 150.0, 0.001      # N/m and preload stretch [EST: about 0.15 to 0.8 N]
SLIDE_K, SLIDE_PRE = 30.0, 0.030           # N/m, target 30 mm past the out stop [EST: 2.1 N in, 0.9 N out]
SLIDE_DAMP = 20.0                # N s/m [CHOICE: the slide comes out in about 0.3 s, not a flick]
# -- key and chain -------------------------------------------------------------------------------------------------------
KEY_L, KEY_W, KEY_T = 0.062, 0.015, 0.003  # [EST: length, width; thickness under half the patent's 7 mm ball, SOURCE]
KEY_EYE = 0.008                  # eye lug the chain hangs from [EST]
KEY_KG = 7900.0 * KEY_L * KEY_W * KEY_T    # [ARITH: stainless 7900 kg/m3, 22 g]
CHAIN_N, CHAIN_P = 11, 0.020     # links and pitch: 220 mm [CHOICE: the listing's chain is 260 mm; 40 mm shorter so the
#                                  key hangs clear of the bench at the lowered handle height]
CHAIN_R = 0.0015                 # link radius [EST]
CHAIN_KG = 0.0015                # per link [EST]
CHAIN_LIM = 1.40                 # rad, each link's bend either way [EST]
CHAIN_DAMP = 0.002               # N m s/rad per joint [CHOICE: a 30 mm swing of the key settles in about 2 s, RUN]
ANCHOR = (-(LOCK_W / 2) - 0.003, SLOT_Y, LOCK_Z0 + 0.004)   # chain leaves the lock's back face, low [CHOICE]
KEY_TOP = ANCHOR[2] - CHAIN_N * CHAIN_P - KEY_EYE          # lock frame [ARITH]
# -- quarter (usmint_quarter.txt) ----------------------------------------------------------------------------------------
COIN_D, COIN_T, COIN_KG = 0.02426, 0.00175, 0.00567   # [SOURCE: 0.955 in, 0.069 in, 5.670 g]
COIN_R = COIN_D / 2
SLIDE_FLOOR = 0.034              # coin slide floor top, lock z [ARITH: layout below]
COIN_X = -0.006                  # coin centre across the lock [CHOICE]
COIN_Y0 = -0.019                 # coin centre along the lock, slide in [CHOICE]
COIN_Z = SLIDE_FLOOR + COIN_R    # [ARITH]
MAT_XY, MAT = (0.060, -0.120), (0.100, 0.100, 0.003)  # rubber mat [CHOICE]

MATS = {"steel": (0.40, 0.30), "plastic": (0.35, 0.30), "inner": (0.15, 0.10), "rubber": (1.00, 0.90),
        "coin": (0.40, 0.30)}    # static / dynamic friction [EST]
TUBE_C, LOCK_C, RED_C, KEY_C, COIN_C, MAT_C = (0.75, 0.76, 0.78), (0.12, 0.12, 0.13), (0.75, 0.12, 0.10), \
    (0.82, 0.83, 0.85), (0.70, 0.70, 0.66), (0.10, 0.10, 0.10)
INNER_C = (0.85, 0.20, 0.15)


def _box(lo, hi, color, material="plastic", mass=0.0, collide=True, offset=0.002):
    c = tuple((np.add(lo, hi) / 2).tolist())
    s = tuple(np.subtract(hi, lo).tolist())
    return Shape("box", c, s, color, material=material, mass=mass, collide=collide, contact_offset=offset)


def _side_tube(y):
    """A side tube from the handle's end, forward and down to the bench: a 10-sided prism (hull) along the slope."""
    ring = [(TUBE_R * math.cos(2 * math.pi * k / 10), TUBE_R * math.sin(2 * math.pi * k / 10)) for k in range(10)]
    d = np.array([FRAME_RUN, 0.0, -H]) / math.hypot(FRAME_RUN, H)
    u = np.array([0.0, 1.0, 0.0])
    v = np.cross(d, u)
    ends = (np.array([0.0, y, 0.0]), np.array([FRAME_RUN, y, -H]))
    pts = [tuple((e + a * u + b * v).tolist()) for e in ends for a, b in ring]
    return hull(pts, TUBE_C, "steel")


def frame_shapes():
    """A trolley's handle: the 25 mm tube along y and its two side tubes down to the bench (lock frame)."""
    return [Shape("cylinder", (0.0, 0.0, 0.0), (TUBE_R, HANDLE_LEN + 2 * TUBE_R), TUBE_C, axis="Y", material="steel"),
            *(_side_tube(s * HANDLE_LEN / 2) for s in (-1, 1))]


# Near trolley's basket rear wall, lock frame: a wire grid hanging under the handle, just ahead of it, where the front
# lock's chain hangs into the nested basket. WHY: the brief asks for the structure that limits access; the hand must reach
# the key over this wall and under the front handle. Its top is BASKET_DROP under the handle axis so the key, turned
# level, still passes over it to the slot.
BASKET_X = 0.030                 # wall plane ahead of the handle axis [CHOICE: just clear of the 12.5 mm tube]
BASKET_DROP = 0.070              # top rail under the handle axis [EST: a basket's top rail sits under the handle]
BASKET_Z0 = -0.300               # wall's low edge, 40 mm over the bench [CHOICE: the basket floor]
BASKET_HY = HANDLE_LEN / 2 - 0.010   # wall half width, inside the side tubes [ARITH]
WIRE_R, WIRE_PITCH = 0.0025, 0.040   # 5 mm wire on a 40 mm grid [EST]


def basket_shapes():
    """Rear wall of the near trolley's basket: horizontal rails and upright wires (lock frame)."""
    top = -BASKET_DROP
    rails = [Shape("cylinder", (BASKET_X, 0.0, z), (WIRE_R, 2 * BASKET_HY), TUBE_C, axis="Y", material="steel")
             for z in (top, (top + BASKET_Z0) / 2, BASKET_Z0)]
    n = round(2 * BASKET_HY / WIRE_PITCH)   # [ARITH: 7 gaps, 8 wires]
    ys = [-BASKET_HY + k * (2 * BASKET_HY) / n for k in range(n + 1)]
    wires = [Shape("cylinder", (BASKET_X, y, (top + BASKET_Z0) / 2), (WIRE_R, top - BASKET_Z0), TUBE_C, axis="Z",
                   material="steel") for y in ys]
    return rails + wires


# Near lock casing, lock frame. Layers from the bottom: floor; key layer (slot from the front face, plunger behind);
# cover; coin-slide layer (the slide runs along y and leaves through the -y end); top.
Z_KEY0, Z_KEY1 = SLOT_Z - SLOT_T / 2, SLOT_Z + SLOT_T / 2   # [ARITH: 0.02175, 0.02625]
Z_COV1 = 0.031                   # cover top = coin slide floor bottom [CHOICE]
Z_SLD1 = 0.0745                  # coin-slide layer top [CHOICE: 16 mm over the coin]
SLD_X = 0.0175                   # slide channel half width [CHOICE]
SLD_Y1 = 0.037                   # slide channel end, lock y [CHOICE: 0.5 mm past the rail]
PL_X0, PL_X1 = FX - PLUNGER_IN - 0.017, FX - PLUNGER_IN    # plunger body along x [ARITH]
SY0, SY1 = SLOT_Y - SLOT_W / 2, SLOT_Y + SLOT_W / 2
HX, HY = LOCK_W / 2, LOCK_L / 2


def casing_shapes():
    c = LOCK_C
    return [_box((-HX, -HY, LOCK_Z0), (HX, HY, Z_KEY0), c),                                   # floor
            _box((-HX, -HY, Z_KEY0), (HX, SY0, Z_KEY1), c),                                   # key layer, -y of slot
            _box((-HX, SY1, Z_KEY0), (HX, HY, Z_KEY1), c),                                    # key layer, +y of slot
            _box((-HX, SY0, Z_KEY0), (PL_X0 - PLUNGER_STOP - 0.001, SY1, Z_KEY1), c),         # behind the plunger
            _box((-HX, -HY, Z_KEY1), (HX, HY, Z_COV1), c),                                    # cover
            _box((-HX, -HY, Z_COV1), (-SLD_X, HY, Z_SLD1), c),                                # slide layer walls
            _box((SLD_X, -HY, Z_COV1), (HX, HY, Z_SLD1), c),
            _box((-SLD_X, SLD_Y1, Z_COV1), (SLD_X, HY, Z_SLD1), c),
            _box((-HX, -HY, Z_SLD1), (HX, HY, LOCK_Z1), RED_C)]                               # top (red cap)


# Plunger (link): the key's tip meets its face; tooth T reaches up into the slide layer. Coin slide (link): a floor, the
# coin's pocket (two walls on its faces) and lug L with a rail behind it. The slide's spring presses L (-y) on T; the
# key pushing the plunger RELEASE_TRAVEL in (-x) takes T past L, the spring pushes the slide out PUSH_TRAVEL, and the
# rail then lies across T's way back: the plunger stays HOME_DEPTH in. The key rests in the slot on its own friction
# (on a real lock the patent's ball also holds the key).
T_X = (PL_X1 - 0.0225, PL_X1 - 0.0145)   # tooth across [CHOICE: clear of the coin pocket when pressed to its stop]
T_Y = (-0.012, -0.006)
L_X = (T_X[1] - RELEASE_TRAVEL, T_X[1] - RELEASE_TRAVEL + 0.008)
L_Y = (T_Y[1] + 0.0005, T_Y[1] + 0.0005 + PUSH_TRAVEL + 0.002)   # lug and rail as one block [ARITH: covers T after the
#                                                                   slide's full travel]
WALL_H, WALL_T, POCKET = 0.0025, 0.0015, 0.00325   # pocket wall height, thickness, gap [CHOICE: 0.75 mm play a face]
HOLE = (T_X[0] - PLUNGER_STOP - 0.001, T_X[1] + 0.001, T_Y[0] - 0.001, T_Y[1] + PUSH_TRAVEL + 0.0015)   # [ARITH]


def plunger_shapes():
    i = INNER_C
    return [_box((PL_X0, SY0 + 0.0005, Z_KEY0 + 0.0005), (PL_X1, SY1 - 0.0005, Z_KEY1 - 0.0005), i, "inner"),
            _box((T_X[0], T_Y[0], Z_KEY1 - 0.0005), (T_X[1], T_Y[1], 0.0415), i, "inner")]


def slide_shapes():
    i = INNER_C
    f0 = Z_COV1
    w_lo, w_hi = COIN_X - POCKET / 2, COIN_X + POCKET / 2
    s = math.sqrt(COIN_R ** 2 - (COIN_R - WALL_H - 0.0004) ** 2)   # [ARITH: end stops 0.4 mm under the rim, 0.5 mm play]
    cy0, cy1 = COIN_Y0 - s, COIN_Y0 + s
    x0, x1, y1 = -SLD_X + 0.0005, SLD_X - 0.0005, L_Y[1]
    hx0, hx1, hy0, hy1 = HOLE   # T comes up through this hole; it runs on in y for the slide's travel
    floor = [_box((x0, -HY, f0), (x1, hy0, SLIDE_FLOOR), i, "inner"),
             _box((x0, hy1, f0), (x1, y1, SLIDE_FLOOR), i, "inner"),
             _box((x0, hy0, f0), (hx0, hy1, SLIDE_FLOOR), i, "inner"),
             _box((hx1, hy0, f0), (x1, hy1, SLIDE_FLOOR), i, "inner")]
    return floor + [
        _box((w_lo - WALL_T, cy0, SLIDE_FLOOR), (w_lo, cy1, SLIDE_FLOOR + WALL_H), i, "inner"),
        _box((w_hi, cy0, SLIDE_FLOOR), (w_hi + WALL_T, cy1, SLIDE_FLOOR + WALL_H), i, "inner"),
        _box((w_lo, cy0 - WALL_T, SLIDE_FLOOR), (w_hi, cy0, SLIDE_FLOOR + WALL_H), i, "inner"),   # end stops: the
        _box((w_lo, cy1, SLIDE_FLOOR), (w_hi, cy1 + WALL_T, SLIDE_FLOOR + WALL_H), i, "inner"),   # quarter cannot roll
        _box((L_X[0], L_Y[0], SLIDE_FLOOR), (L_X[1], L_Y[1], 0.042), i, "inner")]


def pin_shapes():
    """Retention pin, lock frame: upright -x face, bottom sloping up toward +x, flat top."""
    x0, x1, y0, y1 = PIN_X - PIN_HX, PIN_X + PIN_HX, SLOT_Y - PIN_HY, SLOT_Y + PIN_HY
    top = PIN_Z0 + 0.006
    bot = [(x0, y0, PIN_Z0), (x1, y0, PIN_Z0 + PIN_RAMP), (x1, y1, PIN_Z0 + PIN_RAMP), (x0, y1, PIN_Z0)]
    up = [(x0, y0, top), (x1, y0, top), (x1, y1, top), (x0, y1, top)]
    return [hull(bot + up, INNER_C, "inner", contact_offset=0.001)]


def chain_parts(prefix):
    """(links, joints) of a chain hanging from ANCHOR and its key: alternate hinge axes so it bends both ways [CHOICE:
    one rigid capsule per link, no twist]; the key hangs flat across x, its width along y, tip down."""
    ax, ay, az = ANCHOR
    links, joints, parent = [], [], ""
    for k in range(CHAIN_N):
        z0 = az - k * CHAIN_P
        name = f"{prefix}{k + 1}"
        links.append(Link(name, CHAIN_KG, (Shape("capsule", (ax, ay, z0 - CHAIN_P / 2), (CHAIN_R, CHAIN_P - 2 * CHAIN_R),
                                                 KEY_C, axis="Z", material="steel", contact_offset=0.001),)))
        joints.append(Joint(f"{name}_j", "revolute", name, (ax, ay, z0), "X" if k % 2 == 0 else "Y", -CHAIN_LIM,
                            CHAIN_LIM, damping=CHAIN_DAMP, parent=parent))
        parent = name
    zk = az - CHAIN_N * CHAIN_P
    t0 = KEY_TOP - KEY_L                                   # tip
    hx, hy = KEY_T / 2, KEY_W / 2
    tipring = [(ax - hx * 0.6, ay - hy * 0.6, t0), (ax + hx * 0.6, ay - hy * 0.6, t0), (ax + hx * 0.6, ay + hy * 0.6, t0),
               (ax - hx * 0.6, ay + hy * 0.6, t0)]
    upring = [(ax - hx, ay - hy, t0 + 0.004), (ax + hx, ay - hy, t0 + 0.004), (ax + hx, ay + hy, t0 + 0.004),
              (ax - hx, ay + hy, t0 + 0.004)]
    aw = APER_W / 2
    key = Link(f"{prefix}key", KEY_KG, (
        _box((ax - hx, ay - hy, t0 + 0.004), (ax + hx, ay + hy, t0 + APER_D0), KEY_C, "steel", offset=0.001),
        _box((ax - hx, ay - hy, t0 + APER_D0), (ax + hx, ay - aw, t0 + APER_D1), KEY_C, "steel", offset=0.001),
        _box((ax - hx, ay + aw, t0 + APER_D0), (ax + hx, ay + hy, t0 + APER_D1), KEY_C, "steel", offset=0.001),
        _box((ax - hx, ay - hy, t0 + APER_D1), (ax + hx, ay + hy, KEY_TOP), KEY_C, "steel", offset=0.001),
        hull(tipring + upring, KEY_C, "steel", contact_offset=0.001),
        _box((ax - hx, ay - 0.004, KEY_TOP), (ax + hx, ay + 0.004, zk), RED_C, "steel", offset=0.001)))
    links.append(key)
    joints.append(Joint(f"{prefix}key_j", "revolute", f"{prefix}key", (ax, ay, zk), "Y", -1.75, 1.75, damping=CHAIN_DAMP,
                        parent=parent))
    return links, joints


_lock_body = _box((-HX, -HY, LOCK_Z0), (HX, HY, Z_SLD1), LOCK_C)
_lock_cap = _box((-HX, -HY, Z_SLD1), (HX, HY, LOCK_Z1), RED_C)
_front_links, _front_joints = chain_parts("c")
FRONT = mechanism("front", (XF, YL, TOP + H), Link("front_frame", 6.0, tuple(frame_shapes()) + (_lock_body, _lock_cap)),
                  _front_links, _front_joints, MATS, color=LOCK_C, self_collide=False)
NEAR = mechanism(
    "near", (XN, YL, TOP + H), Link("near_frame", 6.0, tuple(frame_shapes() + basket_shapes() + casing_shapes())),
    [Link("plunger", 0.004, tuple(plunger_shapes())), Link("slide", 0.012, tuple(slide_shapes())),
     Link("pin", 0.001, tuple(pin_shapes()))],
    [Joint("plunger_j", "prismatic", "plunger", (0.0, 0.0, 0.0), "X", -PLUNGER_STOP, 0.0, stiffness=PLUNGER_K,
           damping=1.0, target=PLUNGER_PRE),
     Joint("slide_j", "prismatic", "slide", (0.0, 0.0, 0.0), "Y", -PUSH_TRAVEL, 0.0, stiffness=SLIDE_K,
           damping=SLIDE_DAMP, target=-(PUSH_TRAVEL + SLIDE_PRE)),
     Joint("pin_j", "prismatic", "pin", (0.0, 0.0, 0.0), "Z", 0.0, PIN_RISE, stiffness=PIN_K, damping=0.5,
           target=-PIN_PRE)],
    MATS, color=LOCK_C, self_collide=True)
_own_links, _own_joints = chain_parts("n")
NEAR_CHAIN = mechanism("near_chain", (XN, YL, TOP + H),
                       Link("near_anchor", 0.01, (_box((ANCHOR[0] - 0.002, ANCHOR[1] - 0.003, ANCHOR[2]),
                                                       (ANCHOR[0] + 0.002, ANCHOR[1] + 0.003, LOCK_Z0 + 0.008),
                                                       LOCK_C, collide=False),)),
                       _own_links, _own_joints, MATS, color=LOCK_C, self_collide=False)
COIN_START = (XN + COIN_X, YL + COIN_Y0, TOP + H + COIN_Z + 0.0003)
QUARTER = body("quarter", COIN_START, [Shape("cylinder", (0.0, 0.0, 0.0), (COIN_R, COIN_T), COIN_C, axis="X",
                                             material="coin", mass=COIN_KG, contact_offset=0.001)], MATS, color=COIN_C)
MAT_BOX = Box("mat", (MAT_XY[0], MAT_XY[1], TOP + MAT[2] / 2), MAT_C, None, 0.0, MAT)

OBJECTS = [FRONT, NEAR, NEAR_CHAIN, QUARTER, MAT_BOX]

# -- scoring --------------------------------------------------------------------------------------------------------------
KEY_TIP = (ANCHOR[0], ANCHOR[1], KEY_TOP - KEY_L)   # key points in the front mechanism's rest frame [ARITH]
KEY_BACK = (ANCHOR[0], ANCHOR[1], KEY_TOP)
ENTER = 0.005                    # "in the slot": tip 5 mm or more past the mouth [CHOICE]
MAX_IN = 0.020                   # and no more than 20 mm: the plunger stops the tip at 14 mm [ARITH]. WHY: without it a
#                                  key held in line behind the lock's back face read as deep in the slot
LINE_Y, LINE_Z, LINE_DEG = 0.004, 0.0025, 15.0   # tip within these of the slot's centre line, key within 15 deg of it
SLIDE_OUT = 0.035                # coin slide out 35 mm or more of its 40 [CHOICE]
MAT_IN, MAT_REST = COIN_R + 0.002, 0.003  # the whole coin on the mat: centre a radius plus 2 mm inside each edge
#                                  (35.9 mm of the 50 mm half-width) [CHOICE]; its lowest point within 3 mm of the mat top.
#                                  WHY: a centre-only rule counted a quarter hanging 7 mm off the mat
COIN_FLAT_DEG = 15.0             # "laid": the coin's faces within 15 degrees of level, so one standing on edge fails [CHOICE]
HOME_TOL = 0.001                 # "home": tip HOME_DEPTH - 1 mm or more [CHOICE]
PIN_DOWN = Z_KEY0 + KEY_T - PIN_Z0 - 0.001   # "pin down": its upright face still overlaps a key on the slot floor by
#                                  1 mm or more, so the key's hole wall stops on it [ARITH: 1.5 mm up from rest]; riding
#                                  on solid key it stands 2.5 mm up. WHY not 1 mm: under the chain's pull the seated pin
#                                  stands about 1 mm up with the key still home (RUN, hash 18d431ec: 0.6 to 1.0 mm)
CLEAR = 0.030                    # every hand point 30 mm from the key and the quarter [CHOICE]
COIN_FREE = 0.010                # 0.75 also needs every hand point 10 mm from the quarter [CHOICE]. WHY: a flat quarter
#                                  still in the gripper 2.9 mm over the mat read as lying on it
STILL_V, STILL_W = 0.01, 0.5     # m/s, rad/s


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def key_points(front):
    """World (tip, back) of the chain key."""
    st = front.links["ckey"]
    r, p = _rot(st.quat_xyzw), np.asarray(st.pos, float)
    return tuple(p + r @ np.asarray(v, float) for v in (KEY_TIP, KEY_BACK))


def key_in_slot(objects):
    """(depth of the key tip past the near lock's mouth in metres, in line and at most MAX_IN deep)."""
    tip, back = key_points(objects["front"])
    o = np.asarray(objects["near"].pos, float)
    t = tip - o
    d = tip - back
    d = d / max(np.linalg.norm(d), 1e-9)
    in_line = (abs(t[1] - SLOT_Y) <= LINE_Y and abs(t[2] - SLOT_Z) <= LINE_Z
               and -d[0] >= math.cos(math.radians(LINE_DEG)))
    depth = FX - t[0]
    return depth, bool(in_line and depth <= MAX_IN)


def retained(objects, depth):
    """The lock holds the key: its tip home (HOME_DEPTH - 1 mm or more) and the pin down in the key's hole. A key held
    short by the gripper, or with the pin riding on it, is not retained."""
    return depth >= HOME_DEPTH - HOME_TOL and objects["near"].joints["pin_j"] <= PIN_DOWN


def coin_low_z(st):
    """Lowest point of the quarter's rim (world z)."""
    ax = _rot(st.quat_xyzw)[:, 0]
    return float(st.pos[2]) - COIN_R * math.sqrt(max(0.0, 1.0 - ax[2] ** 2)) - COIN_T / 2 * abs(ax[2])


def coin_on_mat(objects):
    st = objects["quarter"]
    c = np.asarray(st.pos, float)
    inside = abs(c[0] - MAT_XY[0]) <= MAT[0] / 2 - MAT_IN and abs(c[1] - MAT_XY[1]) <= MAT[1] / 2 - MAT_IN
    flat = abs(float(_rot(st.quat_xyzw)[2, 0])) >= math.cos(math.radians(COIN_FLAT_DEG))   # face normal (body x) up
    return bool(inside and flat and abs(coin_low_z(st) - (TOP + MAT[2])) <= MAT_REST)


_HANDS = {"left": None, "right": None}


def set_hands(points):
    _HANDS.update(points)


@on_reset
def _reset():
    _HANDS.update({"left": None, "right": None})


@on_step
def read_hands(env):
    try:
        _HANDS.update({side: HAND.from_env(env, side) for side in ("left", "right")})
    except Exception:  # noqa: BLE001  WHY: never stop the run; unread hands score 0
        _HANDS.update({"left": None, "right": None})


def hands_read():
    return all(v is not None for v in _HANDS.values())


def hands_off_coin(objects):
    q = objects["quarter"]
    for pts in _HANDS.values():
        if pts is None or box_distance(np.asarray(pts, float), q.low, q.high).min() < COIN_FREE:
            return False
    return True


def hands_clear(objects):
    tip, back = key_points(objects["front"])
    key_lo, key_hi = np.minimum(tip, back) - KEY_W / 2, np.maximum(tip, back) + KEY_W / 2
    q = objects["quarter"]
    for pts in _HANDS.values():
        if pts is None:
            return False
        p = np.asarray(pts, float)
        if box_distance(p, key_lo, key_hi).min() < CLEAR or box_distance(p, q.low, q.high).min() < CLEAR:
            return False
    return True


_WHY = dict.fromkeys(("quarter", "ckey", "plunger", "slide", "clear", "key_v", "key_w"), 0)   # trace only: last 1.0 checks


def still(objects):
    ok = True
    _WHY["quarter"] = objects["quarter"].resting(STILL_V, STILL_W)
    ok &= _WHY["quarter"]
    _WHY["ckey"] = key_resting(objects["front"])
    ok &= _WHY["ckey"]
    for n in ("plunger", "slide"):
        ln = objects["near"].links.get(n)
        _WHY[n] = ln is not None and ln.resting(STILL_V, STILL_W)
        ok &= _WHY[n]
    return bool(ok)


def key_resting(front):
    """The key's tip and back end under STILL_V and its spin under STILL_W. WHY: a chain link's velocity is read at its
    frame origin, which for the key lies in the chain's rest frame about 0.3 m away, so a small wobble of the held key
    reads there as motion; v + w x r gives the key's own points [ARITH]."""
    ln = front.links.get("ckey")
    if ln is None:
        return False
    w, v, p = np.asarray(ln.ang_vel, float), np.asarray(ln.lin_vel, float), np.asarray(ln.pos, float)
    _WHY["key_w"] = float(np.linalg.norm(w))
    speeds = [float(np.linalg.norm(v + np.cross(w, q - p))) for q in key_points(front)]
    _WHY["key_v"] = max(speeds)
    return bool(max(speeds) < STILL_V and np.linalg.norm(w) < STILL_W)


def stage(objects):
    """0 none (or hands unread); 1 key in the slot, in line, 5 mm in; 2 and the coin slide out and the key retained (home,
    pin down in its hole); 3 and the quarter resting on the mat, no hand within 10 mm of it; 4 and all still, both hands 30 mm from the key and the
    quarter."""
    if not hands_read():
        return 0
    depth, in_line = key_in_slot(objects)
    if not (in_line and depth >= ENTER):
        return 0
    # WHY: the slide comes out only when the key takes the plunger HOME_DEPTH in (a finger reaches short of that); the
    # pin rule makes the lock, not the hand, hold the key
    if objects["near"].joints["slide_j"] > -SLIDE_OUT or not retained(objects, depth):
        return 1
    if not (coin_on_mat(objects) and hands_off_coin(objects)):
        return 2
    st, cl = still(objects), hands_clear(objects)
    _WHY["clear"] = cl
    if not (st and cl):
        return 3
    return 4


SCORES = (0.0, 0.25, 0.5, 0.75, 1.0)


def score(objects):
    """0.25 key in the slot; 0.5 key home and the coin slide out; 0.75 and the quarter on the mat. 1.0 comes only from
    success() held SUCCESS_HOLD_S."""
    # WHY capped at 0.75: SCORE_OVER "end" reads score() at the last step only, so a state reached on the last step
    # must not score the full 1.0 the task text reserves for a state held 1 s
    return SCORES[min(stage(objects), 3)]


def success(objects):
    return stage(objects) == 4


def distance(objects):
    return 1.0 - score(objects)


def trace_row(env):
    """fastrun --trace columns: lock joints, key tip, quarter."""
    out = {}
    try:
        sc = env.unwrapped.scene
        o = getattr(sc.env_origins, "torch", sc.env_origins)[0].detach().cpu().numpy()[:3]
        a = sc["near"]
        q = getattr(a.data.joint_pos, "torch", a.data.joint_pos)[0].detach().cpu().numpy()
        jn = list(a.joint_names)
        out.update({f"ok_{k}": round(float(v), 4) for k, v in _WHY.items()})
        for n in ("plunger_j", "slide_j", "pin_j"):
            out[n] = float(q[jn.index(n)])
        f = sc["front"]
        poses = getattr(f.data.body_link_pose_w, "torch", f.data.body_link_pose_w)[0].detach().cpu().numpy()
        i = list(f.body_names).index("ckey")
        p, r = poses[i, :3] - o, _rot(poses[i, 3:7])
        tip = p + r @ np.asarray(KEY_TIP)
        out.update({"tip_x": float(tip[0]), "tip_y": float(tip[1]), "tip_z": float(tip[2])})
        c = getattr(sc["quarter"].data.root_link_pose_w, "torch", sc["quarter"].data.root_link_pose_w)
        c = c[0].detach().cpu().numpy()[:3] - o
        out.update({"q_x": float(c[0]), "q_y": float(c[1]), "q_z": float(c[2])})
    except Exception as e:  # noqa: BLE001
        out["trace_err"] = str(e)[:60]
    return out


# -- HD recording for the demo video (only with TROLLEY_HD set; Dream and scoring runs never set it) -------------------
class _CamRecorder:
    """Each control step, the full-size render of every camera to <TROLLEY_HD>/<cam>.mp4 (H.264)."""

    def __init__(self):
        self.w = {}

    def add(self, env):
        out_dir = os.environ.get("TROLLEY_HD")
        if not out_dir:
            return
        try:
            import imageio_ffmpeg
            sc = env.unwrapped.scene
            for cam in ("top_cam", "left_cam", "right_cam"):
                rgb = sc[cam].data.output["rgb"]
                rgb = getattr(rgb, "torch", rgb)
                frame = np.ascontiguousarray(rgb[0, ..., :3].cpu().numpy(), dtype=np.uint8)
                if cam not in self.w:
                    os.makedirs(out_dir, exist_ok=True)
                    h, w = frame.shape[:2]
                    import imageio.v2 as imageio
                    imageio.imwrite(os.path.join(out_dir, f"{cam}_first.png"), frame)
                    wr = imageio_ffmpeg.write_frames(os.path.join(out_dir, f"{cam}.mp4"), (w, h), pix_fmt_in="rgb24",
                                                     pix_fmt_out="yuv420p", fps=20, codec="libx264",
                                                     macro_block_size=8, output_params=["-crf", "18"])
                    wr.send(None)
                    self.w[cam] = wr
                self.w[cam].send(frame)
        except Exception as e:  # noqa: BLE001  WHY: a recorder fault must never stop or change a trial
            if "err" not in self.w:
                self.w["err"] = e
                print(f"[trolley] HD recorder off: {e}", flush=True)

    def close(self):
        for k, wr in list(self.w.items()):
            if k != "err":
                try:
                    wr.close()
                except Exception:  # noqa: BLE001
                    pass
        self.w = {}


_REC = _CamRecorder()
if os.environ.get("TROLLEY_HD"):
    import atexit
    atexit.register(_REC.close)
    # WHY: render the cameras at 1280 x 720 for the film only; set before the embodiment loads (see fork-jaw notes)
    if "yam_arena.embodiment" not in sys.modules:
        import yam_arena.constants as _yc
        _yc.CAM_RENDER_HEIGHT, _yc.CAM_RENDER_WIDTH = 720, 1280


@on_step
def _record(env):
    _REC.add(env)


# -- demo (motionkit; scene constants and the arms' own joint readings only) ----------------------------------------------
install(SagCorrector())


def _rx(deg):
    return axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(deg))


def _ry(deg):
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(deg))


G_KEY = level(90.0)              # fingers level toward +y, closing along x: the palm stays clear of the chain above
G_TURN = as_grip(_ry(90.0) @ G_KEY.rot)   # rolled 90 deg about the finger axis: key tip toward -x, key flat
G_COIN = tilted(60.0, 90.0)      # fingers 60 deg off vertical toward +y, closing along x on the quarter's faces
G_PLACE = down(90.0)
LAY_DEG = -55.0                  # the hand tips the quarter this far from upright before letting go, so it falls flat
#                                  instead of rolling away on edge [CHOICE; RUN: set down upright it rolled, hash 5f577f54]
G_LAY = as_grip(_ry(LAY_DEG) @ G_PLACE.rot)
KEY_X, KEY_Y = XF + ANCHOR[0], YL + ANCHOR[1]          # hanging key's line, world [ARITH]
GRIP_DOWN = 0.022                # TCP this far below the key's top: fingers clear of the eye [CHOICE]
GRIP_Z = TOP + H + KEY_TOP - GRIP_DOWN
TIP_FROM_TCP = KEY_L - GRIP_DOWN  # [ARITH: 0.040]
KEY_GRIP_F = 6.0                 # N a finger on the key's faces [CHOICE]
MOUTH_X = XN + FX                # near lock's front face, world x [ARITH]
SLOT_WY, SLOT_WZ = YL + SLOT_Y, TOP + H + SLOT_Z
PUSH_PAST = 0.008                # key sent this far past the plunger's stop [CHOICE: arm compliance; 3 mm left it 0.6 mm short]
KEY_DY = -0.0028                 # aim offset: after the turn the key sits 2.8 mm +y in the grip [RUN: trace, hash 54938662]
COIN_OUT = (XN + COIN_X, YL + COIN_Y0 - PUSH_TRAVEL, TOP + H + COIN_Z)   # quarter with the slide out [ARITH]
COIN_ABOVE = 0.007               # TCP this far over the quarter's centre [CHOICE: pads on its top half, clear of walls]
COIN_GRIP_F = 4.0                # N a finger [CHOICE]
AWAY = (0.15, -0.25, TOP + 0.18)  # right hand parks here [CHOICE]


def _key_take(R):
    R.move((KEY_X - 0.03, KEY_Y - 0.10, GRIP_Z + 0.10), grip=G_KEY, finger=-0.015, label="key: over")
    R.move((KEY_X, KEY_Y - 0.08, GRIP_Z), label="key: line up")
    R.hold(0.5, "key: steady")
    R.move((KEY_X, KEY_Y - 0.002, GRIP_Z), t=2.0, label="key: in")
    R.hold(0.4, "key: settle")
    R.move(R.pos, t=0.8, finger=finger_for(KEY_T, KEY_GRIP_F), label="key: pinch", contact=True)
    R.hold(0.3, "key: hold", contact=True)


def _key_turn(R):
    R.move((KEY_X - 0.02, KEY_Y, GRIP_Z + 0.05), t=1.5, label="key: lift")
    R.move((KEY_X - 0.04, SLOT_WY, SLOT_WZ - 0.04), grip=G_TURN, t=2.5, label="key: turn")


def _key_insert(R, depth, keep=False):
    x0 = MOUTH_X + 0.010 + TIP_FROM_TCP
    R.move((x0, SLOT_WY + KEY_DY, SLOT_WZ), t=2.0, label="key: line up at slot")
    R.hold(0.6, "key: steady at slot")
    R.move((MOUTH_X - depth + TIP_FROM_TCP, SLOT_WY + KEY_DY, SLOT_WZ), t=max(2.0, (depth + 0.01) / 0.012),
           label="key: push", contact=True)
    R.hold(0.8, "key: hold in", contact=True)
    if keep:                     # partway: the hand keeps the key where it stopped (no retention short of home)
        R.hold(3.0, "key: keep holding", contact=True)
        return
    R.move(R.pos + np.array([PUSH_PAST + 0.002, 0.0, 0.0]), t=0.6, label="key: ease")   # WHY: back to where the stop
    #                                                                       really holds the hand before letting go
    R.move(R.pos, t=0.8, finger=FINGER_OPEN, label="key: let go", contact=True)
    RELEASE["right"].add("key: let go")
    # WHY: fully open, the lower finger sits under the chain's first link, which hangs from the eye next to the fingers'
    # back edge; the hand then leaves along its fingers (-y), so they slide off the key's side
    R.move(R.pos + np.array([0.0, -0.07, 0.0]), t=2.0, label="key: back out")


def _coin_take(R):
    cx, cy, cz = COIN_OUT
    a = G_COIN.rot[:, 2]
    tcp = np.array([cx, cy - 0.004, cz + COIN_ABOVE])   # WHY: 4 mm off centre, away from the casing's end face
    R.move(tcp - 0.05 * a + np.array([0.0, 0.0, 0.02]), grip=G_COIN, finger=-0.015, label="coin: over")
    R.hold(0.5, "coin: steady")
    R.move(tcp, t=1.8, label="coin: in")
    R.hold(0.3, "coin: settle")
    R.move(R.pos, t=0.8, finger=finger_for(COIN_T, COIN_GRIP_F), label="coin: pinch", contact=True)
    R.hold(0.3, "coin: hold", contact=True)
    R.move(R.pos + np.array([0.0, 0.0, 0.04]), t=1.2, label="coin: lift")
    return tcp


def _coin_lay(R, tcp, xy, drop=False, hold=False):
    # The quarter's centre and face normal turn with the grip from G_COIN to G_LAY; its lowest rim point sets the height
    g = G_PLACE if drop else G_LAY      # drop_coin lets go upright beside the mat (the tipped grip does not reach there)
    turn = g.rot @ G_COIN.rot.T
    rel, n = turn @ (np.asarray(COIN_OUT) - tcp), turn @ np.array([1.0, 0.0, 0.0])
    low = COIN_R * math.sqrt(max(0.0, 1.0 - n[2] ** 2)) + COIN_T / 2 * abs(n[2])   # [ARITH]
    z_set = TOP + MAT[2] + low + (0.040 if drop else 0.030 if hold else 0.015) - rel[2]   # [CHOICE: 15 mm keeps the
    #                                                                     tipped fingertips off the mat]
    R.move((xy[0], R.pos[1], R.pos[2] - 0.08), t=1.5, label="coin: back")   # WHY: over the near handle first, then down
    R.move((xy[0] - rel[0], xy[1] - rel[1], z_set + 0.08), grip=g, t=2.0, label="coin: carry")
    R.move((xy[0] - rel[0], xy[1] - rel[1], z_set), t=1.5, label="coin: down")
    if hold:
        R.hold(3.0, "coin: keep holding")
        return
    R.move(R.pos, t=0.6, finger=-0.012, label="coin: let go")
    RELEASE["right"].add("coin: let go")
    R.move(R.pos + np.array([0.0, 0.0, 0.08]), t=1.2, label="coin: up")


def plan(seed=None, mode_name="demo"):
    if mode_name == "nothing":
        return {}
    R = Plan("right")
    if mode_name == "pry_coin":
        # The lazy shortcut: no key; the closed fingers push at the coin slide's face and along the casing's -y end
        cx, cy, cz = XN + COIN_X, YL - LOCK_L / 2 - 0.012, TOP + H + COIN_Z
        R.move((cx, cy - 0.06, cz + 0.03), grip=G_COIN, finger=0.0, label="pry: over")
        R.move((cx, cy, cz), t=1.5, label="pry: at slide")
        R.move((cx, cy + 0.008, cz), t=1.0, label="pry: push", contact=True)
        R.move((cx, cy - 0.02, cz + 0.01), t=1.0, label="pry: drag", contact=True)
        R.move((cx, cy - 0.06, cz + 0.04), t=1.0, label="pry: back")
        return plans_out(R)
    elif mode_name == "no_key_coin":
        # Skip the key and reach for the quarter where the slide would bring it: nothing is there to take
        cx, cy, cz = COIN_OUT
        a = G_COIN.rot[:, 2]
        tcp = np.array([cx, cy - 0.008, cz + COIN_ABOVE + 0.01])
        R.move(tcp - 0.05 * a + np.array([0.0, 0.0, 0.02]), grip=G_COIN, finger=-0.015, label="reach: over")
        R.move(tcp, t=1.8, label="reach: in")
        R.move(R.pos, t=0.8, finger=finger_for(COIN_T, COIN_GRIP_F), label="reach: close")
        R.hold(1.0, "reach: wait")
        return plans_out(R)
    else:
        _key_take(R)
        _key_turn(R)
        if mode_name == "partway":
            _key_insert(R, PLUNGER_IN + 0.002, keep=True)
            return plans_out(R)
        else:
            _key_insert(R, PLUNGER_IN + PLUNGER_STOP + PUSH_PAST)
            if mode_name != "key_only":
                tcp = _coin_take(R)
                if mode_name == "drop_coin":
                    _coin_lay(R, tcp, (MAT_XY[0], MAT_XY[1] - 0.09), drop=True)
                elif mode_name == "hold_coin":
                    _coin_lay(R, tcp, MAT_XY, hold=True)
                    return plans_out(R)
                else:
                    _coin_lay(R, tcp, MAT_XY)
                return plans_out(R)     # WHY: from the mat, the way to AWAY swings the forearm into the near trolley
    R.move(AWAY, grip=down(0.0), label="away")
    return plans_out(R)


# WHY near during the push: the plan aims the key PUSH_PAST beyond the plunger's stop to press hard; the stop holds the
# tip at 14 mm, so the commanded fingertips (6 mm nearer the casing than that) are never reached
CHECK_SKIP = {("right", "ckey"): ("key:",), ("right", "quarter"): ("coin:",),
              ("right", "near"): ("pry:", "key: push", "key: hold in", "key: ease")}


def check_parts(seed=None, mode_name="demo"):
    """checkkit parts, one per shape group, so touches by design can be skipped by name."""
    out = []
    for mech in (FRONT, NEAR, NEAR_CHAIN):
        o = np.asarray(mech.pos, float)
        for link in mech.all_links():
            for s in link.shapes:
                if not s.collide:
                    continue
                from checkkit import shape_points
                name = "ckey" if link.name.startswith("c") and link.name != "front_frame" else mech.name
                out.append((name, shape_points(s) + o))
    lo, hi = np.asarray(QUARTER.pos) - (COIN_T / 2, COIN_R, COIN_R), np.asarray(QUARTER.pos) + (COIN_T / 2, COIN_R, COIN_R)
    out.append(("quarter", np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])))
    m = np.asarray(MAT_BOX.pos), np.asarray(MAT) / 2
    out.append(("mat", np.array([[m[0][0] + sx * m[1][0], m[0][1] + sy * m[1][1], m[0][2] + sz * m[1][2]]
                                 for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])))
    return out


def demo():
    return plan(run_seed(), mode())
