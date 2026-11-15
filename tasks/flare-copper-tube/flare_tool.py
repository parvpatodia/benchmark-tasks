"""A short 3/8 in soft copper tube stands in the 3/8 opening of a precision flare tool's closed flaring bar, its
end a little above the bar's top. The tool rests on two blocks; its yoke rests near the bar's hinged end (where along
the bar varies), and its clamp screw and feed screw are loose. Flare the tube's end the way the tool's maker says: set the tube's end even with
the top of the bar, slide the yoke over the tube with its arrow on the 3/8 line, tighten the clamp screw, then turn the
feed screw handle clockwise (seen from above) until the feed screw's red top, standing up through the handle's hub,
stops going down (the cone's clutch has released), then at least two more full turns. Then turn the handle back until the cone's tip is at least 2 mm over the bar's top (2.25 turns back from full
depth), and move both
grippers away from the tool and the tube.
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, body, hull, mechanism, on_reset, on_step, run_env, run_seed  # noqa: E402
from yam_arena import station  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
RUN_LIMIT_STEPS = 3000           # 150 s at 20 Hz: the demo takes about 116 s
TOP = TABLE_TOP[SCENE]
D2R = math.pi / 180
IN = 0.0254
TURN = 2 * math.pi

# -- sources (src/): RIDGID EC42865 "Precision Flare Tools Instructions" (steps 1-10, weights, max wall 0.035 in);
#    CDA Copper Tube Handbook Sec. VIII (flared joints; soft temper; tube sizes); RIDGID's own product photo
#    (src/img/r6.jpg) for the tool's proportions, scaled by the 7-1/2 in overall length that distributors list for the
#    458R (search result, not a RIDGID document). Every size not from those is [EST] or [CHOICE].
TUBE_OD = 0.375 * IN              # [SOURCE: CDA Table, 1/4 nominal Type L, OD .375 in]
TUBE_WALL = 0.030 * IN            # [SOURCE: CDA Table, Type L .030 in; under the tool's 0.035 in maximum]
TUBE_L = 0.075                    # an offcut [CHOICE]: short enough to hang in the bar clear of the table
TUBE_KG = 0.014                   # copper 8.96 g/cm3 x the wall's section x 75 mm [ARITH]

BAR_L = 7.5 * IN                  # 190.5 mm [SOURCE: distributors' 7-1/2 in OAL for the 458R]
BAR_H = 0.024                     # the bar's height [EST from the photo: about 1/8 of its length]
HALF_W = 0.0125                   # each half's width across the split [EST]
BX, BY = 0.240, 0.0               # the bar's middle: the split line runs along y at x = BX [CHOICE of layout]
Z_BAR = TOP + 0.085               # the bar's top. WHY so high: the tube hangs below the bar, clear of the table, and the
#                                   feed handle stays under the arm's reach (tools/reach.py: TOP + 0.17 m reached at x
#                                   0.20-0.28 by the left arm)
HINGE_Y = BY - BAR_L / 2          # the hinge pin at the bar's -y end, on the split line; the free end is at +y
SIZES_IN = (0.125, 0.1875, 0.25, 0.3125, 0.375, 0.5, 0.625, 0.75)   # the openings [SOURCE: EC42865, 458R sizes]
HOLE_CLEAR = 0.00015              # radial play round a tube across the split (y) [CHOICE]
HOLE_SINK = 0.0004                # each half-hole's centre stands this far proud of its half's face, so a bar closed
#                                   on a tube grips it with its faces apart instead of touching [CHOICE]
PRESS_FIT = 0.0001                # m: the hinged half-hole's reach into the tube's place at the hinge's rest (0)
HOLE_SINK_FRONT = 0.0001          # the hinged half's: 0.3 mm less, so it touches the tube at rest (TH_SHUT), plus
#                                   PRESS_FIT (bar_half)
FACE_GAP = 0.0002                 # the halves' faces at the hinge's stop (no tube) [CHOICE: no contact at rest]
SINK_D = 0.0012                   # the countersink's depth at the 3/8 opening's top [EST]
SLICE_RUN = 0.0015                # each half-hole slice's outer edge runs this far past the opening's end (bar_half)
GAP = 0.0123                      # bar material between openings [ARITH: (190.5 - 79.5 mm of openings) / 9]


def hole_centres():
    """Each opening's centre along y, smallest at the hinged end."""
    out, y = [], HINGE_Y + GAP
    for s in SIZES_IN:
        r = s * IN / 2
        out.append(y + r)
        y += 2 * r + GAP
    return out


HOLE_Y = hole_centres()
I38 = SIZES_IN.index(0.375)
HY = HOLE_Y[I38]                  # the 3/8 opening's centre (y)
HR = TUBE_OD / 2 + HOLE_CLEAR     # its radius
TUBE_X = BX - 0.00025             # where the closed bar holds the tube (tools/geom.py)
TH_SHUT = PRESS_FIT / (HY - HINGE_Y)   # rad: the hinged half's angle when the closed bar holds the tube (1.1 mrad
#                                   open, the spring pressing it on), touching both half-holes (tools/geom.py)
TH_LOW = -0.001                   # the hinge's closed stop without a tube: the faces 0.01 mm apart at the free end.
#                                   WHY not lower: a 14 g tube does not stop the stiff clamp in the sim, the stop does
PROUD = 0.004                     # the tube's end stands this far over the bar's top at the start [CHOICE: as put in
#                                   by hand, before step 3's "even with top"]

# -- the yoke, the feed screw, the cone, the clamp screw [EST from the photo, scaled by BAR_L] ---------------------
YOKE_W = 0.040                    # along the bar (y)
YOKE_X = (BX - HALF_W - 0.018, BX + HALF_W + 0.010)   # front (with the clamp screw) and back faces
YOKE_Z = (Z_BAR + 0.003, Z_BAR + 0.048)                # the head above the bar
CHEEK_T = 0.007                   # the cheeks down both sides of the bar
CHEEK_GAP = 0.0035                # each cheek's play off the bar's face (the front half opens into it)
PARK_Y = HINGE_Y - 0.008          # the yoke's middle when parked at the hinged end: its clamp screw behind the hinge
#                                   pin, so the bar opens [CHOICE]
YOKE_TRAVEL = (-0.002, BAR_L - 0.055)
HUB_R, HUB_H = 0.012, 0.022       # the feed screw's handle hub over the head
WING_L, WING_W, WING_T = 0.100, 0.014, 0.010   # the handle's two wings, tip to tip [EST: photo, about half the bar]
WING_Z = YOKE_Z[1] + HUB_H        # the wings' underside: their top is TOP + 0.165
CONE_R = 0.010                    # drawn only
PITCH = 0.002                     # m per turn of the feed screw [EST]
SEAT_D = 0.0025                   # the cone's tip goes this far under the bar's top at full depth (flare formed) [EST]
TIP_START = (0.002, 0.008)        # the cone's tip over the bar's top at rest, by seed [CHOICE]: 1 to 4 turns to the tube
CLAMP_Z = Z_BAR - BAR_H / 2       # the clamp screw's axis (along x), at the yoke's middle
CLAMP_PITCH = 0.0015              # [EST]
CLAMP_GAP = 0.0005                # its tip's play off the front half's face at rest (loosened, step 2) [CHOICE]
TBAR_HALF = 0.050                 # the sliding T-bar's half length through the screw's head [EST]
TBAR_R = 0.003
KNOB_R = 0.005                    # the T-bar's ball ends [EST]
T_GRIP_R = 0.046                  # the balls' centres from the screw's axis: a hand holds the T-bar there
TBAR_START = 80.0                 # deg: the T-bar's -y end starts this far up from level [CHOICE]
CLAMP_K = 80000.0                 # N/m: the screw's tip as a stiff spring on its thread [CHOICE]: 0.4 turn past contact
#                                   (0.6 mm) squeezes the bar onto the tube with about 50 N
CLAMP_TIGHT = 0.4                 # turns past contact that the model counts as tightened [CHOICE]
HINGE_K = 60.0                    # N m/rad: the light spring that closes the bar's front half [CHOICE, see fidelity]
HINGE_TARGET = 0.0                # rad: its rest. WHY 0 with the half-hole 0.1 mm into the tube's place (PRESS_FIT):
#                                   the tube holds the half 1.1 mrad open, so the spring grips it with 0.8 N from the
#                                   first settle on (a target past the stop did not hold in the sim)

# -- the forming model --------------------------------------------------------------------------------------------
THRUST = 15.0                     # N down on the tube while the cone forms it [CHOICE: scaled, see fidelity]
STRIKE_D = 0.0005                 # m: a cone whose tip is already this far below the tube's end when its axis comes over
#                                   the tube struck it from the side [CHOICE: over 10x one control step's feed]
CENTRE_TOL = 0.001                # the cone's axis within 1 mm of the tube's at first contact, or the flare is lopsided
#                                   [CHOICE: a tenth of the tube's 9.5 mm; the real-trial check is concentricity]
FLARE_OK = 0.0005                 # the tube's end within 0.5 mm of the bar's top when the flare is formed [CHOICE:
#                                   EC42865 Figure 2, "Standard Flare - Even with Top"; above or below changes the
#                                   flare's diameter. The real-trial check is the flare's OD, README]
IRON_TURNS = 2.0                  # full handle turns after the clutch releases (EC42865 step 6)
RETRACT = 0.002                   # the cone's tip this far over the bar's top at the end (clear of the flare)

# -- loose and fixed parts ---------------------------------------------------------------------------------------
MATERIALS = {"default": (1.2, 1.0),   # the scene's friction on its own assets (rules.md 3)
             "steel": (0.6, 0.5),     # tool steel on tool steel [EST]
             "copper": (0.53, 0.45),  # copper on steel [EST: handbook range]
             "wood": (0.5, 0.4)}
STEEL, BLACK, SILVER, WHITE = (0.16, 0.16, 0.17), (0.07, 0.07, 0.08), (0.75, 0.76, 0.78), (0.95, 0.95, 0.95)
COPPER, BRASS, WOOD, RED = (0.78, 0.45, 0.30), (0.80, 0.66, 0.30), (0.60, 0.45, 0.30), (0.80, 0.10, 0.10)


def _box(lo, hi, color, material="steel", collide=True):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Shape("box", tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, material=material,
                 collide=collide)


def _cyl(c, r, h, color, axis="Z", material="steel", collide=True, mass=0.0):
    return Shape("cylinder", tuple(float(v) for v in c), (float(r), float(h)), color, axis=axis, material=material,
                 collide=collide, mass=mass)


def bar_half(side):
    """One half of the flaring bar: +1 the back half (fixed), -1 the front half (hinged, with its tab past the free
    end). Solid between the openings, and round each opening a half-hole of polygon slices."""
    face = BX if side > 0 else BX - FACE_GAP
    out_x = face + side * HALF_W
    z0, z1 = Z_BAR - BAR_H, Z_BAR
    shapes, y = [], HINGE_Y
    for yc, s in zip(HOLE_Y, SIZES_IN, strict=True):
        r = s * IN / 2 + HOLE_CLEAR
        if yc - r > y + 1e-6:
            lo, hi = sorted((face, out_x))
            shapes.append(_box((lo, y, z0), (hi, yc - r, z1), STEEL))
        cx = face - side * (HOLE_SINK if side > 0 else HOLE_SINK_FRONT + PRESS_FIT)  # the half-hole's centre stands proud of the face (toward the other half)
        n = 12 if s == 0.375 else 6
        # the 45 deg countersink at each opening's top (the flare's seat), modelled at the 3/8 opening only
        layers = ((z0, z1 - SINK_D, 0.0), (z1 - SINK_D, z1, SINK_D)) if s == 0.375 else ((z0, z1, 0.0),)
        for za, zb, grow in layers:
            for k in range(n):
                a0, a1 = math.pi * k / n, math.pi * (k + 1) / n
                pts = []
                for z, g in ((za, 0.0), (zb, grow)):
                    rr = r + g
                    ya, yb = yc - rr * math.cos(a0), yc - rr * math.cos(a1)
                    xa, xb = cx + side * rr * math.sin(a0), cx + side * rr * math.sin(a1)
                    xa, xb = (max(xa, face), max(xb, face)) if side > 0 else (min(xa, face), min(xb, face))
                    # WHY the outer edge runs on past the opening's end: a slice near the end of the arc is only
                    # 0.15 mm thick, and PhysX may inflate a hull thinner than its 1 mm minimum thickness into the
                    # opening. Running on into the solid bar keeps every slice over 1 mm (test) and leaves the
                    # opening's side as it was: the extra edge starts on the arc and leaves it outward.
                    ya_o = yc - rr - SLICE_RUN if k < n // 2 else ya
                    yb_o = yc + rr + SLICE_RUN if k >= n // 2 else yb
                    ring = [(xa, ya), (out_x, ya_o), (out_x, yb_o), (xb, yb)]
                    if side < 0:
                        ring = [(out_x, ya_o), (xa, ya), (xb, yb), (out_x, yb_o)]
                    pts += [(x, yy, z) for x, yy in ring]
                shapes.append(hull(pts, STEEL, material="steel"))
        y = yc + r
    lo, hi = sorted((face, out_x))
    shapes.append(_box((lo, y, z0), (hi, HINGE_Y + BAR_L, z1), STEEL))
    # the size marks on the bar's top, one white line beside each opening on the front half's top face [drawn only]
    if side < 0:
        for yc in HOLE_Y:
            shapes.append(_box((BX - HALF_W + 0.001, yc - 0.0004, z1), (BX - HALF_W + 0.004, yc + 0.0004, z1 + 0.0003),
                               WHITE, collide=False))
    return shapes


def base_shapes():
    """The fixed parts: the back half of the bar, the hinge knuckle and the two support blocks."""
    sh = bar_half(+1)
    z0 = Z_BAR - BAR_H
    sh.append(_cyl((BX, HINGE_Y - 0.0075, Z_BAR - BAR_H / 2), 0.006, BAR_H, STEEL))    # the hinge knuckle
    for ya, yb in ((HINGE_Y - 0.010, HINGE_Y + 0.022), (HINGE_Y + BAR_L - 0.022, HINGE_Y + BAR_L)):
        sh.append(_box((BX + 0.0005, ya, TOP), (BX + HALF_W - 0.0005, yb, z0), WOOD, material="wood"))
    return sh


def front_half_shapes():
    return bar_half(-1)


def yoke_shapes():
    """The yoke at rest (parked): its head above the bar and two cheeks down the bar's faces. Its arrow (red, on the
    head's front top edge) marks the cone's axis."""
    y0, y1 = PARK_Y - YOKE_W / 2, PARK_Y + YOKE_W / 2
    xf, xb = YOKE_X
    sh = [_box((xf, y0, YOKE_Z[0]), (xb, y1, YOKE_Z[1]), SILVER),
          _box((xf, y0, Z_BAR - BAR_H), (xf + CHEEK_T, y1, YOKE_Z[0]), SILVER),
          _box((BX + HALF_W + CHEEK_GAP, y0, Z_BAR - BAR_H), (xb, y1, YOKE_Z[0]), SILVER),
          _box((xf - 0.0002, PARK_Y - 0.0006, YOKE_Z[1] - 0.012), (xf, PARK_Y + 0.0006, YOKE_Z[1]), RED, collide=False),
          _cyl((BX, PARK_Y, YOKE_Z[1] - 0.004), HUB_R + 0.002, 0.008, BLACK, collide=False)]
    return sh


def handle_shapes():
    c = (BX, PARK_Y)
    return [_cyl((c[0], c[1], YOKE_Z[1] + HUB_H / 2), HUB_R, HUB_H, BLACK),
            _box((c[0] - WING_W / 2, c[1] - WING_L / 2, WING_Z), (c[0] + WING_W / 2, c[1] + WING_L / 2, WING_Z + WING_T),
                 BLACK),
            _box((c[0] + WING_W / 2, c[1] - 0.001, WING_Z + 0.002), (c[0] + WING_W / 2 + 0.0003, c[1] + 0.001,
                                                                     WING_Z + WING_T - 0.002), WHITE, collide=False)]


def cone_shapes():
    """The cone and its screw, drawn only: the model poses it and works the tube (fidelity). The screw's top (red)
    stands up through the handle's hub: it sinks 2 mm a turn and stops sinking when the clutch releases."""
    tip = Z_BAR + TIP_START[1]
    pin_top, pin_bot = WING_Z + WING_T + 0.013, YOKE_Z[1] - 0.004
    return [_cyl((BX, PARK_Y, (pin_top + pin_bot) / 2), 0.0025, pin_top - pin_bot, SILVER, collide=False),
            _cyl((BX, PARK_Y, pin_top - 0.002), 0.0026, 0.004, RED, collide=False),
            _cyl((BX, PARK_Y, tip + 0.004), CONE_R * 0.5, 0.008, SILVER, collide=False),
            _cyl((BX, PARK_Y, tip + 0.026), 0.004, 0.036, SILVER, collide=False),
            _cyl((BX, PARK_Y, tip + 0.040), 0.0042, 0.0015, WHITE, collide=False)]


X_HEAD = YOKE_X[0] - 0.055        # the clamp screw's head (its middle), on a shank in front of the yoke [CHOICE: a
#                                   gripper holding the T-bar reaches 51 mm each way along x; tools/clearance.py]


def _rod(c, half, r, angle_deg, color):
    """A square rod (2r across) of half length ``half`` through ``c``, in the y-z plane, its +y end raised by
    ``angle_deg``: a hull."""
    a = angle_deg * D2R
    u = np.array([0.0, math.cos(a), math.sin(a)])
    v = np.array([0.0, -math.sin(a), math.cos(a)])
    c = np.asarray(c, float)
    ring = [(-r, -r), (r, -r), (r, r), (-r, r)]
    pts = [c - half * u + dx * np.array([1.0, 0, 0]) + dv * v for dx, dv in ring]
    pts += [c + half * u + dx * np.array([1.0, 0, 0]) + dv * v for dx, dv in ring]
    return hull([tuple(p) for p in pts], color, material="steel")


def clamp_shapes():
    """The clamp screw's head (along x, in front of the yoke) and its sliding T-bar, at rest with its -y end
    TBAR_START deg up. A right-hand thread: tightening is clockwise seen from the head (in front, -x), a
    turn about +x, which brings that end down."""
    return [_cyl((X_HEAD, PARK_Y, CLAMP_Z), 0.007, 0.010, BLACK, axis="X"),
            _rod((X_HEAD, PARK_Y, CLAMP_Z), TBAR_HALF, TBAR_R, -TBAR_START, SILVER),
            *_knobs(),
            _cyl(((X_HEAD + YOKE_X[0]) / 2, PARK_Y, CLAMP_Z), 0.003, YOKE_X[0] - X_HEAD - 0.010, SILVER, axis="X",
                 collide=False)]


def _knobs():
    """The T-bar's upset ends, where a hand holds it: a ball on each end. WHY: the pads touch a ball at one point
    each, both on the screw's axis line through it, so the bar turns freely in a closed grip; pads flat on the
    square bar held it from turning and dragged the yoke along the bar instead (sim)."""
    a = -TBAR_START * D2R
    u = np.array([0.0, math.cos(a), math.sin(a)])
    c = np.array([X_HEAD, PARK_Y, CLAMP_Z])
    return [Shape("capsule", tuple((c + s * T_GRIP_R * u).tolist()), (KNOB_R, 0.0005), SILVER, axis="X",
                  material="steel") for s in (-1, 1)]


TIP_X0 = BX - FACE_GAP - HALF_W - CLAMP_GAP   # the clamp tip's face at rest


def tip_shapes():
    """The clamp screw's tip, which bears on the front half's face (its dimple) through the cheek."""
    return [_cyl((TIP_X0 - 0.002, PARK_Y, CLAMP_Z), 0.003, 0.004, STEEL, axis="X")]


def _tool():
    base = Link("base", 2.0, tuple(base_shapes()))
    links = [Link("front_half", 0.20, tuple(front_half_shapes())),
             Link("yoke", 0.60, tuple(yoke_shapes())),
             Link("handle", 0.12, tuple(handle_shapes())),
             Link("cone", 0.05, tuple(cone_shapes())),
             Link("clamp", 0.05, tuple(clamp_shapes())),
             Link("clamp_tip", 0.01, tuple(tip_shapes()))]
    x_head = X_HEAD
    joints = [
        # WHY a spring on the hinge: a closed bar must hold the tube until it is clamped (fidelity, PRESS_FIT)
        Joint("hinge", "revolute", "front_half", (BX, HINGE_Y, Z_BAR), "Z", TH_LOW, 0.20, stiffness=HINGE_K,
              damping=0.3, friction=0.02, target=HINGE_TARGET),
        Joint("slide", "prismatic", "yoke", (BX, PARK_Y, Z_BAR), "Y", *YOKE_TRAVEL, damping=4.0, friction=1.5),
        Joint("feed", "revolute", "handle", (BX, PARK_Y, YOKE_Z[1]), "Z", -math.inf, math.inf, damping=0.02,
              friction=0.03, parent="yoke"),
        Joint("cone_z", "prismatic", "cone", (BX, PARK_Y, YOKE_Z[1]), "Z", -0.030, 0.0, stiffness=4000.0,
              damping=60.0, parent="yoke"),
        # WHY no friction on the clamp screw: Isaac Lab's joint friction scales with the joint's load, so a hand pushing
        # the T-bar met a drag of about 40% of its push and shoved the yoke 2-4 mm off the line instead (sim). The
        # model takes no load on the screw (it poses the tip), so the screw turns on its damping alone
        Joint("clamp_turn", "revolute", "clamp", (x_head, PARK_Y, CLAMP_Z), "X", -math.inf, math.inf, damping=0.02,
              parent="yoke"),
        Joint("clamp_x", "prismatic", "clamp_tip", (x_head, PARK_Y, CLAMP_Z), "X", 0.0, 0.004, stiffness=CLAMP_K,
              damping=80.0, parent="yoke"),
    ]
    return mechanism("tool", (0.0, 0.0, 0.0), base, links, joints, MATERIALS)


def tube_shapes():
    """The tube, frame at its bottom end, z up its axis. WHY no flare nut on it: a nut on the tube could not pass the
    bar's opening, and a short offcut in a holder can only go in from above (fidelity)."""
    return [_cyl((0, 0, TUBE_L / 2), TUBE_OD / 2, TUBE_L, COPPER, material="copper", mass=TUBE_KG)]


TOOL = _tool()
TUBE_START = (TUBE_X, HY, Z_BAR + PROUD - TUBE_L)
TUBE = body("tube", TUBE_START, tube_shapes(), MATERIALS, color=COPPER)
OBJECTS = [TOOL, TUBE]

# -- the model: the feed screw, the cone, the clutch, the clamp screw and the flare, run after every control step ----
_IDS = {}
JOINTS = ("hinge", "slide", "feed", "cone_z", "clamp_turn", "clamp_x")


PARK_OFF = (0.0, 0.004, 0.008)    # m: how far along the bar from the hinged end the yoke rests, by seed [CHOICE]. WHY:
#                                   a script that grips the handle at one place and slides it a fixed way ends off the
#                                   3/8 line on two seeds in three; the policy has to see where the yoke is. 8 mm at most: further, the right
#                                   hand pressing the tube meets the parked handle's wing (tools/clearance.py: -3.5 mm at 16 mm)


def park_off(seed):
    """The yoke's start along the bar (m past its park at the hinged end): 0, 4 or 8 mm by seed (0 with no seed),
    independent of the cone's start (tip_start)."""
    return 0.0 if seed is None else PARK_OFF[(int(seed) // 3) % 3]


def tip_start(seed):
    """The cone's tip over the bar's top at rest (m): 2, 5 or 8 mm by seed (5 with no seed), so the turns to
    the tube differ from trial to trial."""
    return 0.005 if seed is None else TIP_START[0] + 0.003 * (int(seed) % 3)


def _fresh_model(seed=None):
    t0 = tip_start(seed)
    return {"tip0": t0, "feed": 0.0, "feed_full": t0 + SEAT_D, "feed_prev": None, "clamp_prev": None,
            "clamp_turns": 0.0, "touched": False, "tight_at_touch": False, "struck": False, "centre_err": None, "formed": False, "p": None, "iron": 0.0,
            "pushing": False, "tip_z": Z_BAR + t0, "axis": (BX, PARK_Y), "tight": False, "error": None,
            "steps": 0}


_M = _fresh_model()


def _np(x):
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _wrap_pi(a):
    return (a + math.pi) % TURN - math.pi


def cone_joint(m):
    """The cone_z joint position for the model's feed: 0 puts the tip TIP_START[1] (8 mm) over the bar's top."""
    return m["tip0"] - m["feed"] - TIP_START[1]


def clamp_joint(m):
    return float(np.clip(m["clamp_turns"] * CLAMP_PITCH, 0.0, 0.004))


def advance(m, d_feed_turns, d_clamp_turns, tube_top, tube_xy, upright, cone_xy, clamp_x_actual):
    """One step of the model, pure (tests call it). ``d_feed_turns``: the handle's clockwise turns this step (seen from
    above); ``d_clamp_turns``: the clamp screw's tightening turns; the tube's top end height and axis; the cone's axis
    (the yoke's place); the clamp tip's actual travel (m). Updates ``m`` and returns the downward thrust on the tube."""
    m["clamp_turns"] = max(0.0, m["clamp_turns"] + d_clamp_turns)
    m["tight"] = bool(clamp_joint(m) - clamp_x_actual >= CLAMP_TIGHT * CLAMP_PITCH)
    f = m["feed"] + d_feed_turns * PITCH
    f = max(f, m["tip0"] - TIP_START[1])            # the cone's top stop
    if f > m["feed_full"]:                          # the clutch releases: the handle turns, the cone stays
        if m["formed"] and m["tight"]:          # WHY tight: a loosened clamp lets the tube turn and lift; it irons nothing
            m["iron"] += (f - m["feed_full"]) / PITCH
        f = m["feed_full"]
    m["feed"] = f
    tip = Z_BAR + m["tip0"] - f
    m["tip_z"] = tip
    m["axis"] = tuple(cone_xy)
    off = math.hypot(tube_xy[0] - cone_xy[0], tube_xy[1] - cone_xy[1])
    over = upright and off <= TUBE_OD / 2 + 0.002
    thrust = 0.0
    m["pushing"] = False
    if over and tip <= tube_top:
        if not m["touched"]:
            m["touched"], m["centre_err"] = True, off
            # WHY these two at first contact: EC42865 clamps the tube before the cone comes down (steps 5, 6), and a
            # cone brought over the tube from the side with its tip already below the end strikes it, not flares it
            m["tight_at_touch"] = m["tight"]
            m["struck"] = tube_top - tip > STRIKE_D
        if not m["formed"]:
            m["pushing"] = True
            # WHY no push on a clamped tube: the tightened clamp's grip (about 50 N) holds it against the cone in the
            # real tool, but a 14 g tube squeezed between the sim's stiff halves sinks into them instead of being
            # held (sim). So the model holds a clamped tube; a loose one gets the cone's push and slides (fidelity)
            thrust = 0.0 if m["tight"] else THRUST
    if m["touched"] and not m["formed"] and f >= m["feed_full"] - 1e-9:
        m["formed"], m["p"] = True, tube_top - Z_BAR   # where the end stood when the cone reached full depth
    return thrust


def flare_good(m):
    return bool(m["formed"] and m["centre_err"] is not None and m["centre_err"] <= CENTRE_TOL
                and abs(m["p"]) <= FLARE_OK and m["tight_at_touch"] and not m["struck"])


def flare_scrap(m):
    """A flare formed off-centre, with the end too high or too low, on an unclamped tube, or by a cone struck onto the
    tube from the side: it cannot be redone (the end is cut off)."""
    return bool(m["formed"] and not flare_good(m))


def _ids(asset):
    if not _IDS:
        names = list(asset.joint_names)
        _IDS.update({j: names.index(j) for j in JOINTS})
    return _IDS


def _rot3(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tube_top(pos, quat):
    r = _rot3(quat)
    top = np.asarray(pos, float) + r[:, 2] * TUBE_L
    return float(top[2]), (float(top[0]), float(top[1])), bool(r[2, 2] >= math.cos(5 * D2R))


@on_reset
def reset_model():
    reset_memory(run_seed())
    env = run_env()
    if env is None:
        return
    try:
        import torch
        tool = env.unwrapped.scene["tool"]
        ids = _ids(tool)
        dev = _np_device(tool)
        # WHY every joint from the settled state: the hinge keeps the press fit it settled into; only the yoke's
        # start (seeded) and the cone's height change
        q = _torch_of(tool.data.joint_pos).clone()
        q[:, ids["cone_z"]] = cone_joint(_M)
        q[:, ids["slide"]] = park_off(run_seed())
        z = torch.zeros_like(q)
        tgt = torch.zeros_like(q)                   # every spring's target is 0 (HINGE_TARGET too) but the cone's
        tgt[:, ids["cone_z"]] = cone_joint(_M)
        if hasattr(tool, "write_joint_state_to_sim_index"):
            tool.write_joint_state_to_sim_index(position=q, velocity=z)
            tool.set_joint_position_target_index(target=tgt)
        else:
            tool.write_joint_state_to_sim(q, z)
            tool.set_joint_position_target(tgt)
        tool.write_data_to_sim()
    except Exception as e:  # WHY: never stop the run; the error is reported in the model's state
        _M["error"] = "reset " + repr(e)


def _torch_of(x):
    return getattr(x, "torch", x)


def _np_device(asset):
    x = asset.data.joint_pos
    return getattr(x, "torch", x).device


@on_step
def run_model(env):
    """Read the tool's joints and the tube, advance the model, then pose the cone and the clamp tip and push on the
    tube for the next step."""
    try:
        import torch
        base = env.unwrapped
        tool, tube = base.scene["tool"], base.scene["tube"]
        ids = _ids(tool)
        q = _np(tool.data.joint_pos)[0]
        origin = _np(base.scene.env_origins[0])
        feed, clamp = float(q[ids["feed"]]), float(q[ids["clamp_turn"]])
        d_feed = 0.0 if _M["feed_prev"] is None else -_wrap_pi(feed - _M["feed_prev"]) / TURN   # clockwise from above
        d_clamp = 0.0 if _M["clamp_prev"] is None else _wrap_pi(clamp - _M["clamp_prev"]) / TURN    # about +x
        _M["feed_prev"], _M["clamp_prev"] = feed, clamp
        pose = _np(tube.data.root_link_pose_w)[0]
        top, xy, up = tube_top(pose[:3] - origin, pose[3:7])
        cone_xy = (BX, PARK_Y + float(q[ids["slide"]]))
        thrust = advance(_M, d_feed, d_clamp, top, xy, up, cone_xy, float(q[ids["clamp_x"]]))
        dev = _np_device(tool)
        tgt = torch.tensor([[cone_joint(_M), clamp_joint(_M)]], dtype=torch.float32, device=dev)
        j = [ids["cone_z"], ids["clamp_x"]]
        if hasattr(tool, "set_joint_position_target_index"):
            tool.set_joint_position_target_index(target=tgt, joint_ids=j)
        else:
            tool.set_joint_position_target(tgt, joint_ids=j)
        f = torch.tensor([[[0.0, 0.0, -thrust]]], dtype=torch.float32, device=dev)
        t = torch.zeros((1, 1, 3), dtype=torch.float32, device=dev)
        if hasattr(tube, "permanent_wrench_composer"):
            tube.permanent_wrench_composer.set_forces_and_torques_index(forces=f, torques=t, is_global=True)
        else:
            tube.set_external_force_and_torque(f, t)
        _M["steps"] += 1
    except Exception as e:  # WHY: never stop the run; no model step means no flare and so no full score
        _M["error"] = "step " + repr(e)


# -- the grippers, read every control step (scene keys left_robot / right_robot) ------------------------------------
HAND_LINKS = ("gripper", "tip_left", "tip_right")
_HANDS = {"hands": None, "steps": 0, "error": None}
_CLOCK = {"dt": None}


def hand_hulls():
    """Each hand link's URDF collision hulls in its own frame: (link, vertices, face normals, centre, radius)."""
    if "hulls" not in _HULLS:
        import warnings

        from yam_arena.analysis import link_hulls

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            hulls = link_hulls()
        out = []
        for name in HAND_LINKS:
            for h in hulls[name]:
                n = np.unique(h.planes[:, :3].round(9), axis=0)
                c = (h.vertices.min(axis=0) + h.vertices.max(axis=0)) / 2
                out.append((name, h.vertices, n, c, float(np.linalg.norm(h.vertices - c, axis=1).max())))
        _HULLS["hulls"] = tuple(out)
    return _HULLS["hulls"]


_HULLS = {}


@on_step
def read_hands(env):
    """Both hands' collision hulls in the task frame (world less the env origin)."""
    try:
        base = env.unwrapped
        origin = _np(base.scene.env_origins[0])
        hands = []
        for arm in ("left_robot", "right_robot"):
            robot = base.scene[arm]
            names = list(robot.body_names)
            poses = _np(robot.data.body_link_pose_w)[0]
            for link, v, n, c, r in hand_hulls():
                p = poses[names.index(link)]
                rot = _rot3(p[3:7])
                hands.append((v @ rot.T + p[:3] - origin, n @ rot.T, rot @ c + p[:3] - origin, r))
        _HANDS["hands"] = hands
        _HANDS["error"] = None
    except Exception as e:  # WHY: no hand reading means no clearance, so no full score; never stop the run
        _HANDS["hands"], _HANDS["error"] = None, repr(e)
    _HANDS["steps"] += 1


@on_step
def read_clock(env):
    try:
        _CLOCK["dt"] = float(env.unwrapped.step_dt)
    except Exception:  # WHY: never stop the run; no step length means not still, so no full score
        _CLOCK["dt"] = None


_PRISM = 16


def _prism(c, r, z0, z1):
    """A 16-sided prism round radius r about the vertical through c (so it contains that circle): (vertices, normals)."""
    rr = r / math.cos(math.pi / _PRISM)
    ang = [2 * math.pi * (k + 0.5) / _PRISM for k in range(_PRISM)]
    v = np.array([(c[0] + rr * math.cos(u), c[1] + rr * math.sin(u), z) for z in (z0, z1) for u in ang])
    n = np.array([(math.cos(2 * math.pi * k / _PRISM), math.sin(2 * math.pi * k / _PRISM), 0.0)
                  for k in range(_PRISM)] + [(0.0, 0.0, 1.0)])
    return v, n


def _cuboid(lo, hi):
    v = np.array([(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    return v, np.eye(3)


def _sat_gap(hv, hn, kv, kn):
    """A lower bound on the distance between two convex sets (largest projection gap on their face normals)."""
    ax = np.vstack([hn, kn])
    ph, pk = hv @ ax.T, kv @ ax.T
    return float(np.max(np.maximum(pk.min(axis=0) - ph.max(axis=0), ph.min(axis=0) - pk.max(axis=0))))


def keep_out(objects):
    """Convex sets round what the hands must leave at the end: the tube (with its flare) and the tool's
    moving parts at the yoke's place: the head, the handle's whole swing and the clamp's T-bar swing."""
    t = objects["tube"]
    r = _rot3(t.quat_xyzw)
    p = np.asarray(t.pos, float)
    top = p + r[:, 2] * TUBE_L
    out = [_prism((top[0], top[1]), TUBE_OD / 2 + 0.002, float(p[2]), float(top[2]))]
    yc = PARK_Y + float(objects["tool"].joints["slide"])
    out.append(_prism((BX, yc), WING_L / 2, YOKE_Z[0], WING_Z + WING_T))
    out.append(_cuboid((X_HEAD - 0.008, yc - TBAR_HALF - TBAR_R, CLAMP_Z - TBAR_HALF - TBAR_R),
                       (X_HEAD + 0.008, yc + TBAR_HALF + TBAR_R, CLAMP_Z + TBAR_HALF + TBAR_R)))
    res = []
    for v, n in out:
        c = (v.min(axis=0) + v.max(axis=0)) / 2
        res.append((v, n, c, float(np.linalg.norm(v - c, axis=1).max())))
    return res


CLEAR_MIN = 0.010


def hand_gap(objects, hands=None, need=None):
    """Smallest distance (m) from the hands to keep_out() (a lower bound), -inf when the hands were not read."""
    hs = _HANDS["hands"] if hands is None else hands
    if hs is None or len(hs) == 0:
        return -math.inf
    best = math.inf
    for kv, kn, kc, kr in keep_out(objects):
        for hv, hn, hc, hr in hs:
            g = float(np.linalg.norm(hc - kc)) - hr - kr
            if g < (CLEAR_MIN + 0.01 if need is None else need):
                g = max(g, _sat_gap(hv, hn, kv, kn))
            if need is not None and g < need:
                return g
            best = min(best, g)
    return best


def clear(objects):
    return bool(hand_gap(objects, need=CLEAR_MIN) >= CLEAR_MIN)


# -- scoring ---------------------------------------------------------------------------------------------------------
IN_TOL = 0.0015         # the tube's axis within 1.5 mm of the 3/8 opening's centre
LOST_Z = TOP - 0.030
STILL_V, STILL_W = 0.005, 0.05


def in_opening(objects):
    """The tube upright in the 3/8 opening, its end within 10 mm of the bar's top."""
    t = objects["tube"]
    top, xy, up = tube_top(t.pos, t.quat_xyzw)
    return bool(up and math.hypot(xy[0] - BX, xy[1] - HY) <= IN_TOL and abs(top - Z_BAR) <= 0.010)


def set_even(objects):
    t = objects["tube"]
    top, _, _ = tube_top(t.pos, t.quat_xyzw)
    return bool(in_opening(objects) and abs(top - Z_BAR) <= FLARE_OK)


def retracted(m):
    return bool(m["tip_z"] >= Z_BAR + RETRACT)


def failure(objects, m):
    """Latched: the tube off the table, or a flare that cannot be used (off-centre, the end not within 0.5 mm of the
    bar's top when it was formed, the clamp not tight when the cone met the tube, or the cone struck onto the tube from
    the side), or the model not running (an error, or not stepping with the run)."""
    if float(objects["tube"].pos[2]) < LOST_Z:
        return "tube lost"
    if flare_scrap(m):
        why = "" if m["tight_at_touch"] else ", clamp not tight"
        why += ", cone struck from the side" if m["struck"] else ""
        return f"flare scrap (end {1000 * m['p']:+.2f} mm, off-centre {1000 * m['centre_err']:.2f} mm{why})"
    if m["error"]:
        return "model " + m["error"]
    if _HANDS["steps"] - m["steps"] > 5:
        return "model not stepping"
    return None


def on_line(objects):
    """The yoke's arrow (the cone's axis) within CENTRE_TOL of the 3/8 opening's centre along the bar."""
    return bool(abs(PARK_Y + float(objects["tool"].joints["slide"]) - HY) <= CENTRE_TOL)


def stage(objects, m):
    """0 nothing; 1 the tube set even in the 3/8 opening, the yoke on the 3/8 line, the clamp tight; 2 a good flare;
    3 and ironed (2+ turns after the clutch released); 4 and the cone retracted clear of it."""
    if flare_good(m):
        if m["iron"] < IRON_TURNS:
            return 2
        return 4 if retracted(m) else 3
    if set_even(objects) and m["tight"] and on_line(objects):
        return 1
    return 0


def pose(objects):
    t = objects["tube"]
    return (np.asarray(t.pos, float), _rot3(t.quat_xyzw), float(objects["tool"].joints["feed"]))


def rates(prev, now, dt):
    (p0, r0, f0), (p1, r1, f1) = prev, now
    c = (float(np.trace(r0.T @ r1)) - 1.0) / 2.0
    w = max(math.acos(float(np.clip(c, -1.0, 1.0))), abs(_wrap_pi(f1 - f0)))
    return float(np.linalg.norm(p1 - p0)) / dt, w / dt


def still(r):
    return bool(r is not None and r[0] <= STILL_V and r[1] <= STILL_W)


def _fresh():
    return {"last": None, "fail": None, "now": None, "pose": None, "rates": None, "held": 0.0}


_MEM = _fresh()


def reset_memory(seed=None):
    """A fresh trial: the model (cone start by seed), the scorer's latches, the hands and the clock."""
    _M.clear()
    _M.update(_fresh_model(seed))
    _MEM.clear()
    _MEM.update(_fresh())
    _HANDS.update({"hands": None, "steps": 0, "error": None})
    _CLOCK["dt"] = None


def update(objects):
    """One control step: latch a failure and judge this step (same states dict for success() and score())."""
    if objects is _MEM["last"]:
        return _MEM
    _MEM["last"] = objects
    why = failure(objects, _M)
    if why and not _MEM["fail"]:
        _MEM["fail"] = why
    p, dt = pose(objects), _CLOCK["dt"]
    _MEM["rates"] = rates(_MEM["pose"], p, dt) if _MEM["pose"] is not None and dt else None
    _MEM["pose"] = p
    n = {"stage": stage(objects, _M), "clear": clear(objects), "still": still(_MEM["rates"])}
    n["done"] = bool(not _MEM["fail"] and n["stage"] == 4 and n["clear"] and n["still"])
    _MEM["held"] = _MEM["held"] + (dt or 0.0) if n["done"] else 0.0
    _MEM["now"] = n
    return _MEM


def success(objects):
    """A good flare, ironed, the cone retracted, the hands clear and the tube and handle still (held 1 s by
    yam_arena), with no latched failure."""
    return update(objects)["now"]["done"]


LADDER = {0: 0.0, 1: 0.25, 2: 0.5, 3: 0.6}
HOLD_S = 1.0


def score(objects):
    """At the last step: success held 1 s scores 1.0 (yam_arena also ends the trial then); 0 after a latched failure.
    Retracted, but a hand within 10 mm of the tube or the tool's moving parts, or not still, or not yet held 1 s: 0.8. Ironed, not retracted: 0.6. A good flare, not
    ironed 2 turns: 0.5. Clamped in the 3/8 opening, end even with the top, no flare yet: 0.25."""
    m = update(objects)
    if m["fail"]:
        return 0.0
    n = m["now"]
    if n["stage"] == 4:
        return 1.0 if (n["done"] and m["held"] >= HOLD_S - 1e-9) else 0.8
    return LADDER[n["stage"]]


# -- demo -----------------------------------------------------------------------------------------------------------------
TIP = 0.013               # PAD_TIP_PAST_TCP: the pad tips sit 13 mm under the TCP
TIP_UP = 0.002
GRIP = 0.0                # finger target: squeeze (a grip turns or carries a part only through the pads' friction)
OPEN_F = -0.040
RELEASE = -0.018
T_OPEN = -0.012           # the fingers 24 mm apart round the T-bar (wider would reach the yoke)
YAW_X = math.pi / 2       # fingers closing along x (the right hand)
YAW_TUBE = math.pi / 2    # the right hand's yaw on the tube: fingers along x, clear of the parked handle's wings (along y)
YAW_XL = -math.pi / 2     # the same for the left hand (its wrist reaches -160..60 deg here: tools/yaw.py)
SLIDE_TO = HY + 0.0004    # the yoke's axis over the 3/8 opening. WHY 0.4 mm past it: tightening the clamp shifts the
#                           yoke 0.4-0.5 mm back toward the hinge (sim, both start delays), so it ends on the line
T_END = 230.0             # deg from +y about +x: each press carries an end round to 50 deg under level on the -y side
#                           (further, the fingers reach the screw's head)
PRESSES = 3               # the -y end 100 -> 230 deg, then each other end over the top 50 -> 230 (490 deg). WHY 3: the
#                           clamp first closes the bar to its stop, then squeezes; two strokes left it 0.5 of 0.6 mm (sim)
S_START, S_END = 40.0, -140.0   # deg: the wrist's yaw over the handle at each stroke's start and end (reach: -160..60)
FWD_TURNS = (TIP_START[1] + SEAT_D) / PITCH + IRON_TURNS + 0.25   # 7.5: the highest seed's cone to the stop and on
BACK_TURNS = (SEAT_D + RETRACT) / PITCH + 0.25                    # 2.5
PARK_L, PARK_R = (0.20, 0.24, TOP + 0.18), (0.20, -0.20, TOP + 0.18)


def _w(t, p, f, label, yaw):
    return Waypoint(float(t), tuple(float(v) for v in p), float(f), label, float(yaw))


def _hinge_pt(x, y, th):
    """(x, y) on the hinged half turned open by ``th`` about the hinge pin."""
    dx, dy = x - BX, y - HINGE_Y
    return BX + dx * math.cos(th) - dy * math.sin(th), HINGE_Y + dx * math.sin(th) + dy * math.cos(th)


class _Arm:
    def __init__(self):
        self.w, self.t = [], 0.0

    def add(self, dt, p, f, label, yaw):
        self.w.append(_w(dt, p, f, label, yaw))
        self.t += dt
        self.last = (tuple(p), f, yaw)

    def hold_to(self, t, label):
        if t > self.t + 1e-6:
            p, f, yaw = self.last
            self.add(t - self.t, p, f, label, yaw)


def plan(delay=0.0, seed=None):
    """(left, right, info). Right presses the tube's end even with the bar's top with its closed fingers, slides the
    yoke over it by its handle and tightens the clamp screw in three strokes of its T-bar; left turns the handle clockwise
    in half-turn strokes past the clutch's release, then back until the cone is clear; both park."""
    L, R = _Arm(), _Arm()
    # left: wait parked while the right hand sets the tube, slides the yoke and tightens the clamp
    L.add(2.0 + delay, PARK_L, OPEN_F, "to the park", 0.0)
    # right: press the tube's end down even with the bar's top with the closed fingers (their tips stop on the bar)
    # WHY 5 mm into the bar: the pad tips land on the bar's top beside the tube, which stops them with the tube's end
    # flush; the arm only pushes as hard as it lags its target (sim: 1 mm of lag did not beat the bar's grip)
    pz = Z_BAR + TIP - 0.005
    over = (TUBE_X, HY, Z_BAR + PROUD + TIP + 0.03)
    # WHY high first: a straight move from READY down to the tube crosses the parked handle's wing (tools/clearance.py)
    R.add(2.0 + delay, (TUBE_X, HY, TOP + 0.185), GRIP, "high over the tube", YAW_TUBE)
    R.add(1.0, over, GRIP, "over the tube's end", YAW_TUBE)
    R.add(0.5, over, GRIP, "settle over the tube's end", YAW_TUBE)
    R.add(0.8, (TUBE_X, HY, Z_BAR + PROUD + TIP + 0.001), GRIP, "down onto the tube's end", YAW_TUBE)
    R.add(2.0, (TUBE_X, HY, pz), GRIP, "press the end even with the bar", YAW_TUBE)
    R.add(0.5, (TUBE_X, HY, pz), GRIP, "hold even", YAW_TUBE)
    R.add(0.8, (TUBE_X, HY, Z_BAR + 0.05), GRIP, "lift off", YAW_TUBE)
    # right: slide the yoke by its handle, the wing gripped over the hub
    wz = WING_Z + TIP_UP + TIP
    py = PARK_Y + park_off(seed)      # the yoke's start, seen by the policy (the demo reads the seed)
    R.add(1.5, (BX, py, wz + 0.04), OPEN_F, "over the handle", YAW_X)
    R.add(0.5, (BX, py, wz + 0.04), OPEN_F, "settle over the handle", YAW_X)
    R.add(0.8, (BX, py, wz), OPEN_F, "descend to the wing", YAW_X)
    R.add(0.6, (BX, py, wz), GRIP, "close on the wing", YAW_X)
    R.add(0.3, (BX, py, wz), GRIP, "settle on the wing", YAW_X)
    R.add(3.0, (BX, SLIDE_TO, wz), GRIP, "slide the yoke to the 3/8 line", YAW_X)
    R.add(0.6, (BX, SLIDE_TO, wz), GRIP, "hold on the line", YAW_X)
    # WHY open wide and straight up first: half-open fingers lifting off dragged the yoke 2 mm off the line (sim)
    R.add(0.5, (BX, SLIDE_TO, wz), OPEN_F, "let go of the wing", YAW_X)
    R.add(0.6, (BX, SLIDE_TO, wz + 0.02), OPEN_F, "lift off the wing", YAW_X)
    R.add(0.6, (BX - 0.07, SLIDE_TO - 0.02, wz + 0.02), RELEASE, "back away from the handle", YAW_X)
    yaw_w = YAW_XL                    # the left hand's grip on the wing (along y, the fingers across it)
    # right: tighten the clamp screw clockwise (seen from in front): carry the T-bar's raised -y end down, then the
    # other end over the top and down
    for k in range(PRESSES):
        a0 = 180.0 - TBAR_START if k == 0 else T_END - 180.0
        def tp(a):
            return (X_HEAD, SLIDE_TO + T_GRIP_R * math.cos(a * D2R), CLAMP_Z + T_GRIP_R * math.sin(a * D2R) + 0.006)
        p0 = tp(a0)
        R.add(1.5 if k == 0 else 1.0, (p0[0], p0[1], p0[2] + 0.04), T_OPEN, f"over the T-bar's end {k + 1}", YAW_X)
        R.add(0.4, (p0[0], p0[1], p0[2] + 0.04), T_OPEN, f"settle over the T-bar {k + 1}", YAW_X)
        R.add(0.6, p0, T_OPEN, f"down to the T-bar {k + 1}", YAW_X)
        R.add(0.5, p0, GRIP, f"close on the T-bar {k + 1}", YAW_X)
        n = max(6, int(round((T_END - a0) / 20.0)))
        for i in range(1, n + 1):
            a = a0 + (T_END - a0) * i / n
            R.add(0.3, tp(a), GRIP, f"turn the T-bar {k + 1}.{i}", YAW_X)
        R.add(0.4, tp(T_END), T_OPEN, f"let go of the T-bar {k + 1}", YAW_X)
        pl = tp(T_END)
        R.add(0.5, (pl[0], pl[1] - 0.02, pl[2]), T_OPEN, f"slide off the T-bar {k + 1}", YAW_X)
        R.add(0.5, (pl[0], pl[1] - 0.02, pl[2] + 0.06), T_OPEN, f"rise from the T-bar {k + 1}", YAW_X)
    R.add(0.8, (X_HEAD - 0.04, SLIDE_TO - 0.07, TOP + 0.19), T_OPEN, "rise clear of the T-bar", YAW_X)
    R.add(1.5, PARK_R, OPEN_F, "park", 0.0)
    t_clamp = R.t
    # left: turn the handle clockwise (seen from above) in half-turn strokes, then back
    L.hold_to(t_clamp, "wait for the clamp")
    up = wz + 0.016
    L.add(1.5, (BX, SLIDE_TO, up), OPEN_F, "over the handle again", yaw_w)
    L.add(0.6, (BX, SLIDE_TO, wz), OPEN_F, "down to the wing", yaw_w)
    L.add(0.5, (BX, SLIDE_TO, wz), GRIP, "close on the wing", yaw_w)
    turned = 0.0
    yaw = -90.0
    def stroke(y0, y1, label):
        n = max(2, int(abs(y1 - y0) / 30))
        for i in range(1, n + 1):
            L.add(0.25, (BX, SLIDE_TO, wz), GRIP, f"{label} {i}", (y0 + (y1 - y0) * i / n) * D2R)
    def regrip(y_from, y_to, k):
        L.add(0.35, (BX, SLIDE_TO, wz), RELEASE, f"open {k}", y_from * D2R)
        L.add(0.3, (BX, SLIDE_TO, up), RELEASE, f"up {k}", y_from * D2R)
        L.add(0.7, (BX, SLIDE_TO, up), RELEASE, f"turn back {k}", y_to * D2R)
        L.add(0.3, (BX, SLIDE_TO, wz), RELEASE, f"down {k}", y_to * D2R)
        L.add(0.35, (BX, SLIDE_TO, wz), GRIP, f"close {k}", y_to * D2R)
    stroke(yaw, S_END, "turn clockwise 0")
    turned += (yaw - S_END) / 360.0
    k = 0
    while turned < FWD_TURNS - 1e-9:
        k += 1
        regrip(S_END, S_START, k)
        y1 = max(S_END, S_START - 360.0 * (FWD_TURNS - turned))
        stroke(S_START, y1, f"turn clockwise {k}")
        turned += (S_START - y1) / 360.0
        last = y1
    # back: counterclockwise half-turns
    back = 0.0
    yaw = last
    k2 = 0
    if yaw > S_END + 1e-6:            # the last forward stroke stopped short: regrip at the strokes' end yaw
        regrip(yaw, yaw - 180.0 if yaw - 180.0 >= -160.0 else yaw, "b0")
        yaw = yaw - 180.0 if yaw - 180.0 >= -160.0 else yaw
    while back < BACK_TURNS - 1e-9:
        k2 += 1
        y1 = min(S_START, yaw + 360.0 * (BACK_TURNS - back))
        stroke(yaw, y1, f"turn back {k2}")
        back += (y1 - yaw) / 360.0
        if back < BACK_TURNS - 1e-9:
            regrip(y1, y1 - 180.0, f"b{k2}")
            yaw = y1 - 180.0
        else:
            yaw = y1
    L.add(0.4, (BX, SLIDE_TO, wz), RELEASE, "let go of the handle", yaw * D2R)
    L.add(0.5, (BX, SLIDE_TO, wz + 0.014), RELEASE, "lift off the handle", yaw * D2R)
    L.add(0.6, (BX - 0.07, SLIDE_TO + 0.03, wz), RELEASE, "back away from the handle", yaw * D2R)
    L.add(1.5, PARK_L, OPEN_F, "park", 0.0)
    R.hold_to(L.t, "stay parked")
    times = {}
    for arm, a in (("left", L), ("right", R)):
        t = 0.0
        for w in a.w:
            times[(arm, w.label)] = (t, t + w.duration)
            t += w.duration
    return L.w, R.w, {"times": times, "fwd": turned, "back": back}


def demo(delay=0.0):
    """Right presses the tube's end even with the bar's top with its closed fingers, slides the yoke by its handle
    onto the 3/8 line and tightens the clamp screw in three strokes of its T-bar; left turns the handle clockwise in
    half-turn strokes, 7.5 turns (past the clutch's release on every seed, then two more), then 2.5 turns back; both
    park."""
    left, right, _ = plan(delay, run_seed())
    return {"left": left, "right": right}


def _replan_demo_after_reset():
    """WHY: inspect-robots resets the policy before the embodiment (inspect_robots/rollout.py), so yam_scripted would
    call demo() while run_seed() still holds the previous trial's seed and grip the handle where the yoke is not. This
    makes yam_scripted plan again on each trial's first action, after the reset has set the seed: the scripted demo
    reads the seed, as a privileged script may. It changes only the scripted demo policy, never the scene or the score."""
    try:
        from yam_arena import scripted
    except ImportError:
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_flare_replan", False):
        return
    reset, act = cls.reset, cls.act
    unset = object()

    def patched_reset(self, scene):
        self._flare_scene, self._flare_seed = scene, unset
        reset(self, scene)

    def patched_act(self, observation):
        if getattr(self, "_flare_seed", None) is unset:
            self._flare_seed = run_seed()
            reset(self, self._flare_scene)
        return act(self, observation)

    cls.reset, cls.act, cls._flare_replan = patched_reset, patched_act, True


_replan_demo_after_reset()
