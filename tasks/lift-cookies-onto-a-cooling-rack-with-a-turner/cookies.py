"""Three cookie stand-ins lie on a silicone mat on a rimless aluminium cookie sheet. A stainless offset turner lies on
its rest beside a wire cooling rack. Lift each cookie off the sheet with the turner and set it flat on the rack, in one
layer, without a gripper touching a cookie and without dropping one; then put the turner back on its rest.

The stand-ins are 3D-printed PLA discs at a baked cookie's size and weight (README "The cookies"), so trials reset
without food on the rig.
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Shape, body, on_reset, on_step, run_seed  # noqa: E402,F401
from motionkit import HAND, Plan, SagCorrector, as_grip, box_distance, down, install, mode, plans_out  # noqa: E402
from yam_arena.constants import FINGER_OPEN as FINGER_OPEN_  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 1600           # 80 s at 20 Hz
TOP = TABLE_TOP[SCENE]
MODES = ("demo", "nothing", "touch", "drop", "shove")
EXPECT = {"demo": 1.0, "nothing": 0.0, "touch": 0.5, "drop": 0.0, "shove": 0.0}
CHECK_SKIP = {("right", "turner"): ("take turner:", "rest turner:"),   # the hand closes on and lets go of the handle
              ("right", "cookie_1"): ("touch:",)}                          # mode touch presses a cookie by design

# Every number: [SOURCE] quoted in the README (saved page in src/), [ARITH] derived, [EST] estimate with a way to
# measure it in the README, [CHOICE] design choice. World: x forward from the arm bases, y left, z up.

# -- materials: (static, dynamic) friction of each material; PhysX averages the two touching materials ------------------
MATS = {"pla": (0.30, 0.25),          # [EST] printed PLA
        "silicone": (1.20, 1.00),     # [EST] silicone baking mat: PLA on it averages 0.75 / 0.63
        "steel": (0.20, 0.15),        # [EST] polished stainless blade and chrome rack wires: PLA on them 0.25 / 0.20
        "aluminium": (0.40, 0.35),    # [EST] the sheet's underside, on the table (0.5 default): 0.45 / 0.43
        "rubber": (1.20, 1.00)}       # [EST] the turner's soft-grip handle
TAN, ALU, MAT_C, STEEL_C, BLACK, WOOD = (0.78, 0.55, 0.30), (0.80, 0.80, 0.82), (0.86, 0.36, 0.28), \
    (0.62, 0.63, 0.66), (0.10, 0.10, 0.11), (0.55, 0.42, 0.30)

# -- the cookie stand-ins: PLA discs, frame at the bottom centre ---------------------------------------------------------
R = 0.038                        # radius: 76 mm across [CHOICE: a 3 in drop cookie, README "The cookies"]
THICK = 0.006                    # [CHOICE]
EDGE_IN, EDGE_UP = 0.004, 0.0015  # the underside edge is eased: the flat bottom ends 4 mm in and the edge rises 1.5 mm
#                                  [CHOICE: as a baked cookie's rounded edge; it lets the blade in]
PLA_RHO = 1240.0                 # kg/m3 [EST: printed solid; weigh each disc, README]
N_RING = 24


def _ring(r, z, n=N_RING):
    return [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n), z) for k in range(n)]


def _frustum_vol(r1, r2, h):
    return math.pi * h * (r1 * r1 + r1 * r2 + r2 * r2) / 3


EDGE_VOL = _frustum_vol(R - EDGE_IN, R, EDGE_UP)           # [ARITH]
TOP_VOL = math.pi * R * R * (THICK - EDGE_UP)              # [ARITH]
COOKIE_KG = PLA_RHO * (EDGE_VOL + TOP_VOL)                 # [ARITH: 32.9 g]


def cookie_shapes():
    pts = _ring(R - EDGE_IN, 0.0) + _ring(R, EDGE_UP)
    lo, hi = np.min(pts, axis=0), np.max(pts, axis=0)
    edge = Shape("hull", tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), TAN, material="pla",
                 points=tuple(pts), mass=PLA_RHO * EDGE_VOL)
    disc = Shape("cylinder", (0.0, 0.0, (EDGE_UP + THICK) / 2), (R, THICK - EDGE_UP), TAN, material="pla",
                 mass=PLA_RHO * TOP_VOL)
    return [edge, disc]


# -- the cookie sheet with its silicone mat (one loose body, frame at the sheet's underside centre) --------------------------
SHEET_XY = (0.350, -0.030)       # [CHOICE: inside the right arm's reach, checkkit]
SHEET = (0.260, 0.320, 0.0015)   # x, y, thickness [CHOICE: a rimless aluminium sheet; README "The objects"]
MAT = (0.240, 0.300, 0.00075)    # [CHOICE size; EST thickness, calipers]
SHEET_KG, MAT_KG = 0.34, 0.06    # [ARITH: 260 x 320 x 1.5 mm at 2,700 kg/m3; EST mat: kitchen scale]
MAT_TOP = TOP + SHEET[2] + MAT[2]
COOKIE_X = 0.360                 # cookie centres [CHOICE]
COOKIE_Y = (0.050, -0.040, -0.130)   # 90 mm apart: 14 mm between cookies [CHOICE: the 65 mm blade passes a
#                                  neighbour by 20 mm]


def sheet_shapes():
    return [Shape("box", (0.0, 0.0, SHEET[2] / 2), SHEET, ALU, material="aluminium", mass=SHEET_KG),
            Shape("box", (0.0, 0.0, SHEET[2] + MAT[2] / 2), MAT, MAT_C, material="silicone", mass=MAT_KG)]


# -- the cooling rack (loose): parallel wires along y on a frame and four feet; frame at its footprint's centre -----------
RACK_XY = (0.320, -0.380)        # [CHOICE: where the turner sets the cookies down, checkkit IK]
RACK = (0.200, 0.360)            # outside, x by y [CHOICE: a small rectangular rack, README]
RACK_H = 0.020                   # wire tops over the table [EST: ruler]
WIRE = 0.002                     # [EST]
WIRE_PITCH = 0.012               # [EST]
FRAME_W = 0.004                  # [EST]
FOOT = 0.006
RACK_KG = 0.25                   # [EST: kitchen scale]
RACK_TOP = TOP + RACK_H


def rack_boxes():
    """(lo, hi) of each rack part in the rack frame."""
    hx, hy, h = RACK[0] / 2, RACK[1] / 2, RACK_H
    out = []
    n = int((RACK[0] - 2 * FRAME_W) // WIRE_PITCH)
    x0 = -(n - 1) * WIRE_PITCH / 2
    for i in range(n):
        x = x0 + i * WIRE_PITCH
        out.append(((x - WIRE / 2, -hy + FRAME_W, h - WIRE), (x + WIRE / 2, hy - FRAME_W, h)))
    out += [((-hx, -hy, h - FRAME_W), (hx, -hy + FRAME_W, h)), ((-hx, hy - FRAME_W, h - FRAME_W), (hx, hy, h)),
            ((-hx, -hy + FRAME_W, h - FRAME_W), (-hx + FRAME_W, hy - FRAME_W, h)),
            ((hx - FRAME_W, -hy + FRAME_W, h - FRAME_W), (hx, hy - FRAME_W, h))]
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = sx * (hx - FOOT / 2), sy * (hy - FOOT / 2)
            out.append(((cx - FOOT / 2, cy - FOOT / 2, 0.0), (cx + FOOT / 2, cy + FOOT / 2, h - FRAME_W)))
    return out


def rack_shapes():
    boxes = rack_boxes()
    vol = sum(float(np.prod(np.subtract(hi, lo))) for lo, hi in boxes)
    return [Shape("box", tuple(((np.add(lo, hi)) / 2).tolist()), tuple(np.subtract(hi, lo).tolist()), STEEL_C,
                  material="steel", mass=RACK_KG * float(np.prod(np.subtract(hi, lo))) / vol) for lo, hi in boxes]


# -- the offset turner (loose): frame at the middle of the handle, where the hand holds it; x toward the blade's tip ------
OFF = 0.035                      # blade underside below the handle's middle [EST: ruler]
HANDLE = (-0.080, 0.020, 0.022, 0.014)   # x from, x to, width, height [EST: ruler]
BLADE = (0.035, 0.125, 0.065)    # x from, x to, width: a 90 x 65 mm blade [EST: ruler]
BLADE_T = (0.0010, 0.0003)       # thickness at the neck and at the tip edge, ground thin to the edge [EST: calipers]
HANDLE_KG, NECK_KG, BLADE_KG = 0.025, 0.008, 0.030   # [EST: kitchen scale]
TIP = np.array([BLADE[1], 0.0, -OFF])   # the blade's tip edge, underside, in the turner frame
REST_XY = (0.100, -0.600)        # the handle's middle on the rest [CHOICE: right of the rack, checkkit IK]
REST_BLOCK = (-0.075, -0.045, 0.030, OFF - HANDLE[3] / 2)   # x from, x to (turner frame), width, height [CHOICE]


def _hull_shape(pts, color, material, mass):
    pts = np.asarray(pts, float)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    return Shape("hull", tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, material=material,
                 points=tuple(tuple(map(float, p)) for p in pts), mass=float(mass))


def turner_shapes():
    x0, x1, w, h = HANDLE
    hw = BLADE[2] / 2
    blade = [(BLADE[0], -hw, -OFF), (BLADE[1], -hw, -OFF), (BLADE[1], hw, -OFF), (BLADE[0], hw, -OFF),
             (BLADE[0], -hw, -OFF + BLADE_T[0]), (BLADE[1], -hw, -OFF + BLADE_T[1]),
             (BLADE[1], hw, -OFF + BLADE_T[1]), (BLADE[0], hw, -OFF + BLADE_T[0])]
    nw, nt = 0.009, 0.0015
    a, b = (x1 - 0.004, -h / 2), (BLADE[0] + 0.006, -OFF + BLADE_T[0] - nt)
    neck = [(a[0], -nw, a[1] - nt), (b[0], -nw, b[1]), (b[0], nw, b[1]), (a[0], nw, a[1] - nt),
            (a[0], -nw, a[1]), (b[0], -nw, b[1] + nt), (b[0], nw, b[1] + nt), (a[0], nw, a[1])]
    return [Shape("box", ((x0 + x1) / 2, 0.0, 0.0), (x1 - x0, w, h), BLACK, material="rubber", mass=HANDLE_KG),
            _hull_shape(neck, STEEL_C, "steel", NECK_KG),
            _hull_shape(blade, STEEL_C, "steel", BLADE_KG)]


SPAWN_GAP = 0.0005
COOKIES = [body(f"cookie_{i}", (COOKIE_X, y, MAT_TOP + SPAWN_GAP), cookie_shapes(), MATS, color=TAN)
           for i, y in enumerate(COOKIE_Y)]
SHEET_BODY = body("sheet", (*SHEET_XY, TOP + SPAWN_GAP), sheet_shapes(), MATS, color=ALU)
RACK_BODY = body("rack", (*RACK_XY, TOP + SPAWN_GAP), rack_shapes(), MATS, color=STEEL_C)
TURNER = body("turner", (*REST_XY, TOP + OFF + SPAWN_GAP), turner_shapes(), MATS, color=STEEL_C)
_rb = REST_BLOCK
OBJECTS = [Box("rest", (REST_XY[0] + (_rb[0] + _rb[1]) / 2, REST_XY[1], TOP + _rb[3] / 2), WOOD, None, 0.0,
               (_rb[1] - _rb[0], _rb[2], _rb[3])),
           SHEET_BODY, RACK_BODY, TURNER, *COOKIES]
NAMES = [c.name for c in COOKIES]


# -- small maths --------------------------------------------------------------------------------------------------------
def _rot(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tilt_deg(st):
    return math.degrees(math.acos(min(1.0, abs(float(_rot(st.quat_xyzw)[2, 2])))))


def yaw_deg(st):
    r = _rot(st.quat_xyzw)
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def cookie_points(st, n=24):
    """World points on a cookie's outline (both rims) and its axis, for distances to the hands."""
    loc = np.array(_ring(R, EDGE_UP, n) + _ring(R, THICK, n) + _ring(R - EDGE_IN, 0.0, n)
                   + [(0.0, 0.0, 0.0), (0.0, 0.0, THICK)] + _ring(R / 2, THICK, 12) + _ring(R / 2, 0.0, 12))
    return loc @ _rot(st.quat_xyzw).T + np.asarray(st.pos, float)


# -- scoring ------------------------------------------------------------------------------------------------------------
TOUCH = 0.003                    # a hand point within 3 mm of a cookie counts as a touch [CHOICE: the HAND model's
#                                  3 mm sample step; README "Scoring"]
DROP_V = 0.60                    # m/s downward: a free fall of about 18 mm [ARITH: v^2 / 2g = 18 mm; CHOICE height]
DROP_H = 0.018                   # ... or 18 mm lower than 0.1 s before (two control steps), which the 20 Hz speed
#                                  samples can miss [ARITH: an 18 mm free fall takes 61 ms; the demo lowers a cookie
#                                  at most 8.5 mm in 0.1 s, PhysX trace]
FLAT_DEG = 10.0                  # [CHOICE: lying flat on the wires]
LAYER_TOL = 0.003                # underside within 3 mm of the wire tops: on the wires, not on another cookie (6 mm
#                                  thick) [CHOICE]
RACK_TOL = 0.002                 # the whole outline over the rack's outside outline, 2 mm grace [CHOICE: ruler]
REST_TOL, REST_YAW = 0.020, 15.0  # turner back on its rest: handle middle within 20 mm, pointing within 15 deg [CHOICE]
REST_Z = 0.010                   # ... and lying there (not held over it) [CHOICE]
SHEET_TOL = 0.030                # the sheet within 30 mm of its place [CHOICE: tape outline]
RACK_MOVE = 0.030                # the rack within 30 mm of its place [CHOICE: tape outline]
CARRY_UP = 0.010                 # a cookie counts as carried on the blade when its underside is within 3 mm of the
#                                  blade's top, its centre over the blade, and it is 10 mm over the mat [CHOICE]
ON_BLADE = 0.003
CLEAR = 0.030                    # every hand point 30 mm from the cookies and the turner at the end [CHOICE]
PER_COOKIE = 0.25                # [CHOICE]
_HANDS = {"left": None, "right": None}
SPOILED = {n: set() for n in NAMES}   # cookie -> reasons ("touched", "dropped"), latched for the trial
CARRIED = set()                          # cookies seen carried on the turner's blade, latched
UNREAD = [False]                         # a hand could not be read at some step: no full score
_Z = {n: [] for n in NAMES}              # each cookie's last three heights, one per control step


def set_hands(points):
    """Offline tests set the hands here; the simulator does it after every step (read_hands)."""
    _HANDS.update(points)


@on_reset
def _reset():
    _HANDS.update({"left": None, "right": None})
    for v in SPOILED.values():
        v.clear()
    for v in _Z.values():
        v.clear()
    CARRIED.clear()
    UNREAD[0] = False


def blade_frame(turner):
    """(R, origin) of the turner frame, and the blade's top-face height at a turner-frame x."""
    return _rot(turner.quat_xyzw), np.asarray(turner.pos, float)


def over_blade(st, turner, slack=0.0):
    """The cookie's centre over the blade's footprint (turner frame, grown by `slack`), and its underside's height over
    the blade's top face there (m)."""
    Rt, o = blade_frame(turner)
    c = Rt.T @ (np.asarray(st.pos, float) - o)
    inside = BLADE[0] - slack <= c[0] <= BLADE[1] + slack and abs(c[1]) <= BLADE[2] / 2 + slack
    u = (c[0] - BLADE[0]) / (BLADE[1] - BLADE[0])
    top = -OFF + BLADE_T[0] + u * (BLADE_T[1] - BLADE_T[0])
    return bool(inside), float(c[2] - top)


def note_step(objects, hands):
    """Latch every cookie a hand point comes within TOUCH of, and every cookie falling faster than DROP_V or DROP_H
    in two steps."""
    if any(hands.get(s) is None for s in ("left", "right")):
        UNREAD[0] = True
    turner = objects["turner"]
    for n in NAMES:
        st = objects[n]
        inside, gap = over_blade(st, turner)
        if inside and abs(gap) <= ON_BLADE and float(st.pos[2]) >= MAT_TOP + CARRY_UP:
            CARRIED.add(n)
        z = _Z[n]
        z.append(float(st.pos[2]))
        del z[:-3]
        if float(st.lin_vel[2]) < -DROP_V or (len(z) == 3 and z[0] - z[2] > DROP_H):
            SPOILED[n].add("dropped")
        pts = cookie_points(st)
        for side in ("left", "right"):
            h = hands.get(side)
            if h is None:
                continue
            h = np.asarray(h, float)
            near = np.all((h > pts.min(0) - 0.01) & (h < pts.max(0) + 0.01), axis=1)
            if near.any() and _cookie_distance(h[near], st).min() < TOUCH:
                SPOILED[n].add("touched")


def _cookie_distance(points, st):
    """Distance of world points to a cookie (a cylinder R by THICK on its frame, the eased edge ignored)."""
    loc = (np.asarray(points, float) - np.asarray(st.pos, float)) @ _rot(st.quat_xyzw)
    rad = np.hypot(loc[:, 0], loc[:, 1])
    dr = np.maximum(rad - R, 0.0)
    dz = np.maximum(np.maximum(-loc[:, 2], loc[:, 2] - THICK), 0.0)
    return np.hypot(dr, dz)


@on_step
def read_hands(env):
    try:
        _HANDS.update({side: HAND.from_env(env, side) for side in ("left", "right")})
    except Exception:  # noqa: BLE001  WHY: never stop the run; unread hands count as not clear
        _HANDS.update({"left": None, "right": None})
    try:
        from yam_arena import scene as _scene
        note_step(_scene.object_states(env, NAMES + ["turner"]), _HANDS)   # WHY: the carry latch needs the blade
    except Exception:  # noqa: BLE001  WHY: never stop the run; a failed read is retried next step
        pass


def on_rack(st, rack):
    """Lying flat on the rack's wires, in one layer, its whole outline over the rack's outline, still."""
    Rr = _rot(rack.quat_xyzw)
    c = Rr.T @ (np.asarray(st.pos, float) - np.asarray(rack.pos, float))
    inside = abs(c[0]) + R <= RACK[0] / 2 + RACK_TOL and abs(c[1]) + R <= RACK[1] / 2 + RACK_TOL
    on_wires = abs(float(st.pos[2]) - (float(rack.pos[2]) + RACK_H)) <= LAYER_TOL
    return bool(inside and on_wires and tilt_deg(st) <= FLAT_DEG and st.resting())


def resting_on_blade(st, turner):
    """The cookie lies on (or partly on) the blade: its footprint overlaps the blade's, grown by the radius, and its
    underside is within 5 mm of the blade's top there."""
    inside, gap = over_blade(st, turner, slack=R)
    return inside and gap <= 0.005


def good(objects):
    """Cookies that count: carried on the blade at some step, never touched or dropped, on the rack now and not lying
    on the blade."""
    t = objects["turner"]
    return [n for n in NAMES if n in CARRIED and not SPOILED[n] and on_rack(objects[n], objects["rack"])
            and not resting_on_blade(objects[n], t)]


def turner_on_rest(st):
    off = math.hypot(float(st.pos[0]) - REST_XY[0], float(st.pos[1]) - REST_XY[1])
    return bool(off <= REST_TOL and abs(yaw_deg(st)) <= REST_YAW and abs(float(st.pos[2]) - (TOP + OFF)) <= REST_Z
                and st.resting())


def hands_clear(objects):
    """Every hand point CLEAR from every cookie and from the turner's bounds."""
    t = objects["turner"]
    for pts in _HANDS.values():
        if pts is None:
            return False
        pts = np.asarray(pts, float)
        if box_distance(pts, t.low - CLEAR, t.high + CLEAR).min() <= 0.0:
            return False
        for n in NAMES:
            if _cookie_distance(pts, objects[n]).min() < CLEAR:
                return False
    return True


def sheet_in_place(st):
    return math.hypot(float(st.pos[0]) - SHEET_XY[0], float(st.pos[1]) - SHEET_XY[1]) <= SHEET_TOL


def rack_in_place(st):
    return math.hypot(float(st.pos[0]) - RACK_XY[0], float(st.pos[1]) - RACK_XY[1]) <= RACK_MOVE


def score(objects):
    """0.25 for each cookie carried there on the blade and lying flat on the rack in one layer, never touched by a
    gripper and never dropped. 1.0 with all three, the turner back on its rest, the sheet and rack in place, everything
    still and both hands 30 mm clear."""
    g = good(objects)
    if len(g) == len(NAMES) and turner_on_rest(objects["turner"]) and sheet_in_place(objects["sheet"]) \
            and rack_in_place(objects["rack"]) and all(objects[n].resting() for n in ("sheet", "rack")) \
            and hands_clear(objects) and not UNREAD[0]:
        return 1.0
    return round(PER_COOKIE * len(g), 6)


def success(objects):
    return score(objects) >= 1.0


def distance(objects):
    return 1.0 - score(objects)


def trace_row(env):
    """fastrun --trace columns: each cookie's latched state and height."""
    from yam_arena import scene as _scene
    st = _scene.object_states(env, NAMES + ["turner"])
    row = {}
    for n in NAMES:
        row[f"{n}_spoiled"] = "+".join(sorted(SPOILED[n])) or "-"
        row[f"{n}_vz"] = round(float(st[n].lin_vel[2]), 3)
        row[f"{n}_tilt"] = round(tilt_deg(st[n]), 1)
        row[f"{n}_carried"] = int(n in CARRIED)
    row["turner_tilt"] = round(tilt_deg(st["turner"]), 1)
    return row


# -- demo (motionkit; scene constants and the arm's own joint readings only) ----------------------------------------------
install(SagCorrector())
G0 = down(0.0).rot
PITCH_IN = 6.0                   # the blade's nose-down pitch while it slides under [CHOICE]
PITCH_OFF = 22.0                 # nose-down pitch that lets a cookie slide off: over the 14 deg of PLA on steel
#                                  (atan 0.25) [CHOICE; joint 4 limits it, checkkit IK]
YAW_OFF = -60.0                  # the blade's heading over the rack [CHOICE: the tilt then uses joint 5, checkkit IK]
PRESS = 0.0015                   # the tip is aimed this far into the mat (the arm is compliant: a light press) [CHOICE]
START_GAP = 0.010                # the tip starts this far behind a cookie's edge [CHOICE]
UNDER = 0.010                    # ... and ends this far past its far edge [CHOICE]
LIFT_Z = 0.060                   # the tip's height over the table while carrying [CHOICE]
OFF_TIP = (0.290, (-0.250, -0.360, -0.470))   # tip x and y over the rack for each cookie [CHOICE: each lands 10 to 35 mm
#                                  past the tip (PhysX trace), on the rack, checked in the sim]
OFF_UP = 0.004                   # tip over the wires when tilted [CHOICE]
DROP_UP = 0.040                  # mode drop: tilted off this high instead [CHOICE]
SHOVE_UP = 0.005                 # mode shove: the blade held this far over the mat [CHOICE]
FORCE = 20.0                     # N a finger on the handle [CHOICE: under the 40 N per-finger limit]
HIGH = 0.12                      # TCP height of an empty-hand transit [CHOICE]


def _rz(a):
    a = math.radians(a)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(p):
    p = math.radians(p)
    c, s = math.cos(p), math.sin(p)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def tool_rot(pitch, yaw):
    """World rotation of the turner frame: pitched nose-down `pitch`, heading `yaw` (deg)."""
    return _rz(yaw) @ _ry(pitch)


def grip(pitch, yaw):
    return as_grip(tool_rot(pitch, yaw) @ G0)


def tcp_for_tip(tip, pitch, yaw):
    """TCP (handle middle) that puts the blade's tip edge at `tip`."""
    return np.asarray(tip, float) - tool_rot(pitch, yaw) @ TIP


def _take_turner(p):
    rest = (REST_XY[0], REST_XY[1], TOP + OFF)
    p.move((rest[0], rest[1], TOP + HIGH), grip=grip(0, 0), label="take turner: over")
    p.pick(rest, h=0.0, width=HANDLE[2], grip=grip(0, 0), above=HIGH - OFF, lift=0.06, force=FORCE,
           label="take turner")


def _lift_cookie(p, y, mode_name):
    tip0 = np.array([COOKIE_X - R - START_GAP, y, MAT_TOP - PRESS])
    if mode_name == "shove":
        tip0[2], pitch = MAT_TOP + SHOVE_UP, 0.0
    else:
        pitch = PITCH_IN
    p.move(tcp_for_tip(tip0 + [0, 0, 0.040], pitch, 0), grip=grip(pitch, 0), label="scoop: over")
    p.hold(0.8, "scoop: line up")
    p.move(tcp_for_tip(tip0, pitch, 0), t=1.5, label="scoop: down", contact=True)
    p.hold(0.5, "scoop: settle", contact=True)
    stroke = 2 * R + START_GAP + UNDER
    p.move(tcp_for_tip(tip0 + [stroke, 0, 0], pitch, 0), t=3.0, label="scoop: slide", contact=True)
    p.move(p.pos + [0.0, 0.0, 0.004], grip=grip(0, 0), t=1.0, label="scoop: level", contact=True)
    p.move(p.pos + [0.0, 0.0, LIFT_Z - 0.004], t=1.5, label="carry: up")


def _set_on_rack(p, k, mode_name):
    up = DROP_UP if mode_name == "drop" else OFF_UP
    tip = np.array([OFF_TIP[0], OFF_TIP[1][k], RACK_TOP + up])
    p.move(tcp_for_tip(tip + [0, 0, 0.030], 0, YAW_OFF), grip=grip(0, YAW_OFF), t=3.0, label="carry: across")
    p.hold(0.6, "carry: steady")
    p.move(tcp_for_tip(tip + [0, 0, 0.008], 0, YAW_OFF), t=1.0, label="set: down")
    p.move(tcp_for_tip(tip, PITCH_OFF, YAW_OFF), grip=grip(PITCH_OFF, YAW_OFF), t=2.0, label="set: tilt", contact=True)
    p.hold(1.0, "set: slide off", contact=True)
    back = _rz(YAW_OFF) @ np.array([-0.040 if mode_name != "drop" else 0.0, 0.0, 0.0])
    # WHY not in mode drop: 50 mm higher, drawing back puts joint 4 on its limit (checkkit IK); nothing is under it
    rise = 0.0 if mode_name == "drop" else 1.0
    p.move(p.pos + back + [0.0, 0.0, 0.020 * rise], t=1.0, label="set: back")
    p.move(p.pos + [0.0, 0.0, 0.030 * rise], grip=grip(0, YAW_OFF), t=1.0, label="set: level")


def _rest_turner(p):
    rest = np.array([REST_XY[0], REST_XY[1], TOP + OFF])
    p.move(rest + [0.0, 0.0, 0.060], grip=grip(0, 0), label="rest turner: over")
    p.hold(0.6, "rest turner: line up")
    p.move(rest + [0.0, 0.0, 0.001], t=1.5, label="rest turner: down", contact=True)
    p.hold(0.4, "rest turner: set", contact=True)
    p.move(p.pos, t=0.6, finger=p.pre, label="rest turner: let go", contact=True)
    p.move(p.pos + [0.0, 0.0, 0.10], t=1.5, label="rest turner: clear")


TIP_BELOW = 0.0129               # closed fingertips reach this far below the TCP [ARITH: checkkit clearance, an earlier
#                                  plan put the tips 6.6 mm into the table with the TCP 6.25 mm over it]
TOUCH_GAP = 0.001                # mode touch: fingertips 1 mm over the cookie's top, inside TOUCH, not pressing [CHOICE]


def _touch(p):
    """Mode touch: before taking the turner, the empty closed hand comes down onto the middle cookie, as a hand
    straightening it would. (After the demo it could not: success ends the trial first.)"""
    c = np.array([COOKIE_X, COOKIE_Y[1], MAT_TOP + THICK + TIP_BELOW + TOUCH_GAP])
    p.move(c + [0.0, 0.0, 0.060], grip=down(0), finger=0.0, label="touch: over")
    p.hold(0.4, "touch: line up")
    p.move(c, t=1.2, label="touch: down", contact=True)
    p.hold(0.3, "touch: hold", contact=True)
    p.move(c + [0.0, 0.0, 0.060], t=1.0, label="touch: up")
    p.move(p.pos, finger=FINGER_OPEN_, t=0.5, label="touch: open")


def plan(seed=None, mode_name="demo"):
    if mode_name == "nothing":
        return {}
    p = Plan("right")
    if mode_name == "touch":
        _touch(p)
    _take_turner(p)
    for k, y in enumerate(COOKIE_Y):
        _lift_cookie(p, y, mode_name)
        _set_on_rack(p, k, mode_name)
    _rest_turner(p)
    return plans_out(p)


def demo():
    return plan(run_seed(), mode())
