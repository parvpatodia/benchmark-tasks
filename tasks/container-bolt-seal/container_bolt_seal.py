"""A loaded shipping container's right door stands across the far side of the table, its face toward the robot
(blue). Two steel hasps stick out of it, each a pair of upright ears with one round 16 mm seal hole through both,
the holes running left to right. One is the cam keeper at the bottom of the door's centre-most locking bar (the grey
bar rises from it); the other is the door handle's retainer (a yellow handle tab sits between its ears). A red ISO 17712
bolt seal lies on the table in two parts: the bolt (a headed pin) in a black tray on the left, and the
barrel (the locking body, open end toward the left) on the right. Seal the door as the CTPAT seal procedure says: on
the cam keeper, not on the door handle. The bolt goes through both ears and the barrel closes on its tip from the other
side, seated against the bolt's shoulder. A seal closes for good: a barrel seated on a bolt not through the
keeper is spent. Then let go and move both grippers at least 10 mm from the seal.
"""

import functools
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, body, hull, mechanism, on_reset, on_step  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
D2R = math.pi / 180

# -- the seal: J. J. Keller HBSCP8 coated-pin bolt seal, spec sheet 430272-112 (2022) [SOURCE] ------------------------
PIN_L = 0.0870            # pin 87.0 mm long, head included [SOURCE]
HEAD_D = 0.0222           # pin 22.2 x 22.3 mm across (the head) [SOURCE]; drawn round, 22.2 mm
OPER_L = 0.0806           # "operational length" 80.6 mm: read here as the pin below the head [JUDGEMENT]
HEAD_L = PIN_L - OPER_L   # 6.4 mm [ARITH]
STEEL_R = 0.0040          # 8.0 mm steel [SOURCE]
COAT_R = 0.0059           # 11.8 mm over the ABS coating [SOURCE]
ASSEMBLED_L = 0.0959      # overall assembled length 95.9 mm [SOURCE]
BARREL_L = 0.0316         # barrel 31.6 mm long [SOURCE]
BARREL_R = 0.0108         # barrel 21.6 mm across [SOURCE]
ENGAGE = PIN_L + BARREL_L - ASSEMBLED_L   # 22.7 mm of pin inside the barrel when closed [ARITH]
TIP_L = ENGAGE            # the bare steel tip that enters the barrel [JUDGEMENT: the sheet does not draw it]
SHOULDER_Y = -(OPER_L - TIP_L)            # bolt frame: where the coating ends (-57.9 mm from the head) [ARITH]
SHOULDER_CH = COAT_R - STEEL_R            # a 45 deg taper from the tip to the coating, 1.9 mm long [CHOICE]
TIP_CH = 0.0015                           # the tip's end chamfer, 45 deg [CHOICE]
BORE_R = 0.0045           # the barrel's bore for the 8 mm tip: 0.5 mm radial play [CHOICE]. WHY not more: a released
#                           barrel sags by the play onto the tip and slides that far back down the 45 deg taper
MOUTH_CH = 0.0020         # the barrel's mouth chamfer, 45 deg [CHOICE]
BORE_DEPTH = 0.0270       # [CHOICE] deeper than the 25.6 mm the tip reaches, so the shoulder is the stop [ARITH test]
BOLT_KG = {"head": 0.0050, "shank": 0.0265, "taper": 0.0005, "tip": 0.0080, "point": 0.0004}   # 40.4 g [EST: steel
#   8 mm x 80.6 mm is 31.8 g, the ABS coat 3.6 g, the head 5 g; the sheet gives no mass]
BARREL_WALL_KG, BARREL_CH_KG, BARREL_CAP_KG = 0.0012, 0.0001, 0.0098   # 12 walls + 12 chamfers + cap = 25.4 g [EST]
N_SIDES = 12              # the barrel and the hasp holes are 12-sided; faces (not corners) at the stated radii

# -- the door and its hasps (our prop; real hasps vary: every number here is [CHOICE] unless marked) ------------------
HX = 0.345                # x of both hole axes
DOOR_X = 0.410            # the door's face
EAR_X0 = 0.325            # the ears' free edge (ears 85 mm deep: the gripper clears the bar, FK test)
EAR_T = 0.005             # each ear 5 mm steel
EAR_GAP = 0.008           # between the two ears
EAR_HALF_H = 0.022        # ears 44 mm tall around the hole
HOLE_R = 0.0080           # 16 mm seal hole: 2.1 mm radial play on the coated pin
KEEPER = (0.020, TOP + 0.060)   # (y, z) of the correct hasp's hole: the cam keeper, bottom of the centre-most bar
HANDLE = (-0.200, TOP + 0.100)  # the handle retainer's hole: the wrong place
HASPS = {"keeper": KEEPER, "handle": HANDLE}
BAR_R = 0.0110            # the locking bar, 22 mm [CHOICE: real bars are thicker; a 27 mm bar meets the open fingers]
DOOR_Y = (-0.30, 0.30)
DOOR_H = 0.20
BAR_TOP = TOP + 0.125     # the bar's drawn stub ends here. WHY: the wrist passes over it at 0.14-0.16 (FK test)

# -- start: the bolt in its tray, the barrel on the table ---------------------------------------------------------------
AXIS_H = HEAD_D / 2 + 0.0010          # the bolt's axis over the table: its head 1 mm clear
BOLT_START = (0.240, 0.260, TOP + AXIS_H)   # the bolt frame (head underside centre), head toward +y
BARREL_START = (0.240, -0.070, TOP + BARREL_R)   # the barrel frame (mouth centre), mouth toward +y
SADDLES = ((-0.050, -0.040, COAT_R), (-0.078, -0.066, STEEL_R))   # bolt-frame y ranges and the radius they carry

RED, BLUE, STEEL, BLACK, YELLOW, GREY = ((0.85, 0.08, 0.08), (0.10, 0.22, 0.55), (0.45, 0.47, 0.50),
                                         (0.08, 0.08, 0.08), (0.95, 0.80, 0.05), (0.55, 0.55, 0.58))
MATERIALS = {"default": (1.2, 1.0),   # the scene's friction on its own assets (rules.md 3)
             "seal": (0.35, 0.30),    # ABS on steel [EST]
             "steel": (0.30, 0.25)}   # [EST]
AX = np.array([0.0, -1.0, 0.0])        # bolt frame: head to tip; barrel frame: mouth to cap is the same -y


def _box(lo, hi, color, material="default", mass=0.0):
    c = tuple(float(v) for v in (np.asarray(lo, float) + np.asarray(hi, float)) / 2)
    s = tuple(float(v) for v in np.asarray(hi, float) - np.asarray(lo, float))
    return Shape("box", c, s, color, material=material, mass=mass)


def _ycyl(x, z, y0, y1, r, color, material="default", mass=0.0):
    """A solid cylinder along y from y0 to y1."""
    return Shape("cylinder", (float(x), (y0 + y1) / 2, float(z)), (r, abs(y1 - y0)), color, axis="Y",
                 material=material, mass=mass)


def _ring_y(pts_xz, y, axis_sign=1.0):
    """3-D ring points at height y from (x, z) points, wound counter-clockwise about +y times axis_sign."""
    p = np.asarray(pts_xz, float)
    area = 0.0
    for (x0, z0), (x1, z1) in zip(p, np.roll(p, -1, axis=0)):
        area += z0 * x1 - x0 * z1            # positive: counter-clockwise about +y (x cross z = -y)
    if area * axis_sign < 0:
        p = p[::-1]
    return [(float(x), float(y), float(z)) for x, z in p]


def prism_y(pts_xz, y0, y1, color, material="default", mass=0.0, pts_xz_top=None):
    """A convex hull between ring y0 (pts_xz) and ring y1 (pts_xz_top or the same), y0 < y1."""
    lo = _ring_y(pts_xz, y0)
    hi = _ring_y(pts_xz if pts_xz_top is None else pts_xz_top, y1)
    return hull(lo + hi, color, material=material, mass=mass)


def _vertex_r(face_r):
    return face_r / math.cos(math.pi / N_SIDES)


def sector_angles(k):
    """Sector k of N_SIDES: centred on k * 30 deg, so faces point along +-x and +-z."""
    c = 2 * math.pi * k / N_SIDES
    return c - math.pi / N_SIDES, c + math.pi / N_SIDES


def _clip_poly(poly, a, b):
    """Clip a convex polygon (list of 2-D points) to the half-plane a . p <= b."""
    out = []
    for i in range(len(poly)):
        p, q = poly[i], poly[(i + 1) % len(poly)]
        fp, fq = a @ p - b, a @ q - b
        if fp <= 1e-15:
            out.append(p)
        if fp * fq < 0:
            out.append(p + (q - p) * (fp / (fp - fq)))
    return out


def hole_sector(cx, cz, k, face_r, rect):
    """The part of an ear (rectangle rect = (x0, x1, z0, z1)) in sector k around a hole of face radius face_r: the
    rectangle cut by the sector's two radial lines and the hole face's chord. Convex, as the intersection of convex
    sets. Returns (x, z) points."""
    x0, x1, z0, z1 = rect
    poly = [np.array(p) for p in ((x0, z0), (x1, z0), (x1, z1), (x0, z1))]
    a0, a1 = sector_angles(k)
    c = np.array([cx, cz])
    for a, sign in ((a0, -1.0), (a1, 1.0)):    # keep the side of each radial line inside the sector
        n = np.array([-math.sin(a), math.cos(a)]) * sign
        poly = _clip_poly(poly, n, n @ c)
    mid = (a0 + a1) / 2
    u = np.array([math.cos(mid), math.sin(mid)])
    poly = _clip_poly(poly, -u, -(u @ c + face_r))   # outside the hole's face (the chord)
    return poly


def ear_shapes(yc, zc, color=STEEL):
    """Two ears with a 12-sided hole of face radius HOLE_R through both, the hasp centred at y = yc."""
    out = []
    rect = (EAR_X0, DOOR_X, zc - EAR_HALF_H, zc + EAR_HALF_H)
    for y0 in (yc + EAR_GAP / 2, yc - EAR_GAP / 2 - EAR_T):
        for k in range(N_SIDES):
            out.append(prism_y(hole_sector(HX, zc, k, HOLE_R, rect), y0, y0 + EAR_T, color, "steel"))
    return out


def ear_planes(name):
    """Mid-planes (y) of a hasp's two ears, the +y ear first."""
    yc, _ = HASPS[name]
    return (yc + EAR_GAP / 2 + EAR_T / 2, yc - EAR_GAP / 2 - EAR_T / 2)


def door_shapes():
    out = [_box((DOOR_X, DOOR_Y[0], TOP), (DOOR_X + 0.004, DOOR_Y[1], TOP + DOOR_H), BLUE)]
    for name, (yc, zc) in HASPS.items():
        out += ear_shapes(yc, zc)
    ky, kz = KEEPER
    z0 = kz + EAR_HALF_H + 0.002
    out.append(Shape("cylinder", (DOOR_X - BAR_R, ky, (z0 + BAR_TOP) / 2), (BAR_R, BAR_TOP - z0), GREY))
    hy, hz = HANDLE                                  # the handle tab between the retainer's ears, over the hole
    out.append(_box((0.345, hy - EAR_GAP / 2 + 0.0005, hz + 0.012), (DOOR_X, hy + EAR_GAP / 2 - 0.0005, hz + 0.030),
                    YELLOW))
    for y0, y1, r in SADDLES:                        # the seal tray: V saddles under the bolt, world frame
        out += saddle_shapes(y0, y1, r)
    return out


def saddle_shapes(y0, y1, r):
    """Two blocks forming a 90 deg V that carries a pin of radius r on the bolt's start axis. In the axis frame the V
    face is z = |x| - sqrt(2) r, tangent to the pin at 45 deg; each block's top is flat at the tangent height -r/sqrt(2),
    3 mm or more under the finger tips (1 mm under the axis)."""
    bx, by, bz = BOLT_START
    xi, xf, zt = 0.0015, 0.012, -r / math.sqrt(2)
    out = []
    for s in (-1.0, 1.0):
        pts = [(xi, TOP - bz), (xf, TOP - bz), (xf, zt), (-zt, zt), (xi, xi - math.sqrt(2) * r)]
        out.append(prism_y([(bx + s * x, bz + z) for x, z in pts], by + y0, by + y1, BLACK))
    return out


def _yhull_disc(r_face, y, n=N_SIDES):
    rv = _vertex_r(r_face)
    return [(rv * math.cos(a), rv * math.sin(a)) for a in (sector_angles(k)[0] for k in range(n))]


def bolt_shapes():
    m = BOLT_KG
    taper0 = SHOULDER_Y - SHOULDER_CH            # where the taper meets the bare tip
    out = [_ycyl(0, 0, 0.0, HEAD_L, HEAD_D / 2, RED, "seal", m["head"]),
           _ycyl(0, 0, SHOULDER_Y, 0.0, COAT_R, RED, "seal", m["shank"]),
           prism_y(_yhull_disc(STEEL_R, 0, 16), taper0, SHOULDER_Y, RED, "seal", m["taper"],
                   pts_xz_top=_yhull_disc(COAT_R, 0, 16)),
           _ycyl(0, 0, -OPER_L + TIP_CH, taper0, STEEL_R, STEEL, "seal", m["tip"]),
           prism_y(_yhull_disc(STEEL_R - TIP_CH, 0, 16), -OPER_L, -OPER_L + TIP_CH, STEEL, "seal", m["point"],
                   pts_xz_top=_yhull_disc(STEEL_R, 0, 16))]
    return out


def barrel_shapes():
    out = []
    rv_out = _vertex_r(BARREL_R)
    for k in range(N_SIDES):
        a0, a1 = sector_angles(k)

        def wedge(r_in):
            ri = _vertex_r(r_in)
            return [(rv_out * math.cos(a0), rv_out * math.sin(a0)), (rv_out * math.cos(a1), rv_out * math.sin(a1)),
                    (ri * math.cos(a1), ri * math.sin(a1)), (ri * math.cos(a0), ri * math.sin(a0))]
        out.append(prism_y(wedge(BORE_R), -BORE_DEPTH, -MOUTH_CH, RED, "seal", BARREL_WALL_KG))
        out.append(prism_y(wedge(BORE_R), -MOUTH_CH, 0.0, RED, "seal", BARREL_CH_KG,
                           pts_xz_top=wedge(BORE_R + MOUTH_CH)))
    out.append(_ycyl(0, 0, -BARREL_L, -BORE_DEPTH, BARREL_R, RED, "seal", BARREL_CAP_KG))
    return out


def _door():
    base = Link("door", 10.0, tuple(door_shapes()))   # fixed: its mass is not used
    # WHY the tag: Isaac Lab's articulation setup matches its default joint pattern ".*" and faults when an
    # articulation has no joints (first sim run, 2026-11-13: "Not all regular expressions are matched"). The tag is a
    # 10 mm block 16 mm behind the door's back face, out of both arms' reach, on a stiff 1 mm prismatic joint; it
    # touches nothing and nothing reads it.
    tag = Link("tag", 0.02, (_box((DOOR_X + 0.020, -0.005, TOP + 0.200), (DOOR_X + 0.030, 0.005, TOP + 0.210), BLUE,
                                  mass=0.02),))
    joints = [Joint("tag", "prismatic", "tag", (DOOR_X + 0.025, 0.0, TOP + 0.205), "X", -0.0005, 0.0005,
                    stiffness=2000.0, damping=20.0)]
    return mechanism("door", (0.0, 0.0, 0.0), base, [tag], joints, MATERIALS, self_collide=False)


DOOR = _door()
BOLT = body("bolt", BOLT_START, bolt_shapes(), MATERIALS, color=RED)
BARREL = body("barrel", BARREL_START, barrel_shapes(), MATERIALS, color=RED)
OBJECTS = [DOOR, BOLT, BARREL]

# -- scoring ----------------------------------------------------------------------------------------------------------
# bolt-frame y of the barrel's mouth when seated: the bore's entry edge (radius BORE_R, MOUTH_CH inside the mouth)
# rests on the 45 deg taper where the taper's radius is BORE_R, (BORE_R - STEEL_R) above its small end. -57.3 mm [ARITH]
SEAT_Y = (SHOULDER_Y - SHOULDER_CH) + (BORE_R - STEEL_R) + MOUTH_CH
SEAT_TOL = 0.0015          # m: the mouth within 1.5 mm short of the seat: a released barrel sags 0.5 mm and slides
#                           back as far down the taper [ARITH]; the real lock engages at full depth [JUDGEMENT]
SEAT_OVER = 0.0005         # m: contact penetration past the seat
COAX_R = 0.0015            # m: the mouth centre off the bolt's axis (the bore's 0.5 mm play, plus 1.0 mm)
COAX_DEG = 6.0             # deg between the axes (the 0.5 mm play over 23 mm allows 1.3 deg [ARITH]; contact slack)
THROUGH_SLACK = 0.0005     # m: past the hole's radial play (contact penetration)
FAIL_SLACK = 0.0020        # m: the same for the latched failures. WHY: the 12-sided hole's corners give 2.38 mm of play,
#                           so a seated seal pressed 0.3 mm into a corner would read "not through" and latch "spent"
CLEAR_MIN = 0.010          # m: both grippers this far from the bolt and the barrel
STILL_V, STILL_W = 0.005, 0.05   # m/s, rad/s: from the pose change between steps; the turn is of the axis only
LOST_Z = TOP - 0.030       # a seal part this low has left the table


def _rot3(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def axis_of(o):
    """World direction of an object's local -y (bolt: head to tip; barrel: mouth to cap)."""
    return _rot3(o.quat_xyzw) @ AX


def pin_radius(y):
    """The pin's radius at bolt-frame y (0 at the head's underside, -OPER_L at the tip's end)."""
    t0 = SHOULDER_Y - SHOULDER_CH
    if y >= SHOULDER_Y:
        return COAT_R
    if y >= t0:
        return STEEL_R + (y - t0)
    if y >= -OPER_L + TIP_CH:
        return STEEL_R
    return STEEL_R - (-OPER_L + TIP_CH - y)


def crossing(objects, name, plane_y):
    """Where the bolt's axis crosses the ear mid-plane y = plane_y: (bolt-frame y, radial offset from the hole axis),
    or None when the axis runs within 30 deg of the plane."""
    b = objects["bolt"]
    a = axis_of(b)
    if abs(a[1]) < math.sin(60 * D2R):
        return None
    p0 = np.asarray(b.pos, float)
    s = (plane_y - p0[1]) / a[1]                 # distance along the axis from the head's underside
    p = p0 + s * a
    _, zc = HASPS[name]
    return -s, float(math.hypot(p[0] - HX, p[2] - zc))


def through(objects, name, slack=THROUGH_SLACK):
    """The pin passes through both ears of hasp ``name``: each ear's mid-plane is crossed by the pin (between the
    head and the tip's end) within the hole's radial play there, plus ``slack``."""
    for plane in ear_planes(name):
        c = crossing(objects, name, plane)
        if c is None:
            return False
        y, off = c
        if not (-OPER_L < y < 0.0) or off > HOLE_R - pin_radius(y) + slack:
            return False
    return True


def barrel_in_bolt(objects):
    """(mouth centre in the bolt frame, angle in deg between the barrel's and the bolt's axes)."""
    b, c = objects["bolt"], objects["barrel"]
    rb = _rot3(b.quat_xyzw)
    m = rb.T @ (np.asarray(c.pos, float) - np.asarray(b.pos, float))
    cosang = float(np.clip(axis_of(b) @ axis_of(c), -1.0, 1.0))
    return m, math.degrees(math.acos(cosang))


def coaxial(objects):
    m, ang = barrel_in_bolt(objects)
    return bool(math.hypot(m[0], m[2]) <= COAX_R and ang <= COAX_DEG)


def seated(objects):
    """The barrel closed on the bolt: coaxial and its mouth at the shoulder."""
    m, _ = barrel_in_bolt(objects)
    return bool(coaxial(objects) and SEAT_Y - SEAT_TOL <= m[1] <= SEAT_Y + SEAT_OVER)


def started(objects):
    """The barrel on the tip, short of the seat: coaxial and its mouth between the tip's end and the seat."""
    m, _ = barrel_in_bolt(objects)
    return bool(coaxial(objects) and -OPER_L < m[1] < SEAT_Y - SEAT_TOL)


def sealed(objects, name):
    """Closed through both ears of ``name``, the ears between the head and the barrel."""
    if not (through(objects, name) and seated(objects)):
        return False
    return all(SEAT_Y < crossing(objects, name, p)[0] < 0.0 for p in ear_planes(name))


def failure(objects):
    """Latched by update(): a seal closed off any hasp is spent; one closed on the handle retainer is in the wrong place
    (cutting it is the only way off); a part off the table is lost."""
    if any(float(objects[n].pos[2]) < LOST_Z for n in ("bolt", "barrel")):
        return "lost"
    if seated(objects):
        if through(objects, "handle", FAIL_SLACK):
            return "wrong hasp"
        if not through(objects, "keeper", FAIL_SLACK):
            return "spent"
    return None


def pose(objects):
    return tuple((np.asarray(objects[n].pos, float), axis_of(objects[n])) for n in ("bolt", "barrel"))


def rates(prev, now, dt):
    """(fastest frame-origin speed m/s, fastest axis turn rate rad/s) of the bolt and the barrel between two steps.
    WHY the axis only: both parts are round about their axes and PhysX has no rolling resistance (rules.md 3), so a
    spin about the axis is not motion that matters; the reported velocities are noisy (toolchanger round 1)."""
    v = w = 0.0
    for (p0, a0), (p1, a1) in zip(prev, now, strict=True):
        v = max(v, float(np.linalg.norm(p1 - p0)) / dt)
        w = max(w, math.acos(float(np.clip(a0 @ a1, -1.0, 1.0))) / dt)
    return v, w


def still(r):
    return bool(r is not None and r[0] <= STILL_V and r[1] <= STILL_W)


# -- the grippers, read every control step (scene keys left_robot / right_robot) ------------------------------------
HAND_LINKS = ("gripper", "tip_left", "tip_right")
_HANDS = {"hands": None, "steps": 0, "error": None}


@functools.lru_cache(maxsize=1)
def hand_hulls():
    """Each hand link's URDF collision hulls in its own frame: (link, vertices, distinct outward face normals,
    bounding-sphere centre, radius)."""
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
    return tuple(out)


def _np(x):
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


@on_step
def read_hands(env):
    """Both hands' collision hulls in the task frame (world less the env origin). Link poses are (x, y, z, qx, qy,
    qz, qw), as yam_arena.runtime.link_poses reads them."""
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


_CLOCK = {"dt": None}


@on_step
def read_clock(env):
    """This control step's length (s), for still(). Unread means not still."""
    try:
        _CLOCK["dt"] = float(env.unwrapped.step_dt)
    except Exception:  # WHY: never stop the run; no step length means no full score
        _CLOCK["dt"] = None


_PRISM = 16
_YAXES = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]]).T   # columns x, z, y: a cylinder along y


@functools.lru_cache(maxsize=1)
def _keep_out_local():
    """Keep-outs in each part's frame: solid cylinders along y that contain every shape of the part."""
    head = (np.array([0.0, HEAD_L / 2, 0.0]), (HEAD_D / 2, HEAD_L))
    pin = (np.array([0.0, -OPER_L / 2, 0.0]), (_vertex_r(COAT_R), OPER_L))   # the taper's 12-gon corners
    barrel = (np.array([0.0, -BARREL_L / 2, 0.0]), (_vertex_r(BARREL_R), BARREL_L))
    return {"bolt": (head, pin), "barrel": (barrel,)}


def keep_outs(objects):
    """(world centre, world axes, (radius, length)) of each keep-out cylinder."""
    out = []
    for name, parts in _keep_out_local().items():
        o = objects[name]
        r, p = _rot3(o.quat_xyzw), np.asarray(o.pos, float)
        out += [(r @ c + p, r @ _YAXES, d) for c, d in parts]
    return out


def _cyl_dist(points, centre, rot, radius, length):
    """Signed distance from points to a solid cylinder along rot's third column."""
    local = (points - centre) @ rot
    dr = np.hypot(local[:, 0], local[:, 1]) - radius
    dz = np.abs(local[:, 2]) - length / 2
    return np.hypot(np.maximum(dr, 0.0), np.maximum(dz, 0.0)) + np.minimum(np.maximum(dr, dz), 0.0)


def _convex(c, a, d):
    """A keep-out cylinder as the 16-sided prism around it (so the gap to it is a lower bound)."""
    rr, hl = d[0] / math.cos(math.pi / _PRISM), d[1] / 2
    ang = [2 * math.pi * (k + 0.5) / _PRISM for k in range(_PRISM)]
    loc = np.array([(rr * math.cos(u), rr * math.sin(u), z) for z in (-hl, hl) for u in ang])
    nl = np.array([(math.cos(2 * math.pi * k / _PRISM), math.sin(2 * math.pi * k / _PRISM), 0.0)
                   for k in range(_PRISM)] + [(0.0, 0.0, 1.0)])
    return loc @ a.T + c, nl @ a.T, c, float(np.linalg.norm(loc, axis=1).max())


def _sat_gap(hv, hn, kv, kn):
    """A lower bound on the distance between two convex sets (vertices, normals): the largest gap of their
    projections on either set's face normals; negative when they overlap on every one of those axes."""
    ax = np.vstack([hn, kn])
    ph, pk = hv @ ax.T, kv @ ax.T
    return float(np.max(np.maximum(pk.min(axis=0) - ph.max(axis=0), ph.min(axis=0) - pk.max(axis=0))))


def hand_gap(objects, hands=None, need=None):
    """Smallest distance (m) from the hands to the seal's keep-outs, -inf when the hands were not read. ``hands`` is
    an (n, 3) array of points (exact distance) or the hulls read_hands stores (a lower bound: bounding spheres, then
    separating axes). With ``need``, the search stops at the first pair under it."""
    hs = _HANDS["hands"] if hands is None else hands
    if hs is None or len(hs) == 0:
        return -math.inf
    ks = keep_outs(objects)
    if isinstance(hs, np.ndarray):
        return float(min(_cyl_dist(hs, c, a, *d).min() for c, a, d in ks))
    hc = np.array([h[2] for h in hs])
    hr = np.array([h[3] for h in hs])
    refine = CLEAR_MIN + 0.01 if need is None else need
    best = math.inf
    for kv, kn, kc, kr in (_convex(*k) for k in ks):
        g = np.linalg.norm(hc - kc, axis=1) - hr - kr
        for i in np.flatnonzero(g < refine):
            g[i] = max(g[i], _sat_gap(hs[i][0], hs[i][1], kv, kn))
            if need is not None and g[i] < need:
                return float(g[i])
        best = min(best, float(g.min()))
    return best


def clear(objects):
    return bool(hand_gap(objects, need=CLEAR_MIN) >= CLEAR_MIN)


def _fresh():
    return {"last": None, "fail": None, "now": None, "pose": None, "rates": None}


_MEM = _fresh()


@on_reset
def reset_memory():
    _MEM.clear()
    _MEM.update(_fresh())
    _HANDS.update({"hands": None, "steps": 0, "error": None})
    _CLOCK["dt"] = None


def update(objects):
    """One control step: latch a failure and judge this step. WHY the identity check: the embodiment passes the same
    states dict to success() and then score() each step (inspect_embodiment.py _task_step)."""
    if objects is _MEM["last"]:
        return _MEM
    _MEM["last"] = objects
    why = failure(objects)
    if why and not _MEM["fail"]:
        _MEM["fail"] = why
    p, dt = pose(objects), _CLOCK["dt"]
    _MEM["rates"] = rates(_MEM["pose"], p, dt) if _MEM["pose"] is not None and dt else None
    _MEM["pose"] = p
    _MEM["now"] = {"sealed": sealed(objects, "keeper"), "through": through(objects, "keeper"),
                   "started": started(objects), "clear": clear(objects), "still": still(_MEM["rates"])}
    return _MEM


def success(objects):
    """Sealed on the cam keeper, both hands clear, the seal still, no latched failure (held 1 s by yam_arena)."""
    m = update(objects)
    n = m["now"]
    return bool(not m["fail"] and n["sealed"] and n["clear"] and n["still"])


def score(objects):
    """Partial credit at the last step (success held 1 s scores 1.0 and ends the trial). 0 after a latched failure.
    Sealed on the keeper, hands clear and still: 0.8; sealed and clear: 0.7; sealed, a hand on it: 0.55; the bolt
    through the keeper with the barrel started on its tip: 0.4; the bolt through the keeper: 0.25; else 0."""
    m = update(objects)
    if m["fail"]:
        return 0.0
    n = m["now"]
    if n["sealed"]:
        if n["clear"]:
            return 0.8 if n["still"] else 0.7
        return 0.55
    if n["through"]:
        return 0.4 if n["started"] else 0.25
    return 0.0


# -- demo -------------------------------------------------------------------------------------------------------------
LEFT_BASE, RIGHT_BASE = station.BASES["left"], station.BASES["right"]
YAW_L, YAW_R = -math.pi / 2, math.pi / 2     # fingers close along x; the signs keep both wrists in range [RUN IK]
GRIP_DZ = 0.012           # TCP over a gripped part's axis: pad tips 1 mm under it (PAD_TIP_PAST_TCP 13 mm)
SQUEEZE = 0.002           # pads aimed 2 mm inside each face
GRIP_BOLT = -(COAT_R - SQUEEZE)
GRIP_BARREL = -(BARREL_R - SQUEEZE)
OPEN = -0.030             # pads 60 mm apart
RELEASE = -0.016          # open off the barrel only this far (pads 5.2 mm off its faces)
RELEASE_BOLT = -0.012     # open off the bolt: pads 6.1 mm off the coat, the fingers 5 mm or more from the bar (FK)
BOLT_GRIP_Y = -0.007      # bolt frame: the left pads' centre on the coated shank, near the head
BARREL_GRIP_Y = -0.024    # barrel frame: the right pads' centre, toward the cap
HEAD_GAP = 0.025          # the head's underside this far from the +y ear's face, as planned. WHY 25: in the sim the bolt
#                           lags the hand and ends 8.1-8.4 mm shallower at every gap tried (33, 27, 25 mm; 2026-11-13
#                           probes), so the head really sits about 33 mm out. Its centre of mass (33.9 mm from the head)
#                           then rests over the near ear and the released bolt lies level; planned at 27 or 33 mm it
#                           tipped 12-17 deg tip-up and the barrel missed the tip
BOLT_REST = HOLE_R - COAT_R   # WHY: released, the bolt lies on the bottom of both holes, so the barrel aims this low
PRE_TIP = 0.012           # the tip stops this far before the near ear's face, aligned, before the push
PRE_MOUTH = 0.010         # the barrel's mouth stops this far before the tip's end, aligned, before the push
PUSH = 0.010              # the barrel is pushed this far past the planned seat. WHY 10 mm: the bolt sits about 8 mm
#                           shallower than planned (HEAD_GAP), so the barrel must travel on to the real shoulder;
#                           there it seats by contact and slides the free bolt out the last 1-2 mm
INSERT_V = 0.010          # m/s along the hole
CAP_V = 0.008
LIFT_OFF = 0.018          # up off a released part: the pad tips 6 mm over the bolt's head [ARITH]
BACK_OFF = 0.050          # then back toward the robot. WHY: rising straight up, the gripper body meets the keeper's
#                           locking bar (FK test); it spans 32.5 mm either side of the TCP in x
PARK_L = (0.20, 0.38, TOP + 0.20)
PARK_R = (0.20, -0.38, TOP + 0.20)


def near_face(name="keeper"):
    """y of the +y ear's outer face of a hasp."""
    return HASPS[name][0] + EAR_GAP / 2 + EAR_T


def far_face(name="keeper"):
    return HASPS[name][0] - EAR_GAP / 2 - EAR_T


def bolt_origin_in(name="keeper", gap=HEAD_GAP):
    """The bolt frame's origin once pushed in with its head ``gap`` off the near ear (axis on the hole's)."""
    return np.array([HX, near_face(name) + gap, HASPS[name][1]])


def bolt_tcp(origin):
    return np.asarray(origin, float) + np.array([0.0, BOLT_GRIP_Y, GRIP_DZ])


def barrel_tcp(mouth):
    return np.asarray(mouth, float) + np.array([0.0, BARREL_GRIP_Y, GRIP_DZ])


def _w(t, p, f, label, yaw):
    return Waypoint(float(t), tuple(float(v) for v in p), float(f), label, float(yaw))


def plan(delay=0.0):
    """(left, right, info). Left: take the bolt from its tray, carry it level to the keeper, push it through both
    ears, let go and leave. Right, once the left is clear: take the barrel, line it up behind the tip, push it on to
    the seat, let go and leave."""
    zk = KEEPER[1]
    b0 = np.array(BOLT_START)
    t_in = bolt_tcp(bolt_origin_in())
    t_pre = bolt_tcp(np.array([HX, near_face() + PRE_TIP + OPER_L, zk]))   # the tip PRE_TIP short of the near ear
    d_ins = float(t_pre[1] - t_in[1])
    left = [_w(2.0 + delay, bolt_tcp(b0) + [0, 0, 0.060], OPEN, "over the bolt", YAW_L),
            _w(0.5, bolt_tcp(b0) + [0, 0, 0.060], OPEN, "settle over", YAW_L),
            _w(1.2, bolt_tcp(b0), OPEN, "descend", YAW_L),
            _w(0.8, bolt_tcp(b0), GRIP_BOLT, "close", YAW_L),
            _w(0.4, bolt_tcp(b0), GRIP_BOLT, "settle grip", YAW_L),
            _w(1.2, bolt_tcp(b0) + [0, 0, 0.070], GRIP_BOLT, "lift", YAW_L),
            _w(2.5, t_pre + [0, 0, 0.040], GRIP_BOLT, "carry", YAW_L),
            _w(1.2, t_pre, GRIP_BOLT, "down to the hole", YAW_L),
            _w(0.8, t_pre, GRIP_BOLT, "settle aligned", YAW_L),
            _w(d_ins / INSERT_V, t_in, GRIP_BOLT, "insert", YAW_L),
            _w(0.6, t_in, GRIP_BOLT, "settle in", YAW_L),
            _w(0.6, t_in, RELEASE_BOLT, "open", YAW_L),
            _w(0.3, t_in, RELEASE_BOLT, "settle open", YAW_L),
            _w(0.8, t_in + [0, 0, LIFT_OFF], RELEASE_BOLT, "lift off", YAW_L),
            _w(1.0, t_in + [-BACK_OFF, 0, LIFT_OFF], RELEASE_BOLT, "back off", YAW_L),
            _w(1.5, PARK_L, OPEN, "park", YAW_L)]
    t_left_clear = sum(w.duration for w in left[:-1])
    tip_end = near_face() + HEAD_GAP - OPER_L
    m_pre = np.array([HX, tip_end - PRE_MOUTH, zk - BOLT_REST])
    m_seat = np.array([HX, near_face() + HEAD_GAP + SEAT_Y + PUSH, zk - BOLT_REST])
    r0 = np.array(BARREL_START)
    right = [_w(2.0 + delay, barrel_tcp(r0) + [0, 0, 0.060], OPEN, "over the barrel", YAW_R),
             _w(0.5, barrel_tcp(r0) + [0, 0, 0.060], OPEN, "settle over", YAW_R),
             _w(1.2, barrel_tcp(r0), OPEN, "descend", YAW_R),
             _w(0.8, barrel_tcp(r0), GRIP_BARREL, "close", YAW_R),
             _w(0.4, barrel_tcp(r0), GRIP_BARREL, "settle grip", YAW_R),
             _w(1.2, barrel_tcp(r0) + [0, 0, 0.070], GRIP_BARREL, "lift", YAW_R),
             _w(2.0, barrel_tcp(m_pre) + [0, -0.030, 0.050], GRIP_BARREL, "carry", YAW_R)]
    t_r = sum(w.duration for w in right)
    right.append(_w(max(0.1, t_left_clear - t_r), barrel_tcp(m_pre) + [0, -0.030, 0.050], GRIP_BARREL,
                    "wait for the left", YAW_R))
    d_cap = float(m_seat[1] - m_pre[1])
    right += [_w(1.2, barrel_tcp(m_pre), GRIP_BARREL, "down behind the tip", YAW_R),
              _w(0.8, barrel_tcp(m_pre), GRIP_BARREL, "settle aligned", YAW_R),
              _w(d_cap / CAP_V, barrel_tcp(m_seat), GRIP_BARREL, "push on", YAW_R),
              _w(0.6, barrel_tcp(m_seat), GRIP_BARREL, "settle seated", YAW_R),
              _w(0.6, barrel_tcp(m_seat), RELEASE, "open", YAW_R),
              _w(0.3, barrel_tcp(m_seat), RELEASE, "settle open", YAW_R),
              _w(0.8, barrel_tcp(m_seat) + [0, 0, LIFT_OFF], RELEASE, "lift off", YAW_R),
              _w(1.0, barrel_tcp(m_seat) + [-BACK_OFF, 0, LIFT_OFF], RELEASE, "back off", YAW_R),
              _w(1.5, PARK_R, OPEN, "park", YAW_R)]
    times = {}
    for arm, ws in (("left", left), ("right", right)):
        t = 0.0
        for w in ws:
            times[(arm, w.label)] = (t, t + w.duration)
            t += w.duration
    return left, right, {"times": times, "left_clear": t_left_clear}


def demo(delay=0.0):
    """Left: take the bolt from its tray, push it through both ears of the cam keeper, let go and park. Right, once the
    left has lifted off: take the barrel, line it up behind the bolt's tip, push it on until it seats against the
    shoulder, let go and park."""
    left, right, _ = plan(delay)
    return {"left": left, "right": right}
