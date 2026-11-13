"""A 3 kg bench scale (1 g display) stands on the table to the robot's left; at its back is a bare stainless steel platter
263 mm wide and 203 mm deep. Farther left, a 1 kg test weight (a brass knob weight 48 mm across and 80 mm tall) stands
on a weight stand level with the platter. Run the scale's eccentricity test: put the weight's centre on five points of
the platter, first its centre, then, in any order, the four points half way from the centre to each corner along the
diagonals. The platter has no marks: find the points from its edges. The empty platter must read zero before the centre
is loaded. At each point let go and keep every part of both arms at least 10 mm from the weight and the platter until
the reading has held 1000 g (within 3 g) for 2 s. Lift the weight to move it: do not slide, drop or knock it over, and
do not press on the platter. Finish with the weight back on its stand and both arms clear.
"""

import functools
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, hull, mechanism, on_reset, on_step  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, SHOULDER, WRIST  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
G = 9.81            # m/s^2 [UNVERIFIED: Isaac Lab's default gravity, recalled, not read this session; the hold probe
                    # p_hold checks that 1 kg reads 1000 g within IND_TOL]

# -- the scale. [SOURCE] KERN WTB 3K-3NM technical data sheet (kern-sohn.com, TD_WTB+3K-3NM_en.pdf, read 2026-11-09):
# Max 3000 g, d 1 g, housing W x D x H 286 x 316 x 126.5 mm (also "completely mounted"), weighing surface 263 x 203 mm,
# stainless steel weighing plate, 1 load cell, stabilization time 2 s, "Eccentric loading at 1/3 [Max]: 1g", net
# weight approx. 3.2 kg. W runs left to right for the operator (the robot), D front to back. -------------------------
MAX_G = 3000.0
HOUSING_W, HOUSING_D, HOUSING_H = 0.286, 0.316, 0.1265
PLAT_W, PLAT_D = 0.263, 0.203            # along world y, along world x
PLAT_T = 0.006                           # [EST] plate thickness (the sheet gives none)
PLAT_TOP = HOUSING_H                     # the platter's top is the mounted height
PLAT_GAP = 0.003                         # [EST] platter underside to housing top
CONSOLE_H = 0.060                        # [EST] the display console in front of the platter (113 mm deep); the sheet
                                         # gives no layout, so the platter sits flush with the housing's back
SCALE_X, SCALE_Y = 0.27, 0.24            # [CHOICE] the platter centre (world). WHY: every carry height (TCP up to 0.19
                                         # m over the table) solves straight down out to x 0.34 for y 0.14-0.46 [RUN
                                         # work/explore.py]; the back points sit at x 0.321
M_PLAT = 0.5                             # kg [EST] stainless plate
M_HOUSING = 2.7                          # kg: 3.2 kg net [SOURCE] less the platter (the base is fixed; unused)
# [CHOICE] the load cell is a vertical spring on a prismatic joint. STIFFNESS WORKS in this stack, joint friction does
# not (LESSONS 2026-11-09): nothing here relies on friction. A real load cell deflects a fraction of a mm; 4000 N/m
# (2.45 mm per kg) keeps 1 g at 2.45 um of travel, above PhysX's position noise [UNVERIFIED, p_hold measures it].
K_PLAT = 4000.0                          # N/m
C_PLAT = 100.0                           # N s/m [CHOICE]: damping ratio 0.65 loaded (1.5 kg), 1.1 empty [ARITH]
DEFL = 1.0 * G / K_PLAT                  # the platter's sink under the 1 kg weight: 2.45 mm
PLAT_LOW, PLAT_HIGH = -0.010, 0.002      # stops: 4.08 kg down, 0.8 kg-force up [ARITH]

# -- the weight. [SOURCE] OIML R 111-1:2004 Annex A, Table A.1 (read from the page image, PDF page 26): 1 kg
# cylindrical weight D1 = 48 mm (body), D2 = 43 (knob), D3 = 27 (neck); H "depending on material". [SOURCE] KERN 347-11
# data sheet (Individual weight, OIML Class M1, 1 kg, Knob, stainless steel fine turned): "Dimensions (O x H) 48x80
# mm", volume 125 cm^3. The sim's loose object is one shape (rules.md 3): a 48 x 80 mm cylinder, the knob's outline
# filled in, mass 1.000 kg, centre of mass at mid height (a real knob weight's sits lower) [CHOICE]. --------------------
W_R, W_H, W_MASS = 0.024, 0.080, 1.0
BRASS = (0.85, 0.62, 0.12)               # [CHOICE] saturated for the top camera; R 111-1 8.4.2 allows brass M1 weights
STAND_XY = (0.27, 0.455)                 # [CHOICE] beside the scale, 32 mm clear of the housing's left side
STAND_SIZE = 0.080
STAND_TOP = PLAT_TOP                     # [CHOICE] level with the platter, so the carry stays inside the YAM's reach
WEIGHT_START = (STAND_XY[0], STAND_XY[1], TOP + STAND_TOP + W_H / 2)

GREY_D, STEEL, DARK = (0.22, 0.24, 0.30), (0.62, 0.64, 0.68), (0.12, 0.12, 0.14)
MATERIALS = {"default": (1.2, 1.0)}      # [CHOICE] the scene's friction on every asset (rules.md 3)


def _box(lo, hi):
    """8 corners: the bottom ring then the top ring, counter-clockwise seen from above (mechanism.hull's order)."""
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    if not (x1 > x0 and y1 > y0 and z1 > z0):
        raise ValueError(f"empty box {lo} {hi}")
    ring = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return [(x, y, z) for z in (z0, z1) for x, y in ring]


def housing_boxes():
    """Fixed housing boxes in the scale's frame (origin under the platter centre, on the table)."""
    hw = HOUSING_W / 2
    # under the platter: from the table (z 0) up to PLAT_GAP under the platter's underside
    rear = _box((-PLAT_D / 2, -hw, 0.0), (PLAT_D / 2, hw, PLAT_TOP - PLAT_T - PLAT_GAP))
    # the console: the rest of the 316 mm depth, in front (toward the robot, -x)
    console = _box((PLAT_D / 2 - HOUSING_D, -hw, 0.0), (-PLAT_D / 2, hw, CONSOLE_H))
    return {"rear": rear, "console": console}


def platter_box(q=0.0):
    return _box((-PLAT_D / 2, -PLAT_W / 2, PLAT_TOP - PLAT_T + q), (PLAT_D / 2, PLAT_W / 2, PLAT_TOP + q))


def _scale():
    base = Link("housing", M_HOUSING, tuple(hull(b, GREY_D) for b in housing_boxes().values()))
    plat = Link("platter", M_PLAT, (hull(platter_box(), STEEL),))
    # WHY target M_PLAT G / K: at q = 0 the spring carries the platter's own weight, so the empty platter rests at 0
    # (where the reset puts it) and the indication -K q / G is the load on it
    joint = Joint("platter", "prismatic", "platter", (0.0, 0.0, PLAT_TOP - PLAT_T), "Z", PLAT_LOW, PLAT_HIGH,
                  stiffness=K_PLAT, damping=C_PLAT, target=M_PLAT * G / K_PLAT)
    return mechanism("scale", (SCALE_X, SCALE_Y, TOP), base, [plat], [joint], MATERIALS)


SCALE = _scale()
WEIGHT = Cylinder(name="weight", pos=WEIGHT_START, radius=W_R, height=W_H, mass=W_MASS, color=BRASS)
STAND = Box(name="stand", pos=(STAND_XY[0], STAND_XY[1], TOP + STAND_TOP / 2),
            size=(STAND_SIZE, STAND_SIZE, STAND_TOP), color=DARK)
OBJECTS = [SCALE, WEIGHT, STAND]

# -- the five points [SOURCE EURAMET cg-18 v4.0 Fig. 5.3-1: 1 centre, 2 front left, 3 back left, 4 back right, 5 front
# right; on a rectangular platter the off-centre circles sit on the diagonals, about half way to each corner (read
# from the figure, [JUDGEMENT] on "about half way")]. Front = toward the operator (the robot, -x); left = +y. ----------
POINTS = {"centre": (0.0, 0.0), "front_left": (-PLAT_D / 4, PLAT_W / 4), "back_left": (PLAT_D / 4, PLAT_W / 4),
          "back_right": (PLAT_D / 4, -PLAT_W / 4), "front_right": (-PLAT_D / 4, -PLAT_W / 4)}
OFF = ("front_left", "back_left", "back_right", "front_right")

# -- scoring constants ------------------------------------------------------------------------------------------------
IND_TOL = 3.0        # g [JUDGEMENT]: 3 d of the 1 g display. The sim's scale is ideal (one spring), so the weight alone
                     # reads 1000 g; 3 g is 0.03 N, so a finger resting on the weight or the platter with more than
                     # that shows. The maker allows 1 g eccentric error at Max/3 [SOURCE]; p_hold measures the sim's
                     # own scatter, which must stay well under this
DWELL_STEPS = 40     # control steps: 2 s at the run's 20 Hz [SOURCE the scale's stabilization time, 2 s]
R_POS = 0.010        # m [JUDGEMENT]: the weight's axis within 10 mm of a point. EURAMET says "as closely as
                     # possible"; 10 mm is what placing by eye on an unmarked plate gets, under a quarter of the
                     # weight's 48 mm diameter, and 7x smaller than the 71 mm between centre and corner points
STILL_V = 0.002      # m/s: weight and platter this slow are still (the 2 s wait is the real rule)
STILL_W = 0.05       # rad/s: the weight's spin
V_DROP = 0.20        # m/s [JUDGEMENT]: the weight moving down faster than this within DROP_ZONE over the platter or
                     # the stand was dropped or slammed onto it (a fall of about 2 mm reaches it; the demo lowers at
                     # 30 mm/s at most [ARITH test_demo_vertical_speed])
DROP_ZONE = 0.030    # m [JUDGEMENT]: the approach band over a surface where a set-down must be gentle; higher up, an
                     # arm may move the weight down fast
V_FALL = 0.60        # m/s [JUDGEMENT]: down faster than this anywhere is a fall or a throw (free fall reaches it in
                     # 18 mm; a careful carry does not)
V_STRIKE = 0.10      # m/s [JUDGEMENT]: the platter moving faster than this was struck or hit by a dropped weight; the
                     # demo moves it at 2.45 mm per 1.2 s
TILT_UP = 3.0        # deg: standing on its base
TILT_LOST = 30.0     # deg: knocked over
ON_TOL = 0.003       # m: the weight's base within this of a surface's top is standing on it
TABLE_CLEAR = 0.010  # m: the weight's base this close to the table means it left the scale and its stand
SLIDE_MAX = 0.015    # m [JUDGEMENT]: the weight's axis moving more than this while its base stays on the platter
                     # (within ON_TOL) is a slide, not a lift; set-down drift is a few mm
STAND_R = 0.010      # m [JUDGEMENT]: back on the stand = axis within 10 mm of its centre (the 48 mm base then sits
                     # 6 mm or more inside the 80 mm top)


def indication(objects):
    """The scale's reading (g): the platter spring's force, -K q, over g. Physics, not the weight's position: a
    hand pressing the platter or holding the weight changes it."""
    return -K_PLAT * float(objects["scale"].joints["platter"]) / G * 1000.0


def platter_top(objects):
    return TOP + PLAT_TOP + float(objects["scale"].joints["platter"])


def platter_speed(objects):
    return float(np.linalg.norm(objects["scale"].links["platter"].lin_vel))


def tilt_deg(w):
    x, y, _, _ = (float(v) for v in w.quat_xyzw)
    zz = 1.0 - 2.0 * (x * x + y * y)                  # the weight's axis, world z component
    return math.degrees(math.acos(max(-1.0, min(1.0, zz))))


def bottom(w):
    return float(w.pos[2]) - W_H / 2


def weight_still(w):
    return float(np.linalg.norm(w.lin_vel)) <= STILL_V and float(np.linalg.norm(w.ang_vel)) <= STILL_W


def platter_xy(objects):
    """The weight's axis in the platter's frame (m), from its own centre (the scale is not turned)."""
    w, p = objects["weight"], objects["scale"].links["platter"]
    return float(w.pos[0] - p.pos[0]), float(w.pos[1] - p.pos[1])


def on_platter(objects):
    w = objects["weight"]
    dx, dy = platter_xy(objects)
    inside = abs(dx) <= PLAT_D / 2 - W_R and abs(dy) <= PLAT_W / 2 - W_R
    return bool(inside and tilt_deg(w) <= TILT_UP and abs(bottom(w) - platter_top(objects)) <= ON_TOL)


def clear_of_platter(objects):
    """Nothing of the weight over the platter, or its base 5 mm or more above it."""
    w = objects["weight"]
    dx, dy = platter_xy(objects)
    off = abs(dx) > PLAT_D / 2 + W_R + 0.002 or abs(dy) > PLAT_W / 2 + W_R + 0.002
    return bool(off or bottom(w) - platter_top(objects) >= 0.005)


def at_point(objects):
    """The point the weight's axis is within R_POS of, else None."""
    dx, dy = platter_xy(objects)
    for name, (px, py) in POINTS.items():
        if math.hypot(dx - px, dy - py) <= R_POS:
            return name
    return None


def reading(objects):
    """What this step's state reads as: 'zero' (empty platter reading zero), a point's name (the weight alone on the
    platter at that point reading 1000 g), or None. The weight and platter must be still."""
    if platter_speed(objects) > STILL_V:
        return None
    ind = indication(objects)
    if not on_platter(objects):
        return "zero" if (clear_of_platter(objects) and abs(ind) <= IND_TOL) else None
    if not weight_still(objects["weight"]) or abs(ind - 1000.0) > IND_TOL:
        return None
    return at_point(objects)


def on_stand(objects):
    w = objects["weight"]
    d = math.hypot(float(w.pos[0]) - STAND_XY[0], float(w.pos[1]) - STAND_XY[1])
    return bool(d <= STAND_R and tilt_deg(w) <= TILT_UP and abs(bottom(w) - (TOP + STAND_TOP)) <= ON_TOL
                and weight_still(w))


def scale_empty(objects):
    return bool(clear_of_platter(objects) and abs(indication(objects)) <= IND_TOL and platter_speed(objects) <= STILL_V)


def touching_platter(objects):
    """The weight's base on the platter's top (within ON_TOL) with any of its footprint over the plate."""
    dx, dy = platter_xy(objects)
    over = abs(dx) <= PLAT_D / 2 + W_R and abs(dy) <= PLAT_W / 2 + W_R
    return bool(over and abs(bottom(objects["weight"]) - platter_top(objects)) <= ON_TOL)


def near_surface(objects):
    """The weight's base in the band from 10 mm under to DROP_ZONE over the platter or stand top, over it."""
    w = objects["weight"]
    dx, dy = platter_xy(objects)
    h = bottom(w) - platter_top(objects)
    if abs(dx) <= PLAT_D / 2 + W_R and abs(dy) <= PLAT_W / 2 + W_R and -0.010 <= h <= DROP_ZONE:
        return True
    sx, sy = float(w.pos[0]) - STAND_XY[0], float(w.pos[1]) - STAND_XY[1]
    h = bottom(w) - (TOP + STAND_TOP)
    return bool(abs(sx) <= STAND_SIZE / 2 + W_R and abs(sy) <= STAND_SIZE / 2 + W_R and -0.010 <= h <= DROP_ZONE)


def failure(objects):
    """Why this state fails the trial, or None (sliding is checked in update(), which keeps the contact's start)."""
    w = objects["weight"]
    vz = float(w.lin_vel[2])
    if vz < -V_FALL or (vz < -V_DROP and near_surface(objects)):
        return "dropped"
    if platter_speed(objects) > V_STRIKE:
        return "struck"
    if indication(objects) > MAX_G:
        return "overload"
    if tilt_deg(w) > TILT_LOST or bottom(w) < TOP + TABLE_CLEAR:
        return "lost"
    return None


# -- the hands: let go and clear --------------------------------------------------------------------------------------
# success() and score() get object states only. A hook run before each scored step (mechanism.on_step) reads every
# body of both YAM arms from the simulator, so the scorer sees a hand still on the weight or the platter, however
# little of the load it carries (the technique of the tie-rod-end task, which read in the sim a closed grip at -2 mm,
# an open one at about 42 mm and a parked arm at the 50 mm cap).
ARMS = {"left": "left_robot", "right": "right_robot"}  # the embodiment's scene keys
# every rigid body of a YAM arm: the URDF's links, the bodies of the converted USD
ARM_LINKS = ("base", "link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
HAND_CLEAR = 0.010  # m [JUDGEMENT]: a hand nearer than this may be touching (the arm lags its target 2 to 5 mm when
                    # moving, the colliders' contact offsets add 1 to 2 mm); on a real rig a 10 mm gap shows on video
SAMPLE = 0.002      # m: spacing of the points laid over the weight's and the platter's colliders
CLEAR_CAP = 0.05    # m: clearances are reported up to this; farther is "at least 50 mm"
# this step's arm body poses (world, 4x4) and finger joints, from the hook, and the hook's step count
_HANDS = {"poses": None, "finger": None, "step": 0, "memo": None}


def _rot(q):
    """Rotation matrix of a quaternion (x, y, z, w)."""
    x, y, z, w = (float(v) for v in q)
    n = x * x + y * y + z * z + w * w
    x, y, z, w = (v / math.sqrt(n) for v in (x, y, z, w))
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)  # Isaac Lab 3.0 wraps some buffers
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=float)


def read_arms(environment):
    """(poses, finger): the world pose (4x4, the task's frame) of every body of both arms, and each arm's finger
    joints (m, 0 closed, -0.047 open)."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins)[0]
    poses, finger = {}, {}
    for arm, key in ARMS.items():
        robot = base.scene[key]
        bodies = _np(robot.data.body_link_pose_w)[0]
        names = list(robot.body_names)
        poses[arm] = {}
        for link in ARM_LINKS:
            p = bodies[names.index(link)]  # a missing body raises: the check never passes quietly
            t = np.eye(4)
            t[:3, :3] = _rot(p[3:7])
            t[:3, 3] = p[:3] - origin
            poses[arm][link] = t
        q = _np(robot.data.joint_pos)[0]
        joints = list(robot.joint_names)
        finger[arm] = [float(q[joints.index(j)]) for j in ("joint7", "joint8")]
    return poses, finger


@on_step
def _watch_arms(environment):
    _HANDS["poses"], _HANDS["finger"] = read_arms(environment)
    _HANDS["step"] += 1


@on_reset
def _forget_arms():
    _HANDS["poses"] = _HANDS["finger"] = _HANDS["memo"] = None


def _grid(n_u, n_v, u, v):
    return np.stack(np.meshgrid(np.linspace(-u, u, n_u), np.linspace(-v, v, n_v), indexing="ij"), -1).reshape(-1, 2)


def _n(length):
    return max(2, math.ceil(length / SAMPLE) + 1)


def _cylinder_points(r, h):
    """Points over a solid cylinder's side and end faces (axis z, centred), every surface point within SAMPLE of
    one: along the side and around it at most SAMPLE apart, each end a disk of rings at most SAMPLE apart."""
    n_t = max(8, math.ceil(2 * math.pi * r / SAMPLE))
    t = np.linspace(0, 2 * math.pi, n_t, endpoint=False)
    rings = [(r, z) for z in np.linspace(-h / 2, h / 2, _n(h))]
    rings += [(rr, z) for rr in np.linspace(0, r, _n(r))[:-1] for z in (-h / 2, h / 2)]
    return np.vstack([np.c_[rr * np.cos(t), rr * np.sin(t), np.full(n_t, z)] for rr, z in rings])


def _box_points(lo, hi):
    """Points over a box's six faces, grids at most SAMPLE apart (every surface point within SAMPLE of one)."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c, e = (lo + hi) / 2, (hi - lo) / 2
    out = []
    for ax in range(3):
        u, v = [k for k in range(3) if k != ax]
        g = _grid(_n(2 * e[u]), _n(2 * e[v]), e[u], e[v])
        for side in (-1.0, 1.0):
            f = np.zeros((len(g), 3))
            f[:, u], f[:, v], f[:, ax] = g[:, 0], g[:, 1], side * e[ax]
            out.append(f)
    return np.vstack(out) + c


# the weight in its own frame; the platter in the scale's frame at rest (its link's frame there)
_PARTS = {"weight": _cylinder_points(W_R, W_H),
          "platter": _box_points(np.min(platter_box(), axis=0), np.max(platter_box(), axis=0))}


def part_points(objects):
    """{part: its laid points in the world}."""
    w, p = objects["weight"], objects["scale"].links["platter"]
    with np.errstate(all="ignore"):  # WHY: spurious matmul FP flags on macOS numpy (seen 2026-11-12); checked below
        out = {"weight": np.asarray(w.pos, float) + _PARTS["weight"] @ _rot(w.quat_xyzw).T,
               "platter": np.asarray(p.pos, float) + _PARTS["platter"] @ _rot(p.quat_xyzw).T}
    if not all(np.isfinite(v).all() for v in out.values()):
        raise ValueError("part points are not finite")
    return out


@functools.lru_cache(maxsize=1)
def _arm_hulls():
    """Each arm body's collision hulls in its own frame, as (vertices, unique face planes n . x = d), from the URDF's
    CoACD pieces (yam_arena.analysis.link_hulls)."""
    from yam_arena.analysis import link_hulls

    with np.errstate(all="ignore"):  # WHY: macOS numpy's matmul raises spurious FP flags here; the planes are checked
        hulls = link_hulls()
    out = {}
    for link in ARM_LINKS:
        out[link] = []
        for h in hulls[link]:
            planes = np.unique(np.round(h.planes, 9), axis=0)
            if not np.isfinite(planes).all():
                raise ValueError(f"{link}: a hull plane is not finite")
            out[link].append((h.vertices, planes))
    return out


@functools.lru_cache(maxsize=1)
def _link_corners():
    """The 8 corners of each arm body's box around all its hulls, in its own frame (a cheap first test)."""
    out = {}
    for link, hs in _arm_hulls().items():
        v = np.vstack([verts for verts, _ in hs])
        lo, hi = v.min(axis=0), v.max(axis=0)
        out[link] = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    return out


def arm_clearance(points, poses):
    """A lower bound (m) on the gap between any body of one arm and the part the points lie on, capped at CLEAR_CAP;
    -SAMPLE means touching or overlapping. For each point, the bound is the larger of the distance to a hull's box and
    the hull's face-plane distance max_i(n_i . p - d_i): neither is more than the true distance to that convex hull.
    Every surface point is within SAMPLE of a laid point, so the true gap is at least the smallest bound minus
    SAMPLE."""
    with np.errstate(all="ignore"):  # WHY: spurious matmul FP flags on macOS numpy (seen 2026-11-12); checked below
        gap = _gap(points, poses)
    if not math.isfinite(gap):
        raise ValueError("arm clearance is not finite")
    return gap


def _gap(points, poses):
    lo, hi = points.min(axis=0), points.max(axis=0)
    best = CLEAR_CAP + SAMPLE  # the smallest point bound so far
    for link, t in poses.items():
        r, p = t[:3, :3], t[:3, 3]
        c = _link_corners()[link] @ r.T + p  # WHY: the body's whole box first; most bodies are far from the part
        if (c.min(axis=0) - hi).max() >= best or (lo - c.max(axis=0)).max() >= best:
            continue
        for verts, planes in _arm_hulls()[link]:
            v = verts @ r.T + p
            vlo, vhi = v.min(axis=0), v.max(axis=0)
            if (vlo - hi).max() >= best or (lo - vhi).max() >= best:
                continue  # this hull's box is farther than the best bound from every point
            box = np.linalg.norm(np.maximum(np.maximum(vlo - points, points - vhi), 0.0), axis=1)
            near = box < best
            if not near.any():
                continue
            n = planes[:, :3] @ r.T
            d = planes[:, 3] + n @ p
            bound = np.maximum((points[near] @ n.T - d).max(axis=1), box[near])
            best = min(best, float(bound.min()))
    return min(best - SAMPLE, CLEAR_CAP)


def hand_clearance(objects, poses=None):
    """{arm: {"weight": gap, "platter": gap}}, lower bounds (m) for this step from ``poses`` or else the hook's
    reading; None if no arm poses were read (the hook has not run since the reset)."""
    poses = _HANDS["poses"] if poses is None else poses
    if poses is None:
        return None
    # WHY the memo: the answer depends only on the poses (the very object, which the memo keeps alive) and the parts'
    # poses; offline replays repeat both thousands of times
    w, p = objects["weight"], objects["scale"].links["platter"]
    key = tuple(float(v) for o in (w, p) for v in (*o.pos, *o.quat_xyzw))
    memo = _HANDS.get("memo")
    if memo is None or memo[0] is not poses:
        memo = _HANDS["memo"] = (poses, {})
    if key not in memo[1]:
        if len(memo[1]) > 256:
            memo[1].clear()
        pts = part_points(objects)
        memo[1][key] = {arm: {k: arm_clearance(v, poses[arm]) for k, v in pts.items()} for arm in ARMS}
    return {arm: dict(g) for arm, g in memo[1][key].items()}


def hands_clear(gaps, parts=("weight", "platter")):
    """Both arms at least HAND_CLEAR from each of ``parts``. Unknown (None) counts as not clear."""
    return gaps is not None and all(gaps[arm][k] >= HAND_CLEAR for arm in ARMS for k in parts)


def _fresh():
    return {"last": None, "run": None, "n": 0, "zero": False, "centre": False, "off": set(), "fail": None,
            "anchor": None, "home": 0, "gaps": None, "back": False}


_MEM = _fresh()


@on_reset
def reset_memory():
    _MEM.clear()
    _MEM.update(_fresh())


def step_reading(objects, gaps):
    """This step's reading with the hands counted: 'zero' needs both arms HAND_CLEAR from the platter, a point both
    arms HAND_CLEAR from the platter and the weight. WHY: a hand that grips the weight or rests on the platter while
    carrying under 3 g of it does not show in the indication."""
    r = reading(objects)
    if r == "zero":
        return r if hands_clear(gaps, ("platter",)) else None
    return r if (r is None or hands_clear(gaps)) else None


def update(objects):
    """One control step: latch a failure, extend the current reading's run, record it at DWELL_STEPS. WHY the
    identity check: the embodiment passes the same states dict to success() and then score() each step
    (inspect_embodiment.py _task_step), and the run must count steps, not calls."""
    if objects is _MEM["last"]:
        return _MEM
    _MEM["last"] = objects
    gaps = _MEM["gaps"] = hand_clearance(objects)
    why = failure(objects)
    if touching_platter(objects):                                  # one contact: the axis may not travel along it
        xy = platter_xy(objects)
        if _MEM["anchor"] is None:
            _MEM["anchor"] = xy
        elif math.hypot(xy[0] - _MEM["anchor"][0], xy[1] - _MEM["anchor"][1]) > SLIDE_MAX:
            why = why or "slid"
    else:
        _MEM["anchor"] = None
    if why and not _MEM["fail"]:
        _MEM["fail"] = why
    _MEM["back"] = bool(returned(objects) and hands_clear(gaps))
    _MEM["home"] = _MEM["home"] + 1 if _MEM["back"] else 0
    r = step_reading(objects, gaps)
    if r is not None and r == _MEM["run"]:
        _MEM["n"] += 1
    else:
        _MEM["run"], _MEM["n"] = r, (1 if r is not None else 0)
    if _MEM["n"] >= DWELL_STEPS:
        if r == "zero":
            _MEM["zero"] = True
        elif r == "centre":
            _MEM["centre"] = _MEM["centre"] or _MEM["zero"]       # the centre counts only after a zero reading
        elif r in OFF and _MEM["centre"]:                          # corner points only after the centre
            _MEM["off"].add(r)
    return _MEM


def recorded(m):
    return int(m["centre"]) + len(m["off"])


def returned(objects):
    return on_stand(objects) and scale_empty(objects)


def success(objects):
    """Zero, then the centre, then all four corner points recorded; the weight back on its stand, still, the empty
    platter reading zero and both arms HAND_CLEAR from the weight and the platter, for DWELL_STEPS in a row; nothing
    dropped, slid, struck or lost. WHY the 2 s: the same settling a reading gets, with the hand let go and clear."""
    m = update(objects)
    # WHY no separate zero term: the centre is recorded only after the zero, so five recorded implies it
    return bool(not m["fail"] and recorded(m) == 5 and m["home"] >= DWELL_STEPS)


def score(objects):
    """Partial credit at the last step (only success() held 1 s pays 1.0, and ends the trial). 0 after a drop, slide,
    strike, overload or lost weight. n = recorded points (the centre, then corner points). Back = the weight on its
    stand, the platter empty and both arms HAND_CLEAR from both, this step. 5 and back: 0.9; 5 otherwise: 0.75; 1 to
    4: 0.15 n, plus 0.05 if back; 0: 0."""
    m = update(objects)
    if m["fail"]:
        return 0.0
    n = recorded(m)
    if n == 5:
        return 0.9 if m["back"] else 0.75
    if n == 0:
        return 0.0
    return 0.15 * n + (0.05 if m["back"] else 0.0)


# -- demo -------------------------------------------------------------------------------------------------------------
LEFT_BASE = station.BASES["left"]
TIP = PAD_TIP_PAST_TCP
GRIP_Z = 0.026       # [CHOICE] TCP this far over the weight's base: the pad tips 13 mm over the surface it stands on,
                     # the weight's top 54 mm over the TCP, 7.5 mm under the finger carriage (61.5 mm over the TCP,
                     # the only finger part reaching across between the pads) [RUN work/fingers.py]
SQUEEZE = 0.003      # pads aimed 3 mm inside each face: about 15 N a finger (5000 N/m drive, 50 N cap); holding 1 kg
                     # needs 4.1 N a finger at friction 1.2 [ARITH]
GRIP_F = -(W_R - SQUEEZE)
OPEN = -0.040        # pads 80 mm apart: 16 mm clear of the weight each side
CLEAR = 0.018        # the carried weight's base this far over the platter and stand tops (model sag)
RAISE = 0.020        # after letting go the open hand rises this far (the pads still flank the weight, the carriage
                     # 27.5 mm over its top), then slides out along -x
SLIDE = 0.075        # far enough that the 68 mm wide fingers clear the 48 mm weight by 17 mm
YAW = 0.0            # pads close along world y (yam_arena constants.py DOWN_XYZW)
SAG_SCALE = 0.7      # the arm's sag aim as a multiple of the joint-stiffness model below. [RUN 2026-11-13, sim
                     # probes at 0.7 / 1.0 / 1.4: all scored 1.0; the placement offsets at the end of each wait were
                     # 1.2-2.5 mm at 0.7 and grew along the sequence to 6.6-7.2 and 7.6-7.9 mm at 1.0 and 1.4, so the
                     # model's x, y sag is over-corrected at 1.0 and up; the carry cleared 12.2-15.0 mm at 0.7]
PARK_L = (0.20, 0.40, TOP + 0.20)
PARK_R = (0.22, -0.25, TOP + 0.20)
K_JOINT = np.diag([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)   # N m/rad [CODE constants.py SHOULDER,
                                                                            # WRIST]


@functools.lru_cache(maxsize=256)
def _sag_unit(p):
    """TCP deflection (m) of the left arm, straight down at p, under 1 kg hanging from the TCP: J K^-1 J^T F. WHY: the
    arm's gravity compensation covers its own links (embodiment.py), not the payload, so the joint PD springs carry
    it; 2.3-3.5 N m on the shoulder (27 N m limit), about 14-23 mm of sag [RUN work/explore.py]."""
    q, _, _ = ik.solve(np.array(p), ik.matrix_to_quat_xyzw(grip_rotation(YAW)), None, base_pos=LEFT_BASE, iters=200)
    p0, _ = ik.tcp_pose(q, LEFT_BASE)
    jac = np.zeros((3, 6))
    for i in range(6):
        d = q.copy()
        d[i] += 1e-6
        jac[:, i] = (ik.tcp_pose(d, LEFT_BASE)[0] - p0) / 1e-6
    f = np.array([0.0, 0.0, -W_MASS * G])
    return tuple(float(v) for v in jac @ np.linalg.solve(K_JOINT, jac.T @ f))


def sag(p, scale=None):
    s = SAG_SCALE if scale is None else scale
    return np.array(_sag_unit(tuple(round(float(v), 4) for v in p))) * s


def site_xy(site):
    if site == "stand":
        return STAND_XY
    px, py = POINTS[site]
    return (SCALE_X + px, SCALE_Y + py)


def surface(site):
    """(unloaded top z, sink under the weight) of where the weight stands."""
    return (TOP + STAND_TOP, 0.0) if site == "stand" else (TOP + PLAT_TOP, DEFL)


def tcp_rest(site):
    """The TCP on the weight resting at a site (platter sunk by DEFL): the hand carries nothing here."""
    (x, y), (z, d) = site_xy(site), surface(site)
    return np.array([x, y, z - d + GRIP_Z])


def tcp_loaded(site, up=0.0, scale=None):
    """Where to aim so the loaded hand holds the weight's base ``up`` over the site's unloaded top: aim minus sag."""
    (x, y), (z, _) = site_xy(site), surface(site)
    p = np.array([x, y, z + GRIP_Z + up])
    return p - sag(p, scale)


def _w(t, p, f, label):
    return Waypoint(float(t), tuple(float(v) for v in p), float(f), label, YAW)


DWELL_WAIT = 1.6     # s the hand waits out after sliding clear; with open, rise and slide it leaves the weight alone
                     # 3.7 s after it is set down (2 s needed) [ARITH test_dwell_windows]
CYCLE = ("stand", "centre", "front_left", "back_left", "back_right", "front_right", "stand")


def plan(delay=0.0, scale=None):
    """The scripted demo: (left, right, info). The left arm moves the weight stand -> centre -> four corner points ->
    stand; the right arm stays parked. info["moves"]: (index, from site, to site) per move, labels '<i> <step>'."""
    out = lambda site: tcp_rest(site) + [-SLIDE, 0.0, RAISE]   # noqa: E731
    raised = lambda site: tcp_rest(site) + [0.0, 0.0, RAISE]   # noqa: E731
    left = [_w(1.0 + delay, PARK_L, OPEN, "start"), _w(2.0, out("stand"), OPEN, "to the stand")]
    moves = []
    for i, (a, b) in enumerate(zip(CYCLE[:-1], CYCLE[1:])):
        moves.append((i, a, b))
        far = np.hypot(*np.subtract(site_xy(a), site_xy(b))) > 0.12
        left += [_w(1.2, raised(a), OPEN, f"{i} slide in"),
                 _w(0.6, raised(a), OPEN, f"{i} settle over"),
                 _w(0.8, tcp_rest(a), OPEN, f"{i} descend"),
                 _w(0.8, tcp_rest(a), GRIP_F, f"{i} close"),
                 _w(0.4, tcp_rest(a), GRIP_F, f"{i} settle grip"),
                 _w(1.2, tcp_loaded(a, 0.0, scale), GRIP_F, f"{i} take load"),
                 _w(1.0, tcp_loaded(a, CLEAR, scale), GRIP_F, f"{i} lift"),
                 _w(2.2 if far else 1.6, tcp_loaded(b, CLEAR, scale), GRIP_F, f"{i} carry"),
                 _w(0.8, tcp_loaded(b, CLEAR, scale), GRIP_F, f"{i} settle over target"),
                 _w(1.0, tcp_loaded(b, 0.0, scale), GRIP_F, f"{i} lower"),
                 _w(1.2, tcp_rest(b), GRIP_F, f"{i} set down"),
                 _w(0.4, tcp_rest(b), GRIP_F, f"{i} settle down"),
                 _w(0.6, tcp_rest(b), OPEN, f"{i} open"),
                 _w(0.3, tcp_rest(b), OPEN, f"{i} settle open"),
                 _w(0.8, raised(b), OPEN, f"{i} rise"),
                 _w(1.0, out(b), OPEN, f"{i} slide out")]
        if b != "stand":
            left.append(_w(DWELL_WAIT, out(b), OPEN, f"{i} wait"))
    left += [_w(1.5, PARK_L, OPEN, "park")]
    right = [_w(2.0 + delay, PARK_R, OPEN, "park")]
    return left, right, {"moves": moves}


def demo(delay=0.0):
    """Left: take the weight off its stand, set it on the platter's centre, let go and slide clear; then move it to
    the front left, back left, back right and front right points in turn the same way; then back onto its stand.
    Right: parked."""
    left, right, _ = plan(delay)
    return {"left": left, "right": right}
