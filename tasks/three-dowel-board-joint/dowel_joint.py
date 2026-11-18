"""Two loose boards lie flat on the bench, end to end along y, 0.51 m apart. Three hardwood dowels stand 20 mm out of
the end of the dark board on the right; the pale board on the left has three matching round holes in its end. Join
them in a dry fit: bring the boards together so all three dowels go into their holes, push the joint closed until the
two ends meet along the whole seam, with the boards level with each other and their edges flush, then let go of both.
"""
import dataclasses
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Shape, body, body_state, hull, on_place, on_reset, on_step, run_seed  # noqa: E402,F401
from yam_arena import station  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 1200           # 60 s at 20 Hz (the reviewer's demo limit for this idea)
TOP = TABLE_TOP[SCENE]

# -- the parts (sources in README "Sources"; mm in comments, m in code) --------------------------------------------------
# Every number: [SOURCE] quoted in the README, [ARITH] derived, [EST] estimate, [CHOICE] design choice, [RUN] measured.
BOARD_L = 0.150                  # board length along y [CHOICE]
BOARD_W = 0.070                  # board width along x [CHOICE: 70 mm stock; the open pads span 94 mm]
BOARD_T = 0.019                  # thickness [CHOICE: 3/4 in nominal stock]
BOARD_KG = 0.135                 # [ARITH: 150 x 70 x 19 mm at 650 kg/m3, birch [EST]]
R_DOWEL = 0.004                  # [SOURCE: Hafele 267.83.240 "8 x 40 mm"]; the collider is a 24-sided prism, corners
#                                  on the 4.0 mm radius (flats 3.966 mm)
DOWEL_LEN = 0.040                # [SOURCE: same]
DOWEL_OUT = 0.020                # half the dowel stands out of the dark board [SOURCE: JessEm 08370 manual, the bit
#                                  "deep enough to accommodate half the dowel you are using"]
DOWEL_CH = 0.001                 # the free end's 45 deg chamfer [EST: Cindoco "Pins are chamfered for easy insertion",
#                                  size not given]
PITCH = 0.024                    # dowel spacing across the board [CHOICE: centre and one each side, as the JessEm
#                                  manual's three-hole row; 24 mm leaves 7 mm of wood outside each outer hole]
DOWEL_X = (-PITCH, 0.0, PITCH)   # across the board (board frame x)
DOWEL_Z = 0.0                    # mid-thickness (board frame z; the frame is the board's centre)
CLR = 0.0002                     # hole radius less dowel radius in the sim [CHOICE, README "The fit"]: the real hole is
#                                  8.00 to 8.20 mm (go and no-go pin gauges) on a 7.90 to 8.00 mm dowel [EST, measured
#                                  per trial], 0 to 0.15 mm a side; PhysX convex contacts jam well under that
R_HOLE = R_DOWEL + CLR           # the hole's flats (a 16-sided bore) [ARITH]
HOLE_DEPTH = 0.022               # [CHOICE: half the dowel plus 2 mm, so the ends meet before the dowels bottom]
HOLE_SIDES = 16
DOWEL_SIDES = 24
CELL = 0.0065                    # half-size of the square cut round each hole that its wedges fill [CHOICE: the walls
#                                  over and under the cut are 3 mm, under the 19 mm board's 5.5 mm of wood]
CONTACT = 0.001                  # contact offset of dowels and hole walls [CHOICE: 1 mm; rest offset 0]
WOOD = {"wood": (0.40, 0.35)}    # static, dynamic friction [EST: dry hardwood]
DARK = (0.36, 0.24, 0.16)
PALE = (0.86, 0.74, 0.54)
DOWEL_COLOR = (0.93, 0.83, 0.62)

CAPTURE = R_HOLE - (R_DOWEL - DOWEL_CH)   # 1.2 mm: a dowel tip enters only within this of its hole's axis [ARITH]
YAW_CAPTURE = math.atan(CAPTURE / PITCH)  # 2.9 deg: an outer dowel's capture used up by yaw alone [ARITH]
BIND_TILT = 2 * CLR / DOWEL_OUT           # 1.1 deg: past this a fully entered dowel cannot go on [ARITH]

# -- the layout -----------------------------------------------------------------------------------------------------------
X_LINE = 0.35                    # the boards' centre line (world x) [CHOICE: both arms reach it at y 0 +- 0.075]
SEAM_Y = 0.0                     # where the ends meet in the demo [CHOICE: midway between the arms]
B_START = (X_LINE, 0.330)        # the pale board's centre at the start (world x, y) [CHOICE]
A_START = (X_LINE, -0.330)       # the dark board's nominal centre [CHOICE]; each seed shifts and turns it
LAYOUTS = ((0.0, 0.0), (0.012, math.radians(3.0)), (-0.012, math.radians(-3.0)))  # (x shift m, yaw rad) [CHOICE]
Z_C = TOP + BOARD_T / 2
PRE_GAP = 0.030                  # the seam gap where both boards wait, dowel tips 10 mm short [CHOICE]


def layout_index(seed):
    return 0 if seed is None else int(seed) % len(LAYOUTS)


START = "bench"                  # sim/mk.sh sets "seam" in the probe copies: both boards start in line, 30 mm apart


def layout(seed):
    """Start poses (x, y, z, yaw) of both boards for a seed."""
    if START == "seam":  # WHY: the probe tests the fit alone, the boards already in line at the pre-seat gap
        half = BOARD_L / 2 + PRE_GAP / 2
        return {"dowel_board": (X_LINE, SEAM_Y - half, Z_C, 0.0), "hole_board": (X_LINE, SEAM_Y + half, Z_C, 0.0)}
    dx, yaw = LAYOUTS[layout_index(seed)]
    return {"dowel_board": (A_START[0] + dx, A_START[1], Z_C, yaw), "hole_board": (B_START[0], B_START[1], Z_C, 0.0)}


# -- geometry ---------------------------------------------------------------------------------------------------------------
def _ring(r, n, y, xc=0.0, zc=0.0, phase=0.0):
    """n points on a circle of radius r about the axis (xc, *, zc) along y, at height y (board frame)."""
    return [(xc + r * math.cos(phase + 2 * math.pi * k / n), y, zc + r * math.sin(phase + 2 * math.pi * k / n))
            for k in range(n)]


def _box(lo, hi, mass, color):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Shape("box", tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, material="wood",
                 mass=mass)


def dowel_shapes(xc, density):
    """One dowel standing out of the dark board's +y end: a 24-sided prism, then its chamfered tip (two convex hulls)."""
    y0, y1, y2 = BOARD_L / 2, BOARD_L / 2 + DOWEL_OUT - DOWEL_CH, BOARD_L / 2 + DOWEL_OUT
    v = math.pi * R_DOWEL ** 2
    body_pts = _ring(R_DOWEL, DOWEL_SIDES, y0, xc, DOWEL_Z) + _ring(R_DOWEL, DOWEL_SIDES, y1, xc, DOWEL_Z)
    tip_pts = _ring(R_DOWEL, DOWEL_SIDES, y1, xc, DOWEL_Z) + _ring(R_DOWEL - DOWEL_CH, DOWEL_SIDES, y2, xc, DOWEL_Z)
    return [hull(body_pts, DOWEL_COLOR, "wood", CONTACT, density * v * (y1 - y0)),
            hull(tip_pts, DOWEL_COLOR, "wood", CONTACT, density * v * DOWEL_CH)]


def dowel_board_shapes():
    """The dark board: a box with three dowels out of its +y end (the dowels' glued halves are inside the box)."""
    vol = BOARD_L * BOARD_W * BOARD_T
    dens = BOARD_KG / vol
    shapes = [_box((-BOARD_W / 2, -BOARD_L / 2, -BOARD_T / 2), (BOARD_W / 2, BOARD_L / 2, BOARD_T / 2),
                   BOARD_KG, DARK)]
    for xc in DOWEL_X:
        shapes += dowel_shapes(xc, dens)
    return shapes


def _square_r(theta):
    """Distance from a hole's axis to its square cut along direction theta."""
    return CELL / max(abs(math.cos(theta)), abs(math.sin(theta)))


def hole_wedges(xc, density):
    """The wall round one hole: 16 convex wedges between the 16-sided bore (flats at R_HOLE) and the square cut, from the
    pale board's -y end face to HOLE_DEPTH in. The cut's corners fall on wedge edges, so every wedge is convex."""
    y0, y1 = -BOARD_L / 2, -BOARD_L / 2 + HOLE_DEPTH
    rv = R_HOLE / math.cos(math.pi / HOLE_SIDES)
    out = []
    for k in range(HOLE_SIDES):
        a0, a1 = 2 * math.pi * k / HOLE_SIDES, 2 * math.pi * (k + 1) / HOLE_SIDES
        ring = [(rv, a0), (rv, a1), (_square_r(a1), a1), (_square_r(a0), a0)]
        pts = [(xc + r * math.cos(a), y, DOWEL_Z + r * math.sin(a)) for y in (y0, y1) for r, a in ring]
        area = 0.5 * abs(sum(pts[i][0] * pts[(i + 1) % 4][2] - pts[(i + 1) % 4][0] * pts[i][2] for i in range(4)))
        out.append(hull(pts, PALE, "wood", CONTACT, density * area * HOLE_DEPTH))
    return out


def hole_board_shapes():
    """The pale board: a solid box from HOLE_DEPTH back from its -y end, and the end's first HOLE_DEPTH built round the
    three holes (slabs over and under the cuts, blocks between them, and each hole's wedges)."""
    vol = BOARD_L * BOARD_W * BOARD_T
    dens = BOARD_KG / vol
    w2, t2, y0, y1 = BOARD_W / 2, BOARD_T / 2, -BOARD_L / 2, -BOARD_L / 2 + HOLE_DEPTH

    def box(lo, hi):
        return _box(lo, hi, dens * float(np.prod(np.subtract(hi, lo))), PALE)

    shapes = [box((-w2, y1, -t2), (w2, BOARD_L / 2, t2)),
              box((-w2, y0, -t2), (w2, y1, DOWEL_Z - CELL)),
              box((-w2, y0, DOWEL_Z + CELL), (w2, y1, t2))]
    edges = [-w2] + [e for xc in DOWEL_X for e in (xc - CELL, xc + CELL)] + [w2]
    for a, b in zip(edges[0::2], edges[1::2]):
        shapes.append(box((a, y0, DOWEL_Z - CELL), (b, y1, DOWEL_Z + CELL)))
    for xc in DOWEL_X:
        shapes += hole_wedges(xc, dens)
    return shapes


def _scale_mass(shapes, kg):
    """Rescale the shapes' masses so they sum to kg (the hole board's wedges and blocks leave the holes empty)."""
    s = kg / sum(sh.mass for sh in shapes)
    return [Shape(sh.kind, sh.center, sh.size, sh.color, sh.axis, sh.material, sh.contact_offset, sh.points,
                  sh.mass * s, sh.collide) for sh in shapes]


def _objects():
    lay = layout(None)
    a, b = lay["dowel_board"], lay["hole_board"]
    return [body("dowel_board", a[:3], _scale_mass(dowel_board_shapes(), BOARD_KG), WOOD, yaw=a[3], color=DARK),
            body("hole_board", b[:3], _scale_mass(hole_board_shapes(), BOARD_KG), WOOD, yaw=b[3], color=PALE)]


OBJECTS = _objects()


# -- small maths --------------------------------------------------------------------------------------------------------------
def _quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _rot3(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _np(x):
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=np.float64)


def _torch_of(x):
    return getattr(x, "torch", x)


def _write_root(base, name, xyz, yaw):
    """Put a loose body at a world pose, still."""
    import torch
    asset = base.scene[name]
    dev = _torch_of(asset.data.root_link_pose_w).device
    o = _torch_of(base.scene.env_origins)[0]
    pose = torch.tensor([[*xyz, *_quat(yaw)]], dtype=torch.float32, device=dev)
    pose[:, :3] += o
    zero = torch.zeros((1, 6), dtype=torch.float32, device=dev)
    if hasattr(asset, "write_root_pose_to_sim_index"):  # Isaac Lab 3.0
        asset.write_root_pose_to_sim_index(root_pose=pose)
        asset.write_root_velocity_to_sim_index(root_velocity=zero)
    else:
        asset.write_root_state_to_sim(torch.cat([pose, zero], dim=1))
    asset.write_data_to_sim()


@on_place
def place_layout(env):
    """The trial's layout by seed (before the reset settles)."""
    base = env.unwrapped
    for name, (x, y, z, yaw) in layout(run_seed()).items():
        _write_root(base, name, (x, y, z), yaw)


# -- the measurement the scorer reads (also the real trial's checks) -----------------------------------------------------------
SEAT_GAP = 0.0003                # both corner gaps under this: seated [CHOICE: a 0.3 mm feeler gauge must not enter]
FLUSH = 0.0003                   # the edges flush within this [CHOICE: straightedge and the same 0.3 mm feeler]
LEVEL = 0.001                    # the boards' tops within this of each other [CHOICE: straightedge and a 1 mm feeler;
#                                  a board resting on the other stands 19 mm up]
FLAT_DEG = 3.0                   # each board within this of flat on the bench [CHOICE]
OVERLAP = -0.0005                # a corner gap under this means the ends overlap [CHOICE: contacts settle within a
#                                  fraction of a mm; real ends cannot overlap]
TIP_IN = 0.001                   # each dowel's tip at least this far into its own hole for the first rung (past its
#                                  chamfer) [CHOICE]
HALF_GAP = 0.010                 # both corner gaps at most this: more than half the 20 mm in
STILL = 0.005                    # m/s: both boards still
PAD_CLEAR = 0.030                # every pad point at least this far from both boards to count as let go [CHOICE]


def to_frame(R, p, pt):
    return R.T @ (np.asarray(pt, float) - np.asarray(p, float))


def measure(a_pos, a_quat, b_pos, b_quat):
    """The joint from the two boards' poses (a: dowel board, b: hole board), all in the dowel board's frame:
    gaps at the seam's two corners (along its length, 0 when the ends touch, negative when they overlap), the
    sideways offset and height step of the hole board's end centre, the yaw between the boards, each board's tilt from
    flat, and for each dowel its tip's depth in its own hole and its distance off that hole's axis."""
    Ra, Rb = _rot3(a_quat), _rot3(b_quat)
    pa, pb = np.asarray(a_pos, float), np.asarray(b_pos, float)
    gaps = []
    for sx in (-1.0, 1.0):
        c = to_frame(Ra, pa, pb + Rb @ np.array([sx * BOARD_W / 2, -BOARD_L / 2, 0.0]))
        gaps.append(float(c[1] - BOARD_L / 2))
    e = to_frame(Ra, pa, pb + Rb @ np.array([0.0, -BOARD_L / 2, 0.0]))
    rel = Ra.T @ Rb
    yaw = math.atan2(rel[1, 0], rel[0, 0])
    tilt = [math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2]))))) for R in (Ra, Rb)]
    depth, off = [], []
    for xc in DOWEL_X:
        tip = to_frame(Rb, pb, pa + Ra @ np.array([xc, BOARD_L / 2 + DOWEL_OUT, DOWEL_Z]))
        depth.append(float(tip[1] + BOARD_L / 2))
        off.append(float(math.hypot(tip[0] - xc, tip[2] - DOWEL_Z)))
    return {"gaps": gaps, "dx": float(e[0]), "dz": float(e[2]), "yaw": yaw, "tilt": tilt, "depth": depth, "off": off}


def rung(m):
    """0 to 3 from a measurement: 1 every dowel's tip in its own hole; 2 and more than half in; 3 seated. Every rung
    also needs the boards level with each other, flat, and their ends not overlapping, so boards stacked or pushed over
    each other score nothing."""
    level = abs(m["dz"]) <= LEVEL and max(m["tilt"]) <= FLAT_DEG and min(m["gaps"]) >= OVERLAP
    tips = all(d >= TIP_IN for d in m["depth"]) and all(o <= CAPTURE for o in m["off"])
    if not (level and tips):
        return 0
    if max(m["gaps"]) > HALF_GAP:
        return 1
    if max(m["gaps"]) > SEAT_GAP or abs(m["dx"]) > FLUSH:
        return 2
    return 3


def _box_dist(R, p, pt):
    """Distance from a world point to a board's box (0 inside)."""
    q = np.abs(to_frame(R, p, pt)) - np.array([BOARD_W / 2, BOARD_L / 2 + DOWEL_OUT, BOARD_T / 2])
    return float(np.linalg.norm(np.maximum(q, 0.0)))


def pads_clear(pads, boards):
    """Every pad point at least PAD_CLEAR from both boards' boxes (each box stretched over the dowels)."""
    if not pads:
        return False
    return all(_box_dist(_rot3(q), p, pt) >= PAD_CLEAR for pt in pads for p, q in boards)


# -- per-step memory (hands are read from the sim; the boards come in as states) -------------------------------------
TCP_OFFSET = (0.0, 0.0, 0.134)   # yam_arena.constants: the TCP along the gripper's approach axis
PAD_TIP = 0.147                  # the pad tips along it: 13 mm past the TCP (yam_arena.constants.PAD_TIP_PAST_TCP)
PAD_TOP = 0.112                  # the pads' upper end [EST: 35 mm of pad face]
_M = {"pads": None, "steps": 0, "error": None, "boards": None, "tcps": None}
_MEM = {"last": None, "now": None, "held": 0.0}
_CLOCK = {"dt": None, "steps": 0}


def reset_memory():
    _M.update({"pads": None, "steps": 0, "error": None, "boards": None, "tcps": None})
    _MEM.update({"last": None, "now": None, "held": 0.0})
    _CLOCK.update({"dt": None, "steps": 0})


@on_reset
def _reset():
    reset_memory()


def pad_points(g_pos, g_quat, finger):
    """Four points on one hand's two pads (world): tip and upper end of each, the pads |finger| either side of the
    gripper's axis along its x (yam_arena: the fingers close along the gripper's x)."""
    R, g = _rot3(g_quat), np.asarray(g_pos, float)
    s = abs(float(finger))
    return [g + R @ np.array([sx * s, 0.0, z]) for sx in (-1.0, 1.0) for z in (PAD_TIP, PAD_TOP)]


@on_step
def read_hands(env):
    """Both hands' pad points, after every control step; also both TCPs and both boards' poses, which only the
    scripted demo's line-up reads (never the score)."""
    try:
        base = env.unwrapped
        origin = _np(base.scene.env_origins)[0]
        pads, tcps = [], {}
        for side in ("left", "right"):
            rob = base.scene[f"{side}_robot"]
            g = _np(rob.data.body_link_pose_w)[0][list(rob.body_names).index("gripper")]
            q = _np(rob.data.joint_pos)[0]
            pads += pad_points(g[:3] - origin, g[3:7], q[list(rob.joint_names).index("joint7")])
            tcps[side] = g[:3] - origin + _rot3(g[3:7]) @ np.array(TCP_OFFSET)
        _M["pads"], _M["tcps"] = pads, tcps
        _M["steps"] += 1
        _CLOCK["dt"] = float(base.step_dt)
    except Exception as e:  # WHY: never stop the run; unread hands count as not let go, so no full score
        _M["error"] = repr(e)
        _M["pads"] = None
    try:  # WHY apart: a failed read here must not blank the pads the score reads
        base = env.unwrapped
        origin = _np(base.scene.env_origins)[0]
        poses = [_np(base.scene[n].data.root_link_pose_w)[0] for n in ("dowel_board", "hole_board")]
        _M["boards"] = tuple(x for pz in poses for x in (pz[:3] - origin, pz[3:7]))
    except Exception as e:  # noqa: BLE001
        _M["boards"] = None
        _M["error"] = repr(e)
    _CLOCK["steps"] += 1


def update(objects):
    if objects is _MEM["last"]:
        return _MEM
    _MEM["last"] = objects
    a, b = objects["dowel_board"], objects["hole_board"]
    m = measure(a.pos, a.quat_xyzw, b.pos, b.quat_xyzw)
    r = rung(m)
    still = all(float(np.linalg.norm(o.lin_vel)) < STILL for o in (a, b))
    clear = pads_clear(_M["pads"], [(a.pos, a.quat_xyzw), (b.pos, b.quat_xyzw)])
    done = bool(r == 3 and clear and still)
    _MEM["held"] = _MEM["held"] + (_CLOCK["dt"] or 0.0) if done else 0.0
    _MEM["now"] = {"m": m, "rung": r, "clear": clear, "still": still, "done": done}
    return _MEM


def success(objects):
    """Seated, both hands let go and clear, both boards still (yam_arena ends the episode once this holds 1 s)."""
    return update(objects)["now"]["done"]


LADDER = {0: 0.0, 1: 0.25, 2: 0.5, 3: 0.75}


def score(objects):
    """From the last step only: 1.0 seated, let go and still for 1 s; 0.9 the same, under 1 s; 0.75 seated; 0.5 more
    than half in; 0.25 every dowel's tip in its own hole; else 0."""
    n = update(objects)["now"]
    if n["done"]:
        return 1.0 if _MEM["held"] >= SUCCESS_HOLD_S - 1e-9 else 0.9
    return LADDER[n["rung"]]


def distance(objects):
    return 1.0 - score(objects)


# -- demo ------------------------------------------------------------------------------------------------------------------
# Both hands grip their board top-down across its width (the pads close along x and centre it under the TCP), slide
# it along the bench to just short of the dowels' tips, wait for the arms to settle, push the two together slowly,
# hold, let go and rise.
YAW_L = -math.pi / 2             # left hand's yaw: joint6 near 60 to 94 deg over the plan [RUN: ik]
YAW_R = math.pi / 2              # right hand's: the mirror
GRIP_DY = 0.025                  # each grip 25 mm out from its board's centre, away from the seam [CHOICE]
Z_GRIP = TOP + 0.015             # TCP: the pad tips 2 mm over the bench, the pads on the board's sides
Z_HOVER = TOP + 0.070            # TCP over a board before the descent (pad tips 38 mm over its top)
Z_UP = TOP + 0.100               # TCP after letting go
OPEN = -0.0465                   # finger target fully open (pads 93 mm apart, the board 70 mm)
GRIP = -0.030                    # finger target gripping (5 mm past the board's 35 mm half width) [CHOICE]
LETGO = -0.0465
F_PRESS = 1.5                    # N each hand pushes past the seat [CHOICE: over the 0.33 N bench slide of each board]
K_Y = 180.0                      # N/m: each arm's TCP stiffness along y at its seat grip [RUN: tools/stiffness.py,
#                                  J K^-1 J^T from the joints' drive stiffnesses]
PRESS = F_PRESS / K_Y            # 8.3 mm past the seat each way [ARITH]
STEADY = 0.8
MOUTH_GAP = 0.022                # the seam gap of the last line-ups: dowel tips 2 mm short of the hole mouths [CHOICE]
SHUFFLE_GAPS = (0.026, 0.024, MOUTH_GAP, MOUTH_GAP)  # each line-up after the first: back off SHUFFLE, come in to this
SHUFFLE = 0.016                  # [CHOICE: each hand backs 8 mm, past its 2.9 mm band below] WHY: the 2026-11-18 probe 2 showed a correction made with the boards at rest
#                                  is only 10 to 60 % carried out (the bench friction holds the board inside the arm's
#                                  compliance) and the rest lands at the next slide, so the next pass overshoots; a short
#                                  slide after every pass lets each correction land before the next is measured
LINE_UP = ("settle, dowel tips 10 mm short",) + tuple(f"line up {k + 1}" for k in range(len(SHUFFLE_GAPS)))
LAG = 0.007                      # m: seam gap the boards stop short of a commanded inward gap [RUN: batch 4 s_half,
#                                  commanded 8 mm, stopped at 15; ARITH 5.9 mm = 2 x 0.40 x 0.135 kg x 9.81 / 180 N/m,
#                                  the bench's static friction inside each arm's compliance]. WHY: batch 4's come-ins
#                                  to 22 mm left the boards at 30 mm, so the line-up never slid them and its
#                                  corrections all landed in the push
PROBE_GAP = 0.015                # each try pushes to this gap (tips 5 mm in if they entered), commanded LAG closer
#                                  [CHOICE]
ENTERED = 0.003                  # a try counts as in when every tip is this deep in its own hole [CHOICE]
RETRIES = 2                      # tries after the first [CHOICE]
RETRY_GAP = 0.026                # a retry backs the boards off to this gap (commanded LAG wider: they lag outward too),
#                                  lines up, and comes in again [CHOICE]
# WHY the tries: batch 4 (2026-11-18) seated layout 1 but in layouts 0 and 2 the ends met the mouths 0.25 to 0.46 mm
# off, past the 0.2 mm bore, and every dowel stopped at the end of its 1 mm chamfer; a correction that small only lands
# while the board slides, so a stalled try backs off, lines up and comes in again, as a person would
LINE_UP_MAX = (0.006, math.radians(4.0))  # the most one pass moves a hand (m, rad) [CHOICE: past the 4 mm, 0.7 deg
#                                  seen in the 2026-11-18 probe, far under a stem's deliberate offset]
SERVO_VARIANTS = ("demo", "half", "offset", "off", "skew", "relay")
VARIANT = "demo"                 # sim/mk.sh sets this in stage copies: see stem_plan()


def _w(t, p, f, label, yaw):
    return Waypoint(float(t), (float(p[0]), float(p[1]), float(p[2])), float(f), label, float(yaw))


def grip_point(name, x, y, yaw):
    """A board's grip point (world x, y) for its centre (x, y) and yaw: GRIP_DY out from the centre along the board,
    away from the seam."""
    s = 1.0 if name == "hole_board" else -1.0
    return (x - s * GRIP_DY * math.sin(yaw), y + s * GRIP_DY * math.cos(yaw))


def seat_centres(seam=SEAM_Y, dx=0.0):
    """Board centres (world y) with the ends meeting at seam: the dowel board's end on the seam."""
    return {"dowel_board": (X_LINE, seam - BOARD_L / 2), "hole_board": (X_LINE + dx, seam + BOARD_L / 2)}


def hand_plan(name, start, approach, gap_end, b_dx=0.0, b_yaw=0.0, letgo=True, delay=0.0, about_end=False, park=False,
              retries=RETRIES):
    """One hand's waypoints for one board. start: (x, y, z, yaw) of the board. approach: pre-seat gap (m). gap_end:
    the seam gap commanded at the end of the push (negative presses past the seat). b_dx, b_yaw: the hole board held
    off line (the stems): its centre b_dx aside and turned b_yaw about it, or with about_end its end's centre b_dx
    aside and turned about that. park: only set the board down on its seat (no line-up slides, no tries); retries:
    how many retries the tries hold time for."""
    yaw0 = (YAW_L if name == "hole_board" else YAW_R) + start[3]
    yaw1 = (YAW_L + b_yaw) if name == "hole_board" else YAW_R
    s = 1.0 if name == "hole_board" else -1.0
    gx, gy = grip_point(name, start[0], start[1], start[3])
    seat = seat_centres(dx=b_dx)[name]
    if name == "hole_board" and about_end:
        seat = (seat[0] - BOARD_L / 2 * math.sin(b_yaw), seat[1])
    pre = (seat[0], seat[1] + s * approach / 2)
    end = (seat[0], seat[1] + s * gap_end / 2)
    by = b_yaw if name == "hole_board" else 0.0
    gpre = grip_point(name, pre[0], pre[1], by)
    gend = grip_point(name, end[0], end[1], by)
    cycles = []
    for k, g in enumerate(SHUFFLE_GAPS):
        g = min(approach, g)
        back = grip_point(name, seat[0], seat[1] + s * (g + SHUFFLE) / 2, by)
        come = grip_point(name, seat[0], seat[1] + s * max(0.0, g - LAG) / 2, by)
        cycles += [_w(0.6, (back[0], back[1], Z_GRIP), GRIP, f"back off {k + 1}", yaw1),
                   _w(0.6, (come[0], come[1], Z_GRIP), GRIP, f"come in {k + 1}", yaw1),
                   _w(0.4, (come[0], come[1], Z_GRIP), GRIP, LINE_UP[k + 1], yaw1)]
    W = [_w(2.5 + delay, (gx, gy, Z_HOVER), OPEN, "over its board", yaw0),
         _w(STEADY, (gx, gy, Z_HOVER), OPEN, "steady", yaw0),
         _w(1.5, (gx, gy, Z_GRIP), OPEN, "down round the board", yaw0),
         _w(0.6, (gx, gy, Z_GRIP), GRIP, "grip across its width", yaw0),
         _w(5.0, (gpre[0], gpre[1], Z_GRIP), GRIP, "slide it toward the seam", yaw1),
         _w(1.0, (gpre[0], gpre[1], Z_GRIP), GRIP, LINE_UP[0], yaw1),
         *([] if park else cycles),
         *([] if park else tries(name, seat, approach, by, yaw1, retries)),
         _w(3.0, (gend[0], gend[1], Z_GRIP), GRIP, "push the joint closed slowly", yaw1),
         _w(1.5, (gend[0], gend[1], Z_GRIP), GRIP, "hold the push", yaw1)]
    if letgo:
        # WHY ease first: at the full press each arm is bent 8 mm off its aim, and opening there lets the hand spring
        # along the board while the pads still touch it; eased to a quarter of the press it still pushes the seam shut
        ease = (end[0], end[1] - s * 0.75 * min(0.0, gap_end) / 2)
        gease = grip_point(name, ease[0], ease[1], b_yaw if name == "hole_board" else 0.0)
        gend = gease
        W += [_w(1.0, (gend[0], gend[1], Z_GRIP), GRIP, "ease the push to a quarter", yaw1),
              _w(0.6, (gend[0], gend[1], Z_GRIP), LETGO, "let go", yaw1),
              _w(1.2, (gend[0], gend[1], Z_UP), LETGO, "rise clear", yaw1),
              _w(2.0, (gend[0], gend[1] + s * 0.06, Z_UP), LETGO, "draw back", yaw1)]
    return W


def tries(name, seat, approach, by, yaw1, retries=RETRIES):
    """The first try and RETRIES more: push to PROBE_GAP and check; a retry backs off to RETRY_GAP, lines up there,
    comes in to the mouths, lines up again, and pushes. servo() turns the retries into holds once a check finds the
    dowels in."""
    s = 1.0 if name == "hole_board" else -1.0
    # WHY approach 0 holds the seat: the stack stem's right hand only sets its board on the seam line
    at = lambda g: grip_point(name, seat[0], seat[1] + s * (g if approach > 0 else 0.0) / 2, by)  # noqa: E731
    pr, back, mouth = at(PROBE_GAP - LAG), at(RETRY_GAP + LAG), at(MOUTH_GAP - LAG)
    W = [_w(2.0, (pr[0], pr[1], Z_GRIP), GRIP, "push to the holes", yaw1),
         _w(0.4, (pr[0], pr[1], Z_GRIP), GRIP, "check 1", yaw1)]
    for r in range(1, retries + 1):
        W += [_w(1.0, (back[0], back[1], Z_GRIP), GRIP, f"retry {r} back off", yaw1),
              _w(0.4, (back[0], back[1], Z_GRIP), GRIP, f"retry {r} line up", yaw1),
              _w(0.8, (mouth[0], mouth[1], Z_GRIP), GRIP, f"retry {r} come in", yaw1),
              _w(0.4, (mouth[0], mouth[1], Z_GRIP), GRIP, f"retry {r} line up at the mouths", yaw1),
              _w(2.0, (pr[0], pr[1], Z_GRIP), GRIP, f"retry {r} push to the holes", yaw1),
              _w(0.4, (pr[0], pr[1], Z_GRIP), GRIP, f"check {r + 1}", yaw1)]
    return W


def servo_events(W):
    """(time, kind, label) of every line-up pass and check in a hand's plan, in order; each 0.1 s before its
    waypoint ends."""
    out, t = [], 0.0
    for w in W:
        t += w.duration
        if w.label in LINE_UP or (w.label.startswith("retry") and "line up" in w.label):
            out.append((t - 0.1, "line", w.label))
        elif w.label.startswith("check"):
            out.append((t - 0.1, "check", w.label))
    return out


def plan(seed=None, variant=None):
    """Both hands' waypoints for a seed and a variant (demo or a stem); also arms the demo's line-up (_SERVO)."""
    v = VARIANT if variant is None else variant
    p = _plan(seed, v)
    b_dx, b_yaw = {"offset": (PITCH, 0.0), "off": OFF, "skew": (0.0, SKEW)}.get(v, (0.0, 0.0))
    end_dx = b_dx if v == "off" else b_dx + BOARD_L / 2 * math.sin(b_yaw)   # the end's centre aside, as planned
    left = p.get("left", [])
    _SERVO.update({"on": v in SERVO_VARIANTS, "b_dx": end_dx, "b_yaw": b_yaw, "done": set(), "log": [],
                   "times": [t - 0.1 for t in label_ends(left, LINE_UP)], "events": servo_events(left),
                   "entered": False})
    return p


OFF = (0.0005, math.radians(1.0))   # the probe's hole board: its end's centre 0.5 mm aside, turned 1 deg about it;
#                                     the tips meet the mouths 0.5 mm off, inside the 1.2 mm capture (test) [CHOICE]
SKEW = math.radians(5.0)            # the skew stem: turned about its centre, the end 6.5 mm aside [ARITH: 75 sin 5 deg]


def _plan(seed, v):
    lay = layout(seed)
    A, B = lay["dowel_board"], lay["hole_board"]
    push = -PRESS * 2          # the commanded gap: both hands aim PRESS past the seat
    if v == "demo":
        return {"left": hand_plan("hole_board", B, PRE_GAP, push), "right": hand_plan("dowel_board", A, PRE_GAP, push)}
    if v == "nothing":
        return {}
    if v == "half":           # push only to an 8 mm gap and let go: more than half in, not seated
        # WHY minus LAG: batch 4 commanded 8 mm and the boards stopped at 15 (0.25, not 0.5)
        return {"left": hand_plan("hole_board", B, PRE_GAP, 0.008 - LAG),
                "right": hand_plan("dowel_board", A, PRE_GAP, 0.008 - LAG)}
    if v == "offset":         # the hole board held one dowel pitch to the side: two dowels find holes, one misses
        return {"left": hand_plan("hole_board", B, PRE_GAP, push, b_dx=PITCH),
                "right": hand_plan("dowel_board", A, PRE_GAP, push)}
    if v == "off":            # probe: the hole board's end held 0.5 mm to the side and turned 1 deg (OFF)
        return {"left": hand_plan("hole_board", B, PRE_GAP, push, b_dx=OFF[0], b_yaw=OFF[1], about_end=True),
                "right": hand_plan("dowel_board", A, PRE_GAP, push)}
    if v == "skew":           # the hole board turned 5 deg: both outer dowels outside their capture
        return {"left": hand_plan("hole_board", B, PRE_GAP, push, b_yaw=SKEW),
                "right": hand_plan("dowel_board", A, PRE_GAP, push)}
    if v == "left_only":      # the left hand carries its board as far toward the other as it reaches; right idles
        far = (B[0], A[1] + BOARD_L / 2 + BOARD_L / 2 + 0.002, Z_C, 0.0)
        return {"left": hand_plan("hole_board", B, PRE_GAP, far[1] - seat_centres()["hole_board"][1])}
    if v == "relay":          # one hand at a time: right parks its board on the seam line and lets go, then left
        # WHY park and one retry: one arm after the other must still fit the 60 s run limit (test)
        right = hand_plan("dowel_board", A, 0.0, 0.0, park=True)
        t_right = sum(w.duration for w in right)
        left = hand_plan("hole_board", B, PRE_GAP, push, delay=t_right, retries=1)
        return {"left": left, "right": right}
    if v == "stack":          # the left hand lifts its board and sets it down on the dowel board's end
        return {"right": hand_plan("dowel_board", A, 0.0, 0.0), "left": stack_plan(B)}
    raise ValueError(f"unknown variant {v!r}")


def stack_plan(B):
    gx, gy = grip_point("hole_board", B[0], B[1], 0.0)
    seat = seat_centres()["hole_board"]
    gs = grip_point("hole_board", seat[0], seat[1] - 0.040, 0.0)
    z_lift = Z_GRIP + BOARD_T + 0.025
    return [_w(2.5, (gx, gy, Z_HOVER), OPEN, "over its board", YAW_L),
            _w(STEADY, (gx, gy, Z_HOVER), OPEN, "steady", YAW_L),
            _w(1.5, (gx, gy, Z_GRIP), OPEN, "down", YAW_L),
            _w(0.6, (gx, gy, Z_GRIP), GRIP, "grip", YAW_L),
            _w(1.2, (gx, gy, z_lift), GRIP, "lift", YAW_L),
            _w(14.0, (gs[0], gs[1], z_lift), GRIP, "carry over the other board", YAW_L),
            _w(1.5, (gs[0], gs[1], Z_GRIP + BOARD_T + 0.002), GRIP, "set it on the other board's end", YAW_L),
            _w(0.6, (gs[0], gs[1], Z_GRIP + BOARD_T + 0.002), LETGO, "let go", YAW_L),
            _w(1.2, (gs[0], gs[1], Z_UP + 0.02), LETGO, "rise", YAW_L)]


def demo():
    """Both hands: grip, slide together, line the boards up, push the joint closed, let go."""
    return plan(run_seed())


# -- the demo's line-up: a privileged correction from the boards' true poses (the score never reads it) -----------------
# WHY: the 2026-11-18 probe gripped and pushed well but every board came out of the grip turned about 0.7 deg the same
# way; turned about grips 100 mm from the seam each, that put the ends 2.4 mm apart sideways [ARITH: 200 mm x sin 0.7
# deg], twice the 1.2 mm capture, and every dowel stopped on the end face. A real worker sights along the edges; the
# script reads the poses instead.
_SERVO = {"on": False, "b_dx": 0.0, "b_yaw": 0.0, "done": set(), "log": [], "times": [], "events": [],
          "entered": False}


def _yaw_of(q):
    R = _rot3(q)
    return math.atan2(R[1, 0], R[0, 0])


def _turn_about(pt, pivot, ang):
    """A point (x, y) turned by ang about a pivot (x, y)."""
    c, s = math.cos(ang), math.sin(ang)
    dx, dy = pt[0] - pivot[0], pt[1] - pivot[1]
    return np.array([pivot[0] + c * dx - s * dy, pivot[1] + s * dx + c * dy])


def label_ends(waypoints, labels):
    """The plan time (s) at which each labelled waypoint ends, in plan order."""
    t, out = 0.0, []
    for w in waypoints:
        t += w.duration
        if w.label in labels:
            out.append(t)
    return out


def line_up(a_pos, a_quat, b_pos, b_quat, tcp_a, tcp_b, b_dx=0.0, b_yaw=0.0, turn_a=True):
    """One line-up pass: what to add to each hand's later waypoints, {arm: (x shift m, yaw rad)}. The right hand turns
    the dowel board square to the bench's y (if turn_a); the left hand turns the hole board to b_yaw from the dowel
    board's yaw after that turn, and shifts it so its end's centre lands b_dx (along x) from the dowel board's end's
    centre, once both turns are made. turn_a is off once the right hand has let go, and for the last pass, so that pass
    never leaves a turn of the dowel board still to land as the push starts.
    A turn of the hand turns its board about the TCP (the pads hold the board there)."""
    ya, yb = _yaw_of(a_quat), _yaw_of(b_quat)
    da = -ya if turn_a else 0.0
    db = (b_yaw + (0.0 if turn_a else ya)) - yb
    end_a = np.asarray(a_pos, float) + _rot3(a_quat) @ np.array([0.0, BOARD_L / 2, 0.0])
    end_b = np.asarray(b_pos, float) + _rot3(b_quat) @ np.array([0.0, -BOARD_L / 2, 0.0])
    na = _turn_about(end_a, tcp_a, da)
    nb = _turn_about(end_b, tcp_b, db)
    dx = na[0] + b_dx - nb[0]
    clip = lambda v, m: float(max(-m, min(m, v)))  # noqa: E731
    out = {"left": (clip(dx, LINE_UP_MAX[0]), clip(db, LINE_UP_MAX[1]))}
    if turn_a:
        out["right"] = (0.0, clip(da, LINE_UP_MAX[1]))
    return out


def shift_after(waypoints, t, dx, dyaw):
    """Waypoints with every one after the one playing at plan time t moved dx along x and turned dyaw. WHY only later
    ones: the playing segment's goal stays put, so the hand never jumps."""
    end, i = 0.0, len(waypoints)
    for k, w in enumerate(waypoints):
        end += w.duration
        if t < end:
            i = k
            break
    return waypoints[:i + 1] + [dataclasses.replace(w, pos=(w.pos[0] + dx, w.pos[1], w.pos[2]), yaw=w.yaw + dyaw)
                                for w in waypoints[i + 1:]]


def _holding(waypoints, t):
    """True while the plan at time t is before its let-go."""
    end = 0.0
    for w in waypoints:
        end += w.duration
        if w.label == "let go":
            return t < end - w.duration
    return t < end


def entered(m):
    """Every dowel's tip at least ENTERED into its own hole and within the capture of its axis."""
    return min(m["depth"]) >= ENTERED and max(m["off"]) <= CAPTURE


def hold_retries(waypoints, t):
    """Waypoints with every retry and check after time t held where the playing one ends (the dowels are in)."""
    end, i = 0.0, len(waypoints)
    for k, w in enumerate(waypoints):
        end += w.duration
        if t < end:
            i = k
            break
    here = waypoints[i] if i < len(waypoints) else waypoints[-1]
    return waypoints[:i + 1] + [dataclasses.replace(w, pos=here.pos, yaw=here.yaw)
                                if w.label.startswith(("retry", "check")) else w for w in waypoints[i + 1:]]


def servo(plans, t):
    """Run each line-up pass and check whose time has come: plans is {arm: ArmPlan}; edits their waypoints in place.
    The last pass of a line-up (before a push) turns only the hole board; a check that finds the dowels in holds the
    retries and skips their passes."""
    if not _SERVO["on"] or _M.get("boards") is None or _M.get("tcps") is None:
        return
    ev = _SERVO["events"]
    for k, (tk, kind, label) in enumerate(ev):
        if k in _SERVO["done"] or t < tk:
            continue
        _SERVO["done"].add(k)
        if _SERVO["entered"]:
            continue
        pa, qa, pb, qb = _M["boards"]
        m = measure(pa, qa, pb, qb)
        d = {}
        if kind == "check":
            if entered(m):
                _SERVO["entered"] = True
                for arm in plans:
                    plans[arm].waypoints = hold_retries(plans[arm].waypoints, t)
        else:
            held = "right" in plans and _holding(plans["right"].waypoints, t)
            last = k + 1 < len(ev) and ev[k + 1][1] == "check" or "mouths" in label
            d = line_up(pa, qa, pb, qb, _M["tcps"]["right"], _M["tcps"]["left"], _SERVO["b_dx"], _SERVO["b_yaw"],
                        held and not last)
            for arm, (dx, dyaw) in d.items():
                if arm in plans:
                    plans[arm].waypoints = shift_after(plans[arm].waypoints, t, dx, dyaw)
        rec = {"pass": k, "kind": kind, "t": round(t, 2), "dx_mm": round(1000 * m["dx"], 3),
               "relyaw_deg": round(math.degrees(m["yaw"]), 3), "yaw_a_deg": round(math.degrees(_yaw_of(qa)), 3),
               "yaw_b_deg": round(math.degrees(_yaw_of(qb)), 3), "depth_mm": [round(1000 * v, 2) for v in m["depth"]],
               "entered": _SERVO["entered"],
               "cmd": {a: (round(1000 * v[0], 3), round(math.degrees(v[1]), 3)) for a, v in d.items()}}
        _SERVO["log"].append(rec)
        print("[dowel line-up] " + str(rec), flush=True)


def _replan_demo_after_reset():
    """WHY: inspect-robots resets the policy before the embodiment, so yam_scripted would call demo() before the reset
    has set this trial's seed. This replans on the first action, after the reset, and runs the demo's line-up passes
    before each action. It changes only the scripted demo policy, never the scene or the score."""
    try:
        from yam_arena import scripted
    except ImportError:
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_dj_replan", False):
        return
    reset, act = cls.reset, cls.act
    unset = object()

    def patched_reset(self, scene):
        self._dj_scene, self._dj_seed = scene, unset
        reset(self, scene)

    def patched_act(self, observation):
        if getattr(self, "_dj_seed", None) is unset:
            self._dj_seed = run_seed()
            reset(self, self._dj_scene)
        servo(self._plans, float(observation.state_time) + self._dt)
        return act(self, observation)

    cls.reset, cls.act, cls._dj_replan = patched_reset, patched_act, True


_replan_demo_after_reset()
