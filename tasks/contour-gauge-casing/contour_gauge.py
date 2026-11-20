"""Copy a door casing's floor-level profile with a contour gauge and lay the gauge on the vinyl sheet to be fitted.

A door casing stands against its wall at the far side of the bench, where the floor meets it; the doorway opens to the
right of it. A 10 in contour gauge lies flat on the floor with its fingers set straight. A vinyl sheet to be laid
against the casing lies in front of it, slid straight back from its place, with a strip of tape across it on the wall
line. Copy the casing's profile with the gauge, then lay the gauge flat on the sheet with the copied fingertips where
the sheet must be cut, and take both hands away.
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, body, mechanism, on_reset, on_step, run_seed  # noqa: E402,F401
from motionkit import HAND, Plan, SagCorrector, box_distance, down, install, mode, plans_out, together  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 1200           # 60 s at 20 Hz
TOP = TABLE_TOP[SCENE]
MODES = ("demo", "nothing", "one_arm", "no_press", "off_line")
EXPECT = {"demo": 1.0, "nothing": 0.0, "one_arm": 0.0, "no_press": 0.0, "off_line": 0.5}

# Every number: [SOURCE] quoted in the README (saved page in src/), [ARITH] derived, [EST] estimate with a way to
# measure it in the README, [CHOICE] design choice. World: x forward from the arm bases, y left, z up. The bench top is
# the floor. Gauge frame G: origin at the middle of the frame's bottom face, x toward the fingertips, y along the frame.

# -- the wall, the doorway and the casing ---------------------------------------------------------------------------------
X_WALL = 0.400                   # the wall's face, world x [CHOICE: hands at the copy at x 0.36, inside top-down reach]
Y_IN = -0.0605                   # the casing's inner (doorway) edge, world y [CHOICE: in a gap between fingers]
# WHY -60.5 mm and 4 mm fingers: the finger beside the casing's widest step was dragged off its copy by a 1 mm
# sideways drift (toward -y) of the hands in the draw back (GPU traces); this puts 1.9 mm between that finger and
# the step; the finger beside the 6.4 mm step has 0.85 mm, on the side the drift moves away from
REVEAL = 0.003                   # the casing's inner edge stands this far into the opening past the jamb's face
#                                  [CHOICE: a common 1/8 in reveal, not load-bearing]
STOCK_W, STOCK_T = 0.0635, 0.0191  # 1x3 S4S pine casing stock, 2-1/2 x 3/4 in dressed [SOURCE: lumber dressed sizes]
INNER_RAB = (0.0064, 0.0064)     # rabbet on the doorway arris, width x depth: 1/4 x 1/4 in [CHOICE: the recipe]
OUTER_RAB = (0.0159, 0.0095)     # rabbet on the wall-side arris, width x depth: 5/8 x 3/8 in [CHOICE: the recipe]
CASING_H = 0.070                 # height of the wall, jamb and casing mock-up [CHOICE: README "The fitting place"; at the
#                                  copy the gripper housing reaches 9 mm past the wall's face from 85 mm up, and link4
#                                  59 mm past it from 156 mm up (checkkit), so a taller wall blocks the arm]
WALL_D = 0.120                   # wall stub depth behind its face [CHOICE]


def profile():
    """The casing's floor-level outline: (u0, u1, t) runs, u from the doorway edge, t its face's distance from the
    wall (m). Made from 1x3 stock by the README's two table-saw rabbets [ARITH]."""
    wi, di = INNER_RAB
    wo, do = OUTER_RAB
    return ((0.0, wi, STOCK_T - di), (wi, STOCK_W - wo, STOCK_T), (STOCK_W - wo, STOCK_W, STOCK_T - do))


PROFILE = profile()


def outline_t(y0, y1):
    """What a fingertip spanning world y0..y1 meets, as its distance in front of the wall's face (m): the largest t
    under it (a finger stops on the highest point it covers); 0 over bare wall; None over the doorway (nothing)."""
    best = None
    for u0, u1, t in PROFILE:
        if y1 > Y_IN + u0 and y0 < Y_IN + u1:
            best = t if best is None else max(best, t)
    if y1 > Y_IN + STOCK_W:      # the wall beside the casing
        best = 0.0 if best is None else best
    return best


PINE, WALL_C, JAMB_C = (0.82, 0.70, 0.50), (0.88, 0.87, 0.84), (0.76, 0.64, 0.45)


def fixed_boxes():
    z0, z1 = TOP, TOP + CASING_H
    out = [("wall", (X_WALL, Y_IN + REVEAL, z0), (X_WALL + WALL_D, 0.300, z1), WALL_C)]
    for i, (u0, u1, t) in enumerate(PROFILE):
        out.append((f"casing_{i}", (X_WALL - t, Y_IN + u0, z0), (X_WALL, Y_IN + u1, z1), PINE))
    return out


def _fixed(name, lo, hi, color):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Box(name=name, pos=tuple(((lo + hi) / 2).tolist()), size=tuple((hi - lo).tolist()), color=color)


# -- the contour gauge (10 in, plastic, fingers held by friction) ---------------------------------------------------------
FINGER_L = 0.0762                # finger length [SOURCE: "3 in. deep fingers"]
TRAVEL = 0.0318                  # finger travel, the deepest profile [SOURCE: "profiles up to 1-1/4 in."]
SPAN = 0.252                     # fingers' span along the frame [SOURCE: 10 in profile length; ARITH: 42 x 6 mm]
PITCH = 0.006                    # model finger pitch [EST: one model finger stands for several real ones]
N_FING = round(SPAN / PITCH)     # 42 [ARITH]
FING_W, FING_H = 0.004, 0.008    # model finger width (y) and height (z) [EST]
FRAME_D, FRAME_H = 0.020, 0.012  # frame depth (x) and thickness (z) [EST]
PROTRUDE = 0.040                 # fingertips in front of the frame with the fingers set straight [EST]
BLOCK_L = 0.030                  # solid end block past the last finger at each end, along y [EST]
FING_KG, FRAME_KG, BLOCK_KG = 0.0012, 0.030, 0.010   # [EST: about 100 g in all]
F_HOLD = 0.15                    # force that slides one finger in the frame [EST, measure as in the README]
K_FING = 400.0                   # N/m, the stiffness of the model's finger hold below F_HOLD [CHOICE: 0.4 mm give]
D_FING = 1.0                     # N s/m, finger damping [CHOICE: about critical for 1.2 g on 400 N/m]
ABS = (0.40, 0.35)               # plastic on wood and vinyl, static / dynamic [EST]
VINYL = (0.50, 0.45)             # [EST]
FX0 = FRAME_D / 2                # frame front, G x [ARITH]
TIP0 = FX0 + PROTRUDE            # fingertip at rest (fingers straight), G x [ARITH: 50 mm]
TAIL0 = TIP0 - FINGER_L          # finger tail at rest, G x [ARITH]
FZ = (FRAME_H - FING_H) / 2      # finger bottom over the frame's bottom [ARITH: 2 mm]
YF = [(i - (N_FING - 1) / 2) * PITCH for i in range(N_FING)]   # finger centres, G y [ARITH]
YB = SPAN / 2 + BLOCK_L / 2      # end block centres, G y [ARITH: 141 mm]
GREY, ORANGE, DARK = (0.55, 0.56, 0.58), (0.95, 0.50, 0.10), (0.20, 0.20, 0.22)


def _box(lo, hi, color, material, mass=0.0):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Shape("box", tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, material=material,
                 mass=float(mass))


def gauge(pos):
    frame = Link("frame", FRAME_KG + 2 * BLOCK_KG, (
        _box((-FX0, -SPAN / 2, 0.0), (FX0, SPAN / 2, FRAME_H), DARK, "abs"),
        _box((-FX0, SPAN / 2, 0.0), (FX0, SPAN / 2 + BLOCK_L, FRAME_H), ORANGE, "abs"),
        _box((-FX0, -SPAN / 2 - BLOCK_L, 0.0), (FX0, -SPAN / 2, FRAME_H), ORANGE, "abs")))
    links, joints = [], []
    for i, y in enumerate(YF):
        links.append(Link(f"f{i:02d}", FING_KG, (
            _box((TAIL0, y - FING_W / 2, FZ), (TIP0, y + FING_W / 2, FZ + FING_H), GREY, "abs"),)))
        joints.append(Joint(f"j{i:02d}", "prismatic", f"f{i:02d}", (0.0, y, FZ + FING_H / 2), "X",
                            lower=-TRAVEL, upper=0.0, stiffness=K_FING, damping=D_FING, target=0.0))
    return mechanism("gauge", pos, frame, links, joints, {"abs": ABS}, self_collide=False, free=True, color=DARK)


# -- the vinyl sheet to be fitted ---------------------------------------------------------------------------------------------
# Sheet frame S: origin on its doorway edge at the tape line, on its bottom face; x toward the wall, y away from the door.
SHEET_T = 0.0032                 # 1/8 in vinyl [EST: measure with calipers]
SHEET_BACK, SHEET_PAST = 0.120, 0.015   # sheet behind and past the tape line, S x [CHOICE]
SHEET_W = 0.170                  # sheet width, S y [CHOICE: covers the casing and 106 mm of wall line]
TAPE_W = 0.012                   # tape strip, its near edge on the wall line [CHOICE: 1/2 in masking tape]
SHEET_KG = 0.120                 # [EST: 0.135 x 0.170 x 3.2 mm at about 1600 kg/m3]
SLIDE = 0.160                    # the sheet lies this far short of its place, straight back from the wall [CHOICE]
X_TAPE = X_WALL - SLIDE          # the tape line, world x [ARITH: 0.240]
SHEET_ORIGIN = (X_TAPE, Y_IN, TOP)   # S origin, world: its doorway edge in line with the casing's inner edge [CHOICE]
BLUE, SHEET_C = (0.20, 0.35, 0.85), (0.70, 0.72, 0.66)
SPAWN_GAP = 0.0005


def sheet_shapes():
    tape_kg = 0.0005             # [EST]
    out = [_box((-SHEET_BACK, 0.0, 0.0), (SHEET_PAST, SHEET_W, SHEET_T), SHEET_C, "vinyl", SHEET_KG - tape_kg)]
    tape = Shape("box", (TAPE_W / 2, SHEET_W / 2, SHEET_T + 0.0002), (TAPE_W, SHEET_W, 0.0004), BLUE,
                 material="vinyl", mass=tape_kg, collide=False)
    return out + [tape]


# -- the start ----------------------------------------------------------------------------------------------------------
GAUGE_START = (0.290, 0.0)       # frame centre x, y [CHOICE: on the floor between the sheet and the casing]
GAUGE = gauge((*GAUGE_START, TOP + SPAWN_GAP))
SHEET = body("sheet", (SHEET_ORIGIN[0], SHEET_ORIGIN[1], TOP + SPAWN_GAP), sheet_shapes(), {"vinyl": VINYL},
             color=SHEET_C)
OBJECTS = [*(_fixed(*f) for f in fixed_boxes()), GAUGE, SHEET]


# -- small maths --------------------------------------------------------------------------------------------------------
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


# -- the finger hold (Coulomb-like: a finger stays put until F_HOLD pushes it) ----------------------------------------------
_HOLD = {"target": None}


def slip(q, target, f_hold=F_HOLD, k=K_FING):
    """New spring targets: where a finger is pushed past its hold, its target follows it, F_HOLD / k behind, so the
    spring never pulls harder than F_HOLD; elsewhere the target stays (the finger stays where it was pushed)."""
    q, target = np.asarray(q, float), np.asarray(target, float)
    lag = f_hold / k
    return np.clip(target, q - lag, q + lag)


@on_reset
def _reset_hold():
    _HOLD["target"] = None
    _HANDS.update({"left": None, "right": None})


@on_step
def hold_fingers(env):
    try:
        import torch
        asset = env.unwrapped.scene["gauge"]
        names = list(asset.joint_names)
        ids = [names.index(f"j{i:02d}") for i in range(N_FING)]
        q = _np(asset.data.joint_pos)[0][ids]
        if _HOLD["target"] is None:
            _HOLD["target"] = np.zeros(N_FING)
        new = slip(q, _HOLD["target"])
        if np.any(np.abs(new - _HOLD["target"]) > 1e-7):
            _HOLD["target"] = new
            dev = getattr(asset.data.joint_pos, "torch", asset.data.joint_pos).device
            t = torch.tensor([new.tolist()], dtype=torch.float32, device=dev)
            if hasattr(asset, "set_joint_position_target_index"):
                asset.set_joint_position_target_index(target=t, joint_ids=ids)
            else:
                asset.set_joint_position_target(t, joint_ids=ids)
            asset.write_data_to_sim()
    except Exception as e:  # noqa: BLE001  WHY: never stop the run over the hold model; print once
        if not _HOLD.get("warned"):
            print(f"[contour_gauge] finger hold failed: {e!r}", flush=True)
            _HOLD["warned"] = True


# -- scoring ------------------------------------------------------------------------------------------------------------
_HANDS = {"left": None, "right": None}
FIT_MAX = 0.003                      # copy fit, and the full-score cut line: 1/8 in [CHOICE: README "Scoring"]
NEAR_MAX = 0.006                     # the 0.75 cut line: 1/4 in [CHOICE]
FIT_WIN = 0.020                      # the copy is judged over the casing and 20 mm of wall past it [CHOICE]
CONTACT_MIN = 0.001                  # every finger over casing or wall pushed at least 1 mm [CHOICE]
MAX_SLOPE = math.tan(math.radians(5.0))   # a copy taken with the gauge turned up to 5 deg still counts [CHOICE]
FLAT_DEG = 3.0                       # gauge flat: its z axis within 3 deg of vertical [CHOICE]
REST_TOL = 0.002                     # frame bottom within 2 mm over the sheet's top [CHOICE]
STILL = 0.02                         # m/s [CHOICE]
CLEAR = 0.020                        # every hand point 20 mm from the gauge [CHOICE]
EDGE_IN = 0.0025                     # fingers judged on the sheet: tip centre over half a finger inside its side edges, so
#                                    a finger that copied the open doorway is judged only once it lies on the sheet [CHOICE]
G_LO = np.array([TAIL0 - TRAVEL, -SPAN / 2 - BLOCK_L, 0.0])   # the gauge's bounds in G, fingers fully in or out
G_HI = np.array([TIP0, SPAN / 2 + BLOCK_L, FRAME_H])


def set_hands(points):
    _HANDS.update(points)


@on_step
def read_hands(env):
    try:
        _HANDS.update({side: HAND.from_env(env, side) for side in ("left", "right")})
    except Exception:  # noqa: BLE001  WHY: unread hands count as touching
        _HANDS.update({"left": None, "right": None})


def pushes(g):
    """Each finger's push into the frame (m, >= 0), from the joints."""
    return np.array([-float(g.joints.get(f"j{i:02d}", 0.0)) for i in range(N_FING)])


def copy_fit(s):
    """The better of the two finger orders (the gauge may be used either face up)."""
    s = np.asarray(s, float)
    return min(_copy_fit(s), _copy_fit(s[::-1]))


def _copy_fit(s):
    """How well the fingers' pushes copy the casing: the best (rms, worst) over every place the casing could have sat
    under the gauge (0.5 mm steps) and a straight-line fit (offset and slope within 5 deg) of push against the outline,
    judged over the fingers that meet the casing or the wall within FIT_WIN of it; each must be pushed CONTACT_MIN.
    (inf, inf) if no placement fits."""
    best = (math.inf, math.inf)
    yf = np.asarray(YF)
    for dy in np.arange(-0.120, 0.120 + 1e-9, 0.0005):   # the casing's inner edge at G y = dy
        tv, keep = [], []
        for i, y in enumerate(yf):
            y0, y1 = y - FING_W / 2, y + FING_W / 2
            if y1 < dy or y0 > dy + STOCK_W + FIT_WIN:
                continue
            t = outline_t(y0 - dy + Y_IN, y1 - dy + Y_IN)
            if t is None:
                continue
            tv.append(t)
            keep.append(i)
        if len(keep) < round((STOCK_W + FIT_WIN) / PITCH) - 1 or (yf[keep[0]] - FING_W / 2) < -SPAN / 2:
            continue
        si = s[keep]
        if si.min() < CONTACT_MIN:
            continue
        r = si - np.asarray(tv)   # = push of the bare wall + slope * y, if the copy is right
        A = np.c_[np.ones(len(keep)), yf[keep]]
        c, m = np.linalg.lstsq(A, r, rcond=None)[0]
        if abs(m) > MAX_SLOPE:
            continue
        e = r - (c + m * yf[keep])
        cand = (float(np.sqrt(np.mean(e ** 2))), float(np.abs(e).max()))
        if cand < best:
            best = cand
    return best


def copy_ok(g):
    return copy_fit(pushes(g))[1] <= FIT_MAX


def sheet_frame(sh):
    return np.asarray(sh.pos, float), _rot3(sh.quat_xyzw)


def tips_world(g):
    R, p = _rot3(g.quat_xyzw), np.asarray(g.pos, float)
    s = pushes(g)
    local = np.array([[TIP0 - s[i], YF[i], FZ + FING_H / 2] for i in range(N_FING)])
    return local @ R.T + p, R


def flat_on_sheet(g, sh):
    R = _rot3(g.quat_xyzw)
    if math.degrees(math.acos(min(1.0, abs(float(R[2, 2]))))) > FLAT_DEG:   # either face up
        return False
    if np.linalg.norm(g.lin_vel) > STILL or np.linalg.norm(sh.lin_vel) > STILL:
        return False
    po, Rs = sheet_frame(sh)
    c = (np.asarray(g.pos, float) - po) @ Rs   # the frame's bottom centre in S
    on = -SHEET_BACK <= c[0] <= SHEET_PAST and 0.0 <= c[1] <= SHEET_W
    bottom = c[2] if R[2, 2] > 0 else c[2] - FRAME_H   # face down, the frame's top face rests on the sheet
    return bool(on and abs(bottom - SHEET_T) <= REST_TOL)


def cut_line():
    """The cut the sheet needs, in S (x toward the wall from the tape's near edge, y from the doorway edge), as a
    polyline with its step risers: set back by the casing's outline over the casing, on the tape edge beyond it."""
    pts = []
    for u0, u1, t in PROFILE:
        pts += [(-t, u0), (-t, u1)]
    pts += [(0.0, STOCK_W), (0.0, SHEET_W)]
    return np.array(pts)


CUT = cut_line()


def _to_polyline(p, poly):
    best = math.inf
    for a, b in zip(poly[:-1], poly[1:]):
        ab = b - a
        u = 0.0 if not ab.any() else float(np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0))
        best = min(best, float(np.linalg.norm(p - (a + u * ab))))
    return best


def cut_error(g, sh):
    """The copied tip line against the cut the sheet needs: for each fingertip whose centre lies over the sheet, its
    distance in the sheet's plane to the cut polyline (risers included, so a tip set 1 mm sideways of a step is 1 mm
    off, not the step's height). Returns (rms, worst, count)."""
    tips, _R = tips_world(g)
    po, Rs = sheet_frame(sh)
    e = []
    for i in range(N_FING):
        t_s = (tips[i] - po) @ Rs
        if not EDGE_IN <= t_s[1] <= SHEET_W - EDGE_IN:
            continue
        e.append(_to_polyline(t_s[:2], CUT))
    if not e:
        return math.inf, math.inf, 0
    e = np.asarray(e)
    return float(np.sqrt(np.mean(e ** 2))), float(np.abs(e).max()), len(e)


N_ON_SHEET = int((SHEET_W - 2 * EDGE_IN) / PITCH)   # fingers a correctly placed gauge has on the sheet [ARITH: 27]


def hands_clear(g):
    R, p = _rot3(g.quat_xyzw), np.asarray(g.pos, float)
    for pts in _HANDS.values():
        if pts is None:
            return False
        local = (np.asarray(pts, float) - p) @ R
        if box_distance(local, G_LO, G_HI).min() < CLEAR:
            return False
    return True


def score(objects):
    """0.25 for a good copy; 0.5 when the copying gauge also lies flat and still on the sheet; 0.75 when every tip on
    the sheet is within 6 mm of the cut; 1.0 within 3 mm with both hands 20 mm clear. Else 0."""
    g, sh = objects["gauge"], objects["sheet"]
    if not copy_ok(g):
        return 0.0
    if not flat_on_sheet(g, sh):
        return 0.25
    rms, worst, n = cut_error(g, sh)
    if n < N_ON_SHEET - 1 or worst > NEAR_MAX:
        return 0.5
    if worst > FIT_MAX or not hands_clear(g):
        return 0.75
    return 1.0


def success(objects):
    return score(objects) >= 1.0


def distance(objects):
    return 1.0 - score(objects)


# -- demo (motionkit; scene constants and the arms' own joint readings only) ----------------------------------------------
install(SagCorrector())
YAW = {"left": -90.0, "right": 90.0}   # fingers close along x on each end block [CHOICE: checkkit reach]
TCP_H = 0.0145                   # TCP over the gauge's bottom when holding it: pad tips 1.5 mm over its bottom [ARITH:
#                                  TCP to pad tips 13 mm (yam_arena TCP_OFFSET and the pad mesh)]
CREEP = 0.0015                   # the corrector overshoots its push target [RUN: 1.6 mm, GPU trace]
D_COPY = 0.032                   # frame front to wall at the copy [CHOICE: inside the 26 to 39 mm window, README]
X_COPY = X_WALL - D_COPY - FX0   # frame centre at the copy [ARITH: 0.358]
X_NEAR = X_WALL - STOCK_T - 0.004 - TIP0   # frame centre with the straight fingertips 4 mm short of the casing [ARITH]
X_MARK = X_COPY - SLIDE          # frame centre on the sheet: the copy pose slid straight back [ARITH]
LIFT = 0.030


def _corners(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return np.array([[(lo, hi)[i][0], (lo, hi)[j][1], (lo, hi)[k][2]] for i in (0, 1) for j in (0, 1) for k in (0, 1)])


FINGER_ENV = (np.array([TAIL0 - TRAVEL, -SPAN / 2, FZ]), np.array([TIP0, SPAN / 2, FZ + FING_H]))   # G, any setting


def check_parts(seed=None, mode_name="demo"):
    """checkkit's parts: every fixed box, the sheet, and the gauge's rigid frame with its end blocks (carried when
    picked). The fingers are left out here on purpose: they are pushed into the casing by design, and the pads' fit
    beside them is its own test (test_pads_clear_fingers)."""
    out = [(n, _corners(lo, hi)) for n, lo, hi, _c in fixed_boxes()]
    o = np.array(SHEET_ORIGIN)
    out.append(("sheet", _corners(o + (-SHEET_BACK, 0.0, 0.0), o + (SHEET_PAST, SHEET_W, SHEET_T))))
    g = np.array([*GAUGE_START, TOP])
    out.append(("gauge", _corners(g + (-FX0, -SPAN / 2 - BLOCK_L, 0.0), g + (FX0, SPAN / 2 + BLOCK_L, FRAME_H))))
    return out


def plan(seed=None, mode_name="demo"):
    if mode_name == "nothing":
        return {}
    arms = ("right",) if mode_name == "one_arm" else ("left", "right")
    off = 0.012 if mode_name == "off_line" else 0.0   # 12 mm sideways on the sheet
    plans = []
    for side in arms:
        sy = 1.0 if side == "left" else -1.0
        yb = GAUGE_START[1] + sy * YB
        p = Plan(side, where={"gauge": (GAUGE_START[0], yb, TOP)})
        p.travel(0.12, to=(GAUGE_START[0], yb))
        p.pick("gauge", h=TCP_H, width=FRAME_D, grip=down(YAW[side]), above=0.08, lift=LIFT, force=15.0,
               label=f"pick {side}")
        plans.append(p)
    together(*plans)
    for p in plans:
        yb = p.pos[1]
        p.move((X_NEAR, yb, TOP + TCP_H + LIFT), t=1.5, label="over the floor before the casing")
        p.move((X_NEAR, yb, TOP + TCP_H), t=1.5, label="down on the floor", contact=True)
        if mode_name != "no_press":
            # WHY not contact: SagCorrector keeps trimming here, from the arm's own joint readings, so the frame
            # reaches the commanded depth against the fingers' hold instead of lagging by the arm's compliance; the
            # copy depth must be known to the plan, because the place on the sheet is the copy pose slid back
            p.move((X_COPY - 0.006, yb, TOP + TCP_H), t=2.5, label="press: push in")
            # WHY a slow last 6 mm: on a single 3 s push the corrector overshot the target by 1.7 to 2.7 mm (fastrun trace,
            # GPU), and an overshoot is copied into the fingers for good, since they do not spring back
            p.move((X_COPY - CREEP, yb, TOP + TCP_H), t=3.0, label="press: push in slowly")
            # WHY the ease off is not contact and is followed by a still wait: the corrector's trim winds up while
            # the fingers resist (the command ran 47 mm ahead of the left hand, GPU trace); frozen through the draw
            # back, that stored load tipped the gauge 5 deg in the pads and swung the hands sideways, dragging
            # fingers off the copy (four GPU traces). Stepping the target back 4 mm and letting the corrector settle
            # there unloads the arms in place, every axis held, before the gauge moves
            p.hold(1.0, "press: hold")
            p.move((X_COPY - CREEP - 0.004, yb, TOP + TCP_H), t=1.0, label="press: ease off")
            p.hold(1.5, "press: unload the arms")
            p.move((X_COPY - CREEP - 0.004, yb, TOP + TCP_H + 0.003), t=0.8, label="press: ease up", contact=True)
            p.move((X_NEAR - 0.010, yb, TOP + TCP_H + 0.003), t=2.5, label="press: draw back", contact=True)
        p.move((p.pos[0], yb, TOP + TCP_H + LIFT), t=1.2, label="lift off the floor", contact=True)
    together(*plans)
    for p in plans:
        yb = p.pos[1] + off
        p.move((X_MARK, yb, TOP + TCP_H + LIFT), t=2.5, label="carry to the sheet")
        p.hold(0.8, "line up over the sheet")
        p.move((X_MARK, yb, TOP + SHEET_T + TCP_H), t=1.8, label="down on the sheet", contact=True)
        p.hold(0.5, "set on the sheet", contact=True)
        p.move(p.pos, t=0.8, finger=p.pre, label="let go", contact=True)
    together(*plans)
    for p in plans:
        p.move(p.pos + np.array([0.0, 0.0, 0.06]), t=1.2, label="clear: up")
        p.move((0.20, p.pos[1] + (0.08 if p.arm == "left" else -0.08), TOP + 0.16), label="away")
    together(*plans)
    return plans_out(*plans)


def demo():
    return plan(run_seed(), mode())


def trace_row(env):
    """fastrun --trace columns: the gauge's frame pose and tilt, and every finger's push (mm)."""
    asset = env.unwrapped.scene["gauge"]
    origin = _np(env.unwrapped.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    names = list(asset.joint_names)
    q = _np(asset.data.joint_pos)[0]
    R = _rot3(pose[3:7])
    row = {"g_x": pose[0] - origin[0], "g_y": pose[1] - origin[1], "g_z": pose[2] - origin[2],
           "g_yaw": math.degrees(math.atan2(R[1, 0], R[0, 0])), "g_tilt": math.degrees(math.acos(min(1.0, R[2, 2])))}
    for i in range(N_FING):
        row[f"s{i:02d}"] = -1000.0 * q[names.index(f"j{i:02d}")]
    return row
