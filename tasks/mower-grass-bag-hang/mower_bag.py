"""A 14 in corded electric push mower stands on the bench, its rear toward the robot. Its spring-loaded ejector flap
hangs closed over the two coupling hooks on its rear face. Its empty grass bag lies on the bench behind it, mouth
toward the mower. Fit the bag as the mower's manual says: raise the flap, set the bag's rod in both coupling hooks,
release the flap onto the bag, and take both hands away.
"""
import dataclasses
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, body, mechanism, on_place, on_reset, on_step, run_seed  # noqa: E402,F401
from yam_arena import station  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 1200           # 60 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# Every number: [SOURCE] quoted in the README, [ARITH] derived, [EST] estimate, [CHOICE] design choice, [RUN] measured
# by a tool this session. Coordinates: world x forward from the arm bases, y left, z up. The mower's frame M: origin
# on the bench under the middle of its rear face, x into the mower (world +x), z up from the bench.

# -- the mower (Sun Joe MJ401E) --------------------------------------------------------------------------------------------
X_M = 0.53                       # world x of the mower's rear face [CHOICE: both arms reach the hooks and the flap]
MOWER_KG = 13.2                  # [SOURCE: MJ401E manual, "29 lbs (13.2 kg)"]
WHEEL_R = 0.070                  # [SOURCE: same, wheel size "5.5 in. (14 cm)"]
DECK_L, DECK_W = 0.32, 0.36      # deck footprint ahead of the housing [EST: cut so the mower ends at the bench's
#                                  far edge, world x 0.91]
HOUSING = (0.0, 0.060, 0.130, 0.020, 0.400)   # rear housing: x from, x to, half width, z from, z to [EST]
HOOK_Y = 0.080                   # the two coupling hooks at y +-80 mm [EST]
HOOK_W = 0.016                   # each hook's width along y [EST]
FLOOR_Z = 0.285                  # top of each hook's floor plate above the bench [EST]
SLOT_X = (-0.021, 0.0)           # the open slot between the hook's lip and the rear face, M x [EST: 21 mm]
LIP_T, LIP_H = 0.004, 0.020      # each hook's lip: thickness along x, height over the floor [EST]
ROLL_FRICTION = 3.9              # N: the mower's rolling resistance on its slide [EST: 0.03 x 129 N; measured in the
#                                  real trial with a force gauge]
ROLL_DAMP = 10.0                 # N s/m: a little viscous damping on top [CHOICE: 1 N at 0.1 m/s]
FRONT_WHEEL_X = 0.30             # M x of the front axle (the wheels' bench contact) [EST]
BENCH_EDGE = 0.36 + 1.10 / 2     # world x of the bench's far edge (yam_arena station table) [ARITH: 0.91]
ROLL_FWD = BENCH_EDGE - (X_M + FRONT_WHEEL_X)   # the slide's forward stop: the front wheels at the bench edge [ARITH:
#                                  80 mm]
ROLL_BACK = 0.15                 # the slide's rear stop, toward the arms (m) [CHOICE: past any pull the arms can make]
ROLL_SAFE = ROLL_FWD - 0.020     # the score's limit: front wheels at least 20 mm inside the bench edge [CHOICE: a safety
#                                  margin, so the mower cannot roll off the bench; 60 mm]
# WHY a slide: the arms cannot slide 13.2 kg sideways (about 84 N at a rubber-on-bench friction of 0.65 [EST]) or lift
# it; they can roll it fore and aft on its wheels. The sim keeps only that, as a prismatic joint with friction (the
# wheels clear the bench).

# -- the ejector flap ------------------------------------------------------------------------------------------------------
HINGE = (-0.040, 0.355)          # hinge axis (M x, z), along y, behind the hooks [EST]
FLAP_L, FLAP_W, FLAP_T = 0.100, 0.240, 0.004   # [EST]
FLAP_LIP = 0.020                 # lip bent outward along the free edge [EST]
FLAP_KG = 0.15                   # [EST]
FLAP_MAX = math.radians(125.0)   # opens up to this [EST]
FLAP_K = 0.15                    # N m/rad torsion spring rate [EST]
FLAP_PRE = 1.33                  # rad of wind-up when closed: 0.20 N m closing torque at rest [ARITH: 0.15 x 1.33]
FLAP_DAMP = 0.10                 # N m s/rad [CHOICE: let go, the flap closes at about 4 rad/s (0.42 N m / 0.10),
#                                  its edge at 0.4 m/s; a bare hinge may close faster]

# -- the grass bag (MJ401E-10, 10.6 gal) -----------------------------------------------------------------------------------
# Bag frame B: origin at the support rod's centre, x toward the mower, z up when the bag sits level.
BAG_L, BAG_W, BAG_H = 0.500, 0.300, 0.267   # [ARITH: 40.05 l, the manual's "10.6 gal (40 l)" as a box; shape EST]
ROD_R, ROD_L = 0.005, 0.300      # steel support rod across the mouth [EST: 10 mm, 300 mm]
ROD_KG = 0.185                   # [ARITH: 7850 kg/m3 x pi x 5 mm^2 x 300 mm]
BAG_KG = 1.0                     # whole bag [EST]
FRONT = -0.032                   # the bag's front face, 32 mm behind the rod (the rod on two arms) [EST]
TOP_Z = 0.006                    # top of the hard cover in B [EST: 8 mm cover]
BOT_Z = TOP_Z - BAG_H            # bottom of the bag in B [ARITH]
BAR_R, BAR_L = 0.008, 0.150      # carry handle: a bar across the bag (along y) on two posts [EST]
BAR_Z = TOP_Z + 0.040            # bar centre over the cover [EST]
POST_Y = 0.070                   # posts at y +-70 mm [EST]

STEEL, FABRIC, PLASTIC, DARKP = (0.55, 0.56, 0.58), (0.12, 0.13, 0.12), (0.20, 0.21, 0.22), (0.20, 0.21, 0.22)
GREEN, BLACK, GREY = (0.16, 0.55, 0.20), (0.05, 0.05, 0.05), (0.35, 0.36, 0.37)
MATS = {"steel": (0.40, 0.35), "plastic": (0.45, 0.40), "fabric": (0.60, 0.55), "rubber": (0.80, 0.75)}  # [EST]


def _box(lo, hi, color, material, mass=0.0, collide=True):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Shape("box", tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, material=material,
                 mass=mass, collide=collide)


def _cyl(c, r, length, axis, color, material, mass=0.0, collide=True):
    return Shape("cylinder", tuple(float(v) for v in c), (r, length), color, axis, material, mass=mass,
                 collide=collide)


def hook_shapes(s):
    """One coupling hook at y = s * HOOK_Y: a floor plate standing out of the rear face and a lip at its back."""
    y0, y1 = s * HOOK_Y - HOOK_W / 2, s * HOOK_Y + HOOK_W / 2
    x_lip = SLOT_X[0] - LIP_T
    return [_box((x_lip, y0, FLOOR_Z - 0.004), (0.0, y1, FLOOR_Z), STEEL, "steel"),
            _box((x_lip, y0, FLOOR_Z), (SLOT_X[0], y1, FLOOR_Z + LIP_H), STEEL, "steel")]


def mower_shapes():
    hx0, hx1, hw, hz0, hz1 = HOUSING
    deck_x0 = hx1
    wz = WHEEL_R + 0.0005        # WHY: the wheels clear the bench by 0.5 mm; the slide carries the mower
    out = [_box((hx0, -hw, hz0), (hx1, hw, hz1), GREEN, "plastic"),                     # rear housing (discharge)
           _box((deck_x0, -DECK_W / 2, 0.030), (deck_x0 + DECK_L, DECK_W / 2, 0.200), GREEN, "plastic"),   # deck
           _box((deck_x0 + 0.06, -0.12, 0.200), (deck_x0 + 0.28, 0.12, 0.300), GREEN, "plastic"),          # motor
           _box((HINGE[0] - 0.005, 0.119, HINGE[1] - 0.010), (0.0, 0.131, HINGE[1] + 0.010), DARKP, "plastic"),
           _box((HINGE[0] - 0.005, -0.131, HINGE[1] - 0.010), (0.0, -0.119, HINGE[1] + 0.010), DARKP, "plastic"),
           _box((-0.0005, -0.10, 0.11), (0.0, 0.10, 0.27), BLACK, "plastic", collide=False)]   # opening, drawn
    for x in (0.10, FRONT_WHEEL_X):
        for s in (-1.0, 1.0):
            out.append(_cyl((x, s * 0.200, wz), WHEEL_R, 0.040, "Y", BLACK, "rubber"))
    for s in (-1.0, 1.0):        # the handle, folded forward over the deck (drawn only)
        out.append(_cyl((0.25, s * 0.16, 0.36), 0.0125, 0.40, "X", BLACK, "plastic", collide=False))
    return out + hook_shapes(1) + hook_shapes(-1)


def flap_shapes():
    """The flap at rest (closed, hanging), in M: a plate whose inner face is the hinge's x, and the lip at its foot
    bent outward (toward -x)."""
    hx, hz = HINGE
    return [_box((hx - FLAP_T, -FLAP_W / 2, hz - FLAP_L), (hx, FLAP_W / 2, hz), DARKP, "plastic"),
            _box((hx - FLAP_T - FLAP_LIP, -FLAP_W / 2, hz - FLAP_L), (hx - FLAP_T, FLAP_W / 2, hz - FLAP_L + FLAP_T),
                 DARKP, "plastic"),
            _cyl((hx - 0.002, 0.0, hz), 0.005, FLAP_W - 0.004, "Y", DARKP, "plastic")]


def _mower():
    base = Link("chock", 1.0, (_box((0.15, -0.03, 0.09), (0.21, 0.03, 0.15), GREEN, "plastic"),))
    # WHY a base inside the deck: a fixed-base articulation needs a root body; its box sits inside the deck, filtered
    # against the moving links, where nothing else reaches
    links = (Link("mower", MOWER_KG - FLAP_KG, tuple(mower_shapes())), Link("flap", FLAP_KG, tuple(flap_shapes())))
    joints = (Joint("roll", "prismatic", "mower", (0.0, 0.0, 0.0), "X", -ROLL_BACK, ROLL_FWD, damping=ROLL_DAMP,
                    friction=ROLL_FRICTION, dynamic_friction=ROLL_FRICTION),
              Joint("flap", "revolute", "flap", (HINGE[0], 0.0, HINGE[1]), "Y", 0.0, FLAP_MAX, stiffness=FLAP_K,
                    damping=FLAP_DAMP, target=-FLAP_PRE, parent="mower"))
    return mechanism("mower", (X_M, 0.0, TOP), base, links, joints, MATS, color=GREEN, self_collide=False)


def bag_shapes():
    """The bag in B. Masses [EST] sum to BAG_KG; the handle sits over the centre of mass (bag_handle_x)."""
    xb, xf = FRONT - BAG_L, FRONT
    shapes = [_cyl((0, 0, 0), ROD_R, ROD_L, "Y", STEEL, "steel", ROD_KG),
              _box((FRONT, 0.135, -0.005), (0.0, 0.145, 0.005), GREY, "steel", 0.010),
              _box((FRONT, -0.145, -0.005), (0.0, -0.135, 0.005), GREY, "steel", 0.010),
              _box((xb, -BAG_W / 2, TOP_Z - 0.008), (xf, BAG_W / 2, TOP_Z), PLASTIC, "plastic", 0.300),
              _box((xb, -BAG_W / 2, BOT_Z), (xf, BAG_W / 2, TOP_Z - 0.008), FABRIC, "fabric", 0.445)]
    xh = bag_handle_x(shapes)
    shapes += [_cyl((xh, 0, BAR_Z), BAR_R, BAR_L, "Y", GREY, "plastic", 0.030),
               _box((xh - 0.006, POST_Y - 0.005, TOP_Z), (xh + 0.006, POST_Y + 0.005, BAR_Z), DARKP, "plastic",
                    0.010),
               _box((xh - 0.006, -POST_Y - 0.005, TOP_Z), (xh + 0.006, -POST_Y + 0.005, BAR_Z), DARKP, "plastic",
                    0.010)]
    return shapes


def bag_handle_x(shapes):
    """x (in B) of the whole bag's centre of mass with a 50 g handle placed there [ARITH, a fixed point]."""
    m = sum(s.mass for s in shapes)
    mx = sum(s.mass * s.center[0] for s in shapes)
    return mx / m  # the handle at x adds 0.05 x, so the centre stays at mx/m (x = (mx + 0.05x)/(m + 0.05))


BAG_START = (X_M + 0.5 * sum(SLOT_X) - 0.085, 0.0)   # rod centre (world x, y) at the start: 85 mm short of the slots
#                                                       [CHOICE: the opening flap's lip clears the bag's front-top
#                                                       edge (tools/clearance.py)]
Z_LYING = TOP - BOT_Z + 0.001                         # rod centre height when the bag stands on the bench [ARITH]


def bag_layout(seed):
    """The rod centre's start (world x, y) and the bag's yaw for a seed: x 0 to 10 mm further back, y +-15 mm, yaw
    +-3 deg about the rod centre [CHOICE]."""
    if seed is None:
        return (BAG_START[0], BAG_START[1], 0.0)
    rng = np.random.default_rng(int(seed) % (2 ** 32))
    dx, dy, yaw = rng.uniform(-0.010, 0.0), rng.uniform(-0.015, 0.015), math.radians(rng.uniform(-3.0, 3.0))
    return (BAG_START[0] + dx, BAG_START[1] + dy, yaw)


MOWER = _mower()
BAG = body("bag", (BAG_START[0], BAG_START[1], Z_LYING), bag_shapes(), MATS, color=FABRIC)
OBJECTS = [MOWER, BAG]
HANDLE_X = BAG.shapes[5].center[0]   # the bar's x in B [ARITH]


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


def _ry(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


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


@on_place
def place_bag(env):
    """The bag's seeded start (before the reset settles)."""
    import torch
    base = env.unwrapped
    asset = base.scene["bag"]
    x, y, yaw = bag_layout(run_seed())
    dev = _torch_of(asset.data.root_link_pose_w).device
    o = _torch_of(base.scene.env_origins)[0]
    pose = torch.tensor([[x, y, Z_LYING, *_quat_yaw(yaw)]], dtype=torch.float32, device=dev)
    pose[:, :3] += o
    zero = torch.zeros((1, 6), dtype=torch.float32, device=dev)
    if hasattr(asset, "write_root_pose_to_sim_index"):  # Isaac Lab 3.0
        asset.write_root_pose_to_sim_index(root_pose=pose)
        asset.write_root_velocity_to_sim_index(root_velocity=zero)
    else:
        asset.write_root_state_to_sim(torch.cat([pose, zero], dim=1))
    asset.write_data_to_sim()


# -- the measurement the scorer reads (each is also a real-trial check, README) ---------------------------------------------
SPAN_MARGIN = 0.005              # the rod must run 5 mm past each side of each hook [CHOICE: read with a steel rule]
SLOT_SLACK = 0.002               # allowance on the slot walls: the colliders' 2 mm contact offset [CHOICE]
SEAT_BELOW, SEAT_ABOVE = 0.0005, 0.0015   # the rod's centre within -0.5 / +1.5 mm of resting on the floor [CHOICE:
#                                  a 1.5 mm feeler stack (1.0 + 0.5 leaves) must not pass under the rod]
FLAP_GAP = 0.005                 # the flap's free edge within 5 mm over the bag's cover [CHOICE: a 5 mm stack of
#                                  feeler leaves must not pass under it]
FLAP_DIG = -0.003                # an edge point "under" the cover by more than this is not a resting edge [CHOICE: the
#                                  colliders' contact offset and the cover's give]
PAD_CLEAR = 0.030                # every point of either gripper (fingers and housing) at least this far from the bag
#                                  (and, for 0.9, the flap) [CHOICE: read with a tape]
STILL = 0.005                    # m/s: the bag still [CHOICE]
BAG_SPIN = 0.05                  # rad/s: the bag not turning [CHOICE]
FLAP_STILL = 0.05                # rad/s: the flap still [CHOICE]
UPRIGHT = math.radians(30.0)     # the bag's cover within 30 deg of level, so an upside-down bag never counts [CHOICE]
MAX_TURN = math.radians(60.0)    # a rod turned more than 60 deg from the hooks' line fails both hooks [CHOICE]


def mower_frame(mower):
    """(R, p) of the mower link (world), from a MechanismState."""
    link = mower.links["mower"]
    return _rot3(link.quat_xyzw), np.asarray(link.pos, float)


def rod_in_mower(bag_pos, bag_quat, Rm, pm):
    """The rod's centre and unit axis in M."""
    Rb = _rot3(bag_quat)
    c = Rm.T @ (np.asarray(bag_pos, float) - pm)
    u = Rm.T @ Rb @ np.array([0.0, 1.0, 0.0])
    return c, u


def hook_check(c, u, s):
    """The finite rod against hook s (+1 left, -1 right), in M. Three stations across the hook (its two sides widened
    SPAN_MARGIN, and its middle): each must be ON the rod (|t| <= ROD_L / 2, never past an end) with the rod's centre
    inside the slot and resting on the floor. Returns (ok, details)."""
    yh = s * HOOK_Y
    out = {"ok": True, "t": [], "x": [], "z": []}
    if abs(u[1]) < math.cos(MAX_TURN):
        out["ok"] = False
        return out
    for ys in (yh - HOOK_W / 2 - SPAN_MARGIN, yh, yh + HOOK_W / 2 + SPAN_MARGIN):
        t = (ys - c[1]) / u[1]
        p = c + t * u
        out["t"].append(float(t))
        out["x"].append(float(p[0]))
        out["z"].append(float(p[2]))
        on_rod = abs(t) <= ROD_L / 2
        in_x = SLOT_X[0] + ROD_R - SLOT_SLACK <= p[0] <= SLOT_X[1] - ROD_R + SLOT_SLACK
        in_z = FLOOR_Z + ROD_R - SEAT_BELOW <= p[2] <= FLOOR_Z + ROD_R + SEAT_ABOVE
        out["ok"] = out["ok"] and on_rod and in_x and in_z
    return out


def flap_edge_points(theta):
    """Three points on the flap's free edge, inner (bag-side) face, in M, at flap angle theta (0 closed)."""
    hx, hz = HINGE
    R = _ry(theta)
    return [np.array([hx, 0.0, hz]) + R @ np.array([0.0, y, -FLAP_L]) for y in (-0.10, 0.0, 0.10)]


def flap_edge_gap(theta, bag_pos, bag_quat, Rm, pm):
    """(least gap, greatest gap, all three over the cover's footprint) of the flap's free edge over the bag's cover, in
    the bag's frame."""
    Rb, pb = _rot3(bag_quat), np.asarray(bag_pos, float)
    gaps, on = [], True
    for p in flap_edge_points(theta):
        q = Rb.T @ (Rm @ p + pm - pb)
        on = on and (FRONT - BAG_L <= q[0] <= FRONT) and abs(q[1]) <= BAG_W / 2
        gaps.append(float(q[2] - TOP_Z))
    return min(gaps), max(gaps), on


def _box_dist(pt, lo, hi):
    q = np.maximum(np.maximum(np.asarray(lo) - pt, pt - np.asarray(hi)), 0.0)
    return float(np.linalg.norm(q))


def dist_to_shapes(pt_world, R, p, shapes):
    """Distance from a world point to the nearest shape box (bounds) of a body posed (R, p)."""
    q = R.T @ (np.asarray(pt_world, float) - p)
    return min(_box_dist(q, *s.bounds()) for s in shapes if s.collide)


def points_to_shapes(pts_world, R, p, shapes):
    """Least distance from any of the world points (rows) to the shape boxes (bounds) of a body posed (R, p)."""
    q = (np.asarray(pts_world, float) - p) @ R          # rows: R^T (pt - p)
    best = np.inf
    for s in shapes:
        if s.collide:
            lo, hi = s.bounds()
            d = np.linalg.norm(np.maximum(np.maximum(lo - q, q - hi), 0.0), axis=1)
            best = min(best, float(d.min()))
    return best


BAG_SHAPES = BAG.shapes
FLAP_SHAPES = MOWER.links[1].shapes


def measure(objects, pads):
    """Everything the score reads, from the end state."""
    mower, bag = objects["mower"], objects["bag"]
    Rm, pm = mower_frame(mower)
    c, u = rod_in_mower(bag.pos, bag.quat_xyzw, Rm, pm)
    hooks = {s: hook_check(c, u, s) for s in (1, -1)}
    theta = float(mower.joints["flap"])
    gap, gap_hi, on = flap_edge_gap(theta, bag.pos, bag.quat_xyzw, Rm, pm)
    flap = mower.links["flap"]
    Rf, pf = _rot3(flap.quat_xyzw), np.asarray(flap.pos, float)
    Rb, pb = _rot3(bag.quat_xyzw), np.asarray(bag.pos, float)
    if pads is not None and len(pads):
        d_bag = points_to_shapes(pads, Rb, pb, BAG_SHAPES)
        d_flap = points_to_shapes(pads, Rf, pf, FLAP_SHAPES)
    else:
        d_bag = d_flap = 0.0  # WHY: unread hands count as touching, so no credit past the seat
    upright = float((Rm @ np.array([0.0, 0.0, 1.0])) @ (Rb @ np.array([0.0, 0.0, 1.0]))) >= math.cos(UPRIGHT)
    return {"hooks": hooks, "seated": all(h["ok"] for h in hooks.values()) and upright, "upright": upright,
            "theta": theta, "flap_gap": gap, "flap_gap_hi": gap_hi, "flap_over_bag": on, "d_bag": d_bag,
            "d_flap": d_flap, "roll": float(mower.joints["roll"]), "bag_speed": float(np.linalg.norm(bag.lin_vel)),
            "bag_spin": float(np.linalg.norm(bag.ang_vel)), "flap_spin": float(np.linalg.norm(flap.ang_vel))}


def rung(m):
    """0 to 4: 1 the rod seated in both hooks, the bag upright; 2 and the bag let go and still; 3 and the flap resting
    on the bag; 4 and both hands clear of the flap, the flap still, the mower's front wheels 20 mm or more inside the
    bench edge."""
    if not m["seated"]:
        return 0
    if not (m["d_bag"] >= PAD_CLEAR and m["bag_speed"] < STILL and m["bag_spin"] < BAG_SPIN):
        return 1
    if not (m["flap_over_bag"] and m["flap_gap"] >= FLAP_DIG and m["flap_gap_hi"] <= FLAP_GAP):
        return 2
    if not (m["d_flap"] >= PAD_CLEAR and m["flap_spin"] < FLAP_STILL and m["roll"] <= ROLL_SAFE):
        return 3
    return 4


# -- per-step memory (the hands are read from the sim; nothing else is carried but the 1 s hold) ---------------------------
TCP_OFFSET = (0.0, 0.0, 0.134)   # yam_arena.constants: the TCP along the gripper's approach axis
PAD_TIP = 0.147                  # pad tips along it: 13 mm past the TCP (yam_arena.constants.PAD_TIP_PAST_TCP)
PAD_TOP = 0.112                  # the pads' upper end [EST: 35 mm of pad face]
_M = {"pads": None, "tcps": None, "bag": None, "mower": None, "error": None}
_MEM = {"last": None, "now": None, "held": 0.0}
_CLOCK = {"dt": None}


def reset_memory():
    _M.update({"pads": None, "tcps": None, "bag": None, "mower": None, "error": None})
    _MEM.update({"last": None, "now": None, "held": 0.0})
    _CLOCK.update({"dt": None})


@on_reset
def _reset():
    reset_memory()


# The gripper as points (mm back from the pad tips along the approach axis): finger slices (from, to, thickness outward
# of the pad face, half width along the pad axis) and the housing beyond 83.5 mm, 51 x 33 mm half sizes [EST from the
# YAM gripper mesh].
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
    """Both hands' pad points after every control step; also the TCPs, the bag's pose and the mower's roll, which only
    the scripted demo's corrections read (never the score)."""
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
    try:  # WHY apart: a failed read here must not blank the pads the score reads
        base = env.unwrapped
        origin = _np(base.scene.env_origins)[0]
        pb = _np(base.scene["bag"].data.root_link_pose_w)[0]
        _M["bag"] = (pb[:3] - origin, pb[3:7])
        mw = base.scene["mower"]
        _M["mower"] = float(_np(mw.data.joint_pos)[0][list(mw.joint_names).index("roll")])
    except Exception as e:  # noqa: BLE001
        _M["bag"], _M["error"] = None, repr(e)


def update(objects):
    if objects is _MEM["last"]:
        return _MEM
    _MEM["last"] = objects
    m = measure(objects, _M["pads"])
    r = rung(m)
    done = r == 4
    _MEM["held"] = _MEM["held"] + (_CLOCK["dt"] or 0.0) if done else 0.0
    _MEM["now"] = {"m": m, "rung": r, "done": done}
    return _MEM


def success(objects):
    """Rod in both hooks, bag let go, flap resting on it, hands clear, mower on the bench (ends the run after 1 s)."""
    return update(objects)["now"]["done"]


LADDER = {0: 0.0, 1: 0.25, 2: 0.5, 3: 0.75}


def score(objects):
    """From the last step only: 1.0 when rung 4 has held 1 s; 0.9 rung 4 under 1 s; 0.75 flap on the bag; 0.5 bag
    let go in the hooks; 0.25 rod seated in both hooks; else 0."""
    n = update(objects)["now"]
    if n["done"]:
        return 1.0 if _MEM["held"] >= SUCCESS_HOLD_S - 1e-9 else 0.9
    return LADDER[n["rung"]]


def distance(objects):
    return 1.0 - score(objects)


# -- the demo -----------------------------------------------------------------------------------------------------------------
# The stock Waypoint carries only a yaw for a straight-down grip. Here each waypoint's yaw is a Rot: a float (the yaw)
# that also carries the full TCP rotation, and grip_rotation() (which yam_arena's ArmPlan calls for every waypoint)
# returns it. This changes only the scripted demo policy, never the scene or the score.
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
    if getattr(d.grip_rotation, "_mb_patched", False):
        return
    stock = d.grip_rotation

    def grip_rotation(yaw=0.0):
        R = getattr(yaw, "R", None)
        return np.array(R) if R is not None else stock(yaw)

    grip_rotation._mb_patched = True
    d.grip_rotation = grip_rotation


_patch_grip_rotation()


def tcp_frame(approach, closing):
    """A TCP rotation from its approach axis and its fingers' closing axis (made perpendicular)."""
    a = np.asarray(approach, float)
    a = a / np.linalg.norm(a)
    c = np.asarray(closing, float)
    c = c - a * (a @ c)
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def flap_rot(theta):
    """Left hand on the flap: approach along -y (into its left edge), fingers closing across the plate."""
    return Rot(tcp_frame((0.0, -1.0, 0.0), -(_ry(theta) @ np.array([1.0, 0.0, 0.0]))))


BAR_PITCH = math.radians(70.0)   # right hand's approach, from straight down toward +x [CHOICE: reach, RUN
#                                  tools/ikcheck.py]


def bar_rot(yaw=0.0):
    """Right hand on the bag's bar: approach forward and down, fingers closing in the x-z plane across the bar."""
    a = _ry(-BAR_PITCH) @ np.array([0.0, 0.0, -1.0])
    c = np.array([-a[2], 0.0, a[0]])
    return Rot(_rz(yaw) @ tcp_frame(a, c))


OPEN = -0.0465                   # fingers fully open [CHOICE: just inside FINGER_OPEN -0.04695]
PINCH_OPEN = -0.015              # left fingers before the pinch: 30 mm apart over the 4 mm plate [CHOICE: the front
#                                  finger stays 21 mm off the rear housing]
PINCH = 0.0                      # left grip: fully closed, 2 mm past the plate's faces [CHOICE]
BAR_OPEN = -0.024                # right fingers round the bar before the grip: 16 mm clear each side [CHOICE]
BAR_GRIP = -0.003                # 5 mm past the 16 mm bar [CHOICE]
BAR_LETGO = -0.016               # fingers opened off the bar: 8 mm clear of it each side [CHOICE: the lower finger
#                                  stays off the bag's cover, tools/clearance.py]
Y_BESIDE = 0.140                 # the left TCP's y beside the flap: tips 7 mm off its edge [CHOICE: the left wrist
#                                  (joint5) is at its 90 deg stop from here outward, RUN tools/ikcheck.py]
PINCH_R = 0.050                  # left pinch point: 50 mm from the hinge along the flap [CHOICE: the fingers stay
#                                  clear of the bag's cover when the flap is let go]
PINCH_Y = 0.110                  # its y: 10 mm inside the flap's left edge [CHOICE: tips 13 mm past, 9 mm clear of the
#                                  hook]
FLAP_UP = math.radians(110.0)    # the flap held this far open [CHOICE: joint6 -110 of its -120 deg limit]
LET_FLAP = math.radians(30.0)    # the flap is lowered to this over its resting angle, then let go [CHOICE: the left
#                                  fingers stay clear of the bag's cover, tools/clearance.py]
STEADY = 0.8                     # s over a part before a touch-down [CHOICE]
SEAT_PUSH = 0.020                # the bar is lowered this far past the seat so the hooks take the bag [CHOICE: more
#                                  than the arm's sag under the 1 kg bag, about 20 mm, EST: TCP stiffness about 0.5 N/mm]
CARRY_Z = Z_LYING - TOP + 0.010  # rod centre while sliding the bag forward low: 10 mm over its lying height [CHOICE]
LIFT_Z = 0.322                   # rod centre over the hooks: its underside 12 mm over the lips [CHOICE: the demo's
#                                  margin, not a requirement]
SHORT = 0.055                    # the rod stops this far behind the slot before lifting [CHOICE: 34 mm off the lip]
ONE_HOOK_DY = -0.065             # s_one_hook: the bag 65 mm right, so the rod's left end stops 3 mm into the left
#                                  hook (the rod still crosses its inner side; the 5 mm margin past its far side is
#                                  missing)
NUDGE, SHOVE = 0.008, 0.070      # s_nudge / s_shove: how far the mower is rolled, by the bag, after the fit [CHOICE:
#                                  the shove passes the 60 mm limit and stays short of the 80 mm stop]
PUSH_LAG = 0.002                 # m the push stops short of its target against the mower's damping [EST]
VARIANT = "demo"                 # sim/mk.sh sets this in stem copies: see _plan()
SERVO_LABELS = ("line up behind the hooks", "line up over the slots", "line up again")
PUSH_LABEL = "line up the push"
UNLOAD_LABEL = "rest on the hooks"
FOLLOW_LABEL = "follow the roll"


def slot_x_world(roll=0.0):
    return X_M + roll + 0.5 * sum(SLOT_X)


def pinch_point(theta, roll=0.0):
    """World TCP of the left pinch at flap angle theta (the plate's middle, PINCH_R from the hinge), the mower rolled."""
    hx, hz = HINGE
    p = np.array([hx, 0.0, hz]) + _ry(theta) @ np.array([-FLAP_T / 2, 0.0, -PINCH_R])
    return np.array([X_M + roll + p[0], PINCH_Y, TOP + p[2]])


def _w(t, p, f, label, rot):
    return Waypoint(float(t), (float(p[0]), float(p[1]), float(p[2])), float(f), label, rot)


def flap_arc(t0, t1, total, finger, label, roll=0.0):
    """Waypoints that swing the pinched flap from angle t0 to t1 in 'total' s, in chords of at most 10 deg."""
    n = max(1, int(math.ceil(abs(t1 - t0) / math.radians(10.0) - 1e-9)))
    out = []
    for k in range(n):
        th = t0 + (t1 - t0) * (k + 1) / n
        out.append(_w(total / n, pinch_point(th, roll), finger, label, flap_rot(th)))
    return out


def hung_pose(roll=0.0):
    """The bag hung in both hooks (rod centre on the floor, against the lip), turned back-down about the rod until its
    tail meets the bench or its front-bottom edge meets the rear face, whichever comes first [ARITH: the rest the
    scorer expects]. Returns (world pos, quat xyzw, tilt rad, which contact)."""
    rod = np.array([SLOT_X[0] + ROD_R, 0.0, FLOOR_Z + ROD_R])

    def turned(p, tilt):         # about y by -tilt: the bag's tail (B x < 0) goes down
        x, z = p
        return x * math.cos(tilt) - z * math.sin(tilt), x * math.sin(tilt) + z * math.cos(tilt)

    def first(cond):
        lo, hi = 0.0, 0.6
        if not cond(hi):
            return hi
        for _ in range(60):
            mid = (lo + hi) / 2
            lo, hi = (lo, mid) if cond(mid) else (mid, hi)
        return hi

    face = first(lambda tilt: rod[0] + turned((FRONT, BOT_Z), tilt)[0] >= 0.0)
    bench = first(lambda tilt: rod[2] + turned((FRONT - BAG_L, BOT_Z), tilt)[1] <= 0.0)
    tilt, which = (face, "face") if face < bench else (bench, "bench")
    q = (0.0, math.sin(-tilt / 2), 0.0, math.cos(-tilt / 2))
    return np.array([X_M + roll, 0.0, TOP]) + rod, np.array(q), tilt, which


def flap_rest_angle(bag_pos, bag_quat, roll=0.0):
    """The flap angle where its free edge, closing from FLAP_MAX, first meets the bag's cover (0.05 deg scan), or None
    if it closes past the cover."""
    Rm, pm = np.eye(3), np.array([X_M + roll, 0.0, TOP])
    th = FLAP_MAX
    while th > 0.0:
        g, _, on = flap_edge_gap(th, bag_pos, bag_quat, Rm, pm)
        if on and g <= 0.0:
            return th
        th -= math.radians(0.05)
    return None


REST = flap_rest_angle(*hung_pose()[:2])   # the flap's resting angle on the hung bag [ARITH]


def left_plan(t_hold, variant, push=None):
    """The flap hand: pinch, swing up, hold for t_hold s, lower, let go, draw out along +y. push: (s after the hold
    starts, roll) when the right hand rolls the mower during the hold; the pinch then moves with the flap."""
    p0 = pinch_point(0.0)
    out = np.array([p0[0], Y_BESIDE, p0[2]])
    W = [_w(3.0, out, PINCH_OPEN, "beside the flap", flap_rot(0.0)),
         _w(STEADY, out, PINCH_OPEN, "steady", flap_rot(0.0)),
         _w(1.5, p0, PINCH_OPEN, "in round the plate", flap_rot(0.0)),
         _w(0.8, p0, PINCH, "pinch the flap", flap_rot(0.0)),
         *flap_arc(0.0, FLAP_UP, 4.0, PINCH, "raise the flap")]
    roll = 0.0
    if push is None:
        W.append(_w(t_hold, pinch_point(FLAP_UP), PINCH, "hold the flap up", flap_rot(FLAP_UP)))
    else:   # WHY: the flap rolls with the mower; a pinch held at fixed world points would drag it or slip
        t_push, roll = push
        W += [_w(t_push, pinch_point(FLAP_UP), PINCH, "hold the flap up", flap_rot(FLAP_UP)),
              _w(PUSH_T, pinch_point(FLAP_UP, roll), PINCH, "move with the mower", flap_rot(FLAP_UP)),
              _w(0.6, pinch_point(FLAP_UP, roll), PINCH, FOLLOW_LABEL, flap_rot(FLAP_UP)),
              _w(max(0.5, t_hold - t_push - PUSH_T - 0.6), pinch_point(FLAP_UP, roll), PINCH, "hold the flap up",
                 flap_rot(FLAP_UP))]
    if variant == "prop":                                   # the flap is still held up at the end
        return W + [_w(8.0, pinch_point(FLAP_UP, roll), PINCH, "keep holding", flap_rot(FLAP_UP))]
    # WHY tips let go fully open: the held bag sits 20 mm higher than hung, under the hand's usual let-go point
    theta_let = {"slam": FLAP_UP, "pinch": PINCH_LET, "tips": FLAP_UP}.get(variant, REST + LET_FLAP)
    if theta_let < FLAP_UP:
        W += flap_arc(FLAP_UP, theta_let, 2.5, PINCH, "lower the flap", roll)
    pl = pinch_point(theta_let, roll)
    pout = np.array([pl[0], Y_BESIDE, pl[2]])
    return W + [_w(0.6, pl, PINCH_OPEN, "let go of the flap", flap_rot(theta_let)),
                _w(1.5, pout, PINCH_OPEN, "draw out sideways", flap_rot(theta_let)),
                _w(1.5, pout + np.array([0.0, 0.0, 0.06]), PINCH_OPEN, "back off", flap_rot(theta_let))]


def bar_tcp(rod_xy, yaw, rod_z):
    """The right TCP on the bar for a rod centre at world (x, y), rod_z over the bench, the bag turned yaw."""
    return np.array([rod_xy[0], rod_xy[1], TOP + rod_z]) + _rz(yaw) @ np.array([HANDLE_X, 0.0, BAR_Z])


PUSH_T = 2.0                     # s: the nudge or shove [CHOICE]
PINCH_LET = FLAP_UP              # s_pinch: the flap let go fully open, onto the parked right gripper [CHOICE]
TAIL_EXTRA = math.radians(1.0)   # the tail is set down this far past the hung tilt, onto the bench [CHOICE]
UNDER = math.radians(85.0)       # s_pinch: the right gripper parks where the flap passes this angle [CHOICE: the whole
#                                  gripper 38 mm over the bag (so the score is 0.5, not 0.25) and 6.6 mm under the flap
#                                  while it is held at 110 deg]
UNDER_R = 0.095                  # ... this far out from the hinge [CHOICE]
PARK_PITCH = math.radians(40.0)  # ... its approach this far from straight down [CHOICE: the housing stays off the bag]
PARK_Y = -0.090                  # ... at this y [CHOICE]


def right_plan(start, variant, wait):
    """The bag hand: from behind, round the bar, grip, lift, wait for the flap, slide forward low, lift, carry over the
    slots, lower into the hooks, let go, slide off the bar and draw back. start: the rod's (x, y, yaw) (bag_layout)."""
    x0, y0, yaw0 = start
    R0, R1 = bar_rot(yaw0), bar_rot(0.0)
    a = R1.R[:, 2]
    g0 = bar_tcp((x0, y0), yaw0, Z_LYING - TOP)
    back = g0 - 0.04 * R0.R[:, 2]
    sx = slot_x_world()
    W = [_w(1.5, back + np.array([0.0, -0.10, 0.02]), BAR_OPEN, "up beside the bag", R0),
         _w(1.5, back, BAR_OPEN, "behind the handle", R0),
         _w(STEADY, back, BAR_OPEN, "steady", R0),
         _w(1.5, g0, BAR_OPEN, "round the bar", R0),
         _w(0.8, g0, BAR_GRIP, "grip the bar", R0),
         _w(1.2, bar_tcp((x0, y0), yaw0, CARRY_Z), BAR_GRIP, "lift off the bench", R0),
         _w(wait, bar_tcp((x0, y0), yaw0, CARRY_Z), BAR_GRIP, "wait for the flap", R0)]
    seat_z = FLOOR_Z + ROD_R - SEAT_PUSH
    if variant == "lift":                                   # one hand: tip the bag, lift the flap's lip with it
        return W + lift_route(seat_z)
    if variant == "one_arm":                                # carried straight in, the flap closed
        g = bar_tcp((sx, 0.0), 0.0, LIFT_Z)
        return W + [_w(3.0, bar_tcp((sx - SHORT, 0.0), 0.0, CARRY_Z), BAR_GRIP, "slide in, square", R1),
                    _w(1.5, bar_tcp((sx - SHORT, 0.0), 0.0, LIFT_Z), BAR_GRIP, "lift to the hooks", R1),
                    _w(3.0, g, BAR_GRIP, "push in to the slots", R1),
                    _w(2.0, bar_tcp((sx, 0.0), 0.0, seat_z), BAR_GRIP, "lower", R1),
                    _w(0.8, bar_tcp((sx, 0.0), 0.0, seat_z), BAR_OPEN, "let go", R1),
                    _w(1.5, bar_tcp((sx, 0.0), 0.0, seat_z) - 0.08 * a, BAR_OPEN, "draw back", R1)]
    dy = ONE_HOOK_DY if variant == "one_hook" else 0.0
    dx = TIPS_DX if variant == "tips" else 0.0
    if variant == "tips":                                   # the rod set on the lips' tops and held there
        seat_z = FLOOR_Z + LIP_H + ROD_R
    tgt = (sx + dx, dy)
    W += [_w(3.0, bar_tcp((sx - SHORT, dy), 0.0, CARRY_Z), BAR_GRIP, "slide in, square", R1),
          _w(1.5, bar_tcp((sx - SHORT, dy), 0.0, LIFT_Z), BAR_GRIP, "lift over the lips", R1),
          _w(1.0, bar_tcp((sx - SHORT, dy), 0.0, LIFT_Z), BAR_GRIP, SERVO_LABELS[0], R1)]
    if variant == "ground":                                 # let go over the bench, short of the hooks
        drop = bar_tcp((sx - SHORT, dy), 0.0, LIFT_Z)
        return W + [_w(0.8, drop, BAR_OPEN, "let go short of the hooks", R1),
                    _w(1.5, drop - 0.03 * a + np.array([0.0, 0.0, 0.03]), BAR_OPEN, "draw back", R1)]
    W += [_w(2.0, bar_tcp(tgt, 0.0, LIFT_Z), BAR_GRIP, "carry over the slots", R1),
          _w(STEADY, bar_tcp(tgt, 0.0, LIFT_Z), BAR_GRIP, SERVO_LABELS[1], R1),
          _w(0.6, bar_tcp(tgt, 0.0, LIFT_Z), BAR_GRIP, SERVO_LABELS[2], R1),
          _w(2.0, bar_tcp(tgt, 0.0, seat_z), BAR_GRIP, "lower into the hooks", R1),
          _w(0.8, bar_tcp(tgt, 0.0, seat_z), BAR_GRIP, "settle in the hooks", R1)]
    if variant in ("hold", "tips"):                         # the bag is still held at the end
        return W + [_w(14.0, bar_tcp(tgt, 0.0, seat_z), BAR_GRIP, "keep holding", R1)]
    g_let = bar_tcp(tgt, 0.0, seat_z)
    if variant in ("nudge", "shove"):                       # roll the mower by the held bag, then let go
        # WHY while holding: success ends the run 1 s after the fit is complete, so a later push would never happen.
        # The rod first closes its slack to the rear face (the line-up pass measures it), then rolls the mower.
        g_push = g_let + np.array([{"nudge": NUDGE, "shove": SHOVE}[variant] + PUSH_LAG, 0.0, 0.0])
        W += [_w(0.5, g_let, BAR_GRIP, PUSH_LABEL, R1),
              _w(PUSH_T, g_push, BAR_GRIP, "roll the mower", R1),
              _w(0.5, g_push, BAR_GRIP, "settle after the push", R1)]
        g_let = g_push
    # Set the bag's tail down on the bench while still holding it, turning it about the rod as it will hang, so that
    # opening the fingers moves nothing (WHY: let go level, the bar would drop 12 mm onto the lower finger).
    Rt = _ry(-(hung_pose()[2] + TAIL_EXTRA))
    rod = g_let - np.array([HANDLE_X, 0.0, BAR_Z])
    g_let = rod + Rt @ np.array([HANDLE_X, 0.0, BAR_Z])
    Rtail = Rot(Rt @ R1.R)
    at = Rtail.R[:, 2]
    W += [_w(1.5, g_let, BAR_GRIP, "set the tail down", Rtail),
          _w(0.6, g_let, BAR_GRIP, UNLOAD_LABEL, Rtail),
          _w(0.8, g_let, BAR_LETGO, "open round the bar", Rtail),
          _w(1.2, g_let - 0.06 * at, BAR_LETGO, "slide off the bar", Rtail),
          _w(1.5, g_let - 0.06 * at + np.array([-0.05, -0.5 * dy, 0.04]), BAR_LETGO, "draw back", Rtail)]
    if variant == "pinch":                                  # the hand parks under the flap's path and stays
        hx, hz = HINGE
        under = np.array([X_M + hx - UNDER_R * math.sin(UNDER), PARK_Y, TOP + hz - UNDER_R * math.cos(UNDER)])
        a = _ry(-PARK_PITCH) @ np.array([0.0, 0.0, -1.0])
        Rp = Rot(tcp_frame(a, np.array([-a[2], 0.0, a[0]])))
        W += [_w(2.0, under, BAR_GRIP, "under the flap's path", Rp),
              _w(12.0, under, BAR_GRIP, "stay there", Rp)]
    return W


LIFT_TIP = math.radians(20.0)    # s_lift: the bag tipped nose-down this far [ARITH: its front-bottom corner then sits
#                                  234 mm under the rod, so the rod can pass 19 mm under the flap's lip]
LIFT_LOW = 0.236                 # ... rod centre height (M) while it goes in under the lip [ARITH: the corner 2 mm off
#                                  the bench]
LIFT_OUT = -0.080                # ... rod centre (M x) before it goes in [CHOICE: 11 mm outside the lip; the arm
#                                  reaches the tipped bar here, not further out (RUN tools/ikcheck.py)]
LIFT_IN = -0.031                 # ... rod centre (M x) between the flap and the hooks' lips [ARITH: 4 mm off the plate,
#                                  1 mm off the lips; the cover's front corner then sits under the flap's lip]
LIFT_LABEL = "line up under the lip"


def tipped(rod_m, tilt):
    """(TCP, Rot) on the bar for a rod centre at M (x, z), y 0, the bag turned tilt about the rod (tail down > 0)."""
    Rt = _ry(-tilt)
    rod = np.array([X_M + rod_m[0], 0.0, TOP + rod_m[1]])
    return rod + Rt @ np.array([HANDLE_X, 0.0, BAR_Z]), Rot(Rt @ bar_rot(0.0).R)


def lift_route(seat_z):
    """s_lift: the right hand alone tips the bag nose-down, takes the rod in under the closed flap's lip, rises between
    the flap and the hooks so the bag's cover lifts the lip, levels the bag, and sets the rod in the hooks. Whether it
    works is what the stem's sim run shows. [CHOICE: geometry from a 2D sweep of this path, no PhysX]"""
    tilt_hung = hung_pose()[2]
    sx = 0.5 * sum(SLOT_X)
    lie = CARRY_Z
    steps = [(1.5, (LIFT_OUT, lie), 0.0, "square behind the flap"),
             (1.5, (LIFT_OUT, 0.5 * (lie + LIFT_LOW)), -0.5 * LIFT_TIP, "tip nose-down"),
             (1.5, (LIFT_OUT, LIFT_LOW), -LIFT_TIP, "down to the lip's height"),
             (0.8, (LIFT_OUT, LIFT_LOW), -LIFT_TIP, LIFT_LABEL),
             (3.0, (LIFT_IN, LIFT_LOW), -LIFT_TIP, "in under the lip"),
             (4.0, (LIFT_IN, LIFT_Z), tilt_hung, "rise, lifting the lip"),
             (2.0, (sx, LIFT_Z), tilt_hung, "carry over the slots"),
             (2.0, (sx, seat_z), tilt_hung, "lower into the hooks"),
             (0.8, (sx, seat_z), tilt_hung, "settle in the hooks"),
             (1.0, (sx, seat_z), tilt_hung + TAIL_EXTRA, "set the tail down")]
    W = []
    for dur, rod_m, tilt, label in steps:
        g, R = tipped(rod_m, tilt)
        W.append(_w(dur, g, BAR_GRIP, label, R))
    g, R = tipped((sx, seat_z), tilt_hung + TAIL_EXTRA)
    at = R.R[:, 2]
    return W + [_w(0.6, g, BAR_GRIP, UNLOAD_LABEL, R),
                _w(0.8, g, BAR_LETGO, "open round the bar", R),
                _w(1.2, g - 0.06 * at, BAR_LETGO, "slide off the bar", R),
                _w(1.5, g - 0.06 * at + np.array([-0.05, 0.0, 0.04]), BAR_LETGO, "draw back", R)]


TIPS_DX = SLOT_X[0] - LIP_T / 2 - 0.5 * sum(SLOT_X)   # s_tips: the rod centre over the lips' middle [ARITH]
T_UP = 3.0 + STEADY + 1.5 + 0.8 + 4.0                 # s until the flap is fully up [ARITH: left_plan]
T_GRIPPED = 3.0 + STEADY + 1.5 + 0.8 + 1.2            # s until the bag is gripped and lifted [ARITH: right_plan]
HOLD_TAIL = ("keep holding", "stay there")
# Waypoints in free air, away from any part: the IK check allows these 3 mm and 2 deg (tools/ikcheck.py), every other one
# 0.5 mm and 0.5 deg. WHY: the left wrist sits on its stop beside the flap; the nearest part is 7 mm or more away.
FREE_AIR = ("up beside the bag", "beside the flap", "steady", "in round the plate", "draw out sideways", "back off",
            "behind the handle", "draw back", "slide off the bar")


def _plan(seed, v):
    if v == "nothing":
        return {}
    start = bag_layout(seed)
    right = right_plan(start, v, wait=0.0 if v in ("one_arm", "lift") else max(0.5, T_UP - T_GRIPPED + 0.5))
    if v in ("one_arm", "lift"):
        return {"right": right}
    t_right = sum(w.duration for w in right if w.label not in HOLD_TAIL)
    push = None
    if v in ("nudge", "shove"):   # the push starts this long after the left hand's hold begins
        t0 = 0.0
        for w in right:
            if w.label == "roll the mower":
                break
            t0 += w.duration
        push = (t0 - T_UP, {"nudge": NUDGE, "shove": SHOVE}[v])
    return {"left": left_plan(max(1.0, t_right - T_UP + 0.5), v, push), "right": right}


def plan(seed=None, variant=None):
    v = VARIANT if variant is None else variant
    p = _plan(seed, v)
    _SERVO.update({"on": "right" in p, "done": set(), "log": [], "events": servo_events(p.get("right", [])),
                   "left_events": servo_events(p.get("left", []), (FOLLOW_LABEL,)), "left_done": set(),
                   "roll": {"nudge": NUDGE, "shove": SHOVE}.get(v, 0.0),
                   "dy": ONE_HOOK_DY if v == "one_hook" else 0.0, "dx": TIPS_DX if v == "tips" else 0.0})
    return p


def demo():
    """Left hand raises the flap and holds it; right hand hangs the bag on both hooks and lets go; left hand lowers the
    flap onto the bag and lets go; both draw back."""
    return plan(run_seed())


# -- the demo's corrections: a privileged line-up of the rod from the bag's true pose (the score never reads it) -------------
# WHY: the arm is compliant and the 1 kg bag hangs from a pinch, so the rod sits millimetres off where the plan puts it
# (the slot leaves 5.5 mm a side); a person looks at the hooks, the script reads the pose instead.
_SERVO = {"on": False, "done": set(), "log": [], "events": [], "left_events": [], "left_done": set(), "roll": 0.0,
          "dy": 0.0, "dx": 0.0}
SERVO_MAX = (0.025, math.radians(6.0))   # the most one pass moves the hand (m, rad) [CHOICE: over the arm's ~20 mm
#                                           sag under the bag]
SERVO_MAX_Z1 = 0.040             # the first pass may lift the hand this far [CHOICE: the sag under the bag may reach
#                                  about 36 mm, EST]


def servo_events(W, labels=(*SERVO_LABELS, PUSH_LABEL, UNLOAD_LABEL, LIFT_LABEL)):
    out, t = [], 0.0
    for w in W:
        t += w.duration
        if w.label in labels:
            out.append((t - 0.1, w.label))
    return out


def shift_after(waypoints, t, d, dyaw, pivot):
    """Waypoints after the one playing at plan time t, turned dyaw about the world point pivot (vertical axis) and then
    moved by d. WHY only later ones: the playing segment's goal stays put, so the hand never jumps."""
    end, i = 0.0, len(waypoints)
    for k, w in enumerate(waypoints):
        end += w.duration
        if t < end:
            i = k
            break
    out = list(waypoints[:i + 1])
    for w in waypoints[i + 1:]:
        p = np.asarray(w.pos, float)
        p = np.asarray(pivot) + _rz(dyaw) @ (p - np.asarray(pivot)) + np.asarray(d)
        yaw = w.yaw.turned(dyaw) if isinstance(w.yaw, Rot) else w.yaw + dyaw
        out.append(dataclasses.replace(w, pos=(float(p[0]), float(p[1]), float(p[2])), yaw=yaw))
    return out


def rod_error(bag_pos, bag_quat, label, roll):
    """(dx, dy, dz, dyaw) from the rod's true pose to where this line-up wants it (world)."""
    Rb = _rot3(bag_quat)
    u = Rb @ np.array([0.0, 1.0, 0.0])
    yaw_rod = math.atan2(-u[0], u[1])
    sx = slot_x_world(roll) + _SERVO["dx"]
    if label == LIFT_LABEL:
        return np.array([X_M + roll + LIFT_OUT, 0.0, TOP + LIFT_LOW]) - np.asarray(bag_pos, float), -yaw_rod
    want_x = sx - SHORT if label == SERVO_LABELS[0] else sx
    want = np.array([want_x, _SERVO["dy"], TOP + LIFT_Z])
    return want - np.asarray(bag_pos, float), -yaw_rod


def follow_roll(plans, t):
    """The left hand's later waypoints moved by the mower's measured roll less the roll its plan already assumed."""
    if "left" not in plans or _M.get("mower") is None:
        return
    for k, (tk, label) in enumerate(_SERVO["left_events"]):
        if k in _SERVO["left_done"] or t < tk:
            continue
        _SERVO["left_done"].add(k)
        d = np.array([float(_M["mower"]) - _SERVO["roll"], 0.0, 0.0])
        d = np.clip(d, -SERVO_MAX[0], SERVO_MAX[0])
        plans["left"].waypoints = shift_after(plans["left"].waypoints, t, d, 0.0, np.zeros(3))
        rec = {"pass": f"left {k}", "label": label, "t": round(t, 2), "d_mm": [round(1000 * v, 2) for v in d]}
        _SERVO["log"].append(rec)
        print("[mower bag line-up] " + str(rec), flush=True)


def servo(plans, t):
    if _SERVO["on"]:
        follow_roll(plans, t)
    if not _SERVO["on"] or _M.get("bag") is None or _M.get("tcps") is None or "right" not in plans:
        return
    for k, (tk, label) in enumerate(_SERVO["events"]):
        if k in _SERVO["done"] or t < tk:
            continue
        _SERVO["done"].add(k)
        pb, qb = _M["bag"]
        roll = _M.get("mower") or 0.0
        tcp = np.asarray(_M["tcps"]["right"], float)
        dyaw = 0.0
        if label == UNLOAD_LABEL:      # the hooks hold the bag: aim the hand where it now is, so opening moves nothing
            d = tcp - np.asarray(plans["right"].target(t)[0], float)
        elif label == PUSH_LABEL:      # close the rod's slack to the rear face first (then the push rolls the mower)
            d = np.array([X_M + roll - ROD_R - float(pb[0]), 0.0, 0.0])
        else:
            d, dyaw = rod_error(pb, qb, label, roll)
            dyaw = float(np.clip(dyaw, -SERVO_MAX[1], SERVO_MAX[1]))
            # the rod turns about the hand: where the rod centre lands after the turn, then the shift that remains
            pb_turned = tcp + _rz(dyaw) @ (np.asarray(pb) - tcp)
            d = (np.asarray(pb) + d) - pb_turned
        cap = np.array([SERVO_MAX[0], SERVO_MAX[0], SERVO_MAX_Z1 if label == SERVO_LABELS[0] else SERVO_MAX[0]])
        d = np.clip(d, -cap, cap)
        plans["right"].waypoints = shift_after(plans["right"].waypoints, t, d, dyaw, tcp)
        rec = {"pass": k, "label": label, "t": round(t, 2), "d_mm": [round(1000 * v, 2) for v in d],
               "dyaw_deg": round(math.degrees(dyaw), 3)}
        _SERVO["log"].append(rec)
        print("[mower bag line-up] " + str(rec), flush=True)


def _replan_demo_after_reset():
    """WHY: inspect-robots resets the policy before the embodiment, so yam_scripted would call demo() before the reset
    has set this trial's seed. This replans on the first action, after the reset, and runs the demo's line-up passes
    before each action. It changes only the scripted demo policy, never the scene or the score."""
    try:
        from yam_arena import scripted
    except ImportError:
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_mb_replan", False):
        return
    reset, act = cls.reset, cls.act
    unset = object()

    def patched_reset(self, scene):
        self._mb_scene, self._mb_seed = scene, unset
        reset(self, scene)

    def patched_act(self, observation):
        if getattr(self, "_mb_seed", None) is unset:
            self._mb_seed = run_seed()
            reset(self, self._mb_scene)
        servo(self._plans, float(observation.state_time) + self._dt)
        return act(self, observation)

    cls.reset, cls.act, cls._mb_replan = patched_reset, patched_act, True


_replan_demo_after_reset()
