"""A right-hand fingerstick trainer lies palm up, rolled so the sides of its fingertips face up, on two foam rests: one
under the base of its fingers and one under its wrist, so a firm press on a fingertip tips the hand and lifts its wrist
off the rest. A BD Microtainer contact-activated lancet (21 G x 1.8 mm) stands tip down in a stand whose socket holds
its protective tab, and a BD sharps container stands beside. Do a fingerstick as BD directs: twist the lancet's tab off
in the stand before lifting the lancet out; keep the trainer's wrist down on its rest throughout the press; place the
lancet's tip square on the side of the ring or middle fingertip's pad (never the little finger) and press it firmly,
without sliding, until it fires; then drop the used lancet into the sharps container through its lid opening and let go
of it.
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, hull, mechanism, on_place, on_reset, on_step, run_seed  # noqa: E402
from yam_arena import station  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
RUN_LIMIT_STEPS = 2400           # 120 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the lancet (BD Microtainer contact-activated lancet 366577, 21 G x 1.8 mm: a loose tool of two links) -------------
# Lancet frame: origin at the shield's face (shield out), z up the lancet's axis toward its rear.
L_W, L_T = 0.013, 0.010          # the housing's width and thickness [EST: BD publishes no body size; measured on the real
#                                  lancet with calipers before the trials, README Real trials]
L_LEN = 0.037                    # the housing's length [EST, as above]
SHIELD_R = 0.0035                # the contact shield's radius [EST]
SHIELD_OUT = 0.003               # the shield's face stands this far past the housing's front [EST]
SHIELD_TRAVEL = 0.0022           # the shield's travel into the housing to its stop [EST]
K_SHIELD = 4000.0                # N/m: the shield's spring (the drive spring's load path, lumped) [CHOICE]
F_PRE = 1.0                      # N: the shield's preload against its stop [CHOICE]
F_FIRE = 8.0                     # N on the shield fires it [EST: inside the 4 - 15 N activation force of the Owen Mumford
#                                  Unistik Touch contact-activated lancet, FDA 510(k) K222111; BD publishes no value]
F_FIRE_MAX_REAL = 15.0           # N: the real BD lancet is used only if it fires at or under this (README Real trials)
F_TOUCH = 2.0                    # N on the shield: the lancet is on the skin
F_FIRM = 5.0                     # N: "press firmly" for the placed rung, over the hand's 3.5 to 4.4 N tipping force, so
#                                  only a steadied hand reaches it [CHOICE]
LANCET_KG = (0.0020, 0.0020)     # housing, shield [EST: 4 g in all]. WHY 2 g on the shield: at 0.5 g (600x lighter than
#                                  the hand) PhysX let it sink 2.9 mm into the fingertip at 3.8 N, so it never fired [RUN]

# -- the stand (its socket holds the tab; the tab's frangible post is a sprung turn) -----------------------------------
STAND = (0.270, -0.160)          # the stand's centre (world x, y)
STAND_H = 0.025
STAND_HALF = 0.020
CUP_D = 0.008                    # the socket's depth above its floor (the tab)
CUP_CLEAR = 0.0004               # each side between the socket and the housing
CUP_WALL = 0.0025
TAB_H = 0.002                    # the tab cap's height in the socket's floor
T_TAB = 0.030                    # N m: the tab's break torque [EST: under the "< 4.9 N-cm" cap twist-off torque of the
#                                  Facet 28G Universal Lancet, FDA 510(k) K221433; BD publishes no value]
T_TAB_MAX_REAL = 0.049           # N m: the real tab is used only if it breaks at or under this (README Real trials)
TAB_BREAK = math.radians(45.0)   # the tab breaks off at this turn [EST]
K_TAB = T_TAB / TAB_BREAK        # N m/rad: the turn's spring, so its torque reaches T_TAB at the break
LIFT_OUT = 0.010                 # m: the lancet risen this far is out of the socket
L_START = (STAND[0], STAND[1], TOP + STAND_H + TAB_H + 0.0005)

# -- the sharps container (BD 305635, 1 qt phlebotomy collector with needle ports) -------------------------------------
SHARPS = (0.140, -0.225)         # its centre (world x, y). WHY: the YAM fingers reach 52 mm from the TCP axis; the twist
#                                  at the stand swept them into the container at (0.165, -0.215) [RUN, 2026-11-17]
SH_L, SH_W, SH_H = 0.1245, 0.1092, 0.1651   # [SOURCE: BD sharps catalog, 305635 "4.9 x 4.3 x 6.5" in, L x W x H]
SH_OPEN = (0.0635, 0.0305)       # the lid's opening [SOURCE: same catalog, "2.5 × 1.2 open (305635)" in]
SH_WALL = 0.003                  # [EST]

# -- the trainer hand (a heavy base with two foam rests, and the hand on a pivot) ----------------------------------------
# Hand sizes from benchmark-tasks #1837 (merged; ANSUR II median adult male, hand length 193 mm, breadth 88 mm). Hand
# frame (u, v, w): u along the fingers toward the wrist (0 at the index fingertip), v across (thumb +v), w out of the palm.
FINGERS = {  # name: (v, apex u, knuckle u, distal radius, proximal radius)
    "index": (0.000, 0.000, 0.095, 0.0082, 0.0088),
    "middle": (-0.024, -0.010, 0.093, 0.0078, 0.0083),
    "ring": (-0.045, -0.002, 0.097, 0.0073, 0.0078),
    "little": (-0.064, 0.022, 0.102, 0.0065, 0.0068),
}
DIP = 0.0266                     # the distal joint's distance from the apex (#1837, index)
PALM_U, PALM_V, PALM_W = (0.097, 0.176), (-0.073, 0.013), 0.013
THUMB = ((0.042, 0.058, 0.0095), (0.016, 0.165, 0.012))    # (v, u, radius) at its tip and its base (#1837)
STUB_U, STUB_V, STUB_R = (0.180, 0.262), -0.030, 0.022      # the forearm stub, a cylinder (#1837: 60 x 40 mm, here round)
ROLL = math.radians(-30.0)       # the hand turned about its fingers' axis, so the ring's ulnar pad faces straight up
HAND_KG = 0.300                  # #1837: "0.300 kg (silicone skin over a foam core)"
COM_U = 0.130                    # the hand's centre of mass along u [EST: the palm's middle]
PIVOT_U = 0.060                  # the finger rest's edge, under the proximal phalanges (the pivot)
TILT_STOP = -0.30                # rad: the fingertip side tips at most this far (onto the base)
BASE_KG = 3.0                    # the base plate (weighted, taped down in the real rig) [CHOICE]
APEX = (FINGERS["ring"][1], FINGERS["ring"][0])             # (u, v) of the ring's apex: the hand mechanism's origin
HZ = 0.070                       # the ring's axis over the table
HAND0 = (0.135, -0.030)          # the ring's apex over the table (world x, y), layout 0

# -- the site (BD VS7688-1: "Identify the desired puncture site (shaded area)"; "middle or ring finger is preferable";
# "The fifth finger must not be punctured") -----------------------------------------------------------------------------
SITE_FINGERS = ("middle", "ring")
SITE_S = (0.003, 0.015)          # m back from the apex [EST: the shaded crescents of the BD figure lie on the distal pad]
SITE_PHI = (math.radians(15), math.radians(45))   # from the pad's middle line, either side [EST: from the same figure]
ON_SKIN = 0.0015                 # m: the shield's face within this of a finger's surface is on it
TILT_MAX = math.radians(15)      # the lancet's axis within this of the skin's normal is square [CHOICE]
SLIDE_MAX = 0.0010               # m: the face moving more on the skin from first touch to firing is a slide [CHOICE]
WRIST_LIFT_MAX = 0.002           # m: the wrist rising more off its rest during the press is not steadied [CHOICE]

MATERIALS = {"default": (1.2, 1.0),
             "plastic": (0.5, 0.4),
             "skin": (0.5, 0.4),          # #1837: hand friction 0.5/0.4
             "foam": (1.0, 0.9),
             "rubber": (1.0, 0.9)}
WHITE, PINK, SKIN = (0.93, 0.93, 0.93), (0.90, 0.45, 0.60), (0.86, 0.70, 0.60)
RED, DARK, FOAM, GREY = (0.78, 0.10, 0.10), (0.10, 0.10, 0.11), (0.35, 0.40, 0.50), (0.55, 0.55, 0.57)


def _box(lo, hi, color, material="plastic", collide=True, mass=0.0):
    lo, hi = np.minimum(lo, hi).astype(float), np.maximum(lo, hi).astype(float)
    return Shape("box", tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, material=material,
                 collide=collide, mass=mass)


def _cyl(c, r, h, color, axis="Z", material="plastic", collide=True):
    return Shape("cylinder", tuple(float(v) for v in c), (float(r), float(h)), color, axis=axis, material=material,
                 collide=collide)


def _capsule_x(x0, x1, y, z, r, color, material="skin"):
    """A capsule along x whose rounded ends reach x0 and x1."""
    return Shape("capsule", ((x0 + x1) / 2, float(y), float(z)), (float(r), float(x1 - x0 - 2 * r)), color, axis="X",
                 material=material)


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


R_ROLL = _rx(ROLL)


def hand_to_mech(p):
    """A hand-frame point (u, v, w) in the hand mechanism's frame (origin at the ring's apex, x along the fingers)."""
    u, v, w = (float(t) for t in p)
    return R_ROLL @ np.array([u - APEX[0], v - APEX[1], w])


def mech_to_hand(p):
    q = R_ROLL.T @ np.asarray(p, float)
    return np.array([q[0] + APEX[0], q[1] + APEX[1], q[2]])


def _axis_yz(v):
    """Where a finger's axis (v, w = 0) lies in the mechanism frame's y, z (rolling keeps it along x)."""
    q = hand_to_mech((APEX[0], v, 0.0))
    return float(q[1]), float(q[2])


def _rod_hull(a, ra, b, rb, color, n=12):
    """A tapered rod from a to b (hand frame) as a hull of two rings, put in the mechanism frame."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = (b - a) / np.linalg.norm(b - a)
    e1 = np.cross(d, [0.0, 0.0, 1.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(d, e1)
    pts = [tuple(hand_to_mech(c + r * (math.cos(2 * math.pi * i / n) * e1 + math.sin(2 * math.pi * i / n) * e2)))
           for c, r in ((a, ra), (b, rb)) for i in range(n)]
    return hull(pts, color=color, material="skin")


def hand_shapes():
    """The trainer hand in the mechanism frame: two capsules a finger, the palm, the thumb and the forearm stub."""
    out = []
    for name, (v, u0, u1, rd, rp) in FINGERS.items():
        y, z = _axis_yz(v)
        x0, xd, x1 = u0 - APEX[0], u0 + DIP - APEX[0], u1 - APEX[0]
        out.append(_capsule_x(x0, xd + rd, y, z, rd, SKIN))
        out.append(_capsule_x(xd - rp, x1 + rp, y, z, rp, SKIN))
    palm = [tuple(hand_to_mech((u, v, w))) for w in (-PALM_W, PALM_W) for u, v in
            ((PALM_U[0], PALM_V[0]), (PALM_U[1], PALM_V[0]), (PALM_U[1], PALM_V[1]), (PALM_U[0], PALM_V[1]))]
    out.append(hull(palm, color=SKIN, material="skin"))
    (tv, tu, tr), (bv, bu, br) = THUMB
    out.append(_rod_hull((tu, tv, 0.0), tr, (bu, bv, 0.0), br, SKIN))
    sy, sz = _axis_yz(STUB_V)
    out.append(_cyl(((STUB_U[0] + STUB_U[1]) / 2 - APEX[0], sy, sz), STUB_R, STUB_U[1] - STUB_U[0], SKIN, axis="X",
                    material="skin"))
    return tuple(out)


HAND_SHAPES = hand_shapes()


def _lowest(x_lo, x_hi):
    """The hand's lowest point (mechanism z) over x_lo..x_hi: capsules and cylinders along x, hull points."""
    zs = []
    for s in HAND_SHAPES:
        lo, hi = s.bounds()
        if hi[0] < x_lo or lo[0] > x_hi:
            continue
        zs.append(lo[2] if s.kind != "hull" else min(p[2] for p in s.points if x_lo - 0.01 <= p[0] <= x_hi + 0.01))
    return min(zs)


PIVOT_X = PIVOT_U - APEX[0]
STUB_X = (STUB_U[0] + STUB_U[1]) / 2 - APEX[0]
STUB_YZ = _axis_yz(STUB_V)
REST_F_Z = _lowest(PIVOT_X - 0.012, PIVOT_X + 0.012)      # the finger rest's top (mechanism z)
REST_W_Z = STUB_YZ[1] - STUB_R                              # the wrist rest's top
COM_X = COM_U - APEX[0]


def hand_parts():
    """The base (plate and two foam rests) and the hand on its pivot, an axis across the fingers at the finger rest's
    edge. Gravity holds the wrist down on its rest (the pivot's upper stop); a press on a fingertip tips it up."""
    floor = -HZ
    base = (_box((PIVOT_X - 0.030, -0.030, floor), (STUB_X + 0.040, 0.030, floor + 0.005), GREY, material="rubber"),
            _box((PIVOT_X - 0.020, -0.028, floor + 0.005), (PIVOT_X, 0.028, REST_F_Z), FOAM, material="foam",
                 collide=False),
            _box((STUB_X - 0.025, STUB_YZ[0] - 0.020, floor + 0.005), (STUB_X + 0.025, STUB_YZ[0] + 0.020, REST_W_Z),
                 FOAM, material="foam", collide=False))
    com = hand_to_mech((COM_U, (PALM_V[0] + PALM_V[1]) / 2, 0.0))
    hand = Link("hand", HAND_KG, HAND_SHAPES, com=(COM_X, float(com[1]), float(com[2])))
    pivot = Joint("pivot", "revolute", "hand", (PIVOT_X, 0.0, REST_F_Z), "Y", TILT_STOP, 0.0, damping=0.02)
    return Link("hand_base", BASE_KG, base), (hand,), (pivot,)


def lancet_parts():
    """The lancet tip down: the housing (with its rear cap) and the contact shield on its spring."""
    base = (_box((-L_W / 2, -L_T / 2, SHIELD_OUT), (L_W / 2, L_T / 2, SHIELD_OUT + L_LEN), PINK),
            _box((-L_W / 2 + 0.001, -L_T / 2 - 0.0003, SHIELD_OUT + 0.012), (L_W / 2 - 0.001, L_T / 2 + 0.0003,
                 SHIELD_OUT + 0.022), WHITE, collide=False))   # the grip indentations, drawn
    shield = Link("shield", LANCET_KG[1], (_cyl((0.0, 0.0, 0.004), SHIELD_R, 0.008, WHITE),), com=(0.0, 0.0, 0.004))
    joint = Joint("shield", "prismatic", "shield", (0.0, 0.0, 0.0), "Z", 0.0, SHIELD_TRAVEL, stiffness=K_SHIELD,
                  damping=2.0, target=-F_PRE / K_SHIELD)
    return Link("lancet", LANCET_KG[0], base, com=(0.0, 0.0, SHIELD_OUT + L_LEN / 2)), (shield,), (joint,)


def bench_parts():
    """Fixed, in world x, y (z from the table): the stand with its turning socket (the tab in its floor), and the sharps
    container (walls, floor, and a lid with its opening)."""
    sx, sy, h = STAND[0], STAND[1], STAND_H
    s = [_box((sx - STAND_HALF, sy - STAND_HALF, 0.0), (sx + STAND_HALF, sy + STAND_HALF, h), DARK)]
    cx, cy, w = SHARPS[0], SHARPS[1], SH_WALL
    x0, x1, y0, y1 = cx - SH_L / 2, cx + SH_L / 2, cy - SH_W / 2, cy + SH_W / 2
    s += [_box((x0, y0, 0.0), (x1, y1, w), RED),
          _box((x0, y0, 0.0), (x0 + w, y1, SH_H), RED), _box((x1 - w, y0, 0.0), (x1, y1, SH_H), RED),
          _box((x0, y0, 0.0), (x1, y0 + w, SH_H), RED), _box((x0, y1 - w, 0.0), (x1, y1, SH_H), RED)]
    ox, oy = SH_OPEN[0] / 2, SH_OPEN[1] / 2
    lid = SH_H - w
    s += [_box((x0, y0, lid), (cx - ox, y1, SH_H), RED), _box((cx + ox, y0, lid), (x1, y1, SH_H), RED),
          _box((cx - ox, y0, lid), (cx + ox, cy - oy, SH_H), RED), _box((cx - ox, cy + oy, lid), (cx + ox, y1, SH_H), RED)]
    base = Link("bench", 5.0, tuple(s))
    ix, iy, top = L_W / 2 + CUP_CLEAR, L_T / 2 + CUP_CLEAR, h + TAB_H + CUP_D
    cup = (_box((sx - ix, sy - iy, h), (sx + ix, sy + iy, h + TAB_H), WHITE),                       # the tab cap
           _box((sx - ix - CUP_WALL, sy - iy - CUP_WALL, h), (sx - ix, sy + iy + CUP_WALL, top), GREY),
           _box((sx + ix, sy - iy - CUP_WALL, h), (sx + ix + CUP_WALL, sy + iy + CUP_WALL, top), GREY),
           _box((sx - ix, sy - iy - CUP_WALL, h), (sx + ix, sy - iy, top), GREY),
           _box((sx - ix, sy + iy, h), (sx + ix, sy + iy + CUP_WALL, top), GREY))
    socket = Link("socket", 0.004, cup)
    joint = Joint("tab", "revolute", "socket", (sx, sy, h), "Z", -0.05, math.radians(100), stiffness=K_TAB,
                  damping=0.002)
    return base, (socket,), (joint,)


_HB, _HL, _HJ = hand_parts()
HAND = mechanism("hand", (HAND0[0], HAND0[1], TOP + HZ), _HB, _HL, _HJ, MATERIALS, self_collide=False, free=True)
_LB, _LL, _LJ = lancet_parts()
LANCET = mechanism("lancet", L_START, _LB, _LL, _LJ, MATERIALS, self_collide=False, free=True)
_BB, _BL, _BJ = bench_parts()
BENCH = mechanism("bench", (0.0, 0.0, TOP), _BB, _BL, _BJ, MATERIALS, self_collide=False)
OBJECTS = [BENCH, HAND, LANCET]

# -- the layouts by seed: the hand's shift and turn on the table ----------------------------------------------------------
LAYOUTS = ((0.000, 0.000, 0.00),
           (0.012, -0.010, math.radians(6)),
           (-0.010, 0.012, math.radians(-6)))


def layout_index(seed):
    """Which layout a trial gets: inspect-robots' --seed 0, 2, 4 give 1, 0, 2 (README; the tests check)."""
    return 0 if seed is None else int(seed) % 3


def layout(seed):
    dx, dy, yaw = LAYOUTS[layout_index(seed)]
    return {"hand": (HAND0[0] + dx, HAND0[1] + dy, TOP + HZ, yaw)}


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
    """Put a loose object (a free articulation) at a world pose, still."""
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
    """The trial's layout: the hand's base shifted and turned by seed (before the reset settles)."""
    x, y, z, yaw = layout(run_seed())["hand"]
    _write_root(env.unwrapped, "hand", (x, y, z), yaw)


# -- geometry the model and the scorer share ----------------------------------------------------------------------------
def shield_force(q):
    """The push on the shield (N) from its travel: off its stop, the preload plus the spring; on it, 0."""
    return F_PRE + K_SHIELD * q if q > 5e-5 else 0.0


def face_w(l_pos, l_quat, q=0.0):
    """The shield's face centre (world), the shield pushed in by q."""
    return np.asarray(l_pos, float) + _rot3(l_quat) @ np.array([0.0, 0.0, q])


def tcp_over_lancet():
    """The TCP over the lancet's frame when held: pad tips 18 mm up the housing (clear of the socket's rim)."""
    return PAD_TIP_L + TIP


def wrist_lift(theta):
    """How far the stub's centre rises off its rest for a pivot turn theta (negative tips the fingertips down)."""
    return max(0.0, -(STUB_X - PIVOT_X) * math.sin(theta))


def site_of(p_mech):
    """Where a point (hand mechanism frame, at rest pose of the hand link) touches the hand: the nearest finger's distal
    pad as (finger, s from its apex, phi from the pad's middle line, inward normal in the mechanism frame, distance off
    the surface), or (part, ...) for the rest of the hand, or None off the hand."""
    h = mech_to_hand(p_mech)
    best = None
    for name, (v, u0, u1, rd, rp) in FINGERS.items():
        for seg, (a, b, r) in (("pad", (u0 + rd, u0 + DIP, rd)), ("finger", (u0 + DIP, u1, rp))):
            u = min(max(h[0], a), b)
            rho = np.array([h[0] - u, h[1] - v, h[2]])
            d = float(np.linalg.norm(rho)) - r
            if best is None or d < best[4]:
                n_in = -(R_ROLL @ (rho / max(np.linalg.norm(rho), 1e-9)))
                phi = math.atan2(float(rho[1]), float(rho[2]))
                best = (name if seg == "pad" else name + " (not the fingertip)", float(h[0] - u0), phi, n_in, d)
    if best[4] <= ON_SKIN:
        return best
    for part, lo, hi in (("palm", (PALM_U[0], PALM_V[0], -PALM_W), (PALM_U[1], PALM_V[1], PALM_W)),):
        if all(lo[i] - ON_SKIN <= h[i] <= hi[i] + ON_SKIN for i in range(3)):
            return (part, 0.0, 0.0, np.array([0.0, 0.0, -1.0]), 0.0)
    if STUB_U[0] - ON_SKIN <= h[0] <= STUB_U[1] + ON_SKIN and math.hypot(h[1] - STUB_V, h[2]) <= STUB_R + ON_SKIN:
        return ("forearm", 0.0, 0.0, np.array([0.0, 0.0, -1.0]), 0.0)
    return None


def site_ok(site):
    """(ok, why not): the side of a middle or ring fingertip's pad."""
    if site is None:
        return False, "not on the hand"
    name, s, phi = site[0], site[1], abs(site[2])
    if name == "little":
        return False, "on the little finger (BD: the fifth finger must not be punctured)"
    if name not in SITE_FINGERS:
        return False, f"on the {name}, not the side of a middle or ring fingertip"
    if not (SITE_S[0] <= s <= SITE_S[1]):
        return False, f"{1000 * s:.1f} mm from the {name}'s apex, off the pad"
    if phi < SITE_PHI[0]:
        return False, f"on the middle of the {name}'s pad, not its side"
    if phi > SITE_PHI[1]:
        return False, f"too far round the {name}'s side ({math.degrees(phi):.0f} deg)"
    return True, ""


def tilt_of(l_quat, hand_quat, n_in_mech):
    """The angle (rad) between the lancet's pointing direction and the skin's inward normal, both in the world."""
    point = -_rot3(l_quat)[:, 2]
    n = _rot3(hand_quat) @ np.asarray(n_in_mech, float)
    return math.acos(max(-1.0, min(1.0, float(point @ n))))


def in_sharps(l_pos):
    """The lancet's frame inside the container: within its walls and under its lid."""
    p = np.asarray(l_pos, float)
    return bool(abs(p[0] - SHARPS[0]) <= SH_L / 2 - SH_WALL and abs(p[1] - SHARPS[1]) <= SH_W / 2 - SH_WALL
                and TOP + SH_WALL - 0.002 <= p[2] <= TOP + SH_H - SH_WALL)


def in_socket(l_pos):
    p = np.asarray(l_pos, float)
    return bool(math.hypot(p[0] - L_START[0], p[1] - L_START[1]) <= 0.004 and p[2] - L_START[2] <= LIFT_OUT)


def grip_point(l_pos, l_quat):
    return np.asarray(l_pos, float) + _rot3(l_quat) @ np.array([0.0, 0.0, tcp_over_lancet()])


def held_by(hand, l_pos, l_quat):
    """A hand closed on the lancet: its TCP within 20 mm of the grip point, its fingers at most 0.5 mm outside the
    housing's thickness."""
    if hand is None or hand[0] is None or hand[1] is None:
        return False
    return bool(np.linalg.norm(np.asarray(hand[0]) - grip_point(l_pos, l_quat)) <= 0.020 and -hand[1] <= L_T / 2 + 0.0005)


# -- the model -----------------------------------------------------------------------------------------------------------
def _fresh_model(seed=None):
    return {"t": 0.0, "tab_off": False, "out": False, "fault": None, "press": None, "max_lift": 0.0,
            "positioned": False, "fired": False, "fire": None, "free_t": 0.0, "q": 0.0, "theta": 0.0, "tab": 0.0,
            "right": None, "left": None, "steps": 0, "error": None, "layout": layout_index(seed), "base0": None}


_M = _fresh_model()


def _latch(m, why):
    if not m["fault"]:
        m["fault"] = why


def advance(m, dt, l_pos, l_quat, q, tab, theta, hand_pos, hand_quat, right, left=None):
    """One step: the tab's break, the lancet out of the socket, the press on the skin (where, how square, slid or not,
    the wrist steadied or not), the firing, and the used lancet's fate."""
    m["t"] += dt
    m["q"], m["tab"], m["theta"] = float(q), float(tab), float(theta)
    m["right"], m["left"] = right, left
    if tab >= TAB_BREAK:
        m["tab_off"] = True
    if not m["out"] and not in_socket(l_pos):
        m["out"] = True
        if not m["tab_off"]:
            _latch(m, "lancet lifted out of the stand with its tab still on")
    f = shield_force(q)
    Rh = _rot3(hand_quat)
    face = face_w(l_pos, l_quat, q)
    p_mech = Rh.T @ (face - np.asarray(hand_pos, float))
    if m["out"] and m["tab_off"] and not m["fired"] and f >= F_TOUCH:
        if m["press"] is None:
            m["press"], m["max_lift"] = p_mech.copy(), 0.0
        m["max_lift"] = max(m["max_lift"], wrist_lift(theta))
        if m["max_lift"] > WRIST_LIFT_MAX:
            _latch(m, f"pressed with the hand not steadied (the wrist rose {1000 * m['max_lift']:.1f} mm)")
        slide = float(np.linalg.norm(p_mech - m["press"]))
        if slide > SLIDE_MAX:
            _latch(m, f"the press slid {1000 * slide:.1f} mm on the skin")
        site = site_of(p_mech)
        ok, why = site_ok(site)
        square = site is not None and tilt_of(l_quat, hand_quat, site[3]) <= TILT_MAX
        if ok and square and f >= F_FIRM and m["max_lift"] <= WRIST_LIFT_MAX:
            m["positioned"] = True
        if f >= F_FIRE:
            m["fired"] = True
            tilt = tilt_of(l_quat, hand_quat, site[3]) if site is not None else math.pi
            m["fire"] = {"site": None if site is None else site[:3], "tilt": tilt, "lift": m["max_lift"],
                         "slide": slide}
            if not ok:
                _latch(m, "fired " + why)
            elif tilt > TILT_MAX:
                _latch(m, f"fired tilted {math.degrees(tilt):.0f} deg from square to the skin")
            elif m["max_lift"] > WRIST_LIFT_MAX:
                _latch(m, f"fired with the hand not steadied (the wrist rose {1000 * m['max_lift']:.1f} mm)")
    elif f < F_TOUCH:
        m["press"] = None
    if m["out"] and not in_sharps(l_pos) and not held_by(right, l_pos, l_quat) and not held_by(left, l_pos, l_quat):
        m["free_t"] += dt
        if m["free_t"] > DROP_S:
            _latch(m, "lancet dropped outside the sharps container")
    else:
        m["free_t"] = 0.0


DROP_S = 0.5                     # s loose outside the stand and the container is a drop (the fall into it takes 0.2 s)

_IDS = {}
_CLOCK = {"dt": None, "steps": 0}
TCP_OFFSET = (0.0, 0.0, 0.134)   # yam_arena.constants: the TCP along the gripper's approach axis


def _ids(asset, key, names):
    if key not in _IDS:
        all_names = list(asset.joint_names)
        _IDS[key] = {j: all_names.index(j) for j in names}
    return _IDS[key]


@on_reset
def reset_model():
    reset_memory(run_seed())


@on_step
def run_model(env):
    """Read the lancet and its shield, the socket's turn, the hand's pivot and link, and both hands; advance the model."""
    try:
        base = env.unwrapped
        lan, hand, bench = base.scene["lancet"], base.scene["hand"], base.scene["bench"]
        origin = _np(base.scene.env_origins)[0]
        pl = _np(lan.data.root_link_pose_w)[0]
        q = float(_np(lan.data.joint_pos)[0][_ids(lan, "lancet", ("shield",))["shield"]])
        tab = float(_np(bench.data.joint_pos)[0][_ids(bench, "bench", ("tab",))["tab"]])
        theta = float(_np(hand.data.joint_pos)[0][_ids(hand, "hand", ("pivot",))["pivot"]])
        ph = _np(hand.data.body_link_pose_w)[0][list(hand.body_names).index("hand")]
        hands = []
        for side in ("right", "left"):
            rob = base.scene[f"{side}_robot"]
            g = _np(rob.data.body_link_pose_w)[0][list(rob.body_names).index("gripper")]
            qr = _np(rob.data.joint_pos)[0]
            hands.append((g[:3] - origin + _rot3(g[3:7]) @ np.array(TCP_OFFSET),
                          float(qr[list(rob.joint_names).index("joint7")])))
        if _M["base0"] is None:
            _M["base0"] = _np(hand.data.root_link_pose_w)[0][:3] - origin
        _M["base"] = _np(hand.data.root_link_pose_w)[0][:3] - origin
        advance(_M, float(base.step_dt), pl[:3] - origin, pl[3:7], q, tab, theta, ph[:3] - origin, ph[3:7], hands[0],
                hands[1])
        _M["steps"] += 1
    except Exception as e:  # WHY: never stop the run; no model step means no firing and so no full score
        _M["error"] = "step " + repr(e)


@on_step
def read_clock(env):
    try:
        _CLOCK["dt"] = float(env.unwrapped.step_dt)
    except Exception:  # WHY: never stop the run; no step length means no hold, so no full score
        _CLOCK["dt"] = None
    _CLOCK["steps"] += 1


# -- scoring -------------------------------------------------------------------------------------------------------------
LOST_Z = TOP - 0.030
BASE_MOVED = 0.005               # m: the hand's base dragged this far from its start is a fault


def failure(objects, m):
    """Latched faults: the model's (tab on when lifted, a slide, a bad fire, a drop), the hand's base dragged, the lancet
    off the table, or the model not running."""
    if m["fault"]:
        return m["fault"]
    b = objects["hand"]
    start = np.array(HAND0) + np.array(LAYOUTS[m["layout"]][:2])
    if float(np.linalg.norm(np.asarray(b.pos[:2], float) - start)) > BASE_MOVED:
        return "trainer hand's base dragged out of place"
    if float(objects["lancet"].pos[2]) < LOST_Z:
        return "lancet lost"
    if m["error"]:
        return "model " + m["error"]
    if _CLOCK["steps"] - m["steps"] > 5:
        return "model not stepping"
    return None


def stage(objects, m):
    """0 nothing; 1 tab off and the lancet out; 2 and placed square on a good site, steadied; 3 and fired there; 4 and
    the used lancet in the sharps container, let go."""
    if m["fired"]:
        l = objects["lancet"]
        if (in_sharps(l.pos) and not held_by(m["right"], l.pos, l.quat_xyzw)
                and not held_by(m["left"], l.pos, l.quat_xyzw)):
            return 4
        return 3
    if m["positioned"]:
        return 2
    return 1 if (m["tab_off"] and m["out"]) else 0


def _fresh():
    return {"last": None, "fail": None, "now": None, "held": 0.0}


_MEM = _fresh()


def reset_memory(seed=None):
    _M.clear()
    _M.update(_fresh_model(seed))
    _MEM.clear()
    _MEM.update(_fresh())
    _CLOCK.update({"dt": None, "steps": 0})


def update(objects):
    """One control step: latch a failure and judge this step (same states dict for success() and score())."""
    if objects is _MEM["last"]:
        return _MEM
    _MEM["last"] = objects
    why = failure(objects, _M)
    if why and not _MEM["fail"]:
        _MEM["fail"] = why
    n = {"stage": stage(objects, _M)}
    n["done"] = bool(not _MEM["fail"] and n["stage"] == 4)
    _MEM["held"] = _MEM["held"] + (_CLOCK["dt"] or 0.0) if n["done"] else 0.0
    _MEM["now"] = n
    return _MEM


def success(objects):
    """Tab off, fired on a good site square and steadied without a slide, the used lancet let go in the sharps container
    (held 1 s by yam_arena), with no latched fault."""
    return update(objects)["now"]["done"]


LADDER = {0: 0.0, 1: 0.2, 2: 0.4, 3: 0.6}
HOLD_S = 1.0


def score(objects):
    """At the last step: 0 after a latched fault; success held 1 s 1.0; in the container but not yet held 1 s 0.8;
    fired well 0.6; placed well 0.4; tab off and out 0.2."""
    m = update(objects)
    if m["fail"]:
        return 0.0
    n = m["now"]
    if n["stage"] == 4:
        return 1.0 if (n["done"] and m["held"] >= HOLD_S - 1e-9) else 0.8
    return LADDER[n["stage"]]


# -- demo ----------------------------------------------------------------------------------------------------------------
TIP = 0.013                      # PAD_TIP_PAST_TCP: the pad tips sit 13 mm under the TCP
PAD_TIP_L = 0.018                # the pad tips this far up the lancet from its frame (the socket's rim is at 10 mm)
OPEN_F = -0.040
GRIP_L = -(L_T / 2 - 0.004)      # 4 mm into the housing's 10 mm: 20 N a finger at 5000 N/m. WHY: at 2.5 mm the housing
#                                  slid up the pads at 7.9 N, under the 8 N fire (friction about 0.34) [RUN]
TWIST = math.radians(70)         # the demo turns the lancet this far in the socket (the tab breaks at 45 deg)
PRESS_DZ = 0.020                 # the press aims this far under contact. WHY: the arm is about 0.5 N/mm at the TCP
#                                  [EST: mug-handle task, from yam_arena's joint stiffnesses], so 20 mm is about 10 N,
#                                  over the 8 N fire after the shield's 1.75 mm
STUB_TCP = 0.005                 # the left TCP over the stub's axis: pad tips 8 mm under it, across its widest
GRIP_S = -(STUB_R - 0.0025)      # 2.5 mm into the stub's 44 mm: 12.5 N a finger
HOLD_DZ = 0.010                  # the left hand pushes the stub down by aiming this far under it (about 5 N [EST])
RELEASE_Z = TOP + 0.205          # the TCP over the container's opening at release
STEADY = 0.8                     # s held over a target before each descent (the arm lags 2 to 10 mm on a descent [RUN,
#                                  mug-handle task])
SITE_AIM = 0.008                 # the demo's site: 8 mm back from the ring's apex, the side facing up
PARK_L = (0.280, 0.200, TOP + 0.150)   # the left TCP parked, gripper down, while the right takes the lancet
APPROACH_L = 0.030               # the left comes down onto the stub from this far over it (G12: low reach out at x 0.37)


def _w(t, p, f, label, yaw):
    return Waypoint(float(t), tuple(float(v) for v in p), float(f), label, float(yaw))


class _Arm:
    def __init__(self):
        self.w, self.t = [], 0.0

    def add(self, dt, p, f, label, yaw):
        self.w.append(_w(dt, p, f, label, yaw))
        self.t += dt


def site_world(seed, s=SITE_AIM):
    """The demo's contact point (world) and the hand's yaw: on the ring's axis top, s back from its apex."""
    hx, hy, hz, yaw = layout(seed)["hand"]
    c, sn = math.cos(yaw), math.sin(yaw)
    r = FINGERS["ring"][3]
    return np.array([hx + c * s, hy + sn * s, hz + r]), yaw


def stub_world(seed):
    hx, hy, hz, yaw = layout(seed)["hand"]
    c, sn = math.cos(yaw), math.sin(yaw)
    x, y = STUB_X, STUB_YZ[0]
    return np.array([hx + c * x - sn * y, hy + sn * x + c * y, hz + STUB_YZ[1]]), yaw


def plan(delay=0.0, seed=None):
    """Both arms: the right takes the lancet, twists its tab off, carries it over the fingertip; the left holds the wrist
    down; the right presses square until it fires, lifts, and drops it into the container."""
    R, L = _Arm(), _Arm()
    lg = np.array(L_START) + np.array([0.0, 0.0, tcp_over_lancet()])
    R.add(2.0 + delay, lg + [0, 0, 0.05], OPEN_F, "over the lancet", 0.0)
    R.add(STEADY, lg + [0, 0, 0.05], OPEN_F, "steady over the lancet", 0.0)
    R.add(1.5, lg, OPEN_F, "down to the lancet", 0.0)
    R.add(0.6, lg, GRIP_L, "grip the housing", 0.0)
    R.add(0.6, lg, GRIP_L, "settle", 0.0)
    R.add(2.0, lg, GRIP_L, "twist the tab off", TWIST)
    R.add(0.5, lg, GRIP_L, "hold", TWIST)
    R.add(1.2, lg + [0, 0, 0.06], GRIP_L, "lift it out of the stand", TWIST)
    site, hyaw = site_world(seed)
    yaw_p = math.pi / 2 + hyaw     # fingers close along the finger, so the pads pass clear of the neighbours
    touch = site + [0, 0, tcp_over_lancet()]
    R.add(2.5, touch + [0, 0, 0.045], GRIP_L, "over the fingertip", yaw_p)
    R.add(STEADY, touch + [0, 0, 0.045], GRIP_L, "steady over the fingertip", yaw_p)
    t_left = R.t
    R.add(1.5, touch + [0, 0, 0.004], GRIP_L, "just over the site", yaw_p)
    R.add(3.4, touch + [0, 0, 0.004], GRIP_L, "wait for the wrist to be held", yaw_p)
    R.add(0.8, touch, GRIP_L, "tip on the site", yaw_p)
    R.add(1.5, touch - [0, 0, PRESS_DZ], GRIP_L, "press until it fires", yaw_p)
    R.add(1.0, touch - [0, 0, PRESS_DZ], GRIP_L, "hold the press", yaw_p)
    R.add(1.2, touch + [0, 0, 0.045], GRIP_L, "lift off", yaw_p)
    t_free = R.t
    up = (touch[0], touch[1], RELEASE_Z)
    R.add(1.2, up, GRIP_L, "up", yaw_p)
    over = (SHARPS[0], SHARPS[1], RELEASE_Z)
    R.add(2.0, over, GRIP_L, "over the sharps container", 0.0)
    R.add(STEADY, over, GRIP_L, "steady over the opening", 0.0)
    R.add(0.6, over, OPEN_F, "let go", 0.0)
    # WHY +x: straight up over the container's new centre is past the arm's reach (IK 2.5 mm off)
    R.add(1.5, (SHARPS[0] + 0.03, SHARPS[1] + 0.02, RELEASE_Z + 0.01), OPEN_F, "up clear", 0.0)
    # the left hand: wait at READY while the right takes the lancet, hold the wrist down through the press, then back
    sg, syaw = stub_world(seed)
    g = sg + [0, 0, STUB_TCP]
    L.add(2.0, PARK_L, OPEN_F, "park, gripper down", 0.0)
    L.add(t_left - L.t, PARK_L, OPEN_F, "wait parked", 0.0)
    L.add(2.0, g + [0, 0, APPROACH_L], OPEN_F, "over the wrist", syaw)
    L.add(STEADY, g + [0, 0, APPROACH_L], OPEN_F, "steady over the wrist", syaw)
    L.add(1.2, g, OPEN_F, "down round the wrist", syaw)
    L.add(0.6, g, GRIP_S, "grip the wrist", syaw)
    L.add(0.6, g - [0, 0, HOLD_DZ], GRIP_S, "hold it down on its rest", syaw)
    L.add(t_free - L.t, g - [0, 0, HOLD_DZ], GRIP_S, "keep holding through the press", syaw)
    L.add(0.6, g, GRIP_S, "ease off", syaw)
    L.add(0.6, g, OPEN_F, "let go of the wrist", syaw)
    L.add(1.2, g + [0, 0, APPROACH_L], OPEN_F, "up", syaw)
    L.add(2.0, PARK_L, OPEN_F, "back to park", 0.0)
    return R.w, L.w


def demo(delay=0.0):
    right, left = plan(delay, run_seed())
    return {"right": right, "left": left}


def _replan_demo_after_reset():
    """WHY: inspect-robots resets the policy before the embodiment (inspect_robots/rollout.py), so yam_scripted would
    call demo() while run_seed() still holds the previous trial's seed and plan for the wrong layout. This makes
    yam_scripted plan again on each trial's first action, after the reset has set the seed. It changes only the
    scripted demo policy, never the scene or the score."""
    try:
        from yam_arena import scripted
    except ImportError:
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_lancet_replan", False):
        return
    reset, act = cls.reset, cls.act
    unset = object()

    def patched_reset(self, scene):
        self._lancet_scene, self._lancet_seed = scene, unset
        reset(self, scene)

    def patched_act(self, observation):
        if getattr(self, "_lancet_seed", None) is unset:
            self._lancet_seed = run_seed()
            reset(self, self._lancet_scene)
        return act(self, observation)

    cls.reset, cls.act, cls._lancet_replan = patched_reset, patched_act, True


_replan_demo_after_reset()
