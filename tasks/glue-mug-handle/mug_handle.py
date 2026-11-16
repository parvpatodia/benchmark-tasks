"""A stoneware mug whose handle has broken off lies on its side in a foam cradle, the two broken stubs on its side facing
up. The handle, in one piece, stands on its two broken ends on a felt pad. A tube of no-drip gel glue (LePage Extreme
Glue No Drip Gel) stands nozzle down in its holder, its cap off. Glue the handle back on as the glue's maker directs:
first dry-fit it (seat both broken ends on their stubs at once, then lift it off and stand it back on its two ends on the pad); then
squeeze a thin bead from the tube onto each stub's broken face, keeping glue off everything else; put the tube back in
its holder before you seat the handle; and within 5 minutes of the first glue (the maker's limit) seat both ends of
the handle on their stubs at once, flush, press it down firmly on both and let go, leaving the mug and the handle undisturbed in the cradle.
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import (Joint, Link, Shape, body, hull, mechanism, on_place, on_reset, on_step,  # noqa: E402
                       run_seed)
from yam_arena import station  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
RUN_LIMIT_STEPS = 3000           # 150 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the mug (a loose articulated body: the mug, and the two glue beads its model raises) --------------------------------
R_MUG = 0.039                    # the body's radius [ARITH: DINERA "30 cl" in Height "9 cm" less a 7 mm base [EST] is
#                                  bore of 33.9 mm radius; plus a 5 mm wall [EST]]
L_MUG = 0.090                    # [SOURCE: IKEA DINERA mug, Height "9 cm"] its length (it lies along world y)
HANDLE_OUT = 0.030               # the handle stands 30 mm off the side [EST: inside the 12 cm package length less the
#                                  78 mm body, 42 mm]
STUB_H = 0.003                   # the broken stubs stand 3 mm off the side [EST]
STUB_TRAVEL = 0.0025             # each stub rides a stiff spring (only to measure the press on it; the real stub is rigid)
K_STUB = 2500.0                  # N/m
STUB_PRE = 1.0                   # N preload against its stop: the handle's weight (0.12 N an end) never moves it
F_PRESS = 3.0                    # N on each stub, held PRESS_S, is "press firmly" [CHOICE: 12 times the handle's weight]
PRESS_S = 0.5
STUB_Y = (-0.0275, 0.0275)       # the stubs along the mug's axis (from its middle; the rim is at +y) [EST: a 55 mm span]
SEC_W, SEC_D = 0.012, 0.008      # the handle's section: 12 mm across (x) by 8 mm deep (in its plane) [EST]
MUG_KG = 0.28                    # [EST: under IKEA's 0.32 kg package weight, less the 25 g handle and the packing]
PARK_IN, PARK_DZ, PARK_TILT = 0.010, 0.003, 10 * math.pi / 180   # on the pad: its centre this far inside the pad's edge,
#                                  at its start height within this, upright within this
MUG_SIDES = 32                   # the body is a prism of this many sides, 64 vertices: PhysX's default hull limit
MUG_COM_DROP = 0.003             # the body's centre of mass under its axis (see mug_parts)
MX, MY0 = 0.245, -0.095          # the mug's axis over the table (world x, y) for layout 0
RAIL_IN, RAIL_W, RAIL_H, RAIL_L = 0.022, 0.016, 0.015, 0.160   # the cradle's two foam rails, their inner edges 22 mm off
ZC = RAIL_H + math.sqrt(R_MUG ** 2 - RAIL_IN ** 2)   # the mug's axis over the table, resting on the rails' inner edges
Z_FACE = ZC + R_MUG + STUB_H     # the broken faces over the table (mug frame: R_MUG + STUB_H over its axis)

# -- the handle (a loose rigid body; its frame between its broken ends, at their faces) -----------------------------------
ARCH_A = -STUB_Y[0]              # half the span between the broken ends
ARCH_B = HANDLE_OUT - STUB_H - SEC_D / 2   # the section's centre line at the top of the arch (23 mm)
ARCH_N = 8                       # hull segments along the arch
HANDLE_KG = 0.025                # [EST]
PAD = (0.190, -0.215)            # the felt pad's centre (world x, y)
PAD_H = 0.004
PAD_HALF = 0.045

# -- the glue tube (a loose articulated tool: the tube, and its two walls that the fingers squeeze) ----------------------
TUBE_W = 0.022                   # the tube across its walls [EST; the TDS sells it as a "Squeeze Tube", "20 g"]
WALL_T = 0.003
WALL_Y = 0.024                   # the squeezable walls' span across (y)
WALL_LO, WALL_HI = -0.030, 0.060 # and their bottom and top about the tube's frame: the whole tube above the shoulder, so
#                                  the fingers touch only the walls however high they hold it
SQUEEZE_MAX = 0.006              # each wall's travel to its stop
K_WALL = 2000.0                  # N/m: the tube's resistance to a squeeze, per wall [EST]
WALL_PRE = 1.0                   # N: the walls' preload against their stops [EST]
NOZZLE_Z = -0.051                # the nozzle's tip under the tube's frame
SHOULDER_Z = -0.036              # the shoulder's underside (it rests on the holder) under the frame
HOLD = (0.300, -0.215)           # the tube holder's centre (world x, y)
HOLD_H = 0.035
HOLD_HOLE = 0.016                # the holder's square well (the nozzle hangs in it; the shoulder rests on its rim)
TUBE_KG = (0.022, 0.004, 0.004)  # tube with the glue, wall A, wall B [EST: 20 g of gel in a ~10 g tube]

# -- the glue (our model: see fidelity) ---------------------------------------------------------------------------------
F_YIELD = 4.0                    # N per wall: a no-drip gel does not flow under this squeeze [EST]
FLOW_G = 2.5                     # uL per s per N over F_YIELD [EST: 8 N gives 10 uL/s]
V_LO, V_HI = 10.0, 30.0          # uL on each broken face: a thin layer over its 96 mm2 (0.10 to 0.31 mm) [CHOICE]
STRAY_MAX = 1.0                  # uL of glue anywhere but the two broken faces is a fault [CHOICE]
DEP_IN = 0.0015                  # the nozzle's tip at least this far inside a face's edges puts the glue on it
DEP_H = 0.004                    # m: the tip at most this far over a face (a drop bridges it) [CHOICE]
UPRIGHT = math.cos(20 * math.pi / 180)   # the nozzle points down within 20 deg, or the glue goes astray
OPEN_TIME = 300.0                # s [SOURCE: Extreme Glue No Drip Gel TDS, Open Time "3-5 minutes"; "within 5 minutes"]
BEAD_R, BEAD_T = 0.0030, 0.0025  # the drawn bead (drawn pale so it shows; the gel is "Colorless, Clear")
BEAD_DROP = 0.0026               # the bead hides this far inside the stub until glue is on it; it rises by the square
#                                  root of its share of V_HI (10 uL: rises 1.5 mm, 1.4 mm proud), and under a seated end sinks to a
#                                  0.3 mm layer

# -- seating -------------------------------------------------------------------------------------------------------------
SEAT_XY = 0.0015                 # m: a broken end within 1.5 mm of its stub's face across and along [CHOICE: a larger
#                                  step shows and catches a fingertip; the first sim run held the demo's ends 1.0 mm off]
LIFT_OFF = 0.002                 # m: an end more than this over its face is lifted off (see on_stubs). WHY 2 mm: the
#                                  stubs' springs, released in 0.4 s from a press, threw the handle up to 1.3 mm (run 9)
SEAT_Z = (-0.0003, 0.0006)       # m: and from 0.3 mm into it to 0.6 mm over it in height (PhysX rests the bodies 0 to
#                                  0.4 mm apart [RUN]); contact itself is proved by the press on each stub
SEAT_TILT = math.cos(4 * math.pi / 180)   # the handle's plane within 4 deg of the mug's axis (either way) and upright
HAND_CLEAR = 0.030               # m: each hand's TCP this far from the handle's grip (or the tube's) with its fingers
OPEN_CLEAR = 0.003               # open, each finger this far outside the part it held;
FAR_CLEAR = 0.100                # or the TCP this far away, open or not (the idle left hand)
MOVED = 0.0010                   # m: the handle moving this far once let go on its glue is a fault
IN_CRADLE = 0.008                # m: the mug's axis within this of its start
MUG_UPRIGHT = math.cos(5 * math.pi / 180)   # and the stubs within 5 deg of straight up
HOME_XY, HOME_Z = 0.010, 0.006   # the tube back in its holder
STILL_V = 0.01                   # m/s

MATERIALS = {"default": (1.2, 1.0),   # the scene's friction on its own assets (rules.md 3)
             "ceramic": (0.6, 0.5),    # glazed stoneware [EST]
             "foam": (1.0, 0.9),
             "felt": (0.8, 0.7),
             "plastic": (0.5, 0.4)}
GLAZE, BISCUIT, DARK = (0.27, 0.28, 0.30), (0.86, 0.81, 0.70), (0.10, 0.10, 0.11)
WHITE, RED, BLACK = (0.93, 0.93, 0.93), (0.75, 0.12, 0.10), (0.07, 0.07, 0.08)
FELT, FOAM, GEL = (0.15, 0.35, 0.20), (0.35, 0.40, 0.50), (0.97, 0.97, 0.90)


def _box(lo, hi, color, material="plastic", collide=True, mass=0.0):
    lo, hi = np.minimum(lo, hi).astype(float), np.maximum(lo, hi).astype(float)
    return Shape("box", tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, material=material,
                 collide=collide, mass=mass)


def _cyl(c, r, h, color, axis="Z", material="plastic", collide=True, mass=0.0):
    return Shape("cylinder", tuple(float(v) for v in c), (float(r), float(h)), color, axis=axis, material=material,
                 collide=collide, mass=mass)


def _hidden(z=0.0):
    """A 1 mm collider deep inside its parent's body (nothing reaches it), so a drawn-only link is a rigid body."""
    return _box((-0.0005, -0.0005, z - 0.0005), (0.0005, 0.0005, z + 0.0005), DARK)


# -- the scene's parts ---------------------------------------------------------------------------------------------------
def mug_parts():
    """The mug in its own frame (its axis along y, the rim at +y), its two broken stubs on top, each on a stiff
    preloaded spring that measures the press on it, and a glue bead on each stub."""
    face = R_MUG + STUB_H
    # WHY a 32-sided prism, not a cylinder: on the rails' edges PhysX's cylinder collider spun the untouched mug in place,
    # 40 deg in 75 s on every seed (run 7); a 64-sided prism, alone changed, held it within 0.1 deg (run 7 diagnostic),
    # and this 32-sided one within 0.16 deg (run 10). 32 sides is 64 vertices, the installed PhysxSchema's default
    # hullVertexLimit [RUN: omni.usd.schema.physx 110.3.2, generatedSchema.usda], so PhysX cooks the prism as given
    body_ring = [(R_MUG * math.cos(2 * math.pi * i / MUG_SIDES), y, R_MUG * math.sin(2 * math.pi * i / MUG_SIDES))
                 for y in (-L_MUG / 2, L_MUG / 2) for i in range(MUG_SIDES)]
    base = (hull(body_ring, color=GLAZE, material="ceramic"),
            _cyl((0.0, L_MUG / 2 - 0.0005, 0.0), R_MUG - 0.005, 0.0012, DARK, axis="Y", collide=False))   # its mouth
    links, joints = [], []
    for k, y in enumerate(STUB_Y):
        links.append(Link(f"stub{k}", 0.003, (
            _box((-SEC_W / 2, y - SEC_D / 2, R_MUG - 0.004), (SEC_W / 2, y + SEC_D / 2, face), GLAZE, material="ceramic"),
            _box((-SEC_W / 2 + 0.0002, y - SEC_D / 2 + 0.0002, face - 0.0006),
                 (SEC_W / 2 - 0.0002, y + SEC_D / 2 - 0.0002, face + 0.0001), BISCUIT, collide=False)),
            com=(0.0, y, face - 0.003)))
        joints.append(Joint(f"stub{k}", "prismatic", f"stub{k}", (0.0, y, face), "Z", -STUB_TRAVEL, 0.0,
                            stiffness=K_STUB, damping=20.0, target=STUB_PRE / K_STUB))
        links.append(Link(f"bead{k}", 0.001, (_cyl((0.0, y, face - BEAD_DROP + BEAD_T / 2), BEAD_R, BEAD_T, GEL,
                                                   collide=False), _hidden(0.0))))
        joints.append(Joint(f"bead{k}", "prismatic", f"bead{k}", (0.0, y, face), "Z", -0.0005, BEAD_DROP + 0.0005,
                            stiffness=400.0, damping=10.0, target=0.0, parent=f"stub{k}"))
    # WHY: the sprung stubs put 6 g 39 mm over the axis, so with the body's centre on the axis the whole mug's centre
    # sits over it and the mug is top-heavy on the rails (it rolled over in run 6, then still a cylinder collider, so
    # which of the two did it is not separated). 3 mm under the axis puts the whole mug's centre 2 mm under it, so it
    # stays stubs up, as the foam holds a real one
    return Link("mug", MUG_KG, base, com=(0.0, 0.0, -MUG_COM_DROP)), tuple(links), tuple(joints)


def arch_point(t, s):
    """A point of the handle's arch (its frame), at parameter t (0 at the -y end, pi at the +y end) and s along the
    section's depth (outward positive): the centre line is half an ellipse, A across and B high."""
    cy, cz = -ARCH_A * math.cos(t), ARCH_B * math.sin(t)
    ny, nz = -ARCH_B * math.cos(t), ARCH_A * math.sin(t)
    n = math.hypot(ny, nz)
    return cy + s * ny / n, cz + s * nz / n


def handle_shapes():
    """The handle as ARCH_N hull segments of its 12 x 8 mm section; both ends are flat broken faces at z = 0."""
    out, h = [], SEC_D / 2
    for i in range(ARCH_N):
        rings = []
        for t in (math.pi * i / ARCH_N, math.pi * (i + 1) / ARCH_N):
            for x, s in ((-SEC_W / 2, -h), (SEC_W / 2, -h), (SEC_W / 2, h), (-SEC_W / 2, h)):
                y, z = arch_point(t, s)
                rings.append((x, y, max(0.0, z)))
        out.append(hull(rings, color=GLAZE, material="ceramic", mass=HANDLE_KG / ARCH_N))
    return tuple(out)


def tube_parts():
    """The tube standing nozzle down (its frame at the walls' middle): the tube and its glue, and the two walls on
    springs, which slide in (wall A from +x, wall B from -x) as the fingers squeeze them."""
    base = (_cyl((0.0, 0.0, NOZZLE_Z / 2 + SHOULDER_Z / 2), 0.0025, SHOULDER_Z - NOZZLE_Z, WHITE),     # the nozzle
            _cyl((0.0, 0.0, SHOULDER_Z + 0.003), TUBE_W / 2, 0.006, WHITE),                          # the shoulder
            _box((-0.005, -0.009, SHOULDER_Z + 0.006), (0.005, 0.009, WALL_HI), WHITE),            # the core
            _box((-0.0015, -0.013, WALL_HI), (0.0015, 0.013, WALL_HI + 0.012), WHITE))             # the crimp
    x = TUBE_W / 2 - WALL_T / 2

    def wall(sign):
        return (_box((sign * x - WALL_T / 2, -WALL_Y / 2, WALL_LO), (sign * x + WALL_T / 2, WALL_Y / 2, WALL_HI), WHITE),
                _box((sign * (x + WALL_T / 2), -WALL_Y / 2 + 0.002, 0.020),                         # the label
                     (sign * (x + WALL_T / 2 + 0.0003), WALL_Y / 2 - 0.002, 0.045), RED, collide=False))
    zc = (WALL_LO + WALL_HI) / 2
    links = (Link("wall_a", TUBE_KG[1], wall(1), com=(x, 0.0, zc)),
             Link("wall_b", TUBE_KG[2], wall(-1), com=(-x, 0.0, zc)))
    pre = WALL_PRE / K_WALL
    joints = (Joint("wall_a", "prismatic", "wall_a", (x, 0.0, 0.0), "X", -SQUEEZE_MAX, 0.0, stiffness=K_WALL,
                    damping=5.0, target=pre),
              Joint("wall_b", "prismatic", "wall_b", (-x, 0.0, 0.0), "X", 0.0, SQUEEZE_MAX, stiffness=K_WALL,
                    damping=5.0, target=-pre))
    return Link("tube", TUBE_KG[0], base, com=(0.0, 0.0, 0.005)), links, joints


def bench_parts():
    """Fixed, in world x, y (z from the table): the cradle's two foam rails, the felt pad and the tube's holder."""
    s = []
    for sx in (-1, 1):
        x0, x1 = sorted((MX + sx * RAIL_IN, MX + sx * (RAIL_IN + RAIL_W)))
        s.append(_box((x0, MY0 - RAIL_L / 2, 0.0), (x1, MY0 + RAIL_L / 2, RAIL_H), FOAM, material="foam"))
    s.append(_box((PAD[0] - PAD_HALF, PAD[1] - PAD_HALF, 0.0), (PAD[0] + PAD_HALF, PAD[1] + PAD_HALF, PAD_H), FELT,
                  material="felt"))
    hx, hy, h, o = HOLD[0], HOLD[1], HOLD_HOLE / 2, 0.020
    s += [_box((hx - o, hy - o, 0.0), (hx + o, hy - h, HOLD_H), BLACK),
          _box((hx - o, hy + h, 0.0), (hx + o, hy + o, HOLD_H), BLACK),
          _box((hx - o, hy - h, 0.0), (hx - h, hy + h, HOLD_H), BLACK),
          _box((hx + h, hy - h, 0.0), (hx + o, hy + h, HOLD_H), BLACK)]
    base = Link("bench", 5.0, tuple(s))
    # WHY: a mechanism needs one moving link; this 1 mm one sits inside the felt pad, where nothing reaches it
    pin = Link("pin", 0.001, (_box((PAD[0] - 0.0005, PAD[1] - 0.0005, 0.0015), (PAD[0] + 0.0005, PAD[1] + 0.0005, 0.0025),
                                   DARK),))
    joint = Joint("pin", "prismatic", "pin", (PAD[0], PAD[1], 0.002), "Z", -0.0005, 0.0005, stiffness=1000.0,
                  damping=5.0)
    return base, (pin,), (joint,)


_MB, _ML, _MJ = mug_parts()
MUG = mechanism("mug", (MX, MY0, TOP + ZC), _MB, _ML, _MJ, MATERIALS, self_collide=False, free=True)
HANDLE_START = (PAD[0], PAD[1], TOP + PAD_H + 0.0005)
HANDLE = body("handle", HANDLE_START, handle_shapes(), MATERIALS, color=GLAZE)
_TB, _TL, _TJ = tube_parts()
TUBE_START = (HOLD[0], HOLD[1], TOP + HOLD_H - SHOULDER_Z + 0.0005)
TUBE = mechanism("tube", TUBE_START, _TB, _TL, _TJ, MATERIALS, self_collide=False, free=True)
_BB, _BL, _BJ = bench_parts()
BENCH = mechanism("bench", (0.0, 0.0, TOP), _BB, _BL, _BJ, MATERIALS, self_collide=False)
OBJECTS = [BENCH, MUG, HANDLE, TUBE]

# -- the layouts by seed -------------------------------------------------------------------------------------------------
# (handle dx, dy, yaw on the pad; the mug's shift along the cradle)
LAYOUTS = ((0.000, 0.000, 0.00, 0.000),
           (0.012, -0.010, 0.35, 0.020),
           (-0.010, 0.012, -0.30, -0.018))


def layout_index(seed):
    """Which layout a trial gets: inspect-robots' --seed 0, 2, 4 give 1, 0, 2 (README; test_mug_handle checks)."""
    return 0 if seed is None else int(seed) % 3


def layout(seed):
    dx, dy, yaw, mdy = LAYOUTS[layout_index(seed)]
    return {"handle": (HANDLE_START[0] + dx, HANDLE_START[1] + dy, HANDLE_START[2], yaw),
            "mug": (MX, MY0 + mdy, TOP + ZC, 0.0)}


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
    """Put a loose object (a rigid body or a free articulation) at a world pose, still."""
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
    """The trial's layout: the handle on its pad and the mug along its cradle, by seed (before the reset settles)."""
    lay = layout(run_seed())
    base = env.unwrapped
    for name in ("handle", "mug"):
        x, y, z, yaw = lay[name]
        _write_root(base, name, (x, y, z), yaw)


# -- geometry the model and the scorer share ----------------------------------------------------------------------------
def faces_w(mug_pos, mug_quat, stubs=(0.0, 0.0)):
    """The two broken faces' centres (world), each stub at its spring's travel (0 on its stop, negative pressed)."""
    R, p = _rot3(mug_quat), np.asarray(mug_pos, float)
    return [p + R @ np.array([0.0, y, R_MUG + STUB_H + s]) for y, s in zip(STUB_Y, stubs)]


def stub_force(q):
    """The press on a stub (N) from its travel: off its stop, its preload plus its spring; on its stop it tells nothing
    (counted 0)."""
    return STUB_PRE + K_STUB * -q if q < -5e-5 else 0.0


def ends_w(h_pos, h_quat):
    """The handle's two broken ends' centres (world): end 0 meets stub 0."""
    R, p = _rot3(h_quat), np.asarray(h_pos, float)
    return [p + R @ np.array([0.0, -ARCH_A, 0.0]), p + R @ np.array([0.0, ARCH_A, 0.0])]


def grip_w(h_pos, h_quat):
    """The handle's grip point: the top of the arch's section (world)."""
    return np.asarray(h_pos, float) + _rot3(h_quat) @ np.array([0.0, 0.0, ARCH_B])


def nozzle_w(t_pos, t_quat):
    return np.asarray(t_pos, float) + _rot3(t_quat) @ np.array([0.0, 0.0, NOZZLE_Z])


def seated(mug_pos, mug_quat, h_pos, h_quat, stubs=(0.0, 0.0)):
    """(end 0 seated, end 1 seated): each broken end's centre within SEAT_XY of its stub's face across and along the
    mug and every corner of the end within SEAT_Z of the face in height, with the handle's plane square to the mug (its
    y along the mug's axis, its z up the mug's)."""
    Rm, Rh = _rot3(mug_quat), _rot3(h_quat)
    along = float(Rm[:, 1] @ Rh[:, 1])
    square = abs(along) >= SEAT_TILT and float(Rm[:, 2] @ Rh[:, 2]) >= SEAT_TILT
    ends = ends_w(h_pos, h_quat)
    if along < 0:  # WHY: the modelled handle is symmetric, so turned end for end it fits the other way round
        ends = ends[::-1]
    out = []
    for f, e in zip(faces_w(mug_pos, mug_quat, stubs), ends):
        d = Rm.T @ (e - f)
        # WHY: a tilted end can sit flush at its centre and gape at an edge; a feeler gauge finds the edge, so every
        # corner of the end's 12 x 8 mm face must be within SEAT_Z of the face's plane
        cz = [float((Rm.T @ (e + Rh @ np.array([cx, cy, 0.0]) - f))[2])
              for cx in (-SEC_W / 2, SEC_W / 2) for cy in (-SEC_D / 2, SEC_D / 2)]
        out.append(bool(square and abs(d[0]) <= SEAT_XY and abs(d[1]) <= SEAT_XY
                        and SEAT_Z[0] <= min(cz) and max(cz) <= SEAT_Z[1]))
    return tuple(out)


def on_stubs(mug_pos, mug_quat, h_pos, h_quat, stubs=(0.0, 0.0)):
    """Both ends still on their stubs: each end's centre within SEAT_XY of its face across and along the mug and at
    most LIFT_OFF over it. Looser than seated, which an end must also be to join or to be let go."""
    Rm, Rh = _rot3(mug_quat), _rot3(h_quat)
    ends = ends_w(h_pos, h_quat)
    if float(Rm[:, 1] @ Rh[:, 1]) < 0:
        ends = ends[::-1]
    out = []
    for f, e in zip(faces_w(mug_pos, mug_quat, stubs), ends):
        d = Rm.T @ (e - f)
        out.append(bool(abs(d[0]) <= SEAT_XY and abs(d[1]) <= SEAT_XY and d[2] <= LIFT_OFF))
    return all(out)


def on_face(tip, mug_pos, mug_quat, stubs=(0.0, 0.0)):
    """Which broken face a drop from the nozzle's tip lands on (0 or 1), or None (astray). The tip may rest on the
    face (0.5 mm into it: PhysX's contact gap)."""
    Rm = _rot3(mug_quat)
    for k, f in enumerate(faces_w(mug_pos, mug_quat, stubs)):
        d = Rm.T @ (np.asarray(tip, float) - f)
        if (abs(d[0]) <= SEC_W / 2 - DEP_IN and abs(d[1]) <= SEC_D / 2 - DEP_IN and -0.0005 <= d[2] <= DEP_H):
            return k
    return None


def squeeze_force(q_a, q_b):
    """The squeeze on the tube (N, the two walls' mean): a wall off its stop pushes back with its preload plus its
    spring; a wall on its stop tells nothing (counted 0)."""
    f = [WALL_PRE + K_WALL * t if t > 5e-5 else 0.0 for t in (-q_a, q_b)]
    return 0.5 * (f[0] + f[1])


def flow_rate(force):
    """uL/s out of the nozzle: none under the gel's yield, then in proportion to the squeeze over it."""
    return FLOW_G * max(0.0, force - F_YIELD)


# -- the model -----------------------------------------------------------------------------------------------------------
def _fresh_model(seed=None):
    return {"glue": [0.0, 0.0], "stray": 0.0, "out": 0.0, "t": 0.0, "t_glue": None, "fitted": False, "parked": False,
            "glued_early": None, "order": None, "squeezed_out": [False, False], "joined": False, "press_t": 0.0, "pressed": False,
            "let_go": False, "ref": None, "moved": False, "seat": (False, False), "stubs": (0.0, 0.0),
            "tcp": None, "finger": None, "left": None, "steps": 0, "error": None, "layout": layout_index(seed)}


_M = _fresh_model()


def tube_in_holder(tube_pos, tube_quat):
    """The tube back in its holder: upright within 10 deg, its frame within HOME_XY across and HOME_Z in height of its
    start."""
    d = np.asarray(tube_pos, float) - np.asarray(TUBE_START)
    return bool(math.hypot(d[0], d[1]) <= HOME_XY and abs(d[2]) <= HOME_Z
                and float(_rot3(tube_quat)[2, 2]) >= math.cos(10 * math.pi / 180))


def on_pad(h_pos, h_quat):
    """The handle standing on its ends on the felt pad, upright within 10 deg."""
    p = np.asarray(h_pos, float)
    return bool(max(abs(p[0] - PAD[0]), abs(p[1] - PAD[1])) <= PAD_HALF - PARK_IN
                and abs(p[2] - HANDLE_START[2]) <= PARK_DZ and float(_rot3(h_quat)[2, 2]) >= math.cos(PARK_TILT))


def advance(m, dt, tube_pos, tube_quat, q_a, q_b, mug_pos, mug_quat, h_pos, h_quat, tcp, finger, stubs=(0.0, 0.0),
            left=None):
    """One step: glue leaves the nozzle at the squeeze's rate and lands on a broken face or astray; then the seating
    (the dry fit, the handle back on the pad, glue squeezed out from an over-full face, the joint made with the tube
    back, the press on both stubs, the handle let go and then left alone)."""
    m["t"] += dt
    m["tcp"], m["finger"] = (None if tcp is None else np.asarray(tcp, float)), finger
    m["left"] = None if left is None else (np.asarray(left[0], float), float(left[1]))
    m["stubs"] = tuple(float(q) for q in stubs)
    dv = flow_rate(squeeze_force(q_a, q_b)) * dt
    if dv > 0.0:
        k = on_face(nozzle_w(tube_pos, tube_quat), mug_pos, mug_quat, stubs) \
            if float(_rot3(tube_quat)[2, 2]) >= UPRIGHT else None
        if k is None:
            m["stray"] += dv
        else:
            m["glue"][k] += dv
        m["out"] += dv
        if m["t_glue"] is None:
            m["t_glue"] = m["t"]
            if not m["fitted"]:
                m["glued_early"] = "glue put on before the dry fit"
            elif not m["parked"]:
                m["glued_early"] = "glue put on before the handle was set back on the pad"
    s = seated(mug_pos, mug_quat, h_pos, h_quat, stubs)
    m["seat"] = s
    if all(s) and m["out"] == 0.0:
        m["fitted"] = True
    if (m["fitted"] and not m["parked"] and m["out"] == 0.0 and on_pad(h_pos, h_quat)
            and hand_clear_of(m, grip_w(h_pos, h_quat), SEC_W / 2)):
        m["parked"] = True
    for k in (0, 1):
        if s[k] and m["glue"][k] > V_HI:
            m["squeezed_out"][k] = True
    home = tube_in_holder(tube_pos, tube_quat)
    if all(s) and m["t_glue"] is not None and not m["joined"] and not home and not m["order"]:
        m["order"] = "handle seated on its glue before the tube was back in its holder"
    if m["joined"] and not m["let_go"] and not on_stubs(mug_pos, mug_quat, h_pos, h_quat, stubs):
        # WHY: the joint is made only while both ends are on their faces; lifted off before let go it is broken, and
        # must be seated and pressed again (a press does not carry over to a reseated handle). Lifted means an end more
        # than LIFT_OFF up, or its centre over SEAT_XY from its face's centre: released from a press, the stubs' springs
        # throw the handle up to 1.3 mm (run 9), which a rigid real stub would not, so that alone does not break it
        m["joined"], m["pressed"], m["press_t"] = False, False, 0.0
    if (all(s) and m["fitted"] and not m["joined"] and all(V_LO <= g <= V_HI for g in m["glue"])
            and m["t"] - m["t_glue"] <= OPEN_TIME and home):
        m["joined"] = True
    if m["joined"] and not m["pressed"]:
        on = all(s) and all(stub_force(q) >= F_PRESS for q in stubs)
        m["press_t"] = m["press_t"] + dt if on else 0.0
        if m["press_t"] >= PRESS_S - 1e-9:
            m["pressed"] = True
    Rm = _rot3(mug_quat)
    rel = Rm.T @ (np.asarray(h_pos, float) - np.asarray(mug_pos, float))   # the handle in the mug's frame
    if m["pressed"] and not m["let_go"] and all(s) and hand_clear_of(m, grip_w(h_pos, h_quat), SEC_W / 2):
        m["let_go"], m["ref"] = True, rel
    if m["let_go"] and (not all(s) or float(np.linalg.norm(rel - m["ref"])) > MOVED):
        m["moved"] = True


def _one_clear(tcp, finger, point, half_width):
    if tcp is None or finger is None:
        return False
    d = float(np.linalg.norm(tcp - np.asarray(point, float)))
    return bool(d >= FAR_CLEAR or (d >= HAND_CLEAR and -finger >= half_width + OPEN_CLEAR))


def hand_clear_of(m, point, half_width):
    """Both hands clear of ``point``: each TCP HAND_CLEAR from it with its fingers open (each finger OPEN_CLEAR outside
    a part half_width across), or FAR_CLEAR from it. The right hand unread counts as not clear; the left is read every
    sim step (the offline model may omit it)."""
    left_ok = m["left"] is None or _one_clear(m["left"][0], m["left"][1], point, half_width)
    return bool(left_ok and _one_clear(m["tcp"], m["finger"], point, half_width))


def bead_rise(glue, seat):
    """How far a bead stands up out of its stub (m): by the square root of its share of V_HI; under a seated end, a
    0.3 mm layer."""
    if glue <= 0.0:
        return 0.0
    if seat:
        return BEAD_DROP - BEAD_T + 0.0003
    return BEAD_DROP * min(1.0, math.sqrt(glue / V_HI))


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
    """A fresh trial: the model and the scorer. mechanism.py has put the walls and the beads back at 0."""
    reset_memory(run_seed())


@on_step
def run_model(env):
    """Read the tube, the mug and its stubs, the handle and both hands; advance the model; raise each bead by its
    glue."""
    try:
        import torch
        base = env.unwrapped
        tube, mug, han = base.scene["tube"], base.scene["mug"], base.scene["handle"]
        origin = _np(base.scene.env_origins)[0]
        pt, pm, ph = (_np(a.data.root_link_pose_w)[0] for a in (tube, mug, han))
        qt = _np(tube.data.joint_pos)[0]
        tid = _ids(tube, "tube", ("wall_a", "wall_b"))
        qm = _np(mug.data.joint_pos)[0]
        mid = _ids(mug, "mug", ("stub0", "stub1", "bead0", "bead1"))
        hands = []
        for side in ("right", "left"):
            rob = base.scene[f"{side}_robot"]
            g = _np(rob.data.body_link_pose_w)[0][list(rob.body_names).index("gripper")]
            qr = _np(rob.data.joint_pos)[0]
            hands.append((g[:3] - origin + _rot3(g[3:7]) @ np.array(TCP_OFFSET),
                          float(qr[list(rob.joint_names).index("joint7")])))
        stubs = (float(qm[mid["stub0"]]), float(qm[mid["stub1"]]))
        advance(_M, float(base.step_dt), pt[:3] - origin, pt[3:7], float(qt[tid["wall_a"]]), float(qt[tid["wall_b"]]),
                pm[:3] - origin, pm[3:7], ph[:3] - origin, ph[3:7], hands[0][0], hands[0][1], stubs=stubs,
                left=hands[1])
        tgt = torch.tensor([[bead_rise(gk, sk) for gk, sk in zip(_M["glue"], _M["seat"])]], dtype=torch.float32,
                           device=_torch_of(mug.data.joint_pos).device)
        j = [mid["bead0"], mid["bead1"]]
        if hasattr(mug, "set_joint_position_target_index"):
            mug.set_joint_position_target_index(target=tgt, joint_ids=j)
        else:
            mug.set_joint_position_target(tgt, joint_ids=j)
        _M["steps"] += 1
    except Exception as e:  # WHY: never stop the run; no model step means no glue and so no full score
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


def failure(objects, m):
    """Latched faults: glue before the dry fit (or before the handle is back on the pad), the handle seated on its
    glue before the tube is back, glue astray, glue squeezed out of a joint, the handle moved once let go on its glue,
    the mug moved or turned in its cradle, a part off the table, or the model not running."""
    if m["glued_early"]:
        return m["glued_early"]
    if m["order"]:
        return m["order"]
    if m["stray"] > STRAY_MAX:
        return f"{m['stray']:.1f} uL of glue off the broken faces"
    if any(m["squeezed_out"]):
        return "glue squeezed out of a joint onto the glaze"
    if m["moved"]:
        return "handle moved after it was let go on its glue"
    mug = objects["mug"]
    if float(np.linalg.norm(np.asarray(mug.pos[:2], float) - np.asarray(_mug_start(m)[:2]))) > IN_CRADLE:
        return "mug moved in its cradle"
    if float(_rot3(mug.quat_xyzw)[2, 2]) < MUG_UPRIGHT:
        return "mug turned in its cradle"
    for k in ("handle", "tube", "mug"):
        if float(objects[k].pos[2]) < LOST_Z:
            return f"{k} lost"
    if m["error"]:
        return "model " + m["error"]
    if _CLOCK["steps"] - m["steps"] > 5:
        return "model not stepping"
    return None


def _mug_start(m):
    return (MX, MY0 + LAYOUTS[m["layout"]][3], TOP + ZC)


def tube_home(objects, m):
    """The tube back in its holder and both hands clear of it."""
    t = objects["tube"]
    return bool(tube_in_holder(t.pos, t.quat_xyzw) and hand_clear_of(m, np.asarray(t.pos, float), TUBE_W / 2))


def stage(objects, m):
    """0 nothing; 1 dry-fitted; 2 and a thin bead on each face; 3 and the handle seated on both with the tube back; 4
    and pressed on both stubs, let go, still, the tube in its holder and both hands clear."""
    h = objects["handle"]
    if m["joined"] and all(m["seat"]) and tube_in_holder(objects["tube"].pos, objects["tube"].quat_xyzw):
        if (m["pressed"] and m["let_go"] and all(m["seat"]) and float(np.linalg.norm(np.asarray(h.lin_vel, float))) <= STILL_V
                and tube_home(objects, m) and hand_clear_of(m, grip_w(h.pos, h.quat_xyzw), SEC_W / 2)):
            return 4
        return 3
    if m["fitted"] and m["parked"] and all(V_LO <= g <= V_HI for g in m["glue"]):
        return 2
    return 1 if m["fitted"] else 0


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
    """Dry-fitted and set back on the pad, a thin bead on each face, the tube back in its holder, the handle seated
    flush on both, pressed on both and let go, both hands clear (held 1 s by yam_arena), with no latched fault."""
    return update(objects)["now"]["done"]


LADDER = {0: 0.0, 1: 0.2, 2: 0.4, 3: 0.6}
HOLD_S = 1.0


def score(objects):
    """At the last step: 0 after a latched fault; success held 1 s 1.0 (yam_arena also ends the trial then); all done
    but not yet held 1 s 0.8; seated on its glue with the tube back 0.6; both beads 0.4; dry-fitted 0.2."""
    m = update(objects)
    if m["fail"]:
        return 0.0
    n = m["now"]
    if n["stage"] == 4:
        return 1.0 if (n["done"] and m["held"] >= HOLD_S - 1e-9) else 0.8
    return LADDER[n["stage"]]


# -- demo ----------------------------------------------------------------------------------------------------------------
TIP = 0.013                      # PAD_TIP_PAST_TCP: the pad tips sit 13 mm under the TCP
OPEN_F = -0.040
K_FINGER = 5000.0                # yam_arena.constants GRIPPER stiffness (N/m per finger)
TCP_OVER_H = ARCH_B - SEC_D / 2 - 0.002 + TIP   # the TCP over the handle's frame: pad tips 2 mm under the section
GRIP_H = -(SEC_W / 2 - 0.0025)   # finger target on the handle: 2.5 mm into its 12 mm (12.5 N a finger), so the press
#                                  (up to 2 x 5 N) does not slip it
TCP_OVER_T = TIP - 0.005         # the TCP over the tube's frame: pad tips 5 mm under the walls' middle
HOLD_PEN = 0.0010                # the holding grip presses each wall in by this (under the gel's yield)
F_SQUEEZE = 8.0                  # N per wall while dispensing (10 uL/s)
V_AIM = 20.0                     # uL per face
NOZ_OVER = 0.0015                # the nozzle's tip over the face while dispensing
PRESS = 0.020                    # the seating TCP aims this far under contact. WHY so far: the arm's joints are PD
#                                  springs, about 0.5 N/mm at the TCP in z here [EST: the inverse of J K^-1 J^T from yam_arena's joint
#                                  stiffnesses, 0.49 to 0.54 N/mm on seeds 0, 2, 4]; aimed 1.6 mm under, it pushed under
#                                  the stubs' 2 x 1 N preload and neither moved (run 8). 20 mm gives about 4.6 N a stub
#                                  at 0.5 N/mm [EST], and a stiffer arm only takes the stubs to their stops (7.25 N)
DRY_PUSH = 0.006                 # the dry fit's check aims this far under contact (about 3 N in all [EST: the press's
#                                  0.5 N/mm]). WHY: held only, the handle sat 0.7 deg tilted, one end's corners 0.6 to 0.9
#                                  mm over its face, not seated (run 9, seed 0); a light push brings both ends down flush
EASE_S = 1.5                     # s to ease the press. WHY slow: eased in 0.4 s, the stubs' springs threw the handle 0.9
#                                  to 1.3 mm up, which broke the joint under run 9's rule (run 9, seeds 2 and 4)
RELEASE_PUSH = DRY_PUSH          # the press eases to this far under contact, and the hand lets go there. WHY: under the
#                                  press the handle slips 0.7 to 3.8 mm up in the grip (run 10); eased back to contact,
#                                  the hand lifted it up to 2.5 mm off its glue before let go. A light push keeps it down
CARRY = 0.020                    # the broken ends this far over the faces while carrying
HANDLE_DY = 0.0012               # the held handle sits 1.0 to 1.4 mm toward -y of the TCP [RUN: seed 0 traces], so the
#                                  demo aims its seat this far +y
HOLD_ENTRY = 0.028               # the tube stops with its nozzle tip 13 mm over the holder's rim, then goes in slowly (a
#                                  fast descent lags up to 8 mm sideways [RUN]; the well allows 5.5 mm)
HANDLE_SAG = 0.0                 # the held handle sits this much lower than planned [RUN: to be traced]
TUBE_SAG = 0.0                   # the held tube sits this much lower than planned [RUN: to be traced]
STEADY = 0.8                     # s held over a target before each descent: the arm lags 2 to 10 mm on a moving
#                                  descent [RUN], so it lands only after it has caught up
TUBE_ABOVE = 0.105               # the TCP this far over the tube's frame to pass it with the fingers open (tips 10 mm over
#                                  the crimp)
YAW0 = math.pi / 2               # fingers close along world x: across the handle (seated) and across the tube's walls


def finger_for_force(force):
    """The finger target that squeezes each wall with ``force`` (N): its travel against its spring and preload, plus
    the finger's own stiffness."""
    travel = max(0.0, (force - WALL_PRE) / K_WALL)
    return -(TUBE_W / 2 - travel - force / K_FINGER)


def wall_force_at(finger):
    """The squeeze a static finger target gives (the inverse of finger_for_force): 0 if it does not reach the wall."""
    pen = TUBE_W / 2 + finger
    if pen <= 0.0:
        return 0.0
    travel = max(0.0, (K_FINGER * pen - WALL_PRE) / (K_FINGER + K_WALL))
    f = K_FINGER * (pen - travel)
    return f if travel > 0.0 else 0.0


def _w(t, p, f, label, yaw):
    return Waypoint(float(t), tuple(float(v) for v in p), float(f), label, float(yaw))


class _Arm:
    def __init__(self):
        self.w, self.t = [], 0.0

    def add(self, dt, p, f, label, yaw):
        self.w.append(_w(dt, p, f, label, yaw))
        self.t += dt
        self.last = (tuple(p), f, yaw)


def dose(ws, dt=0.01):
    """uL a plan squeezes out (the finger target ramping linearly along each waypoint, the wall following it)."""
    v, prev = 0.0, OPEN_F
    for w in ws:
        n = max(1, int(round(w.duration / dt)))
        for i in range(n):
            f = prev + (w.finger - prev) * (i + 1) / n
            v += flow_rate(wall_force_at(f)) * (w.duration / n)
        prev = w.finger
    return v


def plan(delay=0.0, seed=None):
    """The right arm: dry-fit the handle and set it back on the pad square to the mug; take the tube, put a bead on
    each broken face, put the tube back; then seat the handle on its glue, press it, let go and draw back."""
    lay = layout(seed)
    hx, hy, hz, hyaw = lay["handle"]
    mx, my, mz, _ = lay["mug"]
    A = _Arm()
    fz = TOP + Z_FACE + (mz - (TOP + ZC))              # the faces' height
    seat = (mx, my + HANDLE_DY, fz + TCP_OVER_H - HANDLE_SAG)   # the TCP with the handle's ends on the faces
    over_seat = (seat[0], seat[1], seat[2] + CARRY)
    # 1. the dry fit: pick the handle where it stands, turn it square to the mug, seat it, lift it, set it on the pad
    g_pick = (hx, hy, hz + TCP_OVER_H)
    yaw_pick = hyaw + YAW0
    A.add(2.0 + delay, (g_pick[0], g_pick[1], g_pick[2] + 0.05), OPEN_F, "over the handle", yaw_pick)
    A.add(STEADY, (g_pick[0], g_pick[1], g_pick[2] + 0.05), OPEN_F, "steady over the handle", yaw_pick)
    A.add(1.5, g_pick, OPEN_F, "down to the handle", yaw_pick)
    A.add(0.6, g_pick, GRIP_H, "grip the handle", yaw_pick)
    A.add(0.8, g_pick, GRIP_H, "settle", yaw_pick)
    A.add(1.0, (g_pick[0], g_pick[1], over_seat[2] + 0.02), GRIP_H, "lift the handle", yaw_pick)
    A.add(2.0, (over_seat[0], over_seat[1], over_seat[2] + 0.02), GRIP_H, "over the stubs, square", YAW0)
    A.add(1.0, over_seat, GRIP_H, "ends over the faces", YAW0)
    A.add(STEADY, over_seat, GRIP_H, "steady over the faces", YAW0)
    A.add(1.5, seat, GRIP_H, "dry fit: both ends on their faces", YAW0)
    A.add(1.6, (seat[0], seat[1], seat[2] - DRY_PUSH), GRIP_H, "check the fit: a light push down", YAW0)
    A.add(1.0, over_seat, GRIP_H, "lift it off", YAW0)
    A.add(0.6, (over_seat[0], over_seat[1], over_seat[2] + 0.02), GRIP_H, "up clear of the mug", YAW0)
    pad_g = (PAD[0], PAD[1], HANDLE_START[2] + TCP_OVER_H)
    A.add(2.0, (pad_g[0], pad_g[1], over_seat[2] + 0.02), GRIP_H, "back over the pad", YAW0)
    A.add(1.0, (pad_g[0], pad_g[1], pad_g[2] + 0.03), GRIP_H, "down over the pad", YAW0)
    A.add(STEADY, (pad_g[0], pad_g[1], pad_g[2] + 0.03), GRIP_H, "steady over the pad", YAW0)
    A.add(1.5, (pad_g[0], pad_g[1], pad_g[2] + 0.001), GRIP_H, "set it down", YAW0)
    A.add(0.6, (pad_g[0], pad_g[1], pad_g[2] + 0.001), GRIP_H, "settle on the pad", YAW0)
    A.add(0.6, (pad_g[0], pad_g[1], pad_g[2] + 0.001), OPEN_F, "let go", YAW0)
    # WHY: the open fingers pass the tube only with their tips over its top (TUBE_ABOVE), or they sweep it over
    over_t = TUBE_START[2] + TCP_OVER_T + TUBE_ABOVE
    A.add(1.2, (pad_g[0], pad_g[1], over_t), OPEN_F, "up", YAW0)
    # 2. the glue: take the tube, a bead on each face, the tube back
    t_g = (TUBE_START[0], TUBE_START[1], TUBE_START[2] - 0.0005 + TCP_OVER_T)
    hold_f = -(TUBE_W / 2 - HOLD_PEN)
    sq_f = finger_for_force(F_SQUEEZE)
    A.add(1.5, (t_g[0], t_g[1], over_t), OPEN_F, "over the tube", YAW0)
    A.add(STEADY, (t_g[0], t_g[1], over_t), OPEN_F, "steady over the tube", YAW0)
    A.add(1.5, t_g, OPEN_F, "down to the tube", YAW0)
    A.add(0.6, t_g, hold_f, "grip the tube (no squeeze)", YAW0)
    A.add(0.4, t_g, hold_f, "settle", YAW0)
    # WHY: carry the tube with its nozzle 20 mm over the faces' height, so it never sweeps the mug's side
    lift = max(t_g[2] + HOLD_H + 0.010, fz + NOZ_OVER - NOZZLE_Z + TCP_OVER_T - TUBE_SAG + 0.020)
    A.add(1.2, (t_g[0], t_g[1], lift), hold_f, "lift it out of the holder", YAW0)
    rate = flow_rate(wall_force_at(sq_f))
    for k in (0, 1):
        f = (mx, my + STUB_Y[k], fz)
        tip = (f[0], f[1], f[2] + NOZ_OVER - NOZZLE_Z + TCP_OVER_T - TUBE_SAG)
        A.add(1.5, (tip[0], tip[1], tip[2] + 0.020), hold_f, f"nozzle over face {k}", YAW0)
        A.add(STEADY, (tip[0], tip[1], tip[2] + 0.020), hold_f, f"steady over face {k}", YAW0)
        A.add(1.0, tip, hold_f, f"nozzle down to face {k}", YAW0)
        ramp = 0.3
        ramp_v = dose([_w(ramp, tip, sq_f, "", YAW0)])   # from the hold, about the same as from open
        held = max(0.0, (V_AIM - 2 * ramp_v) / rate)
        A.add(ramp, tip, sq_f, f"squeeze a bead onto face {k}", YAW0)
        A.add(held, tip, sq_f, "keep squeezing", YAW0)
        A.add(ramp, tip, hold_f, "ease off", YAW0)
        A.add(0.4, tip, hold_f, "let the bead settle", YAW0)
        A.add(0.8, (tip[0], tip[1], tip[2] + 0.020), hold_f, f"nozzle up from face {k}", YAW0)
    A.add(2.0, (t_g[0], t_g[1], lift), hold_f, "back over the holder", YAW0)
    A.add(STEADY + 0.4, (t_g[0], t_g[1], lift), hold_f, "steady over the holder", YAW0)
    entry = t_g[2] + HOLD_ENTRY
    A.add(1.5, (t_g[0], t_g[1], entry), hold_f, "down to the holder's mouth", YAW0)
    A.add(STEADY, (t_g[0], t_g[1], entry), hold_f, "steady over the mouth", YAW0)
    A.add(2.5, (t_g[0], t_g[1], t_g[2] + 0.001), hold_f, "nozzle into the holder", YAW0)
    A.add(0.6, (t_g[0], t_g[1], t_g[2] + 0.001), hold_f, "settle in the holder", YAW0)
    A.add(0.6, (t_g[0], t_g[1], t_g[2] + 0.001), OPEN_F, "let go of the tube", YAW0)
    A.add(1.5, (t_g[0], t_g[1], over_t), OPEN_F, "up clear of the tube", YAW0)
    # 3. seat the handle on its glue, press it, let go
    A.add(1.8, (pad_g[0], pad_g[1], over_t), OPEN_F, "over the handle", YAW0)   # level: never under the tube's top
    A.add(1.0, (pad_g[0], pad_g[1], pad_g[2] + 0.04), OPEN_F, "down over the handle", YAW0)
    A.add(STEADY, (pad_g[0], pad_g[1], pad_g[2] + 0.04), OPEN_F, "steady over the handle", YAW0)
    A.add(1.5, pad_g, OPEN_F, "down to the handle", YAW0)
    A.add(0.6, pad_g, GRIP_H, "grip the handle", YAW0)
    A.add(0.8, pad_g, GRIP_H, "settle", YAW0)
    A.add(1.0, (pad_g[0], pad_g[1], over_seat[2] + 0.02), GRIP_H, "lift the handle", YAW0)
    A.add(2.0, (over_seat[0], over_seat[1], over_seat[2] + 0.02), GRIP_H, "over the glued faces", YAW0)
    A.add(1.0, over_seat, GRIP_H, "ends over the glued faces", YAW0)
    A.add(STEADY, over_seat, GRIP_H, "steady over the glued faces", YAW0)
    A.add(1.5, seat, GRIP_H, "seat both ends", YAW0)
    A.add(1.2, seat, GRIP_H, "let it settle on the faces", YAW0)
    press = (seat[0], seat[1], seat[2] - PRESS)
    A.add(1.0, press, GRIP_H, "press it down", YAW0)
    A.add(1.5, press, GRIP_H, "hold the press", YAW0)
    light = (seat[0], seat[1], seat[2] - RELEASE_PUSH)
    A.add(EASE_S, light, GRIP_H, "ease the press to a light push", YAW0)
    A.add(0.8, light, OPEN_F, "let go", YAW0)
    A.add(1.2, (seat[0], seat[1], seat[2] + 0.045), OPEN_F, "draw back up", YAW0)
    A.add(4.0, (seat[0] - 0.04, seat[1] - 0.04, seat[2] + 0.06), OPEN_F, "clear of the mug; leave it", YAW0)
    return A.w, {"dose_aim": V_AIM}


def demo(delay=0.0):
    """The right arm does it all; the left stays at its ready pose (either arm may be used: both are scored)."""
    right, _ = plan(delay, run_seed())
    return {"right": right}


def _replan_demo_after_reset():
    """WHY: inspect-robots resets the policy before the embodiment (inspect_robots/rollout.py), so yam_scripted would
    call demo() while run_seed() still holds the previous trial's seed and plan for the wrong layout. This makes
    yam_scripted plan again on each trial's first action, after the reset has set the seed: the scripted demo reads
    the seed (where the handle and the mug were put), as a privileged script may. It changes only the scripted demo
    policy, never the scene or the score."""
    try:
        from yam_arena import scripted
    except ImportError:
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_mug_replan", False):
        return
    reset, act = cls.reset, cls.act
    unset = object()

    def patched_reset(self, scene):
        self._mug_scene, self._mug_seed = scene, unset
        reset(self, scene)

    def patched_act(self, observation):
        if getattr(self, "_mug_seed", None) is unset:
            self._mug_seed = run_seed()
            reset(self, self._mug_scene)
        return act(self, observation)

    cls.reset, cls.act, cls._mug_replan = patched_reset, patched_act, True


_replan_demo_after_reset()
