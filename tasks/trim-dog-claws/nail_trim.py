"""A dog-paw trimming trainer stands on the bench: a silicone foreleg clamped to its stand, the paw turned sole up
with its four long black claws pointing up. A guillotine nail clipper lies on a mat. Trim all four claws with the
clipper: bring its ring down over the tip of a claw, square to the claw (the blade on the side away from the toe), and
squeeze the handles to cut. Take off a little at a time (1 to 2 mm a cut) and look at the cut face after each cut: the
claw is short enough when a small pink-grey dot shows in the middle of the black cut face. Stop cutting that claw then; cutting further
cuts into the quick and the claw bleeds. The dots are at a different length in each claw. When all four claws show
their dots, put the clipper back on its mat and let go of it.
"""
import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, hull, mechanism, on_reset, on_step, run_seed  # noqa: E402
from yam_arena import station  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.constants import GRIPPER  # noqa: E402

# WHY: the round 2 review budgets "the approximate 40 N available per YAM finger"; yam_arena caps each finger at
# i2rt's 50 N and has no per-task setting. robot.yam_arm_cfg() reads GRIPPER["effort"] when the env is built, which is
# after the task file is imported, so the run's fingers push at most GRIP_N.
GRIP_N = 40.0
GRIPPER["effort"] = GRIP_N

SCENE = "station"
RUN_LIMIT_STEPS = 3000           # 150 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- where the trainer stands -----------------------------------------------------------------------------------------
# The trainer's numbers below are in its own frame (x along the leg from the paw to the stand, z up from the table top).
# It stands turned 90 degrees, the leg coming from the left arm's side (+y) toward the middle, the paw in front of the
# right arm.
T_POS = (0.300, -0.250)          # the trainer frame's origin on the table (world x, y)
T_YAW = math.pi / 2              # its x axis along world +y


def to_world(lx, ly):
    """A point in the trainer's frame (x, y) on the table (world x, y)."""
    c, s = math.cos(T_YAW), math.sin(T_YAW)
    return (T_POS[0] + c * lx - s * ly, T_POS[1] + s * lx + c * ly)


def to_local(wx, wy):
    """A point on the table (world x, y) in the trainer's frame."""
    c, s = math.cos(T_YAW), math.sin(T_YAW)
    dx, dy = wx - T_POS[0], wy - T_POS[1]
    return (c * dx + s * dy, -s * dx + c * dy)


T_QUAT = (0.0, 0.0, math.sin(T_YAW / 2), math.cos(T_YAW / 2))   # x, y, z, w

# -- the claws and the quick -----------------------------------------------------------------------------------------
CLAW_X = 0.245                   # the claws' axes (trainer frame)
CLAW_Y = (-0.021, -0.007, 0.007, 0.021)   # toe pitch 14 mm [EST: a medium dog's forepaw about 55 mm across]
Z_TIP = 0.115                    # the claws' tips over the table at rest (they point up)
Z_TOE = 0.097                    # where the claws leave the toes: 18 mm of claw shows [EST: long claws]
CLAW_W = (0.0036, 0.0044)        # the claw's end, y by x (trainer frame) [EST]
CLAW_ROOT = (0.0050, 0.0060)     # the claw where it leaves the toe [EST]
# The quick's end below each claw's tip at the start (mm), drawn for each claw on its own from the seed. [CHOICE: 4.5 to
# 8.5 mm in 0.5 mm steps, spread round the 5 to 8 mm the clinic took off (Hausler and Sohnel 2026, "between 5 mm and 8
# mm was removed"; the paper also says the proportion removed was not measured)]
QUICK_MM = tuple(4.5 + 0.5 * i for i in range(9))
DOT = 0.002                      # the oval shows once the cut face is within 2 mm of the quick [EST: WSU says the oval
#                                  "starts to appear" before the quick and "It's best to not cut within 2 millimeters of
#                                  the quick"; no source gives the distance at which it shows]
CUT_MAX = 0.012                  # the most the tip slider can rise (more than any quick)

# -- the trainer: a stand with the leg and paw clamped to it (Erler-Zimmer VET4750 is a silicone leg on a plexiglass
# stand; the maker's FAQ: the paw is "mounted on a stable base"). WHY rigid: v1 sprang the leg and had the other hand
# hold it, but with no cutting force nothing pushed the leg, so the hold was decorative (review 2026-11-15, finding 1).
PIVOT = (0.410, 0.0, 0.085)      # where the leg leaves the stand
LEG_R = 0.017
LEG_X = (0.268, 0.410)           # the leg's span along x
PAW = ((0.226, 0.264), (-0.031, 0.031), (0.070, Z_TOE))   # the paw block (sole up: toes and pads), x, y, z spans

# -- the clipper (guillotine): its frame at the grip point between the handles, x toward the head ------------------------
RING_X = 0.055                   # the ring's centre ahead of the grip point (the hand holds the handles near the head,
#                                  so the right wrist camera sees through the ring) [WHY: at 80 mm the ring was out of
#                                  its view, 2026-11-15 film]
RING_IN = 0.0045                 # the ring's hole radius: 9 mm opening [RETAIL: ozfarmer listing of a Resco guillotine,
#                                  "The guillotine opening diameter is 9mm." (src/resco)]
RING_OUT = 0.0085
RING_N = 16                      # segments in the ring's collider; their inner chords stand RING_IN cos(pi/16) off its
#                                  centre (4.41 mm)
IN_RING_R = RING_IN * math.cos(math.pi / RING_N) - math.hypot(*CLAW_W) / 2   # a claw end (its corners 2.84 mm out)
#                                  this near the hole's centre fits inside the collider: 1.57 mm
RING_T = 0.004                   # the head's thickness (z), the ring from -2 to +2 mm in the clipper's frame
Z_BLADE = RING_T / 2             # the blade slides over the ring's top face (away from the toe): the cut plane
HANDLE_Y = 0.016                 # each handle's centre line off the grip point
HANDLE_W, HANDLE_H = 0.010, 0.012
HANDLE_X = (-0.070, 0.015)
SQUEEZE_EACH = 0.006             # each handle's travel to its stop (the handles 12 mm closer, squeezed fully)
Q_CUT = 0.0095                   # the blade has crossed the 9 mm hole
Q_OPEN = 0.002                   # the blade back clear of the hole (it must reopen between cuts)
SPRING_PRE, SPRING_K = 20.0, 500.0   # N and N/m: each handle's return spring [EST]
CLIP_KG = (0.050, 0.018, 0.018, 0.004)   # head and spine, handle A, handle B, blade [EST: a steel guillotine ~90 g]
MAT = (0.170, -0.160)            # the clipper's mat (its grip point rests over it)
MAT_H = 0.020
MAT_HALF = (0.050, 0.026)        # WHY: both handles (y +-11 to +-21 mm) lie on it with 5 mm to spare; at 30 mm wide a
#                                  clipper put back 5 mm off in y rolled 11 deg off the mat's edge (run 5 traces)
CLIP_START = (MAT[0], MAT[1], TOP + MAT_H + HANDLE_H / 2 + 0.0005)

MATERIALS = {"default": (1.2, 1.0),   # the scene's friction on its own assets (rules.md 3)
             "steel": (0.6, 0.5),
             "silicone": (1.0, 0.9),   # [EST]
             "rubber": (1.0, 0.9),     # the clipper's handle grips [EST]
             "claw": (0.5, 0.4)}       # [EST]
STEEL, BLACK, WHITE = (0.55, 0.56, 0.58), (0.06, 0.06, 0.07), (0.92, 0.93, 0.95)
BROWN, CLEAR, RED = (0.32, 0.18, 0.12), (0.80, 0.86, 0.90), (0.70, 0.04, 0.04)
DOTC = (0.95, 0.70, 0.70)        # the oval: WSU, "a gray to pink oval starts to appear" (the light end of that range)
GRIP_RED = (0.75, 0.12, 0.10)


def _box(lo, hi, color, material="steel", collide=True):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Shape("box", tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, material=material,
                 collide=collide)


def _cyl(c, r, h, color, axis="Z", material="steel", collide=True):
    return Shape("cylinder", tuple(float(v) for v in c), (float(r), float(h)), color, axis=axis, material=material,
                 collide=collide)


def _frustum(cx, cy, z0, w0, z1, w1, color, material="claw", collide=True):
    """A hull from a w0 (y, x) rectangle at z0 to a w1 rectangle at z1, centred on (cx, cy)."""
    def ring(z, w):
        return [(cx - w[1] / 2, cy - w[0] / 2, z), (cx + w[1] / 2, cy - w[0] / 2, z),
                (cx + w[1] / 2, cy + w[0] / 2, z), (cx - w[1] / 2, cy + w[0] / 2, z)]
    return hull(ring(z0, w0) + ring(z1, w1), color=color, material=material)


def _hidden(cx, cy, z):
    """A 1 mm collider inside the paw (nothing reaches it), so a drawn-only part is a proper rigid body."""
    return _box((cx - 0.0005, cy - 0.0005, z), (cx + 0.0005, cy + 0.0005, z + 0.001), BLACK, material="claw")


def trainer_base():
    """The stand: a plexiglass foot on the table and an inclined back the leg comes out of; and the clipper's mat (its
    place given on the table, square to the table's axes)."""
    mat = [to_local(MAT[0] + dx, MAT[1] + dy) for dx, dy in ((-MAT_HALF[0], -MAT_HALF[1]), (MAT_HALF[0], -MAT_HALF[1]),
                                                             (MAT_HALF[0], MAT_HALF[1]), (-MAT_HALF[0], MAT_HALF[1]))]
    s = [_box((0.400, -0.060, 0.0), (0.510, 0.060, 0.006), WHITE),
         hull([(0.470, -0.060, 0.006), (0.480, -0.060, 0.006), (0.480, 0.060, 0.006), (0.470, 0.060, 0.006),
               (0.410, -0.060, 0.160), (0.420, -0.060, 0.160), (0.420, 0.060, 0.160), (0.410, 0.060, 0.160)],
              color=WHITE, material="steel"),
         hull([(x, y, 0.0) for x, y in mat] + [(x, y, MAT_H) for x, y in mat], color=BLACK, material="rubber")]
    return Link("stand", 3.0, tuple(s + paw_shapes()))


def paw_shapes():
    """The leg (from the stand to the paw) and the paw block, with the claws' roots (fixed in the toes)."""
    s = [_cyl(((LEG_X[0] + LEG_X[1]) / 2, 0.0, PIVOT[2]), LEG_R, LEG_X[1] - LEG_X[0], BROWN, axis="X",
              material="silicone"),
         _box((PAW[0][0], PAW[1][0], PAW[2][0]), (PAW[0][1], PAW[1][1], PAW[2][1]), BROWN, material="silicone")]
    for y in CLAW_Y:
        s.append(_frustum(CLAW_X, y, Z_TOE - 0.001, CLAW_ROOT, Z_TOE + 0.004, CLAW_ROOT, BLACK))
    return s


def claw_links(k):
    """Claw k's tip (the part that can be cut, sliding down into the toe as it is cut), its dark dot and its blood drop
    (both hidden inside the tip until they show, then on its cut face)."""
    y = CLAW_Y[k]
    tip = Link(f"tip{k}", 0.002, (_frustum(CLAW_X, y, Z_TOE + 0.003, CLAW_ROOT, Z_TIP, CLAW_W, BLACK),))
    dot = Link(f"dot{k}", 0.0005, (_cyl((CLAW_X, y, Z_TIP - 0.0030), DOT_R, 0.0004, DOTC, collide=False),
                                   _hidden(CLAW_X, y, Z_TOE - 0.008)))
    blood = Link(f"blood{k}", 0.0005, (_cyl((CLAW_X, y, Z_TIP - 0.0040), 0.0016, 0.0030, RED, collide=False),
                                       _hidden(CLAW_X, y, Z_TOE - 0.008)))
    joints = (Joint(f"tip{k}", "prismatic", f"tip{k}", (CLAW_X, y, Z_TIP), "Z", -CUT_MAX - 0.0005, 0.0005,
                    stiffness=2000.0, damping=20.0),
              Joint(f"dot{k}", "prismatic", f"dot{k}", (CLAW_X, y, Z_TIP), "Z", -0.0005, DOT_DROP + 0.0005,
                    stiffness=200.0, damping=4.0, parent=f"tip{k}"),
              Joint(f"blood{k}", "prismatic", f"blood{k}", (CLAW_X, y, Z_TIP), "Z", -0.0005, BLOOD_DROP + 0.0005,
                    stiffness=200.0, damping=4.0, parent=f"tip{k}"))
    return (tip, dot, blood), joints


DOT_R = 0.0016                   # the oval's radius on the 3.6 by 4.4 mm cut face [EST]. WHY 1.6 mm: at 1.2 mm it was
#                                  2 to 3 px in the policy's 224 px wrist view (review 2026-11-15, finding 6); 1.6 mm is
#                                  as large as the face allows (it stays 0.3 mm inside the tip's side until it shows)
DOT_DROP = 0.0031                # the dot rises from 3 mm inside the tip to its cut face (0.25 mm proud of it)
BLOOD_DROP = 0.0065              # the drop rises from inside the tip to sit on the cut face


def trainer_parts():
    """Each claw's tip, dot and drop, hanging from the stand (the leg and paw are part of it)."""
    links, joints = [], []
    for k in range(4):
        ls, js = claw_links(k)
        links += ls
        joints += js
    return tuple(links), tuple(joints)


def clipper_parts():
    """The head (the ring and the neck) on a short spine between the handles; handle A (+y) and handle B (-y) slide
    toward each other as they are squeezed, each up to SQUEEZE_EACH, so a centred squeeze leaves the ring where it is;
    the blade (drawn only) slides over the ring's top face by the two handles' travel together, set by the model each
    step as the clipper's linkage would."""
    hz = HANDLE_H / 2
    base = [_box((-0.070, -0.004, -0.003), (0.020, 0.004, 0.003), STEEL),       # the spine between the handles
            hull([(0.015, -0.004, -0.002), (0.015, 0.004, -0.002), (RING_X - 0.006, 0.0085, -0.002),
                  (RING_X - 0.006, -0.0085, -0.002), (0.015, -0.004, 0.002), (0.015, 0.004, 0.002),
                  (RING_X - 0.006, 0.0085, 0.002), (RING_X - 0.006, -0.0085, 0.002)], color=STEEL, material="steel")]
    n = RING_N   # the ring as n segments round the 9 mm hole
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        pts = [(RING_X + r * math.cos(a), r * math.sin(a)) for r, a in
               ((RING_IN, a0), (RING_OUT, a0), (RING_OUT, a1), (RING_IN, a1))]
        base.append(hull([(x, y, -RING_T / 2) for x, y in pts] + [(x, y, RING_T / 2) for x, y in pts],
                         color=STEEL, material="steel"))

    def handle(sign):
        return _box((HANDLE_X[0], sign * HANDLE_Y - HANDLE_W / 2, -hz), (HANDLE_X[1], sign * HANDLE_Y + HANDLE_W / 2, hz),
                    GRIP_RED, material="rubber")
    blade_shape = _box((RING_X - 0.010, -0.020, Z_BLADE), (RING_X + 0.010, -RING_IN - 0.0005, Z_BLADE + 0.0012),
                       (0.80, 0.81, 0.83), collide=False)   # its edge 0.5 mm outside the hole at rest
    links = (Link("handle_a", CLIP_KG[1], (handle(1),), com=(0.0, HANDLE_Y, 0.0)),
             Link("handle_b", CLIP_KG[2], (handle(-1),), com=(0.0, -HANDLE_Y, 0.0)),
             Link("blade", CLIP_KG[3], (blade_shape, _box((-0.0005, -0.0005, -0.0005), (0.0005, 0.0005, 0.0005),
                                                         STEEL)), com=(0.0, 0.0, 0.0)))
    pre = SPRING_PRE / SPRING_K
    joints = (Joint("handle_a", "prismatic", "handle_a", (0.0, HANDLE_Y, 0.0), "Y", -SQUEEZE_EACH, 0.0,
                    stiffness=SPRING_K, damping=2.0, target=pre),
              Joint("handle_b", "prismatic", "handle_b", (0.0, -HANDLE_Y, 0.0), "Y", 0.0, SQUEEZE_EACH,
                    stiffness=SPRING_K, damping=2.0, target=-pre),
              Joint("blade", "prismatic", "blade", (RING_X, 0.0, Z_BLADE), "Y", -0.0005, 2 * SQUEEZE_EACH + 0.0005,
                    stiffness=50.0, damping=0.5))
    return Link("body", CLIP_KG[0], tuple(base), com=(0.0, 0.0, 0.0)), links, joints


C_JOINTS = ("handle_a", "handle_b", "blade")


def blade_travel(q_a, q_b):
    """The blade's travel over the ring: the two handles' travel toward each other together (q_a <= 0, q_b >= 0)."""
    return max(0.0, q_b - q_a)


_T_LINKS, _T_JOINTS = trainer_parts()
TRAINER = mechanism("trainer", (T_POS[0], T_POS[1], TOP), trainer_base(), _T_LINKS, _T_JOINTS, MATERIALS, yaw=T_YAW,
                    self_collide=False)
_C_BASE, _C_LINKS, _C_JOINTS = clipper_parts()
CLIPPER = mechanism("clipper", CLIP_START, _C_BASE, _C_LINKS, _C_JOINTS, MATERIALS, self_collide=False, free=True)
OBJECTS = [TRAINER, CLIPPER]
T_JOINTS = tuple(j.name for j in _T_JOINTS)


# -- the model ---------------------------------------------------------------------------------------------------------
def quicks(seed):
    """Each claw's quick below its tip at the start (mm), drawn on its own from the seed."""
    rng = random.Random(0 if seed is None else int(seed))
    return tuple(rng.choice(QUICK_MM) for _ in range(4))


def _fresh_model(seed=None):
    return {"quick": tuple(q / 1000.0 for q in quicks(seed)), "removed": [0.0] * 4,
            "cuts": [0] * 4, "bled": [False] * 4, "armed": True, "error": None, "steps": 0, "log": [],
            "hands": None, "resist": 0.0, "felt": set()}


_M = _fresh_model()


def _rot3(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def claw_tips(paw_pos, paw_quat, removed):
    """World position of each claw's cut face (its tip at the start, lowered along the claw by what is cut), carried
    by the paw link (its frame is the trainer's frame at rest: at T_POS on the table turned by T_YAW, z from the table
    top)."""
    R, p = _rot3(paw_quat), np.asarray(paw_pos, float)
    return [p + R @ np.array([CLAW_X, CLAW_Y[k], Z_TIP - removed[k]]) for k in range(4)]


UPRIGHT = math.cos(math.radians(15.0))   # the clipper's z within 15 deg of world up: its ring square to the upright
#                                         claws (WSU: "the clipper perpendicular to the nail")


def in_ring(tip_w, clip_pos, clip_quat):
    """(inside, protrusion): whether the claw's end is in the ring's hole (within IN_RING_R of the hole's centre, in
    the clipper's frame: it fits inside the ring's collider) with the clipper upright and square to the claw, and how
    far the end stands above the blade's plane (the ring's top face). WHY the upright test: in v1 a clipper turned
    upside down over a claw "cut" with nothing through its ring, and a tilted one cut on a slant (review finding 5)."""
    R = _rot3(clip_quat)
    c = R.T @ (np.asarray(tip_w, float) - np.asarray(clip_pos, float))
    r = math.hypot(c[0] - RING_X, c[1])
    return bool(r <= IN_RING_R and R[2, 2] >= UPRIGHT), float(c[2] - Z_BLADE)


def advance(m, blade_q, tips, clip_pos, clip_quat):
    """One step: a squeeze that carries the blade across the hole (from open) cuts off, at the blade's plane, every claw
    end standing up through the ring, if the claw pushed back (R_CUT) on an earlier step of the same squeeze. A claw
    cut to or past its quick bleeds. Then it sets this step's push-back, m["resist"], for run_model to apply.
    WHY the "felt" latch: the model steps at the control rate, so a snap squeeze could carry the blade across the
    claw between two steps and cut without ever meeting its resistance (review 2026-11-17, finding 1)."""
    if blade_q <= Q_OPEN:
        m["armed"], m["felt"] = True, set()
    if m["armed"] and blade_q >= Q_CUT:
        m["armed"] = False
        for k, tip in ((k, t) for k, t in enumerate(tips) if k in m.get("felt", ())):
            inside, below = in_ring(tip, clip_pos, clip_quat)
            if inside and below > 0.0:
                cut = min(below, CUT_MAX - m["removed"][k])
                m["removed"][k] += cut
                m["cuts"][k] += 1
                m["log"].append((k, round(cut * 1000, 2)))
                if m["removed"][k] >= m["quick"][k] - 1e-9:
                    m["bled"][k] = True
    m["resist"], k = resisting(m, blade_q, tips, clip_pos, clip_quat)
    if k is not None:
        m.setdefault("felt", set()).add(k)   # WHY per claw: a half-closed blade carried to another claw must meet it too


R_CUT = 8.0                      # N: each handle's extra push-back while the blade is through a claw [EST: no measured
#                                  figure found for cutting a dog's claw; the calibration measures it, see README].
#                                  WHY: without it a squeeze needs only the return springs (the flare task's review,
#                                  #2108, rejected handles that moved without the force the real tool needs)
CLAW_HALF = math.hypot(*CLAW_W) / 2   # the claw end's half width at its corners, across the blade's path
BLADE_EDGE0 = -(RING_IN + 0.0005)     # the blade's edge, clipper y, at rest (0.5 mm outside the hole)


def resisting(m, blade_q, tips, clip_pos, clip_quat):
    """(R_CUT, the claw's index) while a squeeze from open has the blade's edge inside the end of a claw standing up
    through the ring, else (0, None). advance() calls it after its cut check, so a finished cut (disarmed) no longer resists."""
    if not m.get("armed"):
        return 0.0, None
    R = _rot3(clip_quat)
    edge = BLADE_EDGE0 + blade_q
    for k, tip in enumerate(tips):
        inside, above = in_ring(tip, clip_pos, clip_quat)
        if inside and above > 0.0:
            cy = float((R.T @ (np.asarray(tip, float) - np.asarray(clip_pos, float)))[1])
            if cy - CLAW_HALF <= edge <= cy + CLAW_HALF:
                return R_CUT, k
    return 0.0, None


def dot_shows(m, k):
    left = m["quick"][k] - m["removed"][k]
    return (not m["bled"][k]) and 0.0 < left <= DOT + 1e-9


def _np(x):
    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy().astype(np.float64)


def _torch_of(x):
    return getattr(x, "torch", x)


_IDS = {}
_CLOCK = {"dt": None, "steps": 0}


def _ids(asset, key, names):
    if key not in _IDS:
        all_names = list(asset.joint_names)
        _IDS[key] = {j: all_names.index(j) for j in names}
    return _IDS[key]


@on_reset
def reset_model():
    """A fresh trial: the quick's lengths by seed, nothing cut. mechanism.py has put every joint back to 0, the
    targets to theirs, and the clipper on its mat."""
    reset_memory(run_seed())


def _targets(m):
    out = {}
    for k in range(4):
        out[f"tip{k}"] = -m["removed"][k]
        out[f"dot{k}"] = DOT_DROP if dot_shows(m, k) else 0.0
        out[f"blood{k}"] = BLOOD_DROP if m["bled"][k] else 0.0
    return out


@on_step
def run_model(env):
    """Read the trainer, the clipper and both hands, advance the model, then set each claw's tip, dot and drop."""
    try:
        import torch
        base = env.unwrapped
        tr, cl = base.scene["trainer"], base.scene["clipper"]
        origin = _np(base.scene.env_origins[0])
        names = list(tr.body_names)
        poses = _np(tr.data.body_link_pose_w)[0]
        pp = poses[names.index("stand")]
        pc = _np(cl.data.root_link_pose_w)[0]
        qc = _np(cl.data.joint_pos)[0]
        cid = _ids(cl, "clipper", C_JOINTS)
        blade_q = blade_travel(float(qc[cid["handle_a"]]), float(qc[cid["handle_b"]]))
        tips = claw_tips(pp[:3] - origin, pp[3:7], _M["removed"])
        advance(_M, blade_q, tips, pc[:3] - origin, pc[3:7])
        hands = []
        for side in ("right", "left"):
            rob = base.scene[f"{side}_robot"]
            g = _np(rob.data.body_link_pose_w)[0][list(rob.body_names).index("gripper")]
            qr = _np(rob.data.joint_pos)[0]
            hands.append((g[:3] - origin + _rot3(g[3:7]) @ np.array(TCP_OFFSET),
                          float(qr[list(rob.joint_names).index("joint7")])))
        _M["hands"] = hands
        pre = (SPRING_PRE + _M["resist"]) / SPRING_K   # WHY a target shift: the springs' own drive adds the push-back
        tb = torch.tensor([[pre, -pre, blade_q]], dtype=torch.float32, device=_torch_of(cl.data.joint_pos).device)
        jc = [cid["handle_a"], cid["handle_b"], cid["blade"]]
        if hasattr(cl, "set_joint_position_target_index"):
            cl.set_joint_position_target_index(target=tb, joint_ids=jc)
        else:
            cl.set_joint_position_target(tb, joint_ids=jc)
        tg = _targets(_M)
        ids = _ids(tr, "trainer", tuple(tg))
        dev = _torch_of(tr.data.joint_pos).device
        t = torch.tensor([[tg[n] for n in tg]], dtype=torch.float32, device=dev)
        j = [ids[n] for n in tg]
        if hasattr(tr, "set_joint_position_target_index"):
            tr.set_joint_position_target_index(target=t, joint_ids=j)
        else:
            tr.set_joint_position_target(t, joint_ids=j)
        _M["steps"] += 1
    except Exception as e:  # WHY: never stop the run; no model step means no cut and so no score
        _M["error"] = "step " + repr(e)


@on_step
def read_clock(env):
    try:
        _CLOCK["dt"] = float(env.unwrapped.step_dt)
    except Exception:  # WHY: never stop the run; no step length means no hold, so no full score
        _CLOCK["dt"] = None
    _CLOCK["steps"] += 1


# -- scoring ---------------------------------------------------------------------------------------------------------
LOST_Z = TOP - 0.030
ON_MAT = 0.020                   # m: the clipper's grip point within this of its start is back on its mat
STILL_V = 0.02                   # m/s


def failure(objects, m):
    """Latched: a claw cut into the quick (it bleeds), the clipper off the table, or the model not running."""
    for k in range(4):
        if m["bled"][k]:
            return f"claw {k + 1} cut into the quick (it bleeds)"
    if float(objects["clipper"].pos[2]) < LOST_Z:
        return "clipper lost"
    if m["error"]:
        return "model " + m["error"]
    if _CLOCK["steps"] - m["steps"] > 5:
        return "model not stepping"
    return None


TCP_OFFSET = (0.0, 0.0, 0.134)   # yam_arena.constants: the TCP along the gripper's approach axis
CLEAR_R = 0.200                  # m: each hand's TCP this far from the clipper's grip point. WHY: no part of either
#                                  finger can then touch the clipper: its farthest point is 73.3 mm from the grip point
#                                  and the fingers' farthest point 101.4 mm from the TCP (tools/reach.py), 25 mm to
#                                  spare. A finger-opening rule is not enough: the open pads (94 mm apart) can pinch the
#                                  clipper end to end or corner to corner (review 2026-11-17, finding 2).


def hands_clear(m, point):
    """Both hands let go of the clipper. WHY: in v1 a hand holding the clipper still over its mat scored 1.0 (review
    finding 4, the #914 flag). A hand not read counts as holding it."""
    if not m["hands"]:
        return False
    return all(float(np.linalg.norm(np.asarray(tcp, float) - np.asarray(point, float))) >= CLEAR_R
               for tcp, _finger in m["hands"])


def clipper_down(objects, m=None):
    """The clipper back on its mat (its grip point within ON_MAT of its start), lying flat as it started (its up axis
    within 15 deg of vertical; WHY: on its edge it is still within ON_MAT, review 2026-11-17 finding 8), still, and
    let go of."""
    c = objects["clipper"]
    m = _M if m is None else m
    return (float(np.linalg.norm(np.asarray(c.pos, float) - np.asarray(CLIP_START))) <= ON_MAT
            and float(_rot3(c.quat_xyzw)[2, 2]) >= UPRIGHT
            and float(np.linalg.norm(np.asarray(c.lin_vel, float))) <= STILL_V
            and hands_clear(m, c.pos))


def stage(objects, m):
    """The number of claws showing their dots (0 to 4); 5 when all four show and the clipper is back on its mat."""
    k = sum(dot_shows(m, i) for i in range(4))
    return 5 if (k == 4 and clipper_down(objects, m)) else k


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
    n["done"] = bool(not _MEM["fail"] and n["stage"] == 5)
    _MEM["held"] = _MEM["held"] + (_CLOCK["dt"] or 0.0) if n["done"] else 0.0
    _MEM["now"] = n
    return _MEM


def success(objects):
    """All four claws show their dots, none bled, and the clipper is back on its mat (held 1 s by yam_arena)."""
    return update(objects)["now"]["done"]


LADDER = (0.0, 0.1, 0.2, 0.3, 0.5)   # WHY low: with 0.2 a claw a blind script (one length off every claw) earned 0.36
#                                     on average (review 2026-11-15, finding 13); now about 0.2, counting the 1.0 it
#                                     earns when all four dots happen to show and it puts the clipper down
HOLD_S = 1.0


def score(objects):
    """At the last step: 0 after a latched failure; success held 1 s 1.0 (yam_arena also ends the trial then); all four
    dots and the clipper down but not yet held 1 s 0.9; else LADDER by the number of claws showing their dots."""
    m = update(objects)
    if m["fail"]:
        return 0.0
    n = m["now"]
    if n["stage"] == 5:
        return 1.0 if (n["done"] and m["held"] >= HOLD_S - 1e-9) else 0.9
    return LADDER[n["stage"]]


# -- demo -----------------------------------------------------------------------------------------------------------------
TIP = 0.013               # PAD_TIP_PAST_TCP: the pad tips sit 13 mm under the TCP
TCP_OVER_G = 0.008        # the right TCP over the clipper's grip point: pad tips 1 mm over the handles' undersides (clear of the mat)
CLIP_SAG = 0.0019         # the held clipper sits this much under its TCP's plan (traced runs 2026-11-15, 3 seeds, at
#                           the firm hold: 1.94 mm at every still point), so the TCP is planned this much higher
OPEN_F = -0.04695         # FINGER_OPEN
HOLD_F = -0.0180          # the pads 36 mm apart: 3 mm into each handle (42 mm across), 15 N each, under the springs'
#                           20 N preload, so the blade stays open [WHY: at 5 N a touch on the ring turned the clipper
#                           3 to 7 degrees in the pads for good, traced runs 2026-11-15]
SQUEEZE_F = -0.0090       # the pads 18 mm apart: both handles to their stops (30 mm across) with 30 N, the blade
#                           across the hole [WHY: closing to 0 pushed 50 N and shifted the ring 2 mm, traced runs]
YAW_CUT = 0.0             # the clipper's heading while cutting, from the trainer's x: its head between the claws
PLAN_CUT = 0.0013         # each cut takes off about 1.3 mm
OVER = 0.004              # the ring's underside this far over a claw's end before it comes down onto it
SAFE_Z = TOP + Z_TIP + 0.008     # the grip point's carry height between claws (the ring's underside 6 mm over every end)


def _w(t, p, f, label, yaw=0.0):
    return Waypoint(float(t), tuple(float(v) for v in p), float(f), label, float(yaw))


class _Arm:
    def __init__(self):
        self.w, self.t = [], 0.0

    def add(self, dt, p, f, label, yaw=0.0):
        self.w.append(_w(dt, p, f, label, yaw))
        self.t += dt
        self.last = (tuple(p), f, yaw)

    def hold_to(self, t, label):
        if t > self.t + 1e-6:
            p, f, yaw = self.last
            self.add(t - self.t, p, f, label, yaw)


def cut_plan(quick):
    """The cuts the demo makes on one claw (m each): about PLAN_CUT a cut until the dot zone, the last landing in its
    middle (1 mm short of the quick)."""
    goal = quick - DOT / 2
    n = max(1, math.ceil(goal / PLAN_CUT - 1e-9))
    return [goal / n] * n


def tcp_z(ring_centre_z):
    """The right TCP's height that puts the ring's centre (the grip point's height) at ``ring_centre_z``."""
    return ring_centre_z + TCP_OVER_G + CLIP_SAG


def plan(delay=0.0, seed=None):
    """(right, info). The right hand picks up the clipper, and for each claw (from the far one) brings the ring down
    over its end and cuts about 1.3 mm at a time until the dot shows, lifting the ring off to look after each cut; then
    puts the clipper back on its mat and lets go. The left hand stays at its ready pose."""
    quick = [q / 1000.0 for q in quicks(seed)]   # WHY: the scripted demo reads the seed, as a privileged script may
    R = _Arm()
    g_tcp = (CLIP_START[0], CLIP_START[1], CLIP_START[2] + TCP_OVER_G)
    R.add(2.0 + delay, (g_tcp[0], g_tcp[1], g_tcp[2] + 0.05), OPEN_F, "over the clipper")
    R.add(1.0, g_tcp, OPEN_F, "down to the clipper")
    R.add(0.6, g_tcp, HOLD_F, "take hold of the handles")
    R.add(0.4, g_tcp, HOLD_F, "settle")
    R.add(1.2, (g_tcp[0], g_tcp[1], tcp_z(SAFE_Z)), HOLD_F, "lift the clipper")
    yw = T_YAW + YAW_CUT      # the fingers close across the handles, square to the clipper
    dx, dy = RING_X * math.cos(yw), RING_X * math.sin(yw)
    cuts = []
    for k in (0, 1, 2, 3):
        cx, cy = to_world(CLAW_X, CLAW_Y[k])
        x, y = cx - dx, cy - dy
        end = TOP + Z_TIP    # the claw's end (planned)
        R.add(1.6, (x, y, tcp_z(SAFE_Z)), HOLD_F, f"ring over claw {k + 1}", yw)
        for c in cut_plan(quick[k]):
            over = tcp_z(end + OVER + RING_T / 2)
            at = tcp_z(end - c - Z_BLADE)
            R.add(0.8, (x, y, over), HOLD_F, f"claw {k + 1}: ring just over its end", yw)
            R.add(0.8, (x, y, at), HOLD_F, f"claw {k + 1}: ring down, the end {c * 1000:.1f} mm over the blade", yw)
            R.add(0.3, (x, y, at), HOLD_F, "steady", yw)
            R.add(0.5, (x, y, at), SQUEEZE_F, "squeeze: cut", yw)
            R.add(0.5, (x, y, at), HOLD_F, "reopen the handles", yw)
            end -= c
            cuts.append((k, c))
            R.add(0.6, (x, y, tcp_z(end + OVER + RING_T / 2)), HOLD_F, "ring up off the claw: look at the cut face",
                  yw)
        R.add(0.8, (x, y, tcp_z(SAFE_Z)), HOLD_F, f"claw {k + 1} done: its dot shows", yw)
    R.add(2.0, (g_tcp[0], g_tcp[1], tcp_z(SAFE_Z)), HOLD_F, "back over the mat")
    R.add(1.2, (g_tcp[0], g_tcp[1], g_tcp[2] + 0.001), HOLD_F, "clipper onto its mat")
    R.add(0.6, (g_tcp[0], g_tcp[1], g_tcp[2] + 0.001), OPEN_F, "let go")
    R.add(1.0, (g_tcp[0], g_tcp[1], g_tcp[2] + 0.06), OPEN_F, "hand up")
    # WHY up and out, not straight up: 220 mm straight up is out of reach (ikcheck 31 mm off); this pose is 235 mm from
    # the grip point, past CLEAR_R, and moves away from the trainer and the other arm (-y).
    R.add(1.5, (g_tcp[0] + 0.06, g_tcp[1] - 0.18, g_tcp[2] + 0.138), OPEN_F, "hand away")
    R.hold_to(R.t + 3.0, "done")
    return R.w, {"quick": quick, "cuts": cuts}


def demo(delay=0.0):
    """The right hand trims each claw in small cuts to its dot, then puts the clipper back on its mat and lets go; the
    left stays at its ready pose (either hand may be used: both are watched for letting go)."""
    right, _ = plan(delay, run_seed())
    return {"right": right}


def _replan_demo_after_reset():
    """WHY: inspect-robots resets the policy before the embodiment (inspect_robots/rollout.py), so yam_scripted would
    call demo() while run_seed() still holds the previous trial's seed and plan the cuts for the wrong claws. This makes
    yam_scripted plan again on each trial's first action, after the reset has set the seed: the scripted demo reads the
    seed, as a privileged script may. It changes only the scripted demo policy, never the scene or the score."""
    try:
        from yam_arena import scripted
    except ImportError:
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_nail_replan", False):
        return
    reset, act = cls.reset, cls.act
    unset = object()

    def patched_reset(self, scene):
        self._nail_scene, self._nail_seed = scene, unset
        reset(self, scene)

    def patched_act(self, observation):
        if getattr(self, "_nail_seed", None) is unset:
            self._nail_seed = run_seed()
            reset(self, self._nail_scene)
        return act(self, observation)

    cls.reset, cls.act, cls._nail_replan = patched_reset, patched_act, True


_replan_demo_after_reset()
