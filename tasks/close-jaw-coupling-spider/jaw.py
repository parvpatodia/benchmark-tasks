"""Close an L-type jaw coupling on a small drive. A worm gear reducer's output shaft carries the fixed hub of a Lovejoy
L100 coupling, jaws toward the robot. In line with it a 5/8 in driven shaft turns freely in two pillow blocks; its
loose hub sits keyed on it, slid 50 mm back and turned 40 degrees out of mesh. The open-centre spider stands in a
holder on the bench. Put the spider into the fixed hub's jaws, turn the driven shaft until the loose hub's jaws face
the spider's free gaps, and slide the hub along its shaft until the faces stand the catalogue gap G apart.

Basis: Lovejoy L-Line installation guide (2012) 6.5 to 6.7 (src/lovejoy_install_2012.txt) and the Lovejoy jaw catalogue
(2025) L100 and L099/L100 spider tables (src/lovejoy_jaw_2025.txt). The set screw is not tightened (out of scope).
"""
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, body, hull, mechanism, on_reset, on_step, run_seed  # noqa: E402,F401
from motionkit import (HAND, RELEASE, Plan, SagCorrector, as_grip, box_distance, down, install, mode,  # noqa: E402
                       plans_out, together)
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 1200           # 60 s at 20 Hz
TOP = TABLE_TOP[SCENE]
MODES = ("demo", "nothing", "no_spider", "no_turn", "part_turn", "weak_push", "knock")
EXPECT = {"demo": 1.0, "nothing": 0.0, "no_spider": 0.0, "no_turn": 0.25, "part_turn": 0.25, "weak_push": 0.25,
          "knock": 0.0}

# Every number: [SOURCE] from a saved file in src/, [ARITH] derived, [EST] estimate (measure on the bought part),
# [CHOICE] design choice. World: x forward from the arm bases, y left, z up; metres.
IN = 0.0254
# -- Lovejoy L100 coupling (src/lovejoy_jaw_2025.txt, "L and C Type Dimensional Data", row L100) -----------------------
R_OD = 2.54 * IN / 2             # hub outside radius [SOURCE: OD 2.54 in]
LTB = 1.38 * IN                  # hub length through the bore [SOURCE: LTB 1.38 in]
GAP = 0.71 * IN                  # face-to-face gap G of the closed coupling [SOURCE: G 0.71 in, 18.03 mm]
HUB_KG = 0.69                    # [ARITH: complete coupling 3.20 lb at min bore [SOURCE], less a 26 g spider, halved,
#                                  less 25 g for the 5/8 in bore at 7.0 g/cm3 [EST density]]
# -- L099/L100 open-centre spider (src/lovejoy_jaw_2025.txt, "Elastomers In Compression", row L099/L100) -------------
SP_W = 0.61 * IN                 # spider width along the axis [SOURCE: W 0.61 in]
SP_B = 0.43 * IN                 # leg width [SOURCE: B 0.43 in]
SP_HOLE = 1.03 * IN / 2          # open centre radius [SOURCE: H 1.03 in]
SP_RING = 0.0180                 # outer radius of the centre ring, where the legs start [EST: measure]
SP_LEG = math.sqrt(R_OD ** 2 - (SP_B / 2) ** 2)   # leg end face from the centre: its corners on the OD [ARITH]
SP_RHO = 1250.0                  # kg/m3, polyester elastomer [EST]
# -- jaws ----------------------------------------------------------------------------------------------------------------
JAW_RIN = 0.0195                 # jaws start this far from the axis, clear of the spider ring [EST: measure]
PLAY = 0.0010                    # gap from each leg to each jaw flank [CHOICE: the rigid model's stand-in for the
#                                  elastomer's give, so the jaws enter the gaps at a usable phase window]
CHAMFER = 0.0012                 # 45 deg break on each jaw tip's flanks [EST: measure]
EDGE = 0.0010                    # break on each jaw's outer corners, so a finger on a leg's end clears it [EST]
SHAFT_R = 0.625 * IN / 2         # 5/8 in shafts [CHOICE: an L100 bore size; max bore 1.375 in, SOURCE]
# -- layout [CHOICE] -------------------------------------------------------------------------------------------------------
AXIS_H = 0.075                   # shafts' axis over the bench
AXIS_Z = TOP + AXIS_H
FX = 0.175                       # world x of the fixed hub's face; its jaws stand from FX to FX + G, toward +x
DIR = -1.0                       # the mechanisms' frame x runs toward the robot: world x = FX + DIR * frame x
ENGAGE = 0.014                   # loose hub's bore on the shaft when home [CHOICE: the guide's ideal is the shaft's
#                                  diameter, 15.9 mm; 14 mm leaves the spider 2.8 mm a side to come down past the shaft end]
SLIDE = 0.050                    # loose hub's start: this far back from home
TURN0 = math.radians(40.0)       # loose hub's start: turned this far (about the frame's +x) past mesh
SHAFT_R1 = -GAP - LTB + ENGAGE   # driven shaft's coupling end, frame x [ARITH]
SHAFT_R0 = -0.245                # its free end, frame x
BLOCKS = (-0.142, -0.227)        # pillow block centres, frame x
BLOCK = (0.032, 0.090, 0.024)    # pillow block housing x, y, and its top over the axis [EST: UCP202-10 class]
HOLDER = (0.220, 0.170)          # spider holder centre (world x, y), left of the drive


def wx(rel):
    """World x of a frame x."""
    return FX + DIR * rel


MATS = {"steel": (0.40, 0.30), "elast": (0.60, 0.50), "wood": (0.50, 0.40)}   # static / dynamic friction [EST]
STEEL_C, HUB_C, SPIDER_C, CAST_C, WOOD_C = (0.55, 0.56, 0.58), (0.30, 0.31, 0.34), (0.95, 0.55, 0.05), \
    (0.20, 0.35, 0.55), (0.72, 0.58, 0.38)
LINE_C = (0.95, 0.95, 0.95)

FIXED_JAWS = (90.0, 210.0, 330.0)        # jaw centres, deg about +x from +y toward +z [CHOICE: one jaw straight up]
LOOSE_MESH = (30.0, 150.0, 270.0)        # where the loose hub's jaws mesh [ARITH: midway]
LEGS = tuple(60.0 * k for k in range(6))  # spider legs [ARITH: one between each pair of jaws]


def _ring(pts2, x):
    return [(x, float(p[0]), float(p[1])) for p in pts2]


def _prism(pts2, x0, x1, color, material="steel", mass=0.0, collide=True):
    """A convex prism along x: 2D points (y, z about the axis) at x0 and at x1 (mechanism or body frame)."""
    return hull(_ring(pts2, x0) + _ring(pts2, x1), color, material, mass=mass) if collide else \
        Shape("hull", *_hb(pts2, x0, x1), color, material=material, points=tuple(_ring(pts2, x0) + _ring(pts2, x1)),
              mass=mass, collide=False)


def _hb(pts2, x0, x1):
    p = np.array(_ring(pts2, x0) + _ring(pts2, x1))
    lo, hi = p.min(0), p.max(0)
    return tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist())


def _rot2(pts, deg):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return [(c * y - s * z, s * y + c * z) for y, z in pts]


def jaw_outline(inset=0.0):
    """One jaw's cross-section, centred on angle 0 (+y): flanks parallel to the legs either side (at +-30 deg) and
    SP_B/2 + PLAY + inset from their centrelines, from JAW_RIN out to R_OD, outer corners broken by EDGE."""
    d = SP_B / 2 + PLAY + inset

    def flank(r, sign):           # point at radius r on the flank on the +-30 deg side
        a = math.radians(30.0) - math.asin(d / r)
        return (r * math.cos(a), sign * r * math.sin(a))
    lo, hi = flank(JAW_RIN, -1), flank(JAW_RIN, 1)
    ce_lo, ce_hi = flank(R_OD - EDGE, -1), flank(R_OD - EDGE, 1)
    a_out = math.radians(30.0) - math.asin(d / R_OD) - math.radians(3.0)
    arc = [(R_OD * math.cos(a), R_OD * math.sin(a)) for a in np.linspace(-a_out, a_out, 5)]
    return [lo, ce_lo, *arc, ce_hi, hi]


def leg_outline(deg):
    """One spider leg: a bar SP_B wide along the angle `deg`, from the ring to SP_LEG."""
    h = SP_B / 2
    return _rot2([(SP_RING - 0.001, -h), (SP_LEG, -h), (SP_LEG, h), (SP_RING - 0.001, h)], deg)


def ngon(r, n=12, phase=0.0):
    """A regular n-gon with its corners on r and a flat centred on `phase` deg (flats face +-y for n = 12)."""
    return [(r * math.cos(math.radians(phase + 180.0 / n + 360.0 * k / n)),
             r * math.sin(math.radians(phase + 180.0 / n + 360.0 * k / n))) for k in range(n)]


def hub_shapes(face_x, toward, jaw_degs, color, mass, length=LTB):
    """A hub: a 12-sided body from face_x away from the jaws (toward = +1: jaws toward +x), three jaws from the face
    GAP long, each tip chamfered. Mechanism frame (x relative to FX, y and z about the axis)."""
    body_x = (face_x - toward * length, face_x)
    body_pts = ngon(R_OD)
    shapes = [_prism(body_pts, min(body_x), max(body_x), color, mass=mass * 0.7)]
    tip = face_x + toward * GAP
    root_end = tip - toward * CHAMFER
    for deg in jaw_degs:
        main = _rot2(jaw_outline(), deg)
        point = _rot2(jaw_outline(CHAMFER), deg)
        shapes.append(_prism(main, min(face_x, root_end), max(face_x, root_end), color, mass=mass * 0.09))
        shapes.append(hull(_ring(main, root_end) + _ring(point, tip), color, "steel", mass=mass * 0.01))
    return shapes


# -- the drive: worm reducer (base) and its output shaft with the fixed hub (a link that cannot turn by hand) -----------
REDUCER_LO = (LTB + 0.008, -0.055, -AXIS_H)      # housing box in the mechanism frame [EST: a small worm reducer]
REDUCER_HI = (LTB + 0.090, 0.055, 0.045)


def _box(lo, hi, color, material="steel", mass=0.0, collide=True):
    c = tuple((np.add(lo, hi) / 2).tolist())
    s = tuple((np.subtract(hi, lo)).tolist())
    return Shape("box", c, s, color, material=material, mass=mass, collide=collide)


DRIVE = mechanism(
    "drive", (FX, 0.0, AXIS_Z),
    Link("reducer", 5.0, (_box(REDUCER_LO, REDUCER_HI, CAST_C, "steel"),)),
    [Link("output", HUB_KG + 0.05, tuple(hub_shapes(0.0, -1, FIXED_JAWS, HUB_C, HUB_KG))
          + (Shape("cylinder", (LTB + 0.004, 0.0, 0.0), (SHAFT_R, 0.008), STEEL_C, axis="X", material="steel"),))],
    # WHY a stiff, nearly locked joint: a worm reducer's output does not turn by hand; the hub must be a moving link so
    # the driven side's hub (another articulation) and the spider collide with its jaws [EST: stiffness, backlash]
    [Joint("output_turn", "revolute", "output", (0.0, 0.0, 0.0), "X", -0.002, 0.002, stiffness=200.0, damping=2.0)],
    MATS, yaw=math.pi, color=CAST_C, self_collide=False)

# -- the driven side: frame and pillow blocks (base), shaft (turns freely), loose hub (slides on the shaft's key) -------
HOME_FACE = -GAP         # loose hub face at home, mechanism frame [ARITH]
START_FACE = HOME_FACE - SLIDE
MARK_X = HOME_FACE - LTB  # the line scribed round the shaft at the hub's back face when home [ARITH]
_base_shapes = [_box((BLOCKS[1] - 0.03, -0.07, -AXIS_H), (BLOCKS[0] + 0.03, 0.07, -AXIS_H + 0.012), CAST_C)]
for bx in BLOCKS:
    _base_shapes.append(_box((bx - BLOCK[0] / 2, -BLOCK[1] / 2, -AXIS_H + 0.012),
                             (bx + BLOCK[0] / 2, BLOCK[1] / 2, BLOCK[2]), CAST_C))
_shaft_len = SHAFT_R1 - SHAFT_R0
_shaft_kg = math.pi * SHAFT_R ** 2 * _shaft_len * 7850.0   # [ARITH: steel 7850 kg/m3]
_loose = hub_shapes(START_FACE, +1, [d + math.degrees(TURN0) for d in LOOSE_MESH], HUB_C, HUB_KG)
DRIVEN = mechanism(
    "driven", (FX, 0.0, AXIS_Z),
    Link("frame", 4.0, tuple(_base_shapes)),
    [Link("shaft", _shaft_kg, (Shape("cylinder", ((SHAFT_R0 + SHAFT_R1) / 2, 0.0, 0.0), (SHAFT_R, _shaft_len),
                                     STEEL_C, axis="X", material="steel"),
                               _box((MARK_X - 0.0006, -SHAFT_R - 0.0003, -0.003), (MARK_X + 0.0006, SHAFT_R + 0.0003,
                                    SHAFT_R + 0.0003), LINE_C, collide=False))),
     Link("hub", HUB_KG, tuple(_loose))],
    [Joint("shaft_turn", "revolute", "shaft", (0.0, 0.0, 0.0), "X", -math.inf, math.inf, damping=0.02),
     # WHY damping, not friction: viscous drag stands in for the slip fit's light dry friction [EST]
     Joint("hub_slide", "prismatic", "hub", (0.0, 0.0, 0.0), "X", -0.010, SLIDE, damping=6.0, parent="shaft")],
    MATS, yaw=math.pi, color=CAST_C, self_collide=False)


# -- the spider (loose) and its holder -----------------------------------------------------------------------------------
def spider_shapes():
    out = []
    half = SP_W / 2
    for deg in LEGS:
        pts = leg_outline(deg)
        area = SP_B * (SP_LEG - SP_RING + 0.001)
        out.append(_prism(pts, -half, half, SPIDER_C, "elast", mass=area * SP_W * SP_RHO))
    for k in range(6):            # the ring between two legs, as a flat segment
        a0, a1 = math.radians(60 * k), math.radians(60 * k + 60)
        pts = [(SP_HOLE * math.cos(a0), SP_HOLE * math.sin(a0)), (SP_RING * math.cos(a0), SP_RING * math.sin(a0)),
               (SP_RING * math.cos(a1), SP_RING * math.sin(a1)), (SP_HOLE * math.cos(a1), SP_HOLE * math.sin(a1))]
        area = math.pi * (SP_RING ** 2 - SP_HOLE ** 2) / 6
        out.append(_prism(pts, -half, half, SPIDER_C, "elast", mass=area * SP_W * SP_RHO))
    return out


SPIDER_SHAPES = spider_shapes()
SP_LOW = min(p[2] for s in SPIDER_SHAPES for p in s.points)   # lowest point below the centre [ARITH]
HOLDER_WALL = (0.012, 0.050, 0.014)   # two blocks either face of the standing spider [CHOICE]
SPAWN_GAP = 0.0005
SPIDER_START = (HOLDER[0], HOLDER[1], TOP - SP_LOW + SPAWN_GAP)
SPIDER = body("spider", SPIDER_START, SPIDER_SHAPES, MATS, color=SPIDER_C)
SPIDER_KG = SPIDER.mass
_hw = SP_W / 2 + 0.0015 + HOLDER_WALL[0] / 2
HOLDER_BOXES = [Box(f"holder_{i}", (HOLDER[0] + s * _hw, HOLDER[1], TOP + HOLDER_WALL[2] / 2), WOOD_C, None, 0.0,
                    HOLDER_WALL) for i, s in enumerate((-1, 1))]

OBJECTS = [DRIVE, DRIVEN, SPIDER, *HOLDER_BOXES]

# -- scoring --------------------------------------------------------------------------------------------------------------
SEAT_R = 0.0015                  # spider centre within 1.5 mm of the axis [CHOICE: its 1.0 mm play shows as a 1.0 mm drop]
SEAT_TILT = 5.0                  # its axis within 5 deg of the shafts' [CHOICE]
SEAT_FACE = 0.0010               # its far face within 1.0 mm of the fixed hub's face [CHOICE: a feeler gauge]
MESH_IN = 0.003                  # loose jaw tips 3 mm or more past the spider's near face [CHOICE: a tip resting on a
#                                  leg stops at 0 mm; any jaw past it is in a gap]
LINE_TOL = 0.0010                # face gap within 1.0 mm of G [CHOICE: the catalogue gap read with a feeler gauge]
CLEAR = 0.030                    # every hand point 30 mm from the spider, loose hub and driven shaft [CHOICE]
STILL_V, STILL_W = 0.01, 0.5     # m/s, rad/s


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def spider_pose(st):
    """(centre, axis) of the spider in the world."""
    return np.asarray(st.pos, float), _rot(st.quat_xyzw)[:, 0]


def spider_rel(st):
    """(frame x of the spider's centre, its radial offset from the axis, its tilt in deg)."""
    c, ax = spider_pose(st)
    radial = math.hypot(c[1], c[2] - AXIS_Z)
    tilt = math.degrees(math.acos(min(1.0, abs(float(ax[0])))))
    return DIR * (c[0] - FX), radial, tilt


def spider_seated(st):
    rel, radial, tilt = spider_rel(st)
    far = rel + SP_W / 2          # its face toward the fixed hub (frame x 0 is that hub's face) [ARITH]
    return radial <= SEAT_R and tilt <= SEAT_TILT and abs(far) <= SEAT_FACE


def hub_face_rel(driven):
    return START_FACE + float(driven.joints["hub_slide"])


def jaw_depth(objects):
    """How far the loose hub's jaw tips stand past the spider's near face (m)."""
    rel, _, _ = spider_rel(objects["spider"])
    return hub_face_rel(objects["driven"]) + GAP - (rel - SP_W / 2)


def face_gap(objects):
    return -hub_face_rel(objects["driven"])


_HANDS = {"left": None, "right": None}


def set_hands(points):
    _HANDS.update(points)


@on_reset
def _reset():
    _HANDS.update({"left": None, "right": None})


@on_step
def read_hands(env):
    try:
        _HANDS.update({side: HAND.from_env(env, side) for side in ("left", "right")})
    except Exception:  # noqa: BLE001  WHY: never stop the run; unread hands count as touching
        _HANDS.update({"left": None, "right": None})


def _clear_of(boxes):
    for pts in _HANDS.values():
        if pts is None:
            return False
        for lo, hi in boxes:
            if box_distance(np.asarray(pts, float), lo, hi).min() < CLEAR:
                return False
    return True


def spider_box(objects):
    st = objects["spider"]
    return np.asarray(st.low, float), np.asarray(st.high, float)


def _wbox(r0, r1, rad):
    x0, x1 = sorted((wx(r0), wx(r1)))
    return np.array([x0, -rad, AXIS_Z - rad]), np.array([x1, rad, AXIS_Z + rad])


def driven_boxes(objects):
    """The loose hub (body and jaws) and the driven shaft as world boxes about the axis."""
    face = hub_face_rel(objects["driven"])
    return [_wbox(face - LTB, face + GAP, R_OD), _wbox(SHAFT_R0, SHAFT_R1, SHAFT_R)]


def still(objects):
    sp = objects["spider"]
    if not sp.resting(STILL_V, STILL_W):
        return False
    for name in ("shaft", "hub"):
        ln = objects["driven"].links.get(name)
        if ln is not None and not ln.resting(STILL_V, STILL_W):
            return False
    return True


def stage(objects):
    """0 none; 1 spider seated (hands clear of it); 2 and loose jaws in its gaps; 3 and faces at G; 4 and all still,
    hands clear of the spider, loose hub and shaft."""
    if not spider_seated(objects["spider"]) or not _clear_of([spider_box(objects)]):
        return 0
    if jaw_depth(objects) < MESH_IN:
        return 1
    if abs(face_gap(objects) - GAP) > LINE_TOL:
        return 2
    if not (still(objects) and _clear_of(driven_boxes(objects))):
        return 3
    return 4


SCORES = (0.0, 0.25, 0.5, 0.75, 1.0)


def score(objects):
    """0.25 spider seated; 0.5 and the loose jaws meshed; 0.75 and the hub at its line (faces G apart). 0 otherwise,
    including any state with the spider not seated. 1.0 comes only from success() held SUCCESS_HOLD_S: everything
    still with both hands 30 mm clear for 1 s (the harness then ends the trial with 1.0)."""
    # WHY capped at 0.75: SCORE_OVER "end" reads score() at the last step only, so a final state reached on the last
    # step must not score the full 1.0 that the task text reserves for a state held 1 s
    return SCORES[min(stage(objects), 3)]


def success(objects):
    return stage(objects) == 4


def distance(objects):
    return 1.0 - score(objects)


def trace_row(env):
    """fastrun --trace columns: the coupling's joints and the spider's centre."""
    out = {}
    try:
        sc = env.unwrapped.scene
        for mech, names in (("driven", ("shaft_turn", "hub_slide")), ("drive", ("output_turn",))):
            a = sc[mech]
            q = a.data.joint_pos
            q = getattr(q, "torch", q)[0].detach().cpu().numpy()
            jn = list(a.joint_names)
            for n in names:
                out[n] = float(q[jn.index(n)])
        p = sc["spider"].data.root_link_pose_w
        p = getattr(p, "torch", p)[0].detach().cpu().numpy()[:3]
        o = getattr(sc.env_origins, "torch", sc.env_origins)[0].detach().cpu().numpy()[:3]
        p = p - o
        out.update({"sp_x": float(p[0]), "sp_y": float(p[1]), "sp_z": float(p[2])})
    except Exception as e:  # noqa: BLE001
        out["trace_err"] = str(e)[:60]
    return out


# -- HD recording for the demo video (only with JAW_HD set; Dream and scoring runs never set it) -----------------------
class _CamRecorder:
    """Each control step, the full-size render of every camera to <JAW_HD>/<cam>.mp4 (H.264)."""

    def __init__(self):
        self.w = {}

    def add(self, env):
        out_dir = os.environ.get("JAW_HD")
        if not out_dir:
            return
        try:
            import imageio_ffmpeg
            sc = env.unwrapped.scene
            for cam in ("top_cam", "left_cam", "right_cam"):
                rgb = sc[cam].data.output["rgb"]
                rgb = getattr(rgb, "torch", rgb)
                frame = np.ascontiguousarray(rgb[0, ..., :3].cpu().numpy(), dtype=np.uint8)
                if cam not in self.w:
                    os.makedirs(out_dir, exist_ok=True)
                    h, w = frame.shape[:2]
                    import imageio.v2 as imageio
                    imageio.imwrite(os.path.join(out_dir, f"{cam}_first.png"), frame)
                    wr = imageio_ffmpeg.write_frames(os.path.join(out_dir, f"{cam}.mp4"), (w, h), pix_fmt_in="rgb24",
                                                     pix_fmt_out="yuv420p", fps=20, codec="libx264",
                                                     macro_block_size=8, output_params=["-crf", "18"])
                    wr.send(None)
                    self.w[cam] = wr
                self.w[cam].send(frame)
        except Exception as e:  # noqa: BLE001  WHY: a recorder fault must never stop or change a trial
            if "err" not in self.w:
                self.w["err"] = e
                print(f"[jaw] HD recorder off: {e}", flush=True)

    def close(self):
        for k, wr in list(self.w.items()):
            if k != "err":
                try:
                    wr.close()
                except Exception:  # noqa: BLE001
                    pass
        self.w = {}


_REC = _CamRecorder()
if os.environ.get("JAW_HD"):
    import atexit
    atexit.register(_REC.close)
    # WHY: render every camera at 1280 x 720 for the film only (the policy still gets its resized frames). The
    # embodiment takes its default render size from yam_arena.constants when it is imported, so set the constants
    # here, before it loads. WHY not import yam_arena.embodiment here: that loads Isaac Lab and USD before the app
    # starts, and the filmed run then failed at app start ("No to_python converter ... VtValue", film 459b9884, twice)
    if "yam_arena.embodiment" in sys.modules:
        print("[jaw] HD render size not set: embodiment already loaded", flush=True)
    else:
        import yam_arena.constants as _yc
        _yc.CAM_RENDER_HEIGHT, _yc.CAM_RENDER_WIDTH = 720, 1280


@on_step
def _record(env):
    _REC.add(env)


# -- demo (motionkit; scene constants and the arms' own joint readings only) ----------------------------------------------
install(SagCorrector())
SPIDER_GRIP_F = 15.0             # N a finger on the leg ends [CHOICE: 0.28 N spider]
CARRY = 0.085                    # TCP height over the axis while carrying the spider [CHOICE: its legs 54 mm up, 22 mm
#                                  over the hubs]
SLOT_X = wx((SHAFT_R1 - GAP) / 2)  # spider centre while it comes down between shaft end and jaw tips [ARITH]
SEAT_X = wx(-SP_W / 2)           # spider centre when seated [ARITH]
SEAT_PUSH = 0.0020               # spider pushed this far past its seat [CHOICE: arm compliance, the face stops it]
# WHY 2 mm, not 0.5: the spider rests 1 mm low on the jaws' lower flanks (its play) and elastomer-on-steel friction
# stopped it 0.6 to 1.4 mm off the face with a 0.5 mm push (RUN: fastrun 695e1ac4, demo and no_turn), at the edge of
# SEAT_FACE. checkkit sees the planned spider 2 mm into the face, which the face stops, so its carried-part limit is:
PART_OVERLAP = -0.0025           # m, checkkit: a carried part may sit this far into another [CHOICE: SEAT_PUSH + 0.5 mm]
PUSH_Z = 0.026                   # pushing hand's TCP over the axis: tips 13 mm up, 5 mm over the shaft [ARITH]
PUSH_HALF = 0.0070               # closed fingers' half thickness along x at the hub face's height [RUN: HAND.local]
LINE_PUSH = 0.0040               # the hub pushed this far past home [CHOICE: its jaw tips on the fixed face stop it]
PUSH_DEEP = 0.015                # [CHOICE: see _push_go]
SHORT_BY = 0.008                 # "weak_push" mode aims the hub this far short of home, without PUSH_DEEP [CHOICE]
# WHY weak_push scores 0.25, not 0.5: without PUSH_DEEP the jaw tips jam where they meet the seated spider (hub at 35.6
# of 50 mm, tips 1.1 mm in; RUN: fastrun 63668945 seeds 0-2); aimed at the line itself it stopped at 35.2 mm (RUN:
# 966d8520). A push that breaks through runs on to the slide's stop at home, so no pushed end state is meshed but short
AWAY = {"left": (0.20, 0.28, 0.20), "right": (0.30, -0.28, 0.20)}   # [CHOICE]


def _spider_in(L):
    L.where["spider"] = SPIDER_START
    L.travel(0.14, to=HOLDER)
    L.pick("spider", h=0.0, width=2 * SP_LEG, grip=down(0.0), above=0.08, lift=0.07, force=SPIDER_GRIP_F,
           label="pick spider")
    L.move((L.pos[0], L.pos[1], AXIS_Z + CARRY), label="carry: up")
    L.move((SLOT_X, 0.0, AXIS_Z + CARRY), label="carry: across")
    L.hold(0.8, "carry: line up")
    L.move((SLOT_X, 0.0, AXIS_Z), t=2.5, label="seat: down", contact=True)
    L.hold(0.6, "seat: settle", contact=True)
    L.move((SEAT_X + DIR * SEAT_PUSH, 0.0, AXIS_Z), t=2.5, label="seat: in", contact=True)
    L.hold(0.6, "seat: home", contact=True)
    L.move(L.pos, t=0.8, finger=L.pre, label="seat: let go", contact=True)
    RELEASE["left"].add("seat: let go")
    L.held = None
    L.move(L.pos + np.array([0.0, 0.0, 0.07]), t=1.5, label="seat: up")


BACK0 = wx(START_FACE - LTB)     # loose hub's back face at the start, world x [ARITH]
BACK_HOME = wx(HOME_FACE - LTB)  # ... at home [ARITH]
PUSH_X0 = BACK0 - DIR * (PUSH_HALF + 0.004)


def _push_ready(L):
    # WHY 0.06: at 0.09 over the axis joint 4 sat 1.2 deg off its limit (IK); tips still clear the hub by 15 mm
    L.move((PUSH_X0, 0.0, AXIS_Z + 0.06), grip=down(-90.0), finger=0.0, label="push: over")
    L.hold(0.6, "push: line up")
    L.move((PUSH_X0, 0.0, AXIS_Z + PUSH_Z), t=1.5, label="push: down")


def _push_go(L, short=0.0):
    x1 = BACK_HOME - DIR * (PUSH_HALF + short) + DIR * LINE_PUSH * (short == 0.0)
    # WHY the full push aims PUSH_DEEP past the line: the loose jaws must lift the seated spider out of its 1 mm drop
    # on their chamfers, and at a target 4 mm past the line the arm stalled 17 mm short with the hub at 36.6 of 50 mm
    # (RUN: fastrun 629e3235, 36efd5c8, 7bc25add); the slide joint's stop at home ends the hub's travel on the line
    xp = x1 + DIR * PUSH_DEEP * (short == 0.0)
    L.move((xp, 0.0, AXIS_Z + PUSH_Z), t=max(2.0, abs(xp - PUSH_X0) / 0.02), label="push: slide", contact=True)
    L.hold(0.8, "push: hold", contact=True)
    L.move((x1 - DIR * 0.015, 0.0, AXIS_Z + PUSH_Z), t=0.8, label="push: back off")
    L.move((x1 - DIR * 0.015, 0.0, AXIS_Z + 0.06), t=1.0, label="push: up")


# The turn: closed fingers (closing along the shaft) come down beside the top jaw of the loose hub and push it across,
# toward -y, at a fixed height; the hub and shaft turn with it (the hub rides the shaft's key). The hand's end point
# sets the phase. WHY a push, not a wrist turn on a grip: a rigid grasp on a shaft held by two bearings fought the arm
# (the TCP was shoved 13 mm off the axis) and the shaft rolled between the pads the wrong way (RUN: fastrun 5c422718,
# 47173125, 10e9daf2: shaft +11 to +48 deg where -40 was wanted); a one-sided push tolerates the arm's mm errors.
PUSHJ_X = wx(START_FACE + GAP / 2)  # mid-length of the loose hub's jaws at the start, world x [ARITH]
PUSHJ_Z = 0.033                  # TCP over the axis: finger tips 20 mm up, on the top jaw's flank [CHOICE]
PUSHJ_Y0 = 0.030                 # start, clear of every jaw [ARITH: 2D sweep of HAND points against jaw_outline]
PUSHJ_Y = {40.0: -0.0130, 20.0: -0.00175}   # end y for a 40 (mesh) or 20 deg turn [ARITH: same sweep, 0.2 mm steps]
PUSHJ_GRIP = 90.0                # fingers close along the shaft: 8.8 mm thick along x at the tips, inside the 18 mm jaw


def _turn(R, deg):
    y1 = PUSHJ_Y[deg]
    R.move((PUSHJ_X, PUSHJ_Y0, AXIS_Z + PUSHJ_Z + 0.04), grip=down(PUSHJ_GRIP), finger=0.0, label="turn: over")
    R.hold(0.5, "turn: line up")
    R.move((PUSHJ_X, PUSHJ_Y0, AXIS_Z + PUSHJ_Z), t=1.5, label="turn: down")
    R.move((PUSHJ_X, y1, AXIS_Z + PUSHJ_Z), t=max(2.0, abs(PUSHJ_Y0 - y1) / 0.008), label="turn: push", contact=True)
    R.hold(0.5, "turn: hold", contact=True)


def _turn_off(R):
    R.move(R.pos + np.array([0.0, 0.004, 0.0]), t=0.8, label="turn: let go", contact=True)
    R.move(R.pos + np.array([0.0, 0.0, 0.06]), t=1.2, label="turn: back out")


CHECK_SKIP = {("left", "driven"): ("push:",), ("right", "driven"): ("turn:",),
              # WHY: the planned fingers squeeze F/k past the leg ends; in the sim they stop on the rigid legs, so
              # the plan reads them inside the fixed hub. test_jaw checks the real pad plane against the drive
              ("left", "drive"): ("seat:", "knock:")}


def plan(seed=None, mode_name="demo"):
    if mode_name == "nothing":
        return {}
    L, R = Plan("left"), Plan("right")
    turned = {"demo": 40.0, "no_spider": 40.0, "weak_push": 40.0, "part_turn": 20.0}.get(mode_name)
    if turned is not None:
        # WHY the right turns, leaves and parks before the left starts: its hand at the loose hub and the left's
        # carried spider came within 0.1 mm (checkkit ARMGAP) when they overlapped in time
        _turn(R, turned)
        _turn_off(R)
        R.move((AWAY["right"][0], AWAY["right"][1], TOP + AWAY["right"][2]), grip=down(0.0), label="park")
        together(L, R)
    if mode_name == "knock":
        kx = FX - LTB / 2
        L.move((kx, 0.090, AXIS_Z + 0.06), grip=down(-90.0), finger=0.0, label="knock: over")
        L.move((kx, 0.090, AXIS_Z), t=1.0, label="knock: down")
        L.move((kx, 0.025, AXIS_Z), t=1.0, label="knock: shove", contact=True)
        L.move((kx, 0.090, AXIS_Z), t=1.0, label="knock: back")
        L.move((kx, 0.090, AXIS_Z + 0.10), t=1.0, label="knock: up")
    else:
        if mode_name != "no_spider":
            _spider_in(L)
        together(L, R)
        if mode_name == "no_spider":
            # WHY: from its start pose the left's IK reached "push: over" on a branch near joint 4's limit; passing
            # the carry pose first, as the demo does, keeps it on the demo's branch
            L.move((SLOT_X, 0.0, AXIS_Z + CARRY), grip=down(0.0), label="carry: across")
        _push_ready(L)
        _push_go(L, SHORT_BY if mode_name == "weak_push" else 0.0)
    if R.waypoints:
        R.move((AWAY["right"][0], AWAY["right"][1], TOP + AWAY["right"][2]), grip=down(0.0), label="away")
    L.move((AWAY["left"][0], AWAY["left"][1], TOP + AWAY["left"][2]), grip=down(0.0), label="away")
    together(L, R)
    return plans_out(L, R)


def demo():
    return plan(run_seed(), mode())
