"""A vertical 1/2 in copper water-tube joint stands in a pipe vise: the top tube is pushed into a cleaned and fluxed
coupling. A lit hose torch lies on its rest and a small coil of 1/8 in lead-free solder wire stands in its holder, its
straight end down. Sweat the joint: heat it with the torch, its flame on the base of the coupling's cup, never into the
cup's mouth or onto the solder's end. Touch the end of the solder to the joint where the top tube enters the coupling; it
melts only once the joint is hot enough. Once it melts, feed solder into the joint until a ring of solder shows all
round the joint's edge (about 8 mm of the wire). Do not overheat the joint, which burns the flux. Then take the solder
and the torch away from the joint (the flame's tip at least 8 cm from the base of the cup), keeping each held up or
back on its stand, and let the joint cool undisturbed until the solder sets.
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, body, hull, mechanism, on_reset, on_step, run_seed  # noqa: E402
from yam_arena import station  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
RUN_LIMIT_STEPS = 3000           # 150 s at 20 Hz: the demo succeeds after 31 to 38 s [RUN]
TOP = TABLE_TOP[SCENE]
IN = 0.0254
D2R = math.pi / 180

# -- the joint ----------------------------------------------------------------------------------------------------------
TUBE_R = 0.625 * IN / 2          # [SOURCE: CDA Table 1, 1/2 in nominal, tube OD .625 in]
CUP_D = 0.500 * IN               # [SOURCE: CDA Table 1, 1/2 in nominal, cup depth of fitting .500 in]
CUP_R = 0.0091                   # the coupling's outside radius [EST: a wrought coupling's wall about 1.2 mm]
CLEARANCE = 0.004                # in, the joint's clearance [CHOICE: a Table 1 column]
FULL = 0.320 * IN                # 8.13 mm of 1/8 in wire fills it [SOURCE: CDA Table 1, 1/2 in, 0.004 in clearance]
JX = 0.25                        # the joint's axis (x, y = 0)
Z_MOUTH = 0.065                  # the coupling's top (the cup's mouth) over the table
Z_BASE = Z_MOUTH - CUP_D         # the cup's base, where the flame aims [SOURCE: CDA, "to the base of the cup to aid capillary"]
COUPLING_L = 0.028               # two cups and the tube stop between [EST]
STUB_UP = 0.030                  # the top tube stands this far over the coupling (a cut stub) [CHOICE]
VISE_H = 0.030

# -- heat (our model: see fidelity) ------------------------------------------------------------------------------------
T_ROOM = 70.0                    # deg F
LIQUIDUS = 464.0                 # deg F [SOURCE: Harris 95-5 sheet, "Liquidus 464 F (240 C)"]
SOLIDUS = 452.0                  # deg F [SOURCE: Harris 95-5 sheet, "Solidus 452 F (233 C)"]
BURN_T = 600.0                   # deg F, the flux's top [SOURCE: Superior No. 142-5, "Soldering Temperature Range 95C - 315C /200F- 600F"]
TAU = 25.0                       # s, the joint's cooling time constant in still air [EST]
T_SS = (1050.0, 850.0, 720.0)    # deg F the joint would reach with the flame held on its mark, by seed [CHOICE]: how
#                                  fast a joint heats varies with the torch's setting, draft and the tube's heat sink
#                                  (CDA: "Experience will indicate the amount of heat and the time needed")
POWER = tuple((t - T_ROOM) / TAU for t in T_SS)   # deg F/s at the mark
SIGMA = 0.025                    # m: heat falls off with the flame tip's distance from the mark as exp(-d^2/2 sigma^2) [EST]
FEED_RATE = 0.005                # m/s: the most wire a hot joint takes in [EST]
FED_MAX = 0.0125                 # the wire sinks no further than the cup's depth less 0.2 mm
PRESS_D = 0.0005                 # m: the wire pushes the joint's face this far under the model's level, so it presses
ON_TOL = 0.0005                  # m: the lowest point of the wire's end within this of the joint's face touches it
EDGE_R = CUP_R - TUBE_R          # m: the wire's inner edge over the coupling's lip (its 1.2 mm wall): on the joint's edge
FLAME_CLEAR = 0.010              # m: the inner cone nearer than this to the joint's edge or the wire's tip is the flame
#                                  into the face of the cup or onto the solder (CDA: "Do not overheat the joint or
#                                  direct the flame into the face of the fitting cup") [CHOICE: 10 mm]
TOOL_Z = 0.080                   # m: a tool's frame this far over the table is held up (over the vise, the rest, the
#                                  holder and the coupling's mouth)
ON_STAND = 0.020                 # m: a tool's frame within this of its start is on its stand

# -- the torch (a loose body, its frame at the grip, flame along its -y) -------------------------------------------------
PHI = -49.5 * D2R                # the flame's heading in the world (from +x): the torch comes in from the left-front
U = (math.cos(PHI), math.sin(PHI))
TORCH_YAW = PHI + math.pi / 2    # the body's yaw (its -y along U)
FLAME_TIP_Y = -0.165             # the inner cone's tip, the flame's hottest point [EST: a hose torch's 25 mm inner cone]
HANDLE = (-0.020, 0.090)         # the handle's span along the torch (y); the hose would enter at +y (not modelled)
HANDLE_W = 0.022
TORCH_KG = (0.20, 0.05, 0.03)    # handle, burner tube, nozzle [EST: a hose torch's head about 0.3 kg]
MARK = (JX - CUP_R * U[0], -CUP_R * U[1], TOP + Z_BASE)  # the flame's mark: the cup's base, facing the torch
G_HEAT = (MARK[0] + FLAME_TIP_Y * U[0], MARK[1] + FLAME_TIP_Y * U[1], MARK[2])
G_REST = (0.075, 0.215, TOP + Z_BASE)
REST_TOP = Z_BASE - HANDLE_W / 2 - 0.0005   # the rest's top under the handle (0.5 mm under it at the start)
AWAY_FLAME = 0.08                # m: the flame tip this far from the mark counts as away

# -- the solder wire (a loose body, its frame at its middle) -------------------------------------------------------------
WIRE_R = 0.125 * IN / 2          # [SOURCE: CDA Table 1 note, "Using 1/8-inch diameter (No. 9) Wire Solder"]
WIRE_L = 0.055                   # the straightened end below the coil [CHOICE: plumbers hold the coil, the end
#                                  straightened]
WIRE_KG = 7.25e3 * math.pi * WIRE_R ** 2 * WIRE_L   # 95-5 about 7.25 g/cm3 [EST] = 3.1 g
COIL_R, COIL_H = 0.012, 0.018    # the hand coil the robot holds [EST]
COIL_KG = 0.045                  # [ARITH: 7.25 g/cm3 x the coil's volume x 0.75 packing]
HOLD_XY = (0.30, -0.12)
HOLE = 0.010                     # the holder's square well the straight end hangs in [CHOICE]
HOLDER_H = WIRE_L + 0.0015       # the coil sits on the holder, its end 1.5 mm over the well's floor
HOLDER_W = 0.036
AWAY_WIRE = 0.030                # m: the wire's tip this far from the joint's mouth counts as away

# -- the joint's face (the wire rests on it) and the drawn-only rings ---------------------------------------------------
STOP_R = CUP_R                   # the joint's face: the coupling's lip and the gap beside the top tube
STOP_H = 0.003
STOP_TOP = Z_MOUTH - 0.0003      # just under the coupling's top
K_STOP = 800.0                   # N/m: the face's spring (0.4 N at PRESS_D) [CHOICE]
BEAD_RISE, BURN_RISE = 0.0066, 0.0090

MATERIALS = {"default": (1.2, 1.0),   # the scene's friction on its own assets (rules.md 3)
             "steel": (0.6, 0.5),
             "copper": (0.53, 0.45),
             "rubber": (1.0, 0.9),     # the torch's grip [EST]
             "solder": (0.8, 0.7)}     # [EST]
STEEL, BLACK, SILVER, WHITE = (0.30, 0.31, 0.33), (0.07, 0.07, 0.08), (0.78, 0.79, 0.81), (0.95, 0.95, 0.95)
COPPER, BRASS, RED, SOOT = (0.78, 0.45, 0.30), (0.80, 0.66, 0.30), (0.75, 0.12, 0.10), (0.10, 0.08, 0.07)
BLUE, PALE = (0.20, 0.40, 1.00), (0.55, 0.72, 1.00)


def _box(lo, hi, color, material="steel", collide=True, mass=0.0):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Shape("box", tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, material=material,
                 collide=collide, mass=mass)


def _cyl(c, r, h, color, axis="Z", material="steel", collide=True, mass=0.0):
    return Shape("cylinder", tuple(float(v) for v in c), (float(r), float(h)), color, axis=axis, material=material,
                 collide=collide, mass=mass)


def _zcyl(r, z0, z1, color, material="copper", collide=True, x=0.0, y=0.0):
    return _cyl((x, y, (z0 + z1) / 2), r, z1 - z0, color, material=material, collide=collide)


def _turned_block(cx, cy, along, across, z0, z1, heading):
    """A hull block centred at (cx, cy) in the bench frame, ``along`` long on ``heading``, ``across`` wide."""
    a, c = (math.cos(heading), math.sin(heading)), (-math.sin(heading), math.cos(heading))
    ring = [(cx + sa * along / 2 * a[0] + sc * across / 2 * c[0], cy + sa * along / 2 * a[1] + sc * across / 2 * c[1])
            for sa, sc in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return hull([(x, y, z0) for x, y in ring] + [(x, y, z1) for x, y in ring], color=STEEL, material="steel")


def bench_base():
    """In the bench's frame (x from the joint's axis, y, z from the table): the vise, both tubes and the coupling, the
    torch's rest and the wire's holder."""
    s = [_box((-0.030, -0.030, 0.0), (0.030, 0.030, VISE_H), STEEL),
         _zcyl(TUBE_R, VISE_H - 0.010, Z_MOUTH - COUPLING_L / 2 - 0.0008, COPPER),          # the bottom tube
         _zcyl(CUP_R, Z_MOUTH - COUPLING_L, Z_BASE - 0.001, COPPER),                         # the coupling's lower half
         _zcyl(CUP_R, Z_BASE - 0.001, Z_MOUTH, COPPER, collide=False),                      # its top cup (drawn only)
         _zcyl(TUBE_R, Z_BASE, Z_MOUTH + STUB_UP, COPPER)]                                  # the top tube in the cup
    # the torch's rest: a block under the handle's middle
    mid = 0.5 * (HANDLE[0] + HANDLE[1])
    rx, ry = G_REST[0] - mid * U[0] - JX, G_REST[1] - mid * U[1]
    s.append(_turned_block(rx, ry, HANDLE[1] - HANDLE[0] - 0.020, 0.040, 0.0, REST_TOP, PHI))
    # the wire's holder: a block with a square hole, as four blocks round it
    hx, hy, h, o = HOLD_XY[0] - JX, HOLD_XY[1], HOLE / 2, HOLDER_W / 2
    s += [_box((hx - o, hy - o, 0.0), (hx + o, hy - h, HOLDER_H), STEEL),
          _box((hx - o, hy + h, 0.0), (hx + o, hy + o, HOLDER_H), STEEL),
          _box((hx - o, hy - h, 0.0), (hx - h, hy + h, HOLDER_H), STEEL),
          _box((hx + h, hy - h, 0.0), (hx + o, hy + h, HOLDER_H), STEEL)]
    return Link("bench", 5.0, tuple(s))


def _hidden_mass():
    """A 1 mm collider deep in the vise (nothing reaches it) so a drawn-only ring is a proper rigid body."""
    return _box((-0.0005, -0.0005, 0.010), (0.0005, 0.0005, 0.011), STEEL)


def bench_links():
    stop = Link("face", 0.010, (_zcyl(STOP_R, STOP_TOP - STOP_H, STOP_TOP, COPPER),))
    bead = Link("bead", 0.005, (_zcyl(0.0090, Z_MOUTH - BEAD_RISE, Z_MOUTH - BEAD_RISE + 0.0012, SILVER,
                                      collide=False), _hidden_mass()))
    burn = Link("burn", 0.005, (_zcyl(0.00905, Z_MOUTH - BURN_RISE, Z_MOUTH - BURN_RISE + 0.004, SOOT,
                                      collide=False), _hidden_mass()))
    joints = (Joint("face", "prismatic", "face", (0.0, 0.0, STOP_TOP), "Z", -FED_MAX - 0.0010, 0.0,
                    stiffness=K_STOP, damping=8.0, target=0.0),
              Joint("bead", "prismatic", "bead", (0.0, 0.0, Z_MOUTH), "Z", -0.001, BEAD_RISE + 0.001,
                    stiffness=400.0, damping=10.0, target=0.0),
              Joint("burn", "prismatic", "burn", (0.0, 0.0, Z_MOUTH), "Z", -0.001, BURN_RISE + 0.001,
                    stiffness=400.0, damping=10.0, target=0.0))
    return (stop, bead, burn), joints


def torch_shapes():
    h0, h1 = HANDLE
    return (_box((-HANDLE_W / 2, h0, -HANDLE_W / 2), (HANDLE_W / 2, h1, HANDLE_W / 2), RED, material="rubber",
                 mass=TORCH_KG[0]),
            _cyl((0.0, -0.070, 0.0), 0.005, 0.100, BRASS, axis="Y", material="steel", mass=TORCH_KG[1]),
            _cyl((0.0, -0.130, 0.0), 0.009, 0.020, BLACK, axis="Y", material="steel", mass=TORCH_KG[2]),
            _cyl((0.0, -0.1525, 0.0), 0.003, 0.025, BLUE, axis="Y", collide=False, mass=1e-4),     # the inner cone
            _cyl((0.0, -0.1575, 0.0), 0.0055, 0.035, PALE, axis="Y", collide=False, mass=1e-4))   # the outer flame


def wire_shapes():
    """The hand coil (its frame at the coil's middle) and the straight end hanging from its middle."""
    return (_cyl((0.0, 0.0, 0.0), COIL_R, COIL_H, SILVER, material="solder", mass=COIL_KG),
            _cyl((0.0, 0.0, -COIL_H / 2 - WIRE_L / 2), WIRE_R, WIRE_L, SILVER, material="solder", mass=WIRE_KG))


_LINKS, _JOINTS = bench_links()
BENCH = mechanism("bench", (JX, 0.0, TOP), bench_base(), _LINKS, _JOINTS, MATERIALS, self_collide=False)
TORCH = body("torch", (G_REST[0], G_REST[1], G_REST[2]), torch_shapes(), MATERIALS, yaw=TORCH_YAW, color=RED)
WIRE_START = (HOLD_XY[0], HOLD_XY[1], TOP + HOLDER_H + COIL_H / 2 + 0.0005)
TIP_LOCAL = (0.0, 0.0, -COIL_H / 2 - WIRE_L)
WIRE = body("wire", WIRE_START, wire_shapes(), MATERIALS, color=SILVER)
OBJECTS = [BENCH, TORCH, WIRE]
JOINTS = ("face", "bead", "burn")
MOUTH = (JX, 0.0, TOP + Z_MOUTH)


# -- the model ---------------------------------------------------------------------------------------------------------
def heat_index(seed):
    """Which of T_SS a trial gets: inspect-robots' --seed 0, 2, 4 give 1, 0, 2 (README)."""
    return 1 if seed is None else (int(seed) // 3) % 3


def _fresh_model(seed=None):
    return {"idx": heat_index(seed), "T": T_ROOM, "T_max": T_ROOM, "fed": 0.0, "burnt": False, "stuck": False,
            "on": False, "pressed": False, "flamed": False, "steps": 0, "error": None, "t": 0.0, "t_melt": None}


_M = _fresh_model()


def weight(flame_tip):
    d = float(np.linalg.norm(np.asarray(flame_tip, float) - np.asarray(MARK, float)))
    return math.exp(-0.5 * (d / SIGMA) ** 2)


def on_joint(wire_low, face_q):
    """The wire's end on the joint's edge: its inner edge within EDGE_R of the top tube, and its lowest point within
    ON_TOL of the face's top."""
    x, y, z = (float(v) for v in wire_low)
    r = math.hypot(x - JX, y)
    return bool(r - WIRE_R <= TUBE_R + EDGE_R and abs(z - (TOP + STOP_TOP + face_q)) <= ON_TOL)


def edge_dist(p):
    """Distance from a point to the joint's edge: the ring at the coupling's mouth between the top tube and the
    coupling's outside."""
    rho = math.hypot(float(p[0]) - JX, float(p[1]))
    dr = max(0.0, rho - CUP_R, TUBE_R - rho)
    return math.hypot(dr, float(p[2]) - (TOP + Z_MOUTH))


def advance(m, dt, flame_tip, wire_tip, face_q):
    """One step of the joint's heat and the solder's melting; returns the face's target (m, its joint)."""
    t0 = m["T"]
    m["T"] = t0 + dt * (POWER[m["idx"]] * weight(flame_tip) - (t0 - T_ROOM) / TAU)
    m["T_max"] = max(m["T_max"], m["T"])
    m["t"] += dt
    m["on"] = on_joint(wire_tip, face_q)
    ft = np.asarray(flame_tip, float)
    if min(edge_dist(ft), float(np.linalg.norm(ft - np.asarray(wire_tip, float)))) < FLAME_CLEAR:
        m["flamed"] = True
    m["pressed"] = bool(m["on"] and (-m["fed"] - face_q) >= PRESS_D)
    if m["pressed"] and m["T"] >= LIQUIDUS and m["fed"] < FED_MAX:
        m["fed"] = min(FED_MAX, m["fed"] + FEED_RATE * dt)
        if m["t_melt"] is None:
            m["t_melt"] = m["t"]
    if m["T"] > BURN_T:
        m["burnt"] = True
    if m["fed"] > 0 and t0 >= SOLIDUS > m["T"] and m["on"]:
        m["stuck"] = True   # the solder set round the wire's tip
    return -m["fed"]


def _np(x):
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _torch_of(x):
    return getattr(x, "torch", x)


def _rot3(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def flame_tip(pos, quat):
    return np.asarray(pos, float) + _rot3(quat) @ np.array([0.0, FLAME_TIP_Y, 0.0])


def wire_tip(pos, quat):
    return np.asarray(pos, float) + _rot3(quat) @ np.array(TIP_LOCAL)


def wire_low(pos, quat):
    """The lowest point of the wire's end face (its centre lowered by the tilt)."""
    p = wire_tip(pos, quat)
    c = float(_rot3(quat)[2, 2])
    p[2] -= WIRE_R * math.sqrt(max(0.0, 1.0 - c * c))
    return p


_IDS = {}
_CLOCK = {"dt": None, "steps": 0}


def _ids(asset):
    if not _IDS:
        names = list(asset.joint_names)
        _IDS.update({j: names.index(j) for j in JOINTS})
    return _IDS


@on_reset
def reset_model():
    """A fresh trial: the model (the heat rate by seed) and the scorer. The bench's joints are put back to 0 and their
    targets to 0 by mechanism.py's reset."""
    reset_memory(run_seed())


@on_step
def run_model(env):
    """Read the torch, the wire and the joint's face, advance the model, then set the face's level and the rings."""
    try:
        import torch
        base = env.unwrapped
        bench, tor, wir = base.scene["bench"], base.scene["torch"], base.scene["wire"]
        ids = _ids(bench)
        origin = _np(base.scene.env_origins[0])
        q = _np(bench.data.joint_pos)[0]
        pt = _np(tor.data.root_link_pose_w)[0]
        pw = _np(wir.data.root_link_pose_w)[0]
        dt = float(base.step_dt)
        face = advance(_M, dt, flame_tip(pt[:3] - origin, pt[3:7]), wire_low(pw[:3] - origin, pw[3:7]),
                       float(q[ids["face"]]))
        tgt = [face, BEAD_RISE if _M["fed"] >= FULL else 0.0, BURN_RISE if _M["burnt"] else 0.0]
        dev = _torch_of(bench.data.joint_pos).device
        t = torch.tensor([tgt], dtype=torch.float32, device=dev)
        j = [ids["face"], ids["bead"], ids["burn"]]
        if hasattr(bench, "set_joint_position_target_index"):
            bench.set_joint_position_target_index(target=t, joint_ids=j)
        else:
            bench.set_joint_position_target(t, joint_ids=j)
        _M["steps"] += 1
    except Exception as e:  # WHY: never stop the run; no model step means no solder and so no full score
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


def flame_away(objects):
    t = objects["torch"]
    return float(np.linalg.norm(flame_tip(t.pos, t.quat_xyzw) - np.asarray(MARK)))


def wire_away(objects):
    w = objects["wire"]
    return float(np.linalg.norm(wire_tip(w.pos, w.quat_xyzw) - np.asarray(MOUTH)))


def failure(objects, m):
    """Latched: the flux burnt (the joint over BURN_T), the solder set round the wire's tip, a tool off the table, or
    the model not running (an error, or not stepping with the run)."""
    if m["burnt"]:
        return f"flux burnt (joint {m['T_max']:.0f} F)"
    if m["stuck"]:
        return "solder set round the wire"
    if m["flamed"]:
        return "flame on the joint's edge or the solder"
    for k in ("torch", "wire"):
        if float(objects[k].pos[2]) < LOST_Z:
            return f"{k} lost"
    if m["error"]:
        return "model " + m["error"]
    if _CLOCK["steps"] - m["steps"] > 5:
        return "model not stepping"
    return None


def tools_set(objects):
    """Each tool on its stand (its frame within ON_STAND of its start) or held up (its frame TOOL_Z or more over the
    table), not let go on the bench."""
    def ok(k, start):
        p = np.asarray(objects[k].pos, float)
        return float(np.linalg.norm(p - np.asarray(start))) <= ON_STAND or float(p[2]) >= TOP + TOOL_Z
    return ok("torch", G_REST) and ok("wire", WIRE_START)


def stage(objects, m):
    """0 nothing; 1 solder melted into the joint; 2 half full; 3 full; 4 full, set (under the solidus), the torch and
    the wire away."""
    if m["fed"] >= FULL:
        if (m["T"] < SOLIDUS and flame_away(objects) >= AWAY_FLAME and wire_away(objects) >= AWAY_WIRE
                and tools_set(objects)):
            return 4
        return 3
    if m["fed"] >= FULL / 2:
        return 2
    return 1 if m["fed"] > 0 else 0


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
    """The joint full, set, the torch and the wire away (held 1 s by yam_arena), with no latched failure."""
    return update(objects)["now"]["done"]


LADDER = {0: 0.0, 1: 0.25, 2: 0.5, 3: 0.6}
HOLD_S = 1.0


def score(objects):
    """At the last step: success held 1 s scores 1.0 (yam_arena also ends the trial then); 0 after a latched failure.
    Full, set and the tools away but not yet held 1 s: 0.8. Full: 0.6. Half full: 0.5. Melted in: 0.25."""
    m = update(objects)
    if m["fail"]:
        return 0.0
    n = m["now"]
    if n["stage"] == 4:
        return 1.0 if (n["done"] and m["held"] >= HOLD_S - 1e-9) else 0.8
    return LADDER[n["stage"]]


# -- demo -----------------------------------------------------------------------------------------------------------------
TIP = 0.013               # PAD_TIP_PAST_TCP: the pad tips sit 13 mm under the TCP
GRIP, OPEN_F = 0.0, -0.040
TCP_OVER_G = 0.007        # the left TCP over the torch's grip: the pad tips 6 mm under the handle's axis
TORCH_SAG = 0.0034        # the held torch sits this much lower than planned [RUN: traced runs 2026-11-14, all seeds]
TCP_OVER_COIL = 0.007     # the right TCP over the coil's middle: the pad tips 6 mm under it
TCP_OVER_TIP = TCP_OVER_COIL - TIP_LOCAL[2]   # the right TCP over the wire's tip
YAW_WIRE = math.pi / 2    # the right fingers close along x on the coil
WIRE_DRIFT = 0.0011       # the held wire's tip sits this much further out than planned [RUN: traced runs, hover]
R_WIRE = TUBE_R + WIRE_R - 0.0003 - WIRE_DRIFT   # aimed 0.3 mm into the top tube, so the tip rests against it
BACK = 0.040              # the flame comes in and goes back along its own axis (never past the cup's mouth)
W_XY = (JX + R_WIRE * U[0], R_WIRE * U[1])
PRELOAD = 0.0025          # a touch aims the tip this far under the face (2 N on its spring)
FEED_EXTRA = 0.0020       # past the full depth
FEED_SPEED = 0.004        # m/s
MELT_WAIT = 0.5           # s after the planned melt before the touch that feeds
DWELL = 0.5               # s the feeding touch rests before pushing on (it melts in as it rests)
AWAY_G = (0.07, 0.21, TOP + 0.10)


def _w(t, p, f, label, yaw):
    return Waypoint(float(t), tuple(float(v) for v in p), float(f), label, float(yaw))


class _Arm:
    def __init__(self):
        self.w, self.t = [], 0.0

    def add(self, dt, p, f, label, yaw):
        self.w.append(_w(dt, p, f, label, yaw))
        self.t += dt
        self.last = (tuple(p), f, yaw)

    def hold_to(self, t, label):
        if t > self.t + 1e-6:
            p, f, yaw = self.last
            self.add(t - self.t, p, f, label, yaw)


def _g_at(ws, t):
    """The torch's grip at time t along the left plan (linear between waypoints), from its TCP."""
    t0, prev = 0.0, None
    for w in ws:
        p = np.asarray(w.pos, float)
        if prev is None:
            prev = p
        if t <= t0 + w.duration:
            a = (t - t0) / w.duration if w.duration > 0 else 1.0
            q = prev + a * (p - prev)
            return q - np.array([0.0, 0.0, TCP_OVER_G + TORCH_SAG])
        t0 += w.duration
        prev = p
    return prev - np.array([0.0, 0.0, TCP_OVER_G + TORCH_SAG])


def heat_curve(left, idx, held_from, dt=0.05, t_end=None):
    """The joint's temperature along the left plan (the torch counted from ``held_from``, when it is in hand)."""
    m = _fresh_model()
    m["idx"] = idx
    out, t = [], 0.0
    t_end = sum(w.duration for w in left) if t_end is None else t_end
    while t <= t_end + 1e-9:
        g = _g_at(left, t) if t >= held_from else np.asarray(G_REST)
        tip = g - FLAME_TIP_Y * np.array([U[0], U[1], 0.0])   # local -y is along U
        advance(m, dt, tip, (0.0, 0.0, 1.0), 0.0)
        t += dt
        out.append((t, m["T"]))
    return out


def plan(delay=0.0, seed=None):
    """(left, right, info). Left takes the torch and holds its flame on the cup's base; right takes the wire, hovers it
    over the joint opposite the torch, touches it once before the joint is hot (it does not melt), then once the joint
    is hot touches again and feeds the wire down past the full depth; then both take their tools away."""
    idx = heat_index(seed)
    L, R = _Arm(), _Arm()
    yaw_t = PHI                        # the left fingers close across the handle
    g_rest_tcp = (G_REST[0], G_REST[1], G_REST[2] + TCP_OVER_G)
    L.add(2.0 + delay, (g_rest_tcp[0], g_rest_tcp[1], g_rest_tcp[2] + 0.05), OPEN_F, "over the torch", yaw_t)
    L.add(1.0, g_rest_tcp, OPEN_F, "down to the torch", yaw_t)
    L.add(0.6, g_rest_tcp, GRIP, "grip the torch", yaw_t)
    L.add(0.4, g_rest_tcp, GRIP, "settle on the torch", yaw_t)
    held_from = L.t
    L.add(1.0, (g_rest_tcp[0], g_rest_tcp[1], g_rest_tcp[2] + 0.03), GRIP, "lift the torch", yaw_t)
    heat_tcp = (G_HEAT[0], G_HEAT[1], G_HEAT[2] + TCP_OVER_G + TORCH_SAG)
    behind = (heat_tcp[0] - BACK * U[0], heat_tcp[1] - BACK * U[1], heat_tcp[2])
    L.add(2.0, behind, GRIP, "behind the heating place", yaw_t)
    L.add(1.0, heat_tcp, GRIP, "flame in on the cup's base", yaw_t)
    t_on = L.t
    L.add(60.0, heat_tcp, GRIP, "heat", yaw_t)          # trimmed below
    curve = heat_curve(L.w, idx, held_from)
    t_melt = next(t for t, temp in curve if temp >= LIQUIDUS)
    # right: take the wire from its holder and hover it over the joint, opposite the torch
    grip_z = WIRE_START[2] - 0.0005 + TCP_OVER_COIL      # the coil settles 0.5 mm onto the holder
    R.add(2.0 + delay, (HOLD_XY[0], HOLD_XY[1], grip_z + 0.04), OPEN_F, "over the solder", YAW_WIRE)
    R.add(1.0, (HOLD_XY[0], HOLD_XY[1], grip_z), OPEN_F, "down to the coil", YAW_WIRE)
    R.add(0.6, (HOLD_XY[0], HOLD_XY[1], grip_z), GRIP, "grip the coil", YAW_WIRE)
    R.add(0.4, (HOLD_XY[0], HOLD_XY[1], grip_z), GRIP, "settle on the coil", YAW_WIRE)
    R.add(1.2, (HOLD_XY[0], HOLD_XY[1], grip_z + HOLDER_H + 0.006), GRIP, "lift the end out of the well", YAW_WIRE)
    hover = TOP + Z_MOUTH + 0.005 + TCP_OVER_TIP
    R.add(1.5, (W_XY[0], W_XY[1] - 0.035, hover + 0.005), GRIP, "beside the joint", YAW_WIRE)
    R.add(1.0, (W_XY[0], W_XY[1], hover), GRIP, "over the joint's edge", YAW_WIRE)
    touch = TOP + STOP_TOP - PRELOAD + TCP_OVER_TIP
    t_test = max(R.t + 0.5, t_melt - 4.0)
    if t_test + 2.0 < t_melt - 0.5:
        R.hold_to(t_test, "wait")
        R.add(0.5, (W_XY[0], W_XY[1], touch), GRIP, "test touch", YAW_WIRE)
        R.add(1.0, (W_XY[0], W_XY[1], touch), GRIP, "hold the touch (it does not melt)", YAW_WIRE)
        R.add(0.5, (W_XY[0], W_XY[1], hover), GRIP, "take the wire off", YAW_WIRE)
    R.hold_to(t_melt + MELT_WAIT, "keep heating")
    R.add(0.5, (W_XY[0], W_XY[1], touch), GRIP, "touch: it melts", YAW_WIRE)
    R.add(DWELL, (W_XY[0], W_XY[1], touch), GRIP, "let it melt in", YAW_WIRE)
    deep = TOP + STOP_TOP - FULL - PRESS_D - FEED_EXTRA + TCP_OVER_TIP
    R.add((touch - deep) / FEED_SPEED, (W_XY[0], W_XY[1], deep), GRIP, "feed the wire in", YAW_WIRE)
    R.add(0.3, (W_XY[0], W_XY[1], deep), GRIP, "full", YAW_WIRE)
    t_full = R.t
    R.add(0.8, (W_XY[0], W_XY[1], hover + 0.025), GRIP, "lift the wire off", YAW_WIRE)
    R.add(1.0, (W_XY[0], W_XY[1] - 0.06, hover + 0.025), GRIP, "wire away", YAW_WIRE)
    # left: take the flame away as the wire comes off
    L.w.pop()
    L.t = t_on
    L.hold_to(t_full, "heat")
    L.add(0.6, behind, GRIP, "draw the flame back", yaw_t)
    L.add(1.5, (AWAY_G[0], AWAY_G[1], AWAY_G[2] + TCP_OVER_G + TORCH_SAG), GRIP, "torch away", yaw_t)
    t_end = max(L.t, R.t) + 12.0
    L.hold_to(t_end, "let it cool")
    R.hold_to(t_end, "let it cool")
    curve = heat_curve(L.w, idx, held_from)
    return L.w, R.w, {"t_melt": t_melt, "t_full": t_full, "held_from": held_from, "curve": curve}


def demo(delay=0.0):
    """Left takes the torch and holds its flame on the cup's base; right takes the wire, test-touches the joint before
    it is hot, then feeds the joint full once the solder melts; both take their tools away and the joint cools."""
    left, right, _ = plan(delay, run_seed())
    return {"left": left, "right": right}


def _replan_demo_after_reset():
    """WHY: inspect-robots resets the policy before the embodiment (inspect_robots/rollout.py), so yam_scripted would
    call demo() while run_seed() still holds the previous trial's seed and time the feed for the wrong heat rate. This
    makes yam_scripted plan again on each trial's first action, after the reset has set the seed: the scripted demo
    reads the seed, as a privileged script may. It changes only the scripted demo policy, never the scene or the score."""
    try:
        from yam_arena import scripted
    except ImportError:
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_solder_replan", False):
        return
    reset, act = cls.reset, cls.act
    unset = object()

    def patched_reset(self, scene):
        self._solder_scene, self._solder_seed = scene, unset
        reset(self, scene)

    def patched_act(self, observation):
        if getattr(self, "_solder_seed", None) is unset:
            self._solder_seed = run_seed()
            reset(self, self._solder_scene)
        return act(self, observation)

    cls.reset, cls.act, cls._solder_replan = patched_reset, patched_act, True


_replan_demo_after_reset()
