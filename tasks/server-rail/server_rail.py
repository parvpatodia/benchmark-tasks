"""On the station table: the middle (intermediate) member of a three-member ball-bearing slide, 250 mm long, lying fixed
on its side with its open channel facing up, along the robot's left-right axis 32 cm out from the arm bases. Its open
front end (the mouth) points to the robot's left, at y +8 cm; its back end is at y -17 cm. The channel is 32 mm wide
inside between grey steel walls, and two lips 23 mm above the table narrow its top opening to 26 mm. Inside the channel
sits the orange ball retainer: a 93.5 mm long cage carrying three pairs of steel rollers (one under and one over the
inner member's path at each station; the upper rollers are sprung down). The retainer slides along the channel between
a front stop 3 mm inside the mouth and a back stop; it starts 90 mm back from its front stop, and it stays where it is
left (about 7 N of drag). A grey stop block 30 mm inside the back end marks the inner member's home.

The blue inner member (250 x 30 x 8 mm, 150 g, its leading end tapered) lies flat on the table 22 cm out and left of
the midline, parallel to the slide, its tapered end pointing right (toward the mouth's direction of travel).

Push the retainer forward to its front stop first (a fingertip fits through the top opening behind its back bar). Then
insert the inner member's tapered end into the mouth between the rollers and slide it home against the stop block.
The rollers roll, so the retainer travels back at half the inner member's speed: from the front stop it has just enough
travel for the inner member to reach home. Inserted with the retainer back, the inner member runs into the retainer
and stalls 70 mm or more short of home.
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, mechanism, on_reset  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Asset, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HERE = pathlib.Path(__file__).resolve().parent
RAIL_ASSET = "server_rail_inner"


def _register():
    """Add the inner member (a USD file next to this one, written by make_meshes.py) to Arena's asset registry, so
    ``Asset`` can name it. Outside the simulator (tests, checks) there is nothing to register."""
    try:
        from isaaclab_arena.assets.registries import AssetRegistry
    except ImportError:
        return
    reg = AssetRegistry()
    if not reg.is_registered(RAIL_ASSET, ensure_loaded=False):
        reg.register(type(RAIL_ASSET, (), {"name": RAIL_ASSET, "tags": ["object"],
                                           "usd_path": str(HERE / f"{RAIL_ASSET}.usda")}), RAIL_ASSET)


_register()

# -- sizes (m). The slide frame: u along the channel from its back end (0) to the mouth (+), v across it, z up from the
# table. The mechanism's frame is this frame, turned +90 deg so that u runs along world +y. ---------------------------
SLIDE_L = 0.250                 # 3832-C10: slide length 250 mm [READ: Accuride 3832CDO data sheet]
RAIL_L, RAIL_W, RAIL_T = 0.250, 0.030, 0.008   # inner member; width and thickness [EST]
NOSE_L = 0.030                  # its tapered end (make_meshes.py)
WEB_T = 0.002                   # the intermediate member's web, lying on the table [EST]
WALL_IN = 0.016                 # half the channel's inside width: 1 mm side play for the 30 mm inner member [EST]
WALL_T = 0.002
LIP_Z0, LIP_Z1 = 0.0215, 0.023  # the lips' underside and top above the table
LIP_IN = 0.013                  # half the top opening (26 mm): narrower than the inner member, so it only goes in
                                # from the mouth, as on the real channel's curled edges
ROLL_R = 0.0025                 # the rollers stand for 5 mm balls [EST: light-duty slides use 4-5 mm balls]
ROLL_LEN = 0.023                # a roller's straight part (it is a capsule 28 mm end to end)
RAIL_Z0 = WEB_T + 2 * ROLL_R    # the inner member's underside when it rides on the lower rollers: 7 mm
RAIL_ZC = RAIL_Z0 + RAIL_T / 2  # its centre: 11 mm
PRELOAD_DIP = 0.001             # an upper roller hangs 1 mm into the inner member's path until it is pushed up
CAGE_Z0, CAGE_Z1 = 0.017, 0.0205  # the cage's frame: over the inner member's path, under the lips
SHELF_U = (0.240, SLIDE_L)      # the front guide: a block in the mouth 0.5 mm under the inner member's underside
U_HOME = 0.030                  # the inner member's tapered end at home, against the stop block
STOP_U = (0.022, U_HOME)        # the stop block
CAGE_L = 0.0935
STATIONS = (0.012, 0.047, 0.0815)  # roller stations from the cage's back end
FRONT_EDGE = STATIONS[-1] + ROLL_R  # the front lower roller's leading edge, from the cage's back end
CAGE_BACK0 = 0.0635             # the cage's back end at the start
Q_FRONT = 0.090                 # retainer travel from the start to the front stop
# WHY: rolling, the cage moves half as far as the inner member. From first contact at the front edge to home the inner
# member travels (CAGE_BACK0 + FRONT_EDGE + q) - U_HOME, so the cage ends at q_end = q/2 - (CAGE_BACK0 + FRONT_EDGE -
# U_HOME)/2. The back stop is set where a retainer 20 mm short of its front stop just reaches home; any farther back and
# the cage hits its stop first, its rollers cannot turn, and the inner member stalls. From the front stop the cage
# ends 10 mm before its back stop.
K_MAX = 0.020
U_FE0 = CAGE_BACK0 + FRONT_EDGE  # front edge at the start: 147.5 mm
Q_BACK = (Q_FRONT - K_MAX) / 2 - (U_FE0 - U_HOME) / 2  # -23.75 mm
U_ENTRY = U_FE0 + Q_FRONT       # the front edge with the retainer at its front stop: 237.5 mm
# WHY: the real balls are a close fit and cannot skid without damage. Each upper roller is pressed down with about 40 N
# so the YAM arm cannot skid a stalled roller pair either (1.2 x 40 = 48 N), while rolling them costs only the climb
# up the member's 4.8 deg taper (40 N x tan 4.8 deg = 3.3 N) [EST, sized for the arm, not read from a slide]
ROLLER_MASS = 0.004              # kg per roller (a steel roller 5 mm x 28 mm is 4 g)
TOP_PRELOAD_K, TOP_PRELOAD_T = 500.0, -0.080
# WHY: a retainer stays where it is left. Its drag is a pad pressed on the web with 6 N, so 7.2 N slides it: more than
# twice the 3.3 N climb, so an upper roller climbing the taper never shoves the retainer back [EST]
SHOE_K, SHOE_T = 1000.0, -0.0065

# -- layout (world) --------------------------------------------------------------------------------------------------
X_C, Y0 = 0.32, -0.17           # the channel's centre line, and the slide's back end
YAW = math.pi / 2
RAIL_START = (0.22, 0.20)       # the inner member's centre, lying on the table, tapered end toward -y
STEEL, DARK, ORANGE, WOOD = (0.62, 0.63, 0.66), (0.30, 0.31, 0.34), (1.0, 0.45, 0.0), (0.62, 0.45, 0.25)
MATERIALS = {"default": (1.2, 1.0)}
RAIL_MASS = 0.15                # [EST] the hull is 57.4 cm3: aluminium (a real member is steel, about 450 g)


def world(u, v, z):
    """The slide frame to world."""
    return (X_C - v, Y0 + u, TOP + z)


def slide_box(name, u, v, z, color=STEEL):
    """A fixed box spanning u, v and z (pairs, slide frame)."""
    c = world((u[0] + u[1]) / 2, (v[0] + v[1]) / 2, (z[0] + z[1]) / 2)
    return Box(name, pos=c, size=(v[1] - v[0], u[1] - u[0], z[1] - z[0]), color=color)


def _box(c, size, color):
    return Shape("box", tuple(float(x) for x in c), tuple(float(x) for x in size), color, contact_offset=0.001)


def _roller(u, z, color=STEEL):
    return Shape("capsule", (float(u), 0.0, float(z)), (ROLL_R, ROLL_LEN), color, axis="Y", contact_offset=0.001)


def _cage_shapes():
    b = CAGE_BACK0
    zc, h = (CAGE_Z0 + CAGE_Z1) / 2, CAGE_Z1 - CAGE_Z0
    return (
        _box((b + 0.002, 0, zc), (0.004, 0.029, h), ORANGE),                      # back bar: push here
        _box((b + CAGE_L - 0.002, 0, zc), (0.004, 0.029, h), ORANGE),             # front bar
        _box((b + CAGE_L / 2, 0.01275, zc), (CAGE_L, 0.0035, h), ORANGE),         # side strips
        _box((b + CAGE_L / 2, -0.01275, zc), (CAGE_L, 0.0035, h), ORANGE),
    )


def _links_and_joints():
    b = CAGE_BACK0
    z_low = WEB_T + ROLL_R                      # a lower roller resting on the web
    z_top = RAIL_Z0 + RAIL_T - PRELOAD_DIP + ROLL_R  # an upper roller hanging at its stop
    links = [Link("cage", 0.02, _cage_shapes()),
             Link("shoe", 0.002, (_box((b + 0.028, 0, WEB_T + 0.0015), (0.012, 0.016, 0.002), ORANGE),))]
    joints = [Joint("retainer", "prismatic", "cage", (0, 0, 0), "X", Q_BACK, Q_FRONT, damping=0.5),
              Joint("shoe", "prismatic", "shoe", (b + 0.028, 0, WEB_T + 0.0015), "Z", -0.002, 0.001,
                    stiffness=SHOE_K, damping=2.0, target=SHOE_T, parent="cage")]
    for k, s in enumerate(STATIONS, 1):
        u = b + s
        # WHY: each roller hangs from a carrier that slides up and down in the cage, as a ball floats in its pocket, so
        # the load passes through the roller into the web and the inner member and its turning carries the cage along
        links += [Link(f"low_carrier_{k}", 0.001, (_box((u, 0, z_low), (0.002, 0.010, 0.002), DARK),)),
                  Link(f"low_roller_{k}", ROLLER_MASS, (_roller(u, z_low),)),
                  Link(f"top_carrier_{k}", 0.001, (_box((u, 0, z_top), (0.002, 0.010, 0.002), DARK),)),
                  Link(f"top_roller_{k}", ROLLER_MASS, (_roller(u, z_top),))]
        joints += [Joint(f"low_lift_{k}", "prismatic", f"low_carrier_{k}", (u, 0, z_low), "Z", -0.001, 0.001,
                         damping=0.2, parent="cage"),
                   Joint(f"low_spin_{k}", "revolute", f"low_roller_{k}", (u, 0, z_low), "Y", -math.inf, math.inf,
                         damping=1e-5, parent=f"low_carrier_{k}"),
                   Joint(f"top_lift_{k}", "prismatic", f"top_carrier_{k}", (u, 0, z_top), "Z", -0.0002, 0.0015,
                         stiffness=TOP_PRELOAD_K, damping=0.5, target=TOP_PRELOAD_T, parent="cage"),
                   Joint(f"top_spin_{k}", "revolute", f"top_roller_{k}", (u, 0, z_top), "Y", -math.inf, math.inf,
                         damping=1e-5, parent=f"top_carrier_{k}")]
    return tuple(links), tuple(joints)


_LINKS, _JOINTS = _links_and_joints()
RETAINER = mechanism(
    "retainer", world(0, 0, 0), yaw=YAW,
    # the base is the slide's back bracket; the channel itself is fixed boxes, so the rollers roll on it (a mechanism's
    # own links never touch its base)
    base=Link("bracket", 0.1, (_box((-0.007, 0, 0.0115), (0.010, 0.036, 0.023), DARK),)),
    links=_LINKS, joints=_JOINTS, materials=MATERIALS,
    # WHY: the rollers sit inside the cage and its carriers by design; they only need to touch the web and the inner
    # member, which are separate bodies
    self_collide=False,
)

OBJECTS = [
    RETAINER,
    slide_box("web", (0, SLIDE_L), (-WALL_IN - WALL_T, WALL_IN + WALL_T), (0, WEB_T)),
    slide_box("wall_l", (0, SLIDE_L), (WALL_IN, WALL_IN + WALL_T), (WEB_T, LIP_Z0)),
    slide_box("wall_r", (0, SLIDE_L), (-WALL_IN - WALL_T, -WALL_IN), (WEB_T, LIP_Z0)),
    slide_box("lip_l", (0, SLIDE_L), (LIP_IN, WALL_IN + WALL_T), (LIP_Z0, LIP_Z1)),
    slide_box("lip_r", (0, SLIDE_L), (-WALL_IN - WALL_T, -LIP_IN), (LIP_Z0, LIP_Z1)),
    slide_box("guide", SHELF_U, (-WALL_IN + 0.001, WALL_IN - 0.001), (WEB_T, RAIL_Z0 - 0.0005), DARK),
    slide_box("home_stop", STOP_U, (-WALL_IN + 0.0001, WALL_IN - 0.0001), (WEB_T, LIP_Z0), DARK),
    Asset("inner_rail", RAIL_ASSET, xy=RAIL_START, extent=RAIL_L, mass=RAIL_MASS, yaw=-math.pi / 2, convex=True),
]


# -- scoring ---------------------------------------------------------------------------------------------------------
def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_slide(p):
    """World to the slide frame (u, v, z)."""
    return np.array([float(p[1]) - Y0, X_C - float(p[0]), float(p[2]) - TOP])


HOME_TOL = 0.002        # the tapered end within 2 mm of the stop block
FORWARD_K = 0.015       # the retainer counts as forward within 15 mm of its front stop
LAT_TOL = 0.0015        # the inner member's centre off the channel's centre line (1 mm of side play)
Z_TOL = 0.0015          # its centre above or below riding height: off the rollers by more is not between them
YAW_TOL = 1.5           # degrees off the channel's line, turned flat
PITCH_TOL = 1.5         # degrees, nose up or down
CONTACT = 0.002         # the tapered end this close to the front lower roller's leading edge meets the retainer
ENTRY_ANG = 15.0        # degrees: an end inside the channel cannot be more than about 6 deg off its line at the shallowest
                        # strike, so a looser gate cannot miss one; it keeps out an end dipped in through the top opening
ROLL_SLACK = 0.005      # how far behind pure rolling from a forward start the retainer may lag before it counts as back
STILL_V, STILL_W = 0.01, 0.5  # m/s, rad/s (as ObjectState.resting)


def rail_pose(objects):
    """(centre, tapered end, both in the slide frame; yaw and pitch in degrees, 0 along the channel toward its back)."""
    o = objects["inner_rail"]
    a = _rot(o.quat_xyzw)[:, 0]        # the member's x axis, pointing to its tapered end
    c = np.asarray(o.pos, dtype=float)
    au = np.array([a[1], -a[0], a[2]])  # the axis in the slide frame
    yaw = math.degrees(math.atan2(au[1], -au[0]))
    pitch = math.degrees(math.asin(float(np.clip(au[2], -1.0, 1.0))))
    return to_slide(c), to_slide(c + a * RAIL_L / 2), yaw, pitch


def retainer_q(objects):
    """The retainer's travel from its start (m, + toward the mouth)."""
    return float(objects["retainer"].joints["retainer"])


def front_edge(objects):
    return U_FE0 + retainer_q(objects)


def forward(objects):
    return retainer_q(objects) >= Q_FRONT - FORWARD_K


def aligned(objects):
    c, _, yaw, pitch = rail_pose(objects)
    return bool(abs(c[1]) <= LAT_TOL and abs(c[2] - RAIL_ZC) <= Z_TOL and abs(yaw) <= YAW_TOL
                and abs(pitch) <= PITCH_TOL)


def leading_end(objects):
    """(the end that went into the channel first, as a slide-frame point, or None): the tapered end, or the square end
    for a member put in backwards. An end counts when it is inside the channel's cavity (between the walls, under the
    lips, between the back end and the mouth) with the member within ENTRY_ANG of the channel's line."""
    c, n, yaw, pitch = rail_pose(objects)
    if abs(pitch) > ENTRY_ANG:
        return None
    for end, off in ((n, abs(yaw)), (2 * c - n, 180.0 - abs(yaw))):
        if off <= ENTRY_ANG and abs(end[1]) <= WALL_IN and -0.001 <= end[2] <= LIP_Z0 and 0.0 <= end[0] <= SLIDE_L:
            return end
    return None


def touching(objects):
    """An end of the member is inside the channel and at or behind the retainer's front rollers."""
    e = leading_end(objects)
    return bool(e is not None and e[0] <= front_edge(objects) + CONTACT)


def meets(objects):
    """An end of the member is inside the channel and at or behind the cage's front bar, the retainer's part nearest
    the mouth. WHY: the bar sits 9.5 mm ahead of the front rollers, 17 to 20.5 mm up; a member carried 4.5 mm or more
    high passes under the lips but strikes the bar first, and that strike also shoves the retainer (hole hunt 3)."""
    e = leading_end(objects)
    return bool(e is not None and e[0] <= CAGE_BACK0 + CAGE_L + retainer_q(objects) + CONTACT)


def rolled_back(objects):
    """The retainer is further back than rolling from where it was at the first meeting allows, for how far the
    inside end has gone past the front rollers since. Rolling moves it half as far as the end; a shove by an end
    against the bar or the rollers' faces moves it as far, and trips this after 10 mm (5 mm at the bar)."""
    e = leading_end(objects)
    start = min(_MEM["e0"], U_FE0 + _MEM["q0"] + CONTACT)   # WHY: rolling starts at the rollers, not at the bar
    depth = max(0.0, start - float(e[0]))
    return retainer_q(objects) < _MEM["q0"] - depth / 2 - ROLL_SLACK


def engaged(objects):
    """Touching and in riding position. aligned() holds the member within 1.5 deg of nose-first, so a member put in
    square end first is never engaged (its square end inside with the nose behind the back end would need to pass
    through the stop block)."""
    return touching(objects) and aligned(objects)


def home(objects):
    _, n, _, _ = rail_pose(objects)
    return bool(n[0] <= U_HOME + HOME_TOL)


# WHY: the real consequence (Accuride US 8,500,223 B2): with the retainer not forward the inner member strikes it and can
# damage it. The sim shows the stall that follows, but a strong push that skids the rollers, or pulling back and trying
# again, would hide the strike, so the order is remembered. Each time an end first meets the retainer (its front bar or
# rollers), the retainer must be forward; one miss is kept for the rest of the trial. Every reset clears it.
_MEM = {"touching": False, "ok": False, "bad": False, "q0": 0.0, "e0": 0.0}


@on_reset
def reset_memory():
    _MEM.update(touching=False, ok=False, bad=False, q0=0.0, e0=0.0)


def order(objects):
    """Update and return the memory: (met the retainer forward at least once, ever met it while it was back). Calling
    it twice on one state gives the same answer."""
    t = meets(objects)
    if t and not _MEM["touching"]:
        _MEM["q0"], _MEM["e0"] = retainer_q(objects), float(leading_end(objects)[0])
        if forward(objects):
            _MEM["ok"] = True
        else:
            _MEM["bad"] = True
    if t and rolled_back(objects):
        _MEM["bad"] = True       # WHY: the retainer was driven back while an end was already against it
    _MEM["touching"] = t
    return _MEM["ok"], _MEM["bad"]


def _still(objects):
    cage = objects["retainer"].links["cage"]
    return bool(objects["inner_rail"].resting(STILL_V, STILL_W) and float(np.linalg.norm(cage.lin_vel)) < STILL_V)


def done(objects):
    ok, bad = order(objects)
    return bool(ok and not bad and engaged(objects) and home(objects))


def success(objects):
    """Home in the right order, square in the channel, and the inner member and retainer still. Holds for 1 s."""
    return done(objects) and _still(objects)


def score(objects):
    """Partial credit (success, held for 1 s, scores 1.0 and ends the trial). At the last step: 0.9 home in the right
    order and still; 0.75 the same but moving; 0.3 to 0.7 between the rollers in the right order, by how far in; 0.2
    the retainer forward; else 0. Meeting the retainer while it is back scores 0 for the rest of the trial."""
    ok, bad = order(objects)
    if bad:
        return 0.0
    if ok and engaged(objects):
        if home(objects):
            return 0.9 if _still(objects) else 0.75
        _, n, _, _ = rail_pose(objects)
        progress = float(np.clip((U_ENTRY - n[0]) / (U_ENTRY - U_HOME), 0.0, 1.0))
        return 0.3 + 0.4 * progress
    if ok or forward(objects):
        return 0.2               # WHY: the first step was done; a member then held off line must not score below it
    return 0.0


# -- demo ------------------------------------------------------------------------------------------------------------
# WHY (finger hulls, by FK, yam_arena.analysis): closed, the pad tips are 15.4 x 8 mm for 13 mm above their ends, and the
# fingers widen to 38-40 x 20 mm from the TCP up. The TCP is 13 mm above the tips.
TIP = 0.013
PUSH_TIP_Z = 0.0145      # the pusher's tip end: under the cage's back bar (17 to 20.5 mm); its wide part, from the
                         # TCP 27.5 mm up, clears the lips' 23 mm top by 3 mm (FK hulls, test_fingers_clear_the_slide)
GRIP_FROM_END = 0.125    # the grip's centre from the square end: the middle
# WHY (sim, p_ok round 1): gripped 12 mm from its end, the member pivoted about the pads' axis (the fingers close across
# its width, so that axis is its pitch axis) under 0.17 N m of its own weight, hung 11 deg nose down onto the table and
# met the guide's face 3 mm low. Held at its middle it carries no weight torque.
RAIL_GRIP_FINGER = -0.012  # pads 24 mm apart on the 30 mm member: each presses about 3 mm in, under the 50 N cap
RIDE_AIM = 0.0035        # carry the member 3.5 mm high for the arm's sag: it sagged 2.6 mm (p_ok round 2); aimed 2.5
                         # and 3.5 mm high the demo scored 1.0, aimed 1.5 mm it wedged (round 4 margins), so 3.5 sits
                         # 2 mm from that failure
AIM_X = 0.003            # aim the left arm 3 mm to +x at the slide: in p_ok round 2 the member sat 3.4 mm to -x of
                         # the channel's line (1.1 mm shifted in the grip, the rest arm error), and its tapered end
                         # caught the wall's end face; the taper and play capture 3.5 mm
PUSH_OVER = 0.050        # aim the pusher 50 mm past the front stop: aimed 20 mm past, the cage stopped 13 mm short
                         # (the arm lagged 33 mm against the drag, p_ok rounds 1 and 2)
END_OVER = 0.080         # aim the end push 80 mm past home: aimed 50 mm past, it stopped 8.9 mm short of home in one
                         # margin run (round 6, aim 1 mm off); the member's end stops the tips at home, so they never
                         # get there
PUSH_FROM_NOSE = 0.200   # start the end push behind the end for a member that stopped anywhere up to this nose
                         # position (p_ok round 3: 0.180, short of the 0.145 aimed)
PUSH_TIP_LOW = 0.003     # the end push's tip bottom over the table: the tips (13 mm tall) then cover the end whether
                         # the member rides level (end 7 to 15 mm up) or lies tilted with its end on the table
                         # (p_wrong round 3: after letting go short of the rollers it tipped, end on the table)
HOVER = 0.08


def demo(delay=0.0):
    """Right arm: fingertip down through the top opening behind the retainer's back bar, push it 90 mm to its front
    stop (aiming 50 mm past), lift out. Left arm, meanwhile: pick up the inner member at its middle, then line it
    up with the mouth, slide it in until two roller pairs hold it, let go, and push its end home with the
    closed fingertips."""
    def r_at(u, z):
        return (X_C, Y0 + u, TOP + z)

    back = CAGE_BACK0
    push_from = back - 0.004 - 0.003             # tip centre 3 mm behind the back bar
    push_to = back + Q_FRONT - 0.004 + PUSH_OVER  # the bar at its front stop, plus PUSH_OVER
    right = [
        Waypoint(2.0 + delay, r_at(push_from, HOVER), FINGER_CLOSED, "over the retainer", 0.0),
        Waypoint(1.0, r_at(push_from, HOVER), FINGER_CLOSED, "settle", 0.0),
        Waypoint(1.5, r_at(push_from, PUSH_TIP_Z + TIP), FINGER_CLOSED, "tip into the channel", 0.0),
        Waypoint(0.5, r_at(push_from, PUSH_TIP_Z + TIP), FINGER_CLOSED, "settle", 0.0),
        Waypoint(3.0, r_at(push_to, PUSH_TIP_Z + TIP), FINGER_CLOSED, "push the retainer forward", 0.0),
        Waypoint(1.0, r_at(push_to, PUSH_TIP_Z + TIP), FINGER_CLOSED, "hold at the stop", 0.0),
        Waypoint(1.5, r_at(push_to, HOVER), FINGER_CLOSED, "lift out", 0.0),
        Waypoint(2.0, (0.25, -0.25, TOP + 0.15), FINGER_CLOSED, "park", 0.0),
    ]
    yaw = -math.pi / 2  # fingers close across the member's width (world x); -90 deg keeps the wrist in range here
    gx, gy = RAIL_START[0], RAIL_START[1] + RAIL_L / 2 - GRIP_FROM_END
    grasp_z = TOP + 0.0005 + TIP                 # tips 0.5 mm over the table
    lift_z = TOP + 0.060

    def l_at(u_nose, z_bottom):
        """TCP for the member's tapered end at u_nose (slide frame) with its underside at z_bottom over the table."""
        return (X_C + AIM_X, Y0 + u_nose + RAIL_L - GRIP_FROM_END, TOP + z_bottom + 0.0005 + TIP)

    ride = RAIL_Z0 + RIDE_AIM
    # WHY (sim): fed gripped, the member hangs at nose 0.2135 while the front rollers climb its taper, then breaks
    # through and rolls; the deep target is what drives it through (let go at the hang, the end push alone could not,
    # round 5). The arm reaches about 0.18 and lets go there with two roller pairs holding the member.
    release_nose = 0.145
    end_home = U_HOME + RAIL_L
    push_tip0 = PUSH_FROM_NOSE + RAIL_L + 0.0077 + 0.008  # closed tips' centre, behind the end
    push_tip1 = end_home + 0.0077 - END_OVER      # END_OVER past the end at home
    push_z = TOP + PUSH_TIP_LOW + TIP
    left = [
        Waypoint(2.0 + delay, (gx, gy, TOP + HOVER), FINGER_OPEN, "over the inner member", yaw),
        Waypoint(1.0, (gx, gy, TOP + HOVER), FINGER_OPEN, "settle", yaw),
        Waypoint(1.5, (gx, gy, grasp_z), FINGER_OPEN, "descend", yaw),
        Waypoint(1.0, (gx, gy, grasp_z), RAIL_GRIP_FINGER, "grip", yaw),
        Waypoint(1.5, (gx, gy, lift_z), RAIL_GRIP_FINGER, "lift", yaw),
        Waypoint(6.0, (gx, gy, lift_z), RAIL_GRIP_FINGER, "wait for the retainer", yaw),
        Waypoint(2.5, l_at(SLIDE_L + 0.008, 0.046), RAIL_GRIP_FINGER, "carry over the mouth's line", yaw),
        Waypoint(1.0, l_at(SLIDE_L + 0.008, 0.046), RAIL_GRIP_FINGER, "settle", yaw),
        Waypoint(1.5, l_at(SLIDE_L + 0.008, ride), RAIL_GRIP_FINGER, "lower to riding height", yaw),
        Waypoint(1.0, l_at(SLIDE_L + 0.008, ride), RAIL_GRIP_FINGER, "settle", yaw),
        Waypoint(5.0, l_at(release_nose, ride), RAIL_GRIP_FINGER, "slide in", yaw),
        Waypoint(1.0, l_at(release_nose, ride), RAIL_GRIP_FINGER, "settle", yaw),
        Waypoint(0.8, l_at(release_nose, ride), -0.025, "let go", yaw),
        Waypoint(1.0, l_at(release_nose, 0.050), -0.025, "up", yaw),
        Waypoint(1.0, (X_C + AIM_X, Y0 + push_tip0, TOP + 0.050), FINGER_CLOSED, "behind the end, fingers closed", yaw),
        Waypoint(1.0, (X_C + AIM_X, Y0 + push_tip0, push_z), FINGER_CLOSED, "down to the end", yaw),
        Waypoint(0.5, (X_C + AIM_X, Y0 + push_tip0, push_z), FINGER_CLOSED, "settle", yaw),
        Waypoint(6.0, (X_C + AIM_X, Y0 + push_tip1, push_z), FINGER_CLOSED, "push home", yaw),
        Waypoint(1.5, (X_C + AIM_X, Y0 + push_tip1, push_z), FINGER_CLOSED, "hold", yaw),
        Waypoint(2.0, (X_C + AIM_X, Y0 + push_tip0, push_z), FINGER_CLOSED, "back off", yaw),
        Waypoint(1.0, (X_C + AIM_X, Y0 + push_tip0, TOP + HOVER), FINGER_CLOSED, "up", yaw),
    ]
    return {"left": left, "right": right}
