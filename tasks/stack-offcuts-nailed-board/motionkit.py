# robocurve-kit motionkit.py 211ef14add90
"""motionkit: the demo side of a yam_arena task, for the stock ``yam_scripted`` policy. NumPy only.

A task builds its demo as one Plan per arm and returns the waypoints from ``demo()``, so
``inspect-robots run --policy yam_scripted`` plays it unchanged (the film and Dream runs stay stock):

    from motionkit import Plan, down, together, plans_out, SagCorrector, install, mode
    install(SagCorrector())                  # optional: trim droop from the observed joints, see SagCorrector
    def plan(seed=None, mode_name=None):
        L, R = Plan("left"), Plan("right")
        L.pick((0.30, 0.20, TOP + 0.02), h=0.0, width=0.04, grip=down(-90))
        together(L, R)                       # both arms wait for each other here
        L.place((0.30, 0.05, TOP + 0.02), h=0.0); L.release_clear()
        return plans_out(L, R)
    def demo():
        return plan(run_seed(), mode())      # mode(): the cheat mode from TASK_DEMO, "demo" by default

Grips are TCP rotations carried inside a Waypoint's ``yaw`` (a float subclass), which this module teaches
``yam_arena.demo.grip_rotation`` to read; a plain float yaw still means the stock straight-down grip.

HAND is the one gripper model: points on the URDF collision meshes of the gripper and both finger tips (what PhysX
collides), used by the scorer (``HAND.from_env``), by checkkit.py and by the tests (``HAND.world`` / ``HAND.from_q``).

Credits: the grip constructors (down, tilted, level), the Grip float and the speed defaults follow demokit.py by K-XZY
(robocurve/benchmark-tasks tasks/chalk-line, MIT, per that task's README); see fastrun.py for the licence text.
Units: metres, seconds; grip angles in degrees. Positions are world TCP positions.
"""
from __future__ import annotations

import functools
import math
import os
import pathlib
import warnings
import xml.etree.ElementTree as ET

import numpy as np
import yam_arena.demo as harness_demo
from yam_arena import YAM_URDF, ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN, GRIPPER, READY, SHOULDER, TCP_OFFSET, WRIST
from yam_arena.demo import Waypoint
from yam_arena.urdf import axis_angle_matrix, forward_kinematics, rpy_matrix

ARMS = ("left", "right")
TOP = station.TABLE_TOP
SPEED = 0.25          # m/s for a move with no time given [demokit]
TURN_SPEED = 45.0     # deg/s for the wrist [demokit: faster turns dip the fingers]
MIN_MOVE_S = 0.4
IK_WARN = (5.0, 3.0)  # mm, deg: a waypoint whose IK misses by more is printed [demokit]
STEADY_S = 0.4        # hold over a part before going down to it (LESSONS: the arm lags 2 to 10 mm on a descent)
CLOSE_S = 0.8         # a grip closes over at least this long (LESSONS: sim fingers close at about 47 mm/s)
PRESHAPE = 0.012      # open fingers this far clear of each side of a part before a grip [LESSONS]
LOAD_SETTLE_S = 1.0   # hold after a lift so SagCorrector can find the load's droop before the carry
ARM_K = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)   # N m/rad, joints 1 to 6
MESH_STEP = 0.003     # m between sampled points on the collision meshes
HAND_LINKS = ("gripper", "tip_left", "tip_right")
ARM_LINKS = ("base", "link1", "link2", "link3", "link4", "link5", *HAND_LINKS)   # every link with a collider
CONTACT = {"left": set(), "right": set()}   # labels Plan marked as contact stretches, per arm (SagCorrector freezes)
GRASP = {"left": {}, "right": {}}          # close label -> part name, from pick(name): checkkit carries that part
RELEASE = {"left": set(), "right": set()}  # labels where the held part is let go
# WHY: numpy 2.x on some BLAS builds (seen on the Mac) warns "divide by zero encountered in matmul" on finite inputs;
# the results are finite (tests/test_motionkit.py checks the sampled meshes), so only that message is silenced
warnings.filterwarnings("ignore", message=r".*encountered in matmul", category=RuntimeWarning)


def mode(default: str = "demo") -> str:
    """The cheat mode for this trial: env TASK_DEMO (fastrun.py sets it per mode), else `default`."""
    return os.environ.get("TASK_DEMO") or default


# -- grips -----------------------------------------------------------------------------------------------------------

class Grip(float):
    """A waypoint yaw that also carries a whole TCP rotation (columns: closing axis, pad axis, approach)."""

    rot: np.ndarray = np.eye(3)


def as_grip(rot) -> Grip:
    g = Grip(math.atan2(float(rot[1][0]), float(rot[0][0])))
    g.rot = np.asarray(rot, float)
    return g


def _install_grips() -> None:
    """Teach yam_arena.demo.grip_rotation (which ArmPlan calls per waypoint) to read a Grip. Plain floats unchanged."""
    if getattr(harness_demo.grip_rotation, "_takes_grips", False):
        return
    plain = harness_demo.grip_rotation

    def grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return plain(yaw) if rot is None else np.array(rot)

    grip_rotation._takes_grips = True
    grip_rotation._plain = plain
    harness_demo.grip_rotation = grip_rotation


_install_grips()
_plain_grip = harness_demo.grip_rotation._plain


def down(yaw_deg: float = 0.0) -> Grip:
    """Straight down, turned about the vertical. Yaw 0 closes the fingers along world y."""
    return as_grip(_plain_grip(math.radians(yaw_deg)))


def tilted(theta_deg: float, phi_deg: float, flip: bool = False) -> Grip:
    """Tilted theta from straight down toward the direction phi from +x, fingers closing across it. flip turns the
    gripper over about its own axis."""
    phi = math.radians(phi_deg)
    r = axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.radians(theta_deg)) @ _plain_grip(phi)
    return as_grip(r @ axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.pi) if flip else r)


def level(phi_deg: float, flip: bool = False) -> Grip:
    """Fingers level, pointing along phi from +x."""
    return tilted(90.0, phi_deg, flip)


def finger_for(width: float | None, force: float = 0.0) -> float:
    """Finger joint target that closes on a part `width` across with about `force` N a finger (None: fully)."""
    if width is None:
        return FINGER_CLOSED
    return float(np.clip(-(width / 2 - force / GRIPPER["stiffness"]), FINGER_OPEN, FINGER_CLOSED))


def preshape_for(width: float | None) -> float:
    """Fingers open PRESHAPE clear of each side of a part `width` across (fully open if None or too wide)."""
    if width is None:
        return FINGER_OPEN
    return float(np.clip(-(width / 2 + PRESHAPE), FINGER_OPEN, FINGER_CLOSED))


# -- meshes and the hand model ---------------------------------------------------------------------------------------

def _stl(path: pathlib.Path) -> np.ndarray:
    b = path.read_bytes()
    if b[:5] == b"solid" and b"facet" in b[:300]:
        v = [list(map(float, ln.split()[1:4])) for ln in b.decode().splitlines() if ln.strip().startswith("vertex")]
        return np.array(v, float).reshape(-1, 3, 3)
    n = int.from_bytes(b[80:84], "little")
    a = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(float)


def _sample(tri: np.ndarray, step: float) -> np.ndarray:
    """Points on each triangle's face about `step` apart, vertices included."""
    out = [tri.reshape(-1, 3)]
    for a, b, c in tri:
        n = int(max(np.linalg.norm(b - a), np.linalg.norm(c - a)) / step) + 1
        if n < 2:
            continue
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
        m = (i + j) <= n
        u, v = i[m] / n, j[m] / n
        out.append(a + u[:, None] * (b - a) + v[:, None] * (c - a))
    return np.vstack(out)


@functools.lru_cache(maxsize=None)
def link_points(link: str, step: float = MESH_STEP) -> np.ndarray:
    """A link's collision meshes (yam_arm.urdf, the URDF yam_arena loads), sampled on their faces, link frame."""
    root = ET.parse(YAM_URDF).getroot()
    base = pathlib.Path(YAM_URDF).parent
    pts = []
    for ln in root.iter("link"):
        if ln.get("name") != link:
            continue
        for col in ln.iter("collision"):
            o = col.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
            rpy = np.array([float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()])
            pts.append(_sample(_stl(base / col.find("geometry/mesh").get("filename")), step) @ rpy_matrix(rpy).T + xyz)
    if not pts:
        raise KeyError(f"no collision mesh for link {link!r}")
    p = np.vstack(pts)
    # WHY one point per cell: the meshes are dense; this keeps distances within about `step`
    _, keep = np.unique(np.floor(p / step).astype(np.int64), axis=0, return_index=True)
    return p[np.sort(keep)]


def link_poses(side: str, q6, finger: float, base=None) -> dict[str, np.ndarray]:
    """World 4x4 pose of every link of one arm at joints q6 and finger opening `finger` (m, <= 0)."""
    q = {n: float(v) for n, v in zip(ARM_JOINTS, q6)}
    q["joint7"] = q["joint8"] = float(finger)
    shift = np.eye(4)
    shift[:3, 3] = station.BASES[side] if base is None else base
    return {k: shift @ v for k, v in forward_kinematics(YAM_URDF, q).items()}


def arm_points(side: str, q6, finger: float, links=ARM_LINKS, step: float = MESH_STEP, base=None):
    """(points (n, 3), link name per point) of one arm's sampled colliders in the world."""
    poses = link_poses(side, q6, finger, base)
    pts, names = [], []
    for k in links:
        lp = link_points(k, step)
        pts.append(lp @ poses[k][:3, :3].T + poses[k][:3, 3])
        names += [k] * len(lp)
    return np.vstack(pts), np.array(names)


def _rot_xyzw(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=np.float64)


class Hand:
    """The gripper (housing and both finger tips) as points on its URDF collision meshes."""

    def __init__(self, step: float = MESH_STEP):
        self.step = step

    def local(self, finger: float) -> np.ndarray:
        """Points in the TCP frame (x closing, y pad axis, z approach) with the fingers at `finger`."""
        return _hand_local(round(float(finger), 6), self.step)

    def world(self, tcp_pos, rot, finger: float) -> np.ndarray:
        return np.asarray(tcp_pos, float) + self.local(finger) @ np.asarray(rot, float).T

    def from_q(self, side: str, q6, finger: float, base=None) -> np.ndarray:
        poses = link_poses(side, q6, finger, base)
        return np.vstack([link_points(k, self.step) @ poses[k][:3, :3].T + poses[k][:3, 3] for k in HAND_LINKS])

    def from_link_poses(self, poses: dict) -> np.ndarray:
        """From {link: (pos (3,), quat_xyzw (4,))} of the three hand links, as the simulator reports them."""
        out = []
        for k in HAND_LINKS:
            p, q = poses[k]
            out.append(link_points(k, self.step) @ _rot_xyzw(q).T + np.asarray(p, float))
        return np.vstack(out)

    def from_env(self, env, side: str) -> np.ndarray:
        """This hand's points now, from the simulator's link poses (Isaac Lab 3.0, quaternions xyzw), in the task's
        world frame (env origin removed). For an on_step hook; reads the robot only, never an object."""
        base = env.unwrapped
        origin = _np(base.scene.env_origins)[0]
        rob = base.scene[f"{side}_robot"]
        poses = _np(rob.data.body_link_pose_w)[0]
        names = list(rob.body_names)
        return self.from_link_poses({k: (poses[names.index(k)][:3] - origin, poses[names.index(k)][3:7])
                                     for k in HAND_LINKS})


@functools.lru_cache(maxsize=64)
def _hand_local(finger: float, step: float) -> np.ndarray:
    poses = forward_kinematics(YAM_URDF, {"joint7": finger, "joint8": finger}, root="gripper")
    pts = np.vstack([link_points(k, step) @ poses[k][:3, :3].T + poses[k][:3, 3] for k in HAND_LINKS])
    return pts - np.asarray(TCP_OFFSET, float)


HAND = Hand()


def box_distance(points, lo, hi) -> np.ndarray:
    """Distance of each point to an axis-aligned box (0 inside)."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    q = np.maximum(np.maximum(lo - points, points - hi), 0.0)
    return np.linalg.norm(q, axis=1)


# -- one arm's plan --------------------------------------------------------------------------------------------------

def tcp_jacobian(q, base) -> np.ndarray:
    """6x6 TCP Jacobian (linear; angular, world) by finite differences of ik.tcp_pose."""
    p0, r0 = ik.tcp_pose(q, base)
    jac, e = np.zeros((6, 6)), 1e-6
    for i in range(6):
        dq = np.array(q, float)
        dq[i] += e
        p, r = ik.tcp_pose(dq, base)
        w = (r - r0) @ r0.T / e
        jac[:3, i], jac[3:, i] = (p - p0) / e, 0.5 * np.array([w[2, 1] - w[1, 2], w[0, 2] - w[2, 0], w[1, 0] - w[0, 1]])
    return jac


def tcp_compliance(q, base) -> np.ndarray:
    """3x3 linear TCP compliance (m/N) of the joint PD springs at joints q: J K^-1 J^T [ARITH, springs only]."""
    j = tcp_jacobian(q, base)[:3]
    return j @ np.diag(1.0 / ARM_K) @ j.T


class Plan:
    """One arm's waypoints for yam_scripted, built step by step. Every method appends and returns self."""

    def __init__(self, arm: str, q_start=READY, base=None, where: dict | None = None):
        if arm not in ARMS:
            raise ValueError(f"arm must be one of {ARMS}")
        self.arm = arm
        self.base = np.asarray(station.BASES[arm] if base is None else base, float)
        self.q = np.array(q_start, float)
        self.pos, rot = ik.tcp_pose(self.q, self.base)
        self.grip = as_grip(rot)
        self.finger = FINGER_OPEN
        self.pre = FINGER_OPEN       # the pre-shape of the last grip, which a release opens back to
        self.engaged = 0.0           # how far the TCP went in along its approach in the last pick, place or press
        self.where = dict(where or {})
        self.held: str | None = None  # the part name of the last pick by name, until released
        self.waypoints: list[Waypoint] = []
        self.misses: list[tuple[str, float, float]] = []

    @property
    def duration(self) -> float:
        return sum(w.duration for w in self.waypoints)

    def approach(self, grip: Grip | None = None) -> np.ndarray:
        """World direction the fingers point."""
        return (self.grip if grip is None else grip).rot[:, 2]

    def _point(self, what) -> np.ndarray:
        return np.asarray(self.where[what] if isinstance(what, str) else what, float)

    def move(self, pos, grip: Grip | None = None, t: float | None = None, finger: float | None = None,
             label: str = "move", contact: bool = False) -> "Plan":
        """Go to `pos` (world TCP). With no `t`, at SPEED and TURN_SPEED. `contact` marks the stretch for
        SagCorrector (frozen there). IK is solved at the goal and a miss over IK_WARN is printed and kept."""
        grip = self.grip if grip is None else grip
        f = self.finger if finger is None else float(finger)
        goal = np.asarray(pos, float)
        if t is None:
            _, rot = ik.tcp_pose(self.q, self.base)
            turn = math.degrees(float(np.linalg.norm(ik._rot_error(grip.rot, rot))))
            t = max(MIN_MOVE_S, float(np.linalg.norm(goal - self.pos)) / SPEED, turn / TURN_SPEED)
        self.waypoints.append(Waypoint(float(t), tuple(float(v) for v in goal), f, label, grip))
        q, pe, re_ = ik.solve(goal, ik.matrix_to_quat_xyzw(grip.rot), self.q, base_pos=self.base, iters=200)
        self.q, self.pos, self.grip, self.finger = q, goal, grip, f
        mm, deg = pe * 1000, math.degrees(re_)
        if mm > IK_WARN[0] or deg > IK_WARN[1]:
            self.misses.append((label, mm, deg))
            print(f"[motionkit] {self.arm} '{label}': IK missed by {mm:.1f} mm, {deg:.1f} deg", flush=True)
        if contact:
            CONTACT[self.arm].add(label)
        return self

    def hold(self, s: float, label: str = "hold", contact: bool = False) -> "Plan":
        return self.move(self.pos, t=max(float(s), 1e-3), label=label, contact=contact)

    def wait_until(self, t: float, label: str = "wait") -> "Plan":
        """Hold until plan time `t` (no-op if already past it)."""
        if t > self.duration + 1e-9:
            self.hold(t - self.duration, label)
        return self

    def travel(self, z: float, to=None, label: str = "travel") -> "Plan":
        """Rise straight up to `z` over the table top, then (with `to`, an x, y) go there level: transits are level
        at a safe height, never diagonal past a part (LESSONS 21)."""
        self.move((self.pos[0], self.pos[1], TOP + z), label=f"{label}: up")
        if to is not None:
            self.move((to[0], to[1], TOP + z), label=f"{label}: across")
        return self

    def pick(self, what, h: float, width: float | None, grip: Grip | None = None, above: float = 0.10,
             lift: float = 0.08, force: float = 25.0, t: float | None = None, label: str = "pick") -> "Plan":
        """Over the part by `above` along the grip's axis, line up, go in to `h` over its reference point (a name in
        `where` or a point), close on `width` with `force` N a finger, lift `lift` straight up, settle the load."""
        grip = self.grip if grip is None else grip
        goal = self._point(what) + np.array([0.0, 0.0, h])
        over = goal - above * self.approach(grip)
        self.pre = preshape_for(width)
        self.move(over, grip=grip, finger=self.pre, t=t, label=f"{label}: over")
        self.hold(STEADY_S, f"{label}: line up")
        self.move(goal, t=max(1.0, above / 0.1), label=f"{label}: in", contact=True)
        self.move(goal, t=CLOSE_S, finger=finger_for(width, force), label=f"{label}: close", contact=True)
        if isinstance(what, str):
            GRASP[self.arm][f"{label}: close"] = what
            self.held = what
        self.hold(0.3, f"{label}: grip", contact=True)
        self.move(goal + np.array([0.0, 0.0, lift]), t=max(1.0, lift / 0.08), label=f"{label}: lift")
        self.engaged = 0.0
        return self.hold(LOAD_SETTLE_S, f"{label}: settle the load")

    def place(self, what, h: float, above: float = 0.05, t: float | None = None, label: str = "place") -> "Plan":
        """Carry to over the target by `above`, line up, go down to `h` over it, let it settle, open to the pick's
        pre-shape. Follow with release_clear()."""
        goal = self._point(what) + np.array([0.0, 0.0, h])
        over = goal - above * self.approach()
        self.move(over, t=t, label=f"{label}: over")
        self.hold(STEADY_S, f"{label}: line up")
        self.move(goal, t=max(0.8, above / 0.06), label=f"{label}: down", contact=True)
        self.hold(0.3, f"{label}: set", contact=True)
        self.move(goal, t=0.6, finger=self.pre, label=f"{label}: let go", contact=True)
        RELEASE[self.arm].add(f"{label}: let go")
        self.held, self.engaged = None, above
        return self

    def press(self, axis, depth: float | None = None, force: float | None = None, t: float = 1.0, hold: float = 0.5,
              label: str = "press") -> "Plan":
        """From contact (call it with the TCP on the surface), command `depth` past it along `axis`, or the depth
        that makes `force` N on the joint springs (the arm is compliant: a target offset is a force, LESSONS 27),
        hold, and come back."""
        a = np.asarray({"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1), "-x": (-1, 0, 0), "-y": (0, -1, 0),
                        "-z": (0, 0, -1)}.get(axis, axis) if isinstance(axis, str) else axis, float)
        a = a / np.linalg.norm(a)
        if depth is None:
            if force is None:
                raise ValueError("press needs depth or force")
            depth = float(force) * float(a @ tcp_compliance(self.q, self.base) @ a)
        start = self.pos.copy()
        self.move(start + depth * a, t=t, label=f"{label}: push", contact=True)
        self.hold(hold, f"{label}: hold", contact=True)
        self.move(start, t=max(0.5, t / 2), label=f"{label}: ease off", contact=True)
        self.engaged = depth
        return self

    def release_clear(self, gap: float = 0.035, t_open: float = 0.6, label: str = "release") -> "Plan":
        """Open to the pre-shape (if still closed) and back out along the grip's axis until the pad tips are `gap`
        past where they stood before going in."""
        if self.finger > self.pre + 1e-6:
            self.move(self.pos, t=t_open, finger=self.pre, label=f"{label}: open", contact=True)
            RELEASE[self.arm].add(f"{label}: open")
            self.held = None
        back = self.engaged + gap
        self.engaged = 0.0
        return self.move(self.pos - back * self.approach(), label=f"{label}: clear")

    def worst_miss(self) -> tuple[float, float]:
        return (max((m for _, m, _ in self.misses), default=0.0), max((d for _, _, d in self.misses), default=0.0))


def together(*plans: Plan, label: str = "wait for the other hand") -> float:
    """A barrier: each plan holds until all reach the same time. Returns that time."""
    t = max(p.duration for p in plans)
    for p in plans:
        p.wait_until(t, label)
    return t


def plans_out(*plans: Plan) -> dict:
    """{arm: waypoints} for yam_scripted. An arm with no waypoints is left out (yam_scripted holds it at READY)."""
    return {p.arm: list(p.waypoints) for p in plans if p.waypoints}


# -- proprioceptive droop correction ---------------------------------------------------------------------------------

class SagCorrector:
    """Integral joint trim from the OBSERVED joint angles (the policy's own observation), never from object poses.

    The arm holds each joint on a PD spring with gravity compensation for the arm alone, so a held load bends it off
    the plan. Each control step: q_cmd = q_plan + trim, and while the plan target has been still for `settle_s` and the
    stretch is not a contact one, trim += gain * (q_plan_prev - q_obs), clipped to +-max_rad per joint. Frozen
    (held) during contact stretches (Plan marks them; `freeze` adds labels) and while the target moves; zeroed when the
    plan opens the fingers (the load goes with it). Untested in PhysX as of the kit's first version: gain and
    settle_s come from the box tasks' pumpdemo (SAG_GAIN 0.25, read by the workflow study), not from our runs.
    """

    def __init__(self, gain: float = 0.25, max_rad: float = 0.30, settle_s: float = 0.25, still_mps: float = 0.02,
                 still_dps: float = 5.0, freeze=(), clear=()):
        self.gain, self.max_rad, self.settle_s = float(gain), float(max_rad), float(settle_s)
        self.still_mps, self.still_dps = float(still_mps), float(still_dps)
        self.freeze, self.clear = set(freeze), set(clear)
        self.reset()

    def reset(self) -> None:
        self.trim = {a: np.zeros(6) for a in ARMS}
        self.last = {a: None for a in ARMS}
        self.still = {a: 0.0 for a in ARMS}
        self.label = {a: None for a in ARMS}

    def step(self, arm: str, t: float, plan, q_plan, q_obs, dt: float) -> np.ndarray:
        """The joint command for plan time t. `plan` is the policy's ArmPlan; q_obs the observed joints now (the
        result of the command for t - dt)."""
        pos, rot, finger, label = plan.target(t)
        p0, r0, f0, _ = plan.target(max(t - dt, 0.0))
        speed = float(np.linalg.norm(np.asarray(pos) - np.asarray(p0))) / dt
        turn = math.degrees(float(np.linalg.norm(ik._rot_error(rot, r0)))) / dt
        self.still[arm] = self.still[arm] + dt if (speed < self.still_mps and turn < self.still_dps) else 0.0
        opening = finger < f0 - 1e-5   # WHY: finger joint runs 0 closed to -0.047 open
        if opening or (label in self.clear and label != self.label[arm]):
            self.trim[arm] = np.zeros(6)
        frozen = label in CONTACT.get(arm, ()) or label in self.freeze
        if self.last[arm] is not None and not frozen and not opening and self.still[arm] >= self.settle_s:
            err = self.last[arm] - np.asarray(q_obs, float)
            self.trim[arm] = np.clip(self.trim[arm] + self.gain * err, -self.max_rad, self.max_rad)
        self.label[arm] = label
        self.last[arm] = np.asarray(q_plan, float).copy()
        return np.clip(np.asarray(q_plan, float) + self.trim[arm], ik.LOWER, ik.UPPER)

    def tcp_shift_mm(self, arm: str, q_plan) -> float:
        """How far the trim moves the TCP at q_plan, mm (for traces)."""
        if self.last[arm] is None:
            return 0.0
        base = station.BASES[arm]
        a = ik.tcp_pose(np.asarray(q_plan, float), base)[0]
        b = ik.tcp_pose(np.clip(np.asarray(q_plan, float) + self.trim[arm], ik.LOWER, ik.UPPER), base)[0]
        return float(np.linalg.norm(b - a) * 1000)


_STATE = {"corrector": None}


def active_corrector():
    return _STATE["corrector"]


def install(corrector: SagCorrector | None = None) -> None:
    """Patch yam_arena's YamScriptedPolicy (the demo policy only, never the scene or the score):
    1. replan on the first action after each reset, because inspect-robots resets the policy before the embodiment,
       so demo() would otherwise see the previous trial's seed;
    2. with a corrector, add its joint trim to each arm's command, from the observation's joint angles.
    Safe to call again (the newest corrector wins)."""
    _STATE["corrector"] = corrector
    try:
        from yam_arena import scripted
    except ImportError:   # WHY: offline (no inspect_robots) there is no policy to patch
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_motionkit", False):
        return
    reset0, act0 = cls.reset, cls.act

    def reset(self, scene):
        self._mk_scene, self._mk_fresh = scene, True
        reset0(self, scene)
        if _STATE["corrector"] is not None:
            _STATE["corrector"].reset()

    def act(self, observation):
        if getattr(self, "_mk_fresh", False):
            self._mk_fresh = False
            reset0(self, self._mk_scene)
        chunk = act0(self, observation)
        corr = _STATE["corrector"]
        if corr is None:
            return chunk
        import dataclasses

        from inspect_robots_yam.packing import STATE_KEY, pack, split
        observed = split(np.asarray(observation.state[STATE_KEY], float))
        t = float(observation.state_time) + self._dt
        out = []
        for a in chunk.actions:
            per = [np.array(v, float) for v in split(np.asarray(a.data, float))]
            for i, arm in enumerate(ARMS):
                if arm in self._plans:
                    per[i][:6] = corr.step(arm, t, self._plans[arm], per[i][:6], observed[i][:6], self._dt)
            out.append(dataclasses.replace(a, data=pack(*per)))
        return dataclasses.replace(chunk, actions=out)

    cls.reset, cls.act, cls._motionkit = reset, act, True
