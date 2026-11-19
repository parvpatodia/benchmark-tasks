"""armgap.py: the two arms' clearance along the demo and the stems, from the YAM's own collision meshes.

Each arm follows its plan through the same IK the scripted demo uses (yam_arena.demo.ArmPlan); the convex
collision meshes of every moving link (link1 to link5, the gripper, both finger tips; yam_arm.urdf, the URDF yam_arena
loads) are posed by forward kinematics and sampled on their faces about every 6 mm, every 0.25 s. Reported: the least distance
between the two arms, and from link1 to link5 of each arm to the fixed parts (deck and tote; the gripper and tips are
swept against them, the cover and the wrench by tools/clearance.py). BAD when the arms come within 20 mm of each other
or a link within 5 mm of a fixed part. Planned poses, no sim: the arm's lag and sag are not in it. Ends with ARMGAP BAD n.
"""
import functools
import pathlib
import sys
import warnings
import xml.etree.ElementTree as ET

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import numpy as np  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402
from yam_arena import YAM_URDF, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import load  # noqa: E402
from yam_arena.urdf import forward_kinematics, rpy_matrix  # noqa: E402

M = load(HERE / "floor_hole_cover.py").module
LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
UPPER = LINKS[:5]
STEP = 0.006
ARM_FLOOR, FIXED_FLOOR = 0.020, 0.005   # [CHOICE: over the arm's lag and sag]
DT = 0.25                        # s between checks [CHOICE: the TCPs move under 50 mm in it]
FIXED = [(np.asarray(lo, float), np.asarray(hi, float)) for _, lo, hi, _ in M.fixed_boxes()]


def _stl(path):
    b = path.read_bytes()
    if b[:5] == b"solid" and b"facet" in b[:300]:
        v = [list(map(float, ln.split()[1:4])) for ln in b.decode().splitlines() if ln.strip().startswith("vertex")]
        return np.array(v, float).reshape(-1, 3, 3)
    n = int.from_bytes(b[80:84], "little")
    a = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(float)


def _sample(tri):
    out = [tri.reshape(-1, 3)]
    for a, b, c in tri:
        n = int(max(np.linalg.norm(b - a), np.linalg.norm(c - a)) / STEP) + 1
        if n < 2:
            continue
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
        m = (i + j) <= n
        u, v = i[m] / n, j[m] / n
        out.append(a + u[:, None] * (b - a) + v[:, None] * (c - a))
    return np.vstack(out)


@functools.lru_cache(maxsize=None)
def link_points(link):
    root = ET.parse(YAM_URDF).getroot()
    base = pathlib.Path(YAM_URDF).parent
    pts = []
    for ln in root.iter("link"):
        if ln.get("name") != link:
            continue
        for col in ln.iter("collision"):
            o = col.find("origin")
            xyz = np.array([float(v) for v in o.get("xyz").split()])
            rpy = np.array([float(v) for v in o.get("rpy").split()])
            pts.append(_sample(_stl(base / col.find("geometry/mesh").get("filename"))) @ rpy_matrix(rpy).T + xyz)
    p = np.vstack(pts)
    _, keep = np.unique(np.floor(p / STEP).astype(np.int64), axis=0, return_index=True)
    return p[np.sort(keep)]


def arm_points(side, q6, finger, links=LINKS):
    q = {n: float(v) for n, v in zip(ARM_JOINTS, q6)}
    q["joint7"] = q["joint8"] = float(finger)
    poses = forward_kinematics(YAM_URDF, q)
    base = np.asarray(station.BASES[side], float)
    return np.vstack([link_points(k) @ poses[k][:3, :3].T + poses[k][:3, 3] + base for k in links])


def box_gap(pts):
    best = np.inf
    for lo, hi in FIXED:
        q = np.maximum(np.maximum(lo - pts, pts - hi), 0.0)
        best = min(best, float(np.linalg.norm(q, axis=1).min()))
    return best


def check(name, plans):
    A = {arm: ArmPlan(W, station.BASES[arm]) for arm, W in plans.items()}
    T = max((a.duration for a in A.values()), default=0.0)
    arm_low, fix_low, t, bad = (9.0, 0.0), {arm: (9.0, 0.0) for arm in A}, 0.0, 0
    while t <= T:
        pts = {}
        for arm, a in A.items():
            q, _ = a.action7(min(t, a.duration))
            f = a.target(min(t, a.duration))[2]
            pts[arm] = arm_points(arm, q[:6], f)
            g = box_gap(arm_points(arm, q[:6], f, UPPER))
            if g < fix_low[arm][0]:
                fix_low[arm] = (g, round(t, 1))
        if len(pts) == 2:
            d = float(cKDTree(pts["left"]).query(pts["right"])[0].min())
            if d < arm_low[0]:
                arm_low = (d, round(t, 1))
        t += DT
    if len(A) == 2:
        flag = arm_low[0] < ARM_FLOOR
        bad += flag
        print(f"{name:12s} arms apart  {1000 * arm_low[0]:6.1f} mm at t {arm_low[1]:5.1f}{'  BAD' if flag else ''}", flush=True)
    for arm, (g, tt) in fix_low.items():
        flag = g < FIXED_FLOOR
        bad += flag
        print(f"{name:12s} {arm:5s} link1-5 to deck and tote {1000 * g:6.1f} mm at t {tt:5.1f}{'  BAD' if flag else ''}", flush=True)
    return bad


if __name__ == "__main__":
    total = 0
    for s in (0, 1, 2, 3):
        total += check(f"demo {s}", M.plan(s, "demo"))
    for v in M.VARIANTS[2:]:
        total += check(f"s_{v}", M.plan(0, v))
    print("ARMGAP BAD", total)
