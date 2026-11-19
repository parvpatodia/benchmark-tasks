"""grippers.py [cli=N | corners | stems]: the two arms' clearance along the demo, from the YAM's own collision meshes.

Each arm follows its plan through the same IK the scripted demo uses (yam_arena.demo.ArmPlan); at each check the
convex collision meshes of the gripper, both finger tips, link5 and link4 (yam_arm.urdf, the URDF yam_arena loads) are
posed by forward kinematics and sampled on their faces every 1.5 mm. Checked every 0.1 s. Reported: the least distance between the two
arms, and the least distance from each arm to the trainer's fixed parts (palm, thumb, index finger;
not the ring finger, which the holding pads are meant to touch). No sim, no dynamics: the arms' lag on a descent (2 to
10 mm, README) is not in it.

    python3 tools/grippers.py            # --seed 0, 2, 4, the layout corners and the stems
"""
import functools
import math
import pathlib
import sys
import warnings
import xml.etree.ElementTree as ET
import zlib

warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np  # noqa: E402
from scipy.spatial import ConvexHull, cKDTree  # noqa: E402
import lancet_fingerstick as F  # noqa: E402
from yam_arena import YAM_URDF, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.urdf import forward_kinematics, rpy_matrix  # noqa: E402

DT_CHECK = 0.1                   # s between checks [CHOICE; the TCPs move under 10 mm in it, EST]
LINKS = ("gripper", "tip_left", "tip_right", "link5", "link4")
STEP = 0.0015
ARM_FLOOR = 0.010                # m: the least arm-to-arm distance this check passes [CHOICE: over the 2 to 10 mm lag]
HAND_FLOOR = 0.003               # m: the least distance from an arm to the trainer's fixed parts [CHOICE]
LOW = F.TOP + 0.25               # both TCPs under this: the arms are working near the hand


def _stl(path):
    b = path.read_bytes()
    if b[:5] == b"solid" and b"facet" in b[:300]:
        v = [list(map(float, ln.split()[1:4])) for ln in b.decode().splitlines() if ln.strip().startswith("vertex")]
        return np.array(v, float).reshape(-1, 3, 3)
    n = int.from_bytes(b[80:84], "little")
    a = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(float)


def _sample(tri):
    """Points on each triangle's face, about STEP apart."""
    out = [tri.reshape(-1, 3)]
    for t in tri:
        a, b, c = t
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
    """A link's collision meshes, sampled, in its own frame."""
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
            p = _sample(_stl(base / col.find("geometry/mesh").get("filename")))
            pts.append(p @ rpy_matrix(rpy).T + xyz)
    p = np.vstack(pts)
    # WHY: the meshes are dense; one point per 1.5 mm cell keeps the distances to within about 1 mm
    _, keep = np.unique(np.floor(p / STEP).astype(np.int64), axis=0, return_index=True)
    return p[np.sort(keep)]


def arm_points(side, q6, finger):
    """The arm's sampled collision points in the world for joints q6 and finger opening `finger` (m, <= 0)."""
    q = {n: float(v) for n, v in zip(ARM_JOINTS, q6)}
    q["joint7"] = q["joint8"] = float(finger)
    poses = forward_kinematics(YAM_URDF, q)
    base = np.asarray(station.BASES[side], float)
    return np.vstack([link_points(k) @ poses[k][:3, :3].T + poses[k][:3, 3] + base for k in LINKS])


def hand_hulls(seed, grip_index=False):
    """The trainer's fixed parts (world), each as a convex hull: every base shape of the hand mechanism but the plate.
    With grip_index, less the index finger and its rod into the palm. WHY: stem i00's left closes on the index on
    purpose (its pads 5 mm into it, the same squeeze as on the ring), so that contact is the stem, not a collision."""
    hx, hy, hz, yaw = F.layout(seed)["hand"]
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    out = []
    n = len(F._finger_capsules("index"))
    skip = set(range(n)) | {n + 1} if grip_index else set()   # hand_shapes' order: index, palm, index rod, thumb
    for i, shape in enumerate(F.HAND_SHAPES):
        if i in skip:
            continue
        pts = shape_points(shape)
        out.append(ConvexHull(pts @ R.T + np.array([hx, hy, hz])))
    return out


def shape_points(shape):
    """Points whose convex hull is the shape (mechanism frame): a box's corners, a capsule's or cylinder's end rings
    (with a capsule's caps), a hull's own points."""
    if shape.kind == "hull":
        return np.asarray(shape.points, float)
    lo, hi = (np.asarray(v, float) for v in shape.bounds())
    if shape.kind == "box":
        return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    r, length = shape.size
    ax = "XYZ".index(shape.axis)
    c = (lo + hi) / 2
    half = length / 2
    out = []
    for e in (-1, 1):
        for th in np.linspace(0, 2 * math.pi, 24, endpoint=False):
            for k, rr in ((0.0, r),) + (((0.7, r * 0.71), (1.0, 0.0)) if shape.kind == "capsule" else ()):
                p = np.array(c, float)
                p[ax] += e * (half + k * r * (shape.kind == "capsule"))
                o = [i for i in range(3) if i != ax]
                p[o[0]] += rr * math.cos(th)
                p[o[1]] += rr * math.sin(th)
                out.append(p)
    return np.array(out)


def _hull_gap(hull, pts, near=0.03):
    """The least signed distance from points to a convex hull (negative inside), from its face planes; points more
    than `near` outside the hull's box are skipped (they are farther than `near` from it)."""
    lo, hi = hull.min_bound - near, hull.max_bound + near
    pts = pts[np.all((pts >= lo) & (pts <= hi), axis=1)]
    if len(pts) == 0:
        return near
    d = pts @ hull.equations[:, :3].T + hull.equations[:, 3]
    return float(d.max(axis=1).min())


def _arm_gap(a, b, near=0.03):
    """The least distance between two point sets; WHY: only the points inside each other's box (plus `near`) can be
    under `near` apart, so the rest are skipped."""
    a = a[np.all((a >= b.min(0) - near) & (a <= b.max(0) + near), axis=1)]
    b = b[np.all((b >= a.min(0) - near) & (b <= a.max(0) + near), axis=1)] if len(a) else b[:0]
    if len(a) == 0 or len(b) == 0:
        return near
    return float(cKDTree(a).query(b, k=1)[0].min())


def check(seed, name, plans=None, grip_index=False):
    """(least arm-to-arm distance and when, least arm-to-trainer distance and which arm, when)."""
    R, L = F.plan(0.0, seed) if plans is None else plans
    pr, pl = ArmPlan(R, station.BASES["right"]), ArmPlan(L, station.BASES["left"])
    hulls = hand_hulls(seed, grip_index)
    dur = max(pr.duration, pl.duration)
    worst_arm, worst_hand, t = (9.0, None), (9.0, None, None), 0.0
    while t <= dur:
        qr, _ = pr.action7(t)
        ql, _ = pl.action7(t)
        tr, tl = pr.target(t), pl.target(t)
        a = arm_points("right", qr[:6], tr[2])
        b = arm_points("left", ql[:6], tl[2])
        if max(tr[0][2], tl[0][2]) < LOW:
            d = _arm_gap(a, b)
            if d < worst_arm[0]:
                worst_arm = (d, t)
        for side, p, tg in (("right", a, tr), ("left", b, tl)):
            if tg[0][2] < LOW:
                g = min(_hull_gap(h, p) for h in hulls)
                if g < worst_hand[0]:
                    worst_hand = (g, side, t)
        t += DT_CHECK
    ok = worst_arm[0] >= ARM_FLOOR and worst_hand[0] >= HAND_FLOOR
    print(f"{name}: arms {1000 * worst_arm[0]:.1f} mm apart (30.0 means 30 or more) at least (t {worst_arm[1]:.2f} s); arm to the trainer's "
          f"fixed parts {1000 * worst_hand[0]:.1f} mm ({worst_hand[1]}, t {worst_hand[2]:.2f} s) "
          + ("OK" if ok else "BAD"))
    return ok


def main():
    jobs = []
    args = sys.argv[1:] or ["cli=0", "cli=2", "cli=4", "corners", "stems"]
    for a in args:
        if a == "corners":
            jobs += [(c, f"corner {c[0] * 1000:+.0f}, {c[1] * 1000:+.0f} mm, {math.degrees(c[2]):+.0f} deg", None, False)
                     for c in F.corners()]
        elif a == "stems":
            sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
            import ikcheck
            seed = zlib.crc32(b"0:None:0")
            jobs += [(seed, f"--seed 0 stem {k}00", ikcheck.stem_plan(k, seed), k == "i") for k in ikcheck.STEMS]
        else:
            k, v = a.split("=")
            jobs.append((int(v) if k == "seed" else zlib.crc32(f"{v}:None:0".encode()), a, None, False))
    bad = sum(not check(*j) for j in jobs)
    print("GRIPPERS OK" if bad == 0 else f"GRIPPERS BAD {bad}")


if __name__ == "__main__":
    main()
