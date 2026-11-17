"""flame_axis.py [dt=0.25] [seed=..]: the flame-axis check of the real-trial protocol, offline along the scripted demo.
The axis runs from the nozzle's end along the flame for 300 mm. From the moment the torch lifts off its rest to the
end, it reports the nearest approach of that axis to every link of both arms (the left, torch-holding, gripper's
housing and fingers included), from the plan's IK and forward kinematics, with the torch rigid in the grip as planned.
The torch is placed by forward kinematics from the IK's joint solution at each step (the arm's own pose, as the
encoders would give it), rigid in the grip, and the checked segment is lengthened by the collars' 15 mm play at both
ends. Each reported distance is a lower bound, to within the 1 mm sampling, against each link's convex hull: points
every 1 mm along the segment against the hull's triangles (closest point on each triangle). No sim."""
import math, sys, pathlib, warnings
warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np
import solder_joint as F
from scipy.spatial import ConvexHull
from yam_arena import station, ik
from yam_arena.demo import ArmPlan
from yam_arena.constants import FINGER_OPEN, LINKS
from yam_arena.analysis import fk_link_poses_world, world_hulls

NOZZLE_Y = -0.140      # the nozzle's end on the torch's own y (its cylinder: centre -0.130, length 0.020)
REACH = 0.300          # the check covers the axis this far past the nozzle's end
DOWN, UP = 0.006, 0.010  # the handle can slide 6 mm down before it leaves the pads; the distance sensor trips past 10 mm up
BAND_STEP = 0.005      # band mode: parallel copies of the segment every <= 5 mm across that slide
PLAY = 0.015           # the collars' play each way along the handle: the segment is lengthened this much at both ends
STEP = 0.001          # the axis sampled every 1 mm
ARM_LINKS = [l for l in LINKS if l != "base"]
HAND = ("gripper", "tip_left", "tip_right")


def hulls_q(a, arm):
    qd = {f"joint{i + 1}": float(a[i]) for i in range(6)}
    qd["joint7"] = qd["joint8"] = float(a[6]) * FINGER_OPEN
    return [(n, h.vertices) for n, h in world_hulls(fk_link_poses_world(qd, station.BASES[arm]), ARM_LINKS)]


def axis_points(g, u):
    """Points every 1 mm along the checked segment in world coordinates: from PLAY before the nozzle's end to PLAY
    past REACH, along the flame direction u."""
    u = np.asarray(u, float) / np.linalg.norm(u)
    p0 = np.asarray(g, float) - NOZZLE_Y * u - PLAY * u
    return p0 + np.outer(np.arange(0.0, REACH + 2 * PLAY + 1e-9, STEP), u)


def band_points(g, u, up):
    """Band mode: copies of the segment shifted along ``up`` from -DOWN to +UP, at most BAND_STEP apart. Every point
    of the band lies within BAND_STEP / 2 of a copy, so (min distance over copies) - BAND_STEP / 2 bounds the band."""
    n = int(math.ceil((UP + DOWN) / BAND_STEP))
    offs = np.linspace(-DOWN, UP, n + 1)
    base = axis_points(g, u)
    return np.concatenate([base + o * np.asarray(up, float) for o in offs])


def held_torch(PL, grip_rot):
    """(grip point, flame direction) of the held torch from the left arm's solved joints through forward kinematics,
    the torch rigid in the grip: the TCP's pose, the grip point TCP_OVER_G + TORCH_SAG under it, and the flame
    direction carried by the wrist's rotation since the grasp (``grip_rot``: the TCP's rotation at the lift)."""
    pos, rot = ik.tcp_pose(PL.q, PL.base_pos)
    g = np.asarray(pos, float) - np.array([0.0, 0.0, F.TCP_OVER_G + F.TORCH_SAG])
    u0 = np.array([F.U[0], F.U[1], 0.0])
    return g, rot @ (grip_rot.T @ u0), -np.asarray(rot, float)[:, 2]


def tri_dist(P, A, B, C):
    """min distance from each point in P (n,3) to triangles (A,B,C) (m,3 each): Ericson's closest point on a triangle."""
    P = P[:, None, :]
    ab, ac, ap = B - A, C - A, P - A
    d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1)
    bp = P - B; d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1)
    cp = P - C; d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
    va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
    den = np.where(np.abs(va + vb + vc) < 1e-18, 1e-18, va + vb + vc)
    v, w = vb / den, vc / den
    q = A + v[..., None] * ab + w[..., None] * ac                                   # interior
    with np.errstate(divide="ignore", invalid="ignore"):
        cases = [
            ((d1 <= 0) & (d2 <= 0), A + 0 * q),
            ((d3 >= 0) & (d4 <= d3), B + 0 * q),
            ((d6 >= 0) & (d5 <= d6), C + 0 * q),
            ((vc <= 0) & (d1 >= 0) & (d3 <= 0), A + (d1 / (d1 - d3))[..., None] * ab),
            ((vb <= 0) & (d2 >= 0) & (d6 <= 0), A + (d2 / (d2 - d6))[..., None] * ac),
            ((va <= 0) & (d4 - d3 >= 0) & (d5 - d6 >= 0),
             B + ((d4 - d3) / ((d4 - d3) + (d5 - d6)))[..., None] * (C - B)),
        ]
    done = np.zeros(d1.shape, bool)
    out = q.copy()
    for m, val in cases:
        m = m & ~done
        out[m] = val[m]
        done |= m
    return float(np.sqrt(((P - out) ** 2).sum(-1)).min())


def hull_dist(Pts, V):
    """Exact distance from the sampled axis to the convex hull of V (0 if a sample lies inside)."""
    h = ConvexHull(V)
    if np.any(np.all(Pts @ h.equations[:, :3].T + h.equations[:, 3] <= 1e-9, axis=1)):
        return 0.0
    T = V[h.simplices]
    return tri_dist(Pts, T[:, 0], T[:, 1], T[:, 2])


def box_lower(Pts, V):
    """A lower bound on the axis-to-hull distance: the axis points' distance to the hull's bounding box."""
    lo, hi = V.min(0), V.max(0)
    d = np.maximum(np.maximum(lo - Pts, Pts - hi), 0.0)
    return float(np.sqrt((d * d).sum(1)).min())


def sweep(dt=0.05, seed=None, part=0, parts=1, band=0):
    """{(arm, group): (min gap m, t, link)} from the torch's lift to the end, groups: hand (gripper housing and both
    fingers) and other (link1..link5). ``part``/``parts`` split the measured time range for parallel runs; every run
    still steps the IK from t = 0. A link is skipped when its bounding box is already farther than the group's best."""
    L, R, info = F.plan(0.0, seed)
    PL, PR = ArmPlan(L, station.BASES["left"]), ArmPlan(R, station.BASES["right"])
    best, k, every = {}, 0, max(1, round(dt / 0.05))
    end = min(PL.duration, PR.duration)
    t0 = info["held_from"]
    grip_rot = None
    lo_t = t0 + (end - t0) * part / parts
    hi_t = t0 + (end - t0) * (part + 1) / parts
    while k * 0.05 <= min(end, hi_t) + 1e-9:
        # WHY: step the IK from t = 0 at the control rate, as the scripted policy warm-starts it, so the arms take the
        # run's postures; measure only from the lift (the rest's switch reads the torch off its rest)
        t = k * 0.05; k += 1
        acts = {"left": PL.action7(t)[0], "right": PR.action7(t)[0]}
        if t >= t0 - 1e-9 and grip_rot is None:
            grip_rot = np.asarray(ik.tcp_pose(PL.q, PL.base_pos)[1], float)
        if t < max(t0, lo_t) - 1e-9 or (k - 1) % every:
            continue
        g, u, up = held_torch(PL, grip_rot)
        A = band_points(g, u, up) if band else axis_points(g, u)
        for arm in ("left", "right"):
            for name, V in hulls_q(acts[arm], arm):
                key = (arm, "hand" if name in HAND else "other")
                cur = best.get(key, (math.inf,))[0]
                if box_lower(A, V) >= cur:
                    continue
                g = hull_dist(A, V)
                if g < cur:
                    best[key] = (g, t, name)
    return best


if __name__ == "__main__":
    kw = {}
    for a in sys.argv[1:]:
        k, v = a.split("=")
        kw[k] = int(v) if k in ("seed", "part", "parts", "band") else float(v)
    less = BAND_STEP / 2 if kw.get("band") else 0.0
    for (arm, grp), (g, t, name) in sorted(sweep(**kw).items()):
        print(f"{arm:5s} {grp:5s} {1000 * max(0.0, g - less):7.1f} mm at t {t:6.2f} s ({name})")
