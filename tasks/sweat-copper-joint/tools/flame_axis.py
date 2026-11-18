"""flame_axis.py [dt=0.05] [cli=0|2|4 or seed=..] [band=1] [part=i parts=n]: the real trial's flame-axis check,
offline along the scripted demo. No sim.

The live check places the flame from what it measures (``live_axes``): the grip point from the left arm's forward
kinematics, moved along the gripper's approach axis by the distance sensor's slide reading, and two flame directions
from it, the orientation sensor's (the torch's measured turn) and the arm's (the grip measured at the lift, carried
rigid by the wrist). The checked segment runs from the nozzle's end for 300 mm, lengthened by the 15 mm the handle
can sit either way along the grip section. From the lift to the end of the demo it reports the nearest approach of
that segment to every link of both arms, the bases and the left (torch) gripper's housing and fingers included. Here
the torch is rigid, so both directions agree; ``band=1`` adds copies of the segment for every slide the distance
check allows (10 mm either way).

Each figure printed is a lower bound against the link's convex hull (which holds the link): the segment is sampled
every 1 mm (every point of it is within 0.5 mm of a sample) and band copies are at most 5 mm apart (within 2.5 mm of
a copy), so the printed value is the sampled minimum less 0.5 mm, or less 3.0 mm in band mode. Times are sampled at
the 20 Hz control rate, as the live check runs. ``cli=N`` is inspect-robots' ``--seed N`` (the README's seeds; the
trial seed is zlib.crc32(f"{N}:None:0")); ``seed=`` takes a trial seed directly."""
import math, sys, pathlib, warnings, zlib
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
DOWN = UP = 0.010      # the distance check bounds the handle's slide between the pads to 10 mm either way
BAND_STEP = 0.005      # band mode: parallel copies of the segment every <= 5 mm across that slide
PLAY = 0.015           # the grip section's play each way along the handle: the segment is lengthened this much at both ends
STEP = 0.001          # the axis sampled every 1 mm
ARM_LINKS = list(LINKS)  # every link, the base included
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


def band_offsets():
    """Band mode: slides from -DOWN to +UP, at most BAND_STEP apart."""
    n = int(math.ceil((UP + DOWN) / BAND_STEP))
    return list(np.linspace(-DOWN, UP, n + 1))


U_TORCH = np.array([0.0, -1.0, 0.0])   # the flame's direction in the torch's own frame (its -y)


def rz(a):
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])


def live_axes(p_tcp, r_tcp, r_imu, grip0, dd):
    """The live check's grip point and two flame directions.
    p_tcp, r_tcp: the left TCP's position and rotation from the encoders (forward kinematics); its z points from the
    TCP to the fingertips. r_imu: the torch's orientation in the rig frame from the orientation sensor (heading zeroed
    with the torch jigged on its rest). grip0 = (o, r): the grip measured when the rest's switch opens: o the grip
    point in the TCP's frame (centred between the fingers, at the depth the distance sensor reads against its
    rod-set zero) and r = r_tcp_lift.T @ r_imu_lift the torch's rotation in the TCP's frame. dd: the slide
    toward the gripper's housing since the lift (m), from the distance sensor.
    Returns (g, u_imu, u_arm): the grip point moved by the slide, and the flame's direction turned by the measured
    orientation and by the wrist with the grip held rigid. The nozzle's end is g + 0.140 m along either direction."""
    o, r_grip = grip0
    g = np.asarray(p_tcp, float) + np.asarray(r_tcp, float) @ (np.asarray(o, float) - dd * np.array([0.0, 0.0, 1.0]))
    return g, np.asarray(r_imu, float) @ U_TORCH, np.asarray(r_tcp, float) @ r_grip @ U_TORCH


def lift_grip(p_tcp, r_tcp):
    """The demo's grip at the lift: the grip point TCP_OVER_G + TORCH_SAG under the TCP, the torch as it lies on its
    rest (yaw TORCH_YAW). Offline this stands in for what the sensors read when the rest's switch opens."""
    r_tcp = np.asarray(r_tcp, float)
    o = r_tcp.T @ np.array([0.0, 0.0, -(F.TCP_OVER_G + F.TORCH_SAG)])
    return o, r_tcp.T @ rz(F.TORCH_YAW)


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
    fingers) and other (base, link1..link5). ``part``/``parts`` split the measured time range for parallel runs; every run
    still steps the IK from t = 0. A link is skipped when its bounding box is already farther than the group's best."""
    L, R, info = F.plan(0.0, seed)
    PL, PR = ArmPlan(L, station.BASES["left"]), ArmPlan(R, station.BASES["right"])
    best, k, every = {}, 0, max(1, round(dt / 0.05))
    end = min(PL.duration, PR.duration)
    t0 = info["held_from"]
    grip0 = None
    lo_t = t0 + (end - t0) * part / parts
    hi_t = t0 + (end - t0) * (part + 1) / parts
    while k * 0.05 <= min(end, hi_t) + 1e-9:
        # WHY: step the IK from t = 0 at the control rate, as the scripted policy warm-starts it, so the arms take the
        # run's postures; measure only from the lift (the rest's switch reads the torch off its rest)
        t = k * 0.05; k += 1
        acts = {"left": PL.action7(t)[0], "right": PR.action7(t)[0]}
        if t >= t0 - 1e-9 and grip0 is None:
            grip0 = lift_grip(*ik.tcp_pose(PL.q, PL.base_pos))
        if t < max(t0, lo_t) - 1e-9 or (k - 1) % every:
            continue
        p, r = ik.tcp_pose(PL.q, PL.base_pos)
        r_imu = np.asarray(r, float) @ grip0[1]          # the torch rigid in the grip: the sensor turns with the wrist
        segs = []
        for dd in (band_offsets() if band else [0.0]):
            g, u_imu, u_arm = live_axes(p, r, r_imu, grip0, dd)
            segs.append(axis_points(g, u_imu))
            if not np.allclose(u_imu, u_arm, atol=1e-9):    # rigid here, so the two agree; kept for a turned torch
                segs.append(axis_points(g, u_arm))
        A = np.concatenate(segs)
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
        kw[k] = int(v) if k in ("seed", "cli", "part", "parts", "band") else float(v)
    if "cli" in kw:
        kw["seed"] = zlib.crc32(f"{kw.pop('cli')}:None:0".encode())
    less = STEP / 2 + (BAND_STEP / 2 if kw.get("band") else 0.0)
    for (arm, grp), (g, t, name) in sorted(sweep(**kw).items()):
        print(f"{arm:5s} {grp:5s} {1000 * max(0.0, g - less):7.1f} mm at t {t:6.2f} s ({name})")
