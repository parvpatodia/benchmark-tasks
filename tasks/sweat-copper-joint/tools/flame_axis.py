"""flame_axis.py [cli=0|2|4 or seed=..] [band=1 | rest=1 | cradle=1 | box=1 | turn=1 | grip=1 | hose=1]: the real
trial's flame-axis check, offline along the scripted demo. No sim.

Default (off rest): the live check's placement (``live_axes``): the grip point from the left arm's forward kinematics,
moved along the gripper's approach axis to the depth the distance sensor reads, and two flame directions, the
orientation sensor's and the arm's (the lift's grip carried rigid by the wrist). The checked segment runs from
PLAY_NEAR behind the nozzle's end (the handle can sit that much further forward in the grip) to REACH + PLAY_FAR past
it. Its near zone is the part up to R_NEAR + PLAY_FAR past the nozzle's end, the far zone the rest. From the lift to
the end it reports, per arm and group (hand: gripper housing and both fingers; camera: the wrist camera's case box;
other: base and link1..link5), the nearest approach over the whole segment and over the near zone, and the
speed-and-separation margin of the near zone (``margin``). Here the torch is rigid, so both directions agree;
``band=1`` adds copies of the segment for every slide the distance check allows (10 mm either way).

``rest=1``: the on-rest check (``rest_axis``), from t = 0 to the end of the lift: the segment from the torch's rest
pose along the orientation sensor's direction, its near end REST_SLIDE behind the nozzle (the rails fix the torch);
near-zone gaps and margin, far-zone gaps, the far zone's dwell (time any link of a group is within TRIP of it) and
where the rest's line passes the coupling.

``cradle=1``: the real rest's side rails (not in the sim) against the carried torch and every link of both arms; the
sim handle's overlap with the left arm's base; the real torch's rear (handle end, sleeve, sensor case and hose
fitting) against every link of the left arm, its base included, on the rest and carried. ``box=1``: the extents
of the nozzle's end and of every checked point. ``turn=1``: joint6 and joint5 jogged 10 degrees at the lift's pose.
``grip=1``: the pads' width along the handle and the play it leaves. ``hose=1``: the taut hose (the line from the
fitting to the hole in the rest it runs down through, grown by its radius and bow) against every link of both arms
and the segments.

Speeds are the fastest move of any hull vertex or of the segment's ends over one control step. The margin is
gap - c (T_C + T_S) - 2 V_CAP T_C, c the gap's closing speed over the last step; the near zone trips under TRIP.

Each gap printed is a lower bound against the link's convex hull: segment samples every 1 mm (every point within
0.5 mm of one) and band copies at most 5 mm apart (within 2.5 mm), so printed values are sampled minima less 0.5 mm,
or 3.0 mm in band mode. Times are the 20 Hz control steps. ``cli=N`` is inspect-robots' ``--seed N`` (trial seed
zlib.crc32(f"{N}:None:0")); ``seed=`` takes a trial seed directly."""
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
from yam_arena.analysis import fk_link_poses_world, link_hulls
from yam_arena.urdf import quat_xyzw_to_matrix

DT = 0.05              # the control period
NOZZLE_Y = -0.140      # the nozzle's end on the torch's own y (its cylinder: centre -0.130, length 0.020)
REACH = 0.300          # the segment runs this far past the nozzle's end (plus PLAY_FAR)
R_NEAR = 0.150         # near zone: to this far past the nozzle's end (plus PLAY_FAR) [CHOICE, set by calibration]
TRIP = 0.030           # the static trip distance
PLAY_NEAR = 0.034      # the grip can sit this much nearer the nozzle: the pads run off the handle's front (grip=1)
PLAY_FAR = 0.015       # and this much farther from it: a pad meets the rear sleeve (grip=1)
REST_SLIDE = 0.012     # on the rest the rails fix the torch, which can slide this far either way before a switch opens
DOWN = UP = 0.010      # the distance check bounds the handle's slide between the pads to 10 mm either way
BAND_STEP = 0.005
STEP = 0.001
T_C = 0.01             # the live check's period (100 Hz) [CHOICE]
T_S = 0.20             # stop time gate: trip to standstill, measured each session [CHOICE]
V_CAP = 0.25           # speed trip: any checked point faster than this (m/s) [CHOICE]
CAM_CASE = (0.042, 0.042, 0.023)   # D405 case W x H x D (data sheet 337029, Table 3-54)
CAM_GROW = 0.015       # the case grown this much on every side for its mount and the lens's place on it [EST]
ARM_LINKS = list(LINKS)
HAND = ("gripper", "tip_left", "tip_right")
U_TORCH = np.array([0.0, -1.0, 0.0])   # the flame's direction in the torch's own frame (its -y)


def rz(a):
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])


# -- hulls: every link piece and a box for each wrist camera, cached in their own frames -------------------------------
def _cam_box(side):
    """The wrist camera's case box (8 corners) in the gripper frame: the case centred on the optical axis, its front
    face CAM_GROW in front of the lens, grown CAM_GROW on every side (the lens's place on the face is not known)."""
    cam = station.wrist_camera(side)
    r = quat_xyzw_to_matrix(np.array(cam.rot_xyzw, float))
    hw, hh, d = CAM_CASE[0] / 2 + CAM_GROW, CAM_CASE[1] / 2 + CAM_GROW, CAM_CASE[2]
    # opengl: the camera looks along its -z, so the case lies behind the lens at +z
    c = np.array([[x, y, z] for x in (-hw, hw) for y in (-hh, hh) for z in (-CAM_GROW, d + CAM_GROW)])
    return np.asarray(cam.pos, float) + c @ r.T


_PIECES = {}


def _pieces(arm):
    """[(link, frame link, local vertices, ConvexHull)] for one arm; the camera rides the gripper's frame."""
    if arm not in _PIECES:
        out = []
        for name in ARM_LINKS:
            for h in link_hulls().get(name, []):
                out.append((name, name, h.vertices, ConvexHull(h.vertices)))
        V = _cam_box(arm)
        out.append(("camera", "gripper", V, ConvexHull(V)))
        _PIECES[arm] = out
    return _PIECES[arm]


def poses_q(a, arm):
    qd = {f"joint{i + 1}": float(a[i]) for i in range(6)}
    qd["joint7"] = qd["joint8"] = float(a[6]) * FINGER_OPEN
    return fk_link_poses_world(qd, station.BASES[arm])


def hulls_q(a, arm):
    """[(link, world vertices)] for every piece of one arm at action a (the camera box included)."""
    P = poses_q(a, arm)
    return [(n, V @ P[f][:3, :3].T + P[f][:3, 3]) for n, f, V, _ in _pieces(arm)]


def group(name):
    return "hand" if name in HAND else ("camera" if name == "camera" else "other")


# -- distances ---------------------------------------------------------------------------------------------------------
def tri_dist(P, A, B, C):
    """(min distance, index into P) from points P (n,3) to triangles (A,B,C): Ericson's closest point on a triangle."""
    P = P[:, None, :]
    ab, ac, ap = B - A, C - A, P - A
    d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1)
    bp = P - B; d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1)
    cp = P - C; d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
    va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
    den = np.where(np.abs(va + vb + vc) < 1e-18, 1e-18, va + vb + vc)
    v, w = vb / den, vc / den
    q = A + v[..., None] * ab + w[..., None] * ac
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
    d = np.sqrt(((P - out) ** 2).sum(-1)).min(1)
    i = int(d.argmin())
    return float(d[i]), i


def _hull_dist(Pl, V, h, chunk=256):
    """(distance, index) from local points Pl to a piece's hull (0 if a point lies inside). Exact: a point's largest
    signed distance to the hull's face planes is a lower bound on its distance, so points are taken in order of that
    bound, in chunks, and the search stops once the next bound is no nearer than the best exact distance."""
    s = (Pl @ h.equations[:, :3].T + h.equations[:, 3]).max(1)
    if (s <= 1e-9).any():
        return 0.0, int(np.argmax(s <= 1e-9))
    T = V[h.simplices]
    order = np.argsort(s, kind="stable")
    best = (math.inf, -1)
    for k in range(0, len(order), chunk):
        if s[order[k]] >= best[0]:
            break
        idx = np.sort(order[k:k + chunk])
        d, i = tri_dist(Pl[idx], T[:, 0], T[:, 1], T[:, 2])
        if (d, int(idx[i])) < best:
            best = (d, int(idx[i]))
    return best


def hull_dist(Pts, V):
    """Exact distance from points to the convex hull of world vertices V (0 if a point lies inside)."""
    return _hull_dist(Pts, V, ConvexHull(V))[0]


def box_lower(Pts, V):
    lo, hi = V.min(0), V.max(0)
    d = np.maximum(np.maximum(lo - Pts, Pts - hi), 0.0)
    return float(np.sqrt((d * d).sum(1)).min())


def link_gaps(P, a, arm, far=0.15):
    """{link: (gap, index into P)} for every link (and the camera) of one arm at action a. A link whose pieces' boxes
    are all farther than ``far`` gets that box bound instead (a lower bound, flagged by index -1)."""
    poses = poses_q(a, arm)
    out = {}
    for name, frame, V, h in _pieces(arm):
        r, p = poses[frame][:3, :3], poses[frame][:3, 3]
        Pl = (P - p) @ r
        cur = out.get(name, (math.inf, -1))[0]
        lb = box_lower(Pl, V)
        if lb >= cur:
            continue
        if lb >= far:
            out[name] = (lb, -1)
            continue
        d = _hull_dist(Pl, V, h)
        if d[0] < cur:
            out[name] = d
    return out


# -- the live check's placement ----------------------------------------------------------------------------------------
def axis_points(g, u, reach=REACH, near=PLAY_NEAR):
    """(points every 1 mm along the checked segment, their distance past the nominal nozzle end): from ``near`` before
    the nozzle's end to PLAY_FAR past ``reach``, along the flame direction u. On the rest ``near`` is REST_SLIDE: the
    rails, not the grip, fix where the torch lies, and the far end's PLAY_FAR covers the slide that way too."""
    u = np.asarray(u, float) / np.linalg.norm(u)
    a = np.arange(-near, reach + PLAY_FAR + 1e-9, STEP)
    return np.asarray(g, float) - NOZZLE_Y * u + np.outer(a, u), a


def band_offsets():
    n = int(math.ceil((UP + DOWN) / BAND_STEP))
    return list(np.linspace(-DOWN, UP, n + 1))


def live_axes(p_tcp, r_tcp, r_imu, grip0, dd):
    """The live check's grip point and two flame directions.
    p_tcp, r_tcp: the left TCP's position and rotation from the encoders (forward kinematics); its z points from the
    TCP to the fingertips. r_imu: the torch's orientation in the rig frame from the orientation sensor (heading zeroed
    with the torch jigged on its rest). grip0 = (o, r): the grip measured when the rest's switches open: o the grip
    point in the TCP's frame (centred between the fingers, at the depth the distance sensor reads against its
    rod-set zero) and r = r_tcp_lift.T @ r_imu_lift the torch's rotation in the TCP's frame. dd: the slide toward the
    gripper's housing since the lift (m), from the distance sensor.
    Returns (g, u_imu, u_arm): the grip point moved by the slide, and the flame's direction turned by the measured
    orientation and by the wrist with the grip held rigid. The nozzle's end is g + 0.140 m along either direction."""
    o, r_grip = grip0
    g = np.asarray(p_tcp, float) + np.asarray(r_tcp, float) @ (np.asarray(o, float) - dd * np.array([0.0, 0.0, 1.0]))
    return g, np.asarray(r_imu, float) @ U_TORCH, np.asarray(r_tcp, float) @ r_grip @ U_TORCH


def rest_axis(r_imu):
    """The on-rest check's grip point and flame direction: the torch's rest pose (G_REST; the rails hold it there),
    turned by the orientation sensor's reading r_imu (rig frame, zeroed on the jig on the rest)."""
    return np.asarray(F.G_REST, float), np.asarray(r_imu, float) @ U_TORCH


def lift_grip(p_tcp, r_tcp):
    """The demo's grip at the lift: the grip point TCP_OVER_G + TORCH_SAG under the TCP, the torch as it lies on its
    rest (yaw TORCH_YAW). Offline this stands in for what the sensors read when the rest's switches open."""
    r_tcp = np.asarray(r_tcp, float)
    o = r_tcp.T @ np.array([0.0, 0.0, -(F.TCP_OVER_G + F.TORCH_SAG)])
    return o, r_tcp.T @ rz(F.TORCH_YAW)


def lift_end(L):
    t = 0.0
    for w in L:
        t += w.duration
        if w.label == "lift the torch":
            return t
    raise ValueError("no lift waypoint")


def _plans(seed):
    L, R, info = F.plan(0.0, seed)
    return L, ArmPlan(L, station.BASES["left"]), ArmPlan(R, station.BASES["right"]), info


def _walk(seed, rest=False):
    """Yield (t, acts, p_tcp, r_tcp, grip0) at every control step from the start of the measured span to its end; the
    IK is stepped from t = 0 at the control rate, as the scripted policy warm-starts it."""
    L, PL, PR, info = _plans(seed)
    t0, end = info["held_from"], min(PL.duration, PR.duration)
    if rest:
        t0, end = 0.0, lift_end(L)
    grip0, k = None, 0
    while k * DT <= end + 1e-9:
        t = k * DT; k += 1
        acts = {"left": PL.action7(t)[0], "right": PR.action7(t)[0]}
        p, r = ik.tcp_pose(PL.q, PL.base_pos)
        if t >= info["held_from"] - 1e-9 and grip0 is None:
            grip0 = lift_grip(p, r)
        if t < t0 - 1e-9:
            continue
        yield t, acts, p, r, grip0


def _segments(p, r, grip0, band):
    """(points, a) of every checked copy of the off-rest segment at one step."""
    r_imu = np.asarray(r, float) @ grip0[1]          # the torch rigid in the grip: the sensor turns with the wrist
    P, A = [], []
    for dd in (band_offsets() if band else [0.0]):
        g, u_imu, u_arm = live_axes(p, r, r_imu, grip0, dd)
        for u in ([u_imu] if np.allclose(u_imu, u_arm, atol=1e-9) else [u_imu, u_arm]):
            q, a = axis_points(g, u)
            P.append(q); A.append(a)
    return np.concatenate(P), np.concatenate(A)


def _vertices(acts):
    return {arm: np.concatenate([V for _, V in hulls_q(acts[arm], arm)]) for arm in ("left", "right")}


def sweep(seed=None, band=0, rest=0):
    """Series of per-link gaps along the demo. Returns (series, speeds): series[(arm, link, zone)] = [(t, gap, a)],
    zone "all", "near" or "far"; speeds = {"left"/"right"/"segment": (max m/s, t)} over every hull vertex and the
    segment's ends between control steps."""
    series, speeds, prev = {}, {}, None
    for t, acts, p, r, grip0 in _walk(seed, rest):
        if rest:
            P, A = axis_points(*rest_axis(rz(F.TORCH_YAW)), near=REST_SLIDE)
        else:
            P, A = _segments(p, r, grip0, band)
        near = A <= R_NEAR + PLAY_FAR + 1e-9
        for arm in ("left", "right"):
            for zone, m in (("near", near), ("far", ~near)):
                for name, (g, i) in link_gaps(P[m], acts[arm], arm).items():
                    series.setdefault((arm, name, zone), []).append((t, g, float(A[m][i]) if i >= 0 else math.nan))
        cur = dict(_vertices(acts), segment=P[[0, len(A) - 1]] if not band else P)
        if prev is not None and not rest:
            for k_, V in cur.items():
                if prev[k_].shape == V.shape:
                    v = float(np.linalg.norm(V - prev[k_], axis=1).max() / DT)
                    if v > speeds.get(k_, (0.0,))[0]:
                        speeds[k_] = (v, t)
        prev = cur
    for (arm, name, zone), s in list(series.items()):
        if zone == "near":
            far = dict((tt, (g, a)) for tt, g, a in series[(arm, name, "far")])
            series[(arm, name, "all")] = [(tt, min(g, far[tt][0]), a if g <= far[tt][0] else far[tt][1])
                                          for tt, g, a in s]
    return series, speeds


def margin(s, t_c=T_C, t_s=T_S, v=V_CAP):
    """Speed-and-separation margin of one gap series: min over t of gap - c (T_C + T_S) - 2 V T_C, c the gap's closing
    speed over the last control step (0 if opening). The live rule trips when this falls under TRIP."""
    best = (math.inf, None)
    for (t0, g0, _), (t1, g1, _) in zip(s, s[1:]):
        c = max(0.0, (g0 - g1) / (t1 - t0))
        m_ = g1 - c * (t_c + t_s) - 2 * v * t_c
        if m_ < best[0]:
            best = (m_, t1)
    return best


def dwell(s, d=TRIP):
    """Total time (s) a gap series is under d, and the first and last such step."""
    ts = [t for t, g, _ in s if g < d]
    return len(ts) * DT, (ts[0], ts[-1]) if ts else None


# -- the coupling on the rest's line -----------------------------------------------------------------------------------
def rest_line_coupling():
    """Where the resting torch's line (from G_REST along the flame, at its height) passes within the coupling's radius
    of the joint's axis: (a0, a1) past the nozzle's end in m, the line's offset from the axis, and whether the line's
    height lies within the coupling's span."""
    g = np.asarray(F.G_REST[:2], float)
    u = np.array(F.U)
    rel = np.array([F.JX, 0.0]) - g
    s0 = float(rel @ u)
    off = abs(float(rel[0] * u[1] - rel[1] * u[0]))
    half = math.sqrt(max(0.0, F.CUP_R ** 2 - off ** 2))
    z = F.G_REST[2] - F.TOP
    inside = F.Z_MOUTH - F.COUPLING_L <= z <= F.Z_MOUTH
    return (s0 - half + NOZZLE_Y, s0 + half + NOZZLE_Y), off, inside


# -- the real rest's side rails and the real torch's rear (not in the sim) ---------------------------------------------
# In the rest's frame: a along the flame (U) from the grip point, c across it, z over the rest's top.
RAILS = ((-0.056, -0.036), (0.0155, 0.0200), 0.004)   # along span, |across| span, height [CHOICE]
SLEEVE = (0.028, 0.058, 0.015, 0.004)   # the rear clamp sleeve: torch-y span, half-width at its sides, proud on top
REAR = ((-0.011, 0.011), (0.030, 0.052), (0.015, 0.030))   # the sensor case, sleeve's top [CHOICE]
FITTING = ((-0.008, 0.008), (0.016, 0.032), (-0.025, -0.011))   # the hose's elbow fitting under the handle [CHOICE]
HOLE_R = 0.011                          # the hole in the rest the fitting sits in [CHOICE]
RISER = (0.0, 0.024, -0.025)            # the hose leaves the fitting here (torch frame), straight down: seated, the
                                        # fitting sits in a hole in the rest's block and the hose runs down through
                                        # the block and the bench to a counterweight that keeps it taut [CHOICE]
HOSE_R = 0.010                          # the sleeved hose's radius [EST]
HOSE_SAG = 0.015                        # the most the taut hose bows off its line, at mid-span (measured once on
                                        # the rig) [EST]
HOSE_STEP = 0.002
HOSE_PARTS = 20
GRID = 0.001


def _box_pts(x0, x1, y0, y1, z0, z1):
    xs, ys, zs = (np.arange(a, b + 1e-9, GRID) for a, b in ((x0, x1), (y0, y1), (z0, z1)))
    out = []
    for X, Y, Z in ((xs, ys, (z0, z1)), (xs, (y0, y1), zs), ((x0, x1), ys, zs)):
        out.append(np.stack(np.meshgrid(X, Y, Z, indexing="ij"), -1).reshape(-1, 3))
    return np.concatenate(out)


def _torch_points(sim=False):
    """Surface samples (torch frame, <= 1 mm apart). sim: the sim's handle box alone. Else the real torch's parts that
    could meet the rails or the base: the handle to the rear sleeve's end, the sleeve (flush underneath, proud at the
    sides and on top), the sensor case, the hose fitting, the burner tube and the nozzle."""
    hw = F.HANDLE_W / 2
    if sim:
        return _box_pts(-hw, hw, F.HANDLE[0], F.HANDLE[1], -hw, hw)
    out = [_box_pts(-hw, hw, F.HANDLE[0], SLEEVE[1], -hw, hw),
           _box_pts(-SLEEVE[2], SLEEVE[2], SLEEVE[0], SLEEVE[1], -hw, hw + SLEEVE[3]),
           _box_pts(*REAR[0], *REAR[1], *REAR[2]), _box_pts(*FITTING[0], *FITTING[1], *FITTING[2])]

    def cyl(y0, y1, r):
        th = np.arange(0.0, 2 * math.pi, GRID / r)
        ys = np.arange(y0, y1 + 1e-9, GRID)
        T, Y = np.meshgrid(th, ys, indexing="ij")
        out.append(np.stack([r * np.cos(T).ravel(), Y.ravel(), r * np.sin(T).ravel()], -1))

    cyl(-0.120, -0.020, 0.005)
    cyl(-0.140, -0.120, 0.009)
    return np.concatenate(out)


def _rail_frame():
    u = np.array([F.U[0], F.U[1], 0.0])
    c = np.array([-u[1], u[0], 0.0])
    return np.asarray(F.G_REST, float), u, c, F.TOP + F.REST_TOP


def _rail_dist(P):
    g, u, c, z0 = _rail_frame()
    rel = P - g
    a, cc, z = rel @ u, rel @ c, P[:, 2] - z0
    (a0, a1), (c0, c1), h = RAILS
    da = np.maximum(np.maximum(a0 - a, a - a1), 0.0)
    dz = np.maximum(np.maximum(0.0 - z, z - h), 0.0)
    dc = np.maximum(np.maximum(c0 - np.abs(cc), np.abs(cc) - c1), 0.0)
    return np.sqrt(da * da + dc * dc + dz * dz)


def _rail_points():
    g, u, c, z0 = _rail_frame()
    (a0, a1), (c0, c1), h = RAILS
    out = []
    for sgn in (-1.0, 1.0):
        A, Cc, Z = (np.arange(lo, hi + 1e-9, GRID) for lo, hi in ((a0, a1), (c0, c1), (0.0, h)))
        for X, Y, W in ((A, Cc, (0.0, h)), (A, (c0, c1), Z), ((a0, a1), Cc, Z)):
            q = np.stack(np.meshgrid(X, Y, W, indexing="ij"), -1).reshape(-1, 3)
            out.append(g + np.outer(q[:, 0], u) + np.outer(sgn * q[:, 1], c) + np.outer(z0 - g[2] + q[:, 2], [0, 0, 1]))
    return np.concatenate(out)


def _torch_pose(p, r, grip0, g_rest):
    """The carried torch's grip point and rotation, placed TORCH_SAG low in the grip but never under its rest pose."""
    o, r_grip = grip0
    g = np.asarray(p, float) + np.asarray(r, float) @ o
    g[2] = max(g[2], g_rest[2])
    return g, np.asarray(r, float) @ r_grip


def _update(best, P, acts, arms, t, allow=0.0, skip=()):
    """Update best[(arm, group)] = (gap, t, link) with points P (gap = distance less ``allow``) against every piece of
    ``arms`` but the links in ``skip``. A piece whose box is already farther than the group's best is skipped."""
    for arm in arms:
        poses = poses_q(acts[arm], arm)
        for name, frame, V, h in _pieces(arm):
            if name in skip:
                continue
            key = (arm, group(name))
            cur = best.get(key, (math.inf,))[0]
            r, p = poses[frame][:3, :3], poses[frame][:3, 3]
            Pl = (P - p) @ r
            if box_lower(Pl, V) - allow >= cur:
                continue
            d = _hull_dist(Pl, V, h)[0] - allow
            if d < cur:
                best[key] = (d, t, name)
    return best


def cradle(seed=None):
    """The rails and the real torch's rear against the demo. Returns a dict of results (m, s): rails_arm, every arm
    link's nearest approach to the rails; rear, every left-arm link's (the base included) nearest approach to the real
    torch's rear (handle end, sleeve, sensor case and hose fitting), on its rest and carried; sim_overlap, how deep
    the sim's handle lies inside the left base's hull on its rest; rest_base, the real rear's gap to that base on its
    rest; and the lift's offsets and the carried torch's nearest approach to the rails."""
    tp, rp = _torch_points(), _rail_points()
    rear = tp[(tp[:, 1] > SLEEVE[0] - 1e-9) | (tp[:, 2] < -F.HANDLE_W / 2 - 1e-9)]   # the fitting is under the handle
    g_rest = np.asarray(F.G_REST, float)
    r_rest = rz(F.TORCH_YAW)
    under = RAILS[2] - (F.G_REST[2] - F.HANDLE_W / 2 - F.TOP - F.REST_TOP)
    res = {"rails_arm": {}, "rear": {}, "off_max": 0.0, "moved": None, "rails_torch": (math.inf, None)}
    base = [(V, h) for n, f, V, h in _pieces("left") if n == "base"]
    pb = poses_q(np.zeros(7), "left")["base"]

    def base_gap(W):
        Pl = (W - pb[:3, 3]) @ pb[:3, :3]
        return min(_hull_dist(Pl, V, h)[0] for V, h in base)

    def base_depth(W):
        Pl = (W - pb[:3, 3]) @ pb[:3, :3]
        return max(float(-(Pl @ h.equations[:, :3].T + h.equations[:, 3]).max(1).min()) for V, h in base)

    res["sim_overlap"] = base_depth(g_rest + _torch_points(sim=True) @ r_rest.T)
    # WHY: the sim's rest block (solder_joint.bench_base: under the handle's span less 20 mm, 40 mm wide, from the bench
    # to REST_TOP) and the real hole through it, against the left base's hulls (its mounting plate lies on the bench)
    z0 = F.TOP - g_rest[2]
    mid = 0.5 * (F.HANDLE[0] + F.HANDLE[1]) - 0.5 * (F.HANDLE[1] - F.HANDLE[0] - 0.020)
    blk = _box_pts(-0.020, 0.020, mid, mid + F.HANDLE[1] - F.HANDLE[0] - 0.020, z0, F.REST_TOP + F.TOP - g_rest[2])
    res["block_base"] = base_depth(g_rest + blk @ r_rest.T)
    th = np.arange(0.0, 2 * math.pi, GRID / HOLE_R)
    zs = np.arange(z0, F.REST_TOP + F.TOP - g_rest[2] + 1e-9, GRID)
    T, Z = np.meshgrid(th, zs, indexing="ij")
    hole = np.stack([RISER[0] + HOLE_R * np.cos(T).ravel(), RISER[1] + HOLE_R * np.sin(T).ravel(), Z.ravel()], -1)
    res["hole_base"] = base_gap(g_rest + hole @ r_rest.T)
    res["rest_base"] = base_gap(g_rest + rear @ r_rest.T)
    cleared = False
    for t, acts, p, r, grip0 in _walk_all(seed):
        _update(res["rails_arm"], rp, acts, ("left", "right"), t)
        g, rt = (g_rest, r_rest) if grip0 is None else _torch_pose(p, r, grip0, g_rest)
        _update(res["rear"], g + rear @ rt.T, acts, ("left",), t)
        if grip0 is None:
            continue
        W = g + tp @ rt.T
        off = float(np.hypot(*(g - g_rest)[:2]))
        if not cleared:
            if g[2] - g_rest[2] < under - 1e-9:
                res["off_max"] = max(res["off_max"], off)
            else:
                cleared = True
        if res["moved"] is None:
            if off <= 0.0005:
                continue
            res["moved"] = (t, g[2] - g_rest[2])
        d = float(_rail_dist(W).min())
        if d < res["rails_torch"][0]:
            res["rails_torch"] = (d, t)
    return res


def _walk_all(seed):
    """Every control step of the demo from t = 0, with grip0 None until the torch is held."""
    L, PL, PR, info = _plans(seed)
    end, grip0, k = min(PL.duration, PR.duration), None, 0
    while k * DT <= end + 1e-9:
        t = k * DT; k += 1
        acts = {"left": PL.action7(t)[0], "right": PR.action7(t)[0]}
        p, r = ik.tcp_pose(PL.q, PL.base_pos)
        if t >= info["held_from"] - 1e-9 and grip0 is None:
            grip0 = lift_grip(p, r)
        yield t, acts, p, r, grip0


def hose(seed=None):
    """The taut hose, the line from the fitting (RISER) to the mouth of the hole in the rest it runs down through (under
    the rest's top it is inside the steel block, and not checked), against every link of both arms (all the demo; the
    torch on its rest until the lift) and against the off-rest and on-rest segments. The line is split in HOSE_PARTS
    and each part grown by the hose's radius and the most a parabolic bow of HOSE_SAG at mid-span puts there. Returns
    a dict of gaps (m) to the grown hose and the longest span (m) from the fitting to the hole."""
    g_rest, r_rest = np.asarray(F.G_REST, float), rz(F.TORCH_YAW)
    guide = g_rest + r_rest @ np.array(RISER)       # the hole's mouth in the rest's top, under the seated fitting
    guide[2] = F.TOP + F.REST_TOP
    res = {"arms": {}, "off": (math.inf, None), "rest": (math.inf, None), "span": 0.0}
    rest_pts, _ = axis_points(*rest_axis(r_rest), near=REST_SLIDE)
    t_lift = lift_end(F.plan(0.0, seed)[0])
    edges = np.linspace(0.0, 1.0, HOSE_PARTS + 1)
    allow = [HOSE_STEP / 2 + HOSE_R + 4 * HOSE_SAG * max(u * (1 - u) for u in (a0, a1, min(max(0.5, a0), a1)))
             for a0, a1 in zip(edges, edges[1:])]
    for t, acts, p, r, grip0 in _walk_all(seed):
        g, rt = (g_rest, r_rest) if grip0 is None else _torch_pose(p, r, grip0, g_rest)
        e = g + rt @ np.array(RISER)
        res["span"] = max(res["span"], float(np.linalg.norm(guide - e)))
        n = max(2, int(np.linalg.norm(guide - e) / HOSE_STEP) + 1)
        u = np.linspace(0.0, 1.0, n)
        H = e + np.outer(u, guide - e)
        for i, (a0, a1) in enumerate(zip(edges, edges[1:])):
            # WHY: up to the rest's top the hose is inside the hole in the steel block, so only the part above it
            # is exposed (seated, none is)
            m = (u >= a0 - 1e-9) & (u <= a1 + 1e-9) & (H[:, 2] > guide[2] + 1e-6)
            if not m.any():
                continue
            _update(res["arms"], H[m], acts, ("left", "right"), t, allow=allow[i])
            if t <= t_lift + 1e-9:
                d = _pts_dist(rest_pts, H[m]) - allow[i]
                if d < res["rest"][0]:
                    res["rest"] = (d, t)
            if grip0 is not None:
                P, _ = _segments(p, r, grip0, 0)
                d = _pts_dist(P, H[m]) - allow[i]
                if d < res["off"][0]:
                    res["off"] = (d, t)
    return res


def _pts_dist(A, B):
    """Min distance between two point sets (chunked)."""
    best = math.inf
    for i in range(0, len(A), 200):
        d = np.sqrt(((A[i:i + 200, None, :] - B[None, :, :]) ** 2).sum(-1)).min()
        best = min(best, float(d))
    return best


def box(seed=None):
    """Extents (world, m) of the nozzle's end and of every checked point, off rest and on rest."""
    nz, pts = [], []
    for t, acts, p, r, grip0 in _walk(seed):
        P, A = _segments(p, r, grip0, 0)
        nz.append(P[np.argmin(np.abs(A))])
        pts.append(P[[0, -1]])
    rest, _ = axis_points(*rest_axis(rz(F.TORCH_YAW)), near=REST_SLIDE)
    nz, pts = np.array(nz), np.concatenate(pts)
    return (nz.min(0), nz.max(0)), (pts.min(0), pts.max(0)), (rest.min(0), rest.max(0))


def turn():
    """The arm's flame direction turned by jogging joint6, then joint5, 10 degrees at the lift's pose (degrees)."""
    L, PL, PR, info = _plans(None)
    k = 0
    while k * DT <= info["held_from"] + 1e-9:
        PL.action7(k * DT); k += 1
    q = np.array(PL.q, float)
    p, r = ik.tcp_pose(q, PL.base_pos)
    grip0 = lift_grip(p, r)
    u0 = r @ grip0[1] @ U_TORCH
    out = {}
    for j in (5, 4):
        q2 = q.copy(); q2[j] += math.radians(10.0)
        _, r2 = ik.tcp_pose(q2, PL.base_pos)
        u2 = r2 @ grip0[1] @ U_TORCH
        out[f"joint{j + 1}"] = math.degrees(math.acos(max(-1.0, min(1.0, float(u0 @ u2)))))
    return out


def grip():
    """The pads' width along the handle where they meet it (the finger hulls within the handle's height of the pad
    tips, fingers closed on 22 mm), and the play that leaves: forward until the pads run off the handle's front onto
    the burner tube, back until a pad meets the rear sleeve."""
    q = {f"joint{i + 1}": 0.0 for i in range(6)}
    q["joint7"] = q["joint8"] = 0.0
    P = fk_link_poses_world(q, (0.0, 0.0, 0.0))
    gi = np.linalg.inv(P["gripper"])
    lo, hi = math.inf, -math.inf
    for n in ("tip_left", "tip_right"):
        V = np.concatenate([h.vertices for h in link_hulls()[n]])
        W = (gi @ P[n] @ np.c_[V, np.ones(len(V))].T).T[:, :3]
        # WHY: the pad tips sit TIP - TCP_OVER_G (6 mm) under the handle's axis, so the handle meets the pads from
        # the tips to HANDLE_W / 2 plus that over them
        band = W[W[:, 2] >= W[:, 2].max() - (F.HANDLE_W / 2 + F.TIP - F.TCP_OVER_G)]
        lo, hi = min(lo, band[:, 1].min()), max(hi, band[:, 1].max())
    half = (hi - lo) / 2
    return hi - lo, -F.HANDLE[0] + half, SLEEVE[0] - half


if __name__ == "__main__":
    kw = {}
    for a in sys.argv[1:]:
        k, v = a.split("=")
        kw[k] = int(v)
    if "cli" in kw:
        kw["seed"] = zlib.crc32(f"{kw.pop('cli')}:None:0".encode())
    seed = kw.get("seed")
    mm = lambda x: 1000 * x
    if kw.get("grip"):
        w, fwd, back = grip()
        print(f"pads {mm(w):.1f} mm along the handle; play forward {mm(fwd):.1f} mm (pads off the handle's front), "
              f"back {mm(back):.1f} mm (a pad on the rear sleeve)")
    elif kw.get("turn"):
        for j, d in turn().items():
            print(f"{j} jogged 10 deg: the arm's flame direction turns {d:.2f} deg")
    elif kw.get("box"):
        (n0, n1), (p0, p1), (r0, r1) = box(seed)
        f = lambda v: f"x {v[0]:.3f} y {v[1]:.3f} z over bench {mm(v[2] - F.TOP):.0f} mm"
        print(f"nozzle's end off rest: {f(n0)} to {f(n1)}")
        print(f"every checked point off rest: {f(p0)} to {f(p1)}")
        print(f"every checked point on rest: {f(r0)} to {f(r1)}")
    elif kw.get("cradle"):
        # WHY: printed gaps are sampled minima less 1 mm (every surface point is within 0.71 mm of a 1 mm sample)
        res = cradle(seed)
        q = lambda d: mm(max(0.0, d - GRID))
        print(f"sim handle inside the left base's hull by {mm(res['sim_overlap']):.1f} mm on its rest")
        print(f"real torch's rear on its rest: {q(res['rest_base']):.1f} mm from the left base")
        print(f"sim rest block inside the left base's hull by {mm(res['block_base']):.1f} mm; the hose's hole through "
              f"it {q(res['hole_base']):.1f} mm from that hull")
        for (arm, grp), (d, t, name) in sorted(res["rear"].items()):
            print(f"real torch's rear to {arm} {grp:6s} {q(d):7.1f} mm at t {t:6.2f} s ({name})")
        tm, rise = res["moved"]
        print(f"torch: plan offset {mm(res['off_max']):.2f} mm in the lift below the rails' tops; first sideways past "
              f"0.5 mm at t {tm:.2f} s, {mm(rise):.1f} mm up; from then {q(res['rails_torch'][0]):.1f} mm or more "
              f"from the rails (t {res['rails_torch'][1]:.2f} s)")
        for (arm, grp), (d, t, name) in sorted(res["rails_arm"].items()):
            print(f"{arm:5s} {grp:6s} {q(d):7.1f} mm from the rails at t {t:6.2f} s ({name})")
    elif kw.get("hose"):
        res = hose(seed)
        # WHY: gaps are to the hose grown by its radius, its bow and half the 2 mm sample spacing
        q = lambda d: mm(max(0.0, d))
        for (arm, grp), (d, t, name) in sorted(res["arms"].items()):
            print(f"grown hose to {arm:5s} {grp:6s} {q(d):7.1f} mm at t {t:6.2f} s ({name})")
        print(f"hose from the fitting to the hole: {mm(res['span']):.1f} mm at most")
        print(f"grown hose to the off-rest segment {q(res['off'][0]):.1f} mm (t {res['off'][1]:.2f} s), "
              f"to the on-rest segment {q(res['rest'][0]):.1f} mm (t {res['rest'][1]:.2f} s)")
    else:
        band, rest = kw.get("band", 0), kw.get("rest", 0)
        less = STEP / 2 + (BAND_STEP / 2 if band else 0.0)
        series, speeds = sweep(seed, band, rest)
        rows = {}
        for (arm, name, zone), s in series.items():
            key = (arm, group(name), zone)
            g, t, a = min(((g, t, a) for t, g, a in s), key=lambda x: x[0])
            if g < rows.get(key, (math.inf,))[0]:
                rows[key] = (g, t, a, name, s)
        for (arm, grp, zone), (g, t, a, name, s) in sorted(rows.items()):
            extra = ""
            if zone == "near":
                ms = min((margin(series[(arm, n_, "near")]) + (n_,) for (ar, n_, z) in series
                          if ar == arm and z == "near" and group(n_) == grp), key=lambda x: x[0])
                extra = f"; margin {mm(ms[0] - less):.1f} mm at t {ms[1]:.2f} s ({ms[2]})"
            if zone == "far" and rest:
                dw = max((dwell(series[(arm, n_, "far")]) for (ar, n_, z) in series
                          if ar == arm and z == "far" and group(n_) == grp), key=lambda x: x[0])
                extra = f"; under {mm(TRIP):.0f} mm for {dw[0]:.2f} s" + (f" (t {dw[1][0]:.2f} to {dw[1][1]:.2f})"
                                                                           if dw[1] else "")
            where = "beyond 150 mm (box bound)" if math.isnan(a) else f"{mm(a):.0f} mm past the nozzle"
            print(f"{arm:5s} {grp:6s} {zone:4s} {mm(max(0.0, g - less)):7.1f} mm at t {t:6.2f} s, {where} "
                  f"({name}){extra}")
        for k_, (v, t) in sorted(speeds.items()):
            print(f"fastest {k_} point {v:.3f} m/s at t {t:.2f} s")
        if rest:
            (a0, a1), off, inside = rest_line_coupling()
            print(f"the rest's line passes the coupling {mm(a0):.0f} to {mm(a1):.0f} mm past the nozzle's end, "
                  f"{mm(off):.1f} mm off its axis, height within the coupling: {inside}")
