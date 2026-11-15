"""Offline geometry probe: convex pieces of the tool's links and the tube, posed by joint values; reports
overlaps and gaps between links that can touch (not parent/child). No sim."""
import itertools, math, sys, pathlib, warnings
warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial import ConvexHull
import flare_tool as F

AX = {"X": np.array([1.0, 0, 0]), "Y": np.array([0, 1.0, 0]), "Z": np.array([0, 0, 1.0])}


def piece(s):
    c = np.array(s.center, float)
    if s.kind == "box":
        h = np.array(s.size) / 2
        return np.array([c + h * np.array(d) for d in itertools.product((-1, 1), repeat=3)])
    if s.kind == "cylinder":
        r, L = s.size
        a = AX[s.axis]
        u = np.cross(a, [0, 0, 1.0] if s.axis != "Z" else [1.0, 0, 0]); u /= np.linalg.norm(u)
        v = np.cross(a, u)
        return np.array([c + a * z + r * (math.cos(t) * u + math.sin(t) * v)
                         for z in (-L / 2, L / 2) for t in np.linspace(0, 2 * math.pi, 24, endpoint=False)])
    if s.kind == "capsule":   # the straight part's ends plus a sphere's points round each (a hull of both)
        r, L = s.size
        a = AX[s.axis]
        g = [np.array(d, float) for d in itertools.product((-1, 0, 1), repeat=3) if any(d)]
        g = [d / np.linalg.norm(d) for d in g]
        return np.array([c + a * z + r * d for z in (-L / 2, L / 2) for d in g])
    return np.array(s.points, float)


def rot(axis, a):
    k = AX[axis]; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def link_T(q):
    """World 4x4 per link (rest frame = world), from joint values q (dict)."""
    T = {"base": np.eye(4)}
    J = {j.child: j for j in F.TOOL.joints}
    def get(name):
        if name in T:
            return T[name]
        j = J[name]
        P = get(j.parent or "base")
        M = np.eye(4); a = np.array(j.anchor)
        v = q.get(j.name, 0.0)
        if j.kind == "revolute":
            R = rot(j.axis, v); M[:3, :3] = R; M[:3, 3] = a - R @ a
        else:
            M[:3, 3] = AX[j.axis] * v
        T[name] = P @ M
        return T[name]
    for l in F.TOOL.links:
        get(l.name)
    return T


def pieces(q, tube_pose=None):
    T = link_T(q)
    out = {}
    links = [F.TOOL.base] + list(F.TOOL.links)
    for l in links:
        ps = []
        for s in l.shapes:
            if not s.collide:
                continue
            V = piece(s)
            ps.append((V @ T[l.name][:3, :3].T + T[l.name][:3, 3]))
        out[l.name] = ps
    if tube_pose is not None:
        p = np.array(tube_pose)
        out["tube"] = [piece(s) + p for s in F.TUBE.shapes]
    return out


def axes(V):
    """(face normals, edge directions) of the convex hull of V, unique."""
    try:
        h = ConvexHull(V)
    except Exception:
        return np.eye(3), np.eye(3)
    n = np.unique(h.equations[:, :3].round(6), axis=0)
    sx = h.simplices
    e = (V[sx[:, [1, 2, 0]]] - V[sx]).reshape(-1, 3)
    e /= np.linalg.norm(e, axis=1, keepdims=True) + 1e-12
    e *= np.where(e[:, :1] < 0, -1.0, 1.0)          # one sign per direction, so unique() merges opposite edges
    return n, np.unique(e.round(4), axis=0)


SCREEN = 0.003


def gap(A, B):
    """Signed separation: >0 the largest separating gap (exact for convex polytopes using face and edge axes), <0 the
    smallest penetration over those axes."""
    na, ea = axes(A); nb, eb = axes(B)
    ax = np.vstack([na, nb])
    pa, pb = A @ ax.T, B @ ax.T
    gf = float(np.max(np.maximum(pb.min(0) - pa.max(0), pa.min(0) - pb.max(0))))
    if gf >= SCREEN:              # face normals alone already separate them by SCREEN (a lower bound on the gap)
        return gf
    if len(ea) * len(eb) > 40000:  # WHY: dense mesh hulls; keep the edge axes of the coarser one against a subsample
        rng = np.random.default_rng(0)
        if len(ea) > 200:
            ea = ea[rng.choice(len(ea), 200, replace=False)]
        if len(eb) > 200:
            eb = eb[rng.choice(len(eb), 200, replace=False)]
    cand = [na, nb]
    ex = np.cross(ea[:, None, :], eb[None, :, :]).reshape(-1, 3)
    nrm = np.linalg.norm(ex, axis=1); ex = ex[nrm > 1e-6] / nrm[nrm > 1e-6, None]
    cand.append(ex)
    ax = np.vstack(cand)
    pa, pb = A @ ax.T, B @ ax.T
    g = np.maximum(pb.min(0) - pa.max(0), pa.min(0) - pb.max(0))
    return float(g.max())


def pair_gap(PA, PB, near=0.003):
    best = math.inf
    for A in PA:
        for B in PB:
            if np.any(A.min(0) - near > B.max(0)) or np.any(B.min(0) - near > A.max(0)):
                continue
            best = min(best, gap(A, B))
    return best


JOINTED = {frozenset((j.parent or "base", j.child)) for j in F.TOOL.joints}


def report(title, q, tube_pose=None, only=None):
    P = pieces(q, tube_pose)
    print(f"== {title}  q={ {k: round(v, 5) for k, v in q.items()} }")
    names = list(P)
    for a, b in itertools.combinations(names, 2):
        if frozenset((a, b)) in JOINTED:
            continue
        if only and not ({a, b} & set(only)):
            continue
        g = pair_gap(P[a], P[b])
        if g < 0.003:
            print(f"   {a:10s} {b:10s} {1000 * g:+7.3f} mm")


if __name__ == "__main__":
    tube_hole = (F.BX, F.HY, F.Z_BAR - F.TUBE_L)
    report("rest", {}, F.TUBE_START)
    report("open 0.08", {"hinge": 0.08}, F.TUBE_START)
    for th in (0.0, 0.002, 0.003, 0.004, 0.006):
        P = pieces({"hinge": th}, tube_hole)
        print(f"   tube in hole, hinge {th:.3f}: front {1000 * pair_gap(P['front_half'], P['tube']):+.3f} mm, "
              f"back {1000 * pair_gap(P['base'], P['tube']):+.3f} mm")


def probe2():
    th = 0.0034
    tube_c = (F.BX - 0.00025, F.HY, F.Z_BAR - F.TUBE_L)
    s_end = F.HY - F.PARK_Y
    report("rest, new holder", {}, F.TUBE_START)
    report("open 0.20 parked", {"hinge": 0.20}, None)
    for frac in (0.0, 0.3, 0.6, 0.9, 1.0):
        report(f"slide {frac:.1f}", {"hinge": th, "slide": frac * s_end}, tube_c,
               only=("yoke", "clamp_tip", "clamp", "handle", "cone"))
    for cx in (0.0, 0.0003, 0.0005, 0.0006):
        P = pieces({"hinge": th, "slide": s_end, "clamp_x": cx}, tube_c)
        print(f"   clamp_x {1000 * cx:.1f} mm: tip-front {1000 * pair_gap(P['clamp_tip'], P['front_half']):+.3f} mm")
    for a in (80, 0, -80, 100, 180, 260):
        q = {"hinge": th, "slide": s_end, "clamp_turn": -(a - 80) * F.D2R}
        P = pieces(q, tube_c)
        g = min(pair_gap(P["clamp"], P[k]) for k in ("front_half", "base", "tube"))
        print(f"   T-bar +y end at {a:4d} deg: clamp vs bar/tube {1000 * g:+.2f} mm")


if __name__ == "__main__":
    probe2()
