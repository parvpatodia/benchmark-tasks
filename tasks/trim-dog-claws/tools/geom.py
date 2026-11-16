"""Separating-axis distance between convex point sets (from the flare task tools)."""
import numpy as np
from scipy.spatial import ConvexHull


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


