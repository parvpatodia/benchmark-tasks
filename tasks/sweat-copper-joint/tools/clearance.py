"""clearance.py [seed=N] [dt=0.25]: offline, along the demo's plan (IK at the control rate, as the scripted policy runs
it): each gripper's hulls, the carried torch and the carried wire against the bench's colliders, and the torch against
the wire. Prints every pair closer than 3 mm with its time and the plan's step label. No sim."""
import math
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
import solder_joint as F  # noqa: E402
from geom import gap  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.analysis import fk_link_poses_world, world_hulls  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

HAND = ("gripper", "tip_left", "tip_right")


def verts(shape, T=None):
    """A shape's vertices (16-gon prisms for cylinders), turned by T (4x4) if given."""
    c = np.asarray(shape.center, float)
    if shape.kind == "hull":
        v = np.asarray(shape.points, float)
    elif shape.kind == "box":
        h = np.asarray(shape.size) / 2
        v = np.array([c + h * np.array(s) for s in np.ndindex(2, 2, 2)]) if False else np.array(
            [c + h * np.array([sx, sy, sz]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    else:
        r, L = shape.size
        a = "XYZ".index(shape.axis)
        o = [i for i in range(3) if i != a]
        pts = []
        for z in (-L / 2, L / 2):
            for k in range(16):
                p = np.zeros(3)
                p[a] = z
                p[o[0]] = r / math.cos(math.pi / 16) * math.cos(2 * math.pi * k / 16)
                p[o[1]] = r / math.cos(math.pi / 16) * math.sin(2 * math.pi * k / 16)
                pts.append(c + p)
        v = np.array(pts)
    if T is not None:
        v = v @ T[:3, :3].T + T[:3, 3]
    return v


def bench_sets():
    off = np.array([F.JX, 0.0, F.TOP])
    out = [(f"bench{i}", verts(s) + off) for i, s in enumerate(F.BENCH.base.shapes) if s.collide]
    out += [("face", verts(F.BENCH.links[0].shapes[0]) + off)]
    return out


def body_T(pos, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[:3, 3] = pos
    return T


def hand_sets(a, arm):
    qd = {f"joint{i + 1}": float(a[i]) for i in range(6)}
    qd["joint7"] = qd["joint8"] = float(a[6]) * FINGER_OPEN
    return [(n, h.vertices) for n, h in world_hulls(fk_link_poses_world(qd, station.BASES[arm]), HAND)]


def label_at(ws, t):
    t0 = 0.0
    for w in ws:
        if t <= t0 + w.duration:
            return w.label
        t0 += w.duration
    return ws[-1].label


def main(seed=None, dt=0.25, near=0.003):
    L, R, info = F.plan(0.0, seed)
    PL, PR = ArmPlan(L, station.BASES["left"]), ArmPlan(R, station.BASES["right"])
    B = bench_sets()
    torch_shapes = [s for s in F.TORCH.shapes if s.collide]
    held, k, step = info["held_from"], 0, 0.05
    lift_wire = sum(w.duration for w in R[:4])          # the wire rides the hand once gripped
    worst = {}
    end = min(PL.duration, PR.duration)
    while k * step <= end + 1e-9:
        t = k * step
        al, _ = PL.action7(t)
        ar, _ = PR.action7(t)
        k += 1
        if (k - 1) % max(1, round(dt / step)):
            continue
        gl = F._g_at(L, t) if t >= held else np.asarray(F.G_REST)
        tor = [verts(s, body_T(gl, F.TORCH_YAW)) for s in torch_shapes]
        tr = F._g_at(R, t) + np.array([0.0, 0.0, F.TCP_OVER_G + F.TORCH_SAG]) if t >= lift_wire else None
        wire = None
        if tr is not None:
            c = tr - np.array([0.0, 0.0, F.TCP_OVER_COIL])
            wire = [verts(sh, body_T(c, 0.0)) for sh in F.WIRE.shapes]     # coil, straight end (each convex)
        pairs = []
        for nh, H in hand_sets(al, "left"):
            pairs += [(f"left {nh}", nb, H, V) for nb, V in B]
        for nh, H in hand_sets(ar, "right"):
            pairs += [(f"right {nh}", nb, H, V) for nb, V in B]
        for i, V in enumerate(tor):
            pairs += [(f"torch{i}", nb, V, W) for nb, W in B if not (t < held + 1.0 and nb == f"bench{5}")]
        if wire is not None:
            for j, Wv in enumerate(wire):
                nm = ("coil", "wire end")[j]
                pairs += [(nm, nb, Wv, W) for nb, W in B if nb not in ("face",)]
                pairs += [(nm, f"torch{i}", Wv, V) for i, V in enumerate(tor)]
        for a, b, A, Bv in pairs:
            if np.any(A.min(0) - near > Bv.max(0)) or np.any(Bv.min(0) - near > A.max(0)):
                continue
            g = gap(A, Bv)
            if g < near and g < worst.get((a, b), (1, 0, ""))[0]:
                worst[(a, b)] = (g, t, label_at(L if a.startswith(("left", "torch")) else R, t))
    for (a, b), (g, t, lab) in sorted(worst.items(), key=lambda x: x[1][0]):
        print(f"{1000 * g:+7.1f} mm  {a:16s} vs {b:8s} at {t:6.2f} s ({lab})")
    print("pairs under 3 mm:", len(worst))


if __name__ == "__main__":
    kw = {}
    for a in sys.argv[1:]:
        k, v = a.split("=")
        kw[k] = int(v) if k == "seed" else float(v)
    main(**kw)
