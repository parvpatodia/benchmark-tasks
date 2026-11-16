"""clearance.py [seed=N] [dt=0.25]: offline, along the demo's plan (IK at the control rate, as the scripted policy runs
it): the right gripper's hulls against the bench and the mug, and the carried tube and handle (their poses from the
test's kinematic stand-in) against the mug and the bench. Prints every pair closer than 3 mm with its time and step
label. Contact the task needs (the handle's ends on the faces, the nozzle over a face, the tube in its holder at the
start and end) is listed too, so read the labels. No sim."""
import math
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import numpy as np  # noqa: E402
import mug_handle as F  # noqa: E402
import test_mug_handle as T  # noqa: E402
from geom import gap  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.analysis import fk_link_poses_world, world_hulls  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402

HAND = ("gripper", "tip_left", "tip_right")


def verts(shape, T4=None):
    c = np.asarray(shape.center, float)
    if shape.kind == "hull":
        v = np.asarray(shape.points, float)
    elif shape.kind == "box":
        h = np.asarray(shape.size) / 2
        v = np.array([c + h * np.array([sx, sy, sz]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    else:
        r, L = shape.size
        a = "XYZ".index(shape.axis)
        o = [i for i in range(3) if i != a]
        pts = []
        for z in (-L / 2, L / 2):
            for k in range(24):
                p = np.zeros(3)
                p[a] = z
                p[o[0]] = r / math.cos(math.pi / 24) * math.cos(2 * math.pi * k / 24)
                p[o[1]] = r / math.cos(math.pi / 24) * math.sin(2 * math.pi * k / 24)
                pts.append(c + p)
        v = np.array(pts)
    if T4 is not None:
        v = v @ T4[:3, :3].T + T4[:3, 3]
    return v


def pose_T(pos, yaw=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    T4 = np.eye(4)
    T4[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T4[:3, 3] = pos
    return T4


TCP_OFFSET = (0.0, 0.0, 0.134)


def _qd(a):
    qd = {f"joint{i + 1}": float(a[i]) for i in range(6)}
    qd["joint7"] = qd["joint8"] = float(a[6]) * FINGER_OPEN
    return qd


def hand_sets(a):
    return [(n, h.vertices) for n, h in world_hulls(fk_link_poses_world(_qd(a), station.BASES["right"]), HAND)]


def label_at(ws, t):
    t0 = 0.0
    for w in ws:
        if t <= t0 + w.duration:
            return w.label
        t0 += w.duration
    return ws[-1].label


def main(seed=None, dt=0.25, near=0.003):
    R, _ = F.plan(0.0, seed)
    P = ArmPlan(R, station.BASES["right"])
    F.reset_memory(seed)
    kin = T.Kin(seed)
    kin.run(R, tail=0.0)
    bench = [(f"bench{i}", verts(s) + np.array([0, 0, F.TOP])) for i, s in enumerate(F.BENCH.base.shapes) if s.collide]
    mp = np.array(F.layout(seed)["mug"][:3])
    mug = [(f"mug{i}", verts(s) + mp) for i, s in enumerate(F.MUG.base.shapes) if s.collide]
    # WHY: the stubs are links of their own (sprung), so the base alone would leave them out
    mug += [(f"mug_{lk.name}", verts(s) + mp) for lk in F.MUG.links for s in lk.shapes if s.collide]
    tube_sh = [s for s in F.TUBE.base.shapes if s.collide] + [s for lk in F.TUBE.links for s in lk.shapes]
    worst, k, step = {}, 0, 0.05
    while k * step <= P.duration + 1e-9:
        t = k * step
        a, _ = P.action7(t)
        k += 1
        if (k - 1) % max(1, round(dt / step)) or k - 1 >= len(kin.trace):
            continue
        o = kin.trace[k - 1]
        # WHY: a held part rides the FK hand (the stand-in's linear path drifts from the IK plan's), so take it from there
        g = fk_link_poses_world(_qd(a), station.BASES["right"])["gripper"]
        tcp = g[:3, 3] + g[:3, :3] @ np.array(TCP_OFFSET)
        tpos = tcp - np.array([0.0, 0.0, F.TCP_OVER_T]) if kin.holds[k - 1] == "tube" else o["tube"].pos
        q = o["handle"].quat_xyzw
        hyaw = 2 * math.atan2(q[2], q[3])
        hpos = tcp - np.array([0.0, 0.0, F.TCP_OVER_H]) if kin.holds[k - 1] == "handle" else o["handle"].pos
        tube = [(f"tube{i}", verts(s, pose_T(tpos))) for i, s in enumerate(tube_sh)]
        handle = [(f"handle{i}", verts(s, pose_T(hpos, hyaw))) for i, s in enumerate(F.HANDLE.shapes)]
        pairs = []
        rigid = [(f"tube_core{i}", verts(s, pose_T(tpos))) for i, s in enumerate(F.TUBE.base.shapes)
                 if s.collide]
        if kin.holds[k - 1] != "tube":   # an open hand must not touch the tube's walls either
            rigid += [(f"tube_wall{j}", verts(s, pose_T(tpos))) for j, lk in enumerate(F.TUBE.links)
                      for s in lk.shapes if s.collide]
        if kin.holds[k - 1] != "handle":
            rigid += [(f"handle_{i}", verts(s, pose_T(o["handle"].pos, 2 * math.atan2(o["handle"].quat_xyzw[2],
                                                                                       o["handle"].quat_xyzw[3]))))
                      for i, s in enumerate(F.HANDLE.shapes)]
        for nh, H in hand_sets(a):
            pairs += [(nh, nb, H, V) for nb, V in bench + mug + rigid]   # the fingers may touch only the tube's walls
        pairs += [(nt, nb, A, V) for nt, A in tube for nb, V in mug + bench]
        pairs += [(nt, nb, A, V) for nt, A in handle for nb, V in mug + bench]
        for x, y, A, B in pairs:
            if np.any(A.min(0) - near > B.max(0)) or np.any(B.min(0) - near > A.max(0)):
                continue
            g = gap(A, B)
            key = (x, y, label_at(R, t))
            if g < near and g < worst.get(key, (1, 0))[0]:
                worst[key] = (g, t)
    for (x, y, lab), (g, t) in sorted(worst.items(), key=lambda z: z[1][1]):
        print(f"{1000 * g:+7.1f} mm  {x:10s} vs {y:8s} at {t:6.2f} s ({lab})")
    print("pairs under 3 mm:", len(worst))


if __name__ == "__main__":
    kw = {}
    for arg in sys.argv[1:]:
        k, v = arg.split("=")
        kw[k] = int(v) if k == "seed" else float(v)
    main(**kw)
