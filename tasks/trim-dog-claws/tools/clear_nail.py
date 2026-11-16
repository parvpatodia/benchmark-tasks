"""Offline clearance along the demo plan (no sim): both hands and the held clipper against the trainer (stand, mat, leg,
paw, claws at their cut length at that moment), the clipper's ring against every claw, and the hands against the
clipper while it lies on its mat. Prints every pair nearer than ``near`` (m). Run with yam_arena on PYTHONPATH."""
import math
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import nail_trim as F  # noqa: E402
from hulls import hand_sets, label_at, verts  # noqa: E402
from geom import gap  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402


def _T(pos, yaw=0.0):
    T = np.eye(4)
    c, s_ = math.cos(yaw), math.sin(yaw)
    T[:3, :3] = [[c, -s_, 0], [s_, c, 0], [0, 0, 1]]
    T[:3, 3] = pos
    return T


def trainer_sets(removed):
    Tt = _T((F.T_POS[0], F.T_POS[1], F.TOP), F.T_YAW)
    out = [(f"stand{i}", verts(s, Tt)) for i, s in enumerate(F.TRAINER.base.shapes) if s.collide]
    for k in range(4):
        tip = next(link for link in F.TRAINER.links if link.name == f"tip{k}")
        out.append((f"claw{k + 1}", verts(tip.shapes[0], Tt) + np.array([0.0, 0.0, -removed[k]])))
    return out


def clipper_sets(grip, yaw=0.0):
    T = _T(grip, yaw)
    out = [(("spine", "neck")[i] if i < 2 else f"ring{i - 2}", verts(s, T))
           for i, s in enumerate(F.CLIPPER.base.shapes) if s.collide]
    out += [(link.name, verts(link.shapes[0], T)) for link in F.CLIPPER.links if link.name != "blade"]
    return out


def cut_times(R):
    """(t, claw, cut) for each squeeze in the right plan, at the waypoint's end."""
    out, t, k = [], 0.0, 0
    _, info = F.plan(0.0, SEED)
    cuts = info["cuts"]
    for w in R:
        t += w.duration
        if w.label.startswith("squeeze"):
            out.append((t, *cuts[k]))
            k += 1
    return out


SEED = None


class _Idle:
    """The left arm with no plan: yam_scripted holds it at READY with its fingers open."""
    duration = float("inf")

    def action7(self, t):
        import numpy as _n
        from yam_arena.constants import READY
        return _n.r_[READY, 1.0], None


def main(seed=None, dt=0.25, near=0.003):
    global SEED
    SEED = seed
    R, info = F.plan(0.0, seed)
    L = []
    PL, PR = _Idle(), ArmPlan(R, station.BASES["right"])
    picked = sum(w.duration for w in R[:4])      # the clipper rides the hand from here
    k_go = [w.label for w in R].index("let go")
    placed = sum(w.duration for w in R[:k_go])   # the clipper rides the hand until "let go" starts
    cuts = cut_times(R)
    worst, step, k = {}, 0.05, 0
    end = min(PL.duration, PR.duration)
    while k * step <= end + 1e-9:
        t = k * step
        al, _ = PL.action7(t)
        ar, tcp = PR.action7(t) if False else (PR.action7(t)[0], None)
        k += 1
        if (k - 1) % max(1, round(dt / step)):
            continue
        removed = [0.0] * 4
        for tc, kk, c in cuts:
            if t >= tc:
                removed[kk] += c
        TS = trainer_sets(removed)
        pos = _tcp_at(R, t)
        grip = (pos - np.array([0.0, 0.0, F.TCP_OVER_G + F.CLIP_SAG])) if picked <= t <= placed else np.asarray(F.CLIP_START)
        CS = clipper_sets(grip, _yaw_at(R, t) if picked <= t <= placed else 0.0)
        pairs = []
        for nh, H in hand_sets(al, "left"):
            pairs += [(f"left {nh}", nb, H, V) for nb, V in TS + CS]
        for nh, H in hand_sets(ar, "right"):
            pairs += [(f"right {nh}", nb, H, V) for nb, V in TS]
            if not picked - 2.0 <= t <= placed + 2.0:
                pairs += [(f"right {nh}", nb, H, V) for nb, V in CS]
        for nc, C in CS:
            for nb, V in TS:
                if nb.startswith("stand") and picked <= t <= placed:
                    pairs.append((nc, nb, C, V))
                elif not nb.startswith("stand"):
                    pairs.append((nc, nb, C, V))
        for a, b, A, Bv in pairs:
            if np.any(A.min(0) - near > Bv.max(0)) or np.any(Bv.min(0) - near > A.max(0)):
                continue
            g = gap(A, Bv)
            if g < near and g < worst.get((a, b), (1, 0, ""))[0]:
                worst[(a, b)] = (g, t, ("left idle; right: " + label_at(R, t)) if a.startswith("left") else label_at(R, t))
    for (a, b), (g, t, lab) in sorted(worst.items(), key=lambda x: x[1][0]):
        print(f"{1000 * g:+7.1f} mm  {a:16s} vs {b:8s} at {t:6.2f} s ({lab})")
    print("pairs under", near * 1000, "mm:", len(worst))


def _yaw_at(ws, t):
    t0, prev = 0.0, None
    for w in ws:
        if prev is None:
            prev = w.yaw
        if t <= t0 + w.duration:
            a = (t - t0) / w.duration if w.duration > 0 else 1.0
            return prev + a * (w.yaw - prev)
        t0 += w.duration
        prev = w.yaw
    return prev


def _tcp_at(ws, t):
    t0, prev = 0.0, None
    for w in ws:
        p = np.asarray(w.pos, float)
        if prev is None:
            prev = p
        if t <= t0 + w.duration:
            a = (t - t0) / w.duration if w.duration > 0 else 1.0
            return prev + a * (p - prev)
        t0 += w.duration
        prev = p
    return prev


if __name__ == "__main__":
    kw = {}
    for a in sys.argv[1:]:
        k, v = a.split("=")
        kw[k] = int(v) if k == "seed" else float(v)
    main(**kw)
