"""arms.py [dt=0.25] [t0=..] [t1=..] [delay=..] [NAME=value ...]: whole-arm clearance along the demo (every link of the left arm against every link of
the right), offline from the plan's IK. Overrides patch solder_joint constants first (e.g. HINGE_K=40). No sim."""
import math, sys, pathlib, warnings
warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np
import solder_joint as F
from geom import gap
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.constants import FINGER_OPEN, LINKS
from yam_arena.analysis import fk_link_poses_world, world_hulls

ARM_LINKS = [l for l in LINKS if l != "base"]


def hulls_q(a, arm):
    qd = {f"joint{i + 1}": float(a[i]) for i in range(6)}
    qd["joint7"] = qd["joint8"] = float(a[6]) * FINGER_OPEN
    return [(n, h.vertices) for n, h in world_hulls(fk_link_poses_world(qd, station.BASES[arm]), ARM_LINKS)]


def arm_gaps(delay=0.0, dt=0.25, near=0.03, t0=0.0, t1=math.inf, step=0.05, seed=None):
    """[(t, gap, left link, right link)] wherever the arms come within ``near``. The IK is stepped at the control rate
    (``step``) from t = 0, as the scripted policy warm-starts it, so the arms take the run's postures."""
    L, R, _ = F.plan(delay, seed)
    PL, PR = ArmPlan(L, station.BASES["left"]), ArmPlan(R, station.BASES["right"])
    out, k, every = [], 0, max(1, round(dt / step))
    end = min(PL.duration, PR.duration, t1)
    while k * step <= end + 1e-9:
        t = k * step
        al, _ = PL.action7(t)
        ar, _ = PR.action7(t)
        k += 1
        if t < t0 - 1e-9 or (k - 1) % every:
            continue
        best = (math.inf, None, None)
        for nl, A in hulls_q(al, "left"):
            for nr, B in hulls_q(ar, "right"):
                if np.any(A.min(0) - near > B.max(0)) or np.any(B.min(0) - near > A.max(0)):
                    continue
                g = gap(A, B)
                if g < best[0]:
                    best = (g, nl, nr)
        if best[0] < near:
            out.append((t, *best))
    return out


if __name__ == "__main__":
    kw = {"dt": 0.25}
    for a in sys.argv[1:]:
        k, v = a.split("=")
        if k in ("dt", "t0", "t1", "delay"):
            kw[k] = float(v)
        elif k == "seed":
            kw[k] = int(v)
        else:
            setattr(F, k, type(getattr(F, k))(float(v)))
    res = arm_gaps(**kw)
    for t, g, nl, nr in res:
        print(f"{t:6.2f} {1000 * g:+7.1f} mm  left {nl:9s} right {nr}")
    print("worst:", f"{1000 * min((r[1] for r in res), default=math.inf):+.1f} mm")
