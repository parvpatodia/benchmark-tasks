"""Offline hand clearance along the demo: the tool posed by phase (tube pressed even, yoke slide from the right hand, clamp and handle turned with the hand holding them); each hand's hulls
against every tool part and the tube, except the part that hand is holding. No sim."""
import math, sys, pathlib, warnings
warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np
import flare_tool as F
from geom import pieces, gap
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.constants import FINGER_OPEN
from yam_arena.analysis import fk_link_poses_world, world_hulls

SEED = next((int(a.split("=")[1]) for a in sys.argv[1:] if a.startswith("seed=")), None)
L, R, info = F.plan(0.0, SEED)
PL, PR = ArmPlan(L, station.BASES["left"]), ArmPlan(R, station.BASES["right"])


def seg(ws):
    out, t = [], 0.0
    for w in ws:
        out.append((t, t + w.duration, w.label, w)); t += w.duration
    return out


SL, SR = seg(L), seg(R)


def label_at(S, t):
    for a, b, l, w in S:
        if a <= t < b:
            return l
    return S[-1][2]


def t_of(S, label):
    return next(a for a, b, l, w in S if l == label)


def end_of(S, label):
    return next(b for a, b, l, w in S if l == label)


def tcp(P, t):
    pos, rot, finger, label = P.target(t)
    return np.asarray(pos, float), finger


T_PRESS0, T_PRESS1 = t_of(SR, "down onto the tube's end"), end_of(SR, "hold even")
T_SL0, T_SL1 = t_of(SR, "slide the yoke to the 3/8 line"), end_of(SR, "slide the yoke to the 3/8 line")
S_END = F.HY - F.PARK_Y
TOP_IN = F.Z_BAR + F.PROUD


def state(t):
    q, held = {"hinge": F.TH_SHUT}, {"left": set(), "right": set()}
    # tube: proud until pressed, then even
    if t < T_PRESS0:
        tube = F.TUBE_START
    else:
        p, _ = tcp(PR, t)
        top = max(F.Z_BAR, min(TOP_IN, p[2] - F.TIP))
        tube = (F.TUBE_X, F.HY, top - F.TUBE_L)
        if t < T_PRESS1:
            held["right"] |= {"tube"}
    # yoke slide
    if t < T_SL0:
        q["slide"] = F.park_off(SEED)
    elif t < T_SL1:
        p, _ = tcp(PR, t); q["slide"] = p[1] - F.PARK_Y
    else:
        q["slide"] = S_END
    lr = label_at(SR, t); ll = label_at(SL, t)
    if any(k in lr for k in ("wing", "slide the yoke", "hold on the line", "handle")):
        held["right"] |= {"handle"}
    if "T-bar" in lr and "rise clear" not in lr:
        held["right"] |= {"clamp"}
    if any(k in ll for k in ("wing", "turn", "close ", "open ", "up ", "down ", "handle")):
        held["left"] |= {"handle"}
    if "press the end" in lr or "hold even" in lr:
        held["right"] |= {"base", "front_half"}   # the pad tips meet the bar's top on purpose (stopped by it)
    return q, tube, held


def hand(P, arm, t):
    a, _ = P.action7(t)
    qd = {f"joint{i + 1}": float(a[i]) for i in range(6)}
    qd["joint7"] = qd["joint8"] = float(a[6]) * FINGER_OPEN
    return [h.vertices for _, h in world_hulls(fk_link_poses_world(qd, station.BASES[arm]), list(F.HAND_LINKS))]


worst = {}
_pos = [a for a in sys.argv[1:] if "=" not in a]
t, dt = 0.0, float(_pos[0]) if _pos else 0.25
end = PL.duration
while t <= end:
    q, tube, held = state(t)
    P = pieces(q, tube)
    H = {"left": hand(PL, "left", t), "right": hand(PR, "right", t)}
    for arm in ("left", "right"):
        for part, ps in P.items():
            if part in held[arm]:
                continue
            for A in H[arm]:
                for B in ps:
                    if np.any(A.min(0) - 0.004 > B.max(0)) or np.any(B.min(0) - 0.004 > A.max(0)):
                        continue
                    g = gap(A, B)
                    if g < 0.002:
                        lab = label_at(SL if arm == "left" else SR, t)
                        k = (arm, part, lab)
                        worst[k] = min(worst.get(k, 9), g)
    hh = 9.0
    for A in H["left"]:
        for B in H["right"]:
            if np.any(A.min(0) - 0.02 > B.max(0)) or np.any(B.min(0) - 0.02 > A.max(0)):
                continue
            hh = min(hh, gap(A, B))
    if hh < 0.02:
        k = ("hands", "hands", label_at(SL, t) + " / " + label_at(SR, t))
        worst[k] = min(worst.get(k, 9), hh)
    t += dt
for (arm, part, lab), g in sorted(worst.items(), key=lambda kv: kv[1]):
    print(f"{arm:6s} {part:10s} {1000 * g:+7.2f} mm  {lab}")
print("done", len(worst))
