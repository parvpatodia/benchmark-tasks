"""Flare tool reach probe: IK error for the feed-wing swing, clamp T-bar press and tube grip at candidate layouts."""
import math, sys
import numpy as np
from yam_arena import station
from yam_arena.demo import ArmPlan, Waypoint
from yam_arena.constants import FINGER_OPEN
TOP = station.TABLE_TOP
D = math.pi / 180

def worst(arm, wps):
    p = ArmPlan(wps, station.BASES[arm])
    t = 0.0
    while t <= p.duration:
        p.action7(t); t += 0.05
    return p.worst_ik_mm, p.worst_ik_deg

def run(x0, y0, hb, arm, wing_r=0.045, wing_h=0.090):
    zf = TOP + hb + wing_h
    sw = []
    for th in np.linspace(-70, 70, 15):    # wing tip around the feed axis (heading measured from -x)
        a = math.pi + th * D
        sw.append(Waypoint(0.3, (x0 + wing_r * math.cos(a), y0 + wing_r * math.sin(a), zf), -0.004, f"w{th:.0f}", a + math.pi / 2))
    clamp = [Waypoint(0.5, (x0 - 0.07, y0 + s * 0.035, TOP + hb + z), FINGER_OPEN * 0.3, "tbar", 0.0)
             for s in (1, -1) for z in (0.06, 0.035)]
    tube = [Waypoint(0.5, (x0, y0, TOP + hb + z), FINGER_OPEN * 0.5, "tube", math.pi / 2) for z in (0.05, 0.015)]
    out = []
    for nm, w in (("wing", sw), ("clamp", clamp), ("tube", tube)):
        start = [Waypoint(1.0, (x0 - 0.05, y0, TOP + 0.20), FINGER_OPEN, "start", w[0].yaw)]
        mm, dg = worst(arm, start + w)
        out.append(f"{nm} {mm:5.1f}mm {dg:4.1f}d")
    print(f"{arm:5s} x0 {x0:.2f} y0 {y0:+.2f} bar top TOP+{hb:.2f} wing TOP+{hb+wing_h:.2f} | " + " | ".join(out))

for hb in (0.06, 0.08):
    for x0 in (0.20, 0.24, 0.28):
        for y0, arm in ((-0.06, "right"), (0.0, "right"), (0.06, "left"), (0.0, "left")):
            run(x0, y0, hb, arm)
