import math, sys
import numpy as np
from yam_arena import station
from yam_arena.demo import ArmPlan, Waypoint
TOP = station.TABLE_TOP
D = math.pi / 180
def worst(arm, wps):
    p = ArmPlan(wps, station.BASES[arm]); t = 0.0
    while t <= p.duration:
        p.action7(t); t += 0.05
    return p.worst_ik_mm, p.worst_ik_deg
for arm, pt in (("left", (0.240, -0.0068, TOP + 0.170)), ("right", (0.1795, 0.0, TOP + 0.12)), ("right", (0.1795, 0.03, TOP + 0.06))):
    row = []
    for yd in range(-180, 181, 20):
        w = [Waypoint(1.5, pt, -0.04, "s", yd * D), Waypoint(1.0, pt, -0.04, "p", yd * D)]
        mm, dg = worst(arm, w)
        row.append(f"{yd:+d}:{mm:.0f}/{dg:.0f}")
    print(arm, pt, " ".join(row))
