"""Static IK and whole-arm clearance at candidate solder layouts (no sim)."""
import math, sys, pathlib, warnings
warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np
from geom import gap
from yam_arena import station
from yam_arena.demo import ArmPlan, Waypoint
from yam_arena.constants import FINGER_OPEN, LINKS
from yam_arena.analysis import fk_link_poses_world, world_hulls
TOP = station.TABLE_TOP; D = math.pi / 180
ARM = [l for l in LINKS if l != "base"]

def settle(arm, pt, yaw):
    p = ArmPlan([Waypoint(1.5, pt, -0.01, "a", yaw), Waypoint(1.0, pt, -0.01, "b", yaw)], station.BASES[arm])
    t = 0.0; a = None
    while t <= p.duration:
        a, _ = p.action7(t); t += 0.05
    return p.worst_ik_mm, p.worst_ik_deg, a

def hulls(a, arm):
    qd = {f"joint{i+1}": float(a[i]) for i in range(6)}; qd["joint7"] = qd["joint8"] = 0.0
    return [(n, h.vertices) for n, h in world_hulls(fk_link_poses_world(qd, station.BASES[arm]), ARM)]

def armgap(al, ar):
    best = (9, "", "")
    for nl, A in hulls(al, "left"):
        for nr, B in hulls(ar, "right"):
            g = gap(A, B)
            if g < best[0]: best = (g, nl, nr)
    return best

mouth = float(sys.argv[1]) if len(sys.argv) > 1 else 0.065
jx = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
phi = float(sys.argv[3]) if len(sys.argv) > 3 else -49.5   # flame direction (deg)
L = 0.175
u = (math.cos(phi*D), math.sin(phi*D))
Pz = TOP + mouth - 0.0127
G = (jx - 0.0091*u[0] - L*u[0], 0.0091*u[1]*-1 - L*u[1], Pz)
wire = (jx + 0.0096*u[0], 0.0096*u[1])
lw = (G[0], G[1], Pz + 0.007); rw = (wire[0], wire[1], TOP + mouth + 0.093)
ml, dl, al = settle("left", lw, phi*D)
mr, dr, ar = settle("right", rw, math.pi/2)
g = armgap(al, ar)
print(f"mouth TOP+{mouth} jx {jx} phi {phi}: left G {tuple(round(v,3) for v in lw[:2])} z TOP+{lw[2]-TOP:.3f} ik {ml:.1f}mm {dl:.1f}d | right {tuple(round(v,3) for v in rw[:2])} z TOP+{rw[2]-TOP:.3f} ik {mr:.1f}mm {dr:.1f}d | arm gap {1000*g[0]:.0f} mm {g[1]} {g[2]}")
for nm, arm, pt, yw in (("wire pick", "right", (0.30, -0.12, TOP + 0.093), math.pi/2), ("wire carry", "right", (wire[0], wire[1]-0.03, TOP+mouth+0.12), math.pi/2),
                       ("torch rest", "left", (G[0]-0.04, G[1]+0.05, Pz+0.007), phi*D), ("torch away", "left", (G[0]-0.06, G[1]+0.07, Pz+0.05), phi*D)):
    m, d, _ = settle(arm, pt, yw); print(f"  {nm:10s} {arm:5s} {tuple(round(v,3) for v in pt[:2])} z TOP+{pt[2]-TOP:.3f}: ik {m:.1f} mm {d:.1f} deg")
