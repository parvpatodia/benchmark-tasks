"""Hand and part hulls shared by the clearance tools (clear_nail.py): a shape's vertices, the hand's links at a joint
vector, and the plan label at a time."""
import math
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.analysis import fk_link_poses_world, world_hulls  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402

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
