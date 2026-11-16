"""The let-go radius CLEAR_R against geometry: the clipper's farthest point from its grip point (its frame) and the
fingers' farthest point from the TCP, at open, holding and closed. CLEAR_R must exceed their sum."""
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np  # noqa: E402
import nail_trim as M  # noqa: E402
from hulls import verts  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.analysis import fk_link_poses_world, world_hulls  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402


def clipper_extent():
    base, links, _ = M.clipper_parts()
    shapes = list(base.shapes) + [s for link in links for s in link.shapes]
    return float(np.linalg.norm(np.vstack([verts(s) for s in shapes]), axis=1).max())


def finger_reach(links=("tip_left", "tip_right")):
    best = 0.0
    for f in (FINGER_OPEN, M.HOLD_F, M.SQUEEZE_F, 0.0):
        qd = {f"joint{i + 1}": v for i, v in enumerate((0.0, 1.0, 1.0, -0.6, 0.0, 0.0))}
        qd["joint7"] = qd["joint8"] = f
        P = fk_link_poses_world(qd, station.BASES["right"])
        T = np.asarray(P["gripper"])
        tcp = T[:3, 3] + T[:3, :3] @ np.array(M.TCP_OFFSET)
        for _, h in world_hulls(P, links):
            best = max(best, float(np.linalg.norm(np.asarray(h.vertices) - tcp, axis=1).max()))
    return best


if __name__ == "__main__":
    c, f = clipper_extent(), finger_reach()
    print(f"clipper {1000 * c:.1f} mm + fingers {1000 * f:.1f} mm = {1000 * (c + f):.1f} mm; CLEAR_R {1000 * M.CLEAR_R:.0f} mm")
    print("OK" if M.CLEAR_R > c + f + 0.01 else "TOO SMALL")
    w = finger_reach(("gripper",))
    print(f"the gripper housing reaches {1000 * w:.1f} mm from the TCP: it can touch the clipper (press), not hold it")
