"""Write the slide's inner member as USDA text (no pxr needed).

python make_meshes.py [out_dir]

The inner member is one convex hull: a bar 250 mm long, 30 mm wide and 8 mm thick, its leading end (the nose, at +x)
tapered over its last 30 mm to 25 mm wide and 3 mm thick, as a real inner member's nose is ramped so it can ride in
between the ball rows. The frame's origin is the bar's bounding-box centre; x runs along it, y across its width, z
through its thickness.
"""
import sys
from pathlib import Path

LENGTH = 0.250   # 3832-C10 slide length, 250 mm [READ: Accuride 3832CDO data sheet]
WIDTH = 0.030    # [EST] the inner member's height on the real slide (the slide is 45.7 mm high)
THICK = 0.008    # [EST]
NOSE_L = 0.030   # the taper's length
NOSE_DW = 0.0025  # the taper takes 2.5 mm off each side and each face: 4.8 deg from the bar's axis
NAME = "server_rail_inner"
COLOR = "(0.10, 0.30, 0.85)"


def _fmt(p):
    return "(" + ", ".join(f"{v:.6f}" for v in p) + ")"


def ring(x, w, t):
    return [(x, -w / 2, -t / 2), (x, w / 2, -t / 2), (x, w / 2, t / 2), (x, -w / 2, t / 2)]


def rail_points():
    """Three rings of four points: the square back end, where the taper starts, and the nose."""
    h = LENGTH / 2
    return (ring(-h, WIDTH, THICK) + ring(h - NOSE_L, WIDTH, THICK)
            + ring(h, WIDTH - 2 * NOSE_DW, THICK - 2 * NOSE_DW))


def faces():
    out = [[3, 2, 1, 0], [8, 9, 10, 11]]  # back end (facing -x), nose (facing +x)
    for a in (0, 4):
        out += [[a + i, a + (i + 1) % 4, a + 4 + (i + 1) % 4, a + 4 + i] for i in range(4)]
    return out


def usda():
    pts = rail_points()
    f = faces()
    lo = [min(p[k] for p in pts) for k in range(3)]
    hi = [max(p[k] for p in pts) for k in range(3)]
    return f'''#usda 1.0
(
    defaultPrim = "{NAME}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{NAME}" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
)
{{
    bool physics:kinematicEnabled = 0

    def Mesh "member" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
    )
    {{
        uniform token physics:approximation = "convexHull"
        int[] faceVertexCounts = [{", ".join(str(len(x)) for x in f)}]
        int[] faceVertexIndices = [{", ".join(str(i) for x in f for i in x)}]
        point3f[] points = [{", ".join(_fmt(p) for p in pts)}]
        float3[] extent = [{_fmt(lo)}, {_fmt(hi)}]
        color3f[] primvars:displayColor = [{COLOR}]
        uniform token subdivisionScheme = "none"
    }}
}}
'''


def main(out_dir="."):
    path = Path(out_dir) / f"{NAME}.usda"
    path.write_text(usda())
    return path


if __name__ == "__main__":
    print(main(*sys.argv[1:]))
