"""Dimensioned views of the coin lock, drawn from the parameters in coinlock.scad (the same numbers the test checks
against trolley.py). MIT licence.

    python3 drawing.py            # writes drawing.svg and drawing.png next to this file
"""
import math
import pathlib
import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent


def params(path=HERE / "coinlock.scad"):
    """Every top-level `NAME = expr;` of the .scad, evaluated in order (numbers, arithmetic, sqrt, pow, vectors)."""
    env = {"sqrt": math.sqrt, "pow": math.pow}
    for line in path.read_text().splitlines():
        m = re.match(r"^([A-Z][A-Z0-9_]*)\s*=\s*([^;]+);", line)
        if m:
            env[m.group(1)] = eval(m.group(2), {"__builtins__": {}}, env)  # noqa: S307  WHY: our own file, arithmetic only
    return env


P = params()
GREY, ORANGE, STEEL, RED = "#b8b8b8", "#f0a040", "#8fa3b8", "#d04030"


def box(ax, x0, y0, x1, y1, fc, ec="k", lw=0.6, ls="-", alpha=1.0, z=1):
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=fc, ec=ec, lw=lw, ls=ls, alpha=alpha, zorder=z))


def dim(ax, a, b, off, text, vertical=False):
    """A dimension line between points a and b (scalars along the axis), offset `off` across it."""
    if vertical:
        (y0, y1), x = (a, b), off
        ax.annotate("", (x, y0), (x, y1), arrowprops=dict(arrowstyle="<->", lw=0.6), zorder=5)
        ax.text(x + 0.8, (y0 + y1) / 2, text, fontsize=6, va="center", zorder=6)
    else:
        (x0, x1), y = (a, b), off
        ax.annotate("", (x0, y), (x1, y), arrowprops=dict(arrowstyle="<->", lw=0.6), zorder=5)
        ax.text((x0 + x1) / 2, y + 0.8, text, fontsize=6, ha="center", zorder=6)


def key_layer(ax):
    """Section at the key slot's mid height (z SLOT_Z), seen from above; plunger at rest, key homed."""
    p = P
    box(ax, -p["HX"], -p["HY"], p["HX"], p["HY"], GREY)
    box(ax, p["BEHIND_X1"], p["SY0"], p["HX"], p["SY1"], "w")                              # slot and plunger way
    box(ax, p["T_X0"] - p["PLUNGER_STOP"] - p["CLR"], p["ARM_Y0"] - p["CLR"], p["ARM_X1"] + p["CLR"],
        p["T_Y1"] + p["CLR"], "w")                                                         # arm pocket
    box(ax, p["PSPR_X0"], p["SLOT_Y"] - p["RECESS_W"] / 2, p["BEHIND_X1"], p["SLOT_Y"] + p["RECESS_W"] / 2, "w")
    box(ax, p["PL_X0"], p["SY0"] + 0.5, p["PL_X1"] - p["RELEASE_TRAVEL"], p["SY1"] - 0.5, ORANGE)   # plunger (homed)
    box(ax, p["T_X0"] - p["RELEASE_TRAVEL"], p["ARM_Y0"], p["ARM_X1"] - p["RELEASE_TRAVEL"], p["T_Y1"], ORANGE)
    box(ax, p["T_X0"] - p["RELEASE_TRAVEL"], p["T_Y0"], p["T_X1"] - p["RELEASE_TRAVEL"], p["T_Y1"], ORANGE, ls="--")
    tip = p["FX"] - p["HOME_DEPTH"]
    box(ax, tip, p["SLOT_Y"] - p["KEY_W"] / 2, p["FX"] + 12, p["SLOT_Y"] + p["KEY_W"] / 2, STEEL, alpha=0.8)
    box(ax, tip + p["APER_D0"], p["SLOT_Y"] - p["APER_W"] / 2, tip + p["APER_D1"], p["SLOT_Y"] + p["APER_W"] / 2, "w")
    box(ax, p["PIN_X"] - p["PIN_HX"], p["SLOT_Y"] - p["PIN_HY"], p["PIN_X"] + p["PIN_HX"], p["SLOT_Y"] + p["PIN_HY"],
        ORANGE, z=3)
    ax.plot([p["PSPR_X0"], p["PL_X0"] - p["RELEASE_TRAVEL"]], [p["SLOT_Y"]] * 2, "k:", lw=1.2)
    dim(ax, p["SY0"], p["SY1"], p["HX"] + 14, f"slot {p['SLOT_W']:g}", vertical=True)
    dim(ax, p["SLOT_Y"], p["HY"], p["HX"] + 4, f"{p['HY'] - p['SLOT_Y']:g}", vertical=True)
    dim(ax, tip, p["FX"], p["SY0"] - 4, f"home {p['HOME_DEPTH']:g}")
    dim(ax, p["PL_X1"], p["FX"], p["SY1"] + 2, f"{p['PLUNGER_IN']:g}")
    dim(ax, p["PIN_X"] - p["PIN_HX"], p["PIN_X"] + p["PIN_HX"], p["SY0"] - 9, f"pin {2 * p['PIN_HX']:g}")
    dim(ax, -p["HX"], p["HX"], -p["HY"] - 5, f"{p['LOCK_W']:g} (+x: front face)")
    ax.text(-30, -3, "T (dashed) rises\nthrough the cover", fontsize=6)
    ax.text(-31, p["SLOT_Y"] - 1, "spring VD-006C", fontsize=6)
    ax.set_title(f"A. key layer, section at z {p['SLOT_Z']:g} (key homed, plunger {p['RELEASE_TRAVEL']:g} in)", fontsize=8)


def side_section(ax):
    """Section through the slot's centre line (y SLOT_Y), seen from -y: layers, pin, its bores, homed key."""
    p = P
    bands = [("floor", p["LOCK_Z0"], p["Z_KEY0"]), ("key plate", p["Z_KEY0"], p["Z_KEY1"]),
             ("cover", p["Z_KEY1"], p["Z_COV1"]), ("shell", p["Z_COV1"], p["Z_SLD1"]), ("cap", p["Z_SLD1"], p["LOCK_Z1"])]
    for name, z0, z1 in bands:
        box(ax, -p["HX"], z0, p["HX"], z1, RED if name == "cap" else GREY)
        ax.text(-p["HX"] - 2, (z0 + z1) / 2, f"{name} {z0:g}", fontsize=6, ha="right", va="center")
    box(ax, p["BEHIND_X1"], p["Z_KEY0"], p["HX"], p["Z_KEY1"], "w")
    box(ax, p["PIN_X"] - p["PIN_HX"] - 0.1, p["Z_KEY1"], p["PIN_X"] + p["PIN_HX"] + 0.1, p["PIN_TOP"], "w")
    box(ax, p["PIN_X"] - p["PIN_HX"] - 0.2, p["PIN_TOP"], p["PIN_X"] + p["PIN_HX"] + 0.2,
        p["PIN_TOP"] + p["PIN_HEAD_H"] + p["PIN_RISE"], "w")
    box(ax, p["PIN_X"] - p["PIN_BORE_D"] / 2, p["PIN_TOP"] + p["PIN_HEAD_H"] + p["PIN_RISE"],
        p["PIN_X"] + p["PIN_BORE_D"] / 2, p["PIN_SPR_Z1"], "w")
    box(ax, p["REL_X"] - p["REL_D"] / 2, p["LOCK_Z0"], p["REL_X"] + p["REL_D"] / 2, p["Z_KEY0"], "w")
    tip = p["FX"] - p["HOME_DEPTH"]
    box(ax, tip, p["Z_KEY0"], p["FX"] + 12, p["Z_KEY0"] + p["KEY_T"], STEEL, alpha=0.8)
    box(ax, tip + p["APER_D0"], p["Z_KEY0"], tip + p["APER_D1"], p["Z_KEY0"] + p["KEY_T"], "w")
    x0, x1 = p["PIN_X"] - p["PIN_HX"], p["PIN_X"] + p["PIN_HX"]
    ax.add_patch(Polygon([(x0, p["PIN_Z0"]), (x1, p["PIN_Z0"] + p["PIN_RAMP"]), (x1, p["PIN_TOP"]), (x0, p["PIN_TOP"])],
                         fc=ORANGE, ec="k", lw=0.6, zorder=3))
    box(ax, x0, p["PIN_TOP"], x1, p["PIN_TOP"] + p["PIN_HEAD_H"], ORANGE, z=3)
    box(ax, p["PL_X0"], p["Z_KEY0"] + 0.5, p["PL_X1"] - p["RELEASE_TRAVEL"], p["Z_KEY1"] - 0.5, ORANGE)
    ax.plot([p["PIN_X"]] * 2, [p["PIN_TOP"] + p["PIN_HEAD_H"], p["PIN_SPR_Z1"]], "k:", lw=1.2)
    dim(ax, p["Z_KEY0"], p["Z_KEY1"], p["HX"] + 3, f"slot {p['SLOT_T']:g}", vertical=True)
    dim(ax, p["Z_KEY0"], p["PIN_Z0"], p["PIN_X"] - p["PIN_HX"] - 5, f"{p['PIN_Z0'] - p['Z_KEY0']:g}", vertical=True)
    dim(ax, p["LOCK_Z0"], p["LOCK_Z1"], p["HX"] + 12, f"{p['LOCK_H']:g}", vertical=True)
    ax.text(p["PIN_X"] + 4, p["PIN_SPR_Z1"] - 6, f"VD-006, bore {p['PIN_BORE_D']:g}\nto z {p['PIN_SPR_Z1']:g}", fontsize=6)
    ax.text(p["REL_X"] + 2, p["LOCK_Z0"] + 2, f"release hole {p['REL_D']:g}", fontsize=6)
    ax.set_xlim(-p["HX"] - 22, p["HX"] + 25)
    ax.set_ylim(p["LOCK_Z0"] - 4, p["LOCK_Z1"] + 4)
    ax.set_title(f"B. section at y {p['SLOT_Y']:g} (z up from the tube axis): pin at rest in the key's hole", fontsize=8)


def slide_view(ax):
    """The coin slide from above, in (solid) and at the out stop (outline), with its spring, rod and stop block."""
    p = P
    box(ax, -p["HX"], -p["HY"], p["HX"], p["HY"], GREY, alpha=0.4)
    box(ax, -p["SLD_X"], -p["HY"], p["SLD_X"], p["SLD_Y1"], "w")
    x0, x1 = -p["SLD_X"] + 0.5, p["SLD_X"] - 0.5
    for dy, style in ((0, dict(fc=ORANGE)), (-p["PUSH_TRAVEL"], dict(fc="none", ls="--"))):
        box(ax, x0, -p["HY"] + dy, x1, p["L_Y1"] + dy, **style)
        box(ax, p["HOLE_X0"], p["HOLE_Y0"] + dy, p["HOLE_X1"], p["HOLE_Y1"] + dy, "w" if dy == 0 else "none", ls=":")
        box(ax, p["L_X0"], p["L_Y0"] + dy, p["L_X1"], p["L_Y1"] + dy, "#c07020" if dy == 0 else "none")
        box(ax, p["POST_X0"], p["POST_Y0"] + dy, p["POST_X1"], p["POST_Y1"] + dy, "#c07020" if dy == 0 else "none")
        box(ax, p["W_LO"] - p["WALL_T"], p["CY0"] + dy, p["W_HI"] + p["WALL_T"], p["CY1"] + dy,
            "#c07020" if dy == 0 else "none")
    box(ax, p["T_X0"], p["T_Y0"], p["T_X1"], p["T_Y1"], ORANGE, ls="-", z=3)
    box(ax, -p["SLD_X"], p["STOP_Y0"], p["POST_X1"] + 0.5, p["STOP_Y1"], RED, alpha=0.6)
    ax.plot([p["SPR_X"]] * 2, [p["ROD_Y0"], p["HY"]], "k-", lw=0.8)
    ax.plot([p["SPR_X"]] * 2, [p["POST_Y1"], p["SLD_Y1"]], "k:", lw=2)
    dim(ax, p["L_Y0"], p["L_Y0"] - p["PUSH_TRAVEL"], p["L_X1"] + 3, f"travel {p['PUSH_TRAVEL']:g}", vertical=True)
    dim(ax, p["T_X0"], p["T_X1"], p["T_Y0"] - 3, "T")
    dim(ax, p["T_X1"] - p["RELEASE_TRAVEL"], p["T_X1"], p["T_Y1"] + 1, f"{p['RELEASE_TRAVEL']:g}")
    ax.text(p["SPR_X"] - 1, p["SLD_Y1"] + 2, "VD-042J-05 on M3 rod", fontsize=6, ha="center")
    ax.text(-p["SLD_X"] + 0.5, p["STOP_Y0"] - 4, "out stop (cap)", fontsize=6)
    ax.text(p["COIN_X"] - 8, -p["HY"] - p["PUSH_TRAVEL"] + 2, "coin pocket out", fontsize=6)
    ax.set_title("C. coin slide from above: in (filled), at its out stop (dashed); T and lug L", fontsize=8)
    ax.set_ylim(-p["HY"] - p["PUSH_TRAVEL"] - 4, p["HY"] + 6)


def key_view(ax):
    p = P
    hw, aw = p["KEY_W"] / 2, p["APER_W"] / 2
    ax.add_patch(Polygon([(-0.6 * hw, 0), (0.6 * hw, 0), (hw, 4), (hw, p["KEY_L"]), (4, p["KEY_L"]),
                          (4, p["KEY_L"] + p["KEY_EYE"]), (-4, p["KEY_L"] + p["KEY_EYE"]), (-4, p["KEY_L"]),
                          (-hw, p["KEY_L"]), (-hw, 4)], fc=STEEL, ec="k", lw=0.6))
    box(ax, -aw, p["APER_D0"], aw, p["APER_D1"], "w")
    ey = p["KEY_L"] + p["KEY_EYE"] - p["EYE_HOLE_D"] / 2 - 1.5
    ax.add_patch(plt.Circle((0, ey), p["EYE_HOLE_D"] / 2, fc="w", ec="k", lw=0.6))
    dim(ax, 0, p["KEY_L"], hw + 4, f"{p['KEY_L']:g}", vertical=True)
    dim(ax, 0, p["APER_D0"], -hw - 6, f"{p['APER_D0']:g}", vertical=True)
    dim(ax, p["APER_D0"], p["APER_D1"], -hw - 12, f"{p['APER_D1'] - p['APER_D0']:g}", vertical=True)
    dim(ax, -hw, hw, p["KEY_L"] / 2, f"{p['KEY_W']:g}")
    dim(ax, -aw, aw, p["APER_D1"] + 2, f"hole {p['APER_W']:g}")
    dim(ax, -0.6 * hw, 0.6 * hw, -4, f"tip {1.2 * hw:g}")
    ax.text(hw + 2, p["KEY_L"] + 4, f"eye {p['KEY_EYE']:g} long, 8 wide,\nhole {p['EYE_HOLE_D']:g}", fontsize=6)
    ax.text(-hw - 15, p["KEY_L"] - 8, f"{p['KEY_T']:g} mm stainless sheet;\nfile tip faces to {p['KEY_T'] * 0.6:g}\nover 4 mm",
            fontsize=6)
    ax.set_xlim(-hw - 20, hw + 22)
    ax.set_ylim(-8, p["KEY_L"] + p["KEY_EYE"] + 4)
    ax.set_title("D. key (key_dxf), tip down", fontsize=8)


def main():
    fig, axs = plt.subplots(2, 2, figsize=(11, 11))
    for f, ax in zip((key_layer, side_section, slide_view, key_view), axs.flat):
        f(ax)
        ax.set_aspect("equal")
        ax.autoscale_view()
        ax.tick_params(labelsize=6)
    axs[0, 0].set_xlim(-P["HX"] - 3, P["HX"] + 22)
    axs[0, 0].set_ylim(-P["HY"] - 9, P["HY"] + 3)
    axs[1, 0].set_xlim(-P["HX"] - 3, P["HX"] + 3)
    fig.suptitle("Trolley coin lock, all dimensions mm, lock frame (x to the front face, y along the handle, z up from "
                 "the tube axis); from coinlock.scad", fontsize=9)
    fig.tight_layout()
    for ext in ("svg", "png"):
        fig.savefig(HERE / f"drawing.{ext}", dpi=130)


if __name__ == "__main__":
    main()
