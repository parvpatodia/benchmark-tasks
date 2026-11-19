"""rig_scad.py: writes ../rig.scad, the printable real-trial rig, from the task's own numbers (no hand-copied sizes).

Parts: the trainer hand (one rigid print, rolled as in the model, with a keel web round the hinge pin at the ring
finger's axis height and a ballast bore in the forearm stub), the base (plate, two hinge cheeks outside the fingers,
wrist block), and the lancet stand (a fixed block holding the tab cap, and a cup keyed to the housing that turns in
it). Units mm; z from the table top; x along the fingers toward the
wrist from the ring's apex, y to the thumb side (the model's hand mechanism frame, with z shifted by its HZ).

WHY generated: the review asks for a rebuildable rig; generating it from lancet_fingerstick.py makes the print take its
sizes from the model (test_rig_scad_is_current checks the committed file against a fresh run). Not rendered here:
OpenSCAD is not installed, so nothing checks that the solids union, that the keel holds the pin bore all the way
across, or that the parts fit a printer's bed.

    python3 tools/rig_scad.py            # rewrites rig.scad
    python3 tools/rig_scad.py --check    # exits 1 if rig.scad is stale
"""
import math
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import lancet_fingerstick as F  # noqa: E402

OUT = HERE / "rig.scad"
PIN_D = 3.0          # mm: the hinge pin, a 3 mm steel rod [CHOICE]
PIN_CLEAR = 0.2      # mm: on the pin's diameter in the cheeks and the keel
KEEL_T = 14.0        # mm: the keel's thickness along x, centred on the hinge: walls of 5.3 mm round the pin's hole
#                      [CHOICE]. WHY not 8: that left 2.3 mm walls
PIN_BELOW = 6.0      # mm: the keel's web reaches this far under the pin [CHOICE]
BORE_D, BORE_L = 16.0, 60.0   # mm: the ballast bore in the stub, from its wrist end [CHOICE]
CUP_OD = 24.0        # mm: the turning cup's outside diameter [CHOICE]
CUP_SINK = 6.0       # mm: how far the cup sits down in its recess in the stand [CHOICE]
POST_D = 10.0        # mm: the fixed post the cup turns on; the tab cap's slot is in its top [CHOICE]
FIT = 0.4            # mm: on each turning diameter (the cup in its recess, the cup on the post)
SPLIT_X = 200.0      # mm: where the hand print is cut in two (across the forearm stub) [CHOICE]
ALIGN_Y = 14.0       # mm: the two alignment pins either side of the stub's axis. WHY 14: at 8 their holes cut the 16 mm
#                      ballast bore (x 204 to 206); at 14 they clear it by 4.4 mm and the stub's skin by 6.4 mm
HAND_LEN = round(1000 * (F.STUB_U[1] - F.FINGERS["middle"][1]))   # mm: middle fingertip to the stub's end


def mm(v):
    return round(1000.0 * float(v), 3)


def pt(p, dz=0.0):
    return f"[{mm(p[0])}, {mm(p[1])}, {mm(p[2] + dz)}]"


def numbers():
    """The rig's numbers (mm), each from the model. The tests read these."""
    pin_z = F.HZ + F.HINGE_Z
    return {
        "pin": (mm(F.PIVOT_X), mm(pin_z)),
        "cheeks": (mm(F.HINGE_Y[0] - F.CHEEK_GAP - F.CHEEK_T), mm(F.HINGE_Y[0] - F.CHEEK_GAP),
                   mm(F.HINGE_Y[1] + F.CHEEK_GAP), mm(F.HINGE_Y[1] + F.CHEEK_GAP + F.CHEEK_T)),
        "wrist_top": mm(F.HZ + F.REST_W_Z),
        "wrist_block": (mm(F.STUB_X), mm(F.STUB_YZ[0]), 50.0, 40.0),
        "plate": (mm(F.PIVOT_X - 0.030), mm(F.STUB_X + 0.040), 60.0, 5.0),
        "stub_lever": mm(F.STUB_X - F.PIVOT_X),
        "stand": (40.0, mm(F.STAND_H)),
        "cup_pocket": (mm(F.L_W + 2 * F.CUP_CLEAR), mm(F.L_T + 2 * F.CUP_CLEAR), mm(F.CUP_D)),
    }


def hand_lines():
    z = F.HZ
    out = ["module fingers(lance_wall) {"]
    for name, (v, u0, u1, rd, rp) in F.FINGERS.items():
        y, zz = F._axis_yz(v)
        x0, xd, x1 = u0 - F.APEX[0], u0 + F.DIP - F.APEX[0], u1 - F.APEX[0]
        a = f"[{mm(x0 + rd)}, {mm(y)}, {mm(zz + z)}]"
        d = f"[{mm(xd)}, {mm(y)}, {mm(zz + z)}]"
        b = f"[{mm(x1)}, {mm(y)}, {mm(zz + z)}]"
        sleeved = name in F.SITE_FINGERS
        rr = f"{mm(rd)} - lance_wall" if sleeved else f"{mm(rd)}"
        out.append(f"  // {name}: distal radius {mm(rd)}, proximal {mm(rp)}"
                   + (": the distal pad thinner by the Lance finger's wall, which brings it back" if sleeved else ""))
        out.append(f"  capsule({a}, {d}, {rr});")
        out.append(f"  capsule({d}, {b}, {mm(rp)});")
    out.append("}")
    palm = [F.hand_to_mech((u, v, w)) for w in (-F.PALM_W, F.PALM_W) for u, v in
            ((F.PALM_U[0], F.PALM_V[0]), (F.PALM_U[1], F.PALM_V[0]), (F.PALM_U[1], F.PALM_V[1]),
             (F.PALM_U[0], F.PALM_V[1]))]
    out.append("module palm() { hull() for (p = [" + ", ".join(pt(p, z) for p in palm) + "]) translate(p) "
               "cube(0.01, center = true); }")
    (tv, tu, tr), (bv, bu, br) = F.THUMB
    ta, tb = F.hand_to_mech((tu, tv, 0.0)), F.hand_to_mech((bu, bv, 0.0))
    out.append(f"module thumb() {{ hull() {{ translate({pt(ta, z)}) sphere({mm(tr)}); "
               f"translate({pt(tb, z)}) sphere({mm(br)}); }} }}")
    sy, sz = F.STUB_YZ
    x0s, x1s = F.STUB_U[0] - F.APEX[0], F.STUB_U[1] - F.APEX[0]
    out.append(f"module stub() {{ translate([{mm(x0s)}, {mm(sy)}, {mm(sz + z)}]) rotate([0, 90, 0]) "
               f"cylinder(r = {mm(F.STUB_R)}, h = {mm(x1s - x0s)}); }}")
    out.append("// the wrist: the model's palm and stub are one rigid link with a 4 mm gap between them; the print bridges it")
    out.append(f"module wrist() {{ hull() {{ intersection() {{ palm(); translate([{mm(F.PALM_U[1] - F.APEX[0]) - 6}, -100, -10]) "
               f"cube([6, 200, 200]); }} intersection() {{ stub(); translate([{mm(x0s)}, -100, -10]) cube([6, 200, 200]); }} }} }}")
    out.append(f"module stub_bore() {{ translate([{mm(x1s) - BORE_L}, {mm(sy)}, {mm(sz + z)}]) rotate([0, 90, 0]) "
               f"cylinder(d = {BORE_D}, h = {BORE_L + 1}); }}")
    return out


def lines():
    n = numbers()
    px, pz = n["pin"]
    c0, ci0, ci1, c1 = n["cheeks"]
    ky0, ky1 = mm(F.HINGE_Y[0]), mm(F.HINGE_Y[1])   # the keel: the fingers' span, 1 mm inside each cheek
    cl = mm(F.CHEEK_L)
    pin_len = round(c1 - c0 + 2 * 3.0, 3)
    wx, wy, wl, ww = n["wrist_block"]
    x0, x1, pw, pt_ = n["plate"]
    cw, ct, cd = n["cup_pocket"]
    sw, sh = n["stand"]
    r = ["// rig.scad: the real-trial rig for lancet-fingerstick, written by tools/rig_scad.py from lancet_fingerstick.py.",
         "// Do not edit by hand; rerun the tool. Units mm, z from the table top, x toward the wrist from the ring's apex.",
         "// Render each part on its own: part = \"hand_front\", \"hand_back\", \"base\", \"stand\" or \"cup\" (\"all\" views the set).",
         f"// The hand is {HAND_LEN} mm long, so it prints in two parts cut at x = {SPLIT_X:g} mm (across the stub),",
         "// joined with epoxy on the two 3 mm alignment pins. Print PETG at the infill that keeps it under the model's mass",
         f"// ({F.HAND_KG:.3f} kg) with the bore empty; lead shot in the bore then sets the tipping force (README check 4).",
         "// Print (CHOICE, not trialled): hand parts lying on their palm side with tree supports under the fingers and the",
         "// thumb; base on its plate, no supports (the 3.4 mm pin holes bridge); stand on its bottom; cup rim up, no supports.",
         f"// The hinge pin: a 3 mm steel rod {pin_len} mm long through both cheeks and the keel, held by a 3 mm push-on",
         "// retaining washer outside each cheek (CHOICE). It turns in the keel's 3.4 mm hole and in the cheeks'.",
         'part = "all";',
         "lance_wall = 1.0;   // the Lance trainer finger's wall, measured with calipers (README check 3b) [EST until measured]",
         "tab_w = 6.0; tab_t = 3.0; tab_h = 4.0;   // the lancet's tab cap, measured with calipers (README check 3) [EST]",
         "$fn = 48;",
         "module capsule(a, b, r) { hull() { translate(a) sphere(r); translate(b) sphere(r); } }",
         ]
    r += hand_lines()
    face = sh + mm(F.TAB_H)                     # the shield's face rests here: the model's stand top plus its tab layer
    rim = face + cd                               # the cup's rim: the model's socket wall top
    floor = sh - CUP_SINK
    r += [
        f"pin_x = {px}; pin_z = {pz};   // the hinge's axis: the model's (PIVOT_X, HINGE_Z), on the ring finger's axis",
        f"module pin_hole() {{ translate([pin_x, {round(c0 - 3.0, 3)}, pin_z]) rotate([-90, 0, 0]) "
        f"cylinder(d = {PIN_D + 2 * PIN_CLEAR}, h = {pin_len}); }}",
        f"module keel_box() {{ translate([pin_x - {KEEL_T / 2}, {ky0}, pin_z - {PIN_BELOW}]) "
        f"cube([{KEEL_T}, {round(ky1 - ky0, 3)}, 40]); }}",
        "// the keel: a web between the cheeks from under the pin up into the fingers, clipped to the fingers' hull so",
        "// nothing stands above them; the pin's hole runs through it",
        f"module keel() {{ intersection() {{ keel_box(); hull() {{ fingers(0); translate([pin_x - {KEEL_T / 2}, {ky0}, "
        f"pin_z - {PIN_BELOW}]) cube([{KEEL_T}, {round(ky1 - ky0, 3)}, 1]); }} }} }}",
        "module hand() { difference() { union() { fingers(lance_wall); palm(); wrist(); thumb(); stub(); keel(); }",
        "  pin_hole(); stub_bore(); } }",
        "module base() { difference() { union() {",
        f"  translate([{x0}, {-pw / 2}, 0]) cube([{round(x1 - x0, 3)}, {pw}, {pt_}]);   // the plate, ballasted to "
        f"{F.BASE_KG:g} kg and taped down",
        f"  for (y = [{c0}, {ci1}]) translate([pin_x - {cl / 2}, y, 0]) "
        f"cube([{cl}, {round(ci0 - c0, 3)}, pin_z + 4]);   // the hinge cheeks, outside the fingers",
        f"  translate([{round(wx - wl / 2, 3)}, {round(wy - ww / 2, 3)}, 0]) cube([{wl}, {ww}, {n['wrist_top']}]);"
        "   // the wrist block: the stub rests on it (the hinge's upper stop)",
        "  } pin_hole(); } }",
        f"module stand() {{ difference() {{ translate([{-sw / 2}, {-sw / 2}, 0]) cube([{sw}, {sw}, {sh}]);",
        f"  translate([0, 0, {floor}]) cylinder(d = {CUP_OD + FIT}, h = {CUP_SINK + 1}); }}   // the cup's recess",
        f"  difference() {{ cylinder(d = {POST_D}, h = {face});   // the fixed post, its top at the shield's face",
        f"    translate([-tab_w / 2, -tab_t / 2, {face} - tab_h]) cube([tab_w, tab_t, tab_h + 0.01]); }}"
        "   // the slot that holds the tab cap still",
        "}",
        f"module cup() {{ difference() {{ translate([0, 0, {floor}]) cylinder(d = {CUP_OD}, h = {round(rim - floor, 3)});",
        f"  translate([{-cw / 2}, {-ct / 2}, {face}]) cube([{cw}, {ct}, {cd + 1}]);   // keyed to the housing, "
        f"{F.CUP_CLEAR * 1000:g} mm each side",
        f"  translate([0, 0, {floor - 1}]) cylinder(d = {POST_D + FIT}, h = {round(face - floor + 1.01, 3)});"
        "   // turns on the post",
        "} }",
        f"module hand_cut(keep_front) {{ difference() {{ intersection() {{ hand(); translate([keep_front ? -50 : {SPLIT_X}, -100, -10])"
        f" cube([keep_front ? {SPLIT_X} + 50 : 200, 200, 200]); }}",
        f"  for (y = [{round(mm(F.STUB_YZ[0]) - ALIGN_Y, 3)}, {round(mm(F.STUB_YZ[0]) + ALIGN_Y, 3)}]) translate([{SPLIT_X} - 6, y, "
        f"{mm(F.HZ + F.STUB_YZ[1])}])"
        " rotate([0, 90, 0]) cylinder(d = 3.2, h = 12); } }   // the alignment pins' holes",
        'if (part == "hand_front" || part == "all") hand_cut(true);',
        'if (part == "hand_back" || part == "all") hand_cut(false);',
        'if (part == "base" || part == "all") color("grey") base();',
        'if (part == "stand" || part == "all") translate([0, -120, 0]) stand();',
        'if (part == "cup" || part == "all") translate([0, -120, 0]) color("white") cup();',
    ]
    return r


def text():
    return "\n".join(lines()) + "\n"


if __name__ == "__main__":
    if "--check" in sys.argv:
        sys.exit(0 if OUT.exists() and OUT.read_text() == text() else 1)
    OUT.write_text(text())
    print("wrote", OUT, len(text().splitlines()), "lines")
