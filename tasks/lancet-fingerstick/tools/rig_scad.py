"""rig_scad.py: writes ../rig.scad, the printable real-trial rig, from the task's own numbers (no hand-copied sizes).

Parts: the base (plate, the palm block with the index finger and the thumb on a pedestal, and the ring finger's guide
block with its two pin bushings and spring seats), the ring finger (its two segments and the carriage under its
knuckle, with the pins' holes), and the lancet stand (a fixed block and post holding the tab cap, and a cup keyed to
the housing that turns on the post, stopped at +-100 deg). Bought parts: two 4 mm steel pins, two 4 mm shaft collars,
two compression springs (sizes below). Units mm; z from the table top; x along the fingers toward the wrist from the
ring's apex, y to the thumb side (the model's hand mechanism frame, with z shifted by its HZ).

WHY generated: the rig must be rebuildable from the model; generating it from lancet_fingerstick.py makes the print take
its sizes from the model (test_rig_scad_is_current checks the committed file against a fresh run). Not rendered here:
OpenSCAD is not installed, so nothing checks that the solids union or slice.

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
PIN_D = 4.0          # mm: the slide's pins, 4 mm steel dowels [CHOICE]
PIN_PRESS = 8.0      # mm: how deep each pin is pressed into the carriage (a 3.9 mm hole) [CHOICE]
BUSH_CLEAR = 0.2     # mm: on the pin's diameter in the guide block's bushings [CHOICE]
BRIDGE_T = 8.0       # mm: the guide block's top, the pins' bushing length [CHOICE]
COLLAR_H, COLLAR_OD = 5.0, 10.0   # mm: a 4 mm bore shaft collar, its width and diameter [EST: a common size]
SEAT_D, SEAT_H = 3.5, 8.0         # mm: the spring seat posts on the plate under each pin [CHOICE]
GUIDE_W = 24.0       # mm: the guide block across y [CHOICE]
CUP_OD = 24.0        # mm: the turning cup's outside diameter [CHOICE]
CUP_SINK = 6.0       # mm: how far the cup sits down in its recess in the stand [CHOICE]
POST_D = 10.0        # mm: the fixed post the cup turns on; the tab cap's slot is in its top [CHOICE]
FIT = 0.4            # mm: on each turning diameter (the cup in its recess, the cup on the post)
STOP_PIN_D = 3.0     # mm: the stop pin in the recess's floor that the cup's arc slot meets at +-100 deg [CHOICE]
STOP_R = 9.0         # mm: the stop pin's radius from the post's axis [CHOICE]


def mm(v):
    return round(1000.0 * float(v), 3)


def pt(p, dz=0.0):
    return f"[{mm(p[0])}, {mm(p[1])}, {mm(p[2] + dz)}]"


def numbers():
    """The rig's numbers (mm), each from the model. The tests and the README read these."""
    hz = F.HZ
    car_bot, car_top = mm(hz + F.CAR_Z[0]), mm(hz + F.CAR_Z[1])
    guide_top = mm(hz + F.GUIDE_TOP)
    under = round(guide_top - BRIDGE_T, 3)                      # the bridge's underside: the collars stop here
    rest = round(under - COLLAR_H - 5.0, 3)                     # each spring's length at rest (plate top at 5 mm)
    comp = round(1000 * F.ring_target(), 3)                     # its compression there (preload plus weight)
    k_each = F.K_RING / 2 / 1000                                # N/mm, two in parallel
    pin_top = round(car_bot + PIN_PRESS, 3)
    pin_bot = round(rest + 5.0 - 10.0 + 0.0, 3)                 # 10 mm under the collar's bottom
    return {
        "carriage": (mm(F.CAR_X[0]), mm(F.CAR_X[1]), mm(2 * F.CAR_HALF_Y), car_bot, car_top),
        "pins_x": (mm(F.PIN_X[0]), mm(F.PIN_X[1])),
        "guide": (mm(F.CAR_X[0]), mm(F.CAR_X[1]), GUIDE_W, guide_top, under),
        "drop_stop": round(car_bot - guide_top, 3),
        "spring": {"rate_each": round(k_each, 3), "rest": rest, "comp": comp, "free": round(rest + comp, 3),
                   "bottom": round(rest - 1000 * F.DROP_STOP, 3),
                   "force_rest_each": round(k_each * comp, 3)},
        "pin": {"top": pin_top, "bottom": pin_bot, "length": round(pin_top - pin_bot, 3)},
        "seat_top": 5.0 + SEAT_H,
        "plate": (mm(F.PLATE_X[0]), mm(F.PLATE_X[1]), mm(F.PLATE_Y[0]), mm(F.PLATE_Y[1]), 5.0),
        "ring_axis_z": mm(hz),
        "stand": (mm(2 * F.STAND_HALF), mm(F.STAND_H)),
        "cup_pocket": (mm(F.L_W + 2 * F.CUP_CLEAR), mm(F.L_T + 2 * F.CUP_CLEAR), mm(F.CUP_D)),
        "tab_stop_deg": round(math.degrees(F.TAB_STOP), 3),
        "base_size": base_size(),
    }


def base_size():
    """The base part's bounding box (mm, x by y by z): the plate and the fixed parts (index, palm, thumb), rounded up to
    the mm. WHY: the bed it needs (OpenSCAD is not installed, so this is from the model's shapes, not a render)."""
    lo = [min([F.PLATE_X[0]] + [s.bounds()[0][0] for s in F.HAND_SHAPES]),
          min([F.PLATE_Y[0]] + [s.bounds()[0][1] for s in F.HAND_SHAPES]), -F.HZ]
    hi = [max([F.PLATE_X[1]] + [s.bounds()[1][0] for s in F.HAND_SHAPES]),
          max([F.PLATE_Y[1]] + [s.bounds()[1][1] for s in F.HAND_SHAPES]),
          max(s.bounds()[1][2] for s in F.HAND_SHAPES)]
    return tuple(int(math.ceil(1000 * (b - a) - 1e-6)) for a, b in zip(lo, hi))


def finger_lines(name, wall):
    v, u0, u1, rd, rp = F.FINGERS[name]
    y, zz = F._axis_yz(v)
    z = F.HZ
    x0, xd, x1 = u0 - F.APEX[0], u0 + F.DIP - F.APEX[0], u1 - F.APEX[0]
    a = f"[{mm(x0 + rd)}, {mm(y)}, {mm(zz + z)}]"
    d = f"[{mm(xd)}, {mm(y)}, {mm(zz + z)}]"
    b = f"[{mm(x1)}, {mm(y)}, {mm(zz + z)}]"
    rr = f"{mm(rd)} - lance_wall" if wall else f"{mm(rd)}"
    return [f"  // {name}: distal radius {mm(rd)}, proximal {mm(rp)}"
            + (": the distal pad thinner by the Lance finger's wall, which brings it back" if wall else ""),
            f"  capsule({a}, {d}, {rr});", f"  capsule({d}, {b}, {mm(rp)});"]


def hand_lines():
    z = F.HZ
    out = ["module index() {"] + finger_lines("index", False)
    v, _, u1, _, rp = F.FINGERS["index"]
    ra, rb = F.hand_to_mech((u1 - 0.004, v, 0.0)), F.hand_to_mech((F.PALM_U[0] + 0.006, v, 0.0))
    out.append(f"  capsule({pt(ra, z)}, {pt(rb, z)}, {mm(rp)});   // the rod from its knuckle into the palm block")
    out.append("}")
    palm = [F.hand_to_mech((u, v, w)) for w in (-F.PALM_W, F.PALM_W) for u, v in
            ((F.PALM_U[0], F.PALM_V[0]), (F.PALM_U[1], F.PALM_V[0]), (F.PALM_U[1], F.PALM_V[1]),
             (F.PALM_U[0], F.PALM_V[1]))]
    out.append("module palm() { hull() for (p = [" + ", ".join(pt(p, z) for p in palm) + "]) translate(p) "
               "cube(0.01, center = true); }")
    out.append("// the pedestal: the palm block's footprint carried down to the plate (not in the sim, which holds the base "
               "rigid)")
    out.append("module pedestal() { hull() { palm(); translate([0, 0, 5]) linear_extrude(0.01) projection() palm(); } }")
    (tv, tu, tr), (bv, bu, br) = F.THUMB
    ta, tb = F.hand_to_mech((tu, tv, 0.0)), F.hand_to_mech((bu, bv, 0.0))
    out.append(f"module thumb() {{ hull() {{ translate({pt(ta, z)}) sphere({mm(tr)}); "
               f"translate({pt(tb, z)}) sphere({mm(br)}); }} }}")
    out.append("module ring_finger(lance_wall) {")
    out += finger_lines("ring", True)
    out.append("}")
    return out


def lines():
    n = numbers()
    cx0, cx1, cw, cb, ct = n["carriage"]
    px0, px1 = n["pins_x"]
    gx0, gx1, gw, gt, gu = n["guide"]
    sp, pin = n["spring"], n["pin"]
    x0, x1, y0, y1, pt_ = n["plate"]
    cpw, cpt, cpd = n["cup_pocket"]
    sw, sh = n["stand"]
    stop = n["tab_stop_deg"]
    face = sh + mm(F.TAB_H)                     # the shield's face rests here: the model's stand top plus its tab layer
    rim = face + cpd                              # the cup's rim: the model's socket wall top
    floor = sh - CUP_SINK
    r = ["// rig.scad: the real-trial rig for lancet-fingerstick, written by tools/rig_scad.py from lancet_fingerstick.py.",
         "// Do not edit by hand; rerun the tool. Units mm, z from the table top, x toward the wrist from the ring's apex,",
         "// y to the thumb side. Render each part on its own: part = \"base\", \"ring\", \"stand\" or \"cup\" (\"all\" views",
         "// the set assembled, the ring finger on its top stop).",
         f"// The base prints in one piece, {n['base_size'][0]} by {n['base_size'][1]} by {n['base_size'][2]} mm: a bed of at "
         f"least {n['base_size'][0] + 10} by {n['base_size'][1] + 10} mm",
         f"// with {n['base_size'][2] + 5} mm of height (from the model's shapes; not rendered).",
         "// Print (CHOICE, not trialled): PETG; base on its plate with tree supports under the fingers, thumb and palm;",
         "// ring on its side with supports under the pad; stand on its bottom; cup rim up.",
         f"// Bought: two {PIN_D:g} mm steel dowel pins {pin['length']} mm long, pressed {PIN_PRESS:g} mm into the carriage;",
         f"// two {PIN_D:g} mm bore shaft collars ({COLLAR_OD:g} mm OD, {COLLAR_H:g} mm wide) clamped on the pins under the "
         "guide block's top,",
         "// where they stop the finger at the top of its travel; two compression springs, one round each pin under its",
         f"// collar, each {sp['rate_each']} N/mm, free length {sp['free']} mm, inside diameter over {PIN_D + 0.5:g} mm, "
         f"outside under {COLLAR_OD:g} mm,",
         f"// solid length under {sp['bottom']} mm [CHOICE: not sourced; README check R measures the slide as built].",
         'part = "all";',
         "lance_wall = 1.0;   // the Lance trainer finger's wall, measured with calipers (README check 3b) [EST until measured]",
         "tab_w = 6.0; tab_t = 3.0; tab_h = 4.0;   // the lancet's tab cap, measured with calipers (README check 3) [EST]",
         "$fn = 48;",
         "module capsule(a, b, r) { hull() { translate(a) sphere(r); translate(b) sphere(r); } }",
         ]
    r += hand_lines()
    r += [
        f"// the slide: the carriage {cx1 - cx0:g} by {cw:g} mm, z {cb} to {ct}, fused under the ring finger's knuckle; it",
        f"// lands on the guide block's top (z {gt}) after {n['drop_stop']} mm, the lower stop",
        f"module carriage() {{ translate([{cx0}, {-cw / 2}, {cb}]) cube([{round(cx1 - cx0, 3)}, {cw}, {round(ct - cb, 3)}]); }}",
        f"module pin_holes(d, z0, h) {{ for (x = [{px0}, {px1}]) translate([x, 0, z0]) cylinder(d = d, h = h); }}",
        "module ring() { difference() { union() { ring_finger(lance_wall); carriage(); }",
        f"  pin_holes({PIN_D - 0.1:g}, {round(cb - 0.01, 3)}, {PIN_PRESS + 0.01:g}); }} }}   // the pins pressed in from below",
        f"// the guide block: a bridge {gx1 - gx0:g} by {gw:g} mm on the plate, its top z {gu} to {gt} carrying the",
        f"// pins' bushings; under it a slot open across y (z 5 to {gu}) for the collars and the springs",
        "module guide() { difference() {",
        f"  translate([{gx0}, {-gw / 2}, 5]) cube([{round(gx1 - gx0, 3)}, {gw}, {round(gt - 5, 3)}]);",
        f"  translate([{gx0 - 1}, {-gw / 2 - 1}, 5 - 0.01]) cube([{round(gx1 - gx0 + 2, 3)}, {gw + 2}, {round(gu - 5 + 0.01, 3)}]);",
        f"  pin_holes({PIN_D + BUSH_CLEAR:g}, {round(gu - 1, 3)}, {BRIDGE_T + 2:g}); }}",
        f"  for (x = [{px0}, {px1}]) translate([x, 0, 5]) cylinder(d = {SEAT_D}, h = {SEAT_H});   // the spring seats",
        f"  for (y = [{-gw / 2}, {gw / 2 - 4}]) translate([{gx0}, y, 5]) cube([{round(gx1 - gx0, 3)}, 4, {round(gu - 5, 3)}]);"
        "   // the bridge's two walls",
        "}",
        "module base() {",
        f"  translate([{x0}, {y0}, 0]) cube([{round(x1 - x0, 3)}, {round(y1 - y0, 3)}, {pt_}]);   // the plate, ballasted "
        f"to {F.BASE_KG:g} kg and taped down",
        "  guide(); index(); palm(); pedestal(); thumb();",
        "}",
        f"module stand() {{ difference() {{ translate([{-sw / 2}, {-sw / 2}, 0]) cube([{sw}, {sw}, {sh}]);",
        f"  translate([0, 0, {floor}]) cylinder(d = {CUP_OD + FIT}, h = {CUP_SINK + 1}); }}   // the cup's recess",
        f"  difference() {{ cylinder(d = {POST_D}, h = {face});   // the fixed post, its top at the shield's face",
        f"    translate([-tab_w / 2, -tab_t / 2, {face} - tab_h]) cube([tab_w, tab_t, tab_h + 0.01]); }}"
        "   // the slot that holds the tab cap still",
        f"  translate([{STOP_R}, 0, {floor}]) cylinder(d = {STOP_PIN_D}, h = 3);   // the stop pin in the recess's floor",
        "}",
        f"// the cup: keyed to the housing ({cpw} by {cpt} mm, {cpd} mm deep, {mm(F.CUP_CLEAR)} mm each side), turning on the",
        f"// post; an arc slot under it meets the stop pin at +-{stop:g} deg, the model's TAB_STOP either way",
        f"module cup() {{ difference() {{ translate([0, 0, {floor}]) cylinder(d = {CUP_OD}, h = {round(rim - floor, 3)});",
        f"  translate([{-cpw / 2}, {-cpt / 2}, {face}]) cube([{cpw}, {cpt}, {cpd + 1}]);",
        f"  translate([0, 0, {floor - 1}]) cylinder(d = {POST_D + FIT}, h = {round(face - floor + 1.01, 3)});"
        "   // turns on the post",
        f"  translate([0, 0, {floor - 0.01}]) rotate([0, 0, {180 - stop:g}]) rotate_extrude(angle = {2 * stop:g}) "
        f"translate([{STOP_R - STOP_PIN_D / 2 - FIT / 2}, 0]) square([{STOP_PIN_D + FIT}, 3.4]);",
        "} }",
        'if (part == "base" || part == "all") color("grey") base();',
        'if (part == "ring" || part == "all") ring();',
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
