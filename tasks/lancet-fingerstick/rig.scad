// rig.scad: the real-trial rig for lancet-fingerstick, written by tools/rig_scad.py from lancet_fingerstick.py.
// Do not edit by hand; rerun the tool. Units mm, z from the table top, x toward the wrist from the ring's apex,
// y to the thumb side. Render each part on its own: part = "base", "ring", "stand" or "cup" ("all" views
// the set assembled, the ring finger on its top stop).
// The base prints in one piece, 191 by 120 by 96 mm: a bed of at least 201 by 130 mm
// with 101 mm of height (from the model's shapes; not rendered).
// Print (CHOICE, not trialled): PETG; base on its plate with tree supports under the fingers, thumb and palm;
// ring on its side with supports under the pad; stand on its bottom; cup rim up.
// Bought: two 4 mm steel dowel pins 41.0 mm long, pressed 8 mm into the carriage;
// two 4 mm bore shaft collars (10 mm OD, 5 mm wide) clamped on the pins under the guide block's top,
// where they stop the finger at the top of its travel; two compression springs, one round each pin under its
// collar, each 0.05 N/mm, free length 41.471 mm, inside diameter over 4.5 mm, outside under 10 mm,
// solid length under 20.0 mm [CHOICE: not sourced; README check R measures the slide as built].
part = "all";
lance_wall = 1.0;   // the Lance trainer finger's wall, measured with calipers (README check 3b) [EST until measured]
tab_w = 6.0; tab_t = 3.0; tab_h = 4.0;   // the lancet's tab cap, measured with calipers (README check 3) [EST]
$fn = 48;
module capsule(a, b, r) { hull() { translate(a) sphere(r); translate(b) sphere(r); } }
module index() {
  // index: distal radius 8.2, proximal 8.8
  capsule([10.2, 38.971, 47.5], [28.6, 38.971, 47.5], 8.2);
  capsule([28.6, 38.971, 47.5], [97.0, 38.971, 47.5], 8.8);
  capsule([93.0, 38.971, 47.5], [122.0, 38.971, 47.5], 8.8);   // the rod from its knuckle into the palm block
}
module palm() { hull() for (p = [[116.0, -30.749, 72.742], [178.0, -30.749, 72.742], [178.0, 43.729, 29.742], [116.0, 43.729, 29.742], [116.0, -17.749, 95.258], [178.0, -17.749, 95.258], [178.0, 56.729, 52.258], [116.0, 56.729, 52.258]]) translate(p) cube(0.01, center = true); }
// the pedestal: the palm block's footprint carried down to the plate (not in the sim, which holds the base rigid)
module pedestal() { hull() { palm(); translate([0, 0, 5]) linear_extrude(0.01) projection() palm(); } }
module thumb() { hull() { translate([60.0, 75.344, 26.5]) sphere(9.5); translate([167.0, 52.828, 39.5]) sphere(12.0); } }
module ring_finger(lance_wall) {
  // ring: distal radius 7.3, proximal 7.8: the distal pad thinner by the Lance finger's wall, which brings it back
  capsule([7.3, 0.0, 70.0], [26.6, 0.0, 70.0], 7.3 - lance_wall);
  capsule([26.6, 0.0, 70.0], [99.0, 0.0, 70.0], 7.8);
}
// the slide: the carriage 20 by 10 mm, z 58.0 to 67.0, fused under the ring finger's knuckle; it
// lands on the guide block's top (z 48.0) after 10.0 mm, the lower stop
module carriage() { translate([93.0, -5.0, 58.0]) cube([20.0, 10.0, 9.0]); }
module pin_holes(d, z0, h) { for (x = [98.0, 108.0]) translate([x, 0, z0]) cylinder(d = d, h = h); }
module ring() { difference() { union() { ring_finger(lance_wall); carriage(); }
  pin_holes(3.9, 57.99, 8.01); } }   // the pins pressed in from below
// the guide block: a bridge 20 by 24 mm on the plate, its top z 40.0 to 48.0 carrying the
// pins' bushings; under it a slot open across y (z 5 to 40.0) for the collars and the springs
module guide() { difference() {
  translate([93.0, -12.0, 5]) cube([20.0, 24.0, 43.0]);
  translate([92.0, -13.0, 5 - 0.01]) cube([22.0, 26.0, 35.01]);
  pin_holes(4.2, 39.0, 10); }
  for (x = [98.0, 108.0]) translate([x, 0, 5]) cylinder(d = 3.5, h = 8.0);   // the spring seats
  for (y = [-12.0, 8.0]) translate([93.0, y, 5]) cube([20.0, 4, 35.0]);   // the bridge's two walls
}
module base() {
  translate([89.0, -35.0, 0]) cube([104.0, 95.0, 5.0]);   // the plate, ballasted to 3 kg and taped down
  guide(); index(); palm(); pedestal(); thumb();
}
module stand() { difference() { translate([-20.0, -20.0, 0]) cube([40.0, 40.0, 25.0]);
  translate([0, 0, 19.0]) cylinder(d = 24.4, h = 7.0); }   // the cup's recess
  difference() { cylinder(d = 10.0, h = 27.0);   // the fixed post, its top at the shield's face
    translate([-tab_w / 2, -tab_t / 2, 27.0 - tab_h]) cube([tab_w, tab_t, tab_h + 0.01]); }   // the slot that holds the tab cap still
  translate([9.0, 0, 19.0]) cylinder(d = 3.0, h = 3);   // the stop pin in the recess's floor
}
// the cup: keyed to the housing (13.8 by 10.8 mm, 8.0 mm deep, 0.4 mm each side), turning on the
// post; an arc slot under it meets the stop pin at +-100 deg, the model's TAB_STOP either way
module cup() { difference() { translate([0, 0, 19.0]) cylinder(d = 24.0, h = 16.0);
  translate([-6.9, -5.4, 27.0]) cube([13.8, 10.8, 9.0]);
  translate([0, 0, 18.0]) cylinder(d = 10.4, h = 9.01);   // turns on the post
  translate([0, 0, 18.99]) rotate([0, 0, 80]) rotate_extrude(angle = 200) translate([7.3, 0]) square([3.4, 3.4]);
} }
if (part == "base" || part == "all") color("grey") base();
if (part == "ring" || part == "all") ring();
if (part == "stand" || part == "all") translate([0, -120, 0]) stand();
if (part == "cup" || part == "all") translate([0, -120, 0]) color("white") cup();
