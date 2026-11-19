// rig.scad: the real-trial rig for lancet-fingerstick, written by tools/rig_scad.py from lancet_fingerstick.py.
// Do not edit by hand; rerun the tool. Units mm, z from the table top, x toward the wrist from the ring's apex,
// y to the thumb side. Render each part on its own: part = "base", "ring", "stand", "cup" or "shims"
// ("all" views the set assembled, the ring finger on its top stop).
// The base prints in one piece, 191 by 120 by 96 mm: a bed of at least 201 by 130 mm
// with 101 mm of height (from the model's shapes; not rendered).
// Print (CHOICE, not trialled): PETG; base on its plate with tree supports under the fingers, thumb and palm;
// ring on its side with supports under the pad; stand on its bottom; cup rim up; shims flat.
// Bought: one miniature linear guide LILY MGN7C1R40Z0CM (40 mm rail, block 22.5 mm, pitch moment 2.84 N m);
// four M2 x 6 socket screws (web to block, heads in 4 mm counterbores 2.2 deep, so 2.2 mm enters the block's 2.5 mm thread)
// and three M2 x 6 screws with M2 heat-set inserts (rail to post);
// two 4 x 40 mm steel dowel pins pressed 8 mm into the plate and its bosses; two 4 mm bore shaft collars
// (10 mm OD, 5 mm wide) clamped on the pins on the web's foot, the top stop; two compression springs, one on each pin
// under the foot, each music wire 0.32 mm, OD 5.12 mm, 18.8 active coils, 20.8 total, ends closed and ground,
// free length 30.953 mm, rate 0.05 N/mm, solid 6.66 mm (under the 8.5 mm it reaches) [ARITH; README check R measures it].
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
// the slide, on the ring: a web x 93.0 to 99.0 fused under the knuckle, 17 mm wide up to z 56.0 and 10 mm
// above it (clear of the holding pads), the block bolted to its +x face; its foot (z 28.5 to 33.5) rides the two pins,
// pushed up by the springs against the collars, and lands on the stop post after 10.0 mm
module web() { difference() { union() {
  translate([93.0, -8.5, 28.5]) cube([6.0, 17, 27.5]);
  translate([93.0, -5.0, 28.5]) cube([6.0, 10.0, 38.5]);
  translate([93.0, -18.5, 28.5]) cube([6.0, 37, 5.0]); }
  for (y = [-15.0, 15.0]) translate([96.0, y, 27.5]) cylinder(d = 4.5, h = 7.0);   // the pins run free
  for (p = [[-6.0, 35.75], [-6.0, 43.75], [6.0, 35.75], [6.0, 43.75]]) translate([92.99, p[0], p[1]]) rotate([0, 90, 0]) {
    cylinder(d = 2.4, h = 6.02); cylinder(d = 4.0, h = 2.2); }   // the block's four screws
} }
module ring() { union() { ring_finger(lance_wall); web(); } }
// the post: x 107.0 to 115.0, the rail screwed to its -x face through three heat-set inserts (z 16, 31, 46)
module post() { difference() {
  translate([107.0, -12.0, 5.0]) cube([8.0, 24.0, 46.0]);
  for (z = [16.0, 31.0, 46.0]) translate([106.99, 0, z]) rotate([0, 90, 0]) cylinder(d = 3.2, h = 5); } }
// the pins' bosses (their seats, z 5 to 8, then 2 shims of 1 mm: the springs sit at z 10.0) and the lower stop post (top z 18.5)
module bosses() { for (y = [-15.0, 15.0]) translate([96.0, y, 5.0]) cylinder(d = 8, h = 3); }
module stop_post() { translate([93.0, -3.0, 5.0]) cube([6.0, 6, 13.5]); }
module shims() { for (i = [0 : 3]) translate([12 * i, 0, 0]) difference() { cylinder(d = 8, h = 1); translate([0, 0, -0.01]) cylinder(d = 4.3, h = 1.02); } }
module base() { difference() { union() {
  translate([89.0, -35.0, 0]) cube([104.0, 95.0, 5.0]);   // the plate, ballasted to 3 kg and taped down
  post(); bosses(); stop_post(); index(); palm(); pedestal(); thumb(); }
  for (y = [-15.0, 15.0]) translate([96.0, y, -0.01]) cylinder(d = 3.9, h = 8.02);   // the pins pressed in
} }
module stand() { difference() { translate([-20.0, -20.0, 0]) cube([40.0, 40.0, 25.0]);
  translate([0, 0, 19.0]) cylinder(d = 24.4, h = 7.0); }   // the cup's recess
  difference() { cylinder(d = 10.0, h = 27.0);   // the fixed post, its top at the shield's face
    translate([-tab_w / 2, -tab_t / 2, 27.0 - tab_h]) cube([tab_w, tab_t, tab_h + 0.01]); }   // the slot that holds the tab cap still
  translate([9.0, 0, 19.0]) cylinder(d = 3.0, h = 3);   // the stop pin in the recess's floor, at 0 deg
}
// the cup: keyed to the housing (13.8 by 10.8 mm, 8.0 mm deep, 0.4 mm each side), turning on the
// post; an arc slot under it, centred on the stop pin at rest, meets the pin at +-100 deg, the model's TAB_STOP either way
// (the slot spans +-110.888 deg: the stop plus the pin's own 10.888 deg half-width)
module cup() { difference() { translate([0, 0, 19.0]) cylinder(d = 24.0, h = 16.0);
  translate([-6.9, -5.4, 27.0]) cube([13.8, 10.8, 9.0]);
  translate([0, 0, 18.0]) cylinder(d = 10.4, h = 9.01);   // turns on the post
  translate([0, 0, 18.99]) rotate([0, 0, -110.888]) rotate_extrude(angle = 221.776) translate([7.3, 0]) square([3.4, 3.4]);
} }
if (part == "base" || part == "all") color("grey") base();
if (part == "ring" || part == "all") ring();
if (part == "stand" || part == "all") translate([0, -120, 0]) stand();
if (part == "cup" || part == "all") translate([0, -120, 0]) color("white") cup();
if (part == "shims") shims();
