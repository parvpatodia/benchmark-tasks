// rig.scad: the real-trial rig for lancet-fingerstick, written by tools/rig_scad.py from lancet_fingerstick.py.
// Do not edit by hand; rerun the tool. Units mm, z from the table top, x toward the wrist from the ring's apex.
// Render each part on its own: part = "hand_front", "hand_back", "base", "stand" or "cup" ("all" views the set).
// The hand is 272 mm long, so it prints in two parts cut at x = 200 mm (across the stub),
// joined with epoxy on the two 3 mm alignment pins. Print PETG at the infill that keeps it under the model's mass
// (0.200 kg) with the bore empty; lead shot in the bore then sets the tipping force (README check 4).
// Print (CHOICE, not trialled): hand parts lying on their palm side with tree supports under the fingers and the
// thumb; base on its plate, no supports (the 3.4 mm pin holes bridge); stand on its bottom; cup rim up, no supports.
// The hinge pin: a 3 mm steel rod 87.025 mm long through both cheeks and the keel, held by a 3 mm push-on
// retaining washer outside each cheek (CHOICE). It turns in the keel's 3.4 mm hole and in the cheeks'.
part = "all";
lance_wall = 1.0;   // the Lance trainer finger's wall, measured with calipers (README check 3b) [EST until measured]
tab_w = 6.0; tab_t = 3.0; tab_h = 4.0;   // the lancet's tab cap, measured with calipers (README check 3) [EST]
$fn = 48;
module capsule(a, b, r) { hull() { translate(a) sphere(r); translate(b) sphere(r); } }
module fingers(lance_wall) {
  // index: distal radius 8.2, proximal 8.8
  capsule([10.2, 38.971, 47.5], [28.6, 38.971, 47.5], 8.2);
  capsule([28.6, 38.971, 47.5], [97.0, 38.971, 47.5], 8.8);
  // middle: distal radius 7.8, proximal 8.3: the distal pad thinner by the Lance finger's wall, which brings it back
  capsule([-0.2, 18.187, 59.5], [18.6, 18.187, 59.5], 7.8 - lance_wall);
  capsule([18.6, 18.187, 59.5], [95.0, 18.187, 59.5], 8.3);
  // ring: distal radius 7.3, proximal 7.8: the distal pad thinner by the Lance finger's wall, which brings it back
  capsule([7.3, 0.0, 70.0], [26.6, 0.0, 70.0], 7.3 - lance_wall);
  capsule([26.6, 0.0, 70.0], [99.0, 0.0, 70.0], 7.8);
  // little: distal radius 6.5, proximal 6.8
  capsule([30.5, -16.454, 79.5], [50.6, -16.454, 79.5], 6.5);
  capsule([50.6, -16.454, 79.5], [104.0, -16.454, 79.5], 6.8);
}
module palm() { hull() for (p = [[99.0, -30.749, 72.742], [178.0, -30.749, 72.742], [178.0, 43.729, 29.742], [99.0, 43.729, 29.742], [99.0, -17.749, 95.258], [178.0, -17.749, 95.258], [178.0, 56.729, 52.258], [99.0, 56.729, 52.258]]) translate(p) cube(0.01, center = true); }
module thumb() { hull() { translate([60.0, 75.344, 26.5]) sphere(9.5); translate([167.0, 52.828, 39.5]) sphere(12.0); } }
module stub() { translate([182.0, 12.99, 62.5]) rotate([0, 90, 0]) cylinder(r = 22.0, h = 82.0); }
// the wrist: the model's palm and stub are one rigid link with a 4 mm gap between them; the print bridges it
module wrist() { hull() { intersection() { palm(); translate([172.0, -100, -10]) cube([6, 200, 200]); } intersection() { stub(); translate([182.0, -100, -10]) cube([6, 200, 200]); } } }
module stub_bore() { translate([204.0, 12.99, 62.5]) rotate([0, 90, 0]) cylinder(d = 16.0, h = 61.0); }
pin_x = 62.0; pin_z = 70.0;   // the hinge's axis: the model's (PIVOT_X, HINGE_Z), on the ring finger's axis
module pin_hole() { translate([pin_x, -31.254, pin_z]) rotate([-90, 0, 0]) cylinder(d = 3.4, h = 87.025); }
module keel_box() { translate([pin_x - 7.0, -23.254, pin_z - 6.0]) cube([14.0, 71.025, 40]); }
// the keel: a web between the cheeks from under the pin up into the fingers, clipped to the fingers' hull so
// nothing stands above them; the pin's hole runs through it
module keel() { intersection() { keel_box(); hull() { fingers(0); translate([pin_x - 7.0, -23.254, pin_z - 6.0]) cube([14.0, 71.025, 1]); } } }
module hand() { difference() { union() { fingers(lance_wall); palm(); wrist(); thumb(); stub(); keel(); }
  pin_hole(); stub_bore(); } }
module base() { difference() { union() {
  translate([32.0, -30.0, 0]) cube([231.0, 60.0, 5.0]);   // the plate, ballasted to 3 kg and taped down
  for (y = [-28.254, 48.771]) translate([pin_x - 8.0, y, 0]) cube([16.0, 4.0, pin_z + 4]);   // the hinge cheeks, outside the fingers
  translate([198.0, -7.01, 0]) cube([50.0, 40.0, 40.5]);   // the wrist block: the stub rests on it (the hinge's upper stop)
  } pin_hole(); } }
module stand() { difference() { translate([-20.0, -20.0, 0]) cube([40.0, 40.0, 25.0]);
  translate([0, 0, 19.0]) cylinder(d = 24.4, h = 7.0); }   // the cup's recess
  difference() { cylinder(d = 10.0, h = 27.0);   // the fixed post, its top at the shield's face
    translate([-tab_w / 2, -tab_t / 2, 27.0 - tab_h]) cube([tab_w, tab_t, tab_h + 0.01]); }   // the slot that holds the tab cap still
}
module cup() { difference() { translate([0, 0, 19.0]) cylinder(d = 24.0, h = 16.0);
  translate([-6.9, -5.4, 27.0]) cube([13.8, 10.8, 9.0]);   // keyed to the housing, 0.4 mm each side
  translate([0, 0, 18.0]) cylinder(d = 10.4, h = 9.01);   // turns on the post
} }
module hand_cut(keep_front) { difference() { intersection() { hand(); translate([keep_front ? -50 : 200.0, -100, -10]) cube([keep_front ? 200.0 + 50 : 200, 200, 200]); }
  for (y = [-1.01, 26.99]) translate([200.0 - 6, y, 62.5]) rotate([0, 90, 0]) cylinder(d = 3.2, h = 12); } }   // the alignment pins' holes
if (part == "hand_front" || part == "all") hand_cut(true);
if (part == "hand_back" || part == "all") hand_cut(false);
if (part == "base" || part == "all") color("grey") base();
if (part == "stand" || part == "all") translate([0, -120, 0]) stand();
if (part == "cup" || part == "all") translate([0, -120, 0]) color("white") cup();
