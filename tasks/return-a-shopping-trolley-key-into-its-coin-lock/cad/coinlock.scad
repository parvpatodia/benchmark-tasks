// Coin lock, key and handle clamp for the real trial of the trolley task. MIT licence, see ../README.md.
//
// One frame for every part, the sim's lock frame in millimetres: x across the handle (+x is the front face with the
// key slot), y along the handle, z up from the handle tube's axis. Each part is modelled where it sits in the
// assembled lock, so an exported STL is in assembly position; BUILD.md says how to lay it on the bed.
//
// Block 1 (SIM) holds the sim's numbers under the same names as trolley.py, converted to mm. test_trolley.py
// (class Cad) parses every "NAME = number;" line of block 1 and checks it against the scene constant, so the
// printed contact geometry is the simulated geometry. Block 2 (REAL) holds what the sim does not need: clearances,
// spring pockets, fasteners and the handle clamp. Block 3 holds the bought springs' catalogue values.
//
//   openscad -o out.stl -D 'part="slide"' coinlock.scad     (parts: see the list at the end; make.sh exports all)

part = "assembly";
$fn = 48;

// ---- 1. SIM: trolley.py constants in mm (checked by test_trolley.py) ------------------------------------------------
TUBE_R = 12.5;
LOCK_L = 90;
LOCK_W = 70;
LOCK_H = 68;
LOCK_Z0 = 12.5;
LOCK_Z1 = 80.5;
FX = 35;
HX = 35;
HY = 45;
SLOT_Y = -30;
SLOT_Z = 24;
SLOT_W = 21;
SLOT_T = 4.5;
SY0 = -40.5;
SY1 = -19.5;
Z_KEY0 = 21.75;
Z_KEY1 = 26.25;
Z_COV1 = 31;
Z_SLD1 = 74.5;
SLD_X = 17.5;
SLD_Y1 = 37;
PLUNGER_IN = 8;
RELEASE_TRAVEL = 4.5;
PLUNGER_STOP = 6;
HOME_DEPTH = 12.5;
PUSH_TRAVEL = 40;
PL_X0 = 10;
PL_X1 = 27;
T_X0 = 4.5;
T_X1 = 12.5;
T_Y0 = -12;
T_Y1 = -6;
L_X0 = 8;
L_X1 = 16;
L_Y0 = -5.5;
L_Y1 = 36.5;
HOLE_X0 = -2.5;
HOLE_X1 = 13.5;
HOLE_Y0 = -13;
HOLE_Y1 = 35.5;
TOOTH_TOP = 41.5;
LUG_TOP = 42;
WALL_H = 2.5;
WALL_T = 1.5;
POCKET = 3.25;
SLIDE_FLOOR = 34;
COIN_X = -6;
COIN_Y0 = -19;
COIN_D = 24.26;
COIN_T = 1.75;
APER_D0 = 6;
APER_D1 = 14;
APER_W = 9;
PIN_HX = 2.5;
PIN_HY = 1;
PIN_X = 31;
PIN_Z0 = 22.25;
PIN_RAMP = 4.5;
PIN_RISE = 4.5;
PIN_PRE = 4;
PLUNGER_PRE = 1;
SLIDE_PRE = 30;
KEY_L = 62;
KEY_W = 15;
KEY_T = 3;
KEY_EYE = 8;
ANCHOR_X = -38;
ANCHOR_Y = -30;
ANCHOR_Z = 16.5;

// derived exactly as in trolley.py
COIN_R = COIN_D / 2;
PIN_TOP = PIN_Z0 + 6;                       // pin_shapes(): flat top 6 mm over its low edge
W_LO = COIN_X - POCKET / 2;
W_HI = COIN_X + POCKET / 2;
POCKET_S = sqrt(COIN_R * COIN_R - pow(COIN_R - WALL_H - 0.4, 2));   // end stops 0.4 mm under the rim
CY0 = COIN_Y0 - POCKET_S;
CY1 = COIN_Y0 + POCKET_S;
BEHIND_X1 = PL_X0 - PLUNGER_STOP - 1;      // casing behind the plunger ends here (sim: 1 mm past the stop)

// ---- 2. REAL: what the printed lock adds (not in the sim; none of it touches the key, the coin or the hand) -------
CLR = 0.3;              // sliding clearance a side for PETG on PETG [CHOICE]
// The sim's plunger body (in the key slot) and its tooth T (up in the slide layer) are one rigid link 6 to 28 mm apart
// along y, and T passes through the cover. The printed plunger joins them with an arm in the key layer, in a pocket
// beside the slot, and T rises through a slot in the cover whose two ends are the plunger's stops (0 and 6 mm).
ARM_X1 = 20;            // arm's +x end; the homed key's tip stops at FX - 14 = 21 mm [CHOICE]
ARM_Y0 = SY1 - 1;       // arm overlaps the plunger body (which ends at SY1 - 0.5) by 0.5 mm
// plunger spring: VD-006C in a recess behind the plunger; two cheeks on the plunger's back keep it centred
CHEEK_L = 2.5;          // cheek length behind the plunger [CHOICE: 0.5 mm clear of the seat at the 6 mm stop]
CHEEK_T = 1.0;
CHEEK_GAP = 3.2;        // between the cheeks; the spring is 2.5 mm across (bush 2.9 mm, block 3)
RECESS_W = CHEEK_GAP + 2 * CHEEK_T + 2 * CLR + 0.2;
// pin: the sim's 5 x 2 mm blade with a 5 x 4 mm head on top. The head on the bore's shoulder is the pin's down stop
// (sim joint 0); the head bore's top is its up stop (PIN_RISE). The VD-006 spring pushes the head down.
PIN_HEAD_Y = 4;
PIN_HEAD_H = 2.5;
PIN_BORE_D = 3.4;       // spring bore; VD-006 needs a bush of 3.1 mm or more
// release hole: a 1.5 mm wire pushed up through it and through the homed key's hole lifts the pin to reset the lock
REL_D = 1.8;
REL_X = PIN_X - PIN_HX + 2;
// slide spring VD-042J-05 on a 3 mm rod (M3 threaded rod), between the casing's end wall and a post on the slide
SSPR_DEFL_IN = 50;      // spring deflection with the slide in [CHOICE: 3.45 N in, 0.69 N at the out stop]
POST_T = 3;
POST_X0 = -SLD_X + 0.5;
POST_X1 = W_LO - WALL_T - 0.875;  // 0.875 mm clear of the coin pocket's wall
SPR_X = (POST_X0 + POST_X1) / 2;
SPR_Z = 62;             // spring axis height [CHOICE: over the coin's top at 58.3 mm]
POST_TOP = 68;
ROD_D = 3;
// out stop: a block under the cap; the post's -y face meets it at the slide's full PUSH_TRAVEL. The rod (top at
// SPR_Z + 1.5 = 63.5) passes under it.
STOP_Z0 = 64.5;
STOP_Y0 = -26;
// fasteners: M3 x 20 DIN 912 (head 5.5 x 3 mm), into 2.5 mm pilot holes in PETG
M3_CLR = 3.4;
M3_PILOT = 2.5;
M3_HEAD_D = 6.0;        // counterbore for the 5.5 mm head
M3_HEAD_H = 3.5;
STACK = [[-28, -38], [-28, 38], [28, 38], [28, -10]];   // screws through floor, key plate and cover into the shell
CAPS = [[-26, -41], [26, -41], [-26, 41], [26, 41]];     // screws through the cap into the shell
// handle clamp: saddle under the floor (part of the floor plate) and a lower half, round a 25 mm tube
CLAMP_HX = 20;
CLAMP_HY = 40;
CLAMP_GAP = 0.25;       // a side of the split, so the screws pull the halves onto the tube
CLAMP_BOT = -TUBE_R - 4;
CLAMP_SCREWS = [[-16.5, -30], [16.5, -30], [-16.5, 30], [16.5, 30]];
// chain tab: the chain's split ring goes through a hole at the sim's chain anchor
TAB_T = 3;
// key: eye hole for the split ring
EYE_HOLE_D = 3;

// ---- 3. Bought springs: Gutekunst Federn catalogue values (federnshop.com product pages, read 2026-11-21) -----------
// VD-006C plunger spring: d 0.2, De 2.5, L0 10, Ln 2.91, sn 7.09, R 0.135 N/mm, bush >= 2.9, mandrel <= 2
PSPR_L0 = 10;
PSPR_LN = 2.91;
PSPR_SN = 7.09;
PSPR_R = 0.135;
PSPR_DE = 2.5;
PSPR_DH = 2.9;
// VD-006 pin spring: d 0.2, De 2.7, L0 25.3, Ln 5.64, sn 19.66, R 0.048 N/mm, bush >= 3.1
PINSPR_L0 = 25.3;
PINSPR_LN = 5.64;
PINSPR_SN = 19.66;
PINSPR_R = 0.048;
PINSPR_DE = 2.7;
PINSPR_DH = 3.1;
// VD-042J-05 slide spring: d 0.4, De 6, L0 64.7, Ln 11.72, sn 52.98, R 0.069 N/mm, Fn 3.653 N, mandrel <= 4.8
SSPR_L0 = 64.7;
SSPR_LN = 11.72;
SSPR_SN = 52.98;
SSPR_R = 0.069;
SSPR_FN = 3.653;
SSPR_DD = 4.8;
SSPR_DE = 6;

// derived real geometry
PSPR_X0 = PL_X0 - (PSPR_L0 - PLUNGER_PRE);              // plunger spring's seat: it stands L0 - preload at rest
PIN_SPR_Z0 = PIN_TOP + PIN_HEAD_H;                      // spring's low end at rest (head top)
PIN_SPR_Z1 = PIN_SPR_Z0 + PINSPR_L0 - PIN_PRE;          // bore top: the spring stands L0 - preload at rest
POST_Y1 = SLD_Y1 - (SSPR_L0 - SSPR_DEFL_IN);            // post's +y face with the slide in
POST_Y0 = POST_Y1 - POST_T;
STOP_Y1 = POST_Y0 - PUSH_TRAVEL;                        // post's -y face at the full travel
ROD_Y0 = STOP_Y1 - 4;                                   // rod's free end, 4 mm past the post at the out stop
ROD_L = HY - ROD_Y0;    // cut from M3 threaded rod; screwed in from the +y face, flush

module bx(lo, hi) { translate(lo) cube([hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]]); }
module screw_holes(list, z0, z1, d) { for (p = list) translate([p[0], p[1], z0]) cylinder(d = d, h = z1 - z0); }

// ---- parts ----------------------------------------------------------------------------------------------------------
module floor_plate() {        // z LOCK_Z0..Z_KEY0, with the saddle and the chain tab
    difference() {
        union() {
            bx([-HX, -HY, LOCK_Z0], [HX, HY, Z_KEY0]);
            bx([-CLAMP_HX, -CLAMP_HY, CLAMP_GAP], [CLAMP_HX, CLAMP_HY, LOCK_Z0 + 0.01]);
            bx([ANCHOR_X - 3, ANCHOR_Y - TAB_T / 2, LOCK_Z0], [-HX + 0.01, ANCHOR_Y + TAB_T / 2, ANCHOR_Z + 4]);
        }
        rotate([90, 0, 0]) cylinder(r = TUBE_R + CLR, h = 2 * HY + 2, center = true);
        translate([ANCHOR_X, 0, ANCHOR_Z]) rotate([90, 0, 0]) cylinder(d = 3.2, h = 2 * HY, center = true);
        screw_holes(STACK, LOCK_Z0 - 1, Z_KEY0 + 1, M3_CLR);
        screw_holes(STACK, LOCK_Z0 - 1, LOCK_Z0 + M3_HEAD_H, M3_HEAD_D);
        screw_holes(CLAMP_SCREWS, CLAMP_GAP - 1, LOCK_Z0 + 6, M3_PILOT);
        translate([REL_X, SLOT_Y, LOCK_Z0 - 1]) cylinder(d = REL_D, h = Z_KEY0 - LOCK_Z0 + 2);
    }
}

module clamp_lower() {        // the lower half of the handle clamp
    difference() {
        bx([-CLAMP_HX, -CLAMP_HY, CLAMP_BOT], [CLAMP_HX, CLAMP_HY, -CLAMP_GAP]);
        rotate([90, 0, 0]) cylinder(r = TUBE_R + CLR, h = 2 * HY + 2, center = true);
        screw_holes(CLAMP_SCREWS, CLAMP_BOT - 1, 0, M3_CLR);
        screw_holes(CLAMP_SCREWS, CLAMP_BOT - 1, CLAMP_BOT + M3_HEAD_H, M3_HEAD_D);
    }
}

module key_plate() {          // z Z_KEY0..Z_KEY1: the slot, the plunger's way, the arm pocket, the spring recess
    difference() {
        bx([-HX, -HY, Z_KEY0], [HX, HY, Z_KEY1]);
        bx([BEHIND_X1, SY0, Z_KEY0 - 1], [HX + 1, SY1, Z_KEY1 + 1]);                              // slot (sim)
        bx([T_X0 - PLUNGER_STOP - CLR, ARM_Y0 - CLR, Z_KEY0 - 1], [ARM_X1 + CLR, T_Y1 + CLR, Z_KEY1 + 1]);   // arm
        bx([PSPR_X0, SLOT_Y - RECESS_W / 2, Z_KEY0 - 1], [BEHIND_X1 + 0.01, SLOT_Y + RECESS_W / 2, Z_KEY1 + 1]);
        screw_holes(STACK, Z_KEY0 - 1, Z_KEY1 + 1, M3_CLR);
    }
}

module cover_plate() {        // z Z_KEY1..Z_COV1: T's slot (its ends are the plunger's stops) and the pin's bores
    difference() {
        bx([-HX, -HY, Z_KEY1], [HX, HY, Z_COV1]);
        bx([T_X0 - PLUNGER_STOP, T_Y0 - CLR, Z_KEY1 - 1], [T_X1, T_Y1 + CLR, Z_COV1 + 1]);
        bx([PIN_X - PIN_HX - 0.1, SLOT_Y - PIN_HY - 0.1, Z_KEY1 - 1], [PIN_X + PIN_HX + 0.1, SLOT_Y + PIN_HY + 0.1, Z_COV1 + 1]);
        pin_head_bore();
        screw_holes(STACK, Z_KEY1 - 1, Z_COV1 + 1, M3_CLR);
    }
}

module pin_head_bore() {
    bx([PIN_X - PIN_HX - 0.2, SLOT_Y - PIN_HEAD_Y / 2 - 0.2, PIN_TOP],
       [PIN_X + PIN_HX + 0.2, SLOT_Y + PIN_HEAD_Y / 2 + 0.2, PIN_TOP + PIN_HEAD_H + PIN_RISE]);
}

module shell() {              // z Z_COV1..Z_SLD1: the slide channel's walls and end wall, the pin's spring bore
    difference() {
        union() {
            bx([-HX, -HY, Z_COV1], [-SLD_X, HY, Z_SLD1]);
            bx([SLD_X, -HY, Z_COV1], [HX, HY, Z_SLD1]);
            bx([-SLD_X - 0.01, SLD_Y1, Z_COV1], [SLD_X + 0.01, HY, Z_SLD1]);
        }
        pin_head_bore();
        translate([PIN_X, SLOT_Y, Z_COV1 - 1]) cylinder(d = PIN_BORE_D, h = PIN_SPR_Z1 - Z_COV1 + 1);
        translate([SPR_X, SLD_Y1 - 1, SPR_Z]) rotate([-90, 0, 0]) cylinder(d = M3_PILOT, h = HY - SLD_Y1 + 2);   // rod self-taps
        screw_holes(STACK, Z_COV1 - 1, Z_COV1 + 8, M3_PILOT);
        screw_holes(CAPS, Z_SLD1 - 14, Z_SLD1 + 1, M3_PILOT);
    }
}

module cap() {                // z Z_SLD1..LOCK_Z1 (red), with the slide's out-stop block under it
    difference() {
        union() {
            bx([-HX, -HY, Z_SLD1], [HX, HY, LOCK_Z1]);
            bx([-SLD_X - 0.01, STOP_Y0, STOP_Z0], [POST_X1 + 0.5, STOP_Y1, Z_SLD1 + 0.01]);
        }
        screw_holes(CAPS, Z_SLD1 - 1, LOCK_Z1 + 1, M3_CLR);
        screw_holes(CAPS, LOCK_Z1 - M3_HEAD_H, LOCK_Z1 + 1, M3_HEAD_D);
    }
}

module plunger() {            // at rest (sim joint 0). Body and tooth are the sim's boxes; arm and cheeks are added
    union() {
        bx([PL_X0, SY0 + 0.5, Z_KEY0 + 0.5], [PL_X1, SY1 - 0.5, Z_KEY1 - 0.5]);                  // body (sim)
        bx([T_X0, T_Y0, Z_KEY1 - 0.5], [T_X1, T_Y1, TOOTH_TOP]);                                  // tooth T (sim)
        bx([T_X0, ARM_Y0, Z_KEY0 + 0.5], [ARM_X1, T_Y1, Z_KEY1 - 0.5]);                           // arm
        for (s = [-1, 1])
            bx([PL_X0 - CHEEK_L, SLOT_Y + s * CHEEK_GAP / 2 + (s < 0 ? -CHEEK_T : 0), Z_KEY0 + 0.5],
               [PL_X0 + 0.01, SLOT_Y + s * CHEEK_GAP / 2 + (s < 0 ? 0 : CHEEK_T), Z_KEY1 - 0.5]);
    }
}

module slide() {              // slide in (sim joint 0): floor round T's hole, the coin pocket, lug L and rail (sim)
    x0 = -SLD_X + 0.5; x1 = SLD_X - 0.5; f0 = Z_COV1;
    difference() {
        union() {
            bx([x0, -HY, f0], [x1, HOLE_Y0, SLIDE_FLOOR]);
            bx([x0, HOLE_Y1, f0], [x1, L_Y1, SLIDE_FLOOR]);
            bx([x0, HOLE_Y0, f0], [HOLE_X0, HOLE_Y1, SLIDE_FLOOR]);
            bx([HOLE_X1, HOLE_Y0, f0], [x1, HOLE_Y1, SLIDE_FLOOR]);
            bx([W_LO - WALL_T, CY0, SLIDE_FLOOR], [W_LO, CY1, SLIDE_FLOOR + WALL_H]);
            bx([W_HI, CY0, SLIDE_FLOOR], [W_HI + WALL_T, CY1, SLIDE_FLOOR + WALL_H]);
            bx([W_LO, CY0 - WALL_T, SLIDE_FLOOR], [W_HI, CY0, SLIDE_FLOOR + WALL_H]);
            bx([W_LO, CY1, SLIDE_FLOOR], [W_HI, CY1 + WALL_T, SLIDE_FLOOR + WALL_H]);
            bx([L_X0, L_Y0, SLIDE_FLOOR], [L_X1, L_Y1, LUG_TOP]);
            bx([POST_X0, POST_Y0, SLIDE_FLOOR - 0.01], [POST_X1, POST_Y1, POST_TOP]);              // spring post (real)
        }
        translate([SPR_X, POST_Y0 - 1, SPR_Z]) rotate([-90, 0, 0]) cylinder(d = M3_CLR + 0.2, h = POST_T + 2);
    }
}

module pin() {                // at rest: the sim's blade (upright -x face, bottom rising PIN_RAMP toward +x) + head
    x0 = PIN_X - PIN_HX; x1 = PIN_X + PIN_HX; y0 = SLOT_Y - PIN_HY; y1 = SLOT_Y + PIN_HY;
    hull() polyhedron(points = [[x0, y0, PIN_Z0], [x1, y0, PIN_Z0 + PIN_RAMP], [x1, y1, PIN_Z0 + PIN_RAMP], [x0, y1, PIN_Z0],
                                [x0, y0, PIN_TOP], [x1, y0, PIN_TOP], [x1, y1, PIN_TOP], [x0, y1, PIN_TOP]],
                      faces = [[0, 1, 2, 3], [7, 6, 5, 4], [0, 4, 5, 1], [1, 5, 6, 2], [2, 6, 7, 3], [3, 7, 4, 0]]);
    bx([x0, SLOT_Y - PIN_HEAD_Y / 2, PIN_TOP], [x1, SLOT_Y + PIN_HEAD_Y / 2, PIN_TOP + PIN_HEAD_H]);
}

module key_outline() {        // 2D, tip at the origin pointing -y, as cut from 3 mm sheet
    hw = KEY_W / 2; aw = APER_W / 2;
    difference() {
        union() {
            polygon([[-0.6 * hw, 0], [0.6 * hw, 0], [hw, 4], [hw, KEY_L], [-hw, KEY_L], [-hw, 4]]);
            translate([-4, KEY_L - 0.01]) square([8, KEY_EYE + 0.01]);
        }
        translate([-aw, APER_D0]) square([APER_W, APER_D1 - APER_D0]);
        translate([0, KEY_L + KEY_EYE - EYE_HOLE_D / 2 - 1.5]) circle(d = EYE_HOLE_D);
    }
}

module key() {                // homed in the slot: tip HOME_DEPTH past the mouth, flat, on the slot floor, centred
    translate([FX - HOME_DEPTH, SLOT_Y, Z_KEY0]) rotate([0, 0, -90]) linear_extrude(KEY_T) key_outline();
}

module assembly() {
    color("gray") { floor_plate(); key_plate(); cover_plate(); shell(); }
    color("dimgray") clamp_lower();
    color("red") cap();
    color("orange") { plunger(); slide(); pin(); }
    color("silver") key();
    color("silver") rotate([90, 0, 0]) cylinder(r = TUBE_R, h = 120, center = true);
}

if (part == "assembly") assembly();
else if (part == "floor_plate") floor_plate();
else if (part == "clamp_lower") clamp_lower();
else if (part == "key_plate") key_plate();
else if (part == "cover_plate") cover_plate();
else if (part == "shell") shell();
else if (part == "cap") cap();
else if (part == "plunger") plunger();
else if (part == "slide") slide();
else if (part == "pin") pin();
else if (part == "key") key();
else if (part == "key_dxf") key_outline();
