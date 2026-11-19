# Fingerstick a trainer hand with a contact-activated lancet

A fingerstick trainer lies palm up: a palm block with an index finger, a thumb and a ring finger, rolled 30 deg about
its fingers so the side of the ring fingertip faces up. The ring finger rides on a light vertical slide under its
knuckle and gives way when pressed unless it is held. A BD Microtainer contact-activated lancet (366577, 21 G x 1.8 mm)
stands tip down in a stand whose socket holds its protective tab, and a BD 305635 sharps container stands beside. The
robot twists the tab off, holds the ring finger with one gripper ("Grasp the finger", a hospital procedure for this
lancet), places the lancet square on the side of the ring fingertip that faces up with the other, presses until it
fires, and drops the used lancet into the container (the tab, the press and the discard are the steps of BD's guide).

The hold is set by the model's physics and checked by a rule. Physics: the unheld ring finger gives 2 mm at 1.20 N
straight down (a 1 N preload over its weight on a 100 N/m spring), and every press the scorer allows reaches that
long before the lancet fires at 8 N. Rule: from the lancet's first touch until 0.5 s after the click, a gripper must
be closed on the ring finger with both pads on it at every step, and the ring finger must stay within 2 mm of its top
stop, read at its slide and at its fingertip. Holding the wrist, the palm or the index finger, or closing on air
beside the ring finger, does not hold it, so it scores 0 (Review response, point 1). The demo's hold lifts the finger
against its top stop, so the hold pushes up against the press; a grip that does not lift scores 0 (stem g00).

The trainer's place and turn are drawn from the seed (Seeds), so a replayed path misses the site: a plan made for one
seed's layout, replayed on another's, failed in all 20 seed pairs tried, and the unshifted plan failed at all 8 corners
of the layout box (`test_open_loop_plan_fails_other_layouts`). Two seeds whose layouts lie within a few mm of each other
may share a plan; the test does not claim otherwise.

Files: `lancet_fingerstick.py` (the task), `mechanism.py` (articulated objects, adapted from benchmark-tasks #223, MIT),
`test_lancet_fingerstick.py` (offline tests), `task.json`, `rig.scad` (the printable real-trial rig: base with the fixed
fingers and the slide's post, the ring finger with its web, the springs' shims, the stand and its cup), `tools/`
(`gen_taskjson.py` writes task.json; `yieldsweep.py` the one-arm yield sweep; `ikcheck.py` reach; `clearance.py` the
sharps container; `grippers.py` the two arms' meshes against each other and the trainer; `rig_scad.py` writes `rig.scad`
from the task's numbers), `sim/` (run scripts and the sim stems).

## Review response (a5621a2)

1. "the robot must stabilize the punctured finger itself during the press ... rather than immobilizing a rigid finger by
   holding the wrist". The hinged rigid hand is gone. The punctured finger, the ring, is the only part that moves: it is
   on a sprung slide and gives way under any allowed one-handed press (The model, the slide; `tools/yieldsweep.py`;
   `test_unheld_finger_gives_before_fire`). A fire with no gripper closed on the ring finger scores 0 ("fired with no
   gripper holding the ring finger"), and so does the ring finger moving more than 2 mm from first touch to 0.5 s after
   the click. A gripper on the index finger, the palm or the base holds nothing that matters (`test_finger_held_rule`,
   stem i00). The slide is a real part of the rig, dimensioned in `rig.scad` (Real trials, the trainer).
2. "no socket dimensions or printable design". The stand, its post and its cup are parts `stand` and `cup` of
   `rig.scad`, with every size under Real trials, the stand.
3. "Specify a real-trial measurement for the 5 N partial-credit threshold". The 0.4 rung no longer has a firm-press
   force. It is contact: the lancet's shield pushed off its stop on the site (1.2 N), square, with the ring finger held
   and still. A real trial reads it on a kitchen scale under the trainer (1 g), tared after the hold, as a change of
   56 g (0.55 N) or more, a one-way bound (Real trials, scoring).
4. "assess the lancet angle against the local skin normal at each allowed puncture site rather than assuming every site
   faces vertically". Square is the lancet's axis within 10 deg of the skin's inward normal at the point it touches,
   read by `tilt_of` against `site_of`'s normal, never against the vertical
   (`test_square_is_against_the_local_normal_at_every_site`, every allowed site). A vertical lancet at the site's edge,
   where the normal leans 15 deg, is not square. The real trial reads the same angle from two cameras
   (`test_real_square_check_matches_the_scorer`).

## Sources

Saved copies were read; every quote here is from them.

- BD, "Successful Specimen Collection: Fingersticks" (VS7688-1), the guide for this lancet: "Have patient hold their
  hand in a downward position", "Select finger: middle or ring finger is preferable. The fifth finger must not be
  punctured because the tissue depth is insufficient to prevent bone injury.", "Identify the desired puncture site
  (shaded area).", "Twist off the tab to break the seal and discard.", "Position the safety lancet firmly against the
  puncture site as illustrated. Hold lancet between fingers. To activate, press safety lancet firmly against the
  puncture site. Do not remove the device from the site until an audible click is heard.", "Discard used safety lancet
  into a sharps container according to your facility's established procedures", and "CAUTION: Handle all biologic
  samples and blood collection "sharps" (lancets, needles, luer adapters, and blood collection sets) in accordance
  with the policies and procedures of your facility." Its figure for step 4 (lower panel) shows the collector's hand
  holding the patient's finger near its base, below the shaded site; that figure is of selecting the site, and the
  guide shows no figure of the finger during the press. Its figure shades both sides of the fingertip pad; its step 6
  figure shows the tab twisted with curved arrows and no stated direction.
- Akron Children's Hospital, laboratory procedure "Performing a Capillary Puncture" (Version 7), which names this
  lancet's family ("BD Microtainer Contact-Activated Lancet 2.0 mm (blue)"): "Grasp the finger, and using a sterile
  lancet, press firmly against the finger to make a puncture." It does not say how long the grasp lasts; that it lasts
  through the press is our reading of the sentence [inference].
- BD, FDA 510(k) K223243 (BD Microtainer Contact-Activated Lancet): "The BD Microtainer® Contact-Activated Lancet is
  activated by applying pressure against the puncture site upon contact; after skin puncture, the needle/blade returns
  into the device." and "Once activated, the device is auto-disabled". Its table gives the materials: needle carrier
  with protective tab, polystyrene; main housing, tab cap and rear cap, polypropylene; shield, polyethylene. It gives
  no activation force and no body size.
- FDA GUDID / openFDA: 366577 "LANCET CONT-ACT PINK 21G X 1.8MM US", "In Commercial Distribution", K223243. The older
  366593 is "Not in Commercial Distribution", so the task names 366577.
- BD patent US 9,005,233 ("Contact activated lancet device"): the shield pushed against the skin trips the drive
  spring; opposed sides carry "finger grip indentations". The patent allows the tab to break by twist or by pull; the
  task scores the twist BD's guide gives ("Twist off the tab"), and a pull that breaks the tab is not modelled.
- Owen Mumford, FDA 510(k) K222111 (Unistik Touch contact-activated safety lancets), bench test: "The force to activate
  the safety feature: 4 - 15N".
- Facet Technologies, FDA 510(k) K221433 (Facet 28G Universal Lancet): "Cap Twist-off torque" "< 4.9 N-cm".
- BD sharps collectors catalog: 305635 "1 qt with needle ports", "4.9 x 4.3 x 6.5" (in, L x W x H); "2.5 × 1.2 open
  (305635)" (in).
- O*NET 31-9097.00 Phlebotomists: Core task "Draw blood from capillaries by dermal puncture, such as heel or finger
  stick methods."; "143,900 employees" (2025).
- CMA (AAMA) certification exam content outline, Specimen Collection Techniques, Blood: "Capillary/Dermal puncture
  (finger, heel, ear)".
- NCCT, National Certified Medical Assistant detailed test plan (effective January 2024, from a national job analysis):
  "Perform capillary punctures (i.e., fingerstick, heelstick)."
- O*NET 31-9092.00 Medical Assistants: Core task "Collect blood, tissue, or other laboratory specimens, log the
  specimens, and prepare them for testing."; "Employment (2025)" "833,900 employees". O*NET's own definition of Core:
  relevance of at least 67% of incumbents and mean importance of at least 3.0.
- O*NET 29-2061.00 Licensed Practical and Licensed Vocational Nurses: Core task "Collect samples, such as blood, urine,
  or sputum from patients, and perform routine laboratory tests on samples."; "666,900 employees" (2025).
- McKesson item 1251387, Wallcur REM-LANCE-KIT-01D, "Blood Glucose Test Trainer Lance Finger": "Simulated finger stick
  and blood glucose trainer" and "Finger fits over the finger of a standard patient or Mannequin". It gives no size,
  mass or wall thickness.
- benchmark-tasks #1837 (merged): the trainer hand's sizes, "ANSUR II median adult male (hand length 193 mm, breadth 88
  mm)", its finger radii and spacing.

Not sourced, and set in the code with [EST] or [CHOICE]: the lancet's body size (13 x 10 x 37 mm) and mass (4 g); the
fire force, 8 N, chosen inside Unistik's 4 to 15 N because BD publishes none; the tab's break torque, 3 N-cm, under
Facet's 4.9 N-cm limit, and its break angle, 45 deg; the site's bounds, read from BD's shaded figure and narrowed to 8
mm and more from the apex, on the side that faces up; the square, slide, held and still limits; the ring finger's slide
(its 25 g, 100 N/m spring, 1 N preload and 10 mm travel); the half-gap a gripper closed on the ring finger reads (7.2
mm, planned, to be checked against the film's trace); the friction bound of the yield sweep (1.0); the rig's printed
parts, fits and bought parts (`rig.scad`; the linear guide's sizes and ratings are its vendor's page). None has been
measured yet; each one the real trial depends on is measured before the trials (Real trials, checks 1 to 8).

## The model

- The lancet is a loose tool of two links: the housing and its contact shield, which stands 3 mm proud on a spring
  (4 N/mm, 1 N preload). It fires at 8 N on the shield, 1.75 mm of travel, and only once its tab is off and it is out
  of the stand. Firing is read from the shield, so it fires against anything pressed hard enough (skin, table, a
  gripper pad), as the real one does. After firing it is spent. The 4 g is split 2 g housing, 2 g shield [CHOICE: a
  0.5 g shield sank into the fingertip's collider in the sim instead of compressing].
- The tab: the stand's socket is keyed to the lancet's front and turns on a spring, up to 100 deg either way; the tab
  breaks at 45 deg of turn either way, where the spring's torque is 3 N-cm. WHY either way: BD's step 6 figure shows
  the twist with no direction (`test_tab_breaks_either_way`, stem c00). In the real stand a fixed post holds the tab
  cap and a cup keyed to the housing turns on it, so the tab's own plastic gives the torque. BD says to discard the
  tab; here it stays in the post's slot and is taken out at reset, so it is not a step the robot does. Lifted out more
  than 10 mm before the break, the lancet still has its tab on: a fault.
- The trainer: a base (a 3 kg plate taped down, a palm block, an index finger on a rod into the palm, a thumb) and the
  ring finger (25 g: the print, the guide's block and its screws) on a vertical prismatic joint at its knuckle, 99 mm
  behind its apex. A spring (100 N/m) holds it up on its top stop with 1 N to spare over its weight; it stops 10 mm
  down. The middle and little fingers are left off: a gripper closed round the ring finger's base needs its pads 7.8
  to 14.2 mm either side of the finger's axis, where the neighbours' surfaces lie (12.2 and 12.7 mm off it). Practice
  on a lone finger is what the commercial trainer the rig uses already is: Wallcur's Lance "fits over the finger of a
  standard patient or Mannequin" (Sources). The slide lumps a real finger's give (its knuckle, wrist and arm) into one
  vertical give at the knuckle; the finger is rigid on it, so its tip gives what its knuckle gives. That a real finger
  pressed by a lancet and not held moves away is our inference, not measured and not quoted from a source; the
  sources say to grasp the finger (Akron) and show it held while the site is chosen (BD step 4).
- The site: the pad of the ring fingertip, 8 to 15 mm back from the apex and 15 to 45 deg round from the pad's middle
  line, on the side that faces up. The shield's face within 1.5 mm of the skin is on it. WHY the side facing up only:
  the YAM's gripper points down, so it can hold the lancet near vertical; the 30 deg roll turns that side's normal to
  within 15 deg of vertical, where a downward lancet can meet it square, and turns the other side's 45 to 75 deg from
  up, where it cannot (`test_the_side_facing_down`). From 8 mm, past the pad's 7.3 mm radius, the skin's normal has no
  lean along the finger. The demo aims 10 mm back from the ring's apex: at 11.5 mm two corners of the layout
  put the arms under 10 mm apart.
- Square: the lancet's axis within 10 deg of the skin's inward normal where the shield's face touches (`tilt_of` with
  `site_of`'s normal), at each allowed site, not against the vertical.
- The press: from first touch (the shield pushed off its stop, 1.2 N) to firing, the face moving more than 1.0 mm on
  the skin is a slide. Lifting off (the shield back on its stop) ends a placement, so the lancet may be lifted and
  placed again.
- Held: a gripper that is not holding the lancet, closed on the ring finger as it is with both pads on it: its
  half-gap 5.7 to 8.7 mm and its TCP within 15 mm of the ring finger's axis between its distal joint (26.6 mm back
  from the apex) and its base (99.0 mm back), at most 4 mm off the axis along the gripper's closing direction, and its
  pad tips (13 mm under the TCP) below the axis, in the ring finger's own frame. The half-gap window is 1.5 mm either
  side of 7.2 mm, the reading planned for a gripper closed on the 7.8 mm finger [EST: the lancet's grip read 0.2 to
  1.1 mm under the housing's half-thickness; checked against the film's trace, Sim results]. A gripper closed on air
  4 mm beside the finger, or with its tips above the axis, is not holding it (`test_finger_held_rule`). The hold must
  be there at every step from the lancet's first touch until 0.5 s after the click: a step without it is a fault ("not
  held from first touch to 0.5 s after the click"), and a fire with no such gripper is a fault.
- The model reads the hold and the finger at 20 Hz, the control rate, so a jab between two steps is not seen by the
  drop check. It does not need to be: the hold must be contact-like and present at every step of the window, so a jab
  on an unheld finger scores 0 however it is timed, and only a gripper really closed on the finger can score.
- Still: from the lancet's first touch (the shield off its stop, tab off, out of the stand) until 0.5 s after the
  click, the ring finger pushed more than 2 mm down is a fault, whatever the force at the time. The drop is read at
  the slide and at a fingertip mark 11.5 mm back from the apex on its axis, against the base, and the larger counts;
  the real trial reads the same two points. WHY past the click: a jab fast enough to fire on the finger's inertia
  leaves the finger moving down; from 1 m/s the net force on it, the 1 N preload plus 100 N/m times its drop x, stops
  25 g only after 8.7 mm (0.5 x 0.025 x 1^2 = 1.0 x + 50 x^2), past 2 mm [ARITH, 1-D; the 1 m/s is EST]. Slow presses
  are the sweep's. A finger pinned on its lower stop is 10 mm down, so pressing it there is a fault too
  (`test_ring_pinned_at_its_lower_stop`).
- The yield sweep (`tools/yieldsweep.py`): a press is allowed when the shield's face is on the site and the lancet's
  axis d is within 10 deg of the skin's inward normal n. The shield's spring carries the axial load, so the contact
  force C on the skin has C . d = F, the force on the shield. Friction (coefficient up to mu) lets C lean up to atan
  mu off d (the shield's flat face on the skin) or off n (its rim on the skin), so C = F c / (c . d) for such a
  direction c, and its downward part, which the slide feels, is F k with k = -c_z / (c . d). Over the site (every 0.5
  mm and 2.5 deg), 73 lancet directions and 146 contact directions each, the least k [RUN]:

  | friction mu | least k | gives way (2 mm) at, N on the shield | margin at the 8 N fire |
  |---|---|---|---|
  | 0 | 0.9063 | 1.32 | +6.05 N |
  | 0.25 | 0.8007 | 1.50 | +5.21 N |
  | 0.5 | 0.6950 | 1.73 | +4.36 N |
  | 1.0 | 0.4837 | 2.48 | +2.67 N |

  The worst site is 8.0 mm back, 15 deg round (the site's edge nearest the pad's middle). The test asks for a margin of
  at least 1.0 N at every mu up to 1.0 (`test_unheld_finger_gives_before_fire`), and shows the check can fail: a 5 N
  preload would not give way under some allowed presses (`test_the_sweep_can_fail`). With the 15 deg square rule of
  the earlier revision the least k at mu 1.0 would be 0.3660. On the side facing down the least k is -0.909: friction
  can lift the finger there.
- The demo's hold: the left closes on the ring finger 91.5 mm back from its apex, 81.5 mm behind the press, and aims 25
  mm over the finger, lifting it against its top stop with about 12.5 N [EST: the arm's 0.5 N/mm at the TCP], so the
  finger does not move until a press passes 13.5 N. The hold pushes the finger up, against the press: that is what
  holds it still. A grip on the finger that does not lift it (stem g00) leaves the press to meet the slide's spring
  and the arm's stiffness; its offline replay scores 0, not held still (`test_sim_stems`). A jab past the hold
  (stem y00, 40 mm under contact in one step) also scores 0, not held still.
- Out of the stand: the lancet's frame more than 4 mm across or 10 mm up from where it stood. This latches: a lancet
  put back in the stand still counts as out.
- Fired in the stand: with its tab off and still in the stand, the lancet pressed down to 8 N on its shield fires into
  the stand's post, as the real one would. It is then spent, and the trial scores 0.
- Held (the lancet): a gripper's finger half-gap 3.0 to 7.0 mm, from 2 mm inside the housing's narrower side (10 mm)
  to 0.5 mm outside its wider side (13 mm), and its TCP within 15 mm of the lancet's axis, anywhere from 10 mm beyond
  its face to 53 mm up it (`test_slipped_or_deep_grasp_in_the_container_is_still_held`); a gripper closed on nothing
  does not hold it.
- Let go: a gripper's half-gap over 7.0 mm, or its TCP more than 60 mm from the lancet's axis. The 0.8 and 1.0 rungs
  need both grippers to have let go.
- Dropped: the lancet out of the stand, held by neither gripper and not in the container: loose for 0.5 s.
- The trainer's base may turn or tip 2 deg and move 5 mm; more is a fault (dragged, lifted or pushed down, turned or
  tipped).

Two arms on one finger. Two top-down YAM grippers working about 80 mm apart on one finger is near the geometric limit:
along x their link4s meet under about 105 mm. Turning the trainer 30 deg (its fingers point toward the wrist 30 deg
right of +x) gives the most room. `tools/grippers.py` poses each arm's collision meshes (gripper, both tips, link5,
link4 of the YAM URDF) by forward kinematics along both plans, every 0.1 s, and reports the least distance between the
arms and from each arm to the trainer's fixed parts. Over seeds 0, 2 and 4, the layout's eight corners and the two-arm
stems (f00, k00, c00, i00, a00, g00, y00), the arms stay at least 10.5 mm apart (corner -12, -12 mm, +6 deg, through the
press) and each arm at least 3.3 mm from the trainer's fixed parts [RUN: tools/grippers.py, 2026-11-19] (floors 10 and 3
mm [CHOICE]; in i00 the left's grip on the index is the stem, so the index is left out of that check). The arms' lag on
a descent (2 to 10 mm) is not in that check. To keep the arms apart the left takes the finger while the right waits
clear at (0.10, -0.07). After the press the right backs out to that same spot, and only once the left has let go and
parked does the right go to the container: at the hold the two wrists cross in y, so either arm leaving first along
another path met the other.

## Seeds

Each trial draws the trainer's layout from `random.Random(seed)`, the seed inspect-robots passes to the reset (the
CRC-32 of "<--seed>:None:0", its eval seed, scene seed and epoch): the base shifted in x and in y, each uniform within
+-12 mm, and turned uniform within +-6 deg about a point 130 mm behind the ring's apex along the hand (the palm's
middle). Before its shift the ring's apex is at (0.19, -0.005). The bounds are where both arms' reach
(`tools/ikcheck.py`), the sharps clearance (`tools/clearance.py`) and the arms' clearance (`tools/grippers.py`) pass at
all 8 corners of the box; the demo replay scores 1.0 at all 8 corners, on `--seed` 0 to 19 and on every seed below
(tests). Some seeds:

| `--seed` | shift x, y, mm | turn, deg | |
|---|---|---|---|
| 0 | -7.0, 0.1 | -2.71 | the filmed demo and the stems (Sim results); replayed offline |
| 2 | -9.7, -5.5 | -4.11 | replayed offline |
| 4 | -0.2, -2.2 | 2.67 | replayed offline |
| 26385 | -11.9, -11.7 | -5.98 | corner -x, -y, -turn; PhysX demo with a 0.05 s start delay |
| 84277 | -11.6, -11.9 | 5.93 | corner -x, -y, +turn; PhysX demo |
| 5410 | -12.0, 11.9 | -5.90 | corner -x, +y, -turn; PhysX demo |
| 55059 | -11.6, 11.6 | 5.85 | corner -x, +y, +turn; PhysX demo |
| 126748 | 11.8, -11.8 | -5.96 | corner +x, -y, -turn; PhysX demo |
| 5673 | 11.7, -11.7 | 5.83 | corner +x, -y, +turn; PhysX demo |
| 58369 | 11.7, 11.7 | -5.81 | corner +x, +y, -turn; PhysX demo |
| 100825 | 11.5, 11.7 | 5.98 | corner +x, +y, +turn; PhysX demo |

The scripted demo reads the seed's layout and plans from it, as a privileged script may. It shows the task can be done.

## Real trials

Never on a person. The trial uses the trainer only, with nobody in the arms' reach while they move.

Parts and instruments (the printed parts are in `rig.scad`, which `tools/rig_scad.py` writes from the task's own
numbers; `test_rig_scad_is_current` checks it is current):

- BD Microtainer contact-activated lancets 366577 (21 G x 1.8 mm), one per trial; a BD 305635 sharps collector.
- Wallcur Lance fingerstick trainer fingers (REM-LANCE-KIT-01D), one fitted over the printed ring finger, a fresh one
  each trial, taped round its base so it cannot turn or slide. The Lance finger takes the puncture; check 6b shows
  where the blade stops. The ring's distal pad is printed thinner by the Lance finger's wall (`lance_wall`, check 3b).
- The trainer, two prints and bought parts (`rig.scad`; units mm, z over the table, x along the finger from the ring's
  apex):
  - Part `base`, one print: a plate 104 by 95 by 5 mm (x 89 to 193), ballasted to 3 kg and taped down; on it the palm
    block on a pedestal, the index finger on a rod into the palm block, the thumb, the slide's post, the spring pins'
    bosses and the lower stop post. The part is 191 by 120 by 96 mm, so it needs a bed of at least 201 by 130 mm and
    101 mm of height (from the model's shapes; not rendered).
  - The post: x 107 to 115, 24 mm across, z 5 to 51, with three M2 heat-set inserts in its -x face at z 16, 31 and 46
    for the rail's screws. The lower stop post: x 93 to 99, 6 mm across, its top at z 18.5.
  - Part `ring`: the ring finger (distal radius 7.3 mm less the Lance wall, proximal 7.8 mm, axis 70 mm over the table)
    and under its knuckle a web x 93 to 99, fused to it, 17 mm wide up to z 56 and 10 mm wide above (2.3 mm inside the
    holding pads), down to a foot z 28.5 to 33.5 that spans the two spring pins. The guide's block is bolted to the
    web's +x face by four M2 screws (holes 12 by 8 mm apart, at z 35.75 and 43.75). The foot lands on the stop post
    after 10 mm: the lower stop.
  - The slide, bought: a miniature linear guide, LILY MGN7C1R40Z0CM, listed as interchangeable with HIWIN's MGN7C
    [SOURCE: LILY Bearing's product page, read 2026-11-19]: rail 40 mm long, 7 wide, 4.8 high, holes for M2 x 6 at a 15
    mm pitch, 5 mm from each end; block 22.5 long, 17 wide, 8 mm high over the rail's base, M2 holes 2.5 mm deep, 12 by
    8 mm apart; very light preload (Z0); basic static load 1.24 kN; static moments MR 4.7, MP 2.84, MY 2.84 N m; block 8
    g. The block's screws are M2 x 6 in 2.2 mm counterbores in the 6 mm web, so 2.2 mm of each enters the block's 2.5 mm
    thread; the rail's are M2 x 6 into the post's inserts. The rail runs z 11 to 51 on the post; the block, z 28.5 to 51
    at rest, stays on it over the 10 mm travel. The press at the demo's site is 93 mm ahead of the block's middle, so a
    15 N press (check H's) puts 1.40 N m of pitch moment on it, under MP's 2.84 N m [ARITH]. WHY a rolling guide: plain
    pins in printed holes jam when the load is more than L / (2 mu) ahead of them (16 mm for 8 mm of engagement at mu
    0.25), and the press is 88 to 95 mm ahead; a ball block has no such wedge [EST: its friction is not modelled; check
    R measures the give with it]. WHY not LM4UU bushings: THK's LM catalog says "When using the Linear Bushing on a
    single shaft, use two or more units" to avoid a moment load, and a guide carries the pitch moment in one part.
  - Bought, for the springs: two 4 x 40 mm steel dowel pins pressed 8 mm into the plate and their bosses at x 96,
    15 mm either side of the axis, z 0 to 40; two 4 mm bore shaft collars (10 mm OD, 5 mm wide [EST]) clamped on the
    pins on the foot's top, z 33.5 to 38.5: the top stop. Two compression springs, one on each pin between its seat
    and the foot, each music wire 0.32 mm on a 4.8 mm mean diameter (OD 5.12, ID 4.48), 18.8 active coils and 20.8 in
    all, ends closed and ground, 0.05 N/mm, free length 31.0 mm, 18.5 mm long at rest (0.623 N each, 1.25 N in all:
    the 1 N preload plus the finger's weight), 8.5 mm at the lower stop, solid 6.66 mm, peak stress 458 MPa [ARITH:
    `tools/rig_scad.py` from k = G d^4 / (8 D^3 n) with G 79.3 GPa, a handbook value, EST]. The spring index is 15,
    above the 4 to 12 a maker prefers [EST]; the pin inside keeps it from buckling. This is a full order spec for a
    custom spring; a catalog spring within 10% of rate and free length passes if check R passes. The springs sit at
    z 10 on two printed 1 mm shims each (part `shims`); a shim taken out lowers each spring's force by 0.05 N.
- The stand (parts `stand` and `cup`), fixed to the table:

  | part | size, mm |
  |---|---|
  | block | 40 by 40, 25 high, with a recess 24.4 mm across and 6 deep for the cup |
  | post | 10 diameter, fixed in the block's middle, its top 27 over the table, where the shield's face rests; a slot in its top cut to the tab cap's measured size (`tab_w`, `tab_t`, `tab_h`, check 3) holds the cap still |
  | cup | 24 diameter, turning on the post (10.4 bore), from z 19 to its rim at z 35; its pocket keyed to the housing, 13.8 by 10.8, 8 deep, 0.4 each side, as the model's socket |
  | stop | a 3 mm pin in the recess's floor, 9 from the axis, in an arc slot under the cup that stops it at +-100 deg, the model's TAB_STOP |

  Turning the housing turns the cup while the post holds the tab still, so the tab's own plastic gives the twist's
  resistance.
- A kitchen scale (5 kg, 1 g readability) under the trainer's base, its display in the side camera's view; a push-pull
  force gauge (0.1 N readability) with a flat tip; a cap torque gauge (0.1 N-cm); digital calipers (0.01 mm); a
  digital angle gauge (0.1 deg); a 0.01 g scale; a protractor; a 10x loupe; washable ink and a paper template of the
  site zone (1 mm and 5 deg lines); two steel rules (0.5 mm divisions) fixed upright to the base, one beside the web,
  which carries a scribed line, and one beside the fingertip, where an ink mark on the Lance finger's side at its axis
  sits 11.5 mm back from the apex (the model's TIP_MARK); a side camera (60 fps) on both rules and marks and the
  scale's display; a second side camera at 90 deg to it; an overhead camera; a microphone on the same recording.

The rig file has not been rendered or printed: OpenSCAD is not installed where it was written, and the offline test
checks only that the file is current, its brackets balance and its numbers are the model's. A render would add that
each part is one closed solid a slicer accepts and that the web and the block clear the post, the rail and the palm
over their travel.

Checks before any trial, each with its pass criterion (no trial runs until all pass):

1. Fire force: five lancets pressed by hand, shield first, onto the gauge's tip through a 3 mm silicone pad; the peak at
   the click. Pass: every one at or under 15 N, and the lowest at least 4.6 N: at that force every allowed press pushes
   the unheld finger down by 1.0 N more than its 1.20 N give force (the sweep's least k at friction 1.0, 0.4837, times
   4.6 N, less 1.20 N). If the lowest is under 4.6 N, shims are taken out from under the springs to lower the preload
   (0.1 N a mm), check R is re-run, the sim is set to the readings (`F_FIRE` the mean, `RING_PRELOAD` check R's) and the
   offline tests are re-run; `test_unheld_finger_gives_before_fire` fails if the margin no longer holds.
2. Tab: ten tabs twisted off in the stand by the torque gauge on the housing, five each way. Pass: every peak at or
   under 4.9 N-cm (Facet's limit) and every break at or under 60 deg (protractor), under the demo's 70 deg twist.
3. Body size and tab cap: five lancets with calipers. Pass: each side within 1 mm of the sim's 13 x 10 x 37 mm, or the
   sim is updated and its tests re-run. The tab cap's width, thickness and height set `tab_w`, `tab_t` and `tab_h`.
3b. Ring finger: the Lance finger's wall, read with calipers on a cut spare, sets `lance_wall` before the ring is
   printed. After printing, the ring fingertip's radius with the Lance finger on is read with calipers. Pass: within
   0.3 mm of the sim's 7.3 mm; else the ring is reprinted. The ring with the block bolted on weighs 25 g plus or minus
   5 g (0.01 g scale). The Lance finger's length is read with the rule. If it reaches past the distal joint's line
   (26.6 mm from the apex), it is trimmed at the line, so the proximal segment, where the hold closes, keeps its
   7.8 mm radius and the held window its reading; uncut, its wall would add about 1 mm to the half-gap the gripper
   reads there [EST: the 1.0 mm wall], inside the 1.5 mm window but at its edge.
R. Slide: the gauge straight down on the ring's top 10 mm from the apex (the demo's site), read at both points: the
   web's line and the fingertip mark on their rules. Pass, five times: the force at 2 mm of drop at the fingertip is
   1.0 to 1.4 N (the sim's 1.20 N plus or minus 0.2 N); the two drops agree within 0.5 mm at 2 mm (the block does not
   pitch); and the finger returns to within 0.25 mm of its top stop at both points when let go. At 1.4 N every allowed
   press still gives the finger way by 2.47 N at 8 N (0.4837 x 8 - 1.4). If it fails, in order, each followed by check
   R again: over 1.4 N, a shim out under each spring (0.1 N less); no return, the block wiped and re-oiled; still no
   return, or the drops differ by more than 0.5 mm, the block replaced on the same rail; if it still fails, no trial
   runs.
H. Hold: the left arm runs the demo's hold on the ring finger; the gauge pressed straight down at the demo's site to
   15 N. Pass: the web's line and the fingertip mark each move under 1 mm (rules). The left's gripper reading,
   closed on the finger, is logged here: it is the setup reading the trial's held check uses.
5. Square: a 2 mm rod held square to the skin at the demo's site, 10 mm from the apex on the side facing up, is
   within 3 deg of vertical (angle gauge).
6. Patch: the site zone of a spare Lance finger, on the ring, is coated with washable ink and left to dry. Five
   lancets are fired by hand, square and without sliding, the finger held, each on a fresh inked spot; where the
   shield's face pressed, the ink lifts and leaves a bare patch. Pass: a patch is seen under the loupe every time. Its
   longest length (calipers), largest of the five, is L6, the no-slide patch.
6b. Blade: a spare Lance finger over a dowel of the ring fingertip's radius wrapped in carbon paper on white paper;
   five lancets fired by hand through it. Pass: no mark on the paper under the loupe every time. This is expected to
   fail on the estimates (the blade's 1.8 mm against the Lance wall's 1.0 mm [EST]). If it fails, the ring's pad is
   printed thinner by a further 1.0 mm and wrapped in 1.0 mm self-adhesive silicone tape [CHOICE] under the Lance
   finger, so the radius stays the model's and the blade ends in the tape; then check 6b is re-run, then check 3b.
6c. Slide: five lancets fired by hand on fresh inked spots, each slid 2 mm along the finger on the skin before the
   click, the slide set by a stop on a guide. Pass: every patch is at least L6 + 1.0 mm long.
7. Grip: with the pad covers on, the lancet held in the gripper at the demo's target and pressed on the gauge to 15 N
   does not slip in the grip (no change in the camera over 2 s).
8. Covers: five lancets fired by hand, shield on a spare pad cover laid over carbon paper on white card. Pass: no
   mark on the card under the loupe every time.

Scoring a trial, each threshold with its instrument (before the trial the site zone of the Lance finger is inked as in
check 6, and the scale is tared once the trainer is on it):

- Tab off before the lift: the overhead video shows the lancet turned in the cup before it rises 10 mm (the mm scale
  on the stand), and the tab is left in the post's slot. A click while the lancet is still in the cup scores 0 (fired
  in the stand).
- Fired: the click on the audio track. Spent: after the trial the lancet is pressed on the gauge to 15 N with no second
  click (BD: auto-disabled).
- Held: in every frame from the touch (below) until 0.5 s after the click, the overhead and side videos show a
  gripper closed on the ring finger with both pads on it (no light between pad and finger in the side view), its pad
  tips below the finger's axis line (side view), between the distal joint's line (inked 26.6 mm from the apex) and
  the knuckle's line (inked at 99 mm); and its logged gripper reading is within 1.5 mm of that gripper's reading
  closed on the finger at setup. These are the sim's held rule (The model, Held) in what the cameras and the arm's
  log show. A frame without such a gripper scores 0.
- Touch (the 0.4 rung): the scale's reading, with the display filmed. It is tared again once the holding gripper has
  closed on the ring finger and lifted it to its stop, and before the lancet touches: the hold's pull is then constant,
  because a held finger stays on its stop, so the change is the press's downward part. The touch is a change of 56 g
  (0.55 N) or more, read to 1 g (0.01 N). The sim's touch is contact, the shield pushed off its stop (1.2 N on its
  axis); the scale reads the downward part, 0.4837 to 1.5099 times the shield's force over the allowed presses with
  friction up to 1.0 [RUN: `tools/yieldsweep.py`], so every touch the sim counts reads at least 0.4837 x 1.2 = 0.58 N
  on the scale, over 0.55 N [ARITH]. The bound is one-way: a reading of 0.55 N may be a touch as light as
  0.55 / 1.5099 = 0.36 N on the shield, so a real trial can count a lighter touch than the sim does, and it does not
  miss a touch the sim counts.
- Still: from the side camera at 60 fps, the web's line and the fingertip mark each stay within 2 mm of their marks at
  rest on their rules (read to 0.25 mm [EST]), from the first frame the scale's change reaches 56 g until 0.5 s after
  the click (the audio on the same recording).
- The patch scored: where the lancet was placed more than once, the inked finger shows a patch for each placement;
  the one scored is the patch under the press that gave the click, picked on the overhead video.
- Site: that patch's centre lies inside the zone on the template laid on the ring fingertip (8 to 15 mm back from the
  apex, 15 to 45 deg round from the pad's middle line on the side that faces up), checked with the loupe. The template
  reads its s (mm back from the apex) and its theta (deg round from the pad's middle line, toward the side facing up;
  a patch on the other side is off the site).
- Slide: the patch's longest length is at most L6 plus 1.0 mm (calipers).
- Square, against the skin's local normal at the patch: from the two side cameras, set to the trial's turn of the base
  (from its seed) with a protractor against the base plate's edge, one looking across the finger and one along it: at
  the click, the housing top's lean a toward the apex (the across view) and m toward the pad's middle line (the along
  view), protractor overlay, each signed (negative away). The skin's normal at the patch leans b = asin((r - s) / r)
  toward the apex where s is under the fingertip's radius r (check 3b; 0 farther back, so 0 over the whole site), and
  g = 30 - theta deg toward the pad's middle line, signed: negative for theta over 30 deg. Then n = (tan b / cos g,
  tan g, 1), d = (tan a, tan m, 1), and the lancet's angle off the normal is acos(n . d / (|n| |d|)). Pass: at most
  10 deg. These are the scorer's own quantities: `test_real_square_check_matches_the_scorer` checks this reading
  against `tilt_of` at 400 random sites and leans, and at both ends of theta with leans of both signs.
- In the container: the container weighed before and after on the 0.01 g scale gains the lancet's mass (weighed in
  check 3) within 0.1 g; the video shows each gripper open (fingers more than 14 mm apart) or its TCP more than 60 mm
  from the lancet; no lancet loose elsewhere for 0.5 s.
- The base: within 5 mm of its taped outline on the scale's platform, and turned or tipped under 2 deg (angle gauge on
  its plate before and after).

Rig protection and sharps handling (for the trial, not part of the task):

- The blade's exposure: BD's 510(k) says "after skin puncture, the needle/blade returns into the device" and "Once
  activated, the device is auto-disabled". So the blade is out for one stroke of at most 1.8 mm past the shield's face,
  only where the face is pressed at the fire force, and never again.
- Gripper pads, both arms: each finger of both grippers is capped over its tip and outer face with a 3 mm
  polycarbonate shell [CHOICE]; the grip faces carry a 3 mm polycarbonate plate under thin grip tape. Foam is not used:
  it compresses under the press, and the blade would reach the pad. Check 8 proves it. Check 7 is run with the covers
  on.
- Both arms and every rig part: a policy is not the demo, so the lancet may be pressed on anything under it. The table
  under the work area is covered with a 3 mm polycarbonate sheet. On both arms, each gripper housing and the last two
  links, and every exposed cable within 300 mm of the work area, is wrapped in 3 mm polycarbonate sheet or split loom
  [CHOICE]. The printed parts are plastic and cheap to reprint; the cameras and the microphone stand outside the arms'
  reach.
- Sharps: the operator wears puncture-resistant gloves and loads a new lancet (tab on, sealed) into the cup by hand.
  A used lancet is never touched by hand: one that was dropped is picked up with forceps into the container. The tab
  is taken out of the post's slot with tweezers. The container is closed and replaced at its fill line. Handling
  follows the BD CAUTION above.
- E-stop: the operator holds it; the arms are stopped before anyone reaches into the cell.

## Run

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=lancet_fingerstick.py -E object_jitter=0 \
  -E control_hz=20 --instruction "$(python3 -c "import json;print(json.load(open('task.json'))['instruction'])")" \
  --scorer task_score --seed 0 --max-steps 1200
```

The run limit is 1,200 steps (60 s). The demo's plans take 37.7 s (right) and 32.8 s (left); the slowest stem,
k00, 41.1 s. Every run below uses the same 1,200 steps.

## Sim results

The trainer, the site, the hold rule and the placed rung changed in this revision, so every PhysX run of the earlier
files no longer applies and none is quoted. The runs below are planned on the files here; each result is filled in
from its run's log once it has run.

yam_arena, `--seed 0` (the first row under Seeds), `object_jitter=0`, 20 Hz, 1,200-step limit. FILM_PENDING
DREAM_PENDING Both runs hash `lancet_fingerstick.py`, `mechanism.py` and `task.json` before starting (`sim/film.sh`,
`sim/dream.sh`), to be checked against the files here (`task.json` as it was before its `scores` line was filled in).
Expected for the film: 1.0 inside the 1,200 steps; the left's half-gap (`lf` in the trace) inside 5.7 to 8.7 mm from
the touch until 0.5 s after the click; and the base's height (`Hz` in the trace) not rising while the left lifts. If
the left's reading on the finger is not 7.2 mm within 0.5 mm, `HOLD_GAP_ON` is set to the trace's reading and the
tests re-run; if the base rises, the hold lifts the trainer and that is reported.

More PhysX runs of the scripted policy, each on a copy that `sim/mk.sh` makes of this `lancet_fingerstick.py` with a
per-step trace appended (`sim/log_snippet.py`, which writes `sim/out/trace_<copy>_<seed>.csv`), and for the stems
`sim/stems.py` too. Each stem's offline replay gives the expected score (`test_sim_stems`):

| Run | `--seed` | Copy | Expected | PhysX score, steps, fault |
|---|---|---|---|---|
| demo at the 7 other layout corners | 126748, 84277, 100825, 5410, 55059, 5673, 58369 | t00 | 1.0, success | STEMS_PENDING |
| demo with a 0.05 s start delay, corner -x, -y, -turn | 26385 | t05 | 1.0, success | |
| one arm: left parked, the demo's 1.5 s press | 0 | o00 | 0, not held still | |
| one arm: left parked, a 0.15 s jab | 0 | j00 | 0, not held still | |
| one arm: left parked, a jab 30 mm under contact in one control step (0.05 s) | 0 | x00 | 0, not held still (or no gripper holding the ring finger) | |
| one arm: left parked, a press 8.5 mm from the apex driven 10 deg toward the knuckle | 0 | w00 | 0, not held still | |
| both arms: the lancet pressed down in the stand after the twist | 0 | f00 | 0, fired in the stand | |
| both arms: after firing, the lancet held inside the container, never let go | 0 | k00 | 0.6 | |
| both arms: the tab twisted the other way (-70 deg) | 0 | c00 | 1.0, success | |
| both arms: the left closes on the index finger instead of the ring | 0 | i00 | 0, not held still (or no gripper holding the ring finger) | |
| both arms: the left closed on air 14.5 mm beside the ring finger, inside the earlier held rule, and a jab 30 mm under contact in one step | 0 | a00 | 0, not held still (or fired with no gripper holding the ring finger) | |
| both arms: the left closes on the ring finger but does not lift it | 0 | g00 | 0, not held still | |
| both arms: the demo's hold, and a jab 40 mm under contact in one step | 0 | y00 | 0, not held still | |

The stems' plans pass the same reach check as the demo's (`tools/ikcheck.py stems`). The offline replay is
quasi-static, so x00 replays as a slow press; whether a PhysX jab fires before the finger moves is what x00 shows, and
either way the window scores it 0. o00, j00 and w00 are expected to fail as "not held still", the finger giving way
before the fire; if PhysX instead gives "fired with no gripper holding the ring finger" (a fire before the finger
moves 2 mm), or for w00 "slid", the score is the same 0, and the fault is reported as found: it means the light
finger did not give way in time. g00 and y00 test the hold itself in PhysX: the offline replay holds the finger as a joint with the
arm's assumed stiffness, so whether PhysX lets the press through a grip that does not lift, or a 40 mm jab through the
demo's hold, is what those two runs show.

## Sim against real

- The needle, the skin's puncture and the blood are not simulated; the shield's spring stands in for the drive and
  fires at a set force. The real lancet's click and its spent state are read from audio and a re-press.
- A real finger yields at its joints, its wrist and its arm; the trainer lumps that into one vertical sprung slide at
  the ring finger's knuckle, in the sim and in the rig. That an unheld real finger withdraws under a lancet is our
  inference (The model, the trainer).
- The real slide is a rolling linear guide, its pitch moment rated 2.84 N m against 1.40 N m from a 15 N press 93 mm
  ahead [ARITH]; its friction is not modelled. Check R measures the give at the fingertip and at the web, friction and
  any pitch included.
- The palm block's pedestal and the slide's post are printed parts in the rig; in the sim the base is one rigid body
  and the post, rail, block and web are drawn only.
- The yield sweep is quasi-static and assumes skin friction of at most 1.0; a real Lance finger's silicone on the
  shield's polyethylene has not been measured (check R measures the give force, not the friction).
- The scale reads the press's downward part; the sim's touch threshold is on the shield's axis (Real trials, Touch).
- The real slide check reads a patch that also records motion after the click (check 6c); the sim stops counting at
  the fire.
- BD has the patient's hand held downward for blood flow; the trainer lies palm up so its fingertip is in reach of a
  top-down gripper, and the blood flow is out of scope.
- Out of scope, from BD's guide: warming the site, the alcohol wipe and air dry, wiping the first drop, collecting the
  specimen and pressing on the site after. Those need blood, a wipe or a tube on a fingertip the trainer cannot bleed
  from, and none of them is done with the lancet.

## Tests

`python3 test_lancet_fingerstick.py` (with yam_arena on PYTHONPATH) runs the offline tests: the shield and the site (the
demo's site faces up; the pad's middle, too far round, past the pad or short of 8 mm, the side facing down, the index),
the tab either way, a good press, no fire under 8 N, the placed rung needing the finger held, a fire with no hold, a
slide, a small creep and a fresh placement, a tilted lancet, the finger giving way, the jab window past the click and
from first touch, the held rule (index, palm, open, on nothing, past the distal joint), squareness against the local
normal at every site, fires off the hand and in the stand, a drop and the container, a slipped or deep grasp still held,
the yield sweep (and a design that fails it, and the side facing down), the scorer (a model that stops, the base
dragged, the ladder in order along the demo), both arms' demo replayed at the 8 corners of the layout box, on `--seed` 0
to 19 and on every seed in the table, the layout's draw, a plan replayed on another seed's layout failing, the base
clear of the stand and the container, the lazy routes below and the sim stems, the geometry, these numbers against the
code, the real trial's two-camera squareness reading against the scorer's tilt, and `rig.scad` current and at the
model's sizes. Added in this revision: the held rule against a gripper closed on air 4 and 4.5 mm beside the finger,
with its tips above the axis, behind the knuckle, and at stem a00's pose; the touch as contact; the hold needed at every
step from touch to 0.5 s after the click; the ring pinned on its lower stop; still read at the fingertip of a pitched
finger; the base lifted, turned or tipped. `tools/ikcheck.py` (no argument: the 8 corners, `--seed` 0, 2, 4 and the
stems) checks both arms' reach and joint6; `tools/clearance.py` every arm link against the sharps container;
`tools/grippers.py` the arms against each other and the trainer.

### Lazy routes

| Shortcut | Score | Why |
|---|---|---|
| Press without holding the ring finger | 0 | every allowed press gives the unheld finger way by 2.48 N or less (friction up to 1.0), before the 8 N fire; "the ring finger was not held still" |
| Jab so fast it fires before the finger moves | 0 | the finger still gives within 0.5 s of the click, and no gripper held it |
| Hold the index finger, the palm or the base instead | 0 | none holds the ring finger; "not held still" or "fired with no gripper holding the ring finger" |
| Close a gripper on air beside or over the ring finger | 0 | its gap, its offset along the closing direction or its tips above the axis say it is not on the finger; "not held still" or "fired with no gripper holding the ring finger" (stem a00) |
| Grip the ring finger, fire, then let go within 0.5 s of the click | 0 | "not held from first touch to 0.5 s after the click" |
| Grip the ring finger without lifting it | 0 | the press meets only the spring and the arm's stiffness; "not held still" (stem g00) |
| Push the ring finger down to its lower stop, then press | 0 | it is 10 mm down; "not held still" |
| Pull the lancet straight out, no twist | 0 | "lifted out of the stand with its tab still on" |
| Twist, then press the lancet down in the stand | 0 | "fired in the stand" |
| Press on the index fingertip | 0 | "not the side of the ring fingertip" |
| Press on the pad's middle line | 0 | "on the middle of the ring's pad, not its side", and 30 deg off square |
| Press on the side of the pad that faces down | 0 | "not the side that faces up" |
| Press 30 mm back, past the pad | 0 | "not the fingertip" |
| Slide the lancet while pressing | 0 | "the press slid" |
| Drop the used lancet on the table | 0 | "lancet dropped outside the sharps container" |
| Fire, then keep holding it over the container | 0.6 | never let go into the container |
| Fire, then hold it inside the container, at any grasp | 0.6 | held: never let go |
| Do nothing | 0 | no rung |

## Closest tasks

- #2259 (merged): loading a lancet into a pen lancing device, its tab twisted off. That loads a home device and never
  punctures; here a clinical contact-activated lancet is fired on a fingertip held still by the other hand. The tab
  twist is shared and is only the 0.2 rung here. #2259's review flag ("Protect the gripper pads or use suitable blunt
  training lancets") is answered under Rig protection.
- #2599 (open): an intramuscular deltoid injection with a retractable safety syringe. That is an injection into a
  training arm: a needle pushed in and a plunger pressed to deliver a dose. Here nothing is delivered: a capillary
  lancet fires itself on contact, on a fingertip the other hand must hold still, and the site is a fingertip's side.
- #1837 (merged): clipping a pulse oximeter on a trainer's finger while steadying the hand. Its hand sizes are used
  here. It has no sharp, no firing force, and no site or squareness rule.
- #1846 (final): a pulse oximeter clip, a duplicate of #1837.
- #1941 (merged): changing the blood tube on a needle already in a training arm's vein. Venipuncture with the needle
  in place; nothing is fired and no site is chosen.
- #2422 (not accepted, open) and #2403 (open), from their titles only: a glucose meter's control check, which uses
  control solution rather than a puncture, and drawing a splinter out of a fingertip with tweezers.
- #2039 (final, duplicate of an earlier task): filling blood tubes from a needle holder in a training arm. Its review
  says "Blood collection is core work for medical assistants and phlebotomists". That is venipuncture; this is
  capillary puncture, a different device and site.
- #862 (final): vaccinating a training arm with an inert pen, rejected because "The pen has no needle, activation
  mechanism or dose delivery". Here the lancet is a named product whose activation is modelled from its shield, and
  firing is read from the lancet itself.
- #2049, #2082, #1861 (merged): putting used needles in sharps. Here the discard is the last step, not the skill.
- #612 (final, blood tubes in order of draw) and #2242 (open, scanning a wristband): other clinical steps, no sharps.

Category: capillary puncture by finger is the O*NET Core task of Phlebotomists (143,900): "Draw blood from capillaries
by dermal puncture, such as heel or finger stick methods." Both national medical assistant exams test it: the CMA
(AAMA) outline lists "Capillary/Dermal puncture (finger, heel, ear)" and NCCT's plan "Perform capillary punctures (i.e.,
fingerstick, heelstick)." It falls under the Core tasks of Medical Assistants (833,900), "Collect blood, tissue, or
other laboratory specimens, log the specimens, and prepare them for testing.", and of Licensed Practical and Licensed
Vocational Nurses (666,900), "Collect samples, such as blood, urine, or sputum from patients, and perform routine
laboratory tests on samples." The accepted #1951 rests on one Medical Assistants Core task, and #2049
on tasks of three occupations; this follows #2049.

## Third-party code

`mechanism.py` is adapted from benchmark-tasks pull request #223 (MIT License; the licence is kept in its header).
