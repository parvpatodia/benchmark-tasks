# Fingerstick a trainer hand with a contact-activated lancet

A trainer hand lies palm up, hinged across its fingers and resting on a block under its wrist, rolled 30 deg about
its fingers so the little-finger side of the ring and middle fingertips faces up. A BD Microtainer contact-activated lancet (366577, 21 G x 1.8 mm) stands tip down in a
stand whose socket holds its protective tab. A BD 305635 sharps container stands beside. The robot does the fingerstick
as BD's guide gives it: twist the tab off, hold the hand still with one gripper, place the lancet square on the side of
the ring or middle fingertip that faces up with the other, press until it fires, and drop the used lancet into the
container.

The steadying hand is set by the model's physics, not by a rule. The hand turns on a hinge across its fingers, 62 mm
behind the ring's apex and at the height of the ring finger's axis, and rests on the wrist block by its own weight, a
moment of 0.137 N m about the hinge. A straight-down press on the site tips it, fingertips down, at 2.5 N (8 mm from
the ring's apex) to 2.9 N (15 mm back, nearer the hinge), or 2.2 to 2.5 N on the middle fingertip. The lancet fires at
8 N, and the placed rung needs 6 N. Tipping comes first for every press the scorer allows, not only a straight-down
one: `tools/tipsweep.py` sweeps the whole site, every lancet direction the square rule allows (15 deg round the skin's
normal) and the contact force's direction under friction up to 0.5, and the least-tipping of all those presses tips the
hand by 5.0 N (friction 0.5; 3.4 N without friction). At 6 N every one of them tips the hand by at least 0.028 N m more
than its weight holds, and at 8 N by at least 0.083 N m (`test_unsteadied_hand_tips_before_firm_and_fire`). The hinge
sits at the contact's height so that a push toward the wrist has no lever to hold the wrist down; under the earlier
low hinge it had up to 58 mm, and some presses near the apex then fired one-handed (`test_old_design_fails_the_sweep`).
Offline, in a quasi-static replay, a press without the other hand on the forearm stub scores 0 ("pressed with the hand
not steadied", `test_no_steadying_hand`). PhysX one-arm runs of the new design are listed under Sim results.

The hand's place and turn are drawn from the seed (Seeds), so a replayed path misses the site: a plan made for one
seed's layout, replayed on another's, failed in all 20 seed pairs tried, and the unshifted plan failed at all 8 corners
of the layout box (`test_open_loop_plan_fails_other_layouts`). Two seeds whose layouts lie within a few mm of each other
may share a plan; the test does not claim otherwise.

Files: `lancet_fingerstick.py` (the task), `mechanism.py` (articulated objects, adapted from benchmark-tasks #223,
MIT), `test_lancet_fingerstick.py` (offline tests), `task.json`, `rig.scad` (the printable real-trial rig: hand, base,
stand), `tools/` (task.json generator, reach and clearance checks, `tipsweep.py` the tipping sweep, `rig_scad.py`
that writes `rig.scad` from the task's numbers), `sim/` (run scripts and the sim stems).

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
  with the policies and procedures of your facility." Its figure shades both sides of the fingertip pad, and in the
  puncture step the collector's other hand holds the patient's finger.
- BD, FDA 510(k) K223243 (BD Microtainer Contact-Activated Lancet): "The BD Microtainer® Contact-Activated Lancet is
  activated by applying pressure against the puncture site upon contact; after skin puncture, the needle/blade returns
  into the device." and "Once activated, the device is auto-disabled". Its table gives the materials: needle carrier
  with protective tab, polystyrene; main housing, tab cap and rear cap, polypropylene; shield, polyethylene. It gives
  no activation force and no body size.
- FDA GUDID / openFDA: 366577 "LANCET CONT-ACT PINK 21G X 1.8MM US", "In Commercial Distribution", K223243. The older
  366593 is "Not in Commercial Distribution", so the task names 366577.
- BD patent US 9,005,233 ("Contact activated lancet device"): the shield pushed against the skin trips the drive
  spring; opposed sides carry "finger grip indentations". The patent allows the tab to break by twist or by pull; the
  task scores the twist BD's guide gives ("Twist off the tab"), and a pull that breaks the tab is not modelled (in the
  sim the tab stays on under a straight pull).
- Owen Mumford, FDA 510(k) K222111 (Unistik Touch contact-activated safety lancets), bench test: "The force to activate
  the safety feature: 4 - 15N".
- Facet Technologies, FDA 510(k) K221433 (Facet 28G Universal Lancet): "Cap Twist-off torque" "< 4.9 N-cm".
- BD sharps collectors catalog: 305635 "1 qt with needle ports", "4.9 x 4.3 x 6.5" (in, L x W x H); "2.5 × 1.2 open
  (305635)" (in).
- O*NET 31-9092.00 Medical Assistants: Core task "Collect blood, tissue, or other laboratory specimens, log the
  specimens, and prepare them for testing."; "Employment (2025)" "833,900 employees". O*NET's own definition of Core:
  relevance of at least 67% of incumbents and mean importance of at least 3.0.
- O*NET 29-2061.00 Licensed Practical and Licensed Vocational Nurses: Core task "Collect samples, such as blood, urine,
  or sputum from patients, and perform routine laboratory tests on samples."; "666,900 employees" (2025).
- O*NET 31-9097.00 Phlebotomists: Core task "Draw blood from capillaries by dermal puncture, such as heel or finger
  stick methods."; "143,900 employees" (2025).
- CMA (AAMA) certification exam content outline, Specimen Collection Techniques, Blood: "Capillary/Dermal puncture
  (finger, heel, ear)".
- McKesson item 1251387, Wallcur REM-LANCE-KIT-01D, "Blood Glucose Test Trainer Lance Finger": "Simulated finger stick
  and blood glucose trainer" and "Finger fits over the finger of a
  standard patient or Mannequin". It gives no size, mass or wall thickness.
- NCCT, National Certified Medical Assistant detailed test plan (effective January 2024, from a national job analysis):
  "Perform capillary punctures (i.e., fingerstick, heelstick)."
- benchmark-tasks #1837 (merged): the trainer hand's sizes, "ANSUR II median adult male (hand length 193 mm, breadth 88
  mm)", its finger radii and spacing. Its "0.300 kg (silicone skin over a foam core)" is not used: this task's hand is
  printed and ballasted to 0.200 kg.

Not sourced, and set in the code with [EST] or [CHOICE]: the lancet's body size (13 x 10 x 37 mm) and mass (4 g); the
fire force, 8 N, chosen inside Unistik's 4 to 15 N because BD publishes none; the placed rung's 6 N; the tab's break
torque, 3 N-cm, under Facet's 4.9 N-cm limit, and its break angle, 45 deg; the site's bounds, read from BD's shaded
figure and narrowed to 8 mm and more from the apex, on the side that faces up; the square, slide and steadied limits;
the hand's mass (0.200 kg), centre of mass, hinge and wrist block; the friction bound of the tipping sweep (0.5, the
sim's skin friction); the rig's printed parts and fits (`rig.scad`). None has been measured yet; each one the real
trial depends on is to be measured before the trials (Real trials, checks 1 to 4).

## The model

- The lancet is a loose tool of two links: the housing and its contact shield, which stands 3 mm proud on a spring
  (4 N/mm, 1 N preload). It fires at 8 N on the shield, 1.75 mm of travel, and only once its tab is off and it is out
  of the stand. Firing is read from the shield, so it fires against anything pressed hard enough (skin, table, a
  gripper pad), as the real one does. After firing it is spent. The 4 g is split 2 g housing, 2 g shield [CHOICE: a
  0.5 g shield sank into the fingertip's collider in the sim instead of compressing].
- The tab: the stand's socket is keyed to the lancet's front and turns on a spring; the tab breaks at 45 deg of turn,
  where the spring's torque is 3 N-cm. In the real stand a fixed post holds the tab cap and a cup keyed to the housing
  turns on it, so the tab's own plastic gives the torque (Real trials, the stand). BD says to discard the tab; here it
  stays in the stand's slot and is taken out at reset, so it is not a step the robot does. Lifted out
  more than 10 mm before the break, the lancet still has its tab on: a fault.
- The site: the pad of the middle or ring fingertip, 8 to 15 mm back from the apex and 15 to 45 deg round from the
  pad's middle line, on the side that faces up. The shield's face within 1.5 mm of the skin is on it. The roll of 30
  deg turns the little-finger side of each pad straight up, so a downward lancet meets it square. BD shades both sides
  of the pad; the task scores the one that faces up, because the roll turns the other 45 to 75 deg from up, where a
  square press is mostly sideways and some never tip the hand (`test_the_side_facing_down_would_not_tip`). From 8 mm,
  past both pads' radii (7.3 and 7.8 mm), the skin's normal has no lean along the finger. Square: the lancet's axis
  within 15 deg of the skin's normal. The demo aims 11.5 mm back from the ring's apex, the side facing up.
- The press: from first touch (2 N) to firing, the face moving more than 1.0 mm on the skin is a slide. Lifting off
  (under 2 N) ends a placement, so the lancet may be lifted and placed again.
- Steadied: while the lancet presses on anything, the wrist rises more than 2 mm off its block is a fault. The hand is
  rigid, 0.200 kg, its centre of mass 70 mm behind the hinge, which is 62 mm behind the ring's apex and 70.0 mm over
  the table, on the ring finger's axis; the forearm stub is 223 mm behind the apex. The demo's left hand grips the stub
  and aims 10 mm under it, about 5 N down [EST: the arm's 0.5 N/mm]; a 10 N press at the demo's site needs about 2.3
  N there. The hand's support is a hinge: a revolute joint across the fingers, its upper stop holding the wrist on its
  block. The hinge's cheeks and pin and the wrist block are drawn and do not collide; the joint carries the hand. The
  real rig uses the same hinge and stop (Real trials).
- The tipping sweep (`tools/tipsweep.py`): a press is allowed when the shield's face is on the site and the lancet's
  axis d is within 15 deg of the skin's inward normal n. The shield's spring carries the axial load, so the contact
  force C on the skin has C . d = F, the force on the shield. Friction (coefficient up to 0.5) lets C lean up to
  atan 0.5 = 26.6 deg off d (the shield's flat face on the skin) or off n (its rim on the skin), so C = F c / (c . d)
  for such a direction c. The hand tips when C's moment about the hinge passes the hand's 0.137 N m. Over the ring
  and middle pads (every 0.5 mm and 2.5 deg), 73 lancet directions and 146 contact directions each, the least-tipping
  press gives 0.0276 N m per newton on the shield, at the ring 15 mm back on the side 15 deg off up: so every allowed
  press tips the hand by 0.137 / 0.0276 = 5.0 N. Most of that is the sideways lean: C may lean up to 15 + 15 + 26.6 =
  56.6 deg from vertical across the finger, and 0.047 m x cos 56.6 / cos 26.6 = 0.029 N m per N; the rest is friction
  pushing toward the wrist at the contact, 7 mm over the hinge. Directions four times finer give the same value to 5
  places at the worst sites.
- Where the other hand holds: BD's figure shows the collector's other hand on the patient's finger. Two top-down
  grippers' fingers each reach 52 mm from their TCP's axis (the note at `SHARPS` in the code), so the two TCPs need more
  than about 104 mm between them. The ring finger ends at its knuckle 99 mm behind its apex, and the press is 11.5 mm
  behind it, so a gripper anywhere on the finger would meet the one pressing the lancet. On `--seed` 0 the demo keeps
  about 212 mm between the TCPs along x at the press, 167 mm at the closest (`tools/ikcheck.py cli=0`). The scored
  outcome is BD's: the hand does not move during the press. The task holds it at the forearm stub, 161 mm behind the
  hinge; any hold behind the hinge that keeps the wrist down scores the same.
- Out of the stand: the lancet's frame more than 4 mm across or 10 mm up from where it stood. This latches: a lancet
  put back in the stand still counts as out.
- Fired in the stand: with its tab off and still in the stand, the lancet pressed down to 8 N on its shield fires into
  the stand's post, as the real one would. It is then spent, and the trial scores 0.
- Held: a gripper's finger half-gap 3.0 to 7.0 mm, from 2 mm inside the housing's narrower side (10 mm) to 0.5 mm
  outside its wider side (13 mm), and its TCP within 15 mm of the lancet's axis, anywhere from 10 mm beyond its face to
  53 mm up it (the pad tips 13 mm under the TCP, at the lancet's rear). A grasp across either side counts, and so does
  one slipped along the housing or taken deeper or higher (`test_slipped_or_deep_grasp_in_the_container_is_still_held`);
  a gripper closed on nothing does not.
- Let go: a gripper's half-gap over 7.0 mm, or its TCP more than 60 mm from the lancet's axis. The 0.8 and 1.0 rungs
  need both grippers to have let go.
- Dropped: the lancet out of the stand, held by neither gripper and not in the container: loose for 0.5 s. Letting go
  of it back in the stand is a drop. Its fall into the container from the release height takes about 0.2 s.
- The hand's base may move 5 mm; more is a fault (dragged).

## Seeds

Each trial draws the hand's layout from `random.Random(seed)`, the seed inspect-robots passes to the reset (the CRC-32 of
"<--seed>:None:0", its eval seed, scene seed and epoch): the base shifted in x and in y, each uniform within +-12 mm,
and turned uniform within +-6 deg about a point 130 mm behind the ring's apex (the palm's middle). Before its shift the
ring's apex is at (0.11, -0.01). The bounds are where both arms' reach (`tools/ikcheck.py`) and the sharps clearance
(`tools/clearance.py`) pass at all 8 corners of the box; the demo replay scores 1.0 at all 8 corners, on `--seed` 0 to
19 and on every seed below (tests). Some seeds:

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

Never on a person. The trial uses the trainer hand only, with nobody in the arms' reach while they move.

Parts and instruments (the printed parts are in `rig.scad`, which `tools/rig_scad.py` writes from the task's own
numbers, so the print takes its sizes from the model; `test_rig_scad_is_current` checks it is current):

- BD Microtainer contact-activated lancets 366577 (21 G x 1.8 mm), one per trial; a BD 305635 sharps collector.
- Wallcur Lance fingerstick trainer fingers (REM-LANCE-KIT-01D, "3/PK"), one fitted over the trainer hand's ring
  finger and one over its middle finger, a fresh one each trial, each taped round its base to the printed finger so it
  cannot turn or slide. The Lance finger takes the puncture; check 6b shows where the blade stops.
- The trainer hand (`rig.scad`, parts `hand_front` and `hand_back`): the model's hand, printed rigid in PETG at its
  sizes and its 30 deg roll (fingers, palm, thumb and a 44 mm forearm stub, 272 mm from the middle fingertip to the
  stub's end). The ring and middle distal pads are printed thinner by the Lance finger's wall (`lance_wall`, measured in
  check 3b), so with the Lance fingers on their radii are the model's. It prints in two parts cut across the stub at 200
  mm, epoxied on two 3 mm alignment pins 14 mm either side of the stub's axis (clear of the ballast bore). No saved
  product page gives a life-size hand trainer's size and mass, so the hand is printed rather than bought; the model's
  hand is rigid, and so is the print. Print layout (a choice, not yet trialled): the hand parts on their palm side with
  tree supports under the fingers and thumb; the base on its plate and the stand on its bottom, no supports; the cup rim
  up.
- The hinge (`rig.scad`, part `base`, and the hand's keel): in the model the hand turns on a revolute joint across the
  fingers, and the rig uses one. A 3 mm steel pin, 87 mm long, runs across the fingers 62 mm behind the ring's apex
  and 70.0 mm over the table, on the ring finger's axis. It passes through a 14 mm thick keel web printed inside the
  fingers (5.3 mm of wall round its 3.4 mm hole) and through two cheeks on the base, 16 by 4 mm and 74 mm high, standing
  1 mm outside the little and index fingers. A 3 mm push-on retaining washer outside each cheek holds the pin. The wrist
  block, 50 by 40 mm, its top 40.5 mm over the table, centred 223 mm behind the apex and 13 mm to the thumb side of the
  ring's axis in plan, is the hinge's stop: the stub rests on it, 161 mm behind the pin. Both stand on a 231 by 60 by 5
  mm base plate (32 to 263 mm behind the apex), ballasted to 3 kg and taped to the table.
- Ballast: the model's hand is 0.200 kg with its centre of mass 70 mm behind the hinge, a moment of 0.137 N m about it,
  which sets the tipping forces. The hand is printed at an infill that keeps it, bore empty, under 0.200 kg and its
  tipping force under check 4's; lead shot then goes into the 16 mm bore in the stub's end until check 4 passes, and
  tape closes the bore. [EST, not yet printed: the bore holds about 80 g of shot, about 0.135 N m about the pin; a
  print that tips above check 4's force with the bore empty is reprinted at a lower infill.]
- The stand (`rig.scad`, parts `stand` and `cup`), fixed to the table: a 40 mm square block 25 mm high. A fixed 10 mm
  post in its middle holds the tab cap in a slot cut to the cap's measured size (`tab_w`, `tab_t`, `tab_h`, check 3);
  the post's top, 27 mm over the table, is where the shield's face rests, as in the model. A 24 mm cup turns on the
  post, sunk 6 mm into a recess, its pocket keyed to the housing (13.8 by 10.8 mm, 8 mm deep, 0.4 mm each side, as the
  model's socket) and its rim 35 mm over the table. Turning the housing turns the cup while the post holds the tab
  still, so the tab's own plastic gives the twist's resistance.
- A push-pull force gauge (0.1 N readability) with a flat tip; a cap torque gauge (0.1 N-cm readability); digital
  calipers (0.01 mm); a digital angle gauge (0.1 deg); a 0.01 g scale; a protractor; a 10x loupe; washable ink and a
  paper template of the site zone (1 mm and 5 deg lines); a side camera with a mm scale beside the wrist block; a
  second side camera at 90 deg to it; an overhead camera and a microphone.

The rig file has not been rendered or printed yet: OpenSCAD is not installed where it was written, and the offline
test checks only that the file is current, its brackets balance and its numbers are the model's. Rendering each part
(OpenSCAD, F6) would add what the test cannot: that each part is one closed solid a slicer accepts; that the keel's web
is whole round the pin's hole from cheek to cheek; that the cheeks miss the little and index fingers and the thumb
over the hand's whole tip (to 0.30 rad); that the alignment pins' holes miss the bore; each part's size against a
printer's bed (`hand_front` is about 210 mm long); and STL files a reader could print without OpenSCAD.

Checks before any trial, each with its pass criterion (no trial runs until all pass):

1. Fire force: five lancets pressed by hand, shield first, onto the gauge's tip through a 3 mm silicone pad; the peak
   at the click. Pass: every one at or under 15 N (the top of Unistik's 4 to 15 N), and the lowest at least 6.8 N: at
   that force every press the scorer allows tips the hand by 0.05 N m (the sweep's 0.0276 N m per N times 6.8 N, less
   the hand's 0.137 N m), so a one-handed press tips the hand before the lancet fires. If the lowest, F_min, is under
   6.8 N, the hand's moment is lowered to F_min x 0.0276 - 0.05 N m (shot taken out, or a lighter print), check 4's
   forces are scaled by the same ratio, and the sim is set to the readings (`F_FIRE` the mean, `HAND_KG` or `COM_U` to
   check 4's moment, `F_FIRM` 1 N under F_min), then the offline tests are re-run; `test_unsteadied_hand_tips_before_
   firm_and_fire` fails if the margins no longer hold. If no print can get that light, the trials are not run with that
   lancet.
2. Tab: five tabs twisted off in the stand by the torque gauge on the housing. Pass: every peak at or under 4.9 N-cm
   (Facet's limit) and every break at or under 60 deg (protractor), under the demo's 70 deg twist.
3. Body size and tab cap: five lancets with calipers. Pass: each side within 1 mm of the sim's 13 x 10 x 37 mm, or the
   sim is updated and its tests re-run. The tab cap's width, thickness and height set `tab_w`, `tab_t` and `tab_h`
   before the stand is printed.
3b. Hand: the Lance finger's wall, read with calipers on a cut spare, sets `lance_wall` before the hand is printed.
   After printing, the ring and middle fingertips' radii with the Lance fingers on are read with calipers. Pass: each
   within 0.3 mm of the sim's 7.3 and 7.8 mm; else the hand is reprinted, or the sim's distal radii in `FINGERS` are
   set to the measured ones and the offline tests re-run (the sweep among them). And the print, its bore empty, weighs
   under 0.200 kg (0.01 g scale).
4. Tipping force, two presses, each until the wrist lifts off its block (side camera): (a) the gauge straight down on
   the ring's top 11.5 mm from the apex, the demo's site; (b) the gauge along the skin's inward normal at the site's
   worst place, the ring 15 mm from the apex and 45 deg round (15 deg off vertical, set with the angle gauge). Pass:
   (a) 2.5 to 2.9 N (the sim's 2.7 N plus or minus 0.2 N) and (b) 2.8 to 3.2 N (the sim's 3.0 N plus or minus 0.2 N);
   shot is added to or taken from the stub's bore until both pass. Within these bounds the hand's moment is within 7%
   of the sim's, and every allowed press still tips the hand by at least 0.073 N m at 8 N (the sweep's numbers with the
   moment 7% higher).
5. Square: a 2 mm rod held square to the skin at the ring's site, 11.5 mm from the apex on the side facing up, is within
   3 deg of vertical (angle gauge).
6. Patch: the site zone of a spare Lance finger, on the hand, is coated with washable ink and left to dry. Five
   lancets are fired by hand, square and without sliding, each on a fresh inked spot; where the shield's face pressed,
   the ink lifts and leaves a bare patch. Pass: a patch is seen under the loupe every time. Its longest length
   (calipers), largest of the five, is L6, the no-slide patch.
6b. Blade: a spare Lance finger over a dowel of the ring fingertip's radius wrapped in carbon paper on white paper;
   five lancets fired by hand through it. Pass: no mark on the paper under the loupe every time. This is expected to
   fail on the estimates (the blade's 1.8 mm against the Lance wall's 1.0 mm [EST]). If it fails, the ring and middle
   pads are printed thinner by a further 1.0 mm and wrapped in 1.0 mm self-adhesive silicone tape [CHOICE] under the
   Lance finger, so the radius stays the model's and the blade ends in the tape; the dowel is wrapped the same way and
   check 6b is re-run until it passes, then check 3b.
6c. Slide: five lancets fired by hand on fresh inked spots, each slid 2 mm along the finger on the skin before the
   click, the slide set by a stop on a guide. Pass: every patch is at least L6 + 1.0 mm long, so the trial's slide
   rule can see a slide over 1 mm. The real patch also records any motion after the click, which the sim's slide (first
   touch to firing) does not count, so the real slide check is the stricter of the two.
7. Grip: with the pad covers on, the lancet held in the gripper at the demo's target and pressed on the gauge to 15 N
   does not slip in the grip (no change in the camera over 2 s).
8. Covers: five lancets fired by hand, shield on a spare pad cover laid over carbon paper on white card. Pass: no
   mark on the card under the loupe every time.

Scoring a trial, each threshold with its instrument (before the trial the site zone of each Lance finger is inked as
in check 6):

- Tab off before the lift: the overhead video shows the lancet turned in the cup before it rises 10 mm (the mm scale
  on the stand), and the tab is left in the post's slot. A click while the lancet is still in the cup scores 0 (fired
  in the stand).
- Fired: the click on the audio track. Spent: after the trial the lancet is pressed on the gauge to 15 N with no second
  click (BD: auto-disabled).
- The patch scored: where the lancet was placed more than once, the inked finger shows a patch for each placement;
  the one scored is the patch under the press that gave the click, picked on the overhead video.
- Site: that patch's centre lies inside the zone on the template laid on the ring or middle fingertip (8 to 15 mm back
  from the apex, 15 to 45 deg round from the pad's middle line on the side that faces up), checked with the loupe. The
  template reads its s (mm back from the apex) and its theta (deg round from the pad's middle line, toward the side
  facing up; a patch on the other side is off the site).
- Slide: the patch's longest length is at most L6 plus 1.0 mm (calipers). A press that slides drags the face over the
  ink, so the patch grows by the slide's length (check 6c).
- Square: from the two side cameras, set to the trial's turn of the base (from its seed) with a protractor against the
  base plate's edge, one looking across the fingers and one along them: at the click, the housing top's lean a toward
  the apex (the across-the-fingers view) and m toward the pad's middle line (the along view), protractor overlay, each
  signed (negative away). The skin's normal at the patch leans b = asin((r - s) / r) toward the apex where s is under
  the fingertip's radius r (check 3b; 0 farther back, so 0 over the whole site), and g = 30 - theta deg toward the
  pad's middle line, which is signed: negative for theta over 30 deg, where the normal leans away from the middle line.
  Then n = (tan b / cos g, tan g, 1), d = (tan a, tan m, 1), and the lancet's angle off the normal is
  acos(n . d / (|n| |d|)). Pass: at most 15 deg. These are the scorer's own quantities:
  `test_real_square_check_matches_the_scorer` checks this reading against `tilt_of` at 400 random sites and leans and
  at both ends of theta with leans of both signs.
- Steadied: from the side camera, the wrist rises no more than 2 mm off its block (mm scale) while the lancet touches.
- In the container: the container weighed before and after on the 0.01 g scale gains the lancet's mass (weighed in
  check 3) within 0.1 g; the video shows each gripper open (fingers more than 14 mm apart) or its TCP more than 60 mm
  from the lancet; no lancet loose elsewhere for 0.5 s.
- The base: within 5 mm of its taped outline.

Rig protection and sharps handling (for the trial, not part of the task):

- The blade's exposure: BD's 510(k) says "after skin puncture, the needle/blade returns into the device" and "Once
  activated, the device is auto-disabled". So the blade is out for one stroke of at most 1.8 mm past the shield's face,
  only where the face is pressed at the fire force, and never again.
- Gripper pads, both arms: the face rests on whatever it is pressed on, so a cover the face cannot sink into stops the
  blade. Each finger of both grippers is capped over its tip and outer face with a 3 mm polycarbonate shell [CHOICE];
  the grip faces carry a 3 mm polycarbonate plate under thin grip tape. Foam is not used: it compresses under the 8 to
  15 N press, and the blade would reach the pad. Check 8 proves it. Check 7 is run with the covers on.
- Both arms and every rig part: a policy is not the demo, so the lancet may be pressed on anything under it. The table
  under the work area is covered with a 3 mm polycarbonate sheet. On both arms, each gripper housing and the last two
  links, and every exposed cable within 300 mm of the work area, is wrapped in 3 mm polycarbonate sheet or split loom
  [CHOICE], so a press on any of them meets a cover as in check 8. The printed hand, the base and the stand are plastic
  and cheap to reprint; the cameras and the microphone stand outside the arms' reach.
- Sharps: the operator wears puncture-resistant gloves and loads a new lancet (tab on, sealed) into the cup by hand.
  A used lancet is never touched by hand: one that was dropped is picked up with forceps into the container. The tab
  is taken out of the post's slot with tweezers. The container is closed and replaced at its fill line. Handling
  follows the BD CAUTION above.
- E-stop: the operator holds it; the arms are stopped before anyone reaches into the cell.

## Run

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=lancet_fingerstick.py -E object_jitter=0 \
  -E control_hz=20 --instruction "$(python3 -c "import json;print(json.load(open('task.json'))['instruction'])")" \
  --scorer task_score --seed 0 --max-steps 2400
```

The run limit is 2,400 steps (120 s); the demo takes 28 s of plan.

## Sim results

The hinge, site, hand mass and placed-rung force changed in this revision (tipping sweep, above), so every PhysX run
of the earlier files no longer applies and none is quoted. The runs below are planned on the files here; each result
is filled in from its run's log once it has run.

yam_arena, `--seed 0` (the first row under Seeds), `object_jitter=0`, 20 Hz, 2,400-step limit. FILM_PENDING
DREAM_PENDING Both runs hash `lancet_fingerstick.py`, `mechanism.py` and `task.json` before starting (`sim/film.sh`,
`sim/dream.sh`), to be checked against the files here (`task.json` as it was before its `scores` line was filled in).

More PhysX runs of the scripted policy, each on a copy that `sim/mk.sh` makes of this `lancet_fingerstick.py` with a
per-step trace appended (`sim/log_snippet.py`, which writes `sim/out/trace_<copy>_<seed>.csv`), and for the stems
`sim/stems.py` too; the environment revision inspect-robots logs for each is the SHA-256 prefix of that copy. Each
stem's offline replay gives the expected score (`test_sim_stems`):

| Run | `--seed` | Copy | Expected | PhysX score, steps, fault |
|---|---|---|---|---|
| demo at the 7 other layout corners | 126748, 84277, 100825, 5410, 55059, 5673, 58369 | t00 | 1.0, success | STEMS_PENDING |
| demo with a 0.05 s start delay, corner -x, -y, -turn | 26385 | t05 | 1.0, success | |
| one arm: left parked, the demo's 1.5 s press | 0 | o00 | 0, not steadied | |
| one arm: left parked, a 0.15 s jab | 0 | j00 | 0, not steadied | |
| one arm: left parked, a press 8.5 mm from the apex driven 20 deg toward the wrist | 0 | w00 | 0, not steadied (or slid) | |
| both arms: the lancet pressed down in the stand after the twist | 0 | f00 | 0, fired in the stand | |
| both arms: after firing, the lancet held inside the container, never let go | 0 | k00 | 0.6 | |

The stems' plans pass the same reach check as the demo's (`tools/ikcheck.py stems`). What the stems cannot show: a
grasp slipped more than 20 mm along the housing, the case the old held rule misread, is not reachable by a scripted
path from the stand (the socket's rim keeps the pad tips 18 mm up the lancet, and a slip stops when the tips reach
the face); `test_slipped_or_deep_grasp_in_the_container_is_still_held` covers it offline.

## Sim against real

- The needle, the skin's puncture and the blood are not simulated; the shield's spring stands in for the drive and
  fires at a set force. The real lancet's click and its spent state are read from audio and a re-press. Pressed down
  in the stand after the twist, the sim's lancet fires into the post as the real one would, and scores 0.
- The trainer is rigid on a hinge, in the sim and in the printed rig; a live fingertip yields under the press, and the
  Lance finger's silicone a little.
- The hinge's cheeks and pin and the wrist block are, in the rig, printed parts the hand turns on and rests on; in the
  sim they are drawn only, and the joint and its stop carry the hand.
- The tipping sweep is quasi-static and assumes friction at the skin of at most 0.5 (the sim's); a real Lance finger's
  silicone on the shield's polyethylene has not been measured, and a higher friction lowers the margin (check 4
  measures the tipping force, not the friction).
- The real slide check reads a patch that also records motion after the click (check 6c); the sim stops counting at
  the fire.
- BD has the patient's hand held downward for blood flow; the trainer lies palm up so its fingertips are in reach of a
  top-down gripper, and the blood flow is out of scope.
- Out of scope, from BD's guide: warming the site, the alcohol wipe and air dry, wiping the first drop, collecting the
  specimen and pressing on the site after. The task covers the puncture itself: the tab, the steadied hand, the
  placed and fired lancet, and its discard. Those steps need blood, a wipe or a tube on a fingertip the trainer cannot
  bleed from, and none of them is done with the lancet.

## Tests

`python3 test_lancet_fingerstick.py` (with yam_arena on PYTHONPATH) runs the offline tests: the shield and the site
(the demo's site faces up; the pad's middle, too far round, past the pad or short of 8 mm, the side facing down, the
little finger, the index), the tab, a good press, no fire under 8 N, a firm press for the placed rung, a slide, a small
creep and a fresh placement, a tilted lancet, an unsteadied wrist, fires on the little finger, off the hand and in the
stand, a drop and the container, a slipped or deep grasp still held, the tipping sweep against the placed rung and the
fire (and the old design failing it), the scorer (a model that stops, the base dragged, the ladder in order along the
demo), both arms' demo replayed at the 8 corners of the layout box, on `--seed` 0 to 19 and on every seed in the
table, the layout's draw (inside its bounds, filling them, repeatable), a plan replayed on another seed's layout
failing, the base plate clear of the stand and the container, the lazy routes below and the sim stems, the geometry,
these numbers against the code, the real trial's two-camera squareness reading against the scorer's tilt, and
`rig.scad` current and at the model's sizes. `tools/tipsweep.py` prints the tipping table. `tools/ikcheck.py` (no
argument: the 8 corners, `--seed` 0, 2, 4 and the stems; or `cli=N`, `seed=N`, `corners`, `stems`) checks both arms'
reach, joint6 against its limit and the two TCPs' separation along x; `tools/clearance.py` checks every arm link and
fingertip against the sharps container at the same layouts.

### Lazy routes

| Shortcut | Score | Why |
|---|---|---|
| Press without holding the wrist | 0 | every allowed press tips the hand by 5.0 N, before the 6 N placed rung and the 8 N fire; "pressed with the hand not steadied" |
| Pull the lancet straight out, no twist | 0 | "lifted out of the stand with its tab still on" |
| Twist, then press the lancet down in the stand | 0 | "fired in the stand" |
| Press on the little finger | 0 | "on the little finger (BD: the fifth finger must not be punctured)" |
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
  twist is shared and is only the 0.2 rung here; every higher rung needs the steadied, square press that fires it.
  #2259's review flag ("Protect the gripper pads or use suitable blunt training lancets") is answered under Rig
  protection.
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
  firing is read from the lancet itself, not from the arm.
- #2049, #2082, #1861 (merged): putting used needles in sharps. Here the discard is the last step, not the skill.
- #612 (final, blood tubes in order of draw) and #2242 (open, scanning a wristband): other clinical steps, no sharps.

Category: blood collection is an O*NET Core task of Medical Assistants (833,900): "Collect blood, tissue, or other
laboratory specimens, log the specimens, and prepare them for testing."; and of Phlebotomists (143,900): "Draw blood
from capillaries by dermal puncture, such as heel or finger stick methods." Licensed Practical and Licensed Vocational
Nurses (666,900) have the Core task "Collect samples, such as blood, urine, or sputum from patients, and perform routine
laboratory tests on samples." Two national exams for medical assistants (NCCT's National Certified Medical Assistant
and the CMA (AAMA)) test capillary puncture by finger: it is entry-level work both national certifications examine, not a
specialist's step. The accepted #1951 rests on one Medical Assistants Core task, and #2049 on tasks
of three occupations; this follows #2049.

## Third-party code

`mechanism.py` is adapted from benchmark-tasks pull request #223 (MIT License; the licence is kept in its header).
