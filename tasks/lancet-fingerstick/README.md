# Fingerstick a trainer hand with a contact-activated lancet

A trainer hand lies palm up on two foam rests, rolled 30 deg about its fingers so the little-finger side of the ring
and middle fingertips faces up. A BD Microtainer contact-activated lancet (366577, 21 G x 1.8 mm) stands tip down in a
stand whose socket holds its protective tab. A BD 305635 sharps container stands beside. The robot does the fingerstick
as BD's guide gives it: twist the tab off, hold the hand still with one gripper, place the lancet square on the side of
the ring or middle fingertip with the other, press until it fires, and drop the used lancet into the container.

Both hands are needed by the physics, not by a rule. The hand turns on the finger rest's edge and rests on the wrist
rest by its own weight, so a press of 3.5 N (at the site's far end) to 4.4 N (near the apex) tips it, fingertips down.
The lancet fires at 8 N. Without the other hand holding the forearm stub down, the hand tips before the lancet fires,
and a press with the wrist risen more than 2 mm scores 0 (`test_no_steadying_hand`).

The hand's place and turn change by seed, so a replayed path misses the site (`test_open_loop_plan_fails_other_layouts`).

Files: `lancet_fingerstick.py` (the task), `mechanism.py` (articulated objects, adapted from benchmark-tasks #223, MIT,
unchanged from our earlier mug-handle task), `test_lancet_fingerstick.py` (offline tests), `task.json`, `tools/`
(task.json generator, reach check), `sim/` (run scripts).

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
  spring; the tab is broken by "a singular twisting or pulling motion"; opposed sides carry "finger grip indentations".
- Owen Mumford, FDA 510(k) K222111 (Unistik Touch contact-activated safety lancets), bench test: "The force to activate
  the safety feature: 4 - 15N".
- Facet Technologies, FDA 510(k) K221433 (Facet 28G Universal Lancet): "Cap Twist-off torque" "< 4.9 N-cm".
- BD sharps collectors catalog: 305635 "1 qt with needle ports", "4.9 x 4.3 x 6.5" (in, L x W x H); "2.5 × 1.2 open
  (305635)" (in).
- O*NET 31-9092.00 Medical Assistants: Core task "Collect blood, tissue, or other laboratory specimens, log the
  specimens, and prepare them for testing."; "Employment (2025)" "833,900 employees". O*NET's own definition of Core:
  relevance of at least 67% of incumbents and mean importance of at least 3.0.
- NCCT, National Certified Medical Assistant detailed test plan (effective January 2024, from a national job analysis):
  "Perform capillary punctures (i.e., fingerstick, heelstick)."
- benchmark-tasks #1837 (merged): the trainer hand's sizes, "ANSUR II median adult male (hand length 193 mm, breadth 88
  mm)", its finger radii and spacing, and "0.300 kg (silicone skin over a foam core)".

Not sourced, and set in the code with [EST] or [CHOICE]: the lancet's body size (13 x 10 x 37 mm) and mass (4 g); the
fire force, 8 N, chosen inside Unistik's 4 to 15 N because BD publishes none; the tab's break torque, 3 N-cm, under
Facet's 4.9 N-cm limit, and its break angle, 45 deg; the site's bounds, read from BD's shaded figure; the square, slide
and steadied limits; the hand's centre of mass and the rests. Each one the real trial depends on is measured before
the trials (Real trials).

## The model

- The lancet is a loose tool of two links: the housing and its contact shield, which stands 3 mm proud on a spring
  (4 N/mm, 1 N preload). It fires at 8 N on the shield, 1.75 mm of travel, and only once its tab is off and it is out
  of the stand. Firing is read from the shield, so it fires against anything pressed hard enough (skin, table, a
  gripper pad), as the real one does. After firing it is spent.
- The tab: the stand's socket is keyed to the lancet's front and turns on a spring; the tab breaks at 45 deg of turn,
  where the spring's torque is 3 N-cm. The real socket is fixed and the tab's plastic post gives the torque. Lifted out
  more than 10 mm before the break, the lancet still has its tab on: a fault.
- The site: the pad of the middle or ring fingertip, 3 to 15 mm back from the apex and 15 to 45 deg round from the
  pad's middle line, either side (BD shades both). The shield's face within 1.5 mm of the skin is on it. The roll of
  30 deg turns the little-finger side of each pad straight up, so a downward lancet meets it square. Square: the
  lancet's axis within 15 deg of the skin's normal. The demo aims 8 mm back from the ring's apex, the side facing up.
- The press: from first touch (2 N) to firing, the face moving more than 1.0 mm on the skin is a slide. Lifting off
  (under 2 N) ends a placement, so the lancet may be lifted and placed again.
- Steadied: while the lancet presses on anything, the wrist rises more than 2 mm off its rest is a fault. The hand is
  rigid, 0.300 kg, its centre of mass 70 mm behind the pivot, which is 62 mm behind the ring's apex; the forearm stub is
  223 mm behind it. The demo's left hand grips the stub and aims 10 mm under it, about 5 N down [EST: the arm's 0.5
  N/mm]; a 10 N press needs about 2.1 N there.
- Dropped: the lancet out of the stand, not held by either hand and not in the container, loose for 0.5 s. Its fall
  into the container from the release height takes about 0.2 s.
- The hand's base may move 5 mm; more is a fault (dragged).

## Seeds

The hand's base (shift from layout 0, turn about the ring's apex):

| `--seed` | layout | shift, mm | turn, deg |
|---|---|---|---|
| 0 | 1 | 12, -10 | 6 |
| 2 | 0 | 0, 0 | 0 |
| 4 | 2 | -10, 12 | -6 |

The scripted demo reads the seed's layout and plans from it, as a privileged script may. It shows the task can be done.

## Real trials

Never on a person. The trial uses the trainer hand only, with nobody in the arms' reach while they move.

Parts and instruments:

- BD Microtainer contact-activated lancets 366577 (21 G x 1.8 mm), one per trial; a BD 305635 sharps collector.
- An adult right-hand silicone task trainer, life size (193 mm wrist crease to middle fingertip), about 0.3 kg, with a
  forearm stub, as in #1837. It is strapped to a 3D-printed cradle that holds it rolled 30 deg; the cradle sits on a
  fixed block under the proximal phalanges (the pivot) and its wrist end rests on a second block. Both blocks are fixed
  to a 3 kg base plate taped to the table.
- A stand with a printed socket keyed to the lancet's tab cap, fixed to the table.
- A push-pull force gauge (0.1 N readability) with a flat tip; a cap torque gauge (0.1 N-cm readability); digital
  calipers (0.01 mm); a digital angle gauge (0.1 deg); a 0.01 g scale; a protractor; a 10x loupe; washable ink; a side
  camera with a mm scale beside the wrist rest; an overhead camera and a microphone.

Checks before any trial, each with its pass criterion (no trial runs until all pass):

1. Fire force: five lancets pressed by hand, shield first, onto the gauge's tip through a 3 mm silicone pad; the peak
   at the click. Pass: every one at or under 15 N (the top of Unistik's 4 - 15 N). The sim's 8 N is replaced by the
   mean in a run of the offline tests.
2. Tab: five tabs twisted off in the socket by the torque gauge. Pass: every peak at or under 4.9 N-cm (Facet's
   limit) and every break at or under 60 deg (protractor), under the demo's 70 deg twist.
3. Body size: five lancets with calipers. Pass: each side within 1 mm of the sim's 13 x 10 x 37 mm, or the sim is
   updated and its tests re-run.
4. Tipping force: the force gauge pressed down on the ring's site, 8 mm from the apex, until the wrist lifts off its
   rest. Pass: 3.3 to 4.3 N (the sim's 3.8 N there, plus or minus 0.5 N); ballast on the cradle's wrist end is moved
   until it passes.
5. Square: a 2 mm rod held square to the skin at the ring's site is within 3 deg of vertical (angle gauge).
6. Mark: five lancets fired by hand onto a spare trainer finger. Pass: an ink print from the shield's face (inked
   before the press) is seen under the loupe every time.
7. Grip: with the pad covers on, the lancet held in the gripper at the demo's target and pressed on the gauge to 15 N
   does not slip in the grip (no change in the camera over 2 s).

Scoring a trial, each threshold with its instrument:

- Tab off before the lift: the overhead video shows the lancet turned in the socket before it rises 10 mm (the mm
  scale on the stand), and the tab is left in the socket.
- Fired: the click on the audio track. Spent: after the trial the lancet is pressed on the gauge to 15 N with no second
  click (BD: auto-disabled).
- Site: the shield's ink print lies inside a zone drawn on the ring and middle fingertips from a paper template (3 to 15
  mm back from the apex, 15 to 45 deg round from the pad's middle line), checked with the loupe.
- Slide: the print's longest length is at most the shield's diameter plus 1.0 mm (calipers).
- Square: from the side camera, the housing's edge within 15 deg of vertical at the click (protractor overlay), the
  site's normal being vertical by check 5.
- Steadied: from the side camera, the wrist rises no more than 2 mm off its rest (mm scale) while the lancet touches.
- In the container: the container weighed before and after on the 0.01 g scale gains the lancet's mass (weighed in
  check 3) within 0.1 g; the video shows both grippers open and away; no lancet loose elsewhere for 0.5 s.
- The base: within 5 mm of its taped outline.

Rig protection and sharps handling (for the trial, not part of the task):

- Gripper pads: each pad's face is covered with 2 mm EVA foam tape, deeper than the needle's 1.8 mm, replaced after any
  fire against it (check 7 is run with the covers on).
- Both arms and every rig part: the lancet fires only under 4 N or more on its shield and points down, so its needle can
  reach only what is under it. The table under the work area is covered with a 3 mm polycarbonate sheet; the trainer's
  cradle and the stand are plastic; no arm link, camera or cable is placed under the lancet's path from the stand to the
  hand to the container (checked in a dry run with a lancet whose shield is taped down, so it cannot fire).
- Sharps: the operator wears puncture-resistant gloves and loads a new lancet (tab on, sealed) into the socket by hand.
  A used lancet is never touched by hand: one that was dropped is picked up with forceps into the container. The tab
  is taken out of the socket with tweezers. The container is closed and replaced at its fill line. Handling follows the
  BD CAUTION above.
- E-stop: the operator holds it; the arms are stopped before anyone reaches into the cell.

## Run

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=lancet_fingerstick.py -E object_jitter=0 \
  -E control_hz=20 --instruction "$(python3 -c "import json;print(json.load(open('task.json'))['instruction'])")" \
  --scorer task_score --seed 0 --max-steps 2400
```

The run limit is 2,400 steps (120 s); the demo takes 28 s of plan.

## Sim against real

- Not yet run in the simulator. The demo is checked offline only: reach (`tools/ikcheck.py`) and a kinematic replay
  through the scorer (the tests). Scores are pending.
- The needle, the skin's puncture and the blood are not simulated; the shield's spring stands in for the drive and
  fires at a set force. The real lancet's click and its spent state are read from audio and a re-press.
- The trainer is rigid on a pivot; a real silicone fingertip yields a little under the press.
- BD has the patient's hand held downward for blood flow; the trainer lies palm up so its fingertips are in reach of a
  top-down gripper, and the blood flow is out of scope.

## Tests

`python3 test_lancet_fingerstick.py` (with yam_arena on PYTHONPATH) runs the offline tests: the shield and the site
(the demo's site faces up; the pad's middle, too far round, past the pad, the little finger, the index), the tab, a
good press, no fire under 8 N, a firm press for the placed rung, a slide, a small creep and a fresh placement, a tilted
lancet, an unsteadied wrist, fires on the little finger and off the hand, a drop and the container, the tipping force
over the whole site, the scorer (a model that stops, the base dragged, the ladder in order along the demo), both arms'
demo replayed on every film seed, a plan replayed on another layout failing, the lazy routes below, the geometry, and
these numbers against the code. `tools/ikcheck.py seed=N` checks both arms' reach, joint6 against its limit and the
two TCPs' separation along x.

### Lazy routes

| Shortcut | Score | Why |
|---|---|---|
| Press without holding the wrist | 0 | the hand tips at 3.5 to 4.4 N, before the 8 N fire; "pressed with the hand not steadied" |
| Pull the lancet straight out, no twist | 0 | "lifted out of the stand with its tab still on" |
| Press on the little finger | 0 | "on the little finger (BD: the fifth finger must not be punctured)" |
| Press on the pad's middle line | 0 | "on the middle of the ring's pad, not its side", and 30 deg off square |
| Press 30 mm back, past the pad | 0 | "not the fingertip" |
| Slide the lancet while pressing | 0 | "the press slid" |
| Drop the used lancet on the table | 0 | "lancet dropped outside the sharps container" |
| Fire, then keep holding it over the container | 0.6 | never let go into the container |
| Do nothing | 0 | no rung |

## Closest tasks

- #2259 (open): loading a lancet into a pen lancing device, its tab twisted off. That loads a home device and never
  punctures; here a clinical contact-activated lancet is fired on a fingertip held still by the other hand.
- #1837 (merged): clipping a pulse oximeter on a trainer's finger while steadying the hand. Its hand sizes are used
  here. It has no sharp, no firing force, and no site or squareness rule.
- #1846 (final): a pulse oximeter clip, a duplicate of #1837.
- #2039 (final, duplicate of an earlier task): filling blood tubes from a needle holder in a training arm. Its review
  says "Blood collection is core work for medical assistants and phlebotomists". That is venipuncture; this is
  capillary puncture, a different device and site.
- #862 (final): vaccinating a training arm with an inert pen, rejected because "The pen has no needle, activation
  mechanism or dose delivery". Here the lancet is a named product whose activation is modelled from its shield, and
  firing is read from the lancet itself, not from the arm.
- #2049, #2082, #1861 (merged): putting used needles in sharps. Here the discard is the last step, not the skill.
- #612 (final, blood tubes in order of draw) and #2242 (open, scanning a wristband): other clinical steps, no sharps.

Category: the step is the O*NET Core task "Collect blood, tissue, or other laboratory specimens" of Medical Assistants
(833,900), and the national certification test plan lists fingersticks. O*NET's Core means at least 67% of incumbents
rate the task relevant, not every one; some medical assistants may collect specimens only by venipuncture.

## Third-party code

`mechanism.py` is adapted from benchmark-tasks pull request #223 (MIT License; the licence is kept in its header).
