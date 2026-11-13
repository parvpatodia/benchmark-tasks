# scale-eccentricity: run a bench scale's eccentricity (corner-load) test with a 1 kg test weight

Sim round 1 ran on the scorer before the hand check; round 2 ran on the final files with the hand check (both
2026-11-13, inspect-robots 1.0.0, see "Sim results"). Number labels: [SOURCE] read from
a file or page this session, [CODE] read from code, [RUN] printed by a command (offline unless it says sim), [ARITH]
derived, [CHOICE] a design choice, [JUDGEMENT] a call that needs a reason, [EST] an estimate, [UNVERIFIED] not
checked against a source.

## The real step

EURAMET Calibration Guide No. 18, v4.0 (11/2015), "Guidelines on the calibration of non-automatic weighing
instruments", 5.3 Eccentricity test (https://www.nist.gov/document/i-cal-gui-018calibrationguideno18webpdf) [SOURCE,
text and the figure on PDF page 17]:
- "The test comprises placing a test load Lecc in different positions on the load receptor in such a manner that the
  centre of gravity of the applied load takes the positions as indicated in Figure 5.3-1 or equivalent positions, as
  closely as possible."
- Fig. 5.3-1: 1 Centre, 2 Front left, 3 Back left, 4 Back right, 5 Front right. On the rectangular platter the four
  off-centre circles sit on the diagonals, about half way from the centre to each corner (read from the figure image;
  "half way" is [JUDGEMENT], the figure has no dimensions).
- The test load "should be about Max 3 or higher" (a fraction, Max/3, in the PDF; the text extraction drops the bar).
- Method 1: "Prior to the test, the indication is set to zero. The test load is first put on position 1, is then
  moved to the other 4 positions in arbitrary order. Indications I Li are recorded for each position of the load."

The same test in legal metrology (category "Societal relevance") [SOURCE, each text read 2026-11-13]:
- NIST Handbook 44 (2024), Section 2.20 Scales, N.1.3.7 (https://www.nist.gov/document/nist-hb-44-2024-section-220-scales):
  for scales of 500 kg or less, a shift test with a one-third nominal capacity load (30 to 35 %) "centered as nearly as
  possible at the center of each quadrant of the load-receiving element". On a rectangular platter the quadrant
  centres are the half-way diagonal points used here.
- OIML R 76-1:2006, 3.6.2 Eccentric loading and A.4.7 (https://www.oiml.org/en/files/pdf_r/r076-1-e06.pdf): the
  Recommendation covers non-automatic weighing instruments "subject to official metrological control"; the test load
  is 1/3 of Max; for up to four points of support the four quarter segments are loaded in turn.
- EU Directive 2014/31/EU, Art. 1(2) and 3 (eur-lex.europa.eu, CELEX 32014L0031): instruments put into service for
  commercial transactions, for weighing patients in medicine, or for making up medicines on prescription in a pharmacy
  must meet the Directive's requirements and continue to conform.

Why "Societal relevance" and not "Impact on jobs": the reviewer has rejected tasks whose named occupation is under the
500,000-worker threshold, and calibration technicians are a small occupation [JUDGEMENT; no headcount read]. The
evidence that is read is the legal one: scales that price goods and make up medicines are under legal control, and
this test is the one that catches a corner error.

Done by: calibration technicians and inspectors who verify scales in shops, pharmacies and clinics (any scale under a
calibration or verification regime).

## What the error costs

The test exists to find a scale that reads differently with the load off centre (a damaged load cell, a fouled
corner). A skipped or misplaced position, or a reading taken with a hand on the weight, leaves that error unmeasured,
and the scale goes back into service charging buyers and making up medicines by a wrong weight wherever the load sits
off centre. The end-state photo shows the
weight back on its stand; the score comes from the five recorded readings and their order.

## Sources read this session (real sizes)

| Source | What it gave |
|---|---|
| KERN WTB 3K-3NM technical data sheet (kern-sohn.com, TD_WTB+3K-3NM_en.pdf) | Max 3000 g, d 1 g; housing 286 x 316 x 126.5 mm (also "completely mounted"); weighing surface 263 x 203 mm; stainless steel plate, plastic housing, 1 aluminium load cell; stabilization time 2 s; "Eccentric loading at 1/3 [Max]: 1g"; net weight about 3.2 kg |
| OIML R 111-1:2004, Annex A, Table A.1 (read from the page image, PDF page 26) | 1 kg cylindrical (knob) weight: body D1 48 mm, knob D2 43 mm, neck D3 27 mm; height "depending on material" |
| KERN 347-11 data sheets (OIML M1, 1 kg, knob, stainless steel fine turned) | "Dimensions (O x H) 48x80 mm", volume 125 cm^3 |
| KERN 327 data sheet (OIML F1, 1 kg, knob, polished) | 48 x 83 mm: the height varies by class and maker, so 80 mm is one real weight, not a standard |
| KERN FXN3K-4N and FOB3K1 data sheets (3 kg scales) | weighing surfaces 236 x 195 and 175 x 165 mm: the WTB's 263 x 203 is a typical bench-scale platter |

Not read: any OIML or EURAMET dimension for the weight stand (it is a [CHOICE]), any load-cell stiffness for the WTB
(the data sheet gives none), Isaac Lab's gravity constant (9.81 is recalled [UNVERIFIED]; the hold probe checks it),
any YAM payload rating or real pad friction (the 1 kg payload on real arms is [UNVERIFIED]).

## Model and simplifications

- [CHOICE] `mechanism.py` (MIT, ships beside the task file; the same copy as before plus the `on_step` hook from the
  tie-rod-end task, which wraps the embodiment's `_task_step` so a function sees the Isaac Lab env before each scored
  step): one articulation, the housing fixed, one
  prismatic Z joint carrying the platter. The housing is two boxes: under the platter (top 3 mm below the platter's
  underside) and the display console in front (113 mm deep, 60 mm tall [EST], the sheet gives no layout). The platter
  is one 263 x 203 x 6 mm box (thickness [EST]), 0.5 kg [EST].
- [CHOICE] The load cell is a spring: stiffness 4000 N/m, damping 100 N s/m, target M_PLAT g / K so the empty platter
  rests at q = 0. The indication is -K q / g in grams: physics, not a formula of the weight's position, so a hand
  pressing the platter or carrying part of the weight changes it. A real load cell deflects much less [UNVERIFIED, no
  datasheet number read]; 4000 N/m gives 2.45 mm per kg, so 1 g is 2.45 um of travel, which the hold probe checks is
  above PhysX's position noise. Stops at -10 mm (4.08 kg) and +2 mm [ARITH].
- [CHOICE] The ideal scale has no eccentric error, so every point reads 1000 g. The task scores the procedure, not a
  faulty scale; a reading off 1000 g by more than 3 g means something other than the weight is on the platter.
- [CHOICE] The knob weight is one 48 x 80 mm cylinder, 1.000 kg, the knob's outline filled in (a loose object is one
  shape). Its centre of mass sits at mid height; a real knob weight's sits lower. The sourced 48 x 80 mm weight is
  stainless steel; the sim colours it brass for the top camera (R 111-1 allows brass M1 weights [SOURCE 8.4.2]).
- [CHOICE] The weight stand is a fixed dark plastic 80 x 80 mm block beside the scale, top level with the platter
  (126.5 mm), 32 mm clear of the housing. A lower stand puts the carry height out of the YAM's straight-down reach.

## Friction-free design

No joint relies on friction (joint friction had no effect in this stack on an earlier task; drive stiffness did). The
only joint is the platter's prismatic spring; its friction and dynamic friction are 0 (`test_no_friction_anywhere`).
Everything the scorer reads comes from that spring's stiffness and damping and from rigid-body poses. The one friction
the task does use is contact friction between the pads and the weight (1.2 / 1.0 on every asset, the scene's default),
which every pick-and-place task depends on: holding 1 kg needs 4.1 N a finger, the plan squeezes about 15 N [ARITH].

## Layout [CHOICE]

Platter centre at (0.27, 0.24), the stand at (0.27, 0.455); the scale's nearest edge is 0.097 m off the midline.
Points, from the platter centre: (0, 0) and (+-50.75, +-65.75) mm (half of 203/2 in x, half of 263/2 in y). The back
points sit at x 0.321. Straight down, the left YAM solves every carry height (TCP up to 0.19 m over the table) out to x
0.34 for y 0.14 to 0.46 [RUN work/explore.py]. The right arm is not needed and parks.

## Load on the arm [RUN work/explore.py, ARITH]

The YAM's gravity compensation covers its own links, not the payload, so the joint PD springs (80 N m/rad on joints
1-3, 10 on 4-6 [CODE constants.py]) carry the 1 kg. Straight down the wrist joints see no torque; the shoulder sees
2.3 to 3.5 N m (limit 27 N m), the elbow 2.4 N m. The TCP sags J K^-1 J^T F: 12.6 to 20.8 mm down, 2 to 6 mm back
and up to 3 mm sideways, depending on the point and height [RUN sag()]. The plan aims high by that sag while carrying, and hands the load between the hand and the
surface over 1.2 s ('take load', 'set down') so neither the platter nor the weight is jerked.

## Scoring

A reading is a run of 40 control steps (2 s at 20 Hz, the WTB's stabilization time [SOURCE]) of one class:
- zero: the weight's footprint more than 2 mm outside the platter (or its base 5 mm or more above it), the platter
  still (under 2 mm/s), the indication within 3 g of 0;
- a point: the weight upright (3 deg) with its base within 3 mm of the platter top, still (2 mm/s, 0.05 rad/s), the
  platter still, the indication within 3 g of 1000 g, the axis within 10 mm of the point.

Both kinds also need the hands clear for every one of the 40 steps (next section): a point both arms 10 mm or more
from the weight and the platter, the zero both arms 10 mm or more from the platter.

Zero first, then the centre, then the corners in any order, each once. Tolerances [JUDGEMENT]: 3 g is 3 d on the 1 g
display and 0.03 N, so a finger carrying or pressing more than that shows; 10 mm is placing by eye on an unmarked
plate, under a quarter of the weight's diameter and 7x less than the 71 mm between points.

Failures (0, latched) [JUDGEMENT]:
- dropped: the weight moving down faster than 0.2 m/s (about a 2 mm fall) with its base within 30 mm over the platter
  or the stand, where the set-down must be gentle; or faster than 0.6 m/s anywhere (free fall reaches it in 18 mm).
  A carry moving down at 0.25 m/s high over the table is not a drop.
- slid: the axis moving more than 15 mm while the base stays within 3 mm of the platter top (one contact). Dragging
  the weight from point to point instead of lifting it scores 0; set-down drift is a few mm.
- struck: the platter faster than 0.1 m/s. Letting go of a weight resting on the platter (a step load) peaks at
  0.061 m/s [ARITH test_step_load_stays_under_the_strike_limit].
- overload (3000 g); lost (tilt over 30 deg, or its base within 10 mm of the table).

Success needs the weight back on its stand (axis within 10 mm of its centre, base within 3 mm of its top, still), the
platter empty and reading 0, and both arms 10 mm or more from the weight and the platter, for 40 steps in a row, then
1 s more. Tiers: see task.json `scoring` (1.0 only through that success hold; else 0.9, 0.75, 0.15 per point plus 0.05
if back, 0; "back" includes the hands clear). score() never returns more than 0.9.

### The hands (closes the old main hole)

success() and score() get object poses only, so before this change a hand that gripped the weight on a point while
carrying under 3 g of it was invisible (the old README's main open hole). Now a function registered with
`mechanism.on_step` runs before each scored step and reads every body of both YAM arms (`base`, `link1`-`link5`,
`gripper`, `tip_left`, `tip_right`) from `robot.data.body_link_pose_w`, minus the env origin. The scorer then takes a
lower bound on each arm's gap to the weight and to the platter: points laid over the weight's cylinder and the
platter's box at most 2 mm apart, and for each point the larger of its distance to a hull's box and the hull's
face-plane distance max_i(n_i . p - d_i) over the URDF's CoACD collision hulls (`yam_arena.analysis.link_hulls`);
neither exceeds the true distance, so the gap is at least the smallest bound minus 2 mm. 10 mm is the line [JUDGEMENT]:
the arm lags its target 2 to 5 mm when moving and contact offsets add 1 to 2 mm, and on a real rig 10 mm shows on video.
No reading (the hook not wired, or not yet run since the reset) counts as not clear: nothing is recorded.

The same technique ran in the sim on the tie-rod-end task [RUN sim, that task]: the hook fired, the body names
matched, and it read a closed grip at -2 mm, an open hand at about 42 mm and a parked arm at the 50 mm cap. Here it is
checked offline only [RUN pytest]: the bound never exceeds the exact vertex distance and is within 12 mm of it; the
laid points cover every surface within 2 mm; either arm at 9.5 mm blocks a reading and at 10.5 mm lets it count;
a closed grip reads -2 mm; a fake Isaac Lab env with shuffled body order and an env origin round-trips; a missing body
raises. The bound's slack: an open hand flanking the weight (pads 16 mm off its faces) reads 14.0 mm.

Real rig: score from a video showing the display and the platter. Zero the scale and check it reads 1 kg at the
centre first (a real WTB has its own span error [UNVERIFIED] and up to 1 g eccentric error at Max/3 [SOURCE]); a
reading is the display steady within 3 g of the target for 2 s with no hand within 10 mm of the weight or the
platter; drops,
slides, strikes and knocks are judged from the video. The end-state photo alone shows only the weight back home.

What the scorer cannot see, stated plainly:
- No contact forces. The arms are seen by geometry, not force, from the URDF's CoACD hulls. The sim's colliders are
  the same pieces with `approximation = "convexHull"`; PhysX may cook a hull with more vertices than its limit into a
  superset of it (the tip pieces have up to 1,378 vertices) [UNVERIFIED, recalled, not read], so the sim's true gap
  can be somewhat smaller than the bound. The 10 mm line is the margin for that; round 2's p_nogo and round 3's gap
  rows measure it.
- A drag split into hops (slide 14 mm, lift the base over 3 mm, repeat) is not a `slid` failure; each hop is a lift by
  the rule.
- A physics blow-up that makes an arm pose NaN raises in the clearance (as a missing arm body does) and ends the
  trial with an error rather than a score; neither passes quietly.
- The zero reading is met at the start: the empty platter reads 0 while the robot holds still. There is no tare key;
  the rule makes the zero come before the centre's reading, as in the source.
- The 20 Hz samples can miss a short spike: a fall from under about 2 mm, or a brief knock on the platter.
- Walking the weight by rocking it on its rim (tilted over 3 deg, base off the plate by the scorer's measure) is not a
  slide by the rule; it is not a lift either.

## Demo (scripted, left arm)

Order [CHOICE]: stand, centre, front left, back left, back right, front right, stand. Per move: slide in at the grip
height plus 20 mm, descend, close (pads 3 mm inside each face), take the load, lift 18 mm, carry, lower, set down,
open, rise 20 mm, slide out 75 mm along -x (the 80 mm tall weight puts lifting over it out of reach), wait 1.6 s so
the weight stands alone 3.7 s after set down. Timeline, delay 0 [RUN]:

| t (s) | Left |
|---|---|
| 0-3.0 | to the stand |
| 3.0-17.3 | weight to the centre; zero already recorded |
| 17.3-32.6 | to front left |
| 32.6-47.9 | to back left |
| 47.9-63.8 | to back right |
| 63.8-79.1 | to front right |
| 79.1-96.5 | back to the stand (set down 91.9, open 92.9, rise ends 94.0, slide out, park); success holds from 95.40 s, the trial ends at 96.35 s [ARITH replay with FK hands] |

Readings in the replay with the hands read by FK [ARITH weight model, RUN pytest]: zero 1.95 s, centre 17.70, front left
33.00, back left 48.30, back right 64.20, front right 79.50 s; each about 1.4 s after the hand's rise ends.

Hand clearance per dwell, both arms, by FK of the planned joint targets every 0.05 s [RUN
test_fk_demo_hands_clear_through_every_dwell_and_succeed]. Window: end of 'rise' to the end of 'wait' (for the final
stand, to the end). The minimum is the lower bound, while the open pads still flank the weight just after the rise;
the hand is 30 mm or more off during the wait. The clear run is the whole stretch with both arms 10 mm or more off
(needs 2 s):

| Dwell | Min clearance in window | Clear run |
|---|---|---|
| centre | 14.0 mm | 5.55 s (15.75-21.25 s) |
| front left | 14.0 mm | 5.55 s |
| back left | 14.0 mm | 5.55 s |
| back right | 14.0 mm | 5.55 s |
| front right | 14.0 mm | 5.55 s |
| stand (final hold) | 14.0 mm | 6.10 s, to the end |
| zero (first 2 s, platter only) | 47.4 mm | from the start |

With the weight set down 5 mm off sideways (the sim's offsets were 1.2 to 2.5 mm at SAG_SCALE 0.7) each point still
has a clear run of 3 s or more [RUN test_fk_dwells_stay_clear_with_the_weight_set_down_off_by_5_mm]. The slide-out is
unchanged: it already clears 10 mm.

Offline checks [RUN pytest]: worst IK 0.097 mm along the whole demo (delays 0, 0.05, 0.1; sag aims 0.7, 1.0, 1.4x;
the margin to 0.1 mm is thin; the sim tracked it, round 1). The left arm's links 3-5, gripper and fingers stay 5 mm or more from the housing, the
platter, the stand and the table (closest 10.7 mm, the fingers to the platter while closing at the centre), and from
the weight except while closing on, holding or opening off it (closest 7.5 mm, descending at the stand). With the real
down-sag 0.7, 1.0 or 1.1x the model's (the sim measured about 1.0, below), the 0.7 aim keeps the carried weight's
base 8 mm or more over the surfaces (11.8 mm at 1.0 [RUN]). The planned demo,
replayed through success() and score() (an [ARITH] model of the weight, not physics), records all five and holds
success over 1 s.

## Sim results, round 1 [RUN sim, instance, inspect-robots 1.0.0, 2026-11-13; scorer before the hand check]

p_hold (weight placed on the centre by the reset, arms parked, 160 steps): no failure, 999.994 g mean, std 0.005 g,
platter sink -2.4525 mm (the model's 1 kg g / K), reading 'centre' throughout. So G = 9.81 and the spring's gain hold,
and contact jitter is about 100x under the 3 g tolerance.

The three sag aims, full demo, 2,000 steps:

| Probe | SAG_SCALE | task_score | Failure | Carry clear (mm, aim 18) | Offset at the end of each wait (mm), centre .. front right, stand | Recorded | End |
|---|---|---|---|---|---|---|---|
| p_sag_lo | 0.7 | 1.0 | none | 12.2-15.0 | 1.2 / 2.1 / 2.0 / 1.8 / 2.4 / 2.5 | all five, in order | on the stand |
| p_feas | 1.0 | 1.0 | none | 17.3-18.5 | 1.6 / 3.5 / 6.0 / 7.2 / 6.9 / 6.6 | all five, in order | on the stand, empty 0.000 g |
| p_sag_hi | 1.4 | 1.0 | none | 23.4-25.8 | 2.0 / 3.9 / 5.6 / 6.9 / 7.6 / 7.9 | all five | on the stand, empty 0.000 g |

Reading [ARITH]: carry clear = 18 mm + (aim - real) x model down-sag (13 to 21 mm), so 1.0x clearing about 18 mm and
1.4x clearing 5 to 8 mm more put the real down-sag near 1.0x the model. The offsets tell the x, y part: at 1.0 and 1.4
they grow along the sequence to 6.6-7.9 mm, at 0.7 they stay 1.2-2.5 mm, so the model's x, y sag is over-corrected at
1.0 and up. SIMQUEUE's GO rows (no failure, carry clear 8 mm or more, offsets under 5 mm, all five, task_score 1.0)
are all met by p_sag_lo only, so SAG_SCALE = 0.7 [CHOICE from RUN]. It costs about 6 mm of carry clearance against the
1.0 aim (12.2 mm measured, 8 mm needed).

These runs used the scorer before the hand check; the hand check changes when a reading starts (after the rise, not
right after the set down) and needs both arms clear, so round 2 reran the demo on the final files.

Round 2, final files with the hand check (SAG_SCALE 0.7), 2,400-step limit [RUN, sim]:

| Probe | task_score | Steps | Failure | Carry clear (mm) | Offsets (mm) | Hand gap to weight / platter over the waits (mm, min) | Recorded |
|---|---|---|---|---|---|---|---|
| p_feas (demo) | 1.0 | 1,928 | none | 12.2-15.0 | 1.2-2.5 | 12.3 / 30.0 | all five, in order |
| p_d05 (start 0.05 s late) | 1.0 | 1,930 | none | 12.3-14.9 | 1.2-2.6 | 12.4 / 30.0 | all five, in order |
| p_d10 (start 0.10 s late) | 1.0 | 1,930 | none | 12.2-15.0 | 1.2-2.6 | 12.4 / 30.0 | all five, in order |
| p_corner_first (front left before the centre) | 0.65 | 1,991 | none | 12.2-14.8 | 1.5-2.5 | 12.4 / 30.0 | front left never; the other four |
| p_nogo (the hand never lets go) | 0.0 | 1,991 | none | 12.2-15.0 | 0.6-1.4 | -2.0 / 10.9 (closed on the weight) | zero only |

Media, on the final files (sha256 of `scale_eccentricity.py`, `mechanism.py` and `task.json` checked equal to the
runs' before the `scores` field was filled in), inspect-robots 1.0.0: the filmed demo scored 1.0 with success at step
1,928 (96.4 s); the Dream-1 run on the final instruction scored 0 over 2,400 steps (an earlier run, on a longer
instruction the board does not accept, also scored 0 and is not used). The screenshot is the top view at the centre reading
(18 s): at the start the weight on its stand sits under the left wrist in that view.

The hook read both arms on every row of every run (1,928 of 1,928 in p_feas). With the hand closed on it, p_nogo's
weight read 1,014 to 1,039 g, so a held reading is wrong as well as unpaid. p_corner_first and p_nogo match the
offline predictions (0.65 and 0.0).

## Feasibility risks, ranked [JUDGEMENT]

1. The hand check's margin: in the sim the closest hand gap over a reading is 12.3 mm against the 10 mm line (offline
   lower bound 14.0 mm). A real arm that lags more than that would delay, not stop, a reading.
2. The grip holding 1 kg by pad friction while the pads also take the weight's 48 mm face at 3 mm squeeze; a slip
   during the carry is a drop.
3. Contact jitter on the sprung platter: measured 0.005 g std (p_hold), not a risk now.
4. The pads drag the weight while opening or sliding out (16 mm clear each side when open).
5. IK margin: 0.097 mm against the 0.1 mm check (moving the scale 1 cm closer gives 0.094 to 0.099 mm [RUN], no real
   gain, so the layout stays).
6. G and the drive units: G recalled, not read, but p_hold read 999.994 g for 1 kg and p_feas 0.000 g empty.
7. Real arms: no YAM payload rating or real pad friction read [UNVERIFIED]; the sim grip relies on friction 1.2.
8. Acceptance against our own bar (4a.1, 4a.6): the motion is five placements at computed spots. The hard part is
   physical: a 1 kg load (the arm sags 13 to 21 mm under it), set down and let go on a sprung platter without a drop
   or strike, the hand fully clear for each reading, and the weight lifted, never slid, between points.

## Closest tasks on the board (from the spec's sweep, 2026-11-09: 1,383 tasks, 1,412 PRs; not re-swept here)

No hit for eccentric, corner-load, shift test or test weight. Nearest: TK-GPXWRJ dispense 150 g on a digital scale;
TK-CYETN9 analytical balance 1.00 g; TK-DQ423L infant beam scale (sliding poise); TK-DNA63Y balance a plank in matched
pairs; TK-4EZENW dead-weight gauge tester; TK-35NWFU gauge check with a reference block. None tests a scale's
off-centre response, and none needs a heavy load placed on unmarked geometric points.

## Files

| File | What |
|---|---|
| `scale_eccentricity.py` | the task: model, layout, scorer, scripted demo |
| `mechanism.py` | articulated-object helper (MIT; adds the `on_step` hook) |
| `test_scale_eccentricity.py` | offline checks: sources, spring, scorer tiers and holes, hands and the hook, layout, demo IK and clearance, replays |
| `mutate.sh` | mutation check of the scorer and model (every mutant must be caught) |
| `task.json` | the submission fields |
| `SIMQUEUE.md`, `sim/` | probe builder (`mkall.sh`, `mklog.py`), runner (`run.sh`), `smoke.py`, `verdict.py`, the filmed demo (`film.sh`) and the one guarded Dream-1 run (`dream.sh`) |
| `work/` | build-time scripts (reach, payload sag, finger geometry); not part of the task |

The PR folder is `task.json`, `README.md`, `scale_eccentricity.py` and `mechanism.py`. The test file, `mutate.sh`,
`SIMQUEUE.md`, `sim/` and `work/` are build tools kept out of it (they hold instance paths and the team's blind-check
list).

## Run

Keep `mechanism.py` next to `scale_eccentricity.py` (the task file imports it from its own folder).

Offline checks:
```
PYTHONPATH=<yam-isaac-lab-arena checkout> python -m pytest -q -W error::DeprecationWarning test_scale_eccentricity.py
./mutate.sh . <abs path to yam-isaac-lab-arena checkout>
```

Scripted demo in the sim (yam_scripted plays `demo()`):
```
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=scale_eccentricity.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$(python3 -c "import json;print(json.load(open('task.json'))['instruction'])")" \
  --scorer task_score --max-steps 2400
```
Round 1 ran on the scorer before the hand check; round 2 ran on the final files (see Sim results).

## Hole hunt (one fresh-context review of this folder against rules.md)

| Finding | Fix |
|---|---|
| Dragging the weight along the platter between points scored 0.9 | `slid` failure: the axis may move at most 15 mm in one contact |
| No photo or real-rig scoring | real-rig video protocol in `scoring` and above |
| A carry moving down at 0.25 m/s anywhere failed as `dropped` | 0.2 m/s only within 30 mm over the platter or stand; 0.6 m/s elsewhere |
| A gripped weight can read 1000 g; the README called this unlikely | restated as the main open hole; probe `p_nogo` measures it |
| The demo's trial ended with the hand still closed on the weight | success needs 2 s back on the stand first; the trial now ends with the hand open |
| p_hold cannot see the step load (the reset's settle absorbs it); no empty offset check | step load left as [ARITH]; verdict prints p_feas's empty-platter mean; offset and gain rows split in SIMQUEUE |
| Real scale has span and eccentric error, 1000 g absolute may fail | real-rig protocol zeroes and checks 1 kg at the centre first |
| Local paths and member names in sim files and the test's blind list | stated: those files are not in the PR folder; README local paths removed |
| YAM payload on real arms not cited | stated [UNVERIFIED] in sources and risk 7 |
| Weight material, stand material, housing ledge, display, 2 mm clear margin, zero-before-centre, base on the stand top | stated in setup, scoring and the instruction |
| Wrong numbers: 37 mm (is 32), 0.10 m (is 0.097), sag ranges | fixed |
| IK margin thin | tried the scale 1 cm closer: no real gain; stated |
| Same pick-and-place motion as many tasks (bar 4a) | risk 8 names the physical difficulty |
| Not run in the sim, no media | rounds 1 and 2 ran (see Sim results); the media are the filmed demo and the Dream-1 run |

Second hole hunt (fresh context, read-only, 2026-11-13, on the scorer with the hand check): no MUST-FIX.

| Finding | Fix |
|---|---|
| The instruction said "both hands", the code checks every body of both arms (an elbow near the platter blocks a reading) | instruction now says "no part of either arm within 10 mm" |
| PhysX may cook a collider larger than the URDF hull, so the sim gap can be smaller than the bound | stated under "cannot see"; the 10 mm line is the margin; rounds 2 and 3 measure it |
| The hand check is proven offline only | stated at the top; rounds 2 and 3 in SIMQUEUE.md |
| A drag split into hops of under 15 mm with 3 mm lifts is not `slid` | stated (each hop is a lift by the rule) |
| NaN arm poses raise instead of scoring 0 | stated; kept, as on the tie-rod task, so a bad reading never passes quietly |
| The zero is met at the start with no action; DWELL_STEPS assumes the pinned 20 Hz; jitter must be 0 | already stated / pinned in the run settings |
