# Dry-fit two loose boards end to end on three dowels

Two loose boards lie flat on the bench, end to end, 0.51 m apart. Three 8 mm hardwood dowels stand 20 mm out of the
dark board's end; the pale board has three matching holes. The robot brings the boards together so all three dowels
enter their holes at once, pushes the joint closed until the ends meet along the whole seam with the boards level and
their edges flush, and lets go. This is the dry fit (test fit) before glue: no glue is used, nothing is clamped,
nothing is fixed to the bench.

Files: `dowel_joint.py` (scene, measurement, scorer, scripted demo and stems), `mechanism.py` (third party, MIT: loose
multi-shape rigid bodies and the step hooks), `test_dowel_joint.py` (offline tests), `tools/` (task.json generator,
IK check, arm stiffness), `sim/` (run scripts and the stem copies' maker).

## Sources

Copies of each page were saved and searched as text. Every number in the code carries a marker: `[SOURCE]` quoted
here, `[ARITH]` derived, `[EST]` estimate, `[CHOICE]` design choice, `[RUN]` computed or measured by a tool.

| What | Source and quote | Used for |
|---|---|---|
| Occupation | O*NET 47-2031.00 Carpenters, Employment (2025) "889,700 employees", Core task "Assemble and fasten materials to make frameworks or props, using hand tools and wood screws, nails, dowel pins, or glue."; O*NET 51-7011.00 Cabinetmakers and Bench Carpenters, "81,800 employees", Core "Attach parts or subassemblies together to form completed units, using glue, dowels, nails, screws, or clamps." | category |
| Dowel | Hafele 267.83.240: "Wood Dowel, Fluted, Kiln-dried hardwood, 8 x 40 mm" | 8 mm x 40 mm |
| Chamfer | Cindoco 8 mm x 38 mm MG dowel pins: "Pins are chamfered for easy insertion" (size not given) | tip chamfer exists; 1.0 mm is [EST] |
| Force gauge | Mark-10 Series 5 page: "M5-2 Force gauge, 2 lbF / 32 ozF / 1 kgF / 1000 gF / 10 N", resolution 0.002 N in its table, "Accuracy: ±0.1% of full scale" | the slide and press checks |
| Feeler gauge | Starrett 66MA: "THICKNESS GAGE, 20 LEAVES, 0.05-1.0mm", leaves "0.05, 0.10, ... 0.30, ... 1.0" | the 0.3 mm and 1 mm checks |
| Pin gauge | Vermont Gage "Pin Gage, Minus, Class ZZ, 8.00mm", "Tolerance 0.005 mm" (an 8.20 mm one of the same class is assumed to exist, not checked) | the hole checks |
| Jig and bit | JessEm 08370 dowelling jig manual: "standard with a 8 mm drill bit and bushings"; "The drill bit should be deep enough to accommodate half the dowel you are using."; "We want three (3) holes in this first row. One (1) in the center and one (1) in each outside hole."; "Test fit the joint." | 8 mm holes, 20 mm of dowel each side, three in a row, the dry fit |

Not sourced: board size and mass, dowel pitch (24 mm, marked out, not the jig's bushing pitch), hole depth (22 mm),
chamfer size, wood friction, the dry-fit press, the dowel's and hole's real diameters (measured per trial, below).

## The fit, in the sim and real

- Real: holes drilled with the jig's 8 mm bit, each checked with pin gauges: an 8.00 mm go gauge must enter the full
  22 mm, an 8.20 mm no-go gauge must not enter (so the hole is 8.00 to 8.20 mm). Dowels measured with calipers and
  kept if 7.90 to 8.00 mm. Clearance 0 to 0.15 mm a side. Dry fit, no glue.
- Sim: each hole is a 16-sided bore whose flats are 0.20 mm outside the dowel's corners (the dowel is a 24-sided
  prism of 4.0 mm corner radius), 22 mm deep, a sharp rim; the dowel's tip has a 1.0 mm x 45 deg chamfer. The sim
  hole is 0.05 mm a side wider than the real maximum: PhysX convex contact at much tighter clearances tends to jam
  where wood would slide (stated in fidelity). Contact offset 1 mm on dowels and hole walls, rest offset 0.
- What that keeps [ARITH]: a tip enters only within 1.2 mm of its hole's axis (chamfer plus clearance); a fully entered
  dowel cannot sit more than about 1.1 deg off its hole's axis (2 x 0.2 mm over 20 mm), so the two boards must be within
  about that in yaw to close. A board a couple of millimetres off stops on the dowels' chamfers; one turned a degree or
  more binds part way in (probe 2 below: at 0.20 mm a side, 1.6 to 2.0 deg apart bound about 1 mm in; at 0.10 mm,
  0.5 to 1.0 deg did).
- What it leaves out: the flutes, the dry-fit friction (a few newtons [EST], measured with a force gauge in the real
  trial below), wood compliance.

## Why two arms

Each board starts within reach of one arm only [RUN: yam_arena IK, top-down grip at the board's grip height, x 0.35]:
the left arm's error is 0.3 mm at y -0.075, 11 mm at -0.10 and 23 mm at -0.125, while the dark board's grip starts at
y -0.33 and carrying the pale board onto it would need y -0.155. The right arm is the mirror image. What that shows: one
arm alone cannot carry the other board to the joint, or its own board onto the other where it starts (stem
`left_only`, expected 0). It does not show that a one-arm push could never close the joint.

The relay is a sim simplification. The sim has no dry-fit friction, so a relay (the right arm sets its board down on
the seam line and lets go, then the left arm pushes its board on) seats in the sim; stem `relay` runs it and is
expected to score 1.0, and we report it rather than hide it. In the real trial the dowels need a press of a few newtons
[EST], while a free board starts sliding on the bench at about 0.53 N [ARITH: 0.135 kg x 9.81 x 0.40, the sim boards'
static friction, EST for real wood on laminate], so the pushed board would slide away unless the other hand holds it.
Real-trial pass criterion for the two-arm need: it is confirmed only if the measured insertion press (force gauge, below)
exceeds the measured force to slide a free board on the bench.

## Against other benchmarks

robosuite's TwoArmPegInHole has one arm hold a board with a square hole and the other a long peg, both in the air, and
the ACT / ALOHA sim set has a bimanual peg-into-socket insertion. Here the parts are two loose boards lying on a bench,
there are three round dowels that must each find its own hole at the same time (a 1.2 mm capture per dowel), and full
credit needs a closed, flush, level seam and both hands let go, measured with a feeler gauge and a straightedge.

## Scoring (`score()`, the last step only; `success()` must hold 1 s)

Measured from the two boards' poses, in the dowel board's frame: the gap between the ends at each corner of the seam
(along the dowel board, 0 when they touch, negative when they overlap), the pale board's sideways offset and height step
at its end, each board's tilt from flat, and for each dowel its tip's depth in its own hole and its distance off that
hole's axis.

Every rung needs all of: both corner gaps at least -0.5 mm (ends cannot overlap), the height step within 1 mm (a board
resting on the other stands 19 mm up), both boards within 3 deg of flat, and every dowel's tip at least 1 mm into its
own hole and within 1.2 mm of its axis (a board one pitch off has one dowel outside the pale board and fails).

- 0.25: that.
- 0.5: and both corner gaps at most 10 mm (more than half the 20 mm in).
- 0.75: seated: both corner gaps at most 0.3 mm and the edges flush within 0.3 mm.
- 0.9: seated, every pad point of both grippers at least 30 mm from both boards (each board's box stretched over the
  dowels), both boards under 5 mm/s.
- 1.0: that held 1 s (yam_arena ends the episode with `success`).

Nothing earlier counts: a joint seated and pulled apart scores what it ends as (test). Nothing in the scene can break.

Real trial measurement: a 0.3 mm feeler gauge must not enter the seam at either edge; a straightedge across the joint's
side with the same 0.3 mm feeler for flush; a straightedge across both tops with a 1 mm feeler for the height step; the
1 mm tip depth inferred from a seam gap of at most 19 mm (the dowels stand 20 mm out) and the half-in gap, both with a
depth rule; a tape for the 30 mm hand clearance; an overhead 60 fps video over
a 1 mm grid for stillness. All of these resolve their thresholds (each tool's step is at or under the threshold).

## Building the real trial

- Boards: two 150 x 70 x 19 mm pieces of planed hardwood (one stained dark), ends square within 0.2 mm (try square).
- Dowels: Hafele 267.83.240 (8 x 40 mm fluted), caliper-checked as above, glued 20 mm into three holes in the dark
  board's end with PVA and cured before any trial (so that half is rigid, as in the sim). The holes are marked out 24 mm
  apart [CHOICE] at mid-thickness and drilled with the JessEm 08370 jig and its 8 mm bit, stop collar set for 20 mm;
  the pale board's holes are drilled the same way to 22 mm, and stay dry. Each hole checked with Vermont Gage Class ZZ
  pin gauges (0.005 mm tolerance): an 8.00 mm plus gauge as the go and an 8.20 mm minus gauge as the no-go [CHOICE;
  only the 8.00 mm minus listing was checked].
- Feeler gauges: a Starrett 66MA set (0.30 mm and 1.0 mm leaves used).
- Friction checks with a Mark-10 M5-2 force gauge (10 N, 0.002 N steps): pull a free board along the bench and record
  the peak (expect about 0.5 N [EST]); push the pale board onto the dowels along the bench with the gauge while the dark
  board is held, and record the peak (the dry-fit press, a few newtons [EST]). Compare the two (above).
- Bench: a smooth laminate top; the boards on taped outlines, centres at (0.35, 0.33) and (0.35, -0.33) plus the seed's
  shift and turn. Nothing clamps either board. No hazard: rigid wood parts, no blade, heat or chemistry.

## Run

    PYTHONPATH=<yam-isaac-lab-arena checkout> python3 test_dowel_joint.py
    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=dowel_joint.py -E object_jitter=0 \
        -E control_hz=20 -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 --seed 0 \
        --instruction "$(python3 -c 'import json;print(json.load(open("task.json"))["instruction"])')"

The seed picks one of three start layouts for the dark board (x shift 0, +12, -12 mm; turn 0, +3, -3 deg): the
layout is the seed inspect-robots passes to the reset, mod 3. CLI seeds 0, 1 and 2 reach the reset as 1415158438,
2566101560 and 377265627 [RUN], so layouts 1, 2 and 0. The demo
reads the seed (a privileged script). It also lines the boards up from their true poses, as a worker sights along the
edges: five passes, at the end of the 1 s settle and then after each of four short slides (back 4 mm and in again, the
last two ending with the tips 2 mm short of the holes). Each pass turns each hand so its board is square to the bench
and shifts the pale board's hand so the two end centres meet; the last pass turns only the pale board, to the dark
board's yaw. A pass moves a hand at most 6 mm and 4 deg. The slides are there because a correction made with a board at
rest is only partly carried out (bench friction against the arm's compliance) and the rest lands at the next slide. Only the scripted demo reads those poses; the score never uses them. Runtime: about 5 min wall per run on one L40S including start-up [EST].

Stems: `bash sim/mk.sh` writes traced copies to `sim/arm/`; `bash sim/run.sh <stem> <steps> <lock> <seed>` runs one.

| Stem | What it does | Expected |
|---|---|---|
| `t00` | the demo | 1.0 |
| `s_nothing` | both arms stay put | 0 |
| `s_half` | pushes to an 8 mm gap and lets go | 0.5 |
| `s_offset` | the pale board held one pitch (24 mm) to the side | 0 |
| `s_skew` | the pale board turned 5 deg | 0 |
| `s_stack` | the pale board lifted and set down on the dark board's end | 0 |
| `s_left_only` | the left arm alone carries its board as far as it reaches | 0 |
| `s_relay` | one arm at a time (see "Why two arms") | 1.0 |
| `p_fit`, `p_off`, `p_tight`, `p_loose` | the fit alone: boards start in line 30 mm apart; 0.20 mm a side; the pale board's end 0.5 mm aside and turned 1 deg about it; 0.10 mm; 0.35 mm | probe |

## Sim results

Probe 1 (open loop): `p_fit`, `p_off`, `p_tight`, `p_loose` and `t00` all scored 0. Both hands gripped and slid their
boards as planned, but each board sat 0.35 to 1.3 deg off square in its grip, all the same way; turned about grips 100
to 115 mm from the seam, that put the end centres 2.5 to 4.0 mm apart sideways (5.5 mm in `p_off`, held off on
purpose), past the 1.2 mm capture, and every dowel tip stopped on the pale board's end face with the seam 20 mm open.

Probe 2 (three line-up passes, the boards at rest between them): `p_fit` 1.0 in 473 steps (seated: gaps 0.0 and
0.09 mm, flush 0.23 mm). `p_loose` (0.35 mm a side) 0.5: seated (gaps 0.01 and 0.13 mm) but 0.37 mm off flush, which
the looser holes allow. `p_tight` (0.10 mm) 0: the dowels entered 0.2 to 1.0 mm and bound with the boards 0.5 to 1.0
deg apart in yaw. `p_off` 0: the same bind at 1.6 to 2.0 deg. `t00` 0: the passes made at rest landed late and the
last overshot, so the ends met the mouths 1.69 mm apart. The demo now slides between passes (above).

Batch 3 (before the placement fix): until then `mechanism.py` ran `on_place` only beside a Mechanism, so in this
Body-only task the seeded layout was never placed and every run started in layout 0's poses, while the demo planned
for the seed's layout. Layout 1's plan still caught the board (its 12 mm shift is just past the 11.5 mm open margin,
so the hand dragged the board over); layout 2's put a finger on the board, the hand stopped 15 mm high and turned the
board 12.5 deg, and the trial scored 0. `mechanism.py` now places Body-only tasks too (test), and all earlier results
above ran in layout 0.

Demo on three seeds: SEEDS_PENDING

Filmed demo: FILM_PENDING

Dream-1: DREAM_PENDING

## Reset

Dry fit, nothing consumed: pull the boards apart, set them on their outlines.

## Licence

MIT. `mechanism.py` is adapted from robocurve/benchmark-tasks #223 under its MIT licence (header in the file).
