# Dry-fit two loose boards end to end on three dowels

Two loose boards lie flat on the bench, end to end, 0.51 m apart. Three 8 mm hardwood dowels stand 20 mm out of the
dark board's end; the pale board has three matching holes. The robot brings the boards together so all three dowels
enter their holes at once, pushes the joint closed until the ends meet along the whole seam with the boards level and
their edges flush, lets go of both and moves both hands clear. This is the dry fit (test fit) before glue: no glue is
used, nothing is clamped, nothing is fixed to the bench.

Files: `dowel_joint.py` (scene, measurement, scorer, scripted demo and stems), `mechanism.py` (third party, MIT: loose
multi-shape rigid bodies and the step hooks), `test_dowel_joint.py` (offline tests), `tools/` (task.json generator,
IK check, arm stiffness), `sim/` (run scripts and the stem copies' maker).

## Why it matters

A dowel joint is dry-fitted before it is glued: the dowelling jig's manual ends each joint with "Test fit the joint."
Dowel pins are named in the carpenter's own O*NET Core task: "using hand tools and wood screws, nails, dowel pins, or
glue". The skill is lining up two loose parts so three pins enter three holes together, then closing the joint square.

## Sources

Copies of each page were saved and searched as text. Every number in the code carries a marker: `[SOURCE]` quoted
here, `[ARITH]` derived, `[EST]` estimate, `[CHOICE]` design choice, `[RUN]` computed or measured by a tool. Numbers
with none of these are layout and timing choices of the scripted demo.

| What | Source and quote | Used for |
|---|---|---|
| Occupation | O*NET 47-2031.00 Carpenters, Employment (2025) "889,700 employees", Core task "Assemble and fasten materials to make frameworks or props, using hand tools and wood screws, nails, dowel pins, or glue."; O*NET 51-7011.00 Cabinetmakers and Bench Carpenters, "81,800 employees", Core "Attach parts or subassemblies together to form completed units, using glue, dowels, nails, screws, or clamps." | category |
| Dowel | Hafele 267.83.240: "Wood Dowel, Fluted, Kiln-dried hardwood, 8 x 40 mm" | 8 mm x 40 mm |
| Chamfer | Cindoco 8 mm x 38 mm MG dowel pins: "Pins are chamfered for easy insertion" (size not given) | tip chamfer exists; 1.0 mm is [EST] |
| Force gauge | Mark-10 Series 5 page: "M5-2 Force gauge, 2 lbF / 32 ozF / 1 kgF / 1000 gF / 10 N", resolution 0.002 N in its table, "Accuracy is ±0.1% of full scale" | the slide and press checks |
| Feeler gauge | Starrett 66MA: "THICKNESS GAGE, 20 LEAVES, 0.05-1.0mm", leaves "0.05, 0.10, ... 0.30, ... 1.0" | the 0.3 mm, 1 mm and 3.5 mm checks |
| Pin gauge | A reseller's listing of Vermont Gage "Pin Gage, Minus, Class ZZ, 8.00mm", "Tolerance 0.005 mm", "Type NoGo". The maker's own pages for the 8.00 mm plus and 8.20 mm minus pins used below could not be fetched | the hole checks [CHOICE, listing not checked] |
| Jig and bit | JessEm 08370 dowelling jig manual: "standard with a 8 mm drill bit and bushings"; "The drill bit should be deep enough to accommodate half the dowel you are using."; "We want three (3) holes in this first row. One (1) in the center and one (1) in each outside hole."; "Test fit the joint." | 8 mm holes, 20 mm of dowel each side, three in a row, the dry fit |
| Sim friction | Isaac Lab source (main branch; the installed 3.0 not checked), `RigidBodyMaterialBaseCfg`: "static_friction: float = 0.5", "dynamic_friction: float = 0.5", the default for prims with no material. PhysX `PxMaterial`: "Default: PxCombineMode::eAVERAGE" | the bench's friction in the sim |

Not sourced: board size and mass, dowel pitch (24 mm, marked out, not the jig's bushing pitch), hole depth (22 mm),
chamfer size, wood friction, the dry-fit press, the dowel's and hole's real diameters (measured per trial, below), the
YAM gripper's real grip force.

## The fit, in the sim and real

- Real: holes drilled with the jig's 8 mm bit, each checked with pin gauges: an 8.00 mm go gauge must enter the full
  22 mm, an 8.20 mm no-go gauge must not enter (so the hole is 8.00 to 8.20 mm). Dowels measured with calipers and
  kept if 7.90 to 8.00 mm. Clearance 0 to 0.15 mm a side. Dry fit, no glue.
- Sim: each hole is a 16-sided bore whose flats are 0.20 mm outside the dowel's corners (the dowel is a 24-sided
  prism of 4.0 mm corner radius), 22 mm deep, a sharp rim; the dowel's tip has a 1.0 mm x 45 deg chamfer. Contact
  offset 1 mm on dowels and hole walls, rest offset 0.
- Friction in the sim: boards, dowels and bore walls are wood, 0.40 static and 0.35 dynamic [EST]. The bench (the
  station table) authors no material, so it takes the scene default, 0.5 and 0.5, and PhysX averages each pair: board
  on bench 0.45 static, so a free board starts sliding at 0.60 N [ARITH: 0.45 x 0.135 kg x 9.81].
- Kept [ARITH]: a tip enters only within 1.2 mm of its hole's axis (chamfer plus clearance); a fully entered dowel
  cannot sit more than about 1.1 deg off its hole's axis (2 x 0.2 mm over 20 mm), so the boards must be within about
  that in yaw to close; the bore walls' wood friction. A board a couple of millimetres off stops on the chamfers; one
  turned a degree or more binds part way in (probe 2 below).
- Not kept: an interference press. A real dowel a touch over its hole squeezes in and needs a press; the sim's hole is
  0.05 mm a side wider than the real maximum, because PhysX convex contacts at tighter clearances tend to jam where
  wood would slide (probe 2: at 0.10 mm a side the dowels bound 0.2 to 1.0 mm in). The flutes and the wood's give are
  not modelled either. So the sim's press is lighter than a tight real fit. The real trial measures it (below).

## Why two arms

Reach decides it [RUN: yam_arena IK, top-down grip at the board's grip height, x 0.35; test]: the left arm's error is
0.3 mm at y -0.075, 11 mm at -0.10 and 23 mm at -0.125, while the dark board's grip starts at y -0.33 and carrying the
pale board onto it would need y -0.155. The right arm is the mirror image. So one arm alone cannot reach the other
board where it starts, nor carry its own board onto it (stem `left_only`, expected 0).

Reach does not rule out a relay: one arm sets its board on the seam line and lets go, the other pushes its board on.
Stem `relay` runs that in the sim and is reported, not gated. A hypothesis, untested: in a real trial the pushed loose
board would slide away whenever the press exceeds the 0.5 N or so it takes to slide a board on the bench [EST], so the
other hand must hold it. The real trial tests it: it holds only if the measured peak press exceeds the measured slide
force (below).

## Against other benchmarks

robosuite's TwoArmPegInHole fixes a peg and a plate with a hole to the two arms' end effectors, with no gripper
("this environment is not meant to be used with any gripper at all") [SOURCE: its source file]. ACT's sim set has a
"Bimanual Insertion" task [SOURCE: ACT README]. Here the parts are two loose boards lying on a bench, three round
dowels must each find its own hole at the same time (a 1.2 mm capture per dowel), and full credit needs a closed,
flush, level seam and both hands clear. Only these two were checked; no wider survey was made.

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
  dowels), both boards under 5 mm/s. Open pads beside a board sit only 11.5 mm off it, so a hand that lets go but
  stays put scores 0.75.
- 1.0: that held 1 s (yam_arena ends the episode with `success`).

Nothing earlier counts: a joint seated and pulled apart scores what it ends as (test). Nothing in the scene can break.

Real trial measurement:

- Seat: a 0.3 mm feeler gauge must not enter the seam at either edge.
- Flush: a straightedge laid along each long side face across the seam, held flat on the dark board; the 0.3 mm leaf
  must not enter between it and the pale board, on either side.
- Height step: a straightedge across both tops with a 1 mm leaf.
- Flat: a 3.5 mm stack of leaves (1.0 + 1.0 + 1.0 + 0.5) must not pass under any edge of either board. That is 2.9 deg
  across the 70 mm width and 1.3 deg along the 150 mm length [ARITH], so it is at least as strict as the sim's 3 deg
  (stricter along the length).
- Depth: the 1 mm tip depth is inferred from a seam gap of at most 19 mm (the dowels stand 20 mm out), and the half-in
  gap likewise, both with a depth rule.
- Hands: a tape for the 30 mm clearance. Stillness: an overhead 60 fps video over a 1 mm grid.

Each tool's step is at or under its threshold.

## Building the real trial

- Boards: two 150 x 70 x 19 mm pieces of planed hardwood (one stained dark), ends square within 0.2 mm (try square).
- Dowels: Hafele 267.83.240 (8 x 40 mm fluted), caliper-checked as above, glued 20 mm into three holes in the dark
  board's end with PVA and cured before any trial (so that half is rigid, as in the sim). The holes are marked out 24 mm
  apart [CHOICE] at mid-thickness and drilled with the JessEm 08370 jig and its 8 mm bit, stop collar set for 20 mm;
  the pale board's holes are drilled the same way to 22 mm, and stay dry.
- Pin gauges: Vermont Gage Class ZZ, an 8.00 mm plus gauge as the go and an 8.20 mm minus gauge as the no-go
  [CHOICE, listing not checked].
- Feeler gauges: a Starrett 66MA set (0.30 mm, 0.50 mm and 1.0 mm leaves used).
- Force checks with a Mark-10 M5-2 force gauge (10 N, 0.002 N steps). Slide: pull a free board along the bench and
  record the peak (about 0.5 N [EST]). Press: push the pale board onto the dowels along the bench with the gauge while
  the dark board is held, and record the peak.
- Accept rule for a board pair: keep it only if its measured peak press is under 10 N (the gauge's range). Otherwise
  re-drill or swap the dowels.
- Can the YAM pads pass that press? Unverified for the real gripper: its grip force is not sourced. In the sim the
  finger target sits 5 mm past the board's side at 5000 N/m, a 25 N squeeze under the 50 N effort cap, so the two pads
  hold up to about 22 N along the board [ARITH: 2 x 0.45 x 25 N, the pads' friction taken as the 0.5 default, not
  checked in the arm's USD]. In the real trial, pencil-mark each board at the pads and fail the trial if a board
  slipped in the grip.
- Bench: a smooth laminate top; the boards on taped outlines, centres at (0.35, 0.33) and (0.35, -0.33) plus the seed's
  shift and turn. Nothing clamps either board. No hazard: rigid wood parts, no blade, heat or chemistry.

## Run

    PYTHONPATH=<yam-isaac-lab-arena checkout> python3 test_dowel_joint.py
    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=dowel_joint.py -E object_jitter=0 \
        -E control_hz=20 -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 --seed 0 \
        --instruction "$(python3 -c 'import json;print(json.load(open("task.json"))["instruction"])')"

The seed picks one of three start layouts for the dark board (x shift 0, +12, -12 mm; turn 0, +3, -3 deg): the
layout is the seed inspect-robots passes to the reset, mod 3. CLI seeds 0, 1 and 2 reach the reset as 1415158438,
2566101560 and 377265627 [RUN], so layouts 1, 2 and 0.

The scripted demo is privileged: it reads the seed and the boards' true poses. The score never uses them.

1. Grip each board, slide both to a 30 mm gap (tips 10 mm short) and settle 1 s.
2. Line up: a pass at the end of the settle, then four slides, each backing the gap off 16 mm (8 mm a hand) and coming
   in to 26, 24, 22 and 22 mm (the last two leave the tips 2 mm short of the holes), with a pass after each.
3. A pass turns each hand so its board is square to the bench and shifts the pale board's hand so the two end centres
   meet; the last pass before a push turns only the pale board, to the dark board's yaw. A pass moves a hand at most
   6 mm and 4 deg.
4. Why the slides: a correction made with a board at rest is only partly carried out (bench friction against the arm's
   compliance), and the rest lands at the next slide.
5. Every inward gap is commanded 7 mm closer than its target, because the boards stop that far short (batch 4:
   commanded 8 mm, stopped at 15; 6.6 mm [ARITH: 2 x 0.60 N / 180 N/m]). The line-up's back-offs are not offset.
6. Try: push toward a 15 mm gap and check that every tip is at least 3 mm into its own hole.
7. If not, retry: back off to a 26 mm gap (commanded 7 mm wider), line up, come in to the mouths, line up again, push
   again; up to 2 retries. Once a check finds the dowels in, the rest of the retries are held.
8. Square: with the tips in, turn the pale board's hand by the boards' relative turn. Push the joint closed, square
   once more early in the hold at the seat, ease off, let go, rise and draw back.

Runtime: about 5 min wall per run on one L40S including start-up [EST].

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
| `s_hold` | the demo, but both hands keep their grip to the end | 0.75 |
| `s_relay` | one arm at a time (see "Why two arms") | reported, not gated |
| `p_fit`, `p_off`, `p_tight`, `p_loose` | the fit alone: boards start in line 30 mm apart; 0.20 mm a side; the pale board's end 0.5 mm aside and turned 1 deg about it; 0.10 mm; 0.35 mm | probe |

## Sim results

Earlier commits, one line each (the stems' numbers there do not carry over to the final commit):

- Probe 1, open loop: all 0. Each board sat 0.35 to 1.3 deg off square in its grip, so the ends met 2.5 to 4.0 mm apart, past the 1.2 mm capture. Fix: the line-up.
- Probe 2, passes at rest: `p_fit` 1.0 (473 steps); `p_loose` 0.5 (0.37 mm off flush); `p_tight` 0 and `p_off` 0 (bound 0.5 to 2.0 deg apart); `t00` 0 (late corrections overshot, 1.69 mm). Fix: slides between passes.
- Batch 3: `mechanism.py` placed seeded poses only beside a Mechanism, so this Body-only task always started in layout 0; layout 2 scored 0. Fix: Body-only tasks are placed too (test).
- Batch 4: layout 1 1.0; layouts 0 and 2 met the mouths 0.25 to 0.46 mm off and stalled on the chamfers (0). The come-ins stopped 7 mm short inside the friction band; `s_half` 0.25, `s_relay` 0.5. Fix: tries and retries, commands offset by that lag, 8 mm back-offs.
- Batch 5: layouts 0 and 1 1.0 (759 steps); layout 2 0.5: the chamfers turned the boards 0.37 deg apart after entry and one corner stayed 0.32 mm open. Fix: the square passes.

Stems on the final commit: STEMS_PENDING

Demo on three seeds: SEEDS_PENDING

Filmed demo: FILM_PENDING

Dream-1: DREAM_PENDING

## Reset

Dry fit, nothing consumed: pull the boards apart, set them on their outlines.

## Licence

MIT. `mechanism.py` is adapted from robocurve/benchmark-tasks #223 under its MIT licence (header in the file).
