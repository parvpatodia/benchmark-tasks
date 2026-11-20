# Copy a door casing's floor profile with a contour gauge and lay it on the vinyl to mark the cut

**Instruction:** "Copy the door casing's profile at the floor with the contour gauge, then lay the gauge flat on the
vinyl sheet with the copied fingertips where the sheet must be cut to fit around the casing, and take both hands away."

## What it tests

A floor covering has to be cut around a door casing where it meets the floor. The fitter lays a contour gauge flat on
the floor, pushes it square into the casing's foot so its sliding fingers take the casing's outline, draws it back
without disturbing a finger, and lays it flat on the piece to be cut, in register, so the fingertips sit exactly on the
cut line, ready to trace. The skill is a transfer in register: press a many-part, friction-held tool square against a
fixed profile, keep the copy through a lift and a carry, and set it down where the copy becomes the right cut. Nearby
board tasks transfer a single number (a bevel angle, a scribe compass width, a pencil line from a chalk box); here the
whole outline travels in the tool and every finger is judged where it lands on the sheet.

Floor layers have the Core task "Cut flooring material to fit around obstructions." (O*NET 47-2042.00, importance 93).
Carpenters, 889,700 employed (O*NET 47-2031.00 summary, "Employment (2025) 889,700 employees"), have the Core task
"Measure and mark cutting lines on materials, using a ruler, pencil, chalk, and marking gauge." (importance 76). The
gauge's own listing names this use: "fitting carpet, tiles or linoleum against and around door casings and pipes".

## Why two hands

The gauge can be held only by its two solid end blocks: everywhere between them its fingers stick out of the frame, 8 to
40 mm in front and 16 to 48 mm behind, so a gripper closing across the frame would push fingers, and the frame is 252 mm long
along the other axis. Pushed in from one end block, the fingers' hold (31 fingers in contact, about 4.7 N in all,
Forces) acts at their centroid, 0.17 m from the right end block and 0.11 m from the left, so one hand alone must hold
0.5 to 0.8 N m of turn through a pinch on a 30 mm block while the fingers are being set. Tried in the simulator
(mode `one_arm`: the right hand alone, same moves): the frame turned 3 deg and stopped 6 mm short of the copy, so the
fingers over the wall never reached it, and the gauge then hung 6 deg off level from its one end in the carry. No copy,
score 0. That is one scripted try, not a proof that no one-handed way exists.

## The real objects

| part | real (source, saved in `src/`) | model |
|---|---|---|
| contour gauge | General 833, 10 in, plastic: "3 in. deep fingers can measure profiles up to 1-1/4 in. long" (distributor listing `penntool_833.txt`; the maker's own site refused automated fetches, HTTP 429) | frame 252 x 20 x 12 mm [EST], 42 fingers 76.2 mm long and 4 mm wide [EST] at 6 mm pitch, 31.8 mm travel, fingertips 40 mm out of the frame at rest [EST], two 30 mm end blocks [EST], about 100 g in all [EST]; measure the estimates on the real gauge with calipers and a kitchen scale |
| casing | made from 1x3 pine stock, 3/4 x 2-1/2 in dressed, 19 x 64 mm (`lumber_wikipedia.txt`, North American dressed sizes), cut as below | three fixed boxes, 70 mm tall, on a wall and jamb mock-up |
| vinyl sheet | an offcut of 1/8 in sheet vinyl or vinyl tile, cut to 135 x 170 mm, with 1/2 in masking tape on the wall line | loose box 135 x 170 x 3.2 mm, 120 g [EST: weigh it] |

**The casing, made the same way every time.** Crosscut a 70 mm length of 1x3 pine (19.1 x 63.5 mm). On a table saw,
cut a 1/4 x 1/4 in (6.4 x 6.4 mm) rabbet along its doorway edge and a 5/8 in wide, 3/8 in deep (15.9 x 9.5 mm) rabbet
along its wall edge, both on the show face. Its floor outline, from the doorway edge: 6.4 mm wide at 12.7 mm from the
wall, 41.2 mm wide at 19.1 mm, 15.9 mm wide at 9.6 mm. Check it with calipers (depth rod from the back face) at 3, 27 and
56 mm from the doorway edge: 12.7, 19.1 and 9.6 mm, each within 0.5 mm. Then press the real gauge into it and read the
same three stations off the pins with a steel rule against the frame: the steps between them must read 6.4 mm and
9.5 mm within 0.5 mm (the real gauge's pins are much finer than the model's 6 mm pitch [EST: count them over 25 mm], so they read every step).

**The fitting place.** A wall block, 120 mm deep and 70 mm tall, is fixed to the bench top (the floor), its face 400 mm in front of
the arm bases; the casing is glued to it with its doorway edge 3 mm past the board's end (the jamb's edge and the
reveal), and the doorway is open floor to the right of it. The casing is reached only from the room side, at floor
level, square to the wall. The mock-up is 70 mm tall because the YAM's own links reach past its hand at the copy (the
gripper housing 9 mm past the wall's face from 85 mm up, link 4 59 mm past it from 156 mm up, `checkkit.py`); a full
wall would block the arm, so a real trial uses this same mock-up. The sheet lies 160 mm straight back from where it will
be laid, its doorway edge in line with the casing's doorway edge; the tape's near edge is where the wall line falls.
The sheet is the piece that ends at the doorway edge (a threshold covers the joint), so its cut follows the casing and
the wall only. Every constraint on the gauge is kept: the floor, the casing's face, the open doorway beside it, and an
approach from the room side only, square to the wall. What the stub drops is a wall above 70 mm, which constrains the
arm, not the gauge.

## Forces

| quantity | value | basis |
|---|---|---|
| force to slide one finger | 0.15 N | Estimated, not measured. Measure: lay the gauge flat, push one finger along its slot with a 0 to 2 N spring scale, read the peak; repeat on 5 fingers |
| push to copy | about 4.7 N (31 fingers that meet casing or wall, x 0.15 N) | arithmetic; with both hands, about 2.4 N each |
| friction, gauge plastic on pine and vinyl | 0.40 static, 0.35 sliding | Estimated, not measured. Measure: set the gauge on a pine board and on the vinyl, tilt until it slides (static = tan of the angle), or drag it with the spring scale |
| friction, vinyl on the bench | 0.50 static, 0.45 sliding | Estimated, not measured; measured the same way |
| grip | 15 N a finger on each end block | demo; through the press the trace shows each hand within 1.6 mm of its end block along the push and 0.4 mm across it, with the gauge under 1.2 deg off level (no slip) |

If the measured finger force is up to 3 times the estimate (14 N push), each hand still pushes 7 N; the real trial
starts with the measurement, and the model's F_HOLD is set to it.

## How it is modelled

The gauge is one loose articulation: the frame and end blocks, and 42 fingers on prismatic joints (stops at 0 and 31.8
mm). A finger holds by a 400 N/m spring whose target follows the finger when it is pushed past 0.15 N (a Coulomb-like
hold, `slip()`, run after each control step); a released finger eases back 0.375 mm (0.15 N on 400 N/m), the same for every finger. The target moves only at
the 20 Hz control step, so a finger pushed at 10 mm/s resists with about 0.35 N in the model, about twice the
stated hold. The 6 mm finger pitch is coarser than the
real gauge; every feature of the casing is at least one finger wide. Simplified: no pencil trace (the task ends with
the gauge set in place, ready to trace), the tape is drawn only, the wall is a stub (above).

## Scoring

Judged at the end. A finger's push is read from its joint; a fingertip is judged where it lies on the sheet.

| Score | Condition (all numeric) |
|---|---|
| 0 | no copy. A copy means: over the casing and 20 mm of wall past it, every finger pushed at least 1 mm, and the pushes match the casing's outline within 3 mm at every finger, at the best place along the gauge and after a straight-line fit (a gauge turned up to 5 deg while copying still counts) |
| 0.25 | a copy, but the gauge not lying flat and still on the sheet: z axis within 3 deg of vertical, frame bottom within 2 mm over the sheet's top, frame centre over the sheet, both moving under 0.02 m/s |
| 0.5 | flat on the sheet, but some fingertip on the sheet more than 6 mm (1/4 in) from the cut line, or fewer than 26 fingers judged on it |
| 0.75 | every fingertip on the sheet within 6 mm of the cut line, but some over 3 mm, or a hand within 20 mm of the gauge |
| 1.0 | every fingertip on the sheet within 3 mm (1/8 in) of the cut line, both hands at least 20 mm from the gauge |

The cut line, in the sheet's own frame: the tape's near edge where the sheet meets bare wall, and that edge set back by
the casing's outline (12.7, 19.1, 9.6 mm) over the first 63.5 mm from the sheet's doorway edge. Each fingertip is judged by its distance, on the sheet, to that line with its
steps drawn in, so a tip 1 mm beside a step is 1 mm off. Fingers judged: those whose tip centres lie 2.5 mm (over half a
finger) or more inside the sheet's side edges. Real trial: photograph from above with a steel rule on the tape edge, or measure each tip's distance from
the tape edge with the rule (0.5 mm graduations, so it separates 3 mm from 6 mm); the copy check uses the same rule
against the frame. Thresholds: 3 mm is 1/8 in, half the model's finger pitch; 6 mm is 1/4 in, what a cut would leave as
a visible gap at the casing.

## Run it

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=contour_gauge.py -E object_jitter=0 \
      -E control_hz=20 --instruction "<instruction from task.json>" --scorer task_score --max-steps 1200

One fixed layout, `object_jitter=0`, no seed needed. The demo reads only scene constants and the arms' own joints.
Modes (environment variable TASK_DEMO), each scored on GPU in one simulator session:

| mode | what it does | score |
|---|---|---|
| demo | both hands copy the casing, carry the gauge back and lay it on the sheet in register | 1.0 |
| nothing | no motion | 0 |
| one_arm | the right hand alone, same moves | 0 |
| no_press | the demo without the press, the gauge laid on the sheet uncopied | 0 |
| off_line | the demo laid 12 mm sideways on the sheet | 0.5 |

The copy works for any frame-front to wall distance from 27.5 to 38.5 mm (fingers neither bottomed nor short of the
wall); the demo aims at 32 mm.

Offline: `PYTHONPATH=<yam_arena source> python3 test_contour_gauge.py` (scoring tiers, fakes, the hold model, the
copy window, gates). All modes in one simulator session: `fastrun.py contour_gauge.py OUT --trace`.

## Sim results

yam_arena, `object_jitter=0`, 20 Hz, 1,200-step limit, stock `inspect-robots run` with stored frames. The filmed scripted demo scored 1.0, success at step 651 (32.55 s), 396 s of wall time on a shared GPU. The one Dream-1 run scored 0 over all 1,200 steps. Both runs hashed `contour_gauge.py`, `mechanism.py` and `task.json` before starting, and the hashes match the files here (`task.json` as it was before its `scores` line was filled in). The mode scores in the table above come from one GPU session with cameras off (`fastrun.py`).

## Origin

The idea comes from an earlier submission whose review was not accepted and whose 48 h revision hold lapsed with no revision. This implementation, its scene, scorer, demo and text, is new, and answers each point of that review: the gauge lies flat with its copied tips on the sheet in the marking position, the score checks where the copied contour sits for the cut rather than the gauge's pose, the casing stands in its doorway with its approach constraints, the casing profile is a stocked part, and finger friction and press force are stated with how to measure them.

## Runtime

The demo ends at 651 control steps (32.6 s of the 60 s limit). In the simulator on one GPU it runs at about 2 steps a
second, so a trial takes about 5.5 minutes plus about 20 s for the reset. A real trial takes about 2 minutes, plus
resetting the gauge (push every finger back flush against a straight edge) and the sheet.
