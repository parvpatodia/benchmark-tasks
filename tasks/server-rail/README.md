# Push a slide's ball retainer to its front stop, then roll the inner member in between its rollers until it seats

Server and drawer slides have three members. The balls between the middle (intermediate) member and the inner member
ride in a loose cage, the ball retainer. When the inner member comes out, the retainer stays wherever it was. The
technician must push it to the front of the intermediate member before the inner member goes back in:

- Lenovo ThinkSystem SR850 V4, "Install the server to the rack": "Ensure that the ball retainer is at the foremost of
  the intermediate rail before inserting the inner rail into the intermediate rail."
  https://pubs.lenovo.com/sr850-v4/install_the_server_to_rail
- Accuride 3832CDO data sheet (the slide sized here), installation step 4: "Make sure the ball bearing retainers in
  the cabinet side are fully forward. Install drawer into cabinet. Close the drawer."
  https://static.datasheets.com/doc/6720634-accuride-3832-c26-do-ds.pdf

What goes wrong otherwise, from Accuride's patent US 8,500,223 B2: "if the bearing retainer is not maintained in such
a position then misalignment of the inner slide member with respect to the outer slide member during the reinsertion
process may result in inadvertent contact between the inner slide member and the bearing retainer. As the inner slide
member tends to be of a significantly greater thickness than the bearing retainer, this contact may well result in
damage to the bearing retainer." https://patents.google.com/patent/US8500223B2/en

Done by: data-center technicians racking servers; installers fitting drawer and equipment slides.

## The hard part, in numbers

The balls roll, so the retainer travels half as far as the inner member. From its front stop the retainer has just
enough travel for the inner member to reach home (it ends 10 mm before its back stop). It starts 90 mm back. Inserted
like that, the inner member meets the retainer's front rollers 90 mm early. Fed in between them, rolling alone takes
the retainer to its back stop with the member about 70 mm short of home [arithmetic]; past that the stalled rollers
have to skid. In every scripted wrong-order run it struck the front rollers' faces and shoved the retainer 24 mm back
to its stop. With the shipped demo it then stalled 45 mm short of home, sticking out 75 mm; in earlier tuning runs
with the member carried 1 mm lower, 94 mm short, sticking out 124 mm [sim]. Done in order, the same pushes rolled it home with the
retainer moving 0.40 mm per mm of the member [sim, final run]. A retainer up to 20 mm short of its front stop still
lets the inner member home; the score asks for 15 mm.

The robot must:
1. reach a fingertip down through the 26 mm top opening of the channel, behind the retainer's back bar, and push it
   90 mm forward against a drag of about 7 N to its front stop;
2. pick up the inner member (30 x 8 mm, 250 mm long), hold it level and square, and feed its tapered end into the
   mouth at riding height: 1 mm of side play, and the upper rollers dip 1 mm into its path, so a member more than
   2.5 mm high or low meets a roller face instead of the taper;
3. push it 207 mm between the rollers to the stop block (the demo feeds it in gripped at its middle, lets go once two
   roller pairs hold it, and pushes its end home with closed fingertips).

## Scene

Positions are from the midpoint between the arm bases (x out, y left), table top at z 0.

| Object | Type | Size | Material |
|---|---|---|---|
| Intermediate member (fixed) | boxes: web, two walls, two lips | 250 mm long along y from -17 to +8 cm at x 32 cm; channel 32 mm wide inside; walls 21.5 mm tall; lips 23 mm up leave a 26 mm opening | grey steel |
| Front guide (fixed) | box in the mouth | last 10 mm, 4.5 mm tall | dark steel |
| Stop block (fixed) | box | 8 mm long, 22 to 30 mm from the back end, full channel | dark steel |
| Back bracket (fixed) | box | 10 x 36 x 23 mm behind the back end | dark steel |
| Ball retainer (moving, on a slide joint) | cage of 4 boxes over the member's path (17 to 20.5 mm up), 3 roller stations, a drag pad | 93.5 mm long, 29 mm wide; rollers 5 mm diameter, 28 mm long; travel from its start: 90 mm forward, 23.75 mm back | orange steel cage, steel rollers |
| Inner member (loose) | convex hull | 250 x 30 x 8 mm, last 30 mm tapered to 25 x 3 mm; 150 g | blue aluminium (57 cm3; a steel member would be about 450 g) |

The inner member starts flat on the table, centred at x 22 cm, y 20 cm, tapered end toward -y.

## Scoring

`score()` and `success()` see the inner member's pose and speed and the retainer's position and speed (no gripper
state). Riding position: the member's centre within 1.5 mm of the channel's centre line and of riding height (11 mm),
within 1.5 degrees of the channel's line flat and nose up or down. Home: its tapered end within 2 mm of the stop block.
The member meets the retainer when either end is inside the channel's cavity (between the walls, under the lips, between
the back end and the mouth, the member within 15 degrees of the channel's line) and no more than 2 mm ahead of the
cage's front bar. The bar is 9.5 mm ahead of the front rollers, 17 to 20.5 mm up, so a member carried 4.5 mm or more high
strikes it first. Each time an end first meets the retainer, the retainer must be within 15 mm of its front stop. While
the end stays there, the retainer may not fall more than 5 mm behind where rolling from its position at that first
meeting would leave it (half as far as the end has gone past the front rollers since). So shoving it back with an end
against the bar or the rollers' faces, or driving it back by hand, also counts as a miss once it is 5 mm (at the bar)
or 10 mm (at the rollers) out. A member put in square end first is checked the same way and never counts as in riding
position, which needs the tapered end pointing in.

| Outcome | Score |
|---|---|
| Home in riding position, met the retainer forward, member and retainer still (under 1 cm/s and 0.5 rad/s) for 1 s | 1.0, trial ends |
| Same at the last step, still | 0.9 |
| Same, moving | 0.75 |
| Between the rollers in riding position, met the retainer forward | 0.3 to 0.7 by depth from the front rollers to home |
| Retainer within 15 mm of its front stop, or met forward (member since pulled out or held off line) | 0.2 |
| The member ever met the retainer while it was back | 0 for the rest of the trial |
| Anything else | 0 |

## Run

```
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=server_rail.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$(python3 -c "import json;print(json.load(open('task.json'))['instruction'])")" \
  --scorer task_score --max-steps 2400
```

`server_rail.py` imports `mechanism.py` (the retainer as an articulation; adapted from robocurve/benchmark-tasks
#223, MIT) and registers `server_rail_inner.usda` (written by `make_meshes.py`). Offline checks:
`python -m pytest -q test_server_rail.py` (scoring, layout, demo reach, finger clearances). Run limit 2,400
steps; the scripted demo takes 39.8 s (left arm; right arm 12.5 s). One seed; object_jitter 0.

## Results

- Dream-1, 2,400 steps: 0. The run used the same scene, meshes, retainer and docstring as these files, scored before
  two changes to the order memory: the rolling allowance now counts from the first meeting, and a meeting starts at the
  cage's front bar instead of the front rollers. In top-camera frames every 7.5 s the member stays at its start on the
  table, so neither change can raise its 0.
- Scripted demo, official runner, these files: 1.0, success at step 709 (the filmed run).
- Logging copies of the same files (score() also writes poses): started 0, 0.05 and 0.10 s late, 1.0 at steps 698,
  734 and 734. Left arm aimed 1 mm to -x: 0.69, the end push stopped 4.8 mm short of home (a known margin; aiming the
  push further past home fixed it but left the member still moving in 3 of 9 runs, so it was not kept). Right arm
  parked (member first): 0, stalled 45 mm short.

## What is simplified

- The intermediate member lies on its side, open side up, fixed to the table; the outer member is left out. The real
  channel's curled edges are boxes.
- Each pair of balls is one 5 mm roller across the member, at 3 stations. Each upper roller is pressed down with about
  40 N, so a stalled roller pair skids only above 48 N. The real balls are a close fit; the 40 N is sized for the arm.
- The retainer's drag (fit and grease on the real slide) is a pad pressed on the web: about 7 N to slide it.
- The stop block sits 30 mm short of flush (the real member closes flush) so the gripper can push the member's end.
- Estimated, not read: the member's 30 x 8 mm section, its 150 g, the channel's 32 mm width, the 5 mm balls, the
  retainer's size and travel, preload and drag. Read: 250 mm length (3832-C10 data sheet).
- All contacts have friction 1.2 static, 1.0 dynamic.
- The demo aims the fingertip 50 mm past the retainer's front stop and the closed tips 80 mm past home, because the
  position-controlled arm lagged its target by about 35 mm against these loads in simulation; it presses against each
  stop with whatever force that lag gives. On a real arm the push force is bounded only by its gains and current
  limits, so a real-rig demo should aim shorter or limit current.

## Real rig

The order memory and the 1 s stillness need video of the whole trial. A wrong order also shows in a single end photo:
the member stalled sticking out 75 to 124 mm past the mouth instead of 30 mm (our runs).
