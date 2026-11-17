# Trim a dog's black claws with a guillotine clipper

A paw trainer on the left arm's side holds out a silicone foreleg toward the middle of the bench, clamped to its stand,
its paw turned sole up with four long black claws pointing up. The robot picks up a guillotine clipper, brings its
ring down over a claw's end, and squeezes to cut, a little at a time. After each cut the cut face is looked at: when a
pink-grey dot shows in its middle, that claw is done. Cutting on past the dot reaches the quick and the claw bleeds
(score 0). The dot sits at a different length in each claw, drawn for each claw from the seed, so a fixed number of
cuts or a fixed length per claw fails most seeds. All four claws done, the clipper back on its mat and let go of,
scores 1.0.

Cutting nails or claws is part of a Core task in three occupations (O*NET task lists, with the 2025 employment O*NET
gives), 628,800 US workers together [ARITH]. Each task names it among other grooming or hygiene duties, and each
occupation also has members whose work does not include trimming claws (for example aquarists in 39-2021.00 and research
animal attendants in 31-9096.00); O*NET gives no head count for those, so the total is an upper bound on
who trims claws, not a count of it:

- 39-2021.00 Animal Caretakers, which includes groomers (365,300): "Perform animal grooming duties, such as washing, brushing, clipping, and trimming coats, cutting nails, and cleaning ears."
- 31-9096.00 Veterinary Assistants and Laboratory Animal Caretakers (132,100): "Perform hygiene-related duties, such as clipping animals' claws or cleaning and polishing teeth."
- 29-2056.00 Veterinary Technologists and Technicians (131,400): "Bathe animals, clip nails or claws, and brush or cut animals' hair."

In a dark claw the quick cannot be seen through the claw, so it is cut in steps. WSU's guide tells the trimmer to stop
when "a gray to pink oval starts to appear". A veterinary physiotherapy center "trimmed incrementally using a conventional guillotine-style nail clipper", and "Each
cut was visually inspected for the appearance of a central black dot, indicating proximity to the vascularized quick"
(Häusler and Söhnel 2026).

What it asks of a robot: seat a claw's end a millimetre or two through a 9 mm ring, square to it, with the ring's head
passing between neighbouring claws 14 mm apart (1.2 mm from the ring to the claw it cuts at the demo's cuts,
`tools/clear_nail.py`); keep the clipper upright (a clipper tilted more than 15 degrees, or upside down, does not cut);
hold that height through a squeeze that pushes the blade through the claw; and judge every cut face before the next cut. It is a one-arm
task: the trainer is clamped, as the maker mounts the paw "on a stable base", so nothing needs holding. Either hand may
do it; both are watched for letting go of the clipper.

Files: `nail_trim.py` (the task), `mechanism.py` (articulated objects, adapted from benchmark-tasks #223, MIT; here it
also builds a loose articulated tool), `test_nail_trim.py` (offline tests), `task.json`, `tools/` (task.json generator,
reach and clearance checks), `sim/` (run scripts).

## Sources

Saved copies were read in full; every quote here is from them.

- Häusler K, Söhnel K (2026). Trimming of nails in healthy dogs does not change gait parameters when comparing pre-
  and post-nail trim. Front. Vet. Sci. doi:10.3389/fvets.2026.1728382 (open access, CC BY). Method: "trimmed
  incrementally using a conventional guillotine-style nail clipper", "For dark-colored nails, trimming was performed
  cautiously in small steps.", "On average, between 5 mm and 8 mm was removed." Citing an earlier survey (its ref. 1):
  "In a survey of 35 dog owners, 71% considered claw trimming necessary for their pets. Among these, 48% reported
  trimming their dog’s nails two to five times annually, while 40% did it even more frequently". The paper studies
  gait; "removed" is from its method, not a measured outcome, and it says the proportion removed was not measured.
- Washington State University Veterinary Teaching Hospital, "How to clip a dog’s nails" (written for owners):
  "it is a task you can complete at home", "Place the tip of the nail in the stationary ring of the trimmer with the
  clipper perpendicular to the nail.", "The cutting blade should be closest to you, not the dog.", "a gray to pink oval
  starts to appear at the top of the cut surface of the nail. Stop cutting the nail at this point as additional cutting
  will cut into the quick and cause bleeding.", "It’s best to not cut within 2 millimeters of the quick.", "The
  guillotine-type is the easiest to use in dogs."
- VCA Animal Hospitals, "How to Trim a Dog's Nails": "Through a series of small clips, removing 1–2 mm of nail with each
  clip", and for dark nails, "trim just the very tip off the nail".
- O*NET OnLine (US Department of Labor), the three occupations' summary and task pages, saved: each quoted task is
  marked Core, with importance 72 (39-2021.00), 75 (31-9096.00) and 65 (29-2056.00).
- Erler-Zimmer VET4750 "Canine claw trimming paw", user manual and FAQ (maker's PDFs): "Hold the paw securely with one
  hand", "Using the other hand, carefully trim the claws", "model allows for practicing identification and avoiding
  this sensitive area."; the FAQ names "dog owners" among its users.
- Clipper: an ozfarmer listing of a Resco guillotine clipper, "The guillotine opening diameter is 9mm." No maker sheet
  was reachable.

Where the sources differ:

- VCA tells owners with dark nails to take off only the very tip; WSU and the paper cut in steps to the cue. This task
  is the WSU and paper method. VCA's advice is the cautious alternative for an owner who does not trust the cue.
- WSU puts the oval "at the top of the cut surface"; the paper says "central". The dot here is in the middle.
- WSU says "Nails should be cut from underneath at a 45-degree angle"; the paper keeps "a slight distal angle". Here
  the claw points straight up and the clipper is held level, so the cut is square to the claw (WSU's "perpendicular").
- The maker's manual has one hand hold the paw. With the trainer clamped (below), the hold is not needed, and the task
  does not ask for it.

Not sourced (estimates, marked in the code): the claw and toe sizes (no measured source found; the claw's end is 3.6
by 4.4 mm), the leg's length and the trainer's place on the bench, the clipper's length and handle span (chosen to fit
the gripper; a real guillotine's handles are wider), the dot's colour and size (drawn pink-grey, 3.2 mm across, as
large as the cut face allows), and the 2 mm dot zone. No source gives how far before the quick the cue appears; WSU's
"to not cut within 2 millimeters of the quick" is advice on where to stop, not that distance, so 2 mm is an estimate to be
replaced by the calibration below.

## The model

- A cut: squeezing the handles slides the blade over the ring's top face. Each handle travels up to 6 mm toward the
  other (12 mm together); a squeeze that takes the blade from open (under 2 mm) past 9.5 mm (across the 9 mm hole)
  cuts once. It cuts each claw whose end is in the hole (its centre within 1.57 mm of the hole's centre, so the end's
  corners clear the ring), with the clipper's up axis within 15 degrees of vertical, at the ring's top face, taking off
  what stands above it. The handles must reopen before the next cut. This is the side away from the toe, as WSU puts
  the blade.
- The claw resists: while a squeeze from open has the blade's edge inside the end of a claw in the ring, each handle
  pushes back 8 N more than its spring (`R_CUT`, an estimate; see Sim against real). The YAM finger's drive (5,000 N/m
  to its target, capped at 50 N) beats the spring plus that by 6.1 N or more at every point of the cut, the claw
  anywhere in the hole (`test_gripper_can_squeeze_through_a_claw`).
- The quick: its end lies 4.5 to 8.5 mm below a claw's end at the start, in 0.5 mm steps, drawn for each claw on its
  own from the seed (table below). A cut face within 2 mm of it shows the pink-grey dot. A cut to or past it bleeds: a
  red drop shows on the cut face, and the score is 0.
- The trainer is fixed: the leg, the paw and the claws' roots are one body with the stand. Each claw's tip slides down
  into its toe by what has been cut.
- The clipper is one loose articulated body: the head and ring on a spine, two sliding handles with return springs
  (20 N preload each), and the blade (drawn only) set each step to the handles' travel together. Both handles slide, so
  a centred squeeze keeps the ring still; a real guillotine has one fixed handle and the hand takes up the difference.
  Its weight is centred at its grip point so it does not pitch in the pads (as benchmark-tasks #1561 did for pliers).
- Letting go: the clipper counts as put down only when it is on its mat, lying flat (its up axis within 15 degrees of
  vertical), still, and each hand's TCP is 200 mm from its grip point. The clipper's farthest point is 73.3 mm from
  its grip point and the fingers' farthest point 101.4 mm from the TCP (`tools/reach.py`), so no finger can touch it.
  A rule on the fingers' opening is not used: the open pads, 94 mm apart, can pinch the clipper end to end. The
  gripper's housing reaches 147.3 mm from the TCP, so from 200 mm it could still press on the clipper, but it has no
  opposing part to hold it with.

## Seeds

Quick length below each claw's end at the start, mm (claws 1 to 4 in a row at x 321, 307, 293, 279 mm, y -5 mm):

| `--seed` | claw 1 | claw 2 | claw 3 | claw 4 |
|---|---|---|---|---|
| 0 | 6.0 | 8.5 | 8.5 | 5.0 |
| 2 | 5.0 | 6.5 | 8.5 | 5.5 |
| 4 | 8.0 | 7.5 | 6.5 | 5.0 |

Each claw's quick is one of 9 lengths, drawn on its own, so there are 6,561 trials. The best fixed length per claw
position, chosen knowing the draw, shows all four dots on under 5% of 2,000 seeds (`test_best_blind_script_rarely_succeeds`).
A blind script that cuts one length off every claw, then puts the clipper down, earns at most 0.20 on average
(`test_blind_script_earns_little`): partial credit is 0.1, 0.2, 0.3 and 0.5 for one to four dots, and it earns 1.0
only on the seeds where all four dots happen to show. A policy
has to cut, look, and decide per claw.

The scripted demo is not a policy: it reads the seed's quicks and plans its cuts from them (1.0 to 1.3 mm each on the
film seeds). It shows the task can be done, not how to find the dot. How much each cut takes is not scored; the 1 to 2
mm in the instruction is VCA's advice.

The cue is small. On the earlier film the dot (2.4 mm across) was about 2 to 3 px in the right wrist camera at its 320
px width; drawn 3.2 mm across it is a third larger, about 2 to 3 px in the policy's 224 px view (2 to 3 x 4/3 x 224/320)
[ARITH, scaled, not measured]. It shows only in the wrist camera, when the ring is lifted off the claw; the overhead camera cannot resolve
it. The pull request includes a zoomed still.

## Real trials

Hardware:

- Trainer: Erler-Zimmer VET4750 with its long black claws (LB), the blood system filled with its fluid as the manual
  says. Its stand is clamped to the table and the leg is clamped near the paw to a rigid block, so the paw is sole up,
  the claws point up, and the paw does not move when cut. The maker shows the paw hanging; turning it is our choice, so
  the cut faces face the overhead and wrist cameras. Whether the blood system works with the paw turned is checked in
  the calibration below.
- Seeds: the claws are consumables (25 long black claws per kit). Before a trial, claws are pre-shortened off camera by
  a hidden amount and refiled to a point, so the quick sits 4.5 to 8.5 mm from each end as in the table above (from
  the calibration below). A judge who did not prepare them runs the trial. Refiling changes the end's shape; the end's
  width after refiling is measured and reported.
- Clipper: a steel guillotine clipper with a 9 mm ring, the blade sharpened or replaced every session.
- Protecting the shared rig (for the trial, not part of the task):
  - Gripper pads: each pad carries a replaceable skin of the same rubber, held by double-sided tape, changed after any
    trial that leaves the trainer's fluid or claw dust on it and at the end of each session. The fingers touch only the
    clipper's handles; the blade runs inside the ring's head, away from them.
  - Work surface: a plastic-backed absorbent liner over the bench under the trainer, the mat and the arms' reach,
    changed each session; a shallow tray under the paw catches the clippings and any fluid.
  - The trainer's stand is clamped to the bench and the leg to its block, so neither can fall onto the rig.
  - Handling: the operator changes the blade and refiles claws in cut-resistant gloves, with the arms in protective
    stop; clippings are vacuumed from the tray after each trial; the fluid is wiped from the claw and the tray after a
    bleed. A trial in which fluid reaches an arm beyond the pads stops the session until that arm is cleaned.

Calibration, before any trial, on spare long black claws with the paw turned as in the trials:

- Section five claws lengthwise and measure where the blood channel ends from the tip (the uncut quick length).
- Cut five more in 0.5 mm steps; photograph each cut face under the same light with a macro lens, and note the step at
  which the channel shows as a dark dot and the step at which fluid comes out. The sim's 2 mm dot zone is replaced by
  the measured one in a run of the offline tests (`DOT`) before the trial, and the seed table by the pre-shortening
  that the sectioned claws give.
- Cutting force: with a push-pull force gauge on the clipper's handle, cut five long black claws at the trial
  lengths and record the peak handle force beyond the bare spring. `R_CUT` is replaced by it (scaled to the sim's
  handles) in a run of the offline tests before the trial; the gauge reads at the point where the gripper's
  pads hold the handles, and if the peak, spring included, is over 45 N (the gripper's 50 N less a 5 N margin
  [CHOICE], as in the offline margin test), the trials are not run with that clipper.
- If the channel never shows before fluid comes out, the trainer cannot test the stop cue. The trials are then not run
  with it; this is the one premise of the real trial no source confirms.

How each reading is made:

- Claw done: after the run, each cut face is photographed with the same macro set-up and judged against the
  calibration photos (dot showing, no fluid); the removed length is measured with calipers against the pre-run length.
- Bled (score 0): a white tissue touched to each claw's end after the run shows fluid, or fluid is seen on the claw in
  the video at any time.
- Clipper down: on its mat, still, and no gripper touching it at the end (video).
- Each run's video is kept with the photos and readings.

## Run

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=nail_trim.py -E object_jitter=0 \
  -E control_hz=20 --instruction "$(python3 -c "import json;print(json.load(open('task.json'))['instruction'])")" \
  --scorer task_score --seed 0 --max-steps 3000
```

## Sim results

inspect-robots 1.0.0, `yam_arena`, 20 Hz, `object_jitter=0`, one launch on the final files (their hashes are in each
run's `files.sha256`) [RUN]:

| Run | Policy | `--seed` | Score | Steps |
|---|---|---|---|---|
| Filmed demo (`sim/film.sh`) | `yam_scripted` | 0 | 1.0 | 1,834 (91.7 s) |
| Traced demo (`sim/run.sh t00b 3000 c 2`, `sim/log_snippet.py` appended) | `yam_scripted` | 2 | 1.0 | 1,764 |
| Dream-1 (`sim/dream.sh`) | `dream` | 0 | 0 | 3,000 (the limit) |

The seed-2 trace: 19 cuts, 0.75 to 1.65 mm each (planned 1.0 to 1.3 mm; a cut is taken where the ring actually sits,
not where the plan put it); every cut met the claw's push-back first (`resist` above 0 on 38 of the traced steps,
which log every other step); each claw stopped 0.49 to 0.78 mm short of its quick (5.0, 6.5, 8.5 and 5.5 mm), its
dot showing, nothing bled; at the end the hand's TCP was 238 mm from the clipper, past the 200 mm let-go radius.

Dream-1 ran all 3,000 steps (0.49 s mean inference latency) and scored 0. In the top camera its left hand goes to the paw from
about 20 s and its right hand reaches the clipper from about 50 to 75 s; the run does not log the model's state, so
whether it cut any claw is not known.

## Sim against real

- The paw is rigid and clamped; a dog's toes and the trainer's silicone flex, and a dog moves. The claws are straight
  and stand up; real claws curve, and the cut face is oval.
- PhysX does not cut. A cut claw's tip slides down into the toe by what was cut; its cut face stays flat and its width
  does not grow as it gets shorter.
- The blade is drawn only and the handles slide instead of pivoting, with 12 mm of squeeze. The claw's resistance is
  modelled as 8 N on each handle on top of the springs while the blade's edge is inside a claw in the ring [EST: no
  measured figure was found]; the calibration in Real trials measures it. A squeeze that cannot push through stops in the
  claw and cuts nothing.
- The quick's lengths are our choice, spread over the 5 to 8 mm the paper's clinic removed on average; the 2 mm dot
  zone is an estimate, not a measurement.
- The claw's push-back (`R_CUT`, 8 N a handle) has no measured source for dog claws. Published cutting energies and fracture
  toughness for keratin run from about 3 kJ/m2 (human fingernail cut transversely, Farren et al. 2004, J Exp Biol 207:735,
  abstract) to 22.8 kJ/m2 (fracture toughness of horse hoof wall at 75% RH, Bertram and Gosline 1987, J Exp Biol 130:121,
  abstract);
  Farran et al. 2008 (J Exp Biol 211:3677, abstract) give 11 to 22 kJ/m2 for fingernails. Across a 4.4 mm claw end, the blade
  force is then 13 to 100 N (R x chord, our estimate, not a measurement), and 18 to 137 N across the 6 mm root. The sim's 8 N a handle is below that low end unless the
  clipper's blade force is at least about 1.7 times the squeeze (13.2 / 8). With the sim's estimated 23 N spring at the
  stop, the YAM's 50 N leaves 27 N for the cut, so the top of that range needs a ratio of at least 3.7 (5.1 at the
  root). No source gives a guillotine clipper's lever
  ratio, a dog claw's size, or the trainer claws' material; the calibration in Real trials measures the real figure, spring
  included, and a trial runs only if it is at most 45 N.

## Tests

`python3 test_nail_trim.py` (with yam_arena on PYTHONPATH) runs the offline tests:

- the cut model: a cut takes what stands over the blade; the claw resists only while the blade is through it, and the
  gripper can squeeze through that anywhere in the hole; no cut without the claw in the ring; no cut with the clipper
  upside down or tilted 20 degrees (10 degrees still cuts); one cut per squeeze; the dot zone and the bleed; the cut
  judged where the claw is;
- the scorer: the ladder; the latched 0 on a bleed; a moving clipper is not down, nor one on its edge; a hand still
  holding the clipper on its mat, or open within reach of it, or pinching it end to end, is not success, with either
  hand, and unread hands are not success; the let-go radius exceeds the clipper's size plus the fingers' reach; a model that stops stepping;
- the demo's planned cuts replayed through the model on every film seed (every dot shows, nothing bleeds, every cut 2
  mm or less), one arm only, and it lets go;
- the hidden quick: the best blind script rarely works and earns little;
- the geometry: the claw fits the ring, the ring stays off the toes at the deepest cut, the pads hold the clipper
  without cutting, the demo's ring centred on each claw it cuts, the dot hidden in the tip until it shows;
- the README's numbers match the code.

`tools/ikcheck.py` (reach), `tools/reach.py` (the let-go radius), `tools/clear_nail.py` (hand, clipper and trainer clearances) and `tools/arms.py` (arm to arm)
check the demo offline.

## Third-party code

`mechanism.py` is adapted from benchmark-tasks pull request #223 (MIT License; the licence is kept in its header).
