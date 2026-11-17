# Trim a dog's black claws with a guillotine clipper

A paw trainer on the left arm's side holds out a silicone foreleg toward the middle of the bench, clamped to its stand,
its paw turned sole up with four long black claws pointing up. The robot picks up a guillotine clipper, brings its
ring down over a claw's end, and squeezes to cut, a little at a time. After each cut the cut face is looked at: when a
pink-grey dot shows in its middle, that claw is done. Cutting on past the dot reaches the quick and the claw bleeds
(score 0). The dot sits at a different length in each claw, drawn for each claw from the seed, so a fixed number of
cuts or a fixed length per claw fails most seeds. All four claws done, the clipper back on its mat and let go of,
scores 1.0.

Category: Societal relatability. Clipping a dog's nails is familiar pet grooming, and owners do it at home. APPA's
2026 Dog Report (its press release) puts dog ownership at "71 million U.S. households in 2025" (53% of households) and
says more than half of owners (51%) "groom their dogs at home". It does not report nail trimming on its own, so how many
of those households clip nails is not known [UNSOURCED]. Häusler and Söhnel (2026) cite a survey of 35 dog owners in
which "48% reported trimming their dog’s nails two to five times annually". WSU's guide is written for owners: "it is a
task you can complete at home". Not every owner does it: AKC notes "Professional groomers will perform the task for
squeamish owners".

In a dark claw the quick cannot be seen through the claw, so it is cut in steps until a cue shows on the cut face. WSU:
stop when "a gray to pink oval starts to appear". Ontario SPCA: "Check the nail after every clip until you can see a
small shiny dot." AKC: "For dogs with dark nails, watch for a chalky white ring." A veterinary physiotherapy center
"trimmed incrementally using a conventional guillotine-style nail clipper", and "Each cut was visually inspected for
the appearance of a central black dot, indicating proximity to the vascularized quick" (Häusler and Söhnel 2026).

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

Saved copies were read in full; every quote here is from them. Tags: [SOURCE] read in a saved source, [ARITH]
arithmetic on tagged numbers, [EST] our estimate, [CHOICE] our choice, [UNSOURCED] no source found.

- American Pet Products Association, "America's Dog Owners Are Becoming Smarter Spenders" (press release on the 2026
  Dog Report, July 22, 2026): "reaching 71 million U.S. households in 2025 (+4% YoY)", "dog ownership continues to
  expand, reaching 53% of U.S. households in 2025", and more than half of owners (51%) "groom their dogs at home". The
  full report was not read; it is sold to members.
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
- American Kennel Club, "How to Trim Your Dog's Nails at Home": "Professional groomers will perform the task for squeamish
  owners, but dog nail trimming is a simple procedure if done correctly.", "For dogs with dark nails, watch for a
  chalky white ring."
- Ontario SPCA, "How to trim your dog's dark nails safely and efficiently": "Keep the clips small.", "Check the nail
  after every clip until you can see a small shiny dot. That is the quick of the nail."
- VCA Animal Hospitals, "How to Trim a Dog's Nails": "Through a series of small clips", removing 1 to 2 mm of nail with
  each clip (VCA's figure, written with a dash there), and for dark nails, "trim just the very tip off the nail".
- Erler-Zimmer VET4750 "Canine claw trimming paw", user manual and FAQ (maker's PDFs): "Hold the paw securely with one
  hand", "Using the other hand, carefully trim the claws", "model allows for practicing identification and avoiding
  this sensitive area."; the FAQ names "dog owners" among its users.
- Clipper: Shoof (a distributor), "Dog Nail Guillotine 9 mm", SKU 202153, a Resco clipper: "9 mm guillotine opening",
  "The dimension is the guillotine opening diameter." Covetrus lists Resco guillotine clippers as "Supplied By: Tecla".
  Tecla's patent US5533262 describes the mechanism: "One blade is a movable cutting blade reciprocated in a single
  cutting stroke by manually squeezing together the handles of the clipper against the force of a biasing spring
  coupled between the handles.", and "the cutting blade should travel for a substantial distance beyond the retainer
  blade opening to insure that the object is severed cleanly." Coastal Pet's own page for its Safari guillotine
  trimmer gives sizes as "Small (5" L x 3" W) Large (5" L x 3" W)".
- Smooth-On, Dragon Skin series technical bulletin: Dragon Skin 30, Shore 30A, tensile strength 500 psi (table row).

Where the sources differ:

- VCA tells owners with dark nails to take off only the very tip; WSU and the paper cut in steps to the cue. This task
  is the WSU and paper method. VCA's advice is the cautious alternative for an owner who does not trust the cue.
- The cue's look: WSU "a gray to pink oval", Ontario SPCA "a small shiny dot", AKC "a chalky white ring", the paper "a
  central black dot". The task draws WSU's: a pink-grey dot in the middle of the black face. AKC's ring is not drawn.
- WSU puts the oval "at the top of the cut surface"; the paper says "central". The dot here is in the middle.
- WSU says "Nails should be cut from underneath at a 45-degree angle"; the paper keeps "a slight distal angle". Here
  the claw points straight up and the clipper is held level, so the cut is square to the claw (WSU's "perpendicular").
- The maker's manual has one hand hold the paw. With the trainer clamped (below), the hold is not needed, and the task
  does not ask for it.

Not sourced (estimates, marked in the code): the claw and toe sizes (no measured source found; the claw's end is 3.6
by 4.4 mm), the leg's length and the trainer's place on the bench, the clipper's length and handle span (chosen to fit
the gripper; no maker or seller we found publishes a guillotine clipper's handle span, blade travel, lever ratio or
spring force, see Real trials), the dot's colour and size (drawn pink-grey, 3.2 mm across, as large as the cut face
allows), and the 2 mm dot zone. No source gives how far before the quick the cue appears in a real claw; WSU's "to not
cut within 2 millimeters of the quick" is advice on where to stop, not that distance. The real trials therefore use
claw tips built with the cue at a set, measured depth (Real trials), so the 2 mm is a property of the tips, not a
claim about dogs.

## The model

- A cut: squeezing the handles slides the blade over the ring's top face. Each handle travels up to 6 mm toward the
  other (12 mm together); a squeeze that takes the blade from open (under 2 mm) past 9.5 mm (across the 9 mm hole)
  cuts once. It cuts each claw whose end is in the hole (its centre within 1.57 mm of the hole's centre, so the end's
  corners clear the ring), with the clipper's up axis within 15 degrees of vertical, at the ring's top face, taking off
  what stands above it. The handles must reopen before the next cut. This is the side away from the toe, as WSU puts
  the blade.
- The claw resists: while a squeeze from open has the blade's edge inside the end of a claw in the ring, each handle
  pushes back 8 N more than its spring (`R_CUT`, an estimate; see Sim against real). The YAM finger's drive (5,000 N/m
  to its target, capped at 40 N, the round 2 reviewer's figure: `GRIP_N` sets yam_arena's per-finger effort, whose
  own default is 50 N) beats the spring plus that by 6.2 N or more at every point of the cut, the claw anywhere in the
  hole (`test_gripper_can_squeeze_through_a_claw`, `test_squeeze_fits_a_40_n_finger`).
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

Round 2 review asked for a trainer cue that is shown, claws prepared repeatably at the hidden lengths, and a named
clipper the YAM can grasp and squeeze within about 40 N per finger. For the VET4750's own claws, its FAQ says
"200 ml of artificial blood can be injected into the claws. If clipped incorrectly, the fluid simulates bleeding", and
its manual that the model "allows for practicing identification" of the quick in dark claws; neither says what a cut
face shows before the fluid, nor where in the claw the fluid channel ends, and we have not tested one. So the trials
replace its claws with cast claw tips whose cue and quick sit at a measured depth.

Hardware:

- Trainer: the Erler-Zimmer VET4750 leg and stand, clamped as in the sim (stand clamped to the table, the leg clamped
  near the paw to a rigid block, the paw sole up). Its own claws are not fitted and its blood system is not filled. A
  printed carrier (PLA, a bar with four sockets 14.0 mm apart on centres) is bolted to the block under the paw so the
  four tips stand up through the toes, in the sim's row. Fit check before each session: pitch 14.0 +- 0.5 mm and
  each tip's end 115 +- 1 mm over the table (calipers, height gauge) [CHOICE]; the carrier has not been trial-fitted to
  a VET4750.
- Claw tips (the consumable, one per claw per trial): cast Dragon Skin 30 silicone, 26 mm long (18 mm showing, an 8 mm
  root in the socket). The top 11 mm, deeper than any cut (the deepest quick is 8.5 mm), has the sim's cut face, 3.6
  by 4.4 mm, the same all the way down; it then flares to 5 by 6 mm at the toe. Three colours, coaxial:
  - shell: black (Silc Pig black), the whole outside;
  - cue: a pink-grey plug, 2.4 mm across and 2.0 mm long, on the axis, its top at depth c below the end;
  - quick: red, 2.4 mm across, from the plug's bottom (depth q = c + 2.0 mm) down to the root.
  A cut face shallower than c is all black; from c to q it shows a pink-grey dot in the middle of the black face (the
  cue); at q or deeper it shows red (a bleed, score 0). The walls round the core are 0.6 and 1.0 mm [ARITH].
- Making a tip [CHOICE]: a printed two-part mould with a 2.4 mm steel pin entering from the root, its end set by a
  printed stop collar at depth c from the end; pour the black shell, cure, pull the pin, leaving a blind bore from the
  root. Pink-grey plugs are cut from one cast 2.4 mm rod in a printed slicing jig. Push one plug to the bore's bottom
  with the pin; fill the rest with red silicone from a syringe; cure.
- Measuring every tip (digital calipers reading to 0.01 mm; the depth rod touched lightly on the silicone): length L;
  end section; bore depth from the root face before the plug, H1, and after it, H2. Then c = L - H1 and q = L - H2.
  Accept a tip only if the end section is 3.60 +- 0.10 by 4.40 +- 0.10 mm, the plug is 2.00 +- 0.10 mm long, and c
  and q are each within 0.15 mm of their targets [CHOICE: with 0.5 mm between seed lengths, 0.15 mm keeps neighbouring
  lengths at least 0.2 mm apart, ARITH]. The measured c and q are written on a label on the root, hidden in the
  socket.
- Seeds: tips are made in stock for each of the 9 quick lengths (q 4.5 to 8.5 mm, c = q - 2.0 mm). For a trial, the
  person who did not make or measure them fits the four tips the seed table names, labels hidden. A trial uses four
  new tips.
- Clipper: a Resco guillotine clipper with the 9 mm ring (Shoof SKU 202153, Sources), its blade replaced every
  session. Sourced: the 9 mm ring [SOURCE], matching the sim's; the one-moving-blade, sprung-handle mechanism
  [SOURCE, US5533262]. Not published by Resco, Tecla, Shoof, Covetrus, VetUK or Tractor Supply, nor by the makers or
  sellers of the other guillotine clippers we checked (Coastal Safari, Jorgensen J0307N, CHI, Kruuse, Hirota):
  the handle span open and closed, the blade travel, the lever ratio and the spring force [UNSOURCED]. So the grasp and
  the squeeze are checked on the clipper itself (C1, C2 below) before any trial; one that fails is not used, and the
  Coastal Safari Large is tried next (its maker's 3 in overall width, 76.2 mm [ARITH], is under the fingers' 94 mm
  opening, so its open handles fit between the pads) [SOURCE, ARITH].
- Protecting the shared rig (for the trial, not part of the task):
  - Gripper pads: each pad carries a replaceable skin of the same rubber, held by double-sided tape, changed at the
    end of each session or when silicone crumbs stick to it. The fingers touch only the clipper's handles; the blade
    runs inside the ring's head, away from them.
  - Work surface: a liner over the bench under the trainer, the mat and the arms' reach, changed each session; a
    shallow tray under the paw catches the clippings.
  - The trainer's stand is clamped to the bench and the leg to its block, so neither can fall onto the rig.
  - Handling: the operator changes the blade and the tips in cut-resistant gloves, with the arms in protective stop;
    clippings are vacuumed from the tray after each trial. No fluid is used.

Checks before any trial, with pass/fail numbers:

- C1, grasp (once per clipper): calipers across the handles' outer faces at the points the pads hold (the grip point
  55 mm behind the ring's centre, as in the sim). Pass: span open 90 mm or less (the fingers' 94 mm less 4 mm
  [CHOICE]), and the blade fully across the ring before the handles meet.
- C2, squeeze force (once per clipper, and after each blade change): one handle clamped, a push-pull force gauge
  reading to 0.1 N pressing the other handle at the pad point, squeeze from open through a tip set at its deepest
  cut (q 8.5 mm, cut 8 mm down); five tips. By statics the force on the pushed handle is the force each of two
  squeezing fingers must give [ARITH]. Pass: the peak, spring included, 35 N or less on all five (the reviewer's 40 N
  per finger less the 5 N margin of `test_squeeze_fits_a_40_n_finger`). Then `R_CUT` is set to the measured peak less
  the spring force at that travel (scaled to the sim's handles) and the offline tests are re-run; trials go ahead only
  if they pass.
- C3, cue (once per material batch): five spare tips (q 4.5, 5.5, 6.5, 7.5, 8.5 mm) cut by hand with the clipper in
  0.5 mm steps, the removed length measured with calipers after each cut and each face photographed with a macro lens
  under fixed light, a 0.5 mm grid in view. Faces within 0.15 mm of c or q are not judged. Pass, on all five: every
  face shallower than c shows no pink-grey or red; every face between c and q shows a pink-grey dot 2.4 +- 0.3 mm
  across, centred within 0.5 mm; the first face past q shows red.
- C4, build (one tip in each pour of at most ten): slit lengthwise with a razor blade, photographed on the grid. Pass:
  c and q within 0.20 mm of that tip's caliper values, and no void through the black wall in the top 11 mm. A fail
  rejects the pour.

The C2 force is bounded but not shown by published data. Dragon Skin 30's tensile strength, 500 psi = 3.45 MPa
[SOURCE], times the 3.6 by 4.4 mm face, 15.84 mm2 [ARITH], is 54.6 N at the blade [ARITH], taking the face to shear at
no more than its tensile strength [EST; a sharp blade parts the face progressively, so the peak is expected lower, but
that is not proven]. Through the clipper that needs a lever ratio of 54.6 / (35 - spring) or more; with the sim's 23 N
spring at its stop, 4.6 [ARITH]. The real ratio and spring are not published, which is why C2 is a gate, not a claim.

How each reading is made:

- Cut depth: each tip's end height over the carrier's top face, by the calipers' depth rod, before and after the run.
- Claw done: cut depth from c to q (the tip's label) and the face, photographed with the C3 set-up, shows the
  pink-grey dot and no red. Within 0.15 mm of c or q the photo decides.
- Bled (score 0): red on any cut face in the photos after the run or in the video at any time, or a cut depth of q or
  more.
- Clipper down: on its mat, still, and no gripper touching it at the end (video).
- Each run's video is kept with the photos and readings.

## Run

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=nail_trim.py -E object_jitter=0 \
  -E control_hz=20 --instruction "$(python3 -c "import json;print(json.load(open('task.json'))['instruction'])")" \
  --scorer task_score --seed 0 --max-steps 3000
```

## Sim results

With each finger capped at 40 N (`GRIP_N` in `nail_trim.py`), inspect-robots, `yam_arena`, 20 Hz, `object_jitter=0`,
one launch each on this revision's committed files (their hashes are in each run's `files.sha256`) [RUN]:

- Filmed demo (`sim/film.sh`, `yam_scripted`, `--seed` 0): FILM_PENDING
- Dream-1 (`sim/dream.sh`, `dream`, `--seed` 0, 3,000 steps, the one guarded run): DREAM_PENDING

Earlier runs, on the round 1 files with yam_arena's default 50 N finger cap (superseded by the two runs above, kept
for the record):

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

## Changes since the runs

The two runs above hash `nail_trim.py`, `mechanism.py` and `task.json` as committed for this revision. After them,
only `task.json`'s `scores` (filled from the runs) and `media` (the new upload) change. Since the earlier 50 N runs:
`nail_trim.py` caps each finger at 40 N (`GRIP_N`, its only change; `mechanism.py` is unchanged), and `task.json`'s
`category` (Impact on jobs to Societal relatability), `context` (the O*NET text replaced by the pet-grooming sources)
and `scores` changed. Its `instruction`, `setup`, `scoring`, `run_limit`, `reset_notes`, `fidelity`, `title` and
`keywords` are byte-identical to the earlier runs', so Dream-1 receives the same instruction and the scene is the
same apart from the finger cap.

## Sim against real

- The paw is rigid and clamped; a dog's toes and the trainer's silicone flex, and a dog moves. The claws are straight
  and stand up; real claws curve, and the cut face is oval.
- PhysX does not cut. A cut claw's tip slides down into the toe by what was cut; its cut face stays flat and its width
  does not grow as it gets shorter.
- The blade is drawn only and the handles slide instead of pivoting, with 12 mm of squeeze [CHOICE]; a real
  guillotine's handle travel and lever ratio are not published, so the sim's clipper is not a model of a named product's
  geometry, and checks C1 and C2 in Real trials measure the real one. The claw's resistance is
  modelled as 8 N on each handle on top of the springs while the blade's edge is inside a claw in the ring; check C2
  in Real trials measures it. A squeeze that cannot push through stops in the
  claw and cuts nothing.
- The quick's lengths are our choice, spread over the 5 to 8 mm the paper's clinic removed on average; the 2 mm dot
  zone is an estimate, not a measurement.
- The claw's push-back: the sim models it as 8 N a handle (`R_CUT`), so a handle needs at most 31 N (the 23 N spring
  at its stop plus 8 N), under the reviewer's 40 N per finger. The real tips are silicone, not keratin; their bound and
  the C2 gate that replaces `R_CUT` with a measurement are in Real trials. The real cue plug is 2.4 mm across, smaller
  than the sim's 3.2 mm dot (56% of its area [ARITH]); the sim's cue is the easier one to see.

## Tests

`python3 test_nail_trim.py` (with yam_arena on PYTHONPATH) runs the offline tests:

- the cut model: a cut takes what stands over the blade; the claw resists only while the blade is through it, and the
  gripper can squeeze through that anywhere in the hole with its drive capped at 40 N, and the task file sets that cap; no cut without the claw in the ring; no cut with the clipper
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
