# Register a workpiece against a corner stop

An aluminium block (60 x 40 x 40 mm) lies in front of the left arm, turned 20 degrees counterclockwise (seen from
above); a fixed L-shaped stop (two rails, 12 mm tall) stands in front of the right arm. The block must end up flat in
the stop's corner, pressed against both faces and square to them. Neither arm reaches both the block's start and the
stop, so the left arm has to set the block down between the arms for the right arm to take. The score rewards the
smallest gaps, with partial credit at 3 mm and 10 mm.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=corner_stop.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the aluminium block flat in the corner of the L-shaped stop, pressed against both stop faces and square to them." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/corner-stop-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=corner_stop.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the aluminium block flat in the corner of the L-shaped stop, pressed against both stop faces and square to them." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/corner-stop-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/corner-stop-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

## Scoring
A gap is measured at both ends of the block's edge that faces a stop; the largest of the four gaps counts.

| Score | Condition | Check on a real table |
|---|---|---|
| 1.0 | flat on one of its 60 x 40 mm faces (tilted at most 2 degrees, within 1 mm of the table), every gap at most 1 mm, square to the stop within 2 degrees, still for 1 s | a 1 mm feeler gauge doesn't enter at any of the four points |
| 0.5 | flat, every gap at most 3 mm, square within 5 degrees | a 3 mm gauge doesn't enter |
| 0.25 | flat on the table, every gap at most 10 mm | a ruler |
| 0 | anything else, including the block on its 40 x 40 mm end, on a rail or off the table | |

Squareness is measured as the angle between the block's sides and the stop's faces. Sim contacts let a pressed block sit
up to 0.5 mm into a stop, so a gap down to -0.5 mm counts as touching; for 0.5 and 0.25 a block still being pushed may
sit up to 5 mm in. The hooks don't see the grippers, so a block held still in the corner, within 1 mm of the table,
also counts.

## Runtime and seeds
- The scripted demo registers the block and the trial ends with success at 29.4 s of simulated time (588 steps). On
  an L40S the whole demo run, including starting Isaac Sim, took 77 to 151 s; a full 2,400-step trial with
  Dream-1 took about 4.5 minutes. Isaac Sim's first start on a machine takes about 6 minutes.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
