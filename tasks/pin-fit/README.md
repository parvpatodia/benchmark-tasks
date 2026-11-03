# Pick the pin that fits the hole and seat it

Three steel pins (26, 20 and 23 mm across) stand in front of the left arm; a fixture block with a 22 mm square hole is
fixed in front of the right arm. Only the 20 mm pin fits. Neither arm reaches both the pins and the fixture, so the
left arm has to set a pin down between the arms for the right arm to take and seat. The score rewards seating the
right pin fully, with partial credit for a pin that has entered the hole or been brought next to it.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pin_fit.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Find the pin that fits the hole in the fixture block and push it all the way down into the hole." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pin-fit-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pin_fit.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Find the pin that fits the hole in the fixture block and push it all the way down into the hole." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pin-fit-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/pin-fit-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

## Scoring
| Score | Condition | Check on a real table |
|---|---|---|
| 1.0 | the 20 mm pin in the hole, its bottom within 3 mm of the floor, tilted at most 7 degrees (either end up), still for 1 s | the pin's top stands at most 58 mm above the block's top face (80 - 25 + 3); a phone level on its top |
| 0.5 | the 20 mm pin has entered the hole (its lower end over the hole, its bottom at least 2 mm below the top face, tilted at most 30 degrees) but isn't fully down | its top stands at most 78 mm above the block's top face |
| 0.25 | the 20 mm pin stands upright (at most 10 degrees) on the table or on the block (its bottom within 3 mm of it), its centre within 5 cm of the hole's centre | a ruler from the hole's centre |
| 0 | anything else, or any pin off the table | |

Only the 20 mm pin scores, so a wrong pin, a knocked-over pin or a pin left in the row scores 0. The 23 and 26 mm
pins can't enter a 22 mm hole.

## Runtime and seeds
- The scripted demo seats the pin and the trial ends with success at 23.7 s of simulated time (473 steps). On an L40S
  the whole demo run, including starting Isaac Sim, takes about 70 s once the machine is warm; a full 2,400-step
  trial with Dream-1 takes about 4.5 minutes. Isaac Sim's first start on a machine takes about 6 minutes.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
