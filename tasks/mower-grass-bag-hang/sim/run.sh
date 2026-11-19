#!/bin/bash
# run.sh <stem> <max steps> <lock: a|b|c> [cli seed, default 0]: one yam_scripted run of sim/arm/<stem>.py (a start-delay copy made by mk.sh),
# no frames. Refuses a copy older than ../mower_bag.py (rerun mk.sh).
set -eu
cd "$(dirname "$0")"
# WHY env vars: ARENA_ENV (the simulator environment file) and LOCKS/LOCK_BASE (the shared GPU lock files) differ per
# machine; the defaults are the shared instance's.
source "${ARENA_ENV:-/data/IsaacLab-Arena/yam.env}" >/dev/null 2>&1 || true
J=$(cd arm && pwd); F=$1; M=$2; S=${4:-0}; O=${F}_s$S
B=${LOCKS:-$HOME/locks}/${LOCK_BASE:-sim.lock}
case $3 in a) L=$B;; b) L=$B.b;; c) L=$B.c;; *) echo "lock a|b|c"; exit 2;; esac
[ "$J/$F.py" -nt ../mower_bag.py ] || { echo "STALE: $F.py older than mower_bag.py; rerun mk.sh"; exit 3; }
I=$(python3 -c "import json;print(json.load(open('$J/task.json'))['instruction'])")
mkdir -p "$PWD/out"
while [ "$(nvidia-smi --query-compute-apps=pid --format=csv,noheader | wc -l)" -ge "${SIMCAP:-3}" ]; do sleep 20; done
T0=$(date +%s)
flock "$L" inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=$J/$F.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "$I" --scorer task_score \
  --seed $S --max-steps $M --log-dir $PWD/out/$O > $PWD/out/$O.log 2>&1 || true
echo "$O $(grep -h task_score out/$O.log | tail -1) steps $(python3 -c "import json,glob,os;f=[g for g in glob.glob('out/$O/*.json') if not g.endswith('.live.json')];print(json.load(open(max(f,key=os.path.getmtime)))['stats']['total_steps'])" 2>&1 | tail -1) wall $(( $(date +%s) - T0 )) s"
echo RUNDONE $O
