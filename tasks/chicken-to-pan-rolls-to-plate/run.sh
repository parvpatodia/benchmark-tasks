#!/bin/bash
# robocurve-kit run.sh 61f518bb8eb0
# run.sh fast|film|dream <task dir> <lock a|b|c> [extra fastrun.py args]: the one sim launcher for a kit task (box).
#   fast   fastrun.py: every mode in one session, cameras off, GPU physics (add --fast for CPU), out/fast_<hash>_<time>
#   film   the filmed demo: stock inspect-robots run --policy yam_scripted --store-frames, out/film_<hash> (once per hash)
#   dream  the ONE Dream-1 run on these files, out/dream_<hash> (once per hash: the folder is made before the run)
# <hash>: sha256 over the task's *.py and task.json, 8 hex, so outputs name the revision they ran. Guards: exclusive
# flock on $LOCKS/sim.lock[.b|.c] (LOCKS default ~/parv/locks), then a wait while nvidia-smi lists >= SIMCAP (default 3)
# compute apps; film and dream refuse under MIN_FREE_GB (default 5) free disk and refuse an existing output folder.
# The task file is the folder's one *.py that is not a kit file or a test (or $TASK_PY). Ends with RUNDONE <name>.
# Env: ARENA_ENV (default /data/IsaacLab-Arena/yam.env), ARENA_PY (default /data/IsaacLab-Arena/.venv/bin/python),
# DREAM_URL (optional: passed to the dream policy as -P server_url=..., untested).
set -eu
MODE=${1:?fast|film|dream}; TASK=$(cd "${2:?task dir}" && pwd); LK=${3:?lock a|b|c}; shift 3
source "${ARENA_ENV:-/data/IsaacLab-Arena/yam.env}" >/dev/null 2>&1 || true
PY=${ARENA_PY:-/data/IsaacLab-Arena/.venv/bin/python}
B=${LOCKS:-$HOME/parv/locks}; mkdir -p "$B"
case $LK in a) L=$B/sim.lock;; b) L=$B/sim.lock.b;; c) L=$B/sim.lock.c;; *) echo "lock a|b|c"; exit 2;; esac
cd "$TASK"
if [ -n "${TASK_PY:-}" ]; then F=$TASK_PY; else
  F=$(ls *.py | grep -v -E '^(mechanism|motionkit|checkkit|fastrun)\.py$|^test_' || true)
  [ "$(echo "$F" | grep -c .)" = 1 ] || { echo "REFUSED: want one task .py in $TASK, found: $F (set TASK_PY)"; exit 2; }
fi
[ -f task.json ] || { echo "REFUSED: no task.json in $TASK"; exit 2; }
H=$(cat $(ls *.py | sort) task.json | sha256sum | cut -c1-8)
I=$(python3 -c "import json;print(json.load(open('task.json'))['instruction'])")
STEPS=$(python3 -c "import json,re;print(int(re.search(r'[\d,]+',json.load(open('task.json'))['run_limit']).group().replace(',','')))")
mkdir -p sim/out; [ -f sim/.gitignore ] || printf 'arm/\nout/\n' > sim/.gitignore
case $MODE in
  fast) O=sim/out/fast_${H}_$(date -u +%m%d_%H%M%S);;
  film|dream) O=sim/out/${MODE}_$H
    [ -e "$O" ] && { echo "REFUSED: $O exists ($MODE already ran or started for this revision)"; exit 4; }
    FREE=$(df -Pk sim/out | awk 'NR==2{print int($4/1048576)}')
    [ "$FREE" -ge "${MIN_FREE_GB:-5}" ] || { echo "REFUSED: ${FREE} GB free under sim/out (MIN_FREE_GB ${MIN_FREE_GB:-5})"; exit 5; }
    if [ $MODE = dream ]; then [ -n "${DREAM_URL:-}" ] || [ -f ~/.dreamlabs/credentials ] || { echo "REFUSED: no ~/.dreamlabs/credentials"; exit 2; }; fi;;
  *) echo "mode fast|film|dream"; exit 2;;
esac
mkdir -p "$O"; O=$(cd "$O" && pwd)
sha256sum *.py task.json > "$O/files.sha256"
if [ $MODE = fast ]; then
  # WHY no flock here: fastrun.py takes the same lock and runs the SIMCAP wait itself, holding the lock while it waits
  LOCKS=$B "$PY" fastrun.py "$TASK/$F" "$O" --lock "$LK" "$@" > "$O/run.log" 2>&1 || true
  grep -h "RESULT\|DONE" "$O/run.log" | sed 's/^\[fastrun\] //'
  echo RUNDONE fast; exit 0
fi
exec 9>>"$L"
flock 9
# WHY: the team Dream bridge serves one run at a time; teammates queue on ~/dream.lock, so do the same
if [ $MODE = dream ] && [ -n "${DREAM_URL:-}" ]; then exec 8>>"$HOME/dream.lock"; flock 8; fi
while [ "$(nvidia-smi --query-compute-apps=pid --format=csv,noheader | grep -c .)" -ge "${SIMCAP:-3}" ]; do sleep 20; done
inspect-robots --version > "$O/version.txt" 2>&1 || true
POL=yam_scripted; EXTRA=()
if [ $MODE = dream ]; then POL=dream; [ -n "${DREAM_URL:-}" ] && EXTRA=(-P "server_url=$DREAM_URL"); fi
inspect-robots run --embodiment yam_arena --policy $POL ${EXTRA[@]+"${EXTRA[@]}"} -E task="$TASK/$F" -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "$I" --scorer task_score --max-steps "$STEPS" \
  --store-frames --log-dir "$O" > "$O/run.log" 2>&1 || true
for log in "$O"/*.json; do case $log in *.live.json) ;; *) inspect-robots video "$log" --out "$O/cams" >> "$O/run.log" 2>&1 || true;; esac; done
echo "$MODE $H $(grep -h task_score "$O/run.log" | tail -1) steps $(python3 -c "import json,glob;f=[x for x in sorted(glob.glob('$O/*.json')) if not x.endswith('.live.json')];print(json.load(open(f[-1]))['stats']['total_steps'])" 2>&1 | tail -1)"
echo RUNDONE $MODE
