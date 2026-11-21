#!/bin/bash
# run.sh <modes|all> <max steps> <lock a|b|c> <seeds>: fastrun (GPU physics, cameras off) on the task file, for
# gpu-share/rr.sh's run:<stem>:<steps>:<lock>:<seed> jobs. <modes>: a comma list or "all" (every MODES entry);
# <seeds>: a comma list such as 0,1,2. Output: sim/out/fast_<hash>_<time>/<mode>[_s<seed>]/result.json + trace.csv.
set -eu
cd "$(dirname "$0")/.."
MODES=$1; STEPS=$2; LK=$3; SEEDS=${4:-0}
A=(--seeds "$SEEDS" --max-steps "$STEPS" --trace)
[ "$MODES" = all ] || A+=(--modes "$MODES")
bash run.sh fast . "$LK" "${A[@]}"
