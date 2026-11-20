#!/bin/bash
# film.sh <lock a|b|c>: the filmed demo (stock inspect-robots run, yam_scripted, frames stored, cameras 320 x 176 as
# the kit films) PLUS, through env JAW_HD and TOP_CAM_VIDEO, the same run's top, left and right cameras recorded at
# 1280 x 720 under sim/out/hd_<hash>/ for the composed close-up demo.mp4 (see the README).
set -eu
cd "$(dirname "$0")/.."
H=$(cat $(ls *.py | sort) task.json | sha256sum | cut -c1-8)
mkdir -p sim/out/hd_$H
JAW_HD=$PWD/sim/out/hd_$H TOP_CAM_VIDEO=$PWD/sim/out/hd_$H/top_cam_mech.mp4 bash run.sh film . "$1"
