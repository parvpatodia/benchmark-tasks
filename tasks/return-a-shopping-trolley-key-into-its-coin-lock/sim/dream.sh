#!/bin/bash
# dream.sh <lock a|b|c>: the ONE Dream-1 run on these files (the kit's guard refuses a second run per revision).
set -eu
cd "$(dirname "$0")/.."
bash run.sh dream . "$1"
