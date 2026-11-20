#!/bin/bash
# film.sh <lock a|b|c>: the filmed demo, stock inspect-robots with stored frames, via the kit run.sh (out/film_<hash>).
set -eu
cd "$(dirname "$0")/.."
exec bash run.sh film . "${1:?lock a|b|c}"
