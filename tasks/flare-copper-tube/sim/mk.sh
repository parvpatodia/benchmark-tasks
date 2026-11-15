#!/bin/bash
# mk.sh: sim copies of the task in sim/arm/: flare_tool.py as is, plus traced copies (t00 = no delay, t05 = 0.05 s
# start delay) with log_snippet.py appended. Rerun after any task edit.
set -eu
cd "$(dirname "$0")"
rm -rf arm && mkdir arm && cp ../flare_tool.py ../mechanism.py ../task.json arm/
grep -q 'def demo(delay=0.0):' arm/flare_tool.py || { echo "demo() signature changed"; exit 3; }
for s in 00 05; do
  sed "s/def demo(delay=0.0):/def demo(delay=0.$s):/" arm/flare_tool.py > arm/t$s.py
  cat log_snippet.py >> arm/t$s.py
done
echo MK OK
