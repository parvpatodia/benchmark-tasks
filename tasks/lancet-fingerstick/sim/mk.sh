#!/bin/bash
# mk.sh: sim copies of the task in sim/arm/: lancet_fingerstick.py as is, plus traced copies (t00 = no delay, t05 = 0.05 s
# start delay; t00b/t00c and t05b/t05c are copies for parallel runs) with log_snippet.py appended, and the traced stems
# with stems.py appended (o00 one arm, j00 one-arm jab, w00 one-arm press near the apex driven toward the wrist, f00 a
# press in the stand after the twist, k00 the lancet held in the container). Rerun after any task edit.
set -eu
cd "$(dirname "$0")"
rm -rf arm && mkdir arm && cp ../lancet_fingerstick.py ../mechanism.py ../task.json arm/
grep -q 'def demo(delay=0.0):' arm/lancet_fingerstick.py || { echo "demo() signature changed"; exit 3; }
for s in 00 05; do
  sed "s/def demo(delay=0.0):/def demo(delay=0.$s):/" arm/lancet_fingerstick.py > arm/t$s.py
  cat log_snippet.py >> arm/t$s.py
done
for c in b c; do cp arm/t00.py arm/t00$c.py; cp arm/t05.py arm/t05$c.py; done
for k in o j w f k; do
  cat arm/lancet_fingerstick.py log_snippet.py stems.py | sed "s/^_STEM = \"none\"$/_STEM = \"$k\"/" > arm/${k}00.py
  grep -q "^_STEM = \"$k\"$" arm/${k}00.py || { echo "${k}00 stem not set"; exit 3; }
done
echo MK OK
