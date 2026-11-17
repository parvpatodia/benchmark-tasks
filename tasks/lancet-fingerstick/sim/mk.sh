#!/bin/bash
# mk.sh: sim copies of the task in sim/arm/: lancet_fingerstick.py as is, plus traced copies (t00 = no delay, t05 = 0.05 s
# start delay; t00b/t00c and t05b/t05c are copies so parallel runs keep separate traces) with log_snippet.py
# appended, and two traced one-arm copies with one_arm.py appended (o00: the demo's press with the left arm parked;
# j00: the same with a 0.15 s jab). Rerun after any task edit.
set -eu
cd "$(dirname "$0")"
rm -rf arm && mkdir arm && cp ../lancet_fingerstick.py ../mechanism.py ../task.json arm/
grep -q 'def demo(delay=0.0):' arm/lancet_fingerstick.py || { echo "demo() signature changed"; exit 3; }
for s in 00 05; do
  sed "s/def demo(delay=0.0):/def demo(delay=0.$s):/" arm/lancet_fingerstick.py > arm/t$s.py
  cat log_snippet.py >> arm/t$s.py
done
# WHY copies: each traced run writes trace_<stem>.csv, so parallel runs need their own stem
for c in b c; do cp arm/t00.py arm/t00$c.py; cp arm/t05.py arm/t05$c.py; done
cat arm/lancet_fingerstick.py log_snippet.py one_arm.py > arm/o00.py
sed 's/^_JAB = False$/_JAB = True/' arm/o00.py > arm/j00.py
grep -q '^_JAB = True$' arm/j00.py || { echo "j00 not a jab"; exit 3; }
echo MK OK
