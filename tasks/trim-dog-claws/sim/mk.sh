#!/bin/bash
# mk.sh: sim copies of the task in sim/arm/: nail_trim.py as is, plus traced copies (t00 = no delay, t05 = 0.05 s
# start delay; t00b/t00c and t05b/t05c are copies so parallel runs keep separate traces) with log_snippet.py
# appended. Rerun after any task edit.
set -eu
cd "$(dirname "$0")"
rm -rf arm && mkdir arm && cp ../nail_trim.py ../mechanism.py ../task.json arm/
grep -q 'def demo(delay=0.0):' arm/nail_trim.py || { echo "demo() signature changed"; exit 3; }
for s in 00 05; do
  sed "s/def demo(delay=0.0):/def demo(delay=0.$s):/" arm/nail_trim.py > arm/t$s.py
  cat log_snippet.py >> arm/t$s.py
done
# WHY copies: each traced run writes trace_<stem>.csv, so parallel runs need their own stem (flare lesson 2026-11-14)
for c in b c; do cp arm/t00.py arm/t00$c.py; cp arm/t05.py arm/t05$c.py; done
echo MK OK
