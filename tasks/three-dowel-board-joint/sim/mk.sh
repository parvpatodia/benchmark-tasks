#!/bin/bash
# mk.sh: sim copies of the task in sim/arm/: dowel_joint.py as is, plus traced copies (log_snippet.py appended).
# t00 / t00b: the demo (bench start, seeds via the CLI). Probe (the fit alone, boards start in line 30 mm apart):
# p_fit (sim clearance 0.20 mm a side), p_off (hole board end 0.5 mm aside, turned 1 deg about it), p_tight (0.10 mm),
# p_loose (0.35 mm). Stems (expected scores in test_dowel_joint.py): s_nothing, s_half, s_offset, s_skew, s_left_only,
# s_relay, s_stack, s_hold. Rerun after any task edit.
set -eu
cd "$(dirname "$0")"
rm -rf arm && mkdir arm && cp ../dowel_joint.py ../mechanism.py ../task.json arm/
for k in 'START = "bench"' 'VARIANT = "demo"' 'CLR = 0.0002 '; do
  grep -q "^$k" arm/dowel_joint.py || { echo "line '$k' changed; fix mk.sh"; exit 3; }
done
mkcopy(){  # mkcopy <stem> <start> <variant> <clr>
  sed -e "s/^START = \"bench\"/START = \"$2\"/" -e "s/^VARIANT = \"demo\"/VARIANT = \"$3\"/" \
      -e "s/^CLR = 0.0002 /CLR = $4 /" arm/dowel_joint.py > arm/$1.py
  cat log_snippet.py >> arm/$1.py
}
mkcopy t00 bench demo 0.0002
mkcopy t00b bench demo 0.0002
mkcopy p_fit seam demo 0.0002
mkcopy p_off seam off 0.0002
mkcopy p_tight seam demo 0.0001
mkcopy p_loose seam demo 0.00035
for v in nothing half offset skew left_only relay stack hold; do mkcopy s_$v bench $v 0.0002; done
echo MK OK
