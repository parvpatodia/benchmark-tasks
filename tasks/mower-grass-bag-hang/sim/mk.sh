#!/bin/bash
# mk.sh: sim copies of the task in sim/arm/: mower_bag.py as is, plus traced copies (log_snippet.py appended), one per
# stem: t00 (the demo; seeds via the CLI) and s_nothing, s_one_arm, s_one_hook, s_tips, s_ground, s_hold, s_prop,
# s_pinch, s_slam, s_nudge, s_shove (expected scores in README.md and test_mower_bag.py). Rerun after any task edit.
set -eu
cd "$(dirname "$0")"
rm -rf arm && mkdir arm && cp ../mower_bag.py ../mechanism.py ../task.json arm/
grep -q '^VARIANT = "demo"' arm/mower_bag.py || { echo "line 'VARIANT = \"demo\"' changed; fix mk.sh"; exit 3; }
mkcopy(){  # mkcopy <stem> <variant>
  sed -e "s/^VARIANT = \"demo\"/VARIANT = \"$2\"/" arm/mower_bag.py > arm/$1.py
  cat log_snippet.py >> arm/$1.py
}
mkcopy t00 demo
for v in nothing one_arm one_hook tips ground hold prop pinch slam nudge shove; do mkcopy s_$v $v; done
echo MK OK
