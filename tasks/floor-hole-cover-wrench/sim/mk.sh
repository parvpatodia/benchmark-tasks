#!/bin/bash
# mk.sh: sim copies of the task in sim/arm/: floor_hole_cover.py as is, plus traced copies (log_snippet.py appended), one
# per stem: t00 (the demo; seeds via the CLI) and s_nothing, s_held, s_drop, s_slid_off, s_aside, s_on_deck, s_touch,
# s_back (expected scores in README.md and test_floor_hole_cover.py). Rerun after any task edit.
set -eu
cd "$(dirname "$0")"
rm -rf arm && mkdir arm && cp ../floor_hole_cover.py ../mechanism.py ../task.json arm/
grep -q '^VARIANT = "demo"' arm/floor_hole_cover.py || { echo "line 'VARIANT = \"demo\"' changed; fix mk.sh"; exit 3; }
mkcopy(){  # mkcopy <stem> <variant>
  sed -e "s/^VARIANT = \"demo\"/VARIANT = \"$2\"/" arm/floor_hole_cover.py > arm/$1.py
  grep -q "^VARIANT = \"$2\"" arm/$1.py || { echo "variant $2 not set in $1.py"; exit 3; }
  cat log_snippet.py >> arm/$1.py
}
mkcopy t00 demo
for v in nothing held drop slid_off aside on_deck touch back; do mkcopy s_$v $v; done
echo MK OK
