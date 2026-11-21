#!/bin/sh
# Export every printed part (STL, assembly position) and the key's cutting profile (DXF), then redraw drawing.svg/png.
# Needs OpenSCAD (2021.01 or later) on PATH and Python 3 with matplotlib. MIT licence.
set -e
cd "$(dirname "$0")"
mkdir -p stl
for p in floor_plate clamp_lower key_plate cover_plate shell cap plunger slide pin key; do
  openscad -q -o "stl/$p.stl" -D "part=\"$p\"" coinlock.scad
done
openscad -q -o key.dxf -D 'part="key_dxf"' coinlock.scad
python3 drawing.py
