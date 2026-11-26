#!/usr/bin/env bash
# Fit-check the pod and export the printable STLs (and, with --images, the
# pictures in img/). Needs openscad, python3 with trimesh and manifold3d;
# --images also needs xvfb-run.
#   ./build.sh            check + STLs in build/stl
#   ./build.sh --images   also re-render img/*.png
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build/stl
python3 check.py --out build
for p in base sled winch roof sleeve spool rbf_tag; do
  cp "build/$p.stl" "build/stl/pod-$p.stl"
done
echo "STLs: $(pwd)/build/stl (print two spools)"
if [[ "${1:-}" == "--images" ]]; then
  r() { xvfb-run -a openscad -q -o "img/$1.png" --imgsize=1400,1400 --colorscheme=Tomorrow --camera="$2" --projection=p -D "part=\"$1\"" pod.scad; }
  r assembly 0,0,62,62,0,30,380
  r cutaway 0,0,62,75,0,0,330
  r exploded 40,0,118,72,0,20,700
  echo "images: $(pwd)/img"
fi
