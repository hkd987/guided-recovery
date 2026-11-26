#!/usr/bin/env bash
# Formal verification of the pod's decision logic with TLA+ / TLC.
#
#   verification/run_tla.sh            model check + witnesses + mutations + trace validation
#   TLA2TOOLS=/path/tla2tools.jar ...  use an existing jar
#
# Steps:
#  1. Model check PodLogic: every invariant and step property must hold.
#  2. Witnesses: each must be VIOLATED (the situation is reachable, so the
#     properties are not vacuously true).
#  3. Mutations: each re-introduces one known bug; TLC must catch every one.
#  4. Trace validation: every state change the real C++ flight core made in
#     simulated flights and bench sessions must be a step the model allows.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TLA_DIR="$ROOT/verification/tla"
WORK="${WORK:-$ROOT/build/tla}"
JAR="${TLA2TOOLS:-$ROOT/build/tla2tools.jar}"
TLA_VERSION="v1.7.4"
TLA_SHA256="936a262061c914694dfd669a543be24573c45d5aa0ff20a8b96b23d01e050e88"
mkdir -p "$WORK"

if [ ! -f "$JAR" ]; then
  echo "Downloading tla2tools.jar ($TLA_VERSION)..."
  curl -sSfL -o "$JAR" "https://github.com/tlaplus/tlaplus/releases/download/$TLA_VERSION/tla2tools.jar"
  echo "$TLA_SHA256  $JAR" | sha256sum -c - >/dev/null || { echo "tla2tools.jar checksum mismatch"; rm -f "$JAR"; exit 1; }
fi

tlc() {  # tlc <cfg> <module> <log>
  java -XX:+UseParallelGC -cp "$JAR" tlc2.TLC -workers auto -cleanup \
    -metadir "$WORK/states" -config "$1" "$2" > "$3" 2>&1 || true
}

cd "$TLA_DIR"
fail=0

echo "== 1. Model check (all invariants and properties) =="
tlc PodLogic.cfg PodLogic.tla "$WORK/model.log"
if grep -q "Model checking completed. No error has been found." "$WORK/model.log"; then
  grep -E "distinct states found|depth of the complete" "$WORK/model.log" | sed 's/^/   /'
  echo "   PASS"
else
  grep -E "Error|violated|is violated" "$WORK/model.log" | head -5 | sed 's/^/   /'
  echo "   FAIL (see $WORK/model.log)"; fail=1
fi

echo "== 2. Witnesses (each must be reachable) =="
for w in WitnessGuided WitnessGpsSpiral WitnessCollapse WitnessBenchJamFlies \
         WitnessNoHomeSpiral WitnessLandedNoCanopy WitnessArmedWithJam \
         WitnessRestartRelease WitnessRestartGuides WitnessBaroSpiral; do
  printf 'CONSTANT Mutation = "none"\nINIT Init\nNEXT Next\nINVARIANT %s\nCHECK_DEADLOCK FALSE\n' "$w" > "$WORK/$w.cfg"
  tlc "$WORK/$w.cfg" PodLogic.tla "$WORK/$w.log"
  if grep -q "Invariant $w is violated" "$WORK/$w.log"; then
    depth=$(grep -c "^State [0-9]*:" "$WORK/$w.log" || true)
    echo "   reachable  $w  (in $depth steps)"
  else
    echo "   NOT REACHABLE  $w"; fail=1
  fi
done

echo "== 3. Mutations (each known bug must be caught) =="
for m in stale_candidate motors_in_descent home_unfrozen no_gps_failsafe disarm_in_flight resume_steering \
         no_baro_failsafe; do
  sed "s/Mutation = \"none\"/Mutation = \"$m\"/" PodLogic.cfg > "$WORK/mut_$m.cfg"
  tlc "$WORK/mut_$m.cfg" PodLogic.tla "$WORK/mut_$m.log"
  caught=$(grep -oE "(Invariant|Action property|Temporal properties) [A-Za-z]* ?(is )?violated|Action property [A-Za-z]+ of line [0-9]+ of module PodLogic is violated" "$WORK/mut_$m.log" | head -1 || true)
  if [ -n "$caught" ]; then
    steps=$(grep -c "^State [0-9]*:" "$WORK/mut_$m.log" || true)
    echo "   caught  $m: $caught (counterexample $steps steps)"
  else
    echo "   MISSED  $m"; fail=1
  fi
done

if [ -x "$ROOT/build/grtrace" ]; then
  echo "== 4. Trace validation (C++ flight core vs. the model) =="
  "$ROOT/build/grtrace" --out "$WORK/Traces.tla" ${GRTRACE_ARGS:-}
  cp PodTrace.tla PodTrace.cfg "$WORK/"
  cp PodLogic.tla "$WORK/"
  (cd "$WORK" && tlc PodTrace.cfg PodTrace.tla "$WORK/trace.log")
  if grep -q "Model checking completed. No error has been found." "$WORK/trace.log"; then
    grep -E "steps checked|disallowed" "$WORK/trace.log" | sed 's/^/   /'
    echo "   PASS"
  else
    grep -E "disallowed|Assumption|Error" "$WORK/trace.log" | head -8 | sed 's/^/   /'
    echo "   FAIL (see $WORK/trace.log)"; fail=1
  fi
fi

exit $fail
