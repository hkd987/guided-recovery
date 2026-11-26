#!/usr/bin/env bash
# Simulates the motor-driver watchdog in ngspice and checks every requirement
# (see docs/WATCHDOG.md). Exits non-zero if any run fails.
#
#   hardware/watchdog/run_checks.sh             nominal + corners + 40 Monte Carlo draws (~3 min)
#   hardware/watchdog/run_checks.sh --quick     nominal design only (seconds)
#   hardware/watchdog/run_checks.sh --mc 200    more Monte Carlo draws
#
# Needs ngspice (Debian/Ubuntu: apt-get install ngspice) and Python 3.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
if ! command -v ngspice >/dev/null 2>&1; then
  echo "ngspice not found: apt-get install ngspice" >&2
  exit 2
fi
exec python3 "$HERE/run_checks.py" "$@"
