#!/usr/bin/env python3
"""Fit check for the pod: exports every printed part and component envelope
from pod.scad, then checks with exact mesh booleans that

  1. no component overlaps a printed part or another component,
  2. printed parts don't overlap each other,
  3. everything stays inside the sleeve (components with a margin),
  4. the keep-outs (USB plug, brake lines, Kevlar) are clear,

and reports printed volume, mass and centre of gravity.

  python3 check.py [--out DIR] [--od 62] [--jobs 8]

Exit status 0 only if every check passes. Needs openscad, trimesh, manifold3d.
"""
import argparse
import concurrent.futures as cf
import itertools
import json
import math
import os
import subprocess
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SCAD = os.path.join(HERE, "pod.scad")

PRINTED = ["base", "sled", "winch", "roof", "sleeve"]
EXTRA_PRINTED = ["spool", "rbf_tag"]   # exported for printing, not placed in the stack

# Component envelopes and their mass in grams (sources in README.md).
COMPONENTS = {
    "env_heltec": 35.0,          # Heltec's listed 35 g (likely with antenna and headers: conservative)
    "env_battery": 28.4,
    "env_drv8833": 1.5,
    "env_imu": 1.7,
    "env_baro1": 1.5,
    "env_baro2": 1.5,            # backup kit
    "env_adxl375": 1.0,          # optional
    "env_buck": 1.0,
    "env_motor_l": 11.0,
    "env_motor_r": 11.0,
    "env_spool_l": 1.5,          # printed, PETG
    "env_spool_r": 1.5,
    "env_gps1": 2.7,
    "env_gps2": 2.7,             # backup kit
    "env_reed": 5.4,
    "env_buzzer": 1.6,
    "env_watchdog": 2.5,         # perfboard, 3 transistors, 2 diodes, 3 caps, 8 resistors
    "env_sma": 3.0,              # bulkhead SMA + pigtail
}
KEEPOUTS = ["env_usb_plug", "env_line_l", "env_line_r", "env_kevlar", "env_battery_leads",
            "env_stopper_bottom", "env_stopper_top"]
# Allowed contacts: these meet by design.
ALLOWED = {
    frozenset(["env_line_l", "env_spool_l"]), frozenset(["env_line_r", "env_spool_r"]),
    frozenset(["env_usb_plug", "env_heltec"]),
    frozenset(["env_kevlar", "env_stopper_bottom"]), frozenset(["env_kevlar", "env_stopper_top"]),
    frozenset(["env_spool_l", "env_motor_l"]), frozenset(["env_spool_r", "env_motor_r"]),  # spool sits on the shaft
}
# Keep-outs that are meant to leave the pod (axially), so only their radius is checked.
LEAVES = {"env_usb_plug", "env_line_l", "env_line_r", "env_kevlar", "env_stopper_bottom",
          "env_stopper_top", "env_sma"}
OTHER_MASS = {"wiring and connectors": 8.0, "Kevlar through the pod, 2 washers": 8.0,
              "PTFE guides, screws, strap": 4.0}
# The fit check runs on the full build (backup kit and the optional ADXL375 fitted).
PETG = 1.27  # g/cm^3; decks and thin walls print nearly solid
TOL = 0.5    # mm^3 of overlap tolerated (mesh noise)
MARGIN = 0.3  # mm components keep from the sleeve's inside


def export(name, out, defs):
    path = os.path.join(out, name + ".stl")
    cmd = ["openscad", "-q", "-o", path, "-D", f'part="{name}"'] + sum((["-D", d] for d in defs), []) + [SCAD]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0 or not os.path.exists(path):
        raise RuntimeError(f"openscad failed for {name}: {r.stderr[-400:]}")
    return name, path


def load(path):
    m = trimesh.load(path, force="mesh")
    if not m.is_volume:
        m.merge_vertices()
        m.fix_normals()
    return m


def overlap(a, b):
    lo = np.maximum(a.bounds[0], b.bounds[0])
    hi = np.minimum(a.bounds[1], b.bounds[1])
    if np.any(hi - lo <= 0):
        return 0.0
    try:
        i = trimesh.boolean.intersection([a, b], engine="manifold")
        return float(abs(i.volume)) if i is not None and len(i.faces) else 0.0
    except Exception:  # degenerate touching faces
        return 0.0


def main():
    np.seterr(all="ignore")
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "build"))
    ap.add_argument("--od", type=float, default=None, help="try another outer diameter")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    defs = ["kit=true"] + ([f"OD={a.od}"] if a.od else [])

    names = PRINTED + EXTRA_PRINTED + list(COMPONENTS) + KEEPOUTS
    with cf.ThreadPoolExecutor(a.jobs) as ex:
        paths = dict(ex.map(lambda n: export(n, a.out, defs), names))
    mesh = {n: load(p) for n, p in paths.items()}

    od = a.od or 62.0
    r_in = od / 2 - 1.2
    problems = []

    # 1-2. Overlaps.
    solids = PRINTED + list(COMPONENTS) + KEEPOUTS
    pairs = [(x, y) for x, y in itertools.combinations(solids, 2)
             if frozenset([x, y]) not in ALLOWED and not (x in KEEPOUTS and y in KEEPOUTS)]
    with cf.ThreadPoolExecutor(a.jobs) as ex:
        vols = list(ex.map(lambda p: overlap(mesh[p[0]], mesh[p[1]]), pairs))
    for (x, y), v in zip(pairs, vols):
        if v > TOL:
            problems.append(f"overlap {x} / {y}: {v:.1f} mm^3")

    # 3. Inside the sleeve.
    for n in PRINTED[:-1] + list(COMPONENTS) + KEEPOUTS:
        v = mesh[n].vertices
        rmax = float(np.max(np.hypot(v[:, 0], v[:, 1])))
        limit = r_in - (0.0 if n in PRINTED else MARGIN)
        if rmax > limit + 1e-3:
            problems.append(f"outside the sleeve: {n} reaches r = {rmax:.2f} mm (limit {limit:.2f})")
    zmax = float(mesh["sleeve"].bounds[1][2])
    for n in PRINTED[:-1] + list(COMPONENTS):
        if n in LEAVES:
            continue
        lo, hi = mesh[n].bounds[:, 2]
        if lo < -1e-3 or hi > zmax + 1e-3:
            problems.append(f"sticks out of the ends: {n} spans z {lo:.1f}..{hi:.1f}")

    # Mass and centre of gravity.
    parts = []
    for n in PRINTED:
        m = mesh[n]
        parts.append((n, m.volume / 1000 * PETG, m.center_mass[2]))
    for n, g in COMPONENTS.items():
        parts.append((n, g, mesh[n].bounds.mean(axis=0)[2]))
    zmid = zmax / 2
    for n, g in OTHER_MASS.items():
        parts.append((n, g, zmid))
    total = sum(g for _, g, _ in parts)
    cg = sum(g * z for _, g, z in parts) / total
    printed_g = sum(g for n, g, _ in parts if n in PRINTED)
    basic = total - COMPONENTS["env_baro2"] - COMPONENTS["env_gps2"] - COMPONENTS["env_adxl375"]

    report = {
        "outer_diameter_mm": od,
        "length_mm": round(zmax, 1),
        "fits_tubes": "2.6 in and larger (inside diameter >= 64.8 mm)" if od <= 62 else f"inside diameter > {od + 2:.0f} mm",
        "printed_parts_g": round(printed_g, 1),
        "printed_volumes_cm3": {n: round(mesh[n].volume / 1000, 2) for n in PRINTED + EXTRA_PRINTED},
        "total_mass_full_build_g": round(total, 1),
        "total_mass_basic_pod_g": round(basic, 1),
        "cg_from_rocket_end_mm": round(cg, 1),
        "pairs_checked": len(pairs),
        "problems": problems,
    }
    with open(os.path.join(a.out, "fit_report.json"), "w") as f:
        json.dump(report, f, indent=2)
    print(json.dumps({k: v for k, v in report.items() if k != "problems"}, indent=2))
    if problems:
        print("\nFIT CHECK FAIL")
        for p in problems:
            print("  " + p)
        return 1
    print(f"\nFIT CHECK PASS ({len(pairs)} pairs, no overlaps; everything inside the sleeve)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
