#!/usr/bin/env python3
"""Simulates the motor-driver watchdog in ngspice and checks its requirements.

Every run powers the circuit up with the pet pin floating up (as GPIO 20 does
during boot), lets the firmware drive it low, pets it at a given rate, then
stops petting in one of four ways. Requirements (DRV8833 nSLEEP: low <= 0.5 V,
high >= 2.5 V):

  R1  disabled at power-up: SLP stays <= 0.5 V until petting starts
  R2  enabled within 50 ms of the first pet edge, 50-200 Hz
  R3  stays enabled while petted: SLP >= 2.5 V
  R4  disabled within 250 ms of the last pet edge, whether the pin sticks
      high, sticks low, floats up (1.5 kohm pull-up) or floats (high-Z),
      and stays disabled

Runs: the nominal design at every rate and stop mode, worst-case corners, and
a Monte Carlo over part tolerances (C +/-20 %, R +/-5 %, hFE 0.3-1.5x,
Schottky Vf +/-40 mV, 3.0-3.6 V rail, 0-50 C).

  run_checks.py [--mc N] [--seed S] [--jobs J] [--quick]

Exits non-zero if any run fails a requirement.
"""

import argparse
import itertools
import multiprocessing
import os
import random
import string
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
TEMPLATE = open(os.path.join(HERE, "watchdog.cir.tmpl")).read()
MODELS = os.path.join(HERE, "models.lib")

# Nominal design (see docs/WATCHDOG.md).
NOMINAL = dict(
    c1=1.0e-6, c2=1.5e-6, c3=1.0e-6,
    r1=47e3, r2=47e3, r3=10e3, r4=47e3, r5=4.7e3, r6=220e3, r7=22e3, ra=33e3,
    vcc=3.3, temp=25.0, bf=1.0, is_=1.0,
)
TOL = dict(c1=0.20, c2=0.20, c3=0.20, r1=0.05, r2=0.05, r3=0.05, r4=0.05,
           r5=0.05, r6=0.05, r7=0.05, ra=0.05)

VIL, VIH = 0.5, 2.5
T_DRIVE, T_PET, T_STOP, T_END = 0.50, 0.60, 1.00, 1.45
STOP_MODES = ("stuck_high", "stuck_low", "float_up", "float_hiz")
RATES = (50.0, 100.0, 200.0)
EN_LIMIT, DIS_LIMIT = 0.050, 0.250


def pwl(points):
    return " ".join(f"{t:.9g} {v:.6g}" for t, v in points)


def waveforms(p, rate, stop, single_edge=False):
    """Source and switch-control PWLs for one scenario."""
    v = p["vcc"]
    e = 1e-6  # edge time
    src = [(0, 0), (T_DRIVE, 0)]
    ctl = [(0, 0), (T_DRIVE - e, 0), (T_DRIVE, 1)]
    if single_edge:
        # Idle low, then a reset: the pin goes high-Z and floats up once.
        ctl += [(T_STOP - e, 1), (T_STOP, 0), (T_END, 0)]
        src += [(T_END, 0)]
        return src, ctl
    half = 0.5 / rate
    t, level = T_PET, 0
    src.append((T_PET - e, 0))
    while t < T_STOP - 1e-9:
        level = 1 - level
        src += [(t, (1 - level) * v), (t + e, level * v)]
        t += half
    # Stop: hold, drive or release.
    if stop == "stuck_high":
        src += [(T_STOP, v), (T_STOP + e, v), (T_END, v)]
        ctl += [(T_END, 1)]
    elif stop == "stuck_low":
        src += [(T_STOP, level * v), (T_STOP + e, 0), (T_END, 0)]
        ctl += [(T_END, 1)]
    else:  # floating: the driver lets go
        src += [(T_END, level * v)]
        ctl += [(T_STOP - e, 1), (T_STOP, 0), (T_END, 0)]
    # PWL times must increase strictly.
    clean = []
    for tt, vv in src:
        if clean and tt <= clean[-1][0]:
            tt = clean[-1][0] + 1e-9
        clean.append((tt, vv))
    return clean, ctl


def simulate(job):
    p, rate, stop, single_edge, label = job
    pull_up = stop != "float_hiz"
    src, ctl = waveforms(p, rate, stop, single_edge)
    with tempfile.TemporaryDirectory(prefix="wd_") as td:
        out = os.path.join(td, "out.txt")
        net = string.Template(TEMPLATE).substitute(
            label=label, models=MODELS, bf=p["bf"], isscale=p["is_"],
            tramp="1m", vcc=p["vcc"], srcpwl=pwl(src), ctlpwl=pwl(ctl),
            pullnode="vcc" if pull_up else "0", rpull="1.5k" if pull_up else "1Meg",
            c1=p["c1"], c2=p["c2"], c3=p["c3"], r1=p["r1"], r2=p["r2"], r3=p["r3"],
            r4=p["r4"], r5=p["r5"], r6=p["r6"], r7=p["r7"], ra=p["ra"], temp=p["temp"],
            tstep="20u", tstop=T_END, tmax="20u", out=out)
        cir = os.path.join(td, "wd.cir")
        with open(cir, "w") as f:
            f.write(net)
        r = subprocess.run(["ngspice", "-b", cir], capture_output=True, text=True, timeout=600)
        if not os.path.exists(out):
            return dict(label=label, error=(r.stdout + r.stderr)[-800:])
        t, slp = [], []
        with open(out) as f:
            next(f)
            for line in f:
                c = line.split()
                t.append(float(c[0]))
                slp.append(float(c[1]))
    return dict(label=label, rate=rate, stop=stop, single_edge=single_edge, p=p, **measure(t, slp, single_edge))


def measure(t, slp, single_edge):
    m = {}
    m["pu_max"] = max(v for tt, v in zip(t, slp) if tt < T_PET)
    if single_edge:
        hi = [tt for tt, v in zip(t, slp) if tt >= T_STOP and v >= VIH]
        lo_after = [v for tt, v in zip(t, slp) if tt >= T_STOP]
        m["edge_awake"] = (max(hi) - min(hi)) if hi else 0.0
        m["edge_max"] = max(lo_after)
        return m
    # Enabled: SLP stays >= VIH from then until the stop (no start-up chatter
    # counted as "on"). t_en is measured from the first pet edge.
    petted = [(tt, v) for tt, v in zip(t, slp) if T_PET <= tt < T_STOP]
    below = [tt for tt, v in petted if v < VIH]
    t_en = max(below) if below else T_PET
    m["t_en"] = (t_en - T_PET) if t_en < T_STOP - 0.01 else float("inf")
    healthy = [v for tt, v in petted if tt >= T_PET + EN_LIMIT]
    m["healthy_min"] = min(healthy) if healthy else 0.0
    # Disabled: the last time SLP was above VIL after the stop, plus one step.
    after = [(tt, v) for tt, v in zip(t, slp) if tt >= T_STOP]
    above = [tt for tt, v in after if v > VIL]
    m["t_dis"] = (max(above) - T_STOP) if above else 0.0
    m["tripped_v"] = after[-1][1]
    m["end_ok"] = all(v <= VIL for tt, v in after if tt >= T_STOP + DIS_LIMIT)
    return m


def check(r):
    if "error" in r:
        return [f"ngspice failed: {r['error']}"]
    f = []
    if r["pu_max"] > VIL:
        f.append(f"R1 woke at power-up (SLP {r['pu_max']:.2f} V)")
    if r.get("single_edge"):
        return f
    if r["t_en"] > EN_LIMIT:
        f.append(f"R2 enable took {r['t_en']*1e3:.1f} ms")
    if r["healthy_min"] < VIH:
        f.append(f"R3 SLP dipped to {r['healthy_min']:.2f} V while petted")
    if r["t_dis"] > DIS_LIMIT or not r["end_ok"]:
        f.append(f"R4 disable took {r['t_dis']*1e3:.0f} ms")
    return f


def draw(rng):
    p = dict(NOMINAL)
    for k, tol in TOL.items():
        p[k] = NOMINAL[k] * (1.0 + rng.uniform(-tol, tol))
    p["vcc"] = rng.uniform(3.0, 3.6)
    p["temp"] = rng.uniform(0.0, 50.0)
    p["bf"] = rng.uniform(0.3, 1.5)
    p["is_"] = 10 ** rng.uniform(-0.5, 0.5)  # ~+/-30 mV of Schottky Vf
    return p


def corners():
    """Hand-picked worst cases for each requirement."""
    out = []
    for vcc, temp, bf, is_ in itertools.product((3.0, 3.6), (0.0, 50.0), (0.3, 1.5), (0.316, 3.16)):
        # Slow to enable: small pump cap, big hold cap, strong drain.
        slow_en = dict(NOMINAL, vcc=vcc, temp=temp, bf=bf, is_=is_,
                       c1=NOMINAL["c1"] * 0.8, c2=NOMINAL["c2"] * 1.2,
                       r1=NOMINAL["r1"] * 0.95, r2=NOMINAL["r2"] * 0.95)
        # Slow to disable: big caps, weak drain.
        slow_dis = dict(NOMINAL, vcc=vcc, temp=temp, bf=bf, is_=is_,
                        c1=NOMINAL["c1"] * 1.2, c2=NOMINAL["c2"] * 1.2,
                        r1=NOMINAL["r1"] * 1.05, r2=NOMINAL["r2"] * 1.05)
        out += [("corner-slow-enable", slow_en), ("corner-slow-disable", slow_dis)]
    # Power-on clamp at its shortest.
    for vcc in (3.0, 3.6):
        out.append(("corner-short-clamp", dict(NOMINAL, vcc=vcc, c3=NOMINAL["c3"] * 0.8,
                                               r6=NOMINAL["r6"] * 0.95, bf=0.3, temp=50.0)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mc", type=int, default=40, help="Monte Carlo draws")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 2)
    ap.add_argument("--quick", action="store_true", help="nominal design only")
    ap.add_argument("--set", action="append", default=[], metavar="PART=VALUE",
                    help="override a nominal value, e.g. --set c2=1.5e-6 (design exploration)")
    a = ap.parse_args()
    for kv in a.set:
        k, v = kv.split("=", 1)
        if k not in NOMINAL:
            ap.error(f"unknown part {k}")
        NOMINAL[k] = float(v)

    jobs = []
    for rate, stop in itertools.product(RATES, STOP_MODES):
        jobs.append((NOMINAL, rate, stop, False, "nominal"))
    jobs.append((NOMINAL, 100.0, "float_up", True, "nominal"))
    if not a.quick:
        for label, p in corners():
            for rate, stop in itertools.product((50.0, 200.0), STOP_MODES):
                jobs.append((p, rate, stop, False, label))
        rng = random.Random(a.seed)
        for _ in range(a.mc):
            p = draw(rng)
            for rate, stop in itertools.product((50.0, 200.0), STOP_MODES):
                jobs.append((p, rate, stop, False, "monte-carlo"))
            jobs.append((p, 100.0, "float_up", True, "monte-carlo"))

    with multiprocessing.Pool(a.jobs) as pool:
        results = pool.map(simulate, jobs)

    failures = []
    for r in results:
        for f in check(r):
            failures.append((r, f))

    def worst(rows, key, fn):
        vals = [r[key] for r in rows if key in r]
        return fn(vals) if vals else float("nan")

    runs = [r for r in results if "error" not in r and not r.get("single_edge")]
    edges = [r for r in results if "error" not in r and r.get("single_edge")]
    print(f"Watchdog checks: {len(results)} ngspice runs "
          f"(nominal, {0 if a.quick else len(corners())} corners, {0 if a.quick else a.mc} Monte Carlo draws)")
    print()
    print(f"{'group':<22}{'stop':<12}{'runs':>5}{'enable ms':>10}{'SLP petted V':>14}{'disable ms':>12}"
          f"{'SLP tripped V':>15}{'power-up max V':>16}{'fails':>6}")
    groups = {}
    for r in runs:
        groups.setdefault((r["label"], r["stop"]), []).append(r)
    for (label, stop), rows in sorted(groups.items()):
        nf = sum(1 for r in rows if check(r))
        print(f"{label:<22}{stop:<12}{len(rows):>5}{worst(rows,'t_en',max)*1e3:>10.1f}{worst(rows,'healthy_min',min):>14.2f}"
              f"{worst(rows,'t_dis',max)*1e3:>12.0f}{worst(rows,'tripped_v',max):>15.3f}{worst(rows,'pu_max',max):>16.3f}"
              f"{nf:>6}")
    print()
    print("Limits: enable <= 50 ms; petted SLP >= 2.5 V; disable <= 250 ms; tripped and power-up SLP <= 0.5 V.")
    if edges:
        aw = max(r["edge_awake"] for r in edges)
        print(f"Single edge (pin floats up once during a reset, nothing else): driver awake at most "
              f"{aw*1e3:.0f} ms (motor inputs are low then, so nothing moves).")
    if failures:
        print()
        print(f"FAIL: {len(failures)} requirement violations")
        for r, f in failures[:25]:
            pp = r.get("p", {})
            print(f"  {r['label']} rate {r.get('rate')} stop {r.get('stop')} vcc {pp.get('vcc', 0):.2f} "
                  f"temp {pp.get('temp', 0):.0f} bf {pp.get('bf', 0):.2f}: {f}")
        return 1
    print("WATCHDOG PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
