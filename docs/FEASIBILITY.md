# Feasibility: what the software gates show

**Bottom line.** In simulation, the pod design works. The real flight code,
fed by realistic sensor and motor models, lands the S pod within 30 m of the
pad in **94.5%** of 1,000 randomized flights in winds up to 10 mph. The spec's
gate 2 pass mark is 85%. Every injected fault ends the way the spec requires.

The parts are buyable today for about **$191**, and the firmware compiles for
the Heltec V3. What simulation cannot settle is the physics. That's the job
of gates 4–6:
- Does a parafoil open cleanly after ejection?
- What are the real turn and glide numbers?
- How does the pod swing on the harness?

## Gate results

### Batch 1: seed 1, the CI baseline

1,000 nominal flights plus 100 per fault type.

| | Gate 1 (guidance, truth) | Gate 2 (flight core in the loop) |
|---|---|---|
| Within 30 m | **97.8%** | **94.5%** (drop 3.3 points; allowed 5) |
| Median miss | 1.9 m | 6.1 m |
| 90th percentile | 11.8 m | 23.1 m |
| Steering before canopy confirmed | 0 | **0** |
| Crashes / non-finite outputs | 0 | **0** |
| Canopy confirmed after opening | — | mean 3.8 s, max 6.2 s |

| Injected fault (gate 2) | Expected | Result |
|---|---|---|
| GPS loss | spiral within 5 s | 99/99 |
| Stalled winch | spiral within 15 s | 90/90 |
| Lopsided canopy (needs 50–80% brake to fly straight) | spiral within 25 s | 71/71 |
| Gust (+3–6 m/s for 6 s) | still lands | 97/97 |
| Low-level wind shear (direction swings 90–180°) | still lands | 100/100 |

"Exercised" counts only runs where the fault happened early enough before
landing to be judged. No nominal flight ended in a false fallback.

### Batch 2: independent seeds (50000+)

Checks for overfitting to batch 1.

| Measure | Result |
|---|---|
| Gate 1 | 96.4% within 30 m |
| Gate 2 | **93.3%** within 30 m |
| Faults | all exercised faults met their expected outcome: GPS loss 98/98, stalled winch 89/89, lopsided canopy 75/75, gust 100/100, shear 100/100 |
| False fallbacks | 0 |

### Gate 3: log replay

The replay harness passes on 200 synthetic host logs: single and dual deploy,
round mains and parafoils at 3–8 m/s descent, and 9 main failures that must
*not* trigger.

| Measure | Result |
|---|---|
| Main deployments detected | **191/191** |
| False triggers | **0** |
| Detection delay after the main fires | mean 5.5 s, max 8.1 s |

**This is weak evidence.** The synthetic logs come from the same simulator.
The spec asks for 100+ real flights, and none could be fetched here:
- Altus Metrum has no central public archive.
- Its one log repository (git.gag.com/fw/tmflights) was blocked from this environment.
- AltOS's own test suite lists about 80 flights, but their data files aren't distributed.

CI replays any CSV dropped into `logs/real/` (format in its README).
Collecting real logs is the most valuable next software task.

## What size parafoil? (canopy sweep)

Percentage of SITL flights landing within 30 m, 300 flights per cell. Sink
rate is fixed at 3.5 m/s; wind speeds are measured at 10 m, and wind aloft is
stronger.

| Canopy trim airspeed | 5 mph | 10 mph | 15 mph | 20 mph |
|---|---|---|---|---|
| 6 m/s | 97.0% | 84.7% | 56.7% | 46.0% |
| 7 m/s | 98.7% | 92.7% | 65.0% | 49.0% |
| **8 m/s** | 98.3% | **95.7%** | 71.7% | 54.0% |
| 9 m/s | 96.3% | 93.3% | 75.3% | 55.3% |
| 10 m/s | 91.0% | 88.7% | 75.7% | 60.3% |

What the sweep says:
- **Spec for drop testing:** pick test parafoils that trim around **8 m/s
  airspeed** with a glide of about 2.3 at the pod-plus-rocket weight.
- **Slower canopies** get blown back.
- **Faster canopies** lose some accuracy in light wind. Guidance gains are
  tuned for 8 m/s, so they could be retuned.
- **15 mph and above** isn't a realistic accuracy target for the S pod.

## Physical budgets (S pod, estimates)

| Budget | Numbers | Verdict |
|---|---|---|
| Winch force | Pololu 210:1 HPCB: 0.275 N·m stall at 6 V → 34 N on a 16 mm spool. Load: 40% of a 1.5 kg rocket's weight ≈ 5.9 N (single forum data point) → 17% of stall | OK. Pololu advises ≤ 25% |
| Winch speed | 160 rpm no-load → 134 mm/s; ≈ 111 mm/s at 5.9 N → 120 mm full brake in ≈ 1.1 s | OK. The simulated canopy needs about 1 s |
| Battery | Logic ≈ 0.55 W (ESP32-S3, GPS, sensors, LoRa at 2 Hz) → ≈ 90 mA from 2S. Steering ≈ 0.4 A while active. 30 min pad + 5 min flight ≈ 80 mAh | 450 mAh ≈ 5 flights or about 4 h standby |
| Motor driver | 6 V motors on 2S stall at about 2 A instantaneous, the DRV8833 peak. The firmware caps average voltage at 6 V and cuts stalls in about 0.4 s | Acceptable. Watch on the bench |
| Mass | Board 14 g, sensors 8 g, motors 22 g, battery 27 g, shell 40 g, Kevlar/swivels/links 30 g, misc 20 g ≈ **160 g** | About 11% of a 1.5 kg rocket. It counts toward the Class 1 1,500 g limit |
| Log space | 64-byte records at 50 Hz in flight (5 Hz on the pad) in 5.4 MB | About 28 min of flight-rate logging |
| Control loop | 200 Hz on core 1; flight core math is cheap. Flash writes are page programs only (< 1 ms) | To be measured on hardware (`status` shows loop max and overruns) |

## Bugs the gates caught before any hardware existed

Each item was found by a failing Monte Carlo run, traced in a single-flight
replay, and fixed. This is the point of software-first testing.

1. **Steering on GPS course fails when wind aloft beats the canopy.** At 700 m
   with 9–10 m/s of wind, the pod circled. Fix: heading is now derived with no
   compass, as ground velocity minus the estimated wind. The pod crabs, or
   holds into the wind when it can't make headway.
2. **The attitude filter trusted the accelerometer during coast.** There it
   reads drag, not gravity, so turn-rate readings were wrong when the canopy
   opened. Fix: the accelerometer is gated by flight phase, and attitude is
   realigned at canopy confirmation.
3. **The lopsided-canopy check went wrong both ways:**
   - First version: missed 90% of faults.
   - Second version: flagged weak-turning canopies as lopsided.
   - Final version: a small Kalman filter learns each canopy's turn gain and
     pull, and only judges once it is confident. Result: 0 false alarms in
     2,000 nominal flights, 146/146 real faults caught.
4. **Jammed winches went unnoticed.** A winch that was never asked to move
   was never checked. Fix: an idle winch is exercised by 4.5 mm every few
   seconds, and the stall timer is leaky.
5. **That exercise falsely jammed both winches during ascent**, while motor
   power was off. Fix: stalls are only judged while powered.
6. **Chattering at 180°.** A heading error near 180° made the canopy alternate
   full left and full right and circle. Fix: commit to one turn direction.
7. **A miswired encoder would run the spool away.** Fix: a wrong-way detector
   stops the motor in 0.2 s, and the self-test names the setting to change.
8. **Height overestimated at high-elevation fields.** Measuring height from
   ground pressure alone overestimated it by 3.6% at 800 m. Fix: difference of
   standard-atmosphere altitudes.

## Not proven yet

| Gate / question | Status |
|---|---|
| Gate 4 (HIL) | Needs the bench rig. The flight core already runs unchanged on desktop and ESP32, so the same scenarios can drive the real board over USB |
| Gate 5 (ground deploy) and the spec's first open question: does a parafoil open cleanly after a hard ejection? | Simulation assumes it opens in 0.8–2.5 s |
| Gate 6 (drop tests) | The canopy model is the spec's "simple model" and needs fitting to real turn, glide and sink data. Every result above assumes canopies within about ±6% airspeed/sink and ±15% turn gain of the tested design |
| Pod-mounted IMU vs. airframe logs (spec open question) | Replay proves the logic, not the thresholds |
| Firmware on hardware | Drivers are written from datasheets: BMI088, LSM6DSO32, BMP390, ADXL375, u-blox M10 UBX, PCNT encoders, DRV8833, SX1262. They compile but have not run. Bench-check each with `selftest` |
| Heltec V4 | Not supported (different pin map) |

## Known limits the simulation exposed

- **High deployments in shear lose accuracy.** Winds aloft can exceed the
  canopy's speed above about 600 m in a 10 mph surface wind. The pod then
  holds into the wind and drifts, and that's where the 300 m worst cases come
  from. Keep apogee modest, or dual-deploy with the main at 150–250 m.
- **Low deployments far downwind can't make it back.** The pod lands short,
  but on the right line.
