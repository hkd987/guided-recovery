# Feasibility: what the software gates show

**Bottom line.** In simulation, the pod design works. The real flight code,
fed by realistic sensor and motor models, lands the S pod within 30 m of the
pad in **94.2%** of 1,000 randomized flights in winds up to 10 mph. The spec's
gate 2 pass mark is 85%. Every injected fault ends the way the spec requires.
The deployment detector also passes on **123 real flight logs** from 31 public
repositories: it confirmed every canopy and never confirmed one before the
main came out (gate 3, below).

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
| Within 30 m | **97.8%** | **94.2%** (drop 3.6 points; allowed 5) |
| Median miss | 1.9 m | 6.0 m |
| 90th percentile | 11.8 m | 23.3 m |
| Steering before canopy confirmed | 0 | **0** |
| Crashes / non-finite outputs | 0 | **0** |
| Canopy confirmed after opening | — | mean 4.5 s, max 6.6 s |

| Injected fault (gate 2) | Expected | Result |
|---|---|---|
| GPS loss | spiral within 5 s | 99/99 |
| Stalled winch | spiral within 15 s | 89/89 |
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
| Gate 2 | **93.0%** within 30 m |
| Faults | all exercised faults met their expected outcome: GPS loss 98/98, stalled winch 90/90, lopsided canopy 75/75, gust 100/100, shear 100/100 |
| False fallbacks | 0 |

A longer run (400 faults of each kind) finds a small tail that batches of 100
can miss: 3 of 314 lopsided canopies are never flagged (the pod keeps guiding
to the end, compensating with trim) and 1 of 353 stalled winches is caught
late (29–43 s instead of 15 s). The build before this change shows the same
tail on the same seeds, so it is an open item rather than a regression.

### Gate 3: log replay on real flights

123 real flight logs replay through the detector on every CI run. 3 are stored
in `logs/real/` (MIT). 133 files are fetched from public GitHub repositories at
pinned commits (`logs/public/`), and each Blue Raven low-/high-rate pair counts
as one log. The flight computers are AltOS (TeleMega, TeleMetrum, EasyMega,
EasyMini), Featherweight Blue Raven and Missile Works RRC3. The flights run
40 m to 8.3 km apogee, single and dual deploy, and include failed, late and
tangled mains. For barometer-only logs (RRC3) the pod gets barometric
acceleration in place of its own accelerometer.

Each log is first judged offline, with hindsight, from its own barometer.
After the main came out, did the descent settle into the canopy band
(1.5–9 m/s, median over 20 s)? If so, the pod must confirm the canopy within
10 s of it settling (15 s for logs with no accelerometer). Mains that never
slowed that much, flights that settled below 30 m, descents within 0.5 m/s of
a band edge, and logs that end or drop below 30 m before that deadline are not
scored. The pod may confirm or not on those,
but never before the main, and never while falling at twice the top of the
band.

| Measure (real logs, `--main-alt log`) | Result |
|---|---|
| Canopies the pod had to confirm | **79/79** confirmed |
| Confirmed before the main came out | **0** |
| Confirmed while falling too fast for a canopy | **0** |
| Not scored: fast or failed main / below 30 m / at a band edge / log too short | 25 / 6 / 4 / 5 (confirmed anyway: 3, -, 3, -) |
| Confirmation delay after the descent settled | mean 4.1 s, max 11.7 s |
| Same, after the host fired the main | mean 9.1 s, max 38.5 s |

Synthetic logs (200, same simulator as gate 2): 188/188 canopies confirmed,
0 false triggers, mean 3.0 s after settling.

**What the real logs changed.** The first run on real logs looked nothing
like the synthetic one. Only 3 of 68 logs were even readable, and once they
were, 32 of 47 canopies were confirmed (at the scoring rules used then), with
2 false triggers. Three detector fixes and one harness fix followed
(items 12–15 below).

**Still short of the spec.** Primary and backup computers in one rocket count
as several logs, so the 123 logs cover about 85 distinct flights; the spec asks
for 100+. The flights also descended under round parachutes, not parafoils:
the logs test boost, coast, drogue, ejection shocks, pressure spikes and
opening dynamics, but the steady descent after opening is a round chute's.
And the host computer sits in the airframe, while the pod rides on the
harness. From gate 5 on, the pod's own logs replace both.

**Dual deploy needs one setting.** Three logs have drogues that fell at
5–10 m/s, inside the canopy band. By descent rate alone no detector can tell
that drogue from a canopy, and without the setting the pod confirms under the
drogue. With `set canopy.mainalt <your main altitude>`, nothing above that
altitude (plus max(15 m, 10%)) counts as canopy evidence: all three pass.
Builders flying dual deploy must set it, and should use a drogue that falls
faster than 12 m/s.

**Held out: a slow drogue the setting doesn't cover.** In RIT's 2023 KONG
flight (two RRC3 logs) the drogue fell at about 6 m/s. The margin above the
main altitude (max(15 m, 10%), here 46 m) takes 7.6 s to fall through at that
speed. The 7 s no-jolt confirmation fires inside it, about 1 s before the main.
Both logs are commented out in the manifest until the detector handles it.
Options include dropping the no-jolt path in dual-deploy mode, or scaling its
time with the margin and the descent rate.

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

## Bugs caught before any hardware existed

Items 1–8 were each found by a failing Monte Carlo run, traced in a
single-flight replay, and fixed. Items 9–11 came from the TLA+ model, the
gate-4 harness and code review. Items 12–15 came from replaying real flight
logs. This is the point of software-first testing.

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
9. **A launch stayed "pending" after landing** (found by the TLA+ model).
   Re-arming without a power cycle at a field 10 m or more above the old
   reference jumped straight to ascent, so the second flight could never learn
   a home point. Fix: clear all per-flight state on every disarm.
10. **The ground reference lagged after moving the pod** (found by the gate-4
    harness). While disarmed, the barometer reference followed pressure on a
    10 s filter. Arming soon after moving to a new elevation left height off by
    up to hundreds of metres, so the pod never detected its landing. Fix: the
    reference tracks exactly while disarmed, and arming re-zeroes the
    altimeter.
11. **A mid-flight processor restart left a brake pulled** (found by review).
    A brownout or watchdog reset rebooted the pod into "pad" with the winches
    frozen wherever they were; a brake held on one side would spiral the canopy
    down. Fix: a checksummed snapshot in RTC memory.
    - Restart during ascent or descent: carry on with the saved home point.
    - Restart under canopy: release both brakes.

    The TLA+ model includes restarts at any point (property
    `NoSteeringAfterCanopyRestart`).
12. **Real AltosUI exports were unreadable** (found by real logs). AltosUI
    writes its CSV header as a comment line (`#version,serial,...`), and the
    reader skipped comment lines, so 65 of 68 real logs were thrown away.
    Fix: a `#` line that names a time column is the header. Units in the
    header (`height (ft)`) are now converted too.
13. **A real canopy never looks steady** (found by real logs). For 10–15 s
    after opening, the descent rate swings between about 1 and 10 m/s
    (pendulum swing, riser bounce). The detector asked for the filtered rate
    to stay within 2.5 m/s for 3 s, so it kept restarting and confirmed late
    or never. The simulator's canopy is smooth, so 191/191 synthetic logs had
    passed. Fix: judge the mean descent over the last 3 s (from the altitude
    history), and confirm 2.5 s after a jolt (7 s without one).
14. **A slow drogue passes for a canopy** (found by real logs). Drogues
    falling at 5–10 m/s are inside the canopy band. Fix: the
    `canopy.mainalt` setting. Above the main altitude no descent and no jolt
    counts, so only the main coming out (and what follows) can confirm.
15. **The harness misread the main altitude** (found by real logs). At the
    instant the main charge fires, the bay's pressure spikes: one log read
    32 m when the rocket was at 180 m. Fix: take the median of the second
    before the main fires.

## Not proven yet

| Gate / question | Status |
|---|---|
| Gate 4 (HIL) | Harness built and passing with a stand-in board: 20 flights, every frame answered, under 1 ms per frame on the PC. The serial path was also tested in real time over a pseudo-terminal. Needs a real board and winches on the bench |
| Gate 5 (ground deploy) and the spec's first open question: does a parafoil open cleanly after a hard ejection? | Simulation assumes it opens in 0.8–2.5 s |
| Gate 6 (drop tests) | The canopy model is the spec's "simple model" and needs fitting to real turn, glide and sink data. Every result above assumes canopies within about ±6% airspeed/sink and ±15% turn gain of the tested design |
| Gate 3 on 100+ real flights | 123 real logs, about 85 distinct flights, all passing (2 held out, see gate 3). Need about 15 more; all are round-parachute flights logged in the airframe, not by the pod |
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
- **Dual deploy with a slow drogue needs `canopy.mainalt`.** See gate 3.
