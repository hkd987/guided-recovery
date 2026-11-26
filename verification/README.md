# Formal verification (TLA+)

The pod falls out of the sky, so its decision logic is model-checked rather
than only tested. `tla/PodLogic.tla` is a TLA+ model of that logic, written
operator by operator from the C++:

| Model operator | C++ it mirrors |
|---|---|
| `Detect` | `PhaseDetector::update` (`firmware/core/src/phase_detector.cpp`) |
| `Update` | `FlightCore::update` (`firmware/core/src/flight_core.cpp`) |

`Update` covers the pad references, the stage-to-phase mapping, `enterCanopy`,
`checkSafety`, `enterSpiral`, motor power, winch fault latching, and the
console requests the ESP32 target applies before each update.

Every sensor reading and timer (high acceleration, steady descent, GPS
silence, a jammed winch, ...) is an **input TLC chooses freely at every
5 ms step**. So a passing check means the property holds for every possible
sequence of sensor readings, faults and console commands, not just the ones a
simulator happens to produce.

## Run it

```sh
cmake -S . -B build -G Ninja && cmake --build build   # builds grtrace
./verification/run_tla.sh                              # needs Java 11+; downloads tla2tools v1.7.4
```

CI runs the same script on every push, in the `formal` job.

## What is proven

TLC explores all **78,464 reachable states** (about 40 million transitions) in
about two minutes. That includes a processor restart at any moment in flight
(`WarmReset`: brownout, watchdog or crash, then `FlightCore::resume`).

**Invariants**, true in every state:

| Property | Meaning |
|---|---|
| `NoSteeringBeforeCanopy` | Brakes are commanded only once the canopy is confirmed open |
| `MotorsOffInRocket` | Winch motors are unpowered on the pad, in boost, coast and descent |
| `MotorsOnlyWhenAllowed` | Motors are powered only under a canopy, after landing (line release), or on the bench |
| `GuidedHasHome` | Guiding always has a saved home point |
| `NoGuidanceThroughHazard` | GPS loss, geofence, lopsided canopy, low battery or collapse end guidance in the same step they are detected |
| `BenchOnlyWhenDisarmed` | Console motor control exists only with the magnet on |
| `SpiralHasReason` | Every spiral has a recorded fault |
| `PhaseMatchesStage` | What the pod does always agrees with the detected flight stage |
| `DisarmedMeansNoLaunchPending` | Disarming forgets any suspected launch |
| `NoSteeringAfterCanopyRestart` | After a restart under canopy, the pod releases the brakes and never steers again that flight |

**Step properties**, true of every transition:

| Property | Meaning |
|---|---|
| `HomeFrozenInFlight` | The home point cannot change from the moment a launch is suspected until after landing |
| `FreshArming` | Every arming starts with no pending launch and relearns home |
| `NoDisarmInFlight` | The magnet cannot disarm the pod in flight |
| `FallbacksLatch` | Once spiralling or released, the pod never goes back to guiding |
| `StallEndsGuidance` | A jammed winch, even one jammed on the bench, ends guidance at the next step |
| `CollapseReleases` | A collapse always releases both brakes |

**Liveness:** `LandedMotorsOff`. After landing, the motors are eventually
switched off.

## How we know it isn't vacuous

- **Witnesses.** Nine key situations are shown to be reachable: guiding, GPS
  spiral, collapse, a bench jam carried into flight, no-home spiral, landing
  without a canopy, arming with a latched jam, releasing after a restart
  under canopy, and guiding after a restart in descent. If a property only held
  because those states could never happen, a witness would show it.
- **Mutations.** Six known bugs are re-injected one at a time. TLC catches
  every one, with a counterexample of 4–7 steps:
  - `stale_candidate`, the real bug described below;
  - motors powered in descent;
  - home point updated after launch;
  - GPS failsafe removed;
  - magnet able to disarm in flight;
  - steering resumed after a restart under canopy.

## Is the C++ the same as the model? (trace validation)

`sim/apps/grtrace.cpp` runs the real C++ flight core through:

- simulated flights (nominal and every fault type), continuing after
  touchdown: line release, magnet on, re-armed at a field 12 m higher;
- scripted bench sessions: jog, jam, console reset, home learning, handling
  bumps, disarming with a launch pending, bench commands refused while armed;
- processor restarts in descent and under canopy.

It records every change of the decision state. `tla/PodTrace.tla` then checks
that **each recorded change is one step the model allows**. Current result:
226 traces, 2,707 state changes, 0 disallowed.

When the stale-flag bug was put back into the C++, this check rejected 50
steps.

## A bug this found

While modelling the launch-candidate flag, it became clear the C++ never
cleared it after a flight. TLC confirms this with the `stale_candidate`
mutation: a 6-step counterexample of arm → launch → apogee → land → disarm,
ending with a launch still pending.

The consequence: re-arming without a power cycle at a field 10 m or more
above the old reference made the pod jump straight to "ascent". Weather drift
alone can do that. The pod could then never learn a home point, so its second
flight would only spiral.

Fixed in `PhaseDetector::clearFlight()`, with a regression test
(`detector_rearm_after_flight_starts_fresh`).

## What the model does not cover

- **Numeric thresholds.** Whether 3 g or a 3 s steady descent is right is
  settled by gates 1–3 (simulation and log replay) and by flight tests.
- **Guidance, estimation and steering maths.** These are tested by gates 1–2
  and the unit tests.
- **Drivers and timing on the real ESP32**, such as the 200 Hz loop and the
  I2C bus. That's gate 4, hardware in the loop.
- **Trace validation is a sample.** It shows the C++ matched the model on the
  scenarios run, not on all inputs. The model itself is what's checked for
  all inputs.
