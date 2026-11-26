# Guided Recovery OS

Firmware and test harness for the **Guided Recovery Pod**: a small steering
unit that clips into the harness between a model rocket and a ram-air
parafoil, works out on its own when the canopy has opened, and flies it back
to the launch pad. It needs no wiring to the rocket's flight computer.

The first product is a **parts list** (the S pod, rockets up to 1.5 kg).
Builders buy the parts, wire them to a Heltec WiFi LoRa 32 V3 board, and flash
this OS onto it.

## Status

| Gate | What it proves | Where | Result |
|---|---|---|---|
| 1 | Guidance law lands the canopy | Simple parafoil model, perfect knowledge | **PASS** 97.8% within 30 m (1,000 runs, winds to 10 mph) |
| 2 | The real flight code does it with real-world sensors | Flight core + sensor/winch models, pad to touchdown | **PASS** 94.2% within 30 m; every injected fault ends as specified |
| 3 | Deployment detector catches the main, ignores boost/coast/drogue | Replay of **123 real flight logs** from 31 public repos (+200 synthetic) | **PASS** every canopy confirmed (79/79), none before the main; about 85 distinct flights so far, the spec asks for 100+ |
| 4 | The real board flies simulated flights with its real winches | `grhil` over USB, 200 Hz | **Harness ready.** PASS with a stand-in board (20 flights, every frame answered); needs a real board |
| 5–8 | Ground deploys, drop tests, flights | Field | Not started (needs hardware) |
| Formal | Guardrails hold for **every** possible sensor/fault/restart sequence | TLA+ model checked with TLC; C++ traces validated against it | **PASS** 17 properties, 78,464 states; 2,707/2,707 C++ steps conform |

Firmware for the ESP32-S3 (pod and LoRa ground receiver) **compiles**. It has
**not yet run on hardware**. See [docs/FEASIBILITY.md](docs/FEASIBILITY.md)
for the full findings, including what is not proven yet.

## Repository

```
firmware/core/          the OS: portable C++17 flight software (no hardware dependencies)
firmware/targets/esp32s3/  Heltec V3 drivers, 200 Hz flight task, console, self-test (PlatformIO)
sim/                    simulator: rocket, wind, parafoil, winch motors, sensor models, faults
sim/apps/grsim.cpp      Monte Carlo runner for gates 1 and 2 (+ canopy sweep, synthetic logs)
sim/apps/grreplay.cpp   gate 3: replays host flight-computer logs through the detector
sim/apps/grhil.cpp      gate 4: hardware in the loop over USB (or --loopback with a stand-in board)
tests/                  unit tests
verification/           TLA+ model of the decision logic, model checking + trace validation
tools/report.py         HTML report from gate outputs
logs/real/              real flight-log CSVs (yours, plus 3 MIT-licensed); CI replays them
logs/public/            manifest of 133 public real flight-log files CI downloads for gate 3
docs/                   build guide, wiring, architecture, feasibility
```

## Run the software gates

```sh
cmake -S . -B build -G Ninja && cmake --build build
./build/grtests                                   # unit tests
./build/grsim gate1 --runs 1000 --out reports     # gate 1
./build/grsim gate2 --runs 1000 --fault-runs 100 --out reports --baseline sim/baseline.json
./build/grsim synth-logs --count 200 --out logs/synthetic
./build/grreplay --main-alt log --min-flights 100 --out reports/gate3.csv logs/synthetic/*.csv
tools/fetch_public_logs.sh                        # 133 public real log files, pinned + checksummed
./build/grreplay --main-alt log --min-flights 50 --out reports/gate3_real.csv logs/real/*.csv build/public-logs/*.csv
python3 tools/report.py reports reports/index.html
```

Other useful commands:

```sh
./build/grsim single --seed 42 --trace trace.csv          # one flight, full trace
./build/grsim single --seed 7 --fault gps_loss            # one fault scenario
./build/grsim sweep --airspeeds 6,7,8,9 --winds 2.2,4.5,6.7   # canopy size vs wind
```

CI (`.github/workflows/ci.yml`) runs all of this on every push and fails if
gate accuracy drops more than 2 points below `sim/baseline.json`. It also
builds both firmware images.

Formal verification of the decision logic (needs Java): `./verification/run_tla.sh`.
See [verification/README.md](verification/README.md).

## Install the OS on a pod

See [docs/BUILD_GUIDE.md](docs/BUILD_GUIDE.md).

**From the browser, with no tools:** open the installer page
(`https://hkd987.github.io/guided-recovery/`), plug
in the Heltec V3, and click Install. CI builds it from every push to `main`;
`tools/make_installer.sh` builds the same site locally.

**From source:**

```sh
pip install platformio
cd firmware/targets/esp32s3
pio run -e pod -t upload        # USB-C to the Heltec V3
pio device monitor -b 115200    # then type: selftest
```

## Guardrails (enforced in `firmware/core/src/flight_core.cpp`)

- Motors are unpowered until the canopy is confirmed open. Steering never arms before that.
- The only possible target is the home point the pod saves itself on the pad. It is frozen at launch.
- Losing GPS, leaving the geofence, a jammed winch, a lopsided canopy or a low battery ends guidance in a slow spiral.
- A canopy collapse releases both brakes.
- A processor restart in flight resumes from a snapshot kept in RTC memory. Under a canopy it resumes with both brakes released.
- The pod never connects to, and never fires, any pyrotechnic channel.
