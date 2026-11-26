# Build guide: S pod (rockets up to 1.5 kg)

This is the parts list people buy, then install the Guided Recovery OS onto.
It's a **prototype** build. Two choices stay provisional until testing:

- **Winch size:** lock it after the hardware-in-the-loop force tests (gate 4).
- **Parafoil size:** lock it after drop tests (gate 6).

## 1. Parts

Prices are single-unit, checked against web listings in November 2026. Most
came from search snapshots, so confirm them before ordering. Shipping from
several vendors adds up; DigiKey stocks the Adafruit and Pololu parts in one
order.

| # | Part | Suggested listing | ≈ $ | Notes |
|---|---|---|---|---|
| 1 | **Heltec WiFi LoRa 32 V3** (ESP32-S3 + SX1262, 915/868 MHz to match your region) | Heltec store, Rokland | 20 | **Must be V3 / V3.1 / V3.2.** Resellers are starting to ship V4, which needs a different pin map |
| 2 | IMU: **LSM6DSO32** breakout (±32 g, 2000 dps) | Adafruit 4692 | 12.50 | Recommended. **BMI088** (Seeed Grove, ~$28) also works; the firmware detects either |
| 3 | Barometer: **BMP390** breakout | Adafruit 4816 | 10.95 | |
| 4 | GPS: **u-blox M10** module with patch antenna, UART | HGLRC M100 Mini (RaceDayQuads) | 21.50 | Any M10 with UART works. The firmware configures it on every boot |
| 5 | Winches: 2× **Pololu 6 V HPCB 210:1 micro metal gearmotor**, extended shaft, with magnetic encoder | Pololu (210:1 HPCB w/ encoder) | 64.90 | 150:1 also works. **Not** generic N20s: too weak for the brake-line load |
| 6 | Motor driver: **DRV8833** dual H-bridge | Adafruit 3297 | 5.95 | 2S only (10.8 V max) |
| 7 | Battery: **2S LiPo 450 mAh** | Tattu 2S 450 75C | 7.80 | Enough for several flights plus long pad waits |
| 8 | 5 V buck, ≥1 A, 6–9 V in | Pololu D24V10F5 (or Adafruit UBEC 1385) | 13.95 | Feeds the Heltec 5V pin |
| 9 | Arming: reed switch + magnet | Adafruit 375 | 3.95 | Magnet on a remove-before-flight tag |
| 10 | Buzzer, 85 dB, + NPN transistor + diode | Same Sky CMI-1295-85T | 5 | For finding the rocket in tall grass |
| 11 | Structure: printed PETG/nylon shell, Kevlar load loop, 2 swivels, 2 quick links, PTFE line guides, 16 mm spools | — | ~20 | Estimate. The load path is Kevlar, never plastic |
| 12 | Wire, connectors, 100 k + 33 k resistors | — | ~5 | Estimate |
| 13 | **Motor-driver watchdog**: 2× 2N3904, 2N3906, 2× BAT85, 1 µF ×2, 1.5 µF, 8 resistors | [WATCHDOG.md](WATCHDOG.md) | ~3 | Standard. Holds the motor driver asleep unless the flight loop keeps proving it's alive |
| | **Total** | | **≈ $194** | Optional ADXL375 high-g logger (Adafruit 5374, $24.95) brings it to ≈ $219 |

### Optional backup kit

None of this is needed for the pod to be safe: any sensor failure already
ends in "stop steering". The kit lets more flights still come home when a
sensor fails, and adds independent checks on the canopy-open decision. The
firmware finds whatever is fitted.

| Part | ≈ $ | Weight | What it buys |
|---|---|---|---|
| Second BMP390 (Adafruit 4816, SDO soldered to GND) | 11 | 1.5 g | Two barometers cross-check each other, GPS breaks ties: a failed one is dropped and guidance carries on |
| "Main fired" sense line: opto-isolator (e.g. PC817) + resistor | 1 | 1 g | Independent proof the main came out. `set sense.main 2` makes it **required** for the canopy decision |
| Second GPS (any M10 module, like item 4) | 21.50 | 6–10 g | Takes over if GPS 1 goes quiet. Helps most at sites with poor sky view |

The backup kit adds ≈ $34 and 9–13 g (under 1% of a 1.5 kg rocket).
| — | Test parafoils, 2–3 sizes (RC or balloon-payload market) | | TBD | Aim for **~8 m/s trim airspeed** at your pod + rocket weight (see FEASIBILITY.md) |

**Changes from the spec's draft list (v0.1, ~$170):**
- **Dropped the LIS3MDL magnetometer.** Guidance steers on GPS course and
  derives heading from GPS velocity minus the estimated wind. A compass next
  to two motors and a battery would mostly be noise anyway.
- **Dropped the 16 MB SPI flash.** Logs go to a 5.4 MB partition of the
  ESP32's own flash, about 28 minutes at 50 Hz. This also frees 4 GPIOs, and
  the V3 has only 20 spare.
- **N20 gearmotors replaced with Pololu HPCB 210:1.** About 160 rpm, about 34 N
  stall force on a 16 mm spool against a ~4–6 N brake-line load: full brake in
  about 1 s.
- **BMI088 → LSM6DSO32 as the default IMU.** It's easier to buy, half the
  price, and its ±32 g range catches opening shocks. Both are supported.

## 2. Wire it

Follow [WIRING.md](WIRING.md). Key points:
- 2S only.
- Encoders on 3.3 V.
- Reed switch to GND on GPIO 34.
- Don't plug in USB while the battery is connected (unless the buck output has a diode).

## 3. Install the OS

**Browser (easiest):** open the installer page
(`https://hkd987.github.io/guided-recovery/`) in desktop Chrome or Edge. Unplug
the 2S pack, connect the Heltec by USB-C, click **Install**, and accept the
erase prompt on a first install. The page also installs the ground receiver.

**From source:**

```sh
pip install platformio
git clone <this repo> && cd guided-recovery/firmware/targets/esp32s3
pio run -e pod -t upload          # Heltec V3 on USB-C
pio device monitor -b 115200
```

The first flash also writes the partition table, which includes the flight-log
partition. Optional: flash a second Heltec V3 as a ground receiver with
`pio run -e ground -t upload`. It shows the pod's phase, height and position
on its own screen.

With the "main fired" sense line fitted (backup kit), choose how much it
counts: `set sense.main 1` (default) treats it as evidence like an opening
jolt; `set sense.main 2` refuses to confirm a canopy without it. Use 2 only
when the sense line is wired and bench-tested.

Set your radio band before flying outside the US. The default is 915 MHz.

```
set radio.mhz 868
save
```

**Dual deploy:** tell the pod the main altitude you set on your flight
computer (in metres above the pad). The default, 0, means single deploy: the
parafoil comes out at apogee.

```
set canopy.mainalt 150
save
```

Nothing above that altitude counts as the canopy opening. Without it, a
drogue that falls slower than about 9 m/s looks exactly like a parafoil and
the pod would start steering while the parafoil is still packed. Use a drogue
that falls faster than 12 m/s anyway.

## 4. Bench setup and self-test

First time? Follow [BRINGUP.md](BRINGUP.md): it brings the pod up one block
at a time (power, sensors, GPS, winches, inputs, radio) with the readings to
expect, then the gate 4 bench test and a shock check.

Put the magnet on (disarmed = bench mode), stand the pod upright, and type:

```
selftest
```

It checks the following:

| What | Check |
|---|---|
| Sensors | Every sensor answers, with plausible values |
| IMU orientation | Body z points down |
| GPS | Configured and fixing |
| Battery | Voltage in range |
| Log space | Free space left |
| Radio | Starts up |
| Each winch | Pulls 25 mm and releases; reports line speed, direction and jams |

It tells you which setting to change. Common fixes:

| Self-test says | Do |
|---|---|
| body z is not pointing down | `set imu.map ...` (see WIRING.md), `save` |
| encoder counts backwards | `set winch.encL -1` (or swap encoder A/B) |
| line paid out instead of pulling in | `set winch.dirL -1` |
| winch did not reach 25 mm | check encoder wiring; `winch.cpmm` = 12 × gear ratio ÷ (π × spool diameter mm) |

Rig the brake lines so they are just slack at 0 mm. Then run `zero` with the
lines slack, `log erase` (about 15 s; erasing during flight would stall the
control loop), and `save`.

Other console commands: `status`, `watch`, `winch l 40`, `winch release`,
`params`, `set`, `log info`, `log dump`, `gps`, `help`.

## 5. Bench hardware-in-the-loop (gate 4)

Before anything flies, let the real board fly simulated flights. The PC sends
simulated IMU, barometer and GPS readings over USB 200 times a second. The
board makes every decision, its **real winches move**, and their encoder
positions steer the simulated canopy.

1. Mount the pod upright on the bench. Tie each brake line to a spring scale or
   bungee, so the winches pull against roughly flight load (about 4 N at full
   travel for the S pod).
2. Connect USB with the magnet **on**. Build the PC tool (`cmake ... && cmake
   --build build`), then run:
   ```sh
   ./build/grhil --port /dev/ttyUSB0 --flights 3 --out gate4.json
   ```
   The tool types `hil` into the console for you.
3. Remove the board's magnet to let the winches move. While it is on, the
   motors are held off and the tool reports a FAIL.
4. Each flight runs in real time, about 1–4 minutes. Gate 4 passes when:
   - every frame is answered;
   - the board never needs more than 5 ms for a frame;
   - every flight lands, and the board detects the landing;
   - nothing steers before the canopy opens.
5. Reboot the board to leave HIL mode.

`./build/grhil --loopback` runs the same thing against a stand-in board on
the PC. CI runs that on every push.

## 6. At the pad

1. Pack the parafoil and pod as rehearsed in ground deployment tests (gate 5).
2. Pull the magnet. The pod is now **armed**.
3. The buzzer **chirps fast** until it has a GPS home point, then **chirps once every 2 s**.
4. Wait for the slow chirp before launch. The OLED shows `HOME saved`.
5. If the pod never gets a home point, it still flies, but it only spirals
   after the canopy opens. It never guesses a target.

After landing, the buzzer sounds every 2 s. Put the magnet back on, connect
USB, and run `log dump` to get the CSV. Keep every log: from gate 5 on, the
pod's own logs become the real replay test suite.

## 7. Before the first flight

Fly the gates in order: bench HIL (4), ground deploys (5), drop tests (6),
low-power (7), high-power (8). Fly under your club's rules and NAR/Tripoli
codes.

The pod never connects to or fires any pyro channel. Treat the optional
"main fired" input as a sense line only, through an opto-isolator, and
bench-test it against your host computer's continuity check first.
