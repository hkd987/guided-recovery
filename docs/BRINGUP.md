# Bench bring-up: one block at a time

Use this for the first pod you build, and for any pod after a rewire. Each
step proves one block before the next block depends on it, so a fault shows up
on the step that caused it. Wiring: [WIRING.md](WIRING.md). Console: USB at
115200 baud (`pio device monitor -b 115200` or the installer's "Logs &
console").

Keep the magnet on (disarmed) the whole time. Disarmed is bench mode: the
winches only move when you command them.

## 0. Before any power

| Check | Expect | If not |
|---|---|---|
| Resistance 2S+ to GND at the battery plug | not a short; climbs as capacitors charge | find the short before connecting anything |
| Buck output with nothing connected, powered from the 2S pack | 5.0 V ± 0.25 V | wrong buck or wiring; don't connect the Heltec |
| 2S pack | 7.4–8.4 V | charge it |
| Battery divider midpoint (GPIO 3 wire) with the pack connected, Heltec unplugged | about 0.25 × pack voltage (2.1 V at 8.4 V) | check 100 k / 33 k values and order |

## 1. Heltec alone, on USB

Unplug the pack. Flash with the browser installer, open the console.

| Do | Expect |
|---|---|
| Watch the boot text | `GPS: no response`, sensor errors and `Radio: ok` are all fine at this stage; `Radio: failed` is not (board fault, or not a V3) |
| `status` | phase DISARMED |
| `params` | `canopy.mainalt 0 m (0: single deploy...)`; set yours now if you fly dual deploy |

## 2. Sensors (I2C on 41 / 42, 3.3 V)

Wire the IMU and barometer. Stand the pod upright.

| Do | Expect | If not |
|---|---|---|
| `selftest` (sensor lines) | IMU found (LSM6DSO32 at 0x6A/0x6B or BMI088 at 0x18/0x19 + 0x68/0x69), BMP390 at 0x76/0x77, optional ADXL375 at 0x53 | swap SDA/SCL, check 3.3 V and GND, one bad solder joint is the usual cause |
| `selftest` (orientation) | body z points down, about 1 g | `set imu.map ...` as the self-test suggests, then `save` |
| `watch`, then lift the pod 1 m | height rises about 1 m, then returns | barometer not reading: check its address and supply |

## 3. Power from the pack

Unplug USB. Connect the 2S pack through the buck to the Heltec 5V pin. Then
reconnect USB **only if** the buck output has a diode (see WIRING.md); otherwise
use the radio link or reconnect the pack later.

| Do | Expect | If not |
|---|---|---|
| Board boots on the pack alone (OLED shows the status screen) | yes | buck output, 5V pin, GND |
| `status` battery | within 0.1 V of a multimeter on the pack | `set batt.scale <meter / reading × current scale>`, `save` |

## 4. GPS (UART on 38 / 39)

Take the pod outside, antenna up, open sky.

| Do | Expect | If not |
|---|---|---|
| Boot text | `GPS: ok` | swap TX/RX (pod pin 38 goes to GPS TX); check the module's supply voltage |
| `gps` after 1–5 min | 3D fix, 6+ satellites, hAcc under 5 m | wait longer away from buildings; check the antenna faces up |
| Buzzer after arming (magnet off) | fast chirps, then one chirp every 2 s once the home point is saved | no home point means the pod will only spiral; fix GPS first |

Put the magnet back on before moving on.

## 5. Winches, lines off

Spools turn freely, no brake lines attached.

| Do | Expect | If not |
|---|---|---|
| Power up with the pack and watch the spools | neither moves until commanded | DRV8833 inputs miswired; stop |
| `winch l 40` | left spool winds in 40 mm in under 1 s, then holds | `set winch.dirL -1` if it pays out; check encoder wiring if it never stops |
| `winch r 40` | same, right | `set winch.dirR -1` |
| `winch release` | both return to 0 | |
| `selftest` (winch lines) | each pulls 25 mm and releases; line speed, direction and "no jam" reported | follow the setting it names (`winch.encL`, `winch.dirL`, `winch.cpmm`) |

## 6. Inputs and radio

| Do | Expect | If not |
|---|---|---|
| Magnet on / off, `status` | `armed no (magnet on)` / `armed yes`, phase DISARMED / PAD | reed switch to GPIO 34 and GND |
| Optional main-fired input: drive the opto LED from a bench supply through its resistor | `status` shows `main-fired input ON` | opto output polarity; never wire it to the host's igniter circuit |
| Ground receiver (second Heltec) 100 m away | phase and height on its screen | both on the same `radio.mhz` |
| `selftest` (loop timing) | max loop well under 5 ms, 0 overruns | report it: something is blocking the flight task |

## 6b. Watchdog and backup kit

| Do | Expect | If not |
|---|---|---|
| `selftest` (watchdog line) | `motor-driver watchdog holds the motors off`, then the winch tests pass | see [WATCHDOG.md](WATCHDOG.md); "moved without the watchdog signal" means the circuit is missing or SLP is tied to 3.3 V |
| Mid-move, unplug the GPIO 20 wire (`winch l 60`, pull the wire) | the winch stops within a quarter of a second | check the watchdog circuit |
| **Unpowered winch under load:** brakes at 60 mm on the spring scales, unplug the 2S pack | note whether the line pays out | this is what a dead pod does in flight: if the gearbox holds the brake, rig a lighter brake-line tension or accept that a dead pod keeps its last brake (the simulator assumes it pays out above 2.5 N) |
| Backup kit, second barometer: boot text | `baro BMP390 + second BMP390 (backup kit)` | its SDO pad must go to GND (0x76) so the two addresses differ |
| `selftest` | `second barometer ... agrees with the first` | over 300 Pa apart: a damaged sensor or a bad solder joint |
| `status` | `baro trusted, baro 2 ok` | |
| Backup kit, second GPS outdoors: `status` | `gps2 fitted (standby)` | GPS 2 TX to GPIO 19; its RX shares GPIO 39 with GPS 1 |
| Sense line: drive the opto LED | `status` shows `main-fired input ON` | then decide `sense.main` 1 or 2 |

## 7. Rig the lines, then gate 4

1. Rig the brake lines so they are just slack at 0 mm; `zero`, `log erase`, `save`.
2. Tie each brake line to a spring scale or bungee giving about 4 N at full travel.
3. Run the bench hardware-in-the-loop test (BUILD_GUIDE.md section 5):
   `./build/grhil --port /dev/ttyUSB0 --flights 3 --out gate4.json`, magnet off.
4. Note the spring-scale reading at full brake and the line speed `grhil`
   reports. Those are the first real winch numbers; the 4–6 N load in the
   feasibility budget comes from a single forum data point.

## 8. Shock check before anything flies

Real flight logs show the main opening at 8–35 g in the avionics bay, with
outliers up to 140 g (docs/FEASIBILITY.md, gate 3). The simulator assumed 6 g.

- Drop the complete, powered pod (with a dummy load on the Kevlar loop) onto
  a padded floor from 1 m a few times, then from the height that snaps the
  loop taut with the rocket's weight on it.
- After each drop: `selftest` must pass unchanged, the winch zero must not have
  moved (`status` mm), and nothing may be loose.
- Then the spec's gate 5: ground ejection tests of the packed parafoil.

Keep every log from here on (`log dump`). From gate 5 the pod's own logs become
the real replay test suite.
