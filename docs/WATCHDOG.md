# Motor-driver watchdog

A small circuit, standard on every pod, that keeps the winch motor driver
asleep unless the flight software proves, about 100 times a second, that it is
still running. If the firmware hangs, crashes, or simply decides the motors
must be off, the driver goes to sleep within about 0.15 s and both motors
coast, whatever the processor's pins are doing.

It is a second, independent lock on the rule "motors off until the canopy is
open": the firmware only pets the watchdog while the flight core allows the
motors to run, so the motor driver's inputs and its sleep pin both have to
agree before a winch can move.

Simulated in ngspice against every requirement below:
`hardware/watchdog/run_checks.sh` (netlist in `hardware/watchdog/`).

## How it works

1. **Pet.** The 200 Hz flight task toggles Heltec GPIO 20 every control cycle
   (a 100 Hz square wave), but only while the motors may run.
2. **Charge pump.** Each rising edge on the pin pushes a little charge through
   C1 and D2 into the hold capacitor C2. Only a pin that is really being
   driven up and down delivers charge: a pin stuck high, stuck low or floating
   delivers none.
3. **Switch.** While C2 holds more than about 1.2 V, Q1 conducts, Q1 turns on
   Q2, and Q2 pulls the DRV8833's SLP pin up to 3.3 V: driver awake.
4. **Trip.** When the edges stop, C2 drains through R1/R2 in about 0.1 s, Q1
   and Q2 turn off, and R5 pulls SLP to ground: driver asleep, outputs open,
   both motors coast. RA drains C1 so a pin stuck high trips as fast as one
   stuck low.
5. **Power-on clamp.** At power-up GPIO 20 floats up with the 3.3 V rail (it
   is the ESP32-S3's USB D+ pad). That rising edge would pump C2 once, so Q3,
   driven by C3/R6, shorts C2 while the rail comes up and for 50–90 ms after (simulated).

**It fails safe.** Every part in the drive chain (C1, D2, R1, Q1, R3, Q2)
failing open, a broken wire, or a missing 3.3 V leaves SLP pulled low by R5:
the motors never move, which the bench check shows at once. The faults that
would defeat it are a collector-emitter short in Q2 or a solder bridge from
SLP to 3.3 V; step 5 of the bench test catches both.

**Why the DRV8833 needs this circuit at all.** Adafruit's published schematic
for the DRV8833 breakout (product 3297; Eagle files in
`adafruit/Adafruit-DRV8833-Motor-Driver-Breakout-PCB`, commit `dda79a8`)
wires the header's SLP pin straight to the chip's nSLEEP, with no pull-up.
The chip has a 500 kΩ pulldown inside, so an unconnected SLP pin means the
driver stays asleep. Something must drive SLP high, and this circuit is that
something. (Without the watchdog, SLP would have to be tied to 3.3 V.)

The DRV8833's nSLEEP thresholds are stricter than its other inputs (TI
datasheet SLVSAR1E, section 6.5): it reads low below **0.5 V** and high only
above **2.5 V**, and takes up to 1 ms to wake. The circuit drives SLP from
the 3.3 V rail through a saturated transistor so it clears 2.5 V even on a
3.0 V rail.

## Why a charge pump

| Option | Problem |
|---|---|
| **Charge pump + two transistors (chosen)** | Through-hole, about $3, under 2 g. Needs real, full-swing edges to deliver charge, so a floating or noisy pin cannot hold it awake. Fails safe. |
| Retriggerable one-shot (74HC123, DIP-16) | Triggers on any edge, including noise on a floating input; needs input conditioning and a power-on reset of its own. Bigger. |
| Supervisor IC (TPS3823, MAX6369, STWD100) | Surface-mount only; fixed timeouts mostly around 1.6 s (too slow) or extra parts to adjust; their output pulses a reset rather than holding a level. |

## Schematic

Three blocks. Node names (A, B, NB, NC1, NB2, P, NP, SLP) match the
connection list and the simulation netlist. `──►|──` is a diode, anode on
the left.

```
Charge pump and hold

  PET ──┤├──●── A ──►|──●── B ──── R1 47k ──── NB  (to Q1 base)
 GPIO20 C1  │     D2    │
       1µF  │  BAT85    ├── C2 1.5µF ── GND
            │           │
            ├──|◄── GND └── Q3 collector  (power-on clamp, below)
            │   D1 BAT85 (anode GND, cathode A)
            │
            └── RA 33k ── GND


Switch: Q1 (NPN) turns Q2 (PNP high side) on

                3.3V ─────●──────────────┐
                          │              │
                         R4 47k         (E)
                          │          Q2 2N3906
           NC1 ── R3 10k ─●── NB2 ──── (B)
            │                           (C)
           (C)                           │
    NB ── (B) Q1 2N3904                  ●──────► SLP  (DRV8833 breakout)
    │      (E)                           │
   R2 47k   │                          R5 4.7k
    │       │                            │
   GND     GND                          GND


Power-on clamp (shorts C2 while the 3.3 V rail comes up and for 50–90 ms after)

   3.3V ── C3 1µF ──●── P ── R7 22k ── NP ── (B) Q3 2N3904 (C) ── B
                    │                        (E) ── GND
                   R6 220k
                    │
                   GND
```

The same circuit as a connection list (unambiguous; follow this when wiring):

| Part | Value | Pin 1 | Pin 2 (pin 3) |
|---|---|---|---|
| C1 | 1 µF | PET (GPIO 20) | A |
| D1 | BAT85 | anode: GND | cathode: A |
| D2 | BAT85 | anode: A | cathode: B |
| RA | 33 kΩ | A | GND |
| C2 | 1.5 µF | B | GND |
| R1 | 47 kΩ | B | NB |
| R2 | 47 kΩ | NB | GND |
| Q1 | 2N3904 | base: NB | collector: NC1, emitter: GND |
| R3 | 10 kΩ | NC1 | NB2 |
| R4 | 47 kΩ | 3.3 V | NB2 |
| Q2 | 2N3906 | base: NB2 | emitter: 3.3 V, collector: SLP |
| R5 | 4.7 kΩ | SLP | GND |
| C3 | 1 µF | 3.3 V | P |
| R6 | 220 kΩ | P | GND |
| R7 | 22 kΩ | P | NP |
| Q3 | 2N3904 | base: NP | collector: B, emitter: GND |

Current draw: about 1 mA while the motors are enabled, a few µA otherwise.

## Parts

Through-hole, perfboard-friendly, about $3 and about 1.5–2 g with 1/8 W
resistors (estimate; it shares the pod's main perfboard). The distributors'
sites could not be reached from the environment this was written in, so the
table gives manufacturer part numbers (search them on DigiKey or Mouser) for
the semiconductors, and the exact specification for the passives: any part
that meets it works.

| Ref | Qty | Part | Manufacturer number / spec |
|---|---|---|---|
| Q1, Q3 | 2 | NPN small-signal, TO-92 | onsemi **2N3904BU** (any 2N3904) |
| Q2 | 1 | PNP small-signal, TO-92 | onsemi **2N3906BU** (any 2N3906) |
| D1, D2 | 2 | Small-signal Schottky, DO-34/DO-35 | Vishay **BAT85-TR** (BAT43 or 1N5817 also work) |
| C1, C3 | 2 | 1 µF ceramic, X7R, ≥ 16 V, radial, 2.5 or 5 mm leads | e.g. KEMET C330C105K5R5TA or TDK FG28X7R1H105KRT06 |
| C2 | 1 | 1.5 µF ceramic, X7R, ≥ 16 V, radial | ±20 % is fine (simulated). A 1 µF and a 0.47 µF in parallel also work |
| R1, R2, R4 | 3 | 47 kΩ, 5 %, 1/8 W | carbon or metal film |
| R3 | 1 | 10 kΩ, 5 %, 1/8 W | |
| R5 | 1 | 4.7 kΩ, 5 %, 1/8 W | |
| R6 | 1 | 220 kΩ, 5 %, 1/8 W | |
| R7 | 1 | 22 kΩ, 5 %, 1/8 W | |
| RA | 1 | 33 kΩ, 5 %, 1/8 W | |

Avoid electrolytic or tantalum capacitors here: their leakage and wide
tolerance were not simulated.

## Wiring

| Watchdog | Connects to |
|---|---|
| PET | Heltec **GPIO 20** |
| 3.3 V | Heltec **3V3** pin |
| GND | Heltec **GND** (the same ground as the DRV8833) |
| SLP | Adafruit DRV8833 breakout **SLP** pin |

Do not also tie SLP to 3.3 V or to VM: that would defeat the watchdog. Keep
the PET wire short and away from the motor wires.

**Firmware contract.** Toggle GPIO 20 from the flight loop itself with plain
GPIO writes (never from a hardware PWM/LEDC channel or a timer interrupt,
which would keep toggling if the main loop hung), once per control cycle,
only while the flight core enables the motors. Hold it low otherwise. Allow
about 50 ms after the first toggle before expecting the winches to move.

## Simulated margins

ngspice 42; 2,085 runs: the nominal design at 50, 100 and 200 Hz, 34
worst-case corners and 200 Monte Carlo draws over capacitors ±20 %,
resistors ±5 %, transistor gain 0.3–1.5× of the standard model, Schottky
forward voltage about ±30 mV, a 3.0–3.6 V rail and 0–50 °C. Each run powers
up with the pin floating up, pets it, then stops it stuck high, stuck low,
floating up through 1.5 kΩ, or floating free.

| Requirement | Limit | Nominal (3.3 V, 25 °C) | Worst of 2,085 runs |
|---|---|---|---|
| Stays asleep at power-up | SLP ≤ 0.5 V | 0.000 V | 0.000 V |
| Wakes after the first pet edge, 50–200 Hz, and stays awake | ≤ 50 ms | ≤ 20 ms | 40 ms (50 Hz, slow-enable corner: C1 −20 %, C2 +20 %) |
| Stays awake while petted | SLP ≥ 2.5 V | 3.28 V | 2.97 V |
| Sleeps after the last edge (stuck high, stuck low, floating) | ≤ 250 ms | 95–97 ms | 148 ms |
| Stays asleep | SLP ≤ 0.5 V | 0.000 V | 0.000 V |

**One edge on its own.** If the processor resets while idle, GPIO 20 can float
up once. That single edge can wake the driver for up to about 37 ms (worst
case simulated). The motor driver's inputs are low or floating (pulled low
inside the DRV8833) during a reset, which the DRV8833 treats as coast, so
nothing moves.

Models: the standard 2N3904/2N3906 models (as shipped with LTspice and most
vendor libraries) with gain scaled per run, and a BAT85 model approximating
the published one (about 0.22 V at 1 mA). The DRV8833 input is modelled as
its 500 kΩ pulldown plus 10 pF.

Run it yourself: `hardware/watchdog/run_checks.sh` (about 3 minutes with 40
Monte Carlo draws; `--quick` for the nominal design only; `--mc 200` for the
full table above). It exits non-zero if any run misses a requirement.

## Bench test

You need a multimeter; a scope makes steps 2 and 3 easier to see.

1. **Asleep at power-up.** Build the circuit, wire it as above, power the
   Heltec. Measure SLP to GND: **below 0.5 V** (expect about 0 V).
2. **Awake while petted.** In bench mode, command a winch (`winch l 40`):
   the firmware pets GPIO 20 while the winch is commanded. SLP reads **above
   2.5 V** (about 3.2 V). On a scope: a 100 Hz square wave on GPIO 20, a
   steady 2–2.7 V on node B, a flat ~3.2 V on SLP.
3. **Asleep when petting stops.** When the command finishes, SLP falls
   **below 0.5 V within about 0.15 s**.
4. **Unplug the pet wire mid-move.** Command a long move (`winch l 100`) and
   pull the GPIO 20 wire off the circuit while the spool is turning: **the
   motor stops within about 0.15 s** and the encoder stops counting. Plug it
   back in: the firmware is still commanding the move, so it resumes.
5. **Stuck pin.** Repeat step 4, but instead of unplugging, hold the PET input
   to 3.3 V, then to GND, with a jumper wire: the motor stops each time. If
   the motor keeps turning in step 4 or 5, the watchdog is defeated (look for
   a short across Q2 or a bridge from SLP to 3.3 V); do not fly.
6. **Power cycle with the lines rigged.** Neither spool moves at power-up.

Repeat steps 4 and 5 after any rework of the pod's wiring, and before the
first flight of each day.
