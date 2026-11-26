# S pod wiring (Heltec WiFi LoRa 32 V3)

The pin map lives in `firmware/targets/esp32s3/src/board.h`. It assumes a
**V3, V3.1 or V3.2** board. The **V4** uses GPIO 2, 5, 7, 26, 34 and 38–42 for
its own LoRa amplifier, PSRAM and GNSS connector, so it needs a different map.

| Function | Part pin | Heltec V3 GPIO | Notes |
|---|---|---|---|
| I2C SDA | IMU, BMP390, ADXL375 SDA | **41** | 400 kHz, sensors in parallel, 3.3 V |
| I2C SCL | IMU, BMP390, ADXL375 SCL | **42** | |
| GPS → ESP32 | GPS **TX** | **38** | UART1 RX. Firmware finds the baud and configures 115200, UBX, 10 Hz |
| ESP32 → GPS | GPS **RX** | **39** | UART1 TX. The GPS UART must be 3.3 V logic |
| GPS power | GPS VCC / GND | 5 V rail or 3.3 V | per the module listing: most FPV M10 modules (HGLRC M100 Mini included) take 5 V; put a 3.3 V-only module on the 3.3 V pin |
| Left motor | DRV8833 AIN1 / AIN2 | **47 / 48** | 20 kHz PWM. AOUT1 / AOUT2 → left gearmotor |
| Right motor | DRV8833 BIN1 / BIN2 | **5 / 6** | BOUT1 / BOUT2 → right gearmotor |
| Motor power | DRV8833 VM / GND | 2S + / GND | GND shared with the Heltec. Leave SLP as the breakout ships it (pulled high = awake) |
| Left encoder | A / B | **4 / 7** | power encoders from **3.3 V** (outputs pull up to their supply) |
| Right encoder | A / B | **2 / 26** | GPIO 26 is free on the V3 only |
| Arming | reed switch to GND | **34** | magnet on = switch closed = disarmed (bench mode) |
| Main-fired input (optional) | opto-isolator output to GND | **33** | never connect to the host's igniter circuit directly |
| Buzzer | NPN base (via 1 kΩ) | **40** | magnetic buzzer: add a flyback diode |
| Battery sense | 2S+ → 100 kΩ → GPIO → 33 kΩ → GND | **3** | calibrate with `set batt.scale` |
| Onboard LED | — | 35 | built in |
| OLED | — | 17 / 18 / 21, Vext 36 | built in |
| LoRa SX1262 | — | 8–14 | built in |

Spare: 19, 20 (USB D−/D+ pads), 45, 46. 45 and 46 are strapping pins, so
don't put pulled-up encoder inputs on them. Not spare even though they reach
the header: 0 (PRG button), 1 and 37 (the board's own battery sense), 21
(OLED reset), 43 / 44 (USB serial), 36 (Vext).

Checked against the Arduino core's Heltec V3 variant and Meshtastic's V3
definition: no pod pin clashes with the onboard LoRa radio, OLED, LED, USB
serial or boot-mode pins. GPIO 26, 33 and 34 are free because the V3's
ESP32-S3FN8 has no PSRAM (on PSRAM chips those pins are taken).

**Vext (GPIO 36) must stay low.** It powers the OLED and the LoRa front end;
the firmware never turns it off.

**Motors stay off while the board boots.** The DRV8833 inputs have internal
pull-downs, so while the ESP32 pins float during reset both bridges coast.
Check it once: power the pod with the lines attached and watch that neither
spool moves until you command it.

## Power

```
2S LiPo 7.4–8.4 V ──┬──────────────── DRV8833 VM (motors)
                    ├── 100k ──┬── GPIO 3
                    │         33k
                    │          └── GND
                    └── 5 V buck ──── Heltec 5V pin ── (board regulates 3.3 V for sensors/encoders)
```

- **2S only.** The DRV8833 is rated 10.8 V maximum, so a 3S pack destroys it.
- The motors are 6 V parts. The firmware caps average motor voltage at 6 V
  (`motorMaxVolts`) and cuts a stalled motor within about 0.4 s.
- **Do not plug in USB while the 2S pack is connected** unless you add a diode
  or ideal-diode on the buck output. The Heltec 5 V pin and USB VBUS may be
  tied together; this is unverified.
- Feed sensors and encoders from the Heltec's 3.3 V pin (the board's own
  regulator; the pod's 3.3 V load is under about 100 mA).
- Bring the pod up one block at a time: [BRINGUP.md](BRINGUP.md).

## Mounting

- The GPS patch antenna faces up from the top of the pod. The pod hangs under open sky.
- The IMU can sit in any orientation. Run `selftest` with the pod upright,
  and if it reports the orientation is wrong, change `imu.map`. Example:
  `set imu.map +x-y-z` means body x = +sensor x, body y = −sensor y,
  body z = −sensor z (board flat, components up). Body z must point **down**.
- The rocket's weight goes through the Kevlar loop, never the printed shell.
