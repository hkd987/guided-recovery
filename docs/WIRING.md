# S pod wiring (Heltec WiFi LoRa 32 V3)

The pin map lives in `firmware/targets/esp32s3/src/board.h`. It assumes a
**V3, V3.1 or V3.2** board. The **V4** uses GPIO 2, 5, 7, 26, 34 and 38–42 for
its own LoRa amplifier, PSRAM and GNSS connector, so it needs a different map.

| Function | Part pin | Heltec V3 GPIO | Notes |
|---|---|---|---|
| I2C SDA | IMU, BMP390, ADXL375 SDA | **41** | 400 kHz, sensors in parallel, 3.3 V |
| I2C SCL | IMU, BMP390, ADXL375 SCL | **42** | |
| GPS → ESP32 | GPS **TX** | **38** | UART1 RX. Firmware finds the baud and configures 115200, UBX, 10 Hz |
| ESP32 → GPS | GPS **RX** | **39** | UART1 TX |
| Left motor | DRV8833 AIN1 / AIN2 | **47 / 48** | 20 kHz PWM |
| Right motor | DRV8833 BIN1 / BIN2 | **5 / 6** | |
| Left encoder | A / B | **4 / 7** | power encoders from **3.3 V** (outputs pull up to their supply) |
| Right encoder | A / B | **2 / 26** | GPIO 26 is free on the V3 only |
| Arming | reed switch to GND | **34** | magnet on = switch closed = disarmed (bench mode) |
| Main-fired input (optional) | opto-isolator output to GND | **33** | never connect to the host's igniter circuit directly |
| Buzzer | NPN base (via 1 kΩ) | **40** | magnetic buzzer: add a flyback diode |
| Battery sense | 2S+ → 100 kΩ → GPIO → 33 kΩ → GND | **3** | calibrate with `set batt.scale` |
| Onboard LED | — | 35 | built in |
| OLED | — | 17 / 18 / 21, Vext 36 | built in |
| LoRa SX1262 | — | 8–14 | built in |

Spare: 19, 20 (USB D−/D+ pads), 45, 46. These are strapping pins, so don't
put pulled-up encoder inputs on 45 or 46.

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
- Feed sensors and encoders from the Heltec's 3.3 V pin.

## Mounting

- The GPS patch antenna faces up from the top of the pod. The pod hangs under open sky.
- The IMU can sit in any orientation. Run `selftest` with the pod upright,
  and if it reports the orientation is wrong, change `imu.map`. Example:
  `set imu.map +x-y-z` means body x = +sensor x, body y = −sensor y,
  body z = −sensor z (board flat, components up). Body z must point **down**.
- The rocket's weight goes through the Kevlar loop, never the printed shell.
