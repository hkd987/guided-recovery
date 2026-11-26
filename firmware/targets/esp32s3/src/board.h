// Pin map for the S pod on a Heltec WiFi LoRa 32 V3 / V3.1 / V3.2.
// See docs/WIRING.md for the matching wiring table.
#pragma once

#include <stdint.h>

namespace board {

// Sensor I2C bus (BMI088 or LSM6DSO32, BMP390, optional ADXL375).
constexpr int kI2cSda = 41;
constexpr int kI2cScl = 42;
constexpr uint32_t kI2cHz = 400000;

// u-blox M10 GPS on UART1. kGpsRx is the ESP32's RX (wire to the GPS TX).
constexpr int kGpsRx = 38;
constexpr int kGpsTx = 39;

// DRV8833: one H-bridge per winch.
constexpr int kMotorL_In1 = 47;
constexpr int kMotorL_In2 = 48;
constexpr int kMotorR_In1 = 5;
constexpr int kMotorR_In2 = 6;
constexpr uint32_t kMotorPwmHz = 20000;
constexpr uint8_t kMotorPwmBits = 10;

// Quadrature encoders (power them from 3.3 V).
constexpr int kEncL_A = 4;
constexpr int kEncL_B = 7;
constexpr int kEncR_A = 2;
constexpr int kEncR_B = 26;  // free on the V3 (no PSRAM); not on a V4

// Switches and indicators.
constexpr int kArmSwitch = 34;   // reed switch to GND; magnet present = disarmed
constexpr int kMainFired = 33;   // optional opto-isolated input, active low
constexpr int kBuzzer = 40;      // through an NPN transistor
constexpr int kLed = 35;         // onboard white LED
constexpr int kBatterySense = 3; // 2S pack through a 100k / 33k divider
constexpr float kBatteryDivider = (100.0f + 33.0f) / 33.0f;

// Onboard peripherals (fixed by the Heltec board).
constexpr int kVext = 36;        // low = power the OLED
constexpr int kOledSda = 17;
constexpr int kOledScl = 18;
constexpr int kOledRst = 21;
constexpr int kLoraNss = 8;
constexpr int kLoraSck = 9;
constexpr int kLoraMosi = 10;
constexpr int kLoraMiso = 11;
constexpr int kLoraRst = 12;
constexpr int kLoraBusy = 13;
constexpr int kLoraDio1 = 14;

}  // namespace board
