// GPS, winch motors and encoders, switches, battery and buzzer.
#pragma once

#include <Arduino.h>

#include "gr/types.hpp"
#include "gr/ubx.hpp"

// ---- GPS (u-blox M10 on UART1)
struct GpsStatus {
  bool configured = false;
  uint32_t baudFound = 0;
  uint32_t fixes = 0;
};
extern GpsStatus g_gps;
// Find the receiver's baud rate, send the pod configuration, verify NAV-PVT.
bool gpsBegin(uint32_t timeoutMs = 4000);
// Drain the UART; returns true when a new NAV-PVT fix was decoded.
bool gpsPoll(gr::GpsFix& fix);
const gr::ubx::Parser& gpsParser();

// ---- Winches
void motorsBegin();
// duty in -1..1; positive pulls the brake line in. enabled=false lets both
// motors coast (pad, disarmed); duty 0 while enabled brakes the motor.
void motorsWrite(float dutyL, float dutyR, bool enabled);
void encodersBegin();
// Accumulated counts since boot (or since encodersZero).
void encodersRead(int32_t& left, int32_t& right);
void encodersZero();
// After a restart in flight: continue counting from the saved positions.
void encodersRestore(int32_t left, int32_t right);

// ---- Switches, battery, buzzer, LED
void ioBegin();
bool armSwitchArmed();   // debounced: true once the magnet is removed
bool mainFiredInput();   // debounced
float batteryVolts();
void buzzer(bool on);
void led(bool on);
