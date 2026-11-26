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
// sweepAll first sends the configuration at every baud rate, so a second
// receiver sharing the TX line is configured whatever its rate.
bool gpsBegin(uint32_t timeoutMs = 4000, bool sweepAll = true);
// Drain the UART; returns true when a new NAV-PVT fix was decoded.
bool gpsPoll(gr::GpsFix& fix);
const gr::ubx::Parser& gpsParser();

// ---- Backup kit: second GPS on UART2 (RX only; configured with GPS 1)
struct Gps2Status {
  bool started = false;
  bool seen = false;     // valid NAV-PVT frames have arrived since boot
  uint32_t fixes = 0;
};
extern Gps2Status g_gps2;
void gps2Begin();
bool gps2Poll(gr::GpsFix& fix);

// ---- Motor-driver watchdog (docs/WATCHDOG.md)
void watchdogBegin();
// Self-test only: stop petting even while the motors are enabled.
void watchdogInhibit(bool on);

// ---- Winches
void motorsBegin();
// duty in -1..1; positive pulls the brake line in. enabled=false lets both
// motors coast (pad, disarmed); duty 0 while enabled brakes the motor.
// While enabled, each call also toggles the watchdog pin, so the driver only
// stays awake while this keeps being called.
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
