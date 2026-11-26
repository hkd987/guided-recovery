// Shared state between the 200 Hz flight task and the console / display.
#pragma once

#include <Arduino.h>

#include "gr/flight_core.hpp"

struct Snapshot {
  gr::Status status;
  gr::Vec3 accelBody, gyroBody;
  float pressurePa = 0.0f;
  float tempC = 0.0f;
  float batteryV = 0.0f;
  int32_t encoder[2] = {0, 0};
  bool armed = false;
  bool mainFired = false;
  uint32_t loops = 0;
  uint32_t overruns = 0;    // cycles that took longer than 5 ms
  uint32_t maxLoopUs = 0;
};

void appStart();
void appSnapshot(Snapshot& out);
// Bench requests, honoured only while disarmed.
void appBenchSet(int channel, float frac);
void appBenchRelease();
void appZeroWinches();
void appReconfigure();  // rebuild flight parameters from the saved settings
bool appEnterHil();     // gate-4 bench mode (see hilmode.h)
