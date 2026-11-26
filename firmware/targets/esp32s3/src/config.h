// Persistent settings (NVS). Everything a builder may need to change for
// their own pod: canopy numbers from drop tests, winch rigging, IMU mounting.
#pragma once

#include <Arduino.h>

#include "gr/params.hpp"

struct PodSettings {
  // Canopy (update from drop-test fits)
  float canopyAirspeed = 8.0f;
  float canopySink = 3.5f;
  float canopyTurnGain = 0.9f;
  // Winches
  float winchTravelMm = 120.0f;
  float winchCountsPerMm = 50.13f;
  int8_t winchDirL = 1;   // flip if the line pays OUT when it should pull in
  int8_t winchDirR = 1;
  int8_t encSignL = 1;    // flip if the encoder counts backwards vs. the motor
  int8_t encSignR = 1;
  // Safety
  float fenceRadius = 1000.0f;
  // IMU mounting: body axis i = sign[i] * sensor axis index[i]
  // Default: board flat, components up, pod hanging upright -> (+x, -y, -z).
  int8_t imuAxis[3] = {0, 1, 2};
  int8_t imuSign[3] = {1, -1, -1};
  // Radio
  float radioMHz = 915.0f;
  int8_t radioDbm = 14;
  // Battery divider calibration
  float batteryScale = 1.0f;
  // Dual deploy: the main altitude set on the host flight computer (m AGL).
  // 0 = single deploy (parafoil out at apogee). Appended in settings v3.
  float mainAltM = 0.0f;
  // "Main fired" sense line: 0 ignore, 1 counts as an opening jolt, 2 required.
  // Appended in settings v4.
  uint8_t mainSense = 1;
};

extern PodSettings g_settings;

void settingsLoad();
bool settingsSave();
void settingsReset();
// Build flight-core parameters from the stored settings.
gr::Params settingsToParams();

// Text interface for the CLI: list, get and set by name.
void settingsPrint(Print& out);
bool settingsSet(const String& name, const String& value, String& err);
