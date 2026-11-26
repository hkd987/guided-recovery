// Data that crosses the hardware boundary. A target (the ESP32 board, the
// simulator, or the log replayer) fills an Inputs frame from its sensors,
// calls FlightCore::update() at 200 Hz, and applies the Outputs.
#pragma once

#include <cstdint>

#include "gr/math.hpp"

namespace gr {

struct ImuSample {
  Vec3 accel;   // specific force, body frame, m/s^2 (reads about -9.8 on z when level and still)
  Vec3 gyro;    // body rates, rad/s
  bool valid = false;
};

struct GpsFix {
  bool fresh = false;     // true only on the update where a new fix arrived
  uint8_t fixType = 0;    // 0 none, 2 = 2D, 3 = 3D
  uint8_t numSv = 0;
  int32_t latE7 = 0;      // degrees * 1e7 (UBX NAV-PVT units)
  int32_t lonE7 = 0;
  float altMsl = 0.0f;    // m
  float velN = 0.0f;      // m/s, NED
  float velE = 0.0f;
  float velD = 0.0f;
  float hAcc = 99.0f;     // m, receiver's own estimate
  float vAcc = 99.0f;
  float sAcc = 99.0f;     // m/s
};

struct Inputs {
  uint64_t tUs = 0;           // monotonic microseconds
  ImuSample imu;
  bool highGValid = false;    // optional ADXL375, logged only
  Vec3 highG;
  bool baroFresh = false;     // a new pressure sample this frame
  float pressurePa = 101325.0f;
  float baroTempC = 20.0f;
  bool magFresh = false;      // logged only; guidance does not need heading
  Vec3 magUt;
  GpsFix gps;
  int32_t encoder[2] = {0, 0};  // winch encoder counts, 0 = left, 1 = right
  bool armed = false;           // magnetic switch: true once the remove-before-flight magnet is pulled
  bool mainFiredInput = false;  // optional opto-isolated "main fired" line from the host computer
  float batteryV = 8.0f;
};

struct Outputs {
  float motorDuty[2] = {0.0f, 0.0f};  // -1..1, positive pulls the brake line in
  bool motorsEnabled = false;         // false: let the motors coast (unpowered)
  bool buzzer = false;
  bool led = false;
};

// Top-level flight phase as the pod sees it.
enum class Phase : uint8_t {
  Disarmed = 0,  // magnet in: bench mode, motors only move on bench commands
  Pad,           // armed, waiting for launch, learning home and ground pressure
  Ascent,        // launch detected, home locked
  Descent,       // apogee passed, waiting for the canopy to be confirmed open
  Guided,        // canopy confirmed, steering home
  Spiral,        // fallback: slow steady turn, brakes held
  Neutral,       // brakes released (canopy collapse, deployed too low)
  Landed,
};

enum class GuideMode : uint8_t { None = 0, Homing, Loiter, Final, Flare };

// Fault bits reported in status and logs.
enum FaultBits : uint16_t {
  kFaultNone = 0,
  kFaultGpsLost = 1 << 0,
  kFaultFence = 1 << 1,
  kFaultWinchLeft = 1 << 2,
  kFaultWinchRight = 1 << 3,
  kFaultLopsided = 1 << 4,
  kFaultCollapse = 1 << 5,
  kFaultLowBattery = 1 << 6,
  kFaultNoHome = 1 << 7,
  kFaultLowDeploy = 1 << 8,
};

const char* phaseName(Phase p);
const char* guideModeName(GuideMode m);

}  // namespace gr
