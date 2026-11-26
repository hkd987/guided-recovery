// Tunable parameters. Defaults are for the S pod (rockets up to 1.5 kg) and
// are v0 values: drop tests (gate 6) are expected to replace the canopy numbers.
#pragma once

#include <cstdint>

namespace gr {

// What the firmware believes about the canopy. The wind estimator refines
// airspeed in flight; the rest are starting assumptions.
struct CanopyParams {
  float airspeed = 8.0f;          // m/s, trim horizontal airspeed
  float sinkRate = 3.5f;          // m/s, trim sink rate
  float turnRatePerBrake = 0.9f;  // rad/s per unit of differential brake
  float brakeSpeedLoss = 0.30f;   // airspeed fraction lost at full symmetric brake
  float brakeSinkGain = 0.30f;    // sink fraction gained at full symmetric brake (scales with brake^2)
};

struct DetectorParams {
  // Launch: sustained acceleration, confirmed by the barometer.
  float launchAccelG = 3.0f;
  float launchAccelTime = 0.10f;   // s above threshold
  float launchConfirmAlt = 10.0f;  // m AGL the baro must see...
  float launchConfirmWindow = 3.0f;  // ...within this many seconds
  float baroLaunchAlt = 30.0f;     // baro-only launch path (accel missing or saturated)
  float baroLaunchClimb = 10.0f;
  float burnoutAccelG = 1.5f;
  float burnoutTime = 0.2f;
  // Apogee.
  float apogeeClimb = -1.0f;   // m/s
  float apogeeTime = 0.5f;
  float apogeeDrop = 8.0f;     // m below max altitude also counts
  // Canopy confirmation.
  float joltAccelG = 2.5f;     // opening shock or ejection jolt
  float joltWindow = 20.0f;    // s a jolt stays valid as evidence
  float bandLow = 1.5f;        // m/s descent: parafoil band
  float bandHigh = 9.0f;
  float steadySpread = 2.5f;   // m/s, max minus min descent rate within the streak
  float confirmTime = 3.0f;    // s of steady in-band descent after a jolt
  float confirmTimeNoJolt = 8.0f;  // s of steady in-band descent with no jolt seen
  float minGuidedAlt = 25.0f;  // m AGL; confirmed lower than this means brakes stay released
  // Landing.
  float landedClimb = 0.6f;    // |m/s|
  float landedTime = 4.0f;
  float landedMaxAlt = 40.0f;  // m AGL
  // Home point capture on the pad.
  float homeMaxHAcc = 5.0f;
  uint8_t homeMinSv = 6;
  uint16_t homeMinFixes = 50;  // 5 s at 10 Hz
  float homeMaxSpeed = 0.6f;
};

struct GuidanceParams {
  float loiterRadius = 35.0f;      // m, energy-burning orbit around home
  float loiterEnterMargin = 25.0f; // m beyond the orbit radius where loiter can start
  float loiterExcessHeight = 10.0f;  // m above the straight-in need before choosing to loiter
  float orbitGain = 2.0f;
  float courseGain = 1.2f;         // rad/s of turn per rad of course error
  float maxTurnRateCmd = 0.7f;     // rad/s
  float turnRateForMargin = 0.45f; // rad/s assumed when budgeting height for the turn to final
  float finalBrakeNominal = 0.25f; // symmetric brake that leaves authority both ways
  float finalBrakeMax = 0.8f;
  float finalReloiterExcess = 25.0f;  // m too high on final -> go back to orbiting
  float weaveExcess = 6.0f;        // m too high at max brake -> S-turns
  float weaveAngle = 0.8f;         // rad
  float weavePeriod = 4.0f;        // s per leg
  float holdCourseRadius = 8.0f;   // m: inside this, stop chasing the point
  float flareAlt = 3.0f;           // m AGL
  float flareBrake = 1.0f;
  float minGroundSpeedForCourse = 1.0f;
  float strongWindRatio = 0.75f;   // wind/airspeed above which orbiting is pointless
};

struct SteeringParams {
  float kp = 0.6f;          // brake per rad/s of turn-rate error
  float ki = 0.3f;          // brake per rad of accumulated error
  float trimLimit = 0.3f;   // max integrator (trims a canopy that pulls to one side)
  float yawRateTau = 0.25f; // s low-pass on measured turn rate
  // Lopsided canopy: the brake asymmetry needed just to fly straight,
  // estimated as (applied differential brake - turn rate / turn gain).
  float lopsidedTrim = 0.38f;
  float lopsidedTime = 3.0f;    // s above the limit before declaring it
  float lopsidedSettle = 6.0f;  // s after canopy confirmation before judging
  float lopsidedMaxSd = 0.12f;  // only judge once the trim estimate is this certain
};

struct WinchParams {
  float travelMm = 120.0f;       // full brake line travel
  float countsPerMm = 50.13f;    // 12 CPR x 210:1 / (pi x 16 mm spool), Pololu 6V HPCB 210:1
  int8_t direction[2] = {1, 1};  // flip if a winch is wound the other way
  float kp = 0.15f;              // duty per mm
  float kd = 0.004f;             // duty per mm/s
  float deadbandMm = 0.5f;
  float motorMaxVolts = 6.0f;    // cap on average motor voltage (6 V motors on a 2S pack)
  float stallDuty = 0.5f;
  float stallSpeedMmS = 2.0f;
  float stallTime = 0.4f;
  float stallMinErrMm = 3.0f;
  // Exercise a winch that has sat still, so a jam is found before it matters.
  float exerciseIdle = 4.0f;    // s without a target change
  float exerciseMm = 4.5f;
  float exerciseTime = 0.6f;    // s (longer than stallTime)
};

struct SafetyParams {
  float gpsTimeout = 2.0f;      // s without a usable fix ends guidance
  float fenceRadius = 1000.0f;  // m from home ends guidance
  float collapseDescent = 12.0f;  // m/s
  float collapseTime = 2.0f;
  float lowBatteryV = 6.4f;     // 2S at 3.2 V/cell
  float lowBatteryTime = 3.0f;
  float spiralBrake = 0.35f;
};

struct Params {
  CanopyParams canopy;
  DetectorParams det;
  GuidanceParams guide;
  SteeringParams steer;
  WinchParams winch;
  SafetyParams safety;
  float gpsLatency = 0.10f;  // s, receiver output latency compensated by dead reckoning
  float logRateHz = 50.0f;
  float telemetryRateHz = 2.0f;
};

}  // namespace gr
