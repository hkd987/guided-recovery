// One randomized flight: site, wind, rocket, recovery setup, canopy, sensor
// errors and (optionally) one injected fault. Fully determined by its seed.
#pragma once

#include <cstdint>
#include <string>

#include "sim/vec.hpp"

namespace sim {

enum class FaultType { None = 0, GpsLoss, WinchStall, Lopsided, Gust, Shear };
const char* faultName(FaultType f);
FaultType faultFromName(const std::string& s);

// What a fault is expected to end in.
enum class Expectation { Land, Spiral };
Expectation expectedOutcome(FaultType f);

struct ScenarioOptions {
  double windMax = 4.47;          // m/s at 10 m (10 mph)
  double canopyAirspeed = 8.0;    // nominal truth canopy (firmware params may differ)
  double canopySink = 3.5;
  double canopySpread = 0.06;     // 1-sigma fractional variation between canopies
  FaultType fault = FaultType::None;
  bool dualDeployOnly = false;
  bool singleDeployOnly = false;
};

struct Scenario {
  uint64_t seed = 0;
  // Site
  double padLat = 40.0, padLon = -105.0, padElev = 1500.0;
  double armTime = 3.0;       // s after boot the magnet is pulled
  double launchTime = 20.0;   // s after boot the motor lights
  // Wind
  double windRef = 0.0;       // m/s at 10 m
  double windFrom = 0.0;      // rad, direction the wind comes from (met convention)
  double shearExp = 0.14;
  double veer = 0.0;          // rad of direction change from ground to 300 m
  double gustSigma = 0.3;     // m/s
  double gustTau = 3.0;       // s
  // Rocket
  double mass = 1.2;          // kg, liftoff including pod
  double diameter = 0.054;
  double cd = 0.5;
  double thrust = 80.0;       // N average
  double burnTime = 1.4;      // s
  double propFrac = 0.06;
  double railLength = 1.5;
  double railTilt = 0.07;     // rad from vertical
  double railAz = 0.0;        // rad
  // Recovery
  bool dualDeploy = false;
  double drogueRate = 20.0;   // m/s
  bool mainFailure = false;   // main never comes out (rides the drogue down)
  double mainAlt = 150.0;     // m AGL
  double ejectErr = 0.0;      // s relative to apogee (single deploy)
  double ejectShockG = 30.0;
  double openShockG = 6.0;
  double ejectPressurePa = 300.0;
  // Canopy truth
  double Va0 = 8.0, vs0 = 3.5;
  double Kturn = 0.9;         // rad/s per unit differential brake
  double turnTau = 0.6;
  double turnBias = 0.0;      // rad/s, rigging asymmetry
  double brakeTravelMm = 120.0;
  double brakeSpeedLoss = 0.30, brakeSinkGain = 0.30;
  double inflationTime = 1.5;
  double swingAmp = 0.4;      // rad after opening
  double lineForceFull = 3.7; // N per line at full brake
  double suspendedMass = 1.2;
  // Winch truth (Pololu 6V HPCB 210:1 class, 16 mm spool)
  double winchNoLoadMmS = 134.0;  // at 6 V
  double winchStallN = 34.0;      // at 6 V
  double winchCountsPerMm = 50.13;
  double winchBackdriveN = 2.5;   // line force above which an unpowered gearbox slips
  // Sensor errors
  double accelBias = 0.05, gyroBias = 0.005;  // per-axis sigma
  double gpsBiasSigma = 1.2;      // m, slowly wandering
  double gpsLatency = 0.10;
  bool gpsDropDuringBoost = false;
  double gpsReacquire = 4.0;      // s after burnout
  // Fault
  FaultType fault = FaultType::None;
  double faultAfterOpen = 20.0;   // s after the canopy inflates
  int faultSide = 0;              // 0 left, 1 right
  double faultMagnitude = 0.0;
};

Scenario makeScenario(uint64_t seed, const ScenarioOptions& opt);

}  // namespace sim
