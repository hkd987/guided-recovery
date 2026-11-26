#include "sim/scenario.hpp"

#include <algorithm>

namespace sim {

const char* faultName(FaultType f) {
  switch (f) {
    case FaultType::None: return "none";
    case FaultType::GpsLoss: return "gps_loss";
    case FaultType::WinchStall: return "winch_stall";
    case FaultType::Lopsided: return "lopsided";
    case FaultType::Gust: return "gust";
    case FaultType::Shear: return "shear";
  }
  return "?";
}

FaultType faultFromName(const std::string& s) {
  if (s == "gps_loss") return FaultType::GpsLoss;
  if (s == "winch_stall") return FaultType::WinchStall;
  if (s == "lopsided") return FaultType::Lopsided;
  if (s == "gust") return FaultType::Gust;
  if (s == "shear") return FaultType::Shear;
  return FaultType::None;
}

Expectation expectedOutcome(FaultType f) {
  switch (f) {
    case FaultType::GpsLoss:
    case FaultType::WinchStall:
    case FaultType::Lopsided:
      return Expectation::Spiral;
    default:
      return Expectation::Land;
  }
}

Scenario makeScenario(uint64_t seed, const ScenarioOptions& o) {
  Rng r(seed * 0x9E3779B97F4A7C15ULL + 12345);
  Scenario s;
  s.seed = seed;
  s.padElev = r.uniform(0.0, 2000.0);
  s.launchTime = r.uniform(18.0, 30.0);

  // Wind: uniform speed up to the limit, any direction.
  s.windRef = r.uniform(0.0, o.windMax);
  s.windFrom = r.uniform(-kPi, kPi);
  s.shearExp = r.uniform(0.08, 0.22);
  s.veer = r.normal(0.0, 0.25);
  s.gustSigma = 0.05 + 0.15 * s.windRef * r.uniform(0.5, 1.5);
  s.gustTau = r.uniform(2.0, 6.0);

  // Rocket: Class 1 sized, up to 1.5 kg with the pod.
  s.mass = r.uniform(0.6, 1.5);
  s.diameter = r.uniform(0.041, 0.066);
  s.cd = r.uniform(0.45, 0.65);
  s.burnTime = r.uniform(0.8, 2.0);
  // Thrust-to-weight between 5 and 12.
  s.thrust = s.mass * kG * r.uniform(5.0, 12.0);
  s.railTilt = std::fabs(r.normal(0.0, 0.06)) + 0.02;
  // Flyers angle the rail slightly into the wind.
  s.railAz = wrapPi(s.windFrom + r.normal(0.0, 0.5));

  // Recovery: single deploy at apogee (motor or altimeter) or dual deploy.
  if (o.dualDeployOnly) s.dualDeploy = true;
  else if (o.singleDeployOnly) s.dualDeploy = false;
  else s.dualDeploy = r.chance(0.5);
  s.drogueRate = r.uniform(14.0, 28.0);
  s.mainAlt = r.uniform(120.0, 250.0);
  s.ejectErr = r.normal(0.0, 1.0);
  s.ejectShockG = r.uniform(15.0, 60.0);
  s.openShockG = r.uniform(3.0, 12.0);
  s.ejectPressurePa = r.uniform(0.0, 600.0);

  // Canopy: nominal plus manufacturing / packing spread.
  const double sp = o.canopySpread;
  s.Va0 = o.canopyAirspeed * (1.0 + r.normal(0.0, sp));
  s.vs0 = o.canopySink * (1.0 + r.normal(0.0, sp));
  s.Kturn = 0.9 * (1.0 + r.normal(0.0, 0.15));
  s.turnTau = r.uniform(0.4, 0.9);
  s.turnBias = r.normal(0.0, 0.04);
  s.brakeTravelMm = 120.0 * (1.0 + r.normal(0.0, 0.08));
  s.brakeSpeedLoss = r.uniform(0.22, 0.38);
  s.brakeSinkGain = r.uniform(0.2, 0.45);
  s.inflationTime = r.uniform(0.8, 2.5);
  s.swingAmp = r.uniform(0.2, 0.6);
  s.suspendedMass = s.mass * (1.0 - s.propFrac);
  s.lineForceFull = s.suspendedMass * kG * r.uniform(0.15, 0.40);

  // Winch: unit-to-unit spread.
  s.winchNoLoadMmS = 134.0 * (1.0 + r.normal(0.0, 0.08));
  s.winchStallN = 34.0 * (1.0 + r.normal(0.0, 0.1));

  // Sensors.
  s.accelBias = 0.05;
  s.gyroBias = 0.005;
  s.gpsBiasSigma = r.uniform(0.6, 1.8);
  s.gpsLatency = r.uniform(0.07, 0.15);
  s.gpsDropDuringBoost = r.chance(0.4);
  s.gpsReacquire = r.uniform(1.0, 8.0);

  // Fault.
  s.fault = o.fault;
  s.faultAfterOpen = r.uniform(5.0, 30.0);
  s.faultSide = r.pick(2);
  switch (s.fault) {
    case FaultType::Lopsided:
      // Needs 50-80 % brake on one side just to fly straight.
      s.faultMagnitude = r.uniform(0.5, 0.8) * s.Kturn * (s.faultSide ? 1.0 : -1.0);
      break;
    case FaultType::Gust:
      s.faultMagnitude = r.uniform(3.0, 6.0);  // m/s extra for a few seconds
      break;
    case FaultType::Shear:
      s.faultMagnitude = r.uniform(0.5, 1.0) * kPi;  // direction swing below 60 m
      break;
    default:
      break;
  }
  return s;
}

}  // namespace sim
