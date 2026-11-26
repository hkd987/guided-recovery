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
    case FaultType::BaroStuck: return "baro_stuck";
    case FaultType::BaroDrift: return "baro_drift";
    case FaultType::BaroDriftDrogue: return "baro_drift_drogue";
  }
  return "?";
}

FaultType faultFromName(const std::string& s) {
  if (s == "gps_loss") return FaultType::GpsLoss;
  if (s == "winch_stall") return FaultType::WinchStall;
  if (s == "lopsided") return FaultType::Lopsided;
  if (s == "gust") return FaultType::Gust;
  if (s == "shear") return FaultType::Shear;
  if (s == "baro_stuck") return FaultType::BaroStuck;
  if (s == "baro_drift") return FaultType::BaroDrift;
  if (s == "baro_drift_drogue") return FaultType::BaroDriftDrogue;
  return FaultType::None;
}

bool isBaroFault(FaultType f) {
  return f == FaultType::BaroStuck || f == FaultType::BaroDrift || f == FaultType::BaroDriftDrogue;
}

Expectation expectedOutcome(FaultType f, bool baro2, bool gps2) {
  switch (f) {
    case FaultType::GpsLoss:
      return gps2 ? Expectation::Land : Expectation::Spiral;  // receiver 2 takes over
    case FaultType::WinchStall:
    case FaultType::Lopsided:
      return Expectation::Spiral;
    case FaultType::BaroStuck:
    case FaultType::BaroDrift:
      return baro2 ? Expectation::Land : Expectation::Spiral;  // barometer 2 takes over
    default:
      return Expectation::Land;
  }
}

namespace {

// Extra spread for the stress dials, from a separate random stream.
void applyStress(Scenario& s, const StressLevels& st, uint64_t seed) {
  Rng r(seed * 0xD1B54A32D192ED03ULL + 0x5157ULL);
  if (st.canopy > 0) {
    // Total 1-sigma spread on airspeed and sink of ~20 % or ~30 % (the
    // nominal draw already carries 6 %), turn gain 30 % / 45 %, and wider
    // ranges for turn lag, rigging bias and brake effect.
    const double sp = st.canopy == 1 ? 0.20 : 0.30;
    const double extra = std::sqrt(sp * sp - 0.06 * 0.06);
    s.Va0 *= clampd(1.0 + r.normal(0.0, extra), 0.55, 1.6);
    s.vs0 *= clampd(1.0 + r.normal(0.0, extra), 0.55, 1.8);
    const double kt = st.canopy == 1 ? 0.30 : 0.45;
    s.Kturn = std::max(0.25, s.Kturn * (1.0 + r.normal(0.0, std::sqrt(kt * kt - 0.15 * 0.15))));
    s.turnTau = r.uniform(0.4, st.canopy == 1 ? 1.3 : 1.6);
    s.turnBias = r.normal(0.0, st.canopy == 1 ? 0.08 : 0.12);
    s.brakeTravelMm *= 1.0 + r.normal(0.0, st.canopy == 1 ? 0.12 : 0.18);
    s.brakeSpeedLoss = st.canopy == 1 ? r.uniform(0.15, 0.45) : r.uniform(0.10, 0.50);
    s.brakeSinkGain = st.canopy == 1 ? r.uniform(0.15, 0.60) : r.uniform(0.10, 0.75);
  }
  if (st.turbulence > 0) {
    s.gustScale = st.turbulence == 1 ? 2.0 : 3.0;
    s.gustSigma *= s.gustScale;
    s.gustTau = st.turbulence == 1 ? r.uniform(1.0, 3.0) : r.uniform(0.8, 2.5);
    s.vertGustSigma = st.turbulence == 1 ? 0.5 : 1.0;
    s.vertGustTau = r.uniform(3.0, 8.0);
    if (st.turbulence >= 2) {
      s.thermalAmp = r.uniform(1.0, 2.5);
      s.thermalPeriod = r.uniform(20.0, 60.0);
    }
  }
  if (st.swing > 0) {
    s.swingResidual = st.swing == 1 ? 0.12 : 0.25;
    s.swingAmp = std::max(s.swingAmp, st.swing == 1 ? 0.5 : 0.8);
    s.yawWobbleAmp = st.swing == 1 ? 0.15 : 0.35;
    s.yawWobblePeriod = r.uniform(2.0, 4.0);
  }
  if (st.gps > 0) {
    const bool poor = st.gps >= 2;
    s.gpsPosNoise = poor ? 2.0 : 1.0;
    s.gpsVelNoise = poor ? 0.5 : 0.25;
    s.gpsBiasScale = poor ? 3.0 : 2.0;
    s.gpsSvMin = poor ? 5 : 6;
    s.gpsSvRange = poor ? 3 : 4;
    s.gpsHAccBase = poor ? 4.0 : 2.5;
    s.gpsLatencyExtra = poor ? 0.20 : 0.10;
    s.gpsDropRate = poor ? 1.0 / 15.0 : 1.0 / 30.0;
    s.gpsDropMean = poor ? 1.5 : 1.0;
    s.gpsRateDiv = poor ? 40 : 20;  // 5 Hz when poor
  }
}

}  // namespace

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

  if (o.stress.any()) applyStress(s, o.stress, seed);

  // Backup kit and barometer faults draw from their own stream too.
  {
    Rng rk(seed * 0x94D049BB133111EBULL + 0xB4C0ULL);
    s.baro2 = o.baro2;
    s.bayTau = o.bayTau;
    s.gps2 = o.gps2;
    s.baro2Bias = rk.normal(0.0, 50.0);
    if (o.fault == FaultType::BaroDrift) s.faultMagnitude = rk.uniform(3.0, 6.0) * (rk.chance(0.5) ? 1.0 : -1.0);
    if (o.fault == FaultType::BaroDriftDrogue) {
      s.dualDeploy = true;                    // the dangerous case: drogue descent made to look like a canopy
      s.faultMagnitude = rk.uniform(8.0, 14.0);  // m/s of false climb
    }
  }

  // Fault.
  s.fault = o.fault;
  s.faultAfterOpen = r.uniform(5.0, 30.0);
  s.faultSide = r.pick(2);
  switch (s.fault) {
    case FaultType::Lopsided:
      // Needs 50-80 % brake on one side just to fly straight, counting the
      // canopy's own rigging bias (which can add to or cancel the fault).
      s.faultMagnitude = r.uniform(0.5, 0.8) * s.Kturn * (s.faultSide ? 1.0 : -1.0) - s.turnBias;
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
