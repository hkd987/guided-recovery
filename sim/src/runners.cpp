#include "sim/runners.hpp"

#include <cmath>
#include <cstdio>

#include "gr/flight_core.hpp"
#include "gr/guidance.hpp"
#include "sim/sensors.hpp"
#include "sim/world.hpp"

namespace sim {

namespace {

constexpr double kDt = 0.005;

}  // namespace

std::string RunResult::faultLabel() const {
  std::string n = faultName(fault);
  if (baro2) n += "+kit";
  if (gps2) n += "+gps2";
  return n;
}

namespace {

RunResult baseResult(const Scenario& s) {
  RunResult r;
  r.seed = s.seed;
  r.fault = s.fault;
  r.dualDeploy = s.dualDeploy;
  r.windRef = s.windRef;
  r.trueTrim = -s.turnBias / s.Kturn;
  r.baro2 = s.baro2;
  r.gps2 = s.gps2;
  return r;
}

// How long each fault may take to end in the fallback.
double fallbackLimit(FaultType f) {
  switch (f) {
    case FaultType::GpsLoss: return 5.0;
    case FaultType::WinchStall: return 15.0;
    case FaultType::Lopsided: return 25.0;
    case FaultType::BaroStuck: return 10.0;
    case FaultType::BaroDrift: return 12.0;
    default: return 0.0;
  }
}

void finalize(RunResult& r, const World& w, double maxDescent) {
  r.apogee = w.apogeeH();
  r.deployH = w.deployH();
  const V3 dp = w.deployPos();
  r.deployDist = std::hypot(dp.x, dp.y);
  r.maxCanopyDescent = maxDescent;
  if (w.stage() == TruthStage::Landed) {
    r.landed = true;
    const V3 lp = w.landPos();
    r.landN = lp.x;
    r.landE = lp.y;
    r.miss = std::hypot(lp.x, lp.y);
    r.flightTime = w.landT() - w.launchT();
  }
}

TraceRow baseRow(const World& w) {
  TraceRow t{};
  t.t = w.t();
  t.truthStage = static_cast<int>(w.stage());
  t.n = w.pos().x;
  t.e = w.pos().y;
  t.h = w.height();
  t.vn = w.vel().x;
  t.ve = w.vel().y;
  t.vd = w.vel().z;
  t.heading = w.heading();
  t.turnRate = w.turnRate();
  t.brakeL = w.brakeFrac(0);
  t.brakeR = w.brakeFrac(1);
  const V3 wind = w.windHere();
  t.windN = wind.x;
  t.windE = wind.y;
  t.accelG = w.specificForceBody().norm() / kG;
  return t;
}

}  // namespace

RunResult runSimple(const Scenario& s, const RunOptions& opt, std::vector<TraceRow>* trace) {
  RunResult r = baseResult(s);
  World w(s);
  const double duty[2] = {0.0, 0.0};

  // Perfect knowledge of the canopy.
  gr::CanopyParams cp = opt.params.canopy;
  cp.airspeed = static_cast<float>(s.Va0);
  cp.sinkRate = static_cast<float>(s.vs0);
  cp.turnRatePerBrake = static_cast<float>(s.Kturn);
  cp.brakeSpeedLoss = static_cast<float>(s.brakeSpeedLoss);
  cp.brakeSinkGain = static_cast<float>(s.brakeSinkGain);
  gr::Guidance guide(opt.params.guide, cp);
  guide.reset();
  gr::Steering steer(opt.params.steer, cp);
  steer.reset();

  double nextTrace = 0.0, maxDescent = 0.0;
  gr::GuidanceCmd cmd;
  while (w.t() < opt.maxTime) {
    if (w.stage() == TruthStage::Canopy) {
      gr::NavState n;
      n.posValid = true;
      n.pos = gr::Vec2{static_cast<float>(w.pos().x), static_cast<float>(w.pos().y)};
      n.velGround = gr::Vec2{static_cast<float>(w.vel().x), static_cast<float>(w.vel().y)};
      n.groundSpeed = n.velGround.norm();
      n.courseValid = n.groundSpeed > 0.5f;
      n.course = gr::bearingOf(n.velGround);
      n.agl = static_cast<float>(w.height());
      n.sinkRate = static_cast<float>(s.vs0);
      n.windValid = true;
      const V3 wind = w.steadyWindHere();
      n.wind = gr::Vec2{static_cast<float>(wind.x), static_cast<float>(wind.y)};
      n.airspeed = static_cast<float>(s.Va0);
      cmd = guide.update(n, static_cast<float>(kDt));
      const float diff = static_cast<float>(w.brakeFrac(1) - w.brakeFrac(0));
      const gr::BrakeCmd b = steer.update(cmd.turnRate, cmd.symBrake, static_cast<float>(w.turnRate()), diff,
                                          static_cast<float>(kDt));
      w.overrideBrakes(b.left, b.right);
      if (!r.canopyConfirmed) { r.canopyConfirmed = true; r.confirmDelay = 0.0; }
      maxDescent = std::max(maxDescent, w.vel().z);
    }
    w.step(kDt, duty, 8.0);
    if (trace && w.t() >= nextTrace) {
      nextTrace += 1.0 / opt.traceRate;
      TraceRow row = baseRow(w);
      row.phase = w.stage() == TruthStage::Canopy ? static_cast<int>(gr::Phase::Guided) : 0;
      row.mode = static_cast<int>(cmd.mode);
      row.estN = row.n;
      row.estE = row.e;
      row.estH = row.h;
      row.turnCmd = cmd.turnRate;
      row.symCmd = cmd.symBrake;
      trace->push_back(row);
    }
    if (w.stage() == TruthStage::Landed) break;
  }
  r.timedOut = w.stage() != TruthStage::Landed;
  r.finalPhase = static_cast<int>(gr::Phase::Landed);
  finalize(r, w, maxDescent);
  return r;
}

RunResult runSitl(const Scenario& s, const RunOptions& opt, std::vector<TraceRow>* trace) {
  RunResult r = baseResult(s);
  World w(s);
  SensorSuite sensors(s);
  gr::FlightCore core(opt.params);
  double duty[2] = {0.0, 0.0};
  gr::Inputs in;
  double nextTrace = 0.0, maxDescent = 0.0;
  double fallbackT = -1.0;
  try {
    while (w.t() < opt.maxTime) {
      w.step(kDt, duty, sensors.batteryV());
      sensors.sample(w, duty, in);
      const gr::Outputs& out = core.update(in);
      for (int ch = 0; ch < 2; ++ch) {
        duty[ch] = out.motorDuty[ch];
        if (!std::isfinite(duty[ch])) { r.crashed = true; duty[ch] = 0.0; }
      }
      const gr::Status& st = core.status();

      // Invariant: nothing steers before the canopy is really open.
      const bool open = w.openT() >= 0.0;
      if (!open && (st.steeringActive || st.phase == gr::Phase::Guided || st.phase == gr::Phase::Spiral ||
                    w.winch(0).pos > 3.0 || w.winch(1).pos > 3.0))
        r.navBeforeCanopy = true;
      if (r.launchDelay < 0.0 && w.launchT() >= 0.0 && st.stage != gr::FlightStage::Pad &&
          st.stage != gr::FlightStage::Disarmed)
        r.launchDelay = w.t() - w.launchT();
      if (!r.canopyConfirmed && st.stage == gr::FlightStage::Canopy) {
        r.canopyConfirmed = true;
        r.confirmDelay = open ? w.t() - w.openT() : -(w.deployT() >= 0 ? w.t() - w.deployT() : 0.0);
      }
      if (w.stage() == TruthStage::Canopy) maxDescent = std::max(maxDescent, w.vel().z);
      if (fallbackT < 0.0 && (st.phase == gr::Phase::Spiral || st.phase == gr::Phase::Neutral)) fallbackT = w.t();

      if (trace && w.t() >= nextTrace) {
        nextTrace += 1.0 / opt.traceRate;
        TraceRow row = baseRow(w);
        row.phase = static_cast<int>(st.phase);
        row.mode = static_cast<int>(st.mode);
        row.estN = st.nav.pos.x;
        row.estE = st.nav.pos.y;
        row.estH = st.agl;
        row.estWindN = st.nav.wind.x;
        row.estWindE = st.nav.wind.y;
        row.estWindValid = st.nav.windValid ? 1 : 0;
        row.turnCmd = st.guide.turnRate;
        row.symCmd = st.guide.symBrake;
        row.estYawRate = st.yawRateFiltered;
        row.trimNeeded = st.trimNeeded;
        trace->push_back(row);
      }
      if (w.stage() == TruthStage::Landed) break;
    }
  } catch (...) {
    r.crashed = true;
    r.note = "exception";
  }
  const gr::Status& st = core.status();
  r.timedOut = w.stage() != TruthStage::Landed;
  r.finalPhase = static_cast<int>(st.phase);
  r.faults = st.faults;
  finalize(r, w, maxDescent);

  // Did the injected fault end where it should?
  if (s.fault != FaultType::None) {
    const double end = r.landed ? w.landT() : w.t();
    if (expectedOutcome(s.fault, s.baro2, s.gps2) == Expectation::Spiral) {
      const double limit = fallbackLimit(s.fault);
      r.faultExercised = w.openT() >= 0.0 && w.faultT() + limit < end;
      if (fallbackT >= 0.0) r.fallbackDelay = fallbackT - w.faultT();
      if (r.faultExercised) {
        r.expectationMet = fallbackT >= 0.0 && r.fallbackDelay <= limit;
        if (fallbackT >= 0.0 && r.fallbackDelay < 0.0) r.note = "fallback before fault";
      }
    } else {
      r.faultExercised = s.fault == FaultType::BaroDriftDrogue ? w.apogeeT() >= 0.0 && w.apogeeT() + 2.0 < end
                                                               : w.openT() >= 0.0 && w.faultT() < end;
      r.expectationMet = r.landed && !r.crashed;
      // With the backup sensor fitted, its failure must not end guidance.
      if (isBaroFault(s.fault) && s.baro2) r.expectationMet = r.expectationMet && !(r.faults & gr::kFaultBaro);
      if (s.fault == FaultType::GpsLoss && s.gps2) r.expectationMet = r.expectationMet && !(r.faults & gr::kFaultGpsLost);
    }
  }
  return r;
}

void writeTraceCsv(const std::string& path, const std::vector<TraceRow>& rows) {
  FILE* f = std::fopen(path.c_str(), "w");
  if (!f) return;
  std::fprintf(f,
               "t,truth_stage,phase,mode,n,e,h,vn,ve,vd,heading,turn_rate,brake_l,brake_r,est_n,est_e,est_h,"
               "wind_n,wind_e,est_wind_n,est_wind_e,est_wind_valid,turn_cmd,sym_cmd,accel_g,est_yaw_rate,trim_needed\n");
  for (const auto& r : rows) {
    std::fprintf(f,
                 "%.3f,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,"
                 "%.3f,%.3f,%.2f,%.3f,%.3f\n",
                 r.t, r.truthStage, r.phase, r.mode, r.n, r.e, r.h, r.vn, r.ve, r.vd, r.heading, r.turnRate, r.brakeL,
                 r.brakeR, r.estN, r.estE, r.estH, r.windN, r.windE, r.estWindN, r.estWindE, r.estWindValid, r.turnCmd,
                 r.symCmd, r.accelG, r.estYawRate, r.trimNeeded);
  }
  std::fclose(f);
}

}  // namespace sim
