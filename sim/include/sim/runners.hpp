// The two ways of flying a scenario:
//   runSimple - gate 1: guidance and steering laws flown on truth (perfect
//               knowledge, ideal actuators). Measures the algorithm's ceiling.
//   runSitl   - gate 2: the real flight core, fed by sensor models, driving
//               simulated winch motors, from pad to landing.
#pragma once

#include <string>
#include <vector>

#include "gr/params.hpp"
#include "gr/types.hpp"
#include "sim/scenario.hpp"

namespace sim {

struct TraceRow {
  double t;
  int truthStage;
  int phase;
  int mode;
  double n, e, h;          // truth position
  double vn, ve, vd;
  double heading, turnRate;
  double brakeL, brakeR;   // truth brake fractions
  double estN, estE, estH; // firmware estimates
  double windN, windE;     // truth wind here
  double estWindN, estWindE;
  int estWindValid;
  double turnCmd, symCmd;
  double accelG;
  double estYawRate, trimNeeded;
};

struct RunResult {
  uint64_t seed = 0;
  FaultType fault = FaultType::None;
  bool dualDeploy = false;
  double windRef = 0.0;
  double apogee = 0.0;
  double deployH = 0.0;
  double deployDist = 0.0;
  bool landed = false;
  double landN = 0.0, landE = 0.0;
  double miss = 1e9;           // m from the pad
  double flightTime = 0.0;     // launch to landing
  bool canopyConfirmed = false;
  double confirmDelay = -1.0;  // s after the canopy actually opened
  double launchDelay = -1.0;   // s from the motor lighting to the pod's "ascent" (-1: never)
  bool navBeforeCanopy = false;
  bool crashed = false;        // non-finite output or exception
  bool timedOut = false;
  int finalPhase = 0;
  unsigned faults = 0;
  bool faultExercised = false; // fault happened before landing
  double fallbackDelay = -1.0; // s from fault to spiral/neutral
  bool expectationMet = true;
  double maxCanopyDescent = 0.0;
  double trueTrim = 0.0;       // brake asymmetry the canopy's rigging alone needs to fly straight (truth)
  bool baro2 = false, gps2 = false;  // backup kit fitted
  std::string faultLabel() const;    // e.g. "baro_stuck+kit"
  std::string note;
};

struct RunOptions {
  gr::Params params;
  double maxTime = 1200.0;
  double traceRate = 10.0;
};

RunResult runSimple(const Scenario& s, const RunOptions& opt, std::vector<TraceRow>* trace = nullptr);
RunResult runSitl(const Scenario& s, const RunOptions& opt, std::vector<TraceRow>* trace = nullptr);

void writeTraceCsv(const std::string& path, const std::vector<TraceRow>& rows);

}  // namespace sim
