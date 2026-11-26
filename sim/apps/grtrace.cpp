// grtrace: records the real flight core's decision-state changes for TLA+
// trace validation (verification/tla/PodTrace.tla).
//
// Each recorded step is one FlightCore::update() call that changed the
// observable decision state (stage, phase, launch candidate, home point,
// bench mode, motor power, steering, faults). TLC then checks that every one
// of those steps is allowed by the verified model in PodLogic.tla.
//
// Scenarios:
//   * simulated flights (nominal and every fault type) that continue after
//     touchdown: line release, magnet back on, re-armed at a field 12 m higher;
//   * scripted bench sessions: winch jog, jam, console reset, arming, GPS home
//     learning, handling bumps, disarming with a launch pending, bench
//     commands refused while armed.
//
//   grtrace --out Traces.tla [--flights N] [--fault-flights K] [--seed S]

#include <algorithm>
#include <cmath>
#include <memory>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "gr/flight_core.hpp"
#include "sim/scenario.hpp"
#include "sim/sensors.hpp"
#include "sim/world.hpp"

using namespace sim;

namespace {

struct Obs {
  gr::FlightStage stage = gr::FlightStage::Disarmed;
  gr::Phase phase = gr::Phase::Disarmed;
  bool cand = false;
  int home = 0;
  bool bench = false;
  bool motors = false;
  bool steer = false;
  uint16_t faults = 0;
  bool operator==(const Obs& o) const {
    return stage == o.stage && phase == o.phase && cand == o.cand && home == o.home && bench == o.bench &&
           motors == o.motors && steer == o.steer && faults == o.faults;
  }
  bool operator!=(const Obs& o) const { return !(*this == o); }
};

struct Step {
  Obs o;
  bool armed;
  bool resume = false;  // a processor restart + FlightCore::resume(), not an update
};

struct Trace {
  std::string name;
  std::vector<Step> steps;
};

const char* stageName(gr::FlightStage s) {
  switch (s) {
    case gr::FlightStage::Disarmed: return "Disarmed";
    case gr::FlightStage::Pad: return "Pad";
    case gr::FlightStage::Ascent: return "Ascent";
    case gr::FlightStage::Descent: return "Descent";
    case gr::FlightStage::Canopy: return "Canopy";
    case gr::FlightStage::Landed: return "Landed";
  }
  return "?";
}

const char* phaseNameTla(gr::Phase p) {
  switch (p) {
    case gr::Phase::Disarmed: return "Disarmed";
    case gr::Phase::Pad: return "Pad";
    case gr::Phase::Ascent: return "Ascent";
    case gr::Phase::Descent: return "Descent";
    case gr::Phase::Guided: return "Guided";
    case gr::Phase::Spiral: return "Spiral";
    case gr::Phase::Neutral: return "Neutral";
    case gr::Phase::Landed: return "Landed";
  }
  return "?";
}

std::string faultSet(uint16_t f) {
  static const std::pair<uint16_t, const char*> names[] = {
      {gr::kFaultGpsLost, "GpsLost"},   {gr::kFaultFence, "Fence"},           {gr::kFaultWinchLeft, "WinchL"},
      {gr::kFaultWinchRight, "WinchR"}, {gr::kFaultLopsided, "Lopsided"},     {gr::kFaultCollapse, "Collapse"},
      {gr::kFaultLowBattery, "LowBattery"}, {gr::kFaultNoHome, "NoHome"},    {gr::kFaultLowDeploy, "LowDeploy"},
      {gr::kFaultReset, "Reset"},           {gr::kFaultBaro, "Baro"}};
  std::string s = "{";
  bool first = true;
  for (const auto& n : names) {
    if (f & n.first) {
      s += first ? "\"" : ", \"";
      s += n.second;
      s += "\"";
      first = false;
    }
  }
  return s + "}";
}

// Records observation changes, one FlightCore at a time.
class Recorder {
 public:
  explicit Recorder(std::string name) { tr_.name = std::move(name); tr_.steps.push_back(Step{Obs{}, false}); }

  void after(const gr::FlightCore& core, const gr::Outputs& out, bool armed) {
    const gr::Status& st = core.status();
    Obs o;
    o.stage = st.stage;
    o.phase = st.phase;
    o.cand = st.candidate;
    if (!st.homeValid) {
      home_ = 0;
    } else if (home_ == 0 || st.homeLat != lastLat_ || st.homeLon != lastLon_) {
      home_ = home_ == 1 ? 2 : 1;  // a new home value: alternate the abstract token
    }
    lastLat_ = st.homeLat;
    lastLon_ = st.homeLon;
    o.home = home_;
    o.bench = core.benchActive();
    o.motors = out.motorsEnabled;
    o.steer = st.steeringActive;
    o.faults = st.faults;

    const Obs& last = tr_.steps.back().o;
    if (o == last) return;
    // Consecutive home-point refinements on the pad compose into one step.
    const bool homeOnly = o.home != last.home && o.home != 0 && last.home != 0 && sameExceptHome(o, last) &&
                          last.stage == gr::FlightStage::Pad && !last.cand;
    if (homeOnly && lastWasHomeOnly_ && tr_.steps.size() >= 2) {
      tr_.steps.back() = Step{o, armed};
      if (tr_.steps.back().o == tr_.steps[tr_.steps.size() - 2].o) {
        // The refinements cancelled out: drop the merged step. The new last
        // record is a real (non-merged) step, so never merge into it.
        tr_.steps.pop_back();
        lastWasHomeOnly_ = false;
        return;
      }
    } else {
      tr_.steps.push_back(Step{o, armed});
    }
    lastWasHomeOnly_ = homeOnly;
  }

  // Right after a restart and FlightCore::resume(): no update has run yet.
  void resumed(const gr::FlightCore& core) {
    const gr::Status& st = core.status();
    Obs o;
    o.stage = st.stage;
    o.phase = st.phase;
    o.cand = st.candidate;
    o.home = st.homeValid ? home_ : 0;
    o.bench = core.benchActive();
    o.motors = false;
    o.steer = false;
    o.faults = st.faults;
    tr_.steps.push_back(Step{o, false, true});
    lastWasHomeOnly_ = false;
  }

  Trace take() { return tr_; }

 private:
  static bool sameExceptHome(Obs a, const Obs& b) {
    a.home = b.home;
    return a == b;
  }
  Trace tr_;
  int home_ = 0;
  double lastLat_ = 0.0, lastLon_ = 0.0;
  bool lastWasHomeOnly_ = false;
};

// ---------------------------------------------------------------- flights

enum class ResetAt { None, Descent, Canopy };

Trace flightTrace(const Scenario& s, const std::string& name, ResetAt resetAt = ResetAt::None) {
  World w(s);
  SensorSuite sens(s);
  auto corePtr = std::make_unique<gr::FlightCore>();
  Recorder rec(name);
  bool didReset = false;
  double descentSeen = -1.0;
  double duty[2] = {0.0, 0.0};
  gr::Inputs in;
  double landedAt = -1.0;
  const double dt = 0.005;
  while (w.t() < 1500.0) {
    w.step(dt, duty, sens.batteryV());
    sens.sample(w, duty, in);
    // After touchdown: 25 s on the ground, magnet on for 2 s, re-armed for 4 s
    // at a field 12 m higher than the original pad (or after weather drift).
    if (w.stage() == TruthStage::Landed && landedAt < 0.0) landedAt = w.t();
    const double since = landedAt < 0.0 ? -1.0 : w.t() - landedAt;
    if (since >= 25.0 && since < 27.0) in.armed = false;
    if (since >= 27.0 && in.baroFresh) in.pressurePa -= 140.0f;
    // Optional processor restart: snapshot, new core, resume.
    if (descentSeen < 0.0 && corePtr->status().stage == gr::FlightStage::Descent) descentSeen = w.t();
    const bool fire = !didReset && ((resetAt == ResetAt::Descent && descentSeen >= 0.0 && w.t() > descentSeen + 1.0 &&
                                     corePtr->status().stage == gr::FlightStage::Descent) ||
                                    (resetAt == ResetAt::Canopy && w.openT() >= 0.0 && w.t() > w.openT() + 10.0 &&
                                     corePtr->status().stage == gr::FlightStage::Canopy));
    if (fire) {
      const gr::ResumeState snap = corePtr->resumeState(in.encoder);
      corePtr = std::make_unique<gr::FlightCore>();
      if (corePtr->resume(snap)) rec.resumed(*corePtr);
      didReset = true;
    }
    gr::FlightCore& core = *corePtr;
    const gr::Outputs& out = core.update(in);
    duty[0] = out.motorDuty[0];
    duty[1] = out.motorDuty[1];
    rec.after(core, out, in.armed);
    if (since >= 31.0) break;
  }
  return rec.take();
}

// ------------------------------------------------------------ bench sessions

struct Bench {
  gr::FlightCore core;
  Recorder rec;
  gr::Inputs in;
  double t = 0.0;
  double pos[2] = {0.0, 0.0};
  bool jam[2] = {false, false};
  int fixCounter = 0;
  double latJitter = 0.0;

  explicit Bench(const std::string& name) : rec(name) {
    in.imu.valid = true;
    in.imu.accel = gr::Vec3{0.0f, 0.0f, -9.80665f};
    in.batteryV = 8.1f;
  }

  // Run for `seconds` with the given switch position and optional bump.
  void run(double seconds, bool armed, double accelG = 1.0, bool gps = true) {
    const int n = static_cast<int>(seconds / 0.005);
    for (int i = 0; i < n; ++i) {
      t += 0.005;
      in.tUs = static_cast<uint64_t>(std::llround(t * 1e6));
      in.armed = armed;
      in.imu.accel = gr::Vec3{0.0f, 0.0f, static_cast<float>(-9.80665 * accelG)};
      in.baroFresh = (i % 4) == 0;
      in.pressurePa = 84000.0f + static_cast<float>(std::sin(t * 7.0) * 2.0);
      in.gps = gr::GpsFix{};
      if (gps && (i % 20) == 0) {
        in.gps.fresh = true;
        in.gps.fixType = 3;
        in.gps.numSv = 12;
        latJitter = std::sin(++fixCounter * 0.7) * 3e-6;
        in.gps.latE7 = static_cast<int32_t>((40.0 + latJitter) * 1e7);
        in.gps.lonE7 = static_cast<int32_t>(-105.0 * 1e7);
        in.gps.hAcc = 1.5f;
      }
      for (int ch = 0; ch < 2; ++ch) in.encoder[ch] = static_cast<int32_t>(std::lround(pos[ch] * 50.13));
      const gr::Outputs& out = core.update(in);
      for (int ch = 0; ch < 2; ++ch)
        if (out.motorsEnabled && !jam[ch]) pos[ch] += out.motorDuty[ch] * 130.0 * 0.005;
      rec.after(core, out, armed);
    }
  }
  // The ESP32 target's console requests, applied before an update.
  void benchSet(int ch, float frac) { core.benchSetBrake(ch, frac); }
  void consoleReset() {
    if (core.status().phase == gr::Phase::Disarmed) core.reset();
  }
};

Trace benchSession1() {
  Bench b("bench: jog, reset, arm, home, handling bumps, disarm with launch pending, re-arm");
  b.run(1.0, false);
  b.benchSet(0, 0.3f);
  b.run(1.5, false);
  b.benchSet(1, 0.2f);
  b.run(1.5, false);
  b.core.benchRelease();
  b.run(1.5, false);
  b.consoleReset();  // 'zero'
  b.run(0.5, false);
  b.run(8.0, true);             // armed: learns home
  b.benchSet(0, 0.5f);          // refused while armed
  b.run(1.0, true);
  b.run(0.2, true, 4.0);        // handling bump: launch candidate
  b.run(4.0, true);             // no climb: candidate times out
  b.run(0.2, true, 4.0);        // another bump...
  b.run(0.5, true);
  b.run(1.0, false);            // ...then the magnet goes back on
  b.run(6.0, true);             // re-armed: home learned again
  b.run(1.0, true, 1.0, false); // GPS silent on the pad
  b.run(1.0, false);
  return b.rec.take();
}

Trace benchSession2() {
  Bench b("bench: jammed winch latches a fault into the next arming, then 'zero' clears it");
  b.run(0.5, false);
  b.jam[1] = true;
  b.benchSet(1, 0.4f);  // jammed spool: stall fault
  b.run(2.0, false);
  b.run(3.0, true);     // armed with the fault latched
  b.run(1.0, false);
  b.consoleReset();
  b.jam[1] = false;
  b.run(1.0, false);
  b.benchSet(1, 0.2f);
  b.run(1.5, false);
  return b.rec.take();
}

void writeTla(const std::string& path, const std::vector<Trace>& traces) {
  FILE* f = std::fopen(path.c_str(), "w");
  if (!f) { std::perror("open"); std::exit(1); }
  std::fprintf(f, "---- MODULE Traces ----\n\\* Generated by grtrace from the real C++ flight core. Do not edit.\n");
  std::fprintf(f, "EXTENDS Sequences\n");
  std::fprintf(f, "TraceNames == <<");
  for (size_t k = 0; k < traces.size(); ++k) std::fprintf(f, "%s\"%s\"", k ? ", " : "", traces[k].name.c_str());
  std::fprintf(f, ">>\nTraces == <<\n");
  for (size_t k = 0; k < traces.size(); ++k) {
    std::fprintf(f, "  <<");
    for (size_t j = 0; j < traces[k].steps.size(); ++j) {
      const Step& st = traces[k].steps[j];
      const Obs& o = st.o;
      std::fprintf(f,
                   "%s[o |-> [stage |-> \"%s\", phase |-> \"%s\", cand |-> %s, home |-> %d, bench |-> %s, "
                   "motors |-> %s, steer |-> %s, faults |-> %s], armed |-> %s, resume |-> %s]",
                   j ? ",\n    " : "", stageName(o.stage), phaseNameTla(o.phase), o.cand ? "TRUE" : "FALSE", o.home,
                   o.bench ? "TRUE" : "FALSE", o.motors ? "TRUE" : "FALSE", o.steer ? "TRUE" : "FALSE",
                   faultSet(o.faults).c_str(), st.armed ? "TRUE" : "FALSE", st.resume ? "TRUE" : "FALSE");
    }
    std::fprintf(f, ">>%s\n", k + 1 < traces.size() ? "," : "");
  }
  std::fprintf(f, ">>\n====\n");
  std::fclose(f);
}

}  // namespace

int main(int argc, char** argv) {
  std::string out = "Traces.tla";
  int flights = 100, faultFlights = 20;
  uint64_t seed = 1;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!std::strcmp(argv[i], "--out")) out = argv[i + 1];
    else if (!std::strcmp(argv[i], "--flights")) flights = std::atoi(argv[i + 1]);
    else if (!std::strcmp(argv[i], "--fault-flights")) faultFlights = std::atoi(argv[i + 1]);
    else if (!std::strcmp(argv[i], "--seed")) seed = static_cast<uint64_t>(std::atoll(argv[i + 1]));
  }
  std::vector<Trace> traces;
  traces.push_back(benchSession1());
  traces.push_back(benchSession2());
  ScenarioOptions o;
  ScenarioOptions dual;
  dual.dualDeployOnly = true;
  for (int i = 0; i < std::max(2, flights / 8); ++i) {
    traces.push_back(flightTrace(makeScenario(seed + 5000 + i, dual), "restart in descent, seed " + std::to_string(seed + 5000 + i), ResetAt::Descent));
    traces.push_back(flightTrace(makeScenario(seed + 6000 + i, o), "restart under canopy, seed " + std::to_string(seed + 6000 + i), ResetAt::Canopy));
  }
  for (int i = 0; i < flights; ++i)
    traces.push_back(flightTrace(makeScenario(seed + i, o), "flight seed " + std::to_string(seed + i)));
  // Every fault, on a basic pod and with the backup kit where it changes the outcome.
  struct FaultCase { FaultType f; bool baro2, gps2; };
  const FaultCase faults[] = {
      {FaultType::GpsLoss, false, false},   {FaultType::WinchStall, false, false},
      {FaultType::Lopsided, false, false},  {FaultType::Gust, false, false},
      {FaultType::Shear, false, false},     {FaultType::GpsLoss, false, true},
      {FaultType::BaroStuck, false, false}, {FaultType::BaroStuck, true, false},
      {FaultType::BaroDrift, false, false}, {FaultType::BaroDriftDrogue, false, false},
      {FaultType::BaroDriftDrogue, true, false},
  };
  uint64_t fs = seed + 1000000;
  for (const auto& fc : faults) {
    ScenarioOptions fo;
    fo.fault = fc.f;
    fo.baro2 = fc.baro2;
    fo.gps2 = fc.gps2;
    const std::string label = std::string(faultName(fc.f)) + (fc.baro2 ? "+kit" : "") + (fc.gps2 ? "+gps2" : "");
    for (int i = 0; i < faultFlights; ++i, ++fs)
      traces.push_back(flightTrace(makeScenario(fs, fo), "flight " + label + " seed " + std::to_string(fs)));
  }
  size_t steps = 0;
  for (const auto& t : traces) steps += t.steps.size() - 1;
  writeTla(out, traces);
  std::printf("grtrace: %zu traces, %zu recorded state changes -> %s\n", traces.size(), steps, out.c_str());
  return 0;
}
