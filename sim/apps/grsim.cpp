// grsim: Monte Carlo runner for the pod's software test gates.
//
//   grsim gate1  [--runs N] [--seed S] [--out DIR]             guidance on truth, simple model
//   grsim gate2  [--runs N] [--fault-runs K] [--seed S] [--out DIR] [--baseline FILE]
//                                                             real flight core, sensors, faults
//   grsim single --seed S [--mode sitl|simple] [--fault NAME] [--trace FILE]
//   grsim sweep  [--runs N] [--airspeeds 6,7,8,9] [--winds 2.2,4.5,6.7]
//   grsim synth-logs [--count N] [--out DIR]
//   grsim stress [--runs N] [--seed S] [--out DIR]               harsher-than-nominal conditions
//
// Stress dials (any command): --canopy-err L --turbulence L --swing L --gps L
// (--bay-tau S: the pod rides in a bay whose air lags outside pressure by S s)
// with L = 0 (nominal), 1 or 2; see sim::StressLevels.
//
// Common options: --threads T, --wind-max M (m/s at 10 m), --airspeed V, --sink S,
//                 --radius R (landing radius, m), --min-pct P (gate 1 pass mark)
// Exit status is non-zero when a gate fails, so CI can block the merge.

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

#include "gr/types.hpp"
#include "sim/logs.hpp"
#include "sim/report.hpp"
#include "sim/runners.hpp"
#include "sim/scenario.hpp"

using namespace sim;

namespace {

struct Args {
  std::string cmd;
  std::map<std::string, std::string> kv;
  bool has(const std::string& k) const { return kv.count(k) > 0; }
  std::string str(const std::string& k, const std::string& d) const { return has(k) ? kv.at(k) : d; }
  double num(const std::string& k, double d) const { return has(k) ? std::atof(kv.at(k).c_str()) : d; }
  long long integer(const std::string& k, long long d) const { return has(k) ? std::atoll(kv.at(k).c_str()) : d; }
};

Args parse(int argc, char** argv) {
  Args a;
  if (argc > 1) a.cmd = argv[1];
  for (int i = 2; i < argc; ++i) {
    std::string k = argv[i];
    if (k.rfind("--", 0) != 0) continue;
    k = k.substr(2);
    if (i + 1 < argc && std::strncmp(argv[i + 1], "--", 2) != 0) a.kv[k] = argv[++i];
    else a.kv[k] = "1";
  }
  return a;
}

void mkdirs(const std::string& path) {
  std::string cur;
  for (size_t i = 0; i < path.size(); ++i) {
    cur += path[i];
    if (path[i] == '/' || i + 1 == path.size()) mkdir(cur.c_str(), 0755);
  }
}

std::vector<double> parseList(const std::string& s) {
  std::vector<double> v;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) v.push_back(std::atof(item.c_str()));
  return v;
}

ScenarioOptions scenarioOptions(const Args& a) {
  ScenarioOptions o;
  o.windMax = a.num("wind-max", 4.47);
  o.canopyAirspeed = a.num("airspeed", 8.0);
  o.canopySink = a.num("sink", 3.5);
  if (a.has("dual-only")) o.dualDeployOnly = true;
  if (a.has("single-only")) o.singleDeployOnly = true;
  o.stress.canopy = static_cast<int>(a.integer("canopy-err", 0));
  o.stress.turbulence = static_cast<int>(a.integer("turbulence", 0));
  o.stress.swing = static_cast<int>(a.integer("swing", 0));
  o.stress.gps = static_cast<int>(a.integer("gps", 0));
  o.baro2 = a.has("baro2");
  o.gps2 = a.has("gps2");
  o.bayTau = a.num("bay-tau", 0.0);
  return o;
}

RunOptions runOptions(const Args& a) {
  RunOptions r;
  // The firmware's canopy belief follows the nominal canopy being simulated.
  r.params.canopy.airspeed = static_cast<float>(a.num("airspeed", 8.0));
  r.params.canopy.sinkRate = static_cast<float>(a.num("sink", 3.5));
  return r;
}

using RunFn = std::function<RunResult(const Scenario&)>;

std::vector<RunResult> runParallel(const std::vector<Scenario>& scs, const RunFn& fn, int threads) {
  std::vector<RunResult> out(scs.size());
  std::atomic<size_t> next{0};
  std::vector<std::thread> pool;
  for (int t = 0; t < threads; ++t) {
    pool.emplace_back([&]() {
      for (;;) {
        size_t i = next.fetch_add(1);
        if (i >= scs.size()) return;
        out[i] = fn(scs[i]);
      }
    });
  }
  for (auto& th : pool) th.join();
  return out;
}

int threadCount(const Args& a) {
  int hw = static_cast<int>(std::thread::hardware_concurrency());
  return static_cast<int>(a.integer("threads", hw > 0 ? hw : 4));
}

void writeFile(const std::string& path, const std::string& text) {
  std::ofstream f(path);
  f << text << "\n";
}

double readJsonNumber(const std::string& path, const std::string& key, double fallback) {
  std::ifstream f(path);
  if (!f) return fallback;
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string s = ss.str();
  const std::string pat = "\"" + key + "\":";
  size_t p = s.find(pat);
  if (p == std::string::npos) return fallback;
  return std::atof(s.c_str() + p + pat.size());
}

std::vector<Scenario> makeScenarios(uint64_t seed0, int n, const ScenarioOptions& o) {
  std::vector<Scenario> v;
  v.reserve(n);
  for (int i = 0; i < n; ++i) v.push_back(makeScenario(seed0 + static_cast<uint64_t>(i), o));
  return v;
}

void saveWorstTraces(const std::string& dir, const std::string& tag, const Summary& s, const ScenarioOptions& so,
                     const RunOptions& ro, bool sitl, int count) {
  mkdirs(dir + "/traces");
  for (int i = 0; i < count && i < static_cast<int>(s.worstSeeds.size()); ++i) {
    std::vector<TraceRow> tr;
    Scenario sc = makeScenario(s.worstSeeds[i], so);
    if (sitl) runSitl(sc, ro, &tr);
    else runSimple(sc, ro, &tr);
    writeTraceCsv(dir + "/traces/" + tag + "_seed" + std::to_string(s.worstSeeds[i]) + ".csv", tr);
  }
}

// --------------------------------------------------------------- commands

int cmdGate1(const Args& a) {
  const int n = static_cast<int>(a.integer("runs", 1000));
  const uint64_t seed = static_cast<uint64_t>(a.integer("seed", 1));
  const std::string out = a.str("out", "reports");
  const double radius = a.num("radius", 30.0);
  const double minPct = a.num("min-pct", 85.0);
  mkdirs(out);
  ScenarioOptions so = scenarioOptions(a);
  RunOptions ro = runOptions(a);
  auto scs = makeScenarios(seed, n, so);
  auto res = runParallel(scs, [&](const Scenario& s) { return runSimple(s, ro); }, threadCount(a));
  Summary sum = summarize(res, radius);
  printSummary("Gate 1: guidance in a simple model (truth, ideal actuators)", sum);
  writeRunsCsv(out + "/gate1_runs.csv", res);
  const bool pass = sum.withinPct >= minPct && sum.timedOut == 0;
  writeFile(out + "/gate1_summary.json", "{\"gate\":1,\"pass\":" + std::string(pass ? "true" : "false") +
                                             ",\"min_pct\":" + std::to_string(minPct) +
                                             ",\"summary\":" + summaryJson(sum) + "}");
  saveWorstTraces(out, "gate1", sum, so, ro, false, 3);
  std::printf("GATE 1 %s (need >= %.0f%% within %.0f m)\n", pass ? "PASS" : "FAIL", minPct, radius);
  return pass ? 0 : 1;
}

int cmdGate2(const Args& a) {
  const int n = static_cast<int>(a.integer("runs", 1000));
  const int nf = static_cast<int>(a.integer("fault-runs", 100));
  const uint64_t seed = static_cast<uint64_t>(a.integer("seed", 1));
  const std::string out = a.str("out", "reports");
  const double radius = a.num("radius", 30.0);
  const double maxDrop = a.num("max-drop", 5.0);
  mkdirs(out);
  ScenarioOptions so = scenarioOptions(a);
  RunOptions ro = runOptions(a);
  const int threads = threadCount(a);

  // Same seeds through gate 1 for an apples-to-apples comparison.
  auto nominal = makeScenarios(seed, n, so);
  auto g1 = runParallel(nominal, [&](const Scenario& s) { return runSimple(s, ro); }, threads);
  Summary s1 = summarize(g1, radius);

  std::vector<Scenario> all = nominal;
  // Each fault, on a basic pod and (where it matters) with the backup kit.
  struct FaultCase { FaultType f; bool baro2, gps2; };
  const FaultCase faultCases[] = {
      {FaultType::GpsLoss, false, false},         {FaultType::WinchStall, false, false},
      {FaultType::Lopsided, false, false},        {FaultType::Gust, false, false},
      {FaultType::Shear, false, false},           {FaultType::GpsLoss, false, true},
      {FaultType::BaroStuck, false, false},       {FaultType::BaroStuck, true, false},
      {FaultType::BaroDrift, false, false},       {FaultType::BaroDrift, true, false},
      {FaultType::BaroDriftDrogue, false, false}, {FaultType::BaroDriftDrogue, true, false},
  };
  uint64_t fseed = seed + 1000000;
  for (const auto& fc : faultCases) {
    ScenarioOptions fo = so;
    fo.fault = fc.f;
    fo.baro2 = fo.baro2 || fc.baro2;
    fo.gps2 = fo.gps2 || fc.gps2;
    for (int i = 0; i < nf; ++i) all.push_back(makeScenario(fseed++, fo));
  }
  auto res = runParallel(all, [&](const Scenario& s) { return runSitl(s, ro); }, threads);
  std::vector<RunResult> nomRes(res.begin(), res.begin() + n);
  Summary s2 = summarize(nomRes, radius);
  Summary sAll = summarize(res, radius);
  s2.faults = sAll.faults;

  printSummary("Gate 1 on the same seeds", s1);
  printSummary("Gate 2: software-in-the-loop (flight core + sensors + winch motors)", s2);
  writeRunsCsv(out + "/gate2_runs.csv", res);

  // Gate 2 criteria from the spec.
  std::vector<std::string> fails;
  if (s2.withinPct < s1.withinPct - maxDrop) fails.push_back("accuracy more than 5 points below gate 1");
  if (sAll.crashed) fails.push_back("crashes");
  if (sAll.navBeforeCanopy) fails.push_back("steering before canopy confirmed");
  if (s2.notConfirmed) fails.push_back("canopy never confirmed on nominal runs");
  if (sAll.timedOut) fails.push_back("runs timed out");
  for (const auto& kv : s2.faults) {
    if (kv.second.met < kv.second.exercised) fails.push_back("fault " + kv.first + " missed its expected outcome");
  }
  double baseline = -1.0;
  if (a.has("baseline")) {
    baseline = readJsonNumber(a.str("baseline", ""), "within_pct", -1.0);
    const double tol = a.num("baseline-tolerance", 2.0);
    if (baseline >= 0.0 && s2.withinPct < baseline - tol)
      fails.push_back("accuracy regressed vs baseline (" + std::to_string(baseline) + "%)");
  }
  const bool pass = fails.empty();
  std::string failJson = "[";
  for (size_t i = 0; i < fails.size(); ++i) failJson += (i ? ",\"" : "\"") + fails[i] + "\"";
  failJson += "]";
  writeFile(out + "/gate2_summary.json",
            "{\"gate\":2,\"pass\":" + std::string(pass ? "true" : "false") + ",\"failures\":" + failJson +
                ",\"gate1_same_seeds\":" + summaryJson(s1) + ",\"summary\":" + summaryJson(s2) +
                ",\"all_runs\":" + summaryJson(sAll) + "}");
  saveWorstTraces(out, "gate2", s2, so, ro, true, 3);
  for (const auto& f : fails) std::printf("  FAIL: %s\n", f.c_str());
  std::printf("GATE 2 %s (gate 1 %.1f%%, SITL %.1f%%, allowed drop %.0f points)\n", pass ? "PASS" : "FAIL",
              s1.withinPct, s2.withinPct, maxDrop);
  return pass ? 0 : 1;
}

int cmdSingle(const Args& a) {
  ScenarioOptions so = scenarioOptions(a);
  so.fault = faultFromName(a.str("fault", "none"));
  RunOptions ro = runOptions(a);
  Scenario s = makeScenario(static_cast<uint64_t>(a.integer("seed", 1)), so);
  std::vector<TraceRow> tr;
  const bool sitl = a.str("mode", "sitl") == "sitl";
  RunResult r = sitl ? runSitl(s, ro, &tr) : runSimple(s, ro, &tr);
  std::printf("seed %llu  mode %s  fault %s  %s deploy  wind %.1f m/s  Va0 %.2f vs0 %.2f\n",
              static_cast<unsigned long long>(s.seed), sitl ? "sitl" : "simple", faultName(s.fault),
              s.dualDeploy ? "dual" : "single", s.windRef, s.Va0, s.vs0);
  std::printf("canopy truth: turn gain %.2f rad/s per brake, lag %.2f s, rigging bias %.3f rad/s", s.Kturn, s.turnTau,
              s.turnBias);
  if (s.fault == FaultType::Lopsided)
    std::printf(", lopsided %.3f rad/s -> needs %.2f brake to fly straight", s.faultMagnitude,
                -(s.turnBias + s.faultMagnitude) / s.Kturn);
  std::printf("\n");
  std::printf("apogee %.0f m, deploy %.0f m AGL %.0f m from pad, landed %d, miss %.1f m, flight %.0f s\n", r.apogee,
              r.deployH, r.deployDist, r.landed, r.miss, r.flightTime);
  std::printf("canopy confirmed %d (delay %.2f s), nav-before-canopy %d, final phase %s, faults 0x%x, fallback delay %.1f s, "
              "expectation %s\n",
              r.canopyConfirmed, r.confirmDelay, r.navBeforeCanopy, gr::phaseName(static_cast<gr::Phase>(r.finalPhase)),
              r.faults, r.fallbackDelay, r.faultExercised ? (r.expectationMet ? "met" : "MISSED") : "n/a");
  if (a.has("trace")) writeTraceCsv(a.str("trace", "trace.csv"), tr);
  return 0;
}

int cmdSweep(const Args& a) {
  const int n = static_cast<int>(a.integer("runs", 300));
  const uint64_t seed = static_cast<uint64_t>(a.integer("seed", 1));
  const auto speeds = parseList(a.str("airspeeds", "6,7,8,9"));
  const auto winds = parseList(a.str("winds", "2.2,4.47,6.7"));
  const double radius = a.num("radius", 30.0);
  const std::string out = a.str("out", "reports");
  mkdirs(out);
  const bool sitl = a.str("mode", "sitl") == "sitl";
  std::string csv = "airspeed,wind_max,within_pct,cep50,p90\n";
  std::printf("Sweep (%s): %% landing within %.0f m, %d runs per cell\n", sitl ? "SITL" : "simple", radius, n);
  std::printf("airspeed \\ wind max  ");
  for (double w : winds) std::printf("%7.1f m/s", w);
  std::printf("\n");
  for (double va : speeds) {
    std::printf("  %4.1f m/s           ", va);
    for (double wm : winds) {
      Args b = a;
      b.kv["airspeed"] = std::to_string(va);
      b.kv["wind-max"] = std::to_string(wm);
      ScenarioOptions so = scenarioOptions(b);
      RunOptions ro = runOptions(b);
      auto scs = makeScenarios(seed, n, so);
      auto res = runParallel(scs, [&](const Scenario& s) { return sitl ? runSitl(s, ro) : runSimple(s, ro); },
                             threadCount(a));
      Summary sm = summarize(res, radius);
      std::printf("%9.1f%%", sm.withinPct);
      std::fflush(stdout);
      char line[128];
      std::snprintf(line, sizeof line, "%.1f,%.2f,%.2f,%.2f,%.2f\n", va, wm, sm.withinPct, sm.cep50, sm.p90);
      csv += line;
    }
    std::printf("\n");
  }
  writeFile(out + "/sweep.csv", csv);
  return 0;
}

// Stress sweep: the real flight core (SITL) under conditions harsher than
// the nominal Monte Carlo, one dial at a time and then all together. Reports
// accuracy and, more importantly, safety: steering before the canopy, crashes,
// and which fallbacks fired (a spiral is the designed safe outcome).
int cmdStress(const Args& a) {
  const int n = static_cast<int>(a.integer("runs", 500));
  const uint64_t seed = static_cast<uint64_t>(a.integer("seed", 70000));
  const double radius = a.num("radius", 30.0);
  const std::string out = a.str("out", "reports");
  mkdirs(out);
  struct Cond { const char* name; StressLevels st; bool gps2 = false; bool baro2 = false; };
  const Cond conds[] = {
      {"nominal", {0, 0, 0, 0}},
      {"canopy model 20% off", {1, 0, 0, 0}},
      {"canopy model 30% off", {2, 0, 0, 0}},
      {"turbulence moderate", {0, 1, 0, 0}},
      {"turbulence strong + thermals", {0, 2, 0, 0}},
      {"swing and twist moderate", {0, 0, 1, 0}},
      {"swing and twist strong", {0, 0, 2, 0}},
      {"GPS degraded under canopy", {0, 0, 0, 1}},
      {"GPS poor under canopy", {0, 0, 0, 2}},
      {"GPS degraded + 2nd GPS", {0, 0, 0, 1}, true},
      {"GPS poor + 2nd GPS", {0, 0, 0, 2}, true},
      {"combined moderate", {1, 1, 1, 1}},
      {"combined harsh", {2, 2, 2, 2}},
      {"combined moderate + full kit", {1, 1, 1, 1}, true, true},
  };
  std::string csv =
      "condition,canopy,turbulence,swing,gps,runs,within_pct,cep50,p90,max,not_confirmed,nav_before_canopy,crashed,"
      "timed_out,spiral_or_neutral,gps_lost,lopsided,winch,collapse,fence,confirm_delay_mean\n";
  std::printf("Stress (SITL, %d runs each, winds to %.1f m/s, seed %llu)\n", n, a.num("wind-max", 4.47),
              static_cast<unsigned long long>(seed));
  std::printf("%-30s %7s %6s %6s %7s  %5s %5s %5s  %s\n", "condition", "<=30m", "CEP50", "p90", "max", "steer", "crash",
              "noCnf", "fallbacks (gps/lopsided/winch/collapse/fence)");
  int unsafe = 0;
  for (const auto& c : conds) {
    Args b = a;
    ScenarioOptions so = scenarioOptions(b);
    so.stress = c.st;
    so.gps2 = so.gps2 || c.gps2;
    so.baro2 = so.baro2 || c.baro2;
    RunOptions ro = runOptions(b);
    auto scs = makeScenarios(seed, n, so);
    auto res = runParallel(scs, [&](const Scenario& sc) { return runSitl(sc, ro); }, threadCount(a));
    Summary sm = summarize(res, radius);
    std::string tag = c.name;
    for (auto& ch : tag) if (ch == ' ' || ch == '%' || ch == '+') ch = '_';
    writeRunsCsv(out + "/stress_" + tag + ".csv", res);
    int gps = 0, lop = 0, winch = 0, col = 0, fence = 0;
    for (const auto& r : res) {
      if (r.faults & gr::kFaultGpsLost) ++gps;
      if (r.faults & gr::kFaultLopsided) ++lop;
      if (r.faults & (gr::kFaultWinchLeft | gr::kFaultWinchRight)) ++winch;
      if (r.faults & gr::kFaultCollapse) ++col;
      if (r.faults & gr::kFaultFence) ++fence;
    }
    unsafe += sm.navBeforeCanopy + sm.crashed;
    std::printf("%-30s %6.1f%% %5.1fm %5.1fm %6.0fm  %5d %5d %5d  %d / %d / %d / %d / %d\n", c.name, sm.withinPct,
                sm.cep50, sm.p90, sm.maxMiss, sm.navBeforeCanopy, sm.crashed, sm.notConfirmed, gps, lop, winch, col,
                fence);
    std::fflush(stdout);
    char line[512];
    std::snprintf(line, sizeof line, "%s,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f\n", c.name,
                  c.st.canopy, c.st.turbulence, c.st.swing, c.st.gps, sm.runs, sm.withinPct, sm.cep50, sm.p90,
                  sm.maxMiss, sm.notConfirmed, sm.navBeforeCanopy, sm.crashed, sm.timedOut, sm.unexpectedFallback, gps,
                  lop, winch, col, fence, sm.confirmDelayMean);
    csv += line;
  }
  writeFile(out + "/stress.csv", csv);
  std::printf("steer = steering before the canopy was confirmed; noCnf = canopy never confirmed.\n");
  std::printf("STRESS %s (safety: %d runs steered early or crashed)\n", unsafe ? "FAIL" : "SAFE", unsafe);
  return unsafe ? 1 : 0;
}

int cmdSynthLogs(const Args& a) {
  const int n = static_cast<int>(a.integer("count", 200));
  const uint64_t seed = static_cast<uint64_t>(a.integer("seed", 1));
  const std::string out = a.str("out", "logs/synthetic");
  mkdirs(out);
  int mains = 0, failures = 0;
  for (int i = 0; i < n; ++i) {
    FlightLog log = synthesizeLog(seed + static_cast<uint64_t>(i));
    char name[64];
    std::snprintf(name, sizeof name, "/synthetic_%04d.csv", i);
    writeFlightCsv(out + name, log);
    (log.mainT >= 0 ? mains : failures)++;
  }
  std::printf("wrote %d synthetic logs to %s (%d with a main deployment, %d main failures)\n", n, out.c_str(), mains,
              failures);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Args a = parse(argc, argv);
  if (a.cmd == "gate1") return cmdGate1(a);
  if (a.cmd == "gate2") return cmdGate2(a);
  if (a.cmd == "single") return cmdSingle(a);
  if (a.cmd == "sweep") return cmdSweep(a);
  if (a.cmd == "synth-logs") return cmdSynthLogs(a);
  if (a.cmd == "stress") return cmdStress(a);
  std::fprintf(stderr,
               "usage: grsim gate1|gate2|single|sweep|synth-logs|stress [options]\n"
               "  see the comment at the top of sim/apps/grsim.cpp\n");
  return 2;
}
