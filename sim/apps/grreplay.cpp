// grreplay: gate 3. Feeds recorded flight logs through the pod's deployment
// detector and checks it catches every main deployment without triggering
// during boost, coast or drogue descent.
//
//   grreplay [--min-flights N] [--window S] [--main-alt M|log] [--out FILE]
//            [--trace DIR] LOG.csv [LOG.csv ...]
//
// Each log is judged offline first (sim::judgeMain): did the descent after
// the main settle into the canopy band (the pod must confirm it), or did the
// main fail or come down too fast for any parafoil (confirming is optional),
// or does the log end too soon to tell?
//
// Gate 3 fails on:
//   - any confirmation before the main event (or with no main at all);
//   - any confirmation while the log shows a fall far too fast for a canopy;
//   - a canopy the pod did not confirm within --window s of it settling;
//   - fewer than --min-flights logs with a main deployment.
//
// A dual deploy flight whose drogue already fell inside the canopy band is
// indistinguishable from a canopy by descent rate. Without canopy.mainAlt set
// such a confirmation is reported, not failed (an unsupported setup, see
// docs/BUILD_GUIDE.md); with it set, it fails like any other.
//
// --main-alt log  sets the pod's canopy.mainAlt from each dual deploy log (the
//                 height at the host's main event), as a builder would.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "gr/params.hpp"
#include "sim/logs.hpp"

using namespace sim;

namespace {
std::string baseName(const std::string& p) {
  const size_t k = p.find_last_of('/');
  return k == std::string::npos ? p : p.substr(k + 1);
}
}  // namespace

int main(int argc, char** argv) {
  int minFlights = 0;
  ReplayOptions opt;
  std::string out, traceDir;
  std::vector<std::string> files;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--min-flights") && i + 1 < argc) minFlights = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--window") && i + 1 < argc) opt.window = std::atof(argv[++i]);
    else if (!std::strcmp(argv[i], "--main-alt") && i + 1 < argc) {
      const char* v = argv[++i];
      opt.mainAlt = !std::strcmp(v, "log") ? -1.0 : std::atof(v);
    } else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
    else if (!std::strcmp(argv[i], "--trace") && i + 1 < argc) traceDir = argv[++i];
    else files.push_back(argv[i]);
  }
  gr::Params params;
  int replayed = 0, withMain = 0, unreadable = 0;
  int falseTriggers = 0, implausible = 0, missed = 0, slowDrogueUnset = 0, slowDrogueFlights = 0;
  std::map<MainOutcome, int> byOutcome, detectedBy;
  double delaySum = 0.0, delayMax = 0.0, settleDelaySum = 0.0, settleDelayMax = 0.0;
  int canopyDetected = 0;
  std::string csv =
      "log,accel_source,has_main,main_t,deploy_t,slow_drogue,main_alt,outcome,settle_t,detected,detect_t,detect_agl,"
      "detect_descent,delay,false_trigger,implausible,missed,descent_dt\n";
  int companions = 0, noTruth = 0;
  for (const auto& f : files) {
    FlightLog log;
    std::string err;
    bool companion = false;
    const bool ok = readFlightCsv(f, log, &err, &companion);
    if (companion) {  // Blue Raven high-rate file: read with its low-rate file
      ++companions;
      continue;
    }
    if (!ok || !log.hasTruth) {
      std::fprintf(stderr, "skip %s: %s\n", f.c_str(), err.empty() ? "no deployment truth" : err.c_str());
      if (ok) ++noTruth;
      else ++unreadable;
      continue;
    }
    ReplayOptions o = opt;
    FILE* tf = nullptr;
    if (!traceDir.empty()) {
      tf = std::fopen((traceDir + "/" + baseName(f) + ".trace.csv").c_str(), "w");
      o.trace = tf;
    }
    ReplayResult r = replayLog(log, params, o);
    if (tf) std::fclose(tf);
    ++replayed;
    if (r.hasMain) ++withMain;
    ++byOutcome[r.outcome];
    if (r.detected && !r.falseTrigger) ++detectedBy[r.outcome];
    if (r.outcome == MainOutcome::Canopy && r.detected && !r.falseTrigger && !r.missed) {
      ++canopyDetected;
      delaySum += r.delay;
      if (r.delay > delayMax) delayMax = r.delay;
      const double sd = r.detectT - r.settleT;
      settleDelaySum += sd;
      if (sd > settleDelayMax) settleDelayMax = sd;
    }
    const std::string name = baseName(f);
    if (r.slowDrogue) ++slowDrogueFlights;
    if (r.falseTrigger && r.slowDrogue && r.mainAlt <= 0.0) {
      ++slowDrogueUnset;
      std::printf("SLOW DROGUE %s: confirmed under the drogue at %.2f s (%.0f m up), main at %.2f s; needs canopy.mainAlt\n",
                  name.c_str(), r.detectT, r.detectAgl, r.mainT);
    } else if (r.falseTrigger) {
      ++falseTriggers;
      std::printf("FALSE TRIGGER %s: confirmed at %.2f s (%.0f m up), main at %.2f s%s\n", name.c_str(), r.detectT,
                  r.detectAgl, r.deployT, r.slowDrogue ? " (slow drogue)" : "");
    }
    if (r.implausible) {
      ++implausible;
      std::printf("IMPLAUSIBLE %s: confirmed at %.2f s while falling %.1f m/s\n", name.c_str(), r.detectT,
                  r.detectDescent);
    }
    if (r.missed) {
      ++missed;
      std::printf("MISSED %s: main at %.2f s, settled at %.2f s, confirmed %s %.2f s\n", name.c_str(), r.mainT,
                  r.settleT, r.detected ? "at" : "never", r.detectT);
    }
    char line[1024];
    std::snprintf(line, sizeof line, "%s,%s,%d,%.3f,%.3f,%d,%.0f,%s,%.3f,%d,%.3f,%.1f,%.2f,%.3f,%d,%d,%d,%.3f\n",
                  name.c_str(), log.accelSource.c_str(), r.hasMain, r.mainT, r.deployT, r.slowDrogue, r.mainAlt,
                  outcomeName(r.outcome), r.settleT, r.detected, r.detectT, r.detectAgl, r.detectDescent, r.delay,
                  r.falseTrigger, r.implausible, r.missed, r.descentDt);
    csv += line;
  }
  const int nCanopy = byOutcome[MainOutcome::Canopy];
  std::printf("== Gate 3: log replay ==\n");
  std::printf("  logs replayed %d (unreadable %d), with a main deployment %d, without %d\n", replayed, unreadable,
              withMain, replayed - withMain);
  if (noTruth || companions)
    std::printf("  skipped: %d with no deployment truth, %d Blue Raven high-rate files (read with their low-rate file)\n",
                noTruth, companions);
  std::printf(
      "  outcome after the main: canopy %d, at a band edge %d, fast or failed %d, settled below 30 m %d, log too short "
      "%d\n",
      nCanopy, byOutcome[MainOutcome::Borderline], byOutcome[MainOutcome::FastOrFailed], byOutcome[MainOutcome::TooLow],
      byOutcome[MainOutcome::Truncated]);
  std::printf("  canopies confirmed %d/%d, missed %d\n", canopyDetected, nCanopy, missed);
  std::printf("  confirmed anyway (allowed): at a band edge %d/%d, after a fast/failed main %d/%d\n",
              detectedBy[MainOutcome::Borderline], byOutcome[MainOutcome::Borderline],
              detectedBy[MainOutcome::FastOrFailed], byOutcome[MainOutcome::FastOrFailed]);
  std::printf("  false triggers %d, implausible confirmations %d\n", falseTriggers, implausible);
  std::printf("  slow-drogue flights %d (confirmed under the drogue with no canopy.mainAlt: %d)\n", slowDrogueFlights,
              slowDrogueUnset);
  if (canopyDetected)
    std::printf("  delay after main: mean %.2f s, max %.2f s; after the descent settled: mean %.2f s, max %.2f s\n",
                delaySum / canopyDetected, delayMax, settleDelaySum / canopyDetected, settleDelayMax);
  if (!out.empty()) {
    std::ofstream fo(out);
    fo << csv;
  }
  const bool enough = withMain >= minFlights && replayed > 0;
  const bool pass = falseTriggers == 0 && implausible == 0 && missed == 0 && enough;
  std::printf("GATE 3 %s%s\n", pass ? "PASS" : "FAIL", enough ? "" : " (not enough flights)");
  return pass ? 0 : 1;
}
