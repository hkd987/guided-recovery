// grreplay: gate 3. Feeds recorded flight logs through the pod's deployment
// detector and checks it catches every main deployment without triggering
// during boost, coast or drogue descent.
//
//   grreplay [--min-flights N] [--window S] [--out FILE] LOG.csv [LOG.csv ...]
//
// Exit status is non-zero on any false trigger or missed deployment, or if
// fewer than --min-flights logs with ground truth were replayed.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "gr/params.hpp"
#include "sim/logs.hpp"

using namespace sim;

int main(int argc, char** argv) {
  int minFlights = 0;
  double window = 20.0;
  std::string out;
  std::vector<std::string> files;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--min-flights") && i + 1 < argc) minFlights = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--window") && i + 1 < argc) window = std::atof(argv[++i]);
    else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
    else files.push_back(argv[i]);
  }
  gr::Params params;
  int replayed = 0, withMain = 0, detected = 0, falseTriggers = 0, missed = 0, unreadable = 0;
  double delaySum = 0.0, delayMax = 0.0;
  std::string csv = "log,has_main,main_t,detected,detect_t,delay,false_trigger,missed\n";
  for (const auto& f : files) {
    FlightLog log;
    std::string err;
    if (!readFlightCsv(f, log, &err) || !log.hasTruth) {
      std::fprintf(stderr, "skip %s: %s\n", f.c_str(), err.empty() ? "no deployment truth" : err.c_str());
      ++unreadable;
      continue;
    }
    ReplayResult r = replayLog(log, params, window);
    ++replayed;
    if (r.hasMain) ++withMain;
    if (r.hasMain && r.detected && !r.falseTrigger && !r.missed) {
      ++detected;
      delaySum += r.delay;
      if (r.delay > delayMax) delayMax = r.delay;
    }
    if (r.falseTrigger) {
      ++falseTriggers;
      std::printf("FALSE TRIGGER %s: detected at %.2f s, main at %.2f s\n", f.c_str(), r.detectT, r.mainT);
    }
    if (r.missed) {
      ++missed;
      std::printf("MISSED %s: main at %.2f s, detected %s %.2f s\n", f.c_str(), r.mainT, r.detected ? "at" : "never",
                  r.detectT);
    }
    char line[512];
    std::snprintf(line, sizeof line, "%s,%d,%.3f,%d,%.3f,%.3f,%d,%d\n", f.c_str(), r.hasMain, r.mainT, r.detected,
                  r.detectT, r.delay, r.falseTrigger, r.missed);
    csv += line;
  }
  std::printf("== Gate 3: log replay ==\n");
  std::printf("  logs replayed %d (unreadable %d), with a main deployment %d, without %d\n", replayed, unreadable,
              withMain, replayed - withMain);
  std::printf("  detected %d/%d, missed %d, false triggers %d\n", detected, withMain, missed, falseTriggers);
  if (detected) std::printf("  detection delay after main: mean %.2f s, max %.2f s\n", delaySum / detected, delayMax);
  if (!out.empty()) {
    std::ofstream f(out);
    f << csv;
  }
  const bool pass = falseTriggers == 0 && missed == 0 && replayed >= minFlights && replayed > 0;
  std::printf("GATE 3 %s%s\n", pass ? "PASS" : "FAIL",
              replayed < minFlights ? " (not enough flights)" : "");
  return pass ? 0 : 1;
}
