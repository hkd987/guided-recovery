// Flight logs for gate 3: read host flight-computer CSV exports (AltOS style
// or a simple generic format), synthesize labelled logs from the simulator,
// and replay a log through the pod's deployment detector.
#pragma once

#include <cstdio>
#include <string>
#include <vector>

#include "gr/params.hpp"

namespace sim {

struct LogSample {
  double t = 0.0;         // s
  double accel = 0.0;     // m/s^2 along the airframe axis, gravity removed (AltOS convention: 0 on the pad)
  double pressure = 0.0;  // Pa
  // Cross-axis specific force (m/s^2), when the log has a 3-axis IMU
  // (TeleMega, EasyMega: accel_x/y/z). Lets the replay feed the pod the true
  // magnitude when the airframe is not vertical, e.g. hanging under a chute.
  double lat1 = 0.0, lat2 = 0.0;
  std::string state;      // host's flight state name, if logged
};

struct FlightLog {
  std::string name;
  std::vector<LogSample> samples;
  // Ground truth: time the main (or parafoil) came out; < 0 if it never did.
  double mainT = -1.0;
  double landT = -1.0;
  bool hasTruth = false;
  bool hasLateral = false;  // lat1/lat2 are real measurements
  // "3-axis", "axial", or "baro-derived" (EasyMini: no accelerometer; AltosUI
  // fills the acceleration column from the barometer, so no opening shocks).
  std::string accelSource;
};

// Reads a CSV with a header row. Recognised columns (case-insensitive):
//   time | accel, acceleration | pressure (Pa or hPa) | height, altitude (m, if no pressure) | state_name, state
// A column "main_t" or a state change to "main" gives the deployment truth.
bool readFlightCsv(const std::string& path, FlightLog& out, std::string* err = nullptr);
bool writeFlightCsv(const std::string& path, const FlightLog& log);

// Generates labelled synthetic logs with the simulator: random rockets,
// single and dual deploy, round chutes and parafoils at 3-8 m/s, some main
// failures. Written as host-style CSV at 50 Hz.
FlightLog synthesizeLog(uint64_t seed);

// What the log itself says happened after the main came out, judged offline
// with the whole log in hand (smoothed, non-causal): the yardstick the
// detector is scored against.
enum class MainOutcome {
  None,        // no main deployment in the log
  Canopy,      // descent settled into the canopy band: the pod must confirm it
  FastOrFailed,  // main out, but descent never slowed into the band (failed,
                 // tangled, or a round chute heavier than any parafoil flies)
  Truncated,   // log ends too soon after the main to tell
  TooLow,      // the canopy only settled below 30 m: nothing left to steer
  Borderline,  // settled within 0.5 m/s of a band edge: not a fair test either way
};
const char* outcomeName(MainOutcome o);

struct ReplayResult {
  std::string name;
  bool hasMain = false;
  double mainT = -1.0;
  MainOutcome outcome = MainOutcome::None;
  double settleT = -1.0;   // Canopy outcome: when the descent settled into the band
  double deployT = -1.0;   // main event used for scoring (host's main, or apogee if that came later)
  bool slowDrogue = false; // dual deploy whose drogue already fell inside the canopy band
  double mainAlt = 0.0;    // canopy.mainAlt the pod was given for this replay
  bool detected = false;
  double detectT = -1.0;
  double detectAgl = 0.0;  // pod's altitude estimate at confirmation
  double detectDescent = 0.0;  // the log's own descent rate around confirmation, m/s
  double delay = -1.0;     // detection - main event
  bool falseTrigger = false;  // confirmed before the main event, or with no main at all
  bool implausible = false;   // confirmed while the log shows a fall far too fast for any canopy
  bool missed = false;     // Canopy outcome but no confirmation within the window after settling
};

struct ReplayOptions {
  double window = 10.0;  // s after the descent settles into the band
  // Logs with no accelerometer show the pod no opening jolt, leaving only the
  // slower no-jolt confirmation path; they get this window instead.
  double windowNoAccel = 15.0;
  // Main deployment altitude the builder sets on the pod (canopy.mainAlt).
  // < 0: take it from the log (height at the host's main event) for dual
  // deploy logs, 0 for single deploy. 0: not set.
  double mainAlt = 0.0;
  std::FILE* trace = nullptr;  // per-sample CSV trace of the detector, if set
};

ReplayResult replayLog(const FlightLog& log, const gr::Params& params, const ReplayOptions& opt = ReplayOptions{});

// Offline judgement of the log (see MainOutcome). band = canopy descent band, m/s.
struct MainJudgement {
  MainOutcome outcome = MainOutcome::None;
  double deployT = -1.0;  // host's main event, moved to apogee if it came before it
  double settleT = -1.0;  // Canopy: when the descent had settled into the band
  bool slowDrogue = false;
};
MainJudgement judgeMain(const FlightLog& log, double bandLow, double bandHigh);

}  // namespace sim
