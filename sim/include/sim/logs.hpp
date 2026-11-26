// Flight logs for gate 3: read host flight-computer CSV exports (AltOS style
// or a simple generic format), synthesize labelled logs from the simulator,
// and replay a log through the pod's deployment detector.
#pragma once

#include <string>
#include <vector>

#include "gr/params.hpp"

namespace sim {

struct LogSample {
  double t = 0.0;         // s
  double accel = 0.0;     // m/s^2 along the airframe axis, gravity removed (AltOS convention: 0 on the pad)
  double pressure = 0.0;  // Pa
  std::string state;      // host's flight state name, if logged
};

struct FlightLog {
  std::string name;
  std::vector<LogSample> samples;
  // Ground truth: time the main (or parafoil) came out; < 0 if it never did.
  double mainT = -1.0;
  double landT = -1.0;
  bool hasTruth = false;
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

struct ReplayResult {
  std::string name;
  bool hasMain = false;
  double mainT = -1.0;
  bool detected = false;
  double detectT = -1.0;
  double delay = -1.0;     // detection - main event
  bool falseTrigger = false;  // detected before the main event, or with no main at all
  bool missed = false;     // main came out but no detection within the window
};

ReplayResult replayLog(const FlightLog& log, const gr::Params& params, double window = 20.0);

}  // namespace sim
