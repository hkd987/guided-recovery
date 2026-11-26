#include "sim/logs.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>

#include "gr/flight_core.hpp"
#include "sim/sensors.hpp"
#include "sim/world.hpp"

namespace sim {

namespace {

std::string lower(std::string s) {
  for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

std::string trim(const std::string& s) {
  size_t a = 0, b = s.size();
  while (a < b && (std::isspace(static_cast<unsigned char>(s[a])) || s[a] == '"')) ++a;
  while (b > a && (std::isspace(static_cast<unsigned char>(s[b - 1])) || s[b - 1] == '"')) --b;
  return s.substr(a, b - a);
}

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : line) {
    if (c == ',') { out.push_back(trim(cur)); cur.clear(); }
    else cur += c;
  }
  out.push_back(trim(cur));
  return out;
}

int findCol(const std::vector<std::string>& h, std::initializer_list<const char*> names) {
  for (const char* n : names)
    for (size_t i = 0; i < h.size(); ++i)
      if (h[i] == n) return static_cast<int>(i);
  return -1;
}

}  // namespace

bool readFlightCsv(const std::string& path, FlightLog& out, std::string* err) {
  std::ifstream f(path);
  if (!f) { if (err) *err = "cannot open"; return false; }
  out = FlightLog{};
  out.name = path;
  std::string line;
  std::vector<std::string> header;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    header = splitCsv(line);
    for (auto& h : header) h = lower(h);
    break;
  }
  const int cT = findCol(header, {"time", "t", "time_s"});
  const int cA = findCol(header, {"acceleration", "accel", "accel_ms2"});
  const int cP = findCol(header, {"pressure", "pressure_pa"});
  const int cH = findCol(header, {"height", "altitude", "height_m"});
  const int cS = findCol(header, {"state_name", "state"});
  const int cM = findCol(header, {"main_t"});
  if (cT < 0 || cA < 0 || (cP < 0 && cH < 0)) {
    if (err) *err = "missing time/acceleration/pressure-or-height columns";
    return false;
  }
  std::string prevState;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    auto c = splitCsv(line);
    auto get = [&](int i) -> double {
      if (i < 0 || i >= static_cast<int>(c.size()) || c[i].empty()) return NAN;
      try { return std::stod(c[i]); } catch (...) { return NAN; }
    };
    LogSample s;
    s.t = get(cT);
    s.accel = get(cA);
    if (cP >= 0) {
      s.pressure = get(cP);
      if (s.pressure > 0 && s.pressure < 2000.0) s.pressure *= 100.0;  // hPa
    } else {
      s.pressure = isaPressure(1000.0 + get(cH));
    }
    if (!std::isfinite(s.t) || !std::isfinite(s.accel) || !std::isfinite(s.pressure)) continue;
    if (cS >= 0 && cS < static_cast<int>(c.size())) s.state = lower(c[cS]);
    if (cM >= 0 && !out.hasTruth) {
      const double m = get(cM);
      if (std::isfinite(m)) { out.mainT = m; out.hasTruth = true; }
    }
    if (!s.state.empty() && s.state != prevState) {
      if (s.state == "main" && cM < 0 && out.mainT < 0) { out.mainT = s.t; out.hasTruth = true; }
      if (s.state == "landed" && out.landT < 0) out.landT = s.t;
      prevState = s.state;
    }
    out.samples.push_back(s);
  }
  if (cS >= 0 && cM < 0) out.hasTruth = true;  // states logged: no "main" state means no main
  if (out.samples.size() < 10) { if (err) *err = "too few samples"; return false; }
  return true;
}

bool writeFlightCsv(const std::string& path, const FlightLog& log) {
  FILE* f = std::fopen(path.c_str(), "w");
  if (!f) return false;
  std::fprintf(f, "# synthetic host flight log (guided-recovery sim). acceleration: axial, gravity removed\n");
  std::fprintf(f, "time,state_name,acceleration,pressure,main_t\n");
  for (const auto& s : log.samples)
    std::fprintf(f, "%.3f,%s,%.2f,%.1f,%.3f\n", s.t, s.state.c_str(), s.accel, s.pressure, log.mainT);
  std::fclose(f);
  return true;
}

FlightLog synthesizeLog(uint64_t seed) {
  Rng r(seed * 31337ULL + 99ULL);
  ScenarioOptions o;
  o.windMax = 9.0;
  o.canopySink = r.uniform(3.0, 8.0);  // parafoils and round mains alike
  o.canopyAirspeed = o.canopySink > 5.5 ? 1.0 : r.uniform(5.0, 9.0);  // round chutes barely glide
  Scenario s = makeScenario(seed, o);
  s.launchTime = 5.0;
  s.armTime = 0.0;
  if (s.dualDeploy && r.chance(0.15)) s.mainFailure = true;

  World w(s);
  SensorSuite sens(s);
  FlightLog log;
  log.name = "synthetic_" + std::to_string(seed);
  const double duty[2] = {0.0, 0.0};
  gr::Inputs in;
  double lastLogT = -1.0;
  double landedAt = -1.0;
  float lastP = 0.0f;
  while (w.t() < 900.0) {
    w.step(0.005, duty, 8.0);
    sens.sample(w, duty, in);
    if (in.baroFresh) lastP = in.pressurePa;
    if (w.t() - lastLogT >= 0.02 - 1e-9 && lastP > 0.0f) {
      lastLogT = w.t();
      LogSample ls;
      ls.t = w.t();
      // Host accelerometer: along the airframe axis, gravity removed.
      ls.accel = -static_cast<double>(in.imu.accel.z) - kG;
      ls.pressure = lastP;
      const TruthStage st = w.stage();
      if (st == TruthStage::Pad) ls.state = "pad";
      else if (st == TruthStage::Boost) ls.state = "boost";
      else if (st == TruthStage::Coast) ls.state = w.apogeeT() >= 0 ? "drogue" : "coast";
      else if (st == TruthStage::Drogue) ls.state = "drogue";
      else if (st == TruthStage::Landed) ls.state = "landed";
      else ls.state = "main";
      log.samples.push_back(ls);
    }
    if (w.stage() == TruthStage::Landed) {
      if (landedAt < 0) landedAt = w.t();
      if (w.t() > landedAt + 5.0) break;
    }
  }
  log.mainT = w.deployT();
  log.landT = landedAt;
  log.hasTruth = true;
  return log;
}

ReplayResult replayLog(const FlightLog& log, const gr::Params& params, double window) {
  ReplayResult res;
  res.name = log.name;
  res.hasMain = log.mainT >= 0.0;
  res.mainT = log.mainT;
  if (log.samples.empty()) return res;

  gr::FlightCore core(params);
  std::vector<LogSample> s = log.samples;
  // Logs that start mid-boost get a short synthetic pad segment.
  if (std::fabs(s.front().accel) > 2.0 * kG) {
    LogSample pad = s.front();
    pad.accel = 0.0;
    for (int i = 20; i >= 1; --i) {
      pad.t = s.front().t - 0.1 * i;
      s.insert(s.begin(), pad);
    }
  }
  const double t0 = s.front().t;
  const double t1 = s.back().t;
  size_t idx = 0;
  size_t lastIdx = static_cast<size_t>(-1);
  gr::Inputs in;
  for (double tt = t0; tt <= t1; tt += 0.005) {
    while (idx + 1 < s.size() && s[idx + 1].t <= tt) ++idx;
    in = gr::Inputs{};
    in.tUs = static_cast<uint64_t>(std::llround((tt - t0 + 1.0) * 1e6));
    const double f = s[idx].accel + kG;  // back to specific force along the axis
    in.imu.accel = gr::Vec3{0.0f, 0.0f, static_cast<float>(-f)};
    in.imu.valid = true;
    if (idx != lastIdx) {
      in.baroFresh = true;
      in.pressurePa = static_cast<float>(s[idx].pressure);
      lastIdx = idx;
    }
    in.armed = true;
    in.batteryV = 8.0f;
    core.update(in);
    if (!res.detected && core.status().stage == gr::FlightStage::Canopy) {
      res.detected = true;
      res.detectT = tt;
    }
  }
  if (res.hasMain) {
    if (res.detected) {
      res.delay = res.detectT - res.mainT;
      if (res.delay < -0.5) res.falseTrigger = true;
      else if (res.delay > window) res.missed = true;
    } else {
      res.missed = true;
    }
  } else if (res.detected) {
    res.falseTrigger = true;
  }
  return res;
}

}  // namespace sim
