#include "sim/logs.hpp"

#include <algorithm>
#include <array>
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

// "Height (ft)" -> name "height", unit "ft". Lower-cases both.
void splitUnit(const std::string& col, std::string& name, std::string& unit) {
  std::string s = lower(col);
  unit.clear();
  const size_t open = s.find_first_of("([");
  if (open != std::string::npos) {
    const size_t close = s.find_first_of(")]", open);
    unit = trim(s.substr(open + 1, close == std::string::npos ? std::string::npos : close - open - 1));
    s = s.substr(0, open);
  }
  name = trim(s);
}

bool looksLikeHeader(const std::vector<std::string>& cols) {
  std::string n, u;
  for (const auto& c : cols) {
    splitUnit(c, n, u);
    if (n == "time" || n == "t" || n == "time_s") return true;
  }
  return false;
}

double field(const std::vector<std::string>& c, int i) {
  if (i < 0 || i >= static_cast<int>(c.size()) || c[i].empty()) return NAN;
  try { return std::stod(c[i]); } catch (...) { return NAN; }
}

// Header names after splitUnit (lower case, units stripped).
std::vector<std::string> readHeaderNames(std::ifstream& f) {
  std::string line;
  if (!std::getline(f, line)) return {};
  if (!line.empty() && line.back() == '\r') line.pop_back();
  if (line.size() >= 3 && static_cast<unsigned char>(line[0]) == 0xEF) line.erase(0, 3);
  auto h = splitCsv(line);
  std::string u;
  for (auto& x : h) splitUnit(x, x, u);
  return h;
}

struct BrLow {
  double t, p, axialG, latG;
  std::string state;
};

// Blue Raven low-rate file, header already read. Its high-rate sibling, if
// present, supplies the 3-axis accelerometer at 500 Hz.
bool readBlueRaven(std::ifstream& f, const std::vector<std::string>& header, const std::string& path,
                   FlightLog& out, std::string* err) {
  const int cT = findCol(header, {"flight_time_"});
  const int cP = findCol(header, {"baro_press_"});
  const int cLift = findCol(header, {"liftoff"});
  const int cApo = findCol(header, {"apogee"});
  const int cAf = findCol(header, {"apo_fired"});
  const int cMf = findCol(header, {"main_fired"});
  const int cAx = findCol(header, {"axial accel"});
  const int cLat = findCol(header, {"lateral accel"});
  if (cT < 0 || cP < 0) { if (err) *err = "Blue Raven file without flight time or pressure"; return false; }

  std::vector<BrLow> low;
  std::string line;
  bool lifted = false;
  double mainT = -1.0;
  while (std::getline(f, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) continue;
    const auto c = splitCsv(line);
    const double t = field(c, cT), atm = field(c, cP);
    if (!std::isfinite(t) || !std::isfinite(atm) || atm <= 0.0) continue;
    if (!low.empty() && t <= low.back().t) continue;
    const bool lift = field(c, cLift) > 0.5, apo = field(c, cApo) > 0.5;
    const bool af = field(c, cAf) > 0.5, mf = field(c, cMf) > 0.5;
    // Some exports hold the flight twice: the flags drop back to zero where
    // the second copy starts. Keep the first.
    if (cLift >= 0 && lifted && !lift && !apo && !af && !mf) break;
    lifted = lifted || lift;
    BrLow r{t, atm * 101325.0, field(c, cAx), field(c, cLat), ""};
    if (mf) {
      r.state = "main";
      if (mainT < 0.0) mainT = t;
    } else if (apo || af) {
      r.state = af ? "drogue" : "coast";
    } else {
      r.state = lift ? "boost" : "pad";
    }
    low.push_back(r);
  }
  if (low.size() < 10) { if (err) *err = "too few samples"; return false; }
  out.mainT = mainT;
  // The main channel's flag is the only deployment truth: a unit whose main
  // never fired (backup computer, or the main fired by another one) says
  // nothing about when the canopy came out.
  out.hasTruth = mainT >= 0.0;
  if (!out.hasTruth && err) *err = "Blue Raven main channel never fired: no deployment truth";

  // High-rate sibling: 3-axis specific force in g.
  std::vector<std::array<double, 4>> high;  // t, x, y, z
  const std::string hrPath = blueRavenHighRatePath(path);
  if (!hrPath.empty()) {
    std::ifstream hf(hrPath);
    if (hf) {
      const auto hh = readHeaderNames(hf);
      const int hT = findCol(hh, {"flight_time_"});
      const int hX = findCol(hh, {"accel_x"}), hY = findCol(hh, {"accel_y"}), hZ = findCol(hh, {"accel_z"});
      if (hT >= 0 && hX >= 0 && hY >= 0 && hZ >= 0) {
        while (std::getline(hf, line)) {
          if (!line.empty() && line.back() == '\r') line.pop_back();
          const auto c = splitCsv(line);
          const double t = field(c, hT);
          if (!std::isfinite(t) || t < low.front().t) continue;
          if (t > low.back().t) break;
          if (!high.empty() && t <= high.back()[0]) continue;
          const double x = field(c, hX), y = field(c, hY), z = field(c, hZ);
          if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) high.push_back({t, x, y, z});
        }
      }
    }
  }

  out.samples.clear();
  if (high.size() >= 100) {
    // The airframe axis carries gravity on the pad (before t = 0, liftoff).
    double sum[3] = {0, 0, 0};
    int n = 0;
    for (const auto& r : high) {
      if (r[0] >= 0.0 || n >= 500) break;
      for (int k = 0; k < 3; ++k) sum[k] += r[static_cast<size_t>(k + 1)];
      ++n;
    }
    if (n < 10)
      for (size_t i = 0; i < 50 && i < high.size(); ++i, ++n)
        for (int k = 0; k < 3; ++k) sum[k] += high[i][static_cast<size_t>(k + 1)];
    int ax = 0;
    for (int k = 1; k < 3; ++k)
      if (std::fabs(sum[k]) > std::fabs(sum[ax])) ax = k;
    const double sign = sum[ax] < 0.0 ? -1.0 : 1.0;
    const int l1 = (ax + 1) % 3, l2 = (ax + 2) % 3;
    size_t j = 0;
    auto lowSample = [&](const BrLow& r, bool fresh) {
      LogSample s;
      s.t = r.t;
      s.pressure = r.p;
      s.state = r.state;
      s.baroFresh = fresh;
      return s;
    };
    // Barometer rows before the accelerometer starts.
    while (j < low.size() && low[j].t < high.front()[0]) out.samples.push_back(lowSample(low[j++], true));
    size_t lastJ = static_cast<size_t>(-1);
    for (const auto& r : high) {
      while (j + 1 < low.size() && low[j + 1].t <= r[0]) ++j;
      LogSample s = lowSample(low[j], j != lastJ);
      lastJ = j;
      s.t = r[0];
      s.accel = (sign * r[static_cast<size_t>(ax + 1)] - 1.0) * kG;  // gravity removed, 0 on the pad
      s.lat1 = r[static_cast<size_t>(l1 + 1)] * kG;
      s.lat2 = r[static_cast<size_t>(l2 + 1)] * kG;
      out.samples.push_back(s);
    }
    // Barometer rows after the accelerometer log ended (it stops sooner).
    for (++j; j < low.size(); ++j) out.samples.push_back(lowSample(low[j], true));
    out.hasLateral = true;
    out.accelSource = "3-axis";
  } else {
    // No high-rate file: the low-rate file's own axial accelerometer column,
    // if it has one, else nothing (no opening jolts reach the pod).
    double pad = 0.0;
    int n = 0;
    for (const auto& r : low)
      if (r.t < 0.0 && std::isfinite(r.axialG)) { pad += r.axialG; ++n; }
    pad = n > 0 ? pad / n : 1.0;
    bool any = false;
    for (const auto& r : low) {
      LogSample s;
      s.t = r.t;
      s.pressure = r.p;
      s.state = r.state;
      if (std::isfinite(r.axialG)) { s.accel = (r.axialG - pad) * kG; any = true; }
      if (std::isfinite(r.latG)) s.lat1 = r.latG * kG;
      out.samples.push_back(s);
    }
    out.accelSource = any ? "axial" : "none";
  }
  return true;
}

}  // namespace

std::string blueRavenHighRatePath(const std::string& lowRatePath) {
  const size_t slash = lowRatePath.find_last_of('/');
  const size_t base = slash == std::string::npos ? 0 : slash + 1;
  std::string p = lowRatePath;
  std::string lower = p;
  for (auto& ch : lower) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  // "low-rate", "low_rate", "lowrate" -> "high..." (keeping the separator and case).
  for (const char* sep : {"-", "_", ""}) {
    const std::string key = std::string("low") + sep + "rate";
    const size_t at = lower.rfind(key);
    if (at != std::string::npos && at >= base) {
      const bool upper = std::isupper(static_cast<unsigned char>(p[at])) != 0;
      return p.replace(at, 3, upper ? "High" : "high");
    }
  }
  // The last "LR" (any case) standing alone in the file name: "X_LR.csv",
  // "X LR_date.csv", "featherweight_lr.csv".
  auto alnum = [](char ch) { return std::isalnum(static_cast<unsigned char>(ch)) != 0; };
  for (size_t i = p.size(); i-- > base;) {
    if (i + 1 >= p.size() || lower[i] != 'l' || lower[i + 1] != 'r') continue;
    const bool leftOk = i == base || !alnum(p[i - 1]);
    const bool rightOk = i + 2 >= p.size() || !alnum(p[i + 2]);
    if (leftOk && rightOk) {
      p[i] = p[i] == 'l' ? 'h' : 'H';
      p[i + 1] = p[i + 1] == 'r' ? 'r' : 'R';
      return p;
    }
  }
  return "";
}

bool readFlightCsv(const std::string& path, FlightLog& out, std::string* err, bool* companion) {
  if (companion) *companion = false;
  std::ifstream f(path);
  if (!f) { if (err) *err = "cannot open"; return false; }
  out = FlightLog{};
  out.name = path;
  std::string line;
  std::vector<std::string> header, units;
  while (std::getline(f, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.size() >= 3 && static_cast<unsigned char>(line[0]) == 0xEF) line.erase(0, 3);  // UTF-8 BOM
    if (line.empty()) continue;
    if (line[0] == '#') {
      // AltosUI writes its header as a comment: "#version,serial,flight,...,time,...".
      auto cols = splitCsv(line.substr(1));
      if (cols.size() < 3 || !looksLikeHeader(cols)) continue;
      header = cols;
    } else {
      header = splitCsv(line);
    }
    break;
  }
  units.resize(header.size());
  for (size_t i = 0; i < header.size(); ++i) splitUnit(header[i], header[i], units[i]);
  // Featherweight Blue Raven: "Flight_Time_(s)" plus either the barometer
  // (low-rate file) or the 3-axis accelerometer alone (high-rate file).
  if (findCol(header, {"flight_time_"}) >= 0) {
    if (findCol(header, {"baro_press_"}) >= 0) return readBlueRaven(f, header, path, out, err);
    if (findCol(header, {"accel_x"}) >= 0) {
      if (companion) *companion = true;
      if (err) *err = "Blue Raven high-rate file: replayed with its low-rate file";
      return false;
    }
  }
  const int cT = findCol(header, {"time", "t", "time_s"});
  const int cA = findCol(header, {"acceleration", "accel", "accel_ms2"});
  const int cP = findCol(header, {"pressure", "pressure_pa"});
  // Eggtimer (Quantum, Quasar, TRS): "T,Alt,Veloc,FAlt,FVeloc,...,Drogue,Main",
  // Alt in feet above the pad, Drogue/Main nonzero once that channel fired.
  const bool eggtimer = findCol(header, {"falt"}) >= 0 && findCol(header, {"fveloc"}) >= 0;
  const int cH = eggtimer ? findCol(header, {"alt"}) : findCol(header, {"height", "altitude", "height_m"});
  const int cMainFlag = eggtimer ? findCol(header, {"main"}) : -1;
  const int cDrogueFlag = eggtimer ? findCol(header, {"drogue"}) : -1;
  const int cS = findCol(header, {"state_name", "state"});
  const int cM = findCol(header, {"main_t"});
  const int cE = findCol(header, {"events", "event"});  // Missile Works RRC3: "Drogue", "Main"
  const int cX = findCol(header, {"accel_x"});
  const int cY = findCol(header, {"accel_y"});
  const int cZ = findCol(header, {"accel_z"});
  const bool imu3 = cX >= 0 && cY >= 0 && cZ >= 0;
  // AltOS exports from boards with a radio carry a callsign column; EasyMini
  // has neither radio nor accelerometer.
  const bool altos = findCol(header, {"version"}) >= 0 && findCol(header, {"serial"}) >= 0;
  const bool hasCall = findCol(header, {"call"}) >= 0;
  std::vector<std::array<double, 3>> imu;  // raw 3-axis specific force, m/s^2
  // Barometer-only altimeters (RRC3 and the like) have no acceleration
  // column: accepted when they log pressure and deployment events.
  const bool baroOnly = cA < 0 && ((cP >= 0 && cE >= 0) || (eggtimer && cH >= 0 && cMainFlag >= 0));
  if (cT < 0 || (cA < 0 && !baroOnly) || (cP < 0 && cH < 0)) {
    if (err) *err = "missing time/acceleration/pressure-or-height columns";
    return false;
  }
  // Units, when the header states them; metric (AltosUI's export) otherwise.
  auto unitOf = [&](int c) { return c >= 0 ? units[static_cast<size_t>(c)] : std::string(); };
  const std::string uA = unitOf(cA), uP = unitOf(cP), uH = unitOf(cH);
  double kA = 1.0, kP = 0.0, kH = 1.0;  // kP 0: guess Pa vs hPa from the value
  if (uA == "g") kA = kG;
  else if (uA.rfind("ft", 0) == 0) kA = 0.3048;
  if (uP == "pa") kP = 1.0;
  else if (uP == "hpa" || uP == "mbar" || uP == "mb") kP = 100.0;
  else if (uP == "kpa") kP = 1000.0;
  else if (uP == "atm") kP = 101325.0;
  else if (uP == "psi") kP = 6894.757;
  if (uH == "ft" || uH == "feet" || eggtimer) kH = 0.3048;
  std::string prevState;
  std::string eventState = "ascent";
  while (std::getline(f, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    auto c = splitCsv(line);
    auto get = [&](int i) -> double {
      if (i < 0 || i >= static_cast<int>(c.size()) || c[i].empty()) return NAN;
      try { return std::stod(c[i]); } catch (...) { return NAN; }
    };
    LogSample s;
    s.t = get(cT);
    s.accel = baroOnly ? 0.0 : get(cA) * kA;
    const double p = cP >= 0 ? get(cP) : NAN;
    if (std::isfinite(p) && p > 0.0) {
      s.pressure = kP > 0.0 ? p * kP : (p < 2000.0 ? p * 100.0 : p);
    } else if (cH >= 0) {
      // No usable pressure (some exports leave it blank): rebuild it from height.
      s.pressure = isaPressure(1000.0 + get(cH) * kH);
    } else {
      s.pressure = NAN;
    }
    if (!std::isfinite(s.t) || !std::isfinite(s.accel) || !std::isfinite(s.pressure)) continue;
    if (cS >= 0 && cS < static_cast<int>(c.size())) s.state = lower(c[cS]);
    if (eggtimer && cMainFlag >= 0) {
      if (get(cMainFlag) > 0.0) eventState = "main";
      else if (eventState != "main" && get(cDrogueFlag) > 0.0) eventState = "drogue";
      s.state = eventState;
    }
    if (cS < 0 && cE >= 0 && cE < static_cast<int>(c.size())) {
      const std::string ev = lower(c[cE]);
      if (ev.find("main") != std::string::npos) eventState = "main";
      else if (eventState != "main" && (ev.find("drogue") != std::string::npos || ev.find("apogee") != std::string::npos))
        eventState = "drogue";
      s.state = eventState;
    }
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
    if (imu3) imu.push_back({get(cX), get(cY), get(cZ)});
  }
  // States or events logged: no "main" means no main.
  if ((cS >= 0 || cE >= 0 || cMainFlag >= 0) && cM < 0) out.hasTruth = true;
  if (out.samples.size() < 10) { if (err) *err = "too few samples"; return false; }
  out.accelSource = baroOnly ? "baro-derived" : (altos && !hasCall && !imu3) ? "baro-derived" : "axial";
  if (baroOnly) {
    // The pod has its own accelerometer; the nearest stand-in from a
    // barometer-only log is the barometric acceleration (what AltosUI writes
    // for EasyMini): enough for the boost, no opening jolts. Without it the
    // pod would only see the launch at 30 m and its ground reference would
    // drift up with the climb.
    auto& v = out.samples;
    const size_t n = v.size();
    std::vector<double> h(n), vel(n, 0.0);
    for (size_t i = 0; i < n; ++i) h[i] = 44330.0 * (1.0 - std::pow(v[i].pressure / 101325.0, 0.190263));
    auto slope = [&](const std::vector<double>& y, size_t i) {
      size_t a = i, b = i;
      while (a > 0 && v[i].t - v[a - 1].t <= 0.25) --a;
      while (b + 1 < n && v[b + 1].t - v[i].t <= 0.25) ++b;
      return b > a && v[b].t > v[a].t ? (y[b] - y[a]) / (v[b].t - v[a].t) : 0.0;
    };
    for (size_t i = 0; i < n; ++i) vel[i] = slope(h, i);
    for (size_t i = 0; i < n; ++i) v[i].accel = slope(vel, i);
  }
  if (imu3) {
    // The airframe axis is the IMU axis carrying gravity before launch; the
    // other two are cross-axis. The axial value itself keeps coming from the
    // "acceleration" column (the high-g sensor: the IMU saturates in boost).
    double sum[3] = {0, 0, 0};
    int n = 0;
    for (size_t i = 0; i < imu.size() && n < 50; ++i) {
      if (std::fabs(out.samples[i].accel) > 0.5 * kG) continue;
      bool ok = true;
      for (double v : imu[i]) ok = ok && std::isfinite(v);
      if (!ok) continue;
      for (int k = 0; k < 3; ++k) sum[k] += imu[i][k];
      ++n;
    }
    if (n >= 3) {
      int ax = 0;
      for (int k = 1; k < 3; ++k)
        if (std::fabs(sum[k]) > std::fabs(sum[ax])) ax = k;
      const int l1 = (ax + 1) % 3, l2 = (ax + 2) % 3;
      for (size_t i = 0; i < imu.size(); ++i) {
        const double a = imu[i][static_cast<size_t>(l1)], b = imu[i][static_cast<size_t>(l2)];
        out.samples[i].lat1 = std::isfinite(a) ? a : 0.0;
        out.samples[i].lat2 = std::isfinite(b) ? b : 0.0;
      }
      out.hasLateral = true;
      out.accelSource = "3-axis";
    }
  }
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

const char* outcomeName(MainOutcome o) {
  switch (o) {
    case MainOutcome::None: return "no-main";
    case MainOutcome::Canopy: return "canopy";
    case MainOutcome::FastOrFailed: return "fast-or-failed";
    case MainOutcome::Truncated: return "truncated";
    case MainOutcome::TooLow: return "too-low";
    case MainOutcome::Borderline: return "borderline";
  }
  return "?";
}

namespace {

double baroAlt(double p) { return 44330.0 * (1.0 - std::pow(p / 101325.0, 0.190263)); }

// Least-squares descent rate (m/s, positive down) of the log over [ta, tb].
bool descentOver(const std::vector<LogSample>& s, const std::vector<double>& h, double ta, double tb, double& rate) {
  auto lo = std::lower_bound(s.begin(), s.end(), ta, [](const LogSample& x, double t) { return x.t < t; });
  size_t i0 = static_cast<size_t>(lo - s.begin());
  double st = 0, sh = 0, stt = 0, sth = 0;
  int n = 0;
  for (size_t i = i0; i < s.size() && s[i].t <= tb; ++i) {
    const double t = s[i].t - ta;
    st += t; sh += h[i]; stt += t * t; sth += t * h[i];
    ++n;
  }
  if (n < 5) return false;
  const double den = n * stt - st * st;
  if (den <= 1e-9) return false;
  rate = -(n * sth - st * sh) / den;
  return true;
}

}  // namespace

MainJudgement judgeMain(const FlightLog& log, double bandLow, double bandHigh) {
  MainJudgement J;
  if (log.mainT < 0.0) return J;
  const auto& s = log.samples;
  std::vector<double> h(s.size());
  const double h0 = baroAlt(s.front().pressure);
  size_t iApo = 0;
  for (size_t i = 0; i < s.size(); ++i) {
    h[i] = baroAlt(s[i].pressure) - h0;
    if (h[i] > h[iApo]) iApo = i;
  }
  // Single deploy on a small motor: the host may enter "main" before apogee
  // (its main altitude is above apogee); the chute comes out at ejection.
  J.deployT = std::max(log.mainT, s[iApo].t);
  const double tEnd = log.landT >= 0.0 ? log.landT : s.back().t;
  auto hAt = [&](double t) {
    auto it = std::lower_bound(s.begin(), s.end(), t, [](const LogSample& x, double tt) { return x.t < tt; });
    return it == s.end() ? h.back() : h[static_cast<size_t>(it - s.begin())];
  };
  bool dual = false;
  for (const auto& x : s) dual = dual || x.state == "drogue";
  if (dual) {
    double r = 0.0;
    if (descentOver(s, h, log.mainT - 8.0, log.mainT - 1.0, r) && r <= bandHigh + 1.0) J.slowDrogue = true;
  }
  // Canopy: from some moment after deployment, the 3 s least-squares descent
  // rate is in the band and its median over the next 20 s (or to landing) is
  // in the band too. Smooth, non-causal and with hindsight: the detector gets
  // none of these luxuries.
  const double kWin = 3.0, kStep = 0.25, kHorizon = 20.0, kMinAlt = 30.0;
  std::vector<std::pair<double, double>> rates;  // (t, rate)
  for (double t = J.deployT; t + kWin <= tEnd; t += kStep) {
    double r = 0.0;
    if (descentOver(s, h, t, t + kWin, r)) rates.push_back({t, r});
  }
  bool sawLowCanopy = false;
  const double kEdge = 0.5;  // m/s: closer than this to a band edge is borderline
  for (size_t k = 0; k < rates.size(); ++k) {
    const double t = rates[k].first, r = rates[k].second;
    if (r < bandLow || r > bandHigh) continue;
    std::vector<double> win;
    for (size_t m = k; m < rates.size() && rates[m].first <= t + kHorizon; ++m) win.push_back(rates[m].second);
    if (win.size() < 12) continue;  // under 3 s of evidence
    std::nth_element(win.begin(), win.begin() + static_cast<long>(win.size() / 2), win.end());
    const double med = win[win.size() / 2];
    if (med < bandLow || med > bandHigh) continue;
    if (hAt(t) < kMinAlt) { sawLowCanopy = true; break; }
    if (med < bandLow + kEdge || med > bandHigh - kEdge) {
      J.outcome = MainOutcome::Borderline;  // judged where it first settled
      J.settleT = t + kWin;
      return J;
    }
    J.outcome = MainOutcome::Canopy;
    J.settleT = t + kWin;  // the window had to fill
    J.judgeEndT = tEnd;
    for (size_t i = 0; i < s.size(); ++i)
      if (s[i].t >= J.settleT && h[i] < kMinAlt) { J.judgeEndT = std::min(tEnd, s[i].t); break; }
    return J;
  }
  if (sawLowCanopy || h[iApo] < kMinAlt + 10.0) { J.outcome = MainOutcome::TooLow; return J; }
  const double lastH = h.back();
  if (log.landT < 0.0 && s.back().t - J.deployT < 12.0 && lastH > kMinAlt) J.outcome = MainOutcome::Truncated;
  else J.outcome = MainOutcome::FastOrFailed;
  return J;
}

ReplayResult replayLog(const FlightLog& log, const gr::Params& params0, const ReplayOptions& opt) {
  ReplayResult res;
  res.name = log.name;
  res.hasMain = log.mainT >= 0.0;
  res.mainT = log.mainT;
  if (log.samples.empty()) return res;
  const MainJudgement J = judgeMain(log, params0.det.bandLow, params0.det.bandHigh);
  res.outcome = J.outcome;
  res.settleT = J.settleT;
  res.deployT = J.deployT;
  res.slowDrogue = J.slowDrogue;
  {
    std::vector<double> dts;
    double last = -1.0;
    for (const auto& x : log.samples) {
      if (!x.baroFresh || x.t < J.deployT || J.deployT < 0.0) continue;
      if (last >= 0.0) dts.push_back(x.t - last);
      last = x.t;
    }
    if (!dts.empty()) {
      std::nth_element(dts.begin(), dts.begin() + static_cast<long>(dts.size() / 2), dts.end());
      res.descentDt = dts[dts.size() / 2];
    }
  }

  std::vector<LogSample> s = log.samples;
  std::vector<double> h(s.size());
  const double h0 = baroAlt(s.front().pressure);
  for (size_t i = 0; i < s.size(); ++i) h[i] = baroAlt(s[i].pressure) - h0;

  gr::Params params = params0;
  if (opt.mainAlt < 0.0) {
    // The builder sets the pod to the main altitude they gave their flight
    // computer. Dual deploy logs show it as the height at the main event.
    bool dual = false;
    for (const auto& x : s) dual = dual || x.state == "drogue";
    params.det.mainAlt = 0.0f;
    if (dual && log.mainT >= 0.0) {
      // Median height over the second before the main fired: the sample at
      // the event itself often carries the ejection charge's pressure spike.
      std::vector<double> before;
      for (size_t i = 0; i < s.size(); ++i)
        if (s[i].t >= log.mainT - 1.0 && s[i].t < log.mainT) before.push_back(h[i]);
      if (!before.empty()) {
        std::nth_element(before.begin(), before.begin() + static_cast<long>(before.size() / 2), before.end());
        params.det.mainAlt = static_cast<float>(std::max(0.0, before[before.size() / 2]));
      }
    }
  } else {
    params.det.mainAlt = static_cast<float>(opt.mainAlt);
  }
  res.mainAlt = params.det.mainAlt;
  gr::FlightCore core(params);

  // The pod is armed on the pad well before launch; logs start a moment
  // before liftoff (or mid-boost). Give it 10 s of quiet pad first.
  {
    LogSample pad = s.front();
    pad.accel = 0.0;
    pad.lat1 = pad.lat2 = 0.0;
    pad.baroFresh = true;
    pad.state = "pad";
    std::vector<LogSample> pre;
    const double tFirst = s.front().t;
    for (int i = 100; i >= 1; --i) {
      pad.t = tFirst - 0.1 * i;
      pre.push_back(pad);
    }
    s.insert(s.begin(), pre.begin(), pre.end());
    h.insert(h.begin(), pre.size(), h.front());
  }
  const double t0 = s.front().t;
  const double t1 = s.back().t;
  size_t idx = 0;
  bool started = false;
  double lastP = s.front().pressure;
  gr::Inputs in;
  if (opt.trace)
    std::fprintf(opt.trace, "t,host_state,log_h,stage,agl,climb,descent_mean,steady_s,accel_g,last_jolt\n");
  int traceDiv = 0;
  for (double tt = t0; tt <= t1; tt += 0.005) {
    // Every barometer reading passed since the last step counts once, even
    // when accelerometer rows (Blue Raven, 500 Hz) outnumber the steps.
    bool fresh = !started;
    started = true;
    while (idx + 1 < s.size() && s[idx + 1].t <= tt) {
      ++idx;
      if (s[idx].baroFresh) {
        fresh = true;
        lastP = s[idx].pressure;
      }
    }
    in = gr::Inputs{};
    in.tUs = static_cast<uint64_t>(std::llround((tt - t0 + 1.0) * 1e6));
    const double f = s[idx].accel + kG;  // back to specific force along the axis
    // Body frame: z down the airframe axis (the pod's "down" when hanging).
    in.imu.accel = gr::Vec3{static_cast<float>(s[idx].lat1), static_cast<float>(s[idx].lat2), static_cast<float>(-f)};
    in.imu.valid = true;
    if (fresh) {
      in.baroFresh = true;
      in.pressurePa = static_cast<float>(lastP);
    }
    in.armed = true;
    in.batteryV = 8.0f;
    core.update(in);
    const auto& st = core.status();
    if (opt.trace && (++traceDiv % 4 == 0)) {
      const auto& d = core.detector();
      float mean = NAN;
      if (!d.meanDescent(mean)) mean = NAN;
      std::fprintf(opt.trace, "%.3f,%s,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", tt, s[idx].state.c_str(), h[idx],
                   static_cast<int>(st.stage), st.agl, st.climbRate, mean, d.steadyFor(), st.accelG,
                   d.lastJoltTime());
    }
    if (!res.detected && st.stage == gr::FlightStage::Canopy) {
      res.detected = true;
      res.detectT = tt;
      res.detectAgl = st.agl;
      double r = 0.0;
      if (descentOver(s, h, tt - 1.5, tt + 1.5, r)) res.detectDescent = r;
    }
  }
  if (res.detected) {
    res.delay = res.hasMain ? res.detectT - res.deployT : -1.0;
    if (!res.hasMain || res.detectT < res.deployT - 0.5) res.falseTrigger = true;
    // A canopy confirmation while the log is falling at twice the top of the
    // band means the detector was fooled, main or no main.
    if (res.detectDescent > 2.0 * params0.det.bandHigh) res.implausible = true;
  }
  if (res.outcome == MainOutcome::Canopy && !res.falseTrigger) {
    const bool noAccel = log.accelSource == "baro-derived" || log.accelSource == "none";
    const double w = noAccel ? opt.windowNoAccel : opt.window;
    const bool late = !res.detected || res.detectT > res.settleT + w;
    if (late && J.judgeEndT < res.settleT + w) {
      // The log ends, or the canopy is below 30 m, before the deadline: no
      // fair verdict either way.
      res.outcome = J.judgeEndT < (log.landT >= 0.0 ? log.landT : log.samples.back().t) ? MainOutcome::TooLow
                                                                                         : MainOutcome::Truncated;
    } else if (late) {
      res.missed = true;
    }
  }
  return res;
}

}  // namespace sim
