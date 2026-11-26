// grhil: gate 4, hardware in the loop.
//
// The simulator flies the flight; the pod board makes every decision and its
// REAL winches move. Each 5 ms the PC sends simulated IMU, barometer and GPS
// readings; the board answers with its motor commands and real encoder
// positions, which become the simulated canopy's brake positions.
//
//   grhil --port /dev/ttyUSB0 [--flights 3]       real Heltec V3 (console command 'hil')
//   grhil --loopback [--flights 20]               stand-in board on this PC (same core code)
//
// Options: --seed S, --out FILE.json, --lockstep (wait for each reply),
//          --realtime (pace at 200 Hz; default with --port)
//
// Gate 4 passes when: every frame is answered, the board never needs more
// than 5 ms for a frame, every flight lands with no steering before the
// canopy opened, and the canopy is confirmed on every flight.

#include <fcntl.h>
#include <pty.h>
#include <poll.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "gr/flight_core.hpp"
#include "gr/hil.hpp"
#include "sim/scenario.hpp"
#include "sim/sensors.hpp"
#include "sim/world.hpp"

using namespace sim;
using Clock = std::chrono::steady_clock;

namespace {

uint32_t nowUs() {
  return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count());
}

bool writeAll(int fd, const uint8_t* p, size_t n) {
  while (n) {
    const ssize_t w = ::write(fd, p, n);
    if (w < 0) { if (errno == EINTR || errno == EAGAIN) continue; return false; }
    p += w;
    n -= static_cast<size_t>(w);
  }
  return true;
}

// ------------------------------------------------------------- serial port

int openSerial(const std::string& dev, speed_t baud) {
  const int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) { std::perror(dev.c_str()); return -1; }
  termios t{};
  tcgetattr(fd, &t);
  cfmakeraw(&t);
  cfsetispeed(&t, baud);
  cfsetospeed(&t, baud);
  t.c_cflag |= CLOCAL | CREAD;
  t.c_cc[VMIN] = 0;
  t.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSANOW, &t);
  tcflush(fd, TCIOFLUSH);
  return fd;
}

// Ask the board's console to switch into HIL mode.
bool enterHilMode(int fd) {
  const char* cmd = "\nhil\n";
  writeAll(fd, reinterpret_cast<const uint8_t*>(cmd), std::strlen(cmd));
  std::string seen;
  const auto t0 = Clock::now();
  while (Clock::now() - t0 < std::chrono::seconds(3)) {
    char c;
    const ssize_t n = ::read(fd, &c, 1);
    if (n == 1) {
      seen += c;
      if (seen.find("HIL READY") != std::string::npos) return true;
      if (seen.find("refused") != std::string::npos) {
        std::fprintf(stderr, "board refused HIL mode: put the magnet on (disarm) and retry\n");
        return false;
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  std::fprintf(stderr, "no 'HIL READY' from the board. Console output was:\n%s\n", seen.c_str());
  return false;
}

// ------------------------------------------------- stand-in board (loopback)

// Plays the hardware: two winch motors with encoders pulling against a bench
// spring, a battery and the physical magnet (removed). The flight logic is the
// real gr::hil::HilBoard + gr::FlightCore, exactly as on the ESP32.
class StandInBoard : public gr::hil::BoardIo {
 public:
  explicit StandInBoard(int fd) : fd_(fd), board_(core_) { scen_ = Scenario(); }
  void run(std::atomic<bool>& stop) {
    uint8_t buf[512];
    while (!stop) {
      pollfd p{fd_, POLLIN, 0};
      if (::poll(&p, 1, 50) <= 0) continue;
      const ssize_t n = ::read(fd_, buf, sizeof buf);
      if (n <= 0) break;
      board_.onBytes(buf, static_cast<size_t>(n), *this);
    }
  }
  void readEncoders(int32_t enc[2]) override {
    enc[0] = winch_[0].counts(scen_);
    enc[1] = winch_[1].counts(scen_);
  }
  float batteryVolts() override { return 8.1f; }
  bool interlockArmed() override { return true; }
  void writeMotors(float l, float r, bool enabled) override {
    const double d[2] = {enabled ? l : 0.0, enabled ? r : 0.0};
    for (int ch = 0; ch < 2; ++ch) {
      const double spring = 0.3 + 0.03 * std::max(0.0, winch_[ch].pos);  // ~3.9 N at 120 mm
      winch_[ch].step(d[ch], 8.1, spring, 0.005, false, scen_);
    }
  }
  uint32_t micros() override { return nowUs(); }
  void send(const uint8_t* data, size_t len) override { writeAll(fd_, data, len); }
  const gr::hil::HilBoard& board() const { return board_; }

 private:
  int fd_;
  gr::FlightCore core_;
  gr::hil::HilBoard board_;
  Scenario scen_;
  WinchTruth winch_[2];
};

// ------------------------------------------------------------- host side

struct FlightReport {
  uint64_t seed = 0;
  bool landed = false;
  double miss = 0.0;
  bool canopyConfirmed = false;
  bool navBeforeCanopy = false;
  int finalPhase = 0;
  unsigned faults = 0;
  std::string phases;
  uint32_t sent = 0, received = 0;
  uint32_t maxLoopUs = 0;
  uint32_t overruns = 0;
  bool interlockSeen = false;
  bool boardLanded = false;    // the board itself detected the touchdown
  double landedDetectS = -1.0;
  double maxLineSpeed = 0.0;  // mm/s measured from the real encoders
};

class Link {
 public:
  explicit Link(int fd) : fd_(fd) {}
  void send(uint8_t type, const void* p, uint8_t len) {
    uint8_t buf[gr::hil::kOverhead + gr::hil::kMaxPayload];
    const size_t n = gr::hil::encode(type, p, len, buf, sizeof buf);
    writeAll(fd_, buf, n);
  }
  // Read whatever is available (up to `waitMs`), return actuator frames.
  void poll(int waitMs, std::vector<gr::hil::ActuatorFrame>& out) {
    pollfd p{fd_, POLLIN, 0};
    if (::poll(&p, 1, waitMs) <= 0) return;
    uint8_t buf[1024];
    const ssize_t n = ::read(fd_, buf, sizeof buf);
    for (ssize_t i = 0; i < n; ++i) {
      if (!dec_.feed(buf[i]) || dec_.type() != gr::hil::kTypeActuator || dec_.length() != sizeof(gr::hil::ActuatorFrame))
        continue;
      gr::hil::ActuatorFrame a;
      std::memcpy(&a, dec_.payload(), sizeof a);
      out.push_back(a);
    }
  }
  uint32_t bad() const { return dec_.framesBad(); }

 private:
  int fd_;
  gr::hil::Decoder dec_;
};

FlightReport flyOne(Link& link, const Scenario& s, bool lockstep, bool realtime, uint64_t& simUs) {
  FlightReport r;
  r.seed = s.seed;
  World w(s);
  SensorSuite sens(s);
  const double dt = 0.005;
  const double cpmm = 50.13;
  double duty[2] = {0.0, 0.0};
  gr::Inputs in;
  int32_t enc[2] = {0, 0};
  double lastPosMm[2] = {0.0, 0.0};
  bool havePos = false;
  int boardPhase = -1;
  double landedAt = -1.0;
  auto next = Clock::now();
  std::vector<gr::hil::ActuatorFrame> got;
  while (w.t() < 1500.0) {
    // The real winch positions are the canopy's brake positions.
    w.overrideBrakes(enc[0] / cpmm / s.brakeTravelMm, enc[1] / cpmm / s.brakeTravelMm);
    w.step(dt, duty, sens.batteryV());
    sens.sample(w, duty, in);
    if (w.stage() == TruthStage::Landed && landedAt < 0.0) landedAt = w.t();
    const double since = landedAt < 0.0 ? -1.0 : w.t() - landedAt;
    // After touchdown: wait for the board to detect the landing (it must), then
    // put the magnet on so the next flight starts from a fresh arming.
    const bool boardLanded = boardPhase == static_cast<int>(gr::Phase::Landed);
    if (boardLanded && !r.boardLanded) { r.boardLanded = true; r.landedDetectS = since; }
    if (r.boardLanded || since >= 30.0) in.armed = false;
    simUs += 5000;
    in.tUs = simUs;
    if (in.gps.fresh) {
      const gr::hil::GpsFrame g = gr::hil::toGpsFrame(in.gps);
      link.send(gr::hil::kTypeGps, &g, sizeof g);
    }
    const gr::hil::SensorFrame f = gr::hil::toSensorFrame(in);
    link.send(gr::hil::kTypeSensor, &f, sizeof f);
    ++r.sent;

    got.clear();
    if (lockstep) {
      const auto t0 = Clock::now();
      while (got.empty() && Clock::now() - t0 < std::chrono::milliseconds(250)) link.poll(20, got);
    } else {
      next += std::chrono::microseconds(5000);
      if (realtime) std::this_thread::sleep_until(next);
      link.poll(0, got);
    }
    for (const auto& a : got) {
      ++r.received;
      enc[0] = a.encoder[0];
      enc[1] = a.encoder[1];
      duty[0] = a.duty[0] * 1e-4;
      duty[1] = a.duty[1] * 1e-4;
      r.maxLoopUs = std::max<uint32_t>(r.maxLoopUs, a.loopUs);
      r.overruns = a.overruns;
      if (a.flags & gr::hil::kActInterlock) r.interlockSeen = true;
      const bool steering = (a.flags & gr::hil::kActSteering) != 0;
      if (steering && w.openT() < 0.0) r.navBeforeCanopy = true;
      if (static_cast<gr::FlightStage>(a.stage) == gr::FlightStage::Canopy) r.canopyConfirmed = true;
      if (a.phase != r.finalPhase || r.phases.empty()) { r.phases += gr::phaseName(static_cast<gr::Phase>(a.phase)); r.phases += ' '; }
      r.finalPhase = a.phase;
      r.faults = a.faults;
      boardPhase = a.phase;
      for (int ch = 0; ch < 2; ++ch) {
        const double mm = a.encoder[ch] / cpmm;
        if (havePos && std::fabs(duty[ch]) > 0.6) r.maxLineSpeed = std::max(r.maxLineSpeed, std::fabs(mm - lastPosMm[ch]) / dt);
        lastPosMm[ch] = mm;
      }
      havePos = true;
    }
    if (since >= 0.0 && boardPhase == static_cast<int>(gr::Phase::Disarmed)) break;  // next flight
    if (since >= 40.0) break;
  }
  r.landed = landedAt >= 0.0;
  r.miss = std::hypot(w.landPos().x, w.landPos().y);
  return r;
}

}  // namespace

// Test aid: act as a board console on a pseudo-terminal (answers 'hil' with
// 'HIL READY', then runs the stand-in board), so the serial path of this tool
// can be exercised without hardware:  grhil --serve-pty  &  grhil --port <printed path>
int servePty() {
  int master = -1, slave = -1;
  char name[128];
  if (::openpty(&master, &slave, name, nullptr, nullptr) != 0) { std::perror("openpty"); return 1; }
  termios t{};
  tcgetattr(slave, &t);
  cfmakeraw(&t);
  tcsetattr(slave, TCSANOW, &t);
  std::printf("%s\n", name);
  std::fflush(stdout);
  std::string line;
  for (;;) {
    char c;
    if (::read(master, &c, 1) != 1) return 1;
    if (c == '\n') {
      if (line == "hil") break;
      line.clear();
    } else if (c != '\r') {
      line += c;
    }
  }
  const char* ready = "HIL READY\r\n> ";
  writeAll(master, reinterpret_cast<const uint8_t*>(ready), std::strlen(ready));
  std::atomic<bool> stop{false};
  StandInBoard board(master);
  board.run(stop);
  return 0;
}

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == "--serve-pty") return servePty();
  std::string port, out;
  bool loopback = false, lockstep = false, realtime = false;
  int flights = -1;
  uint64_t seed = 1;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--port" && i + 1 < argc) port = argv[++i];
    else if (a == "--loopback") loopback = true;
    else if (a == "--lockstep") lockstep = true;
    else if (a == "--realtime") realtime = true;
    else if (a == "--flights" && i + 1 < argc) flights = std::atoi(argv[++i]);
    else if (a == "--seed" && i + 1 < argc) seed = static_cast<uint64_t>(std::atoll(argv[++i]));
    else if (a == "--out" && i + 1 < argc) out = argv[++i];
  }
  if (port.empty() && !loopback) {
    std::fprintf(stderr, "usage: grhil --port /dev/ttyUSB0 | --loopback  [--flights N] [--seed S] [--out FILE]\n");
    return 2;
  }

  int fd = -1;
  std::atomic<bool> stop{false};
  std::thread boardThread;
  StandInBoard* standIn = nullptr;
  if (loopback) {
    int sv[2];
    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sv) != 0) { std::perror("socketpair"); return 1; }
    fd = sv[0];
    standIn = new StandInBoard(sv[1]);
    boardThread = std::thread([&]() { standIn->run(stop); });
    if (!realtime) lockstep = true;
    if (flights < 0) flights = 20;
  } else {
    fd = openSerial(port, B115200);
    if (fd < 0 || !enterHilMode(fd)) return 1;
    realtime = true;
    if (flights < 0) flights = 3;
    std::printf("Board in HIL mode. Remove the board's magnet to let the winches move.\n");
  }

  Link link(fd);
  std::vector<FlightReport> reports;
  uint64_t simUs = 1000000;
  ScenarioOptions o;
  for (int i = 0; i < flights; ++i) {
    Scenario s = makeScenario(seed + static_cast<uint64_t>(i), o);
    s.launchTime = 20.0;
    const FlightReport r = flyOne(link, s, lockstep, realtime, simUs);
    reports.push_back(r);
    std::printf("flight %2d seed %-5llu landed %d  miss %6.1f m  canopy %d  replies %u/%u  board max %u us  overruns %u%s\n",
                i + 1, static_cast<unsigned long long>(r.seed), r.landed, r.miss, r.canopyConfirmed, r.received, r.sent,
                r.maxLoopUs, r.overruns, r.navBeforeCanopy ? "  STEERED BEFORE CANOPY" : "");
    if (std::getenv("GRHIL_VERBOSE")) std::printf("      phases: %s faults 0x%x\n", r.phases.c_str(), r.faults);
    std::fflush(stdout);
  }

  // Gate 4 verdict.
  uint32_t sent = 0, recv = 0, maxLoop = 0, overruns = 0;
  int landed = 0, within30 = 0, confirmed = 0, early = 0, boardLanded = 0;
  double maxDetect = 0.0;
  bool interlock = false;
  double lineSpeed = 0.0;
  std::vector<double> misses;
  for (const auto& r : reports) {
    sent += r.sent; recv += r.received; maxLoop = std::max(maxLoop, r.maxLoopUs); overruns = std::max(overruns, r.overruns);
    landed += r.landed; within30 += r.miss <= 30.0; confirmed += r.canopyConfirmed; early += r.navBeforeCanopy;
    interlock |= r.interlockSeen; lineSpeed = std::max(lineSpeed, r.maxLineSpeed);
    boardLanded += r.boardLanded; maxDetect = std::max(maxDetect, r.landedDetectS);
    misses.push_back(r.miss);
  }
  std::sort(misses.begin(), misses.end());
  const double median = misses.empty() ? 0.0 : misses[misses.size() / 2];
  const double replyRate = sent ? static_cast<double>(recv) / sent : 0.0;
  std::vector<std::string> fails;
  if (replyRate < (lockstep ? 1.0 : 0.999)) fails.push_back("board missed frames");
  if (maxLoop >= 5000 || overruns > 0) fails.push_back("board could not hold 200 Hz");
  if (landed != static_cast<int>(reports.size())) fails.push_back("not every flight landed");
  if (early) fails.push_back("steering before the canopy opened");
  if (confirmed != static_cast<int>(reports.size())) fails.push_back("canopy not confirmed on every flight");
  if (boardLanded != static_cast<int>(reports.size())) fails.push_back("board did not detect every landing");
  if (interlock) fails.push_back("board magnet was on (winches held still): remove it and rerun");
  std::printf("== Gate 4: hardware in the loop (%s) ==\n", loopback ? "stand-in board" : port.c_str());
  std::printf("  frames answered %u/%u, link errors %u, board max %u us per frame, overruns %u\n", recv, sent, link.bad(),
              maxLoop, overruns);
  std::printf("  flights %zu, landed %d (board detected %d, within %.1f s), within 30 m %d, median miss %.1f m\n",
              reports.size(), landed, boardLanded, maxDetect, within30, median);
  std::printf("  max measured line speed %.0f mm/s\n", lineSpeed);
  for (const auto& f : fails) std::printf("  FAIL: %s\n", f.c_str());
  std::printf("GATE 4 %s\n", fails.empty() ? "PASS" : "FAIL");

  if (!out.empty()) {
    FILE* f = std::fopen(out.c_str(), "w");
    if (f) {
      std::fprintf(f, "{\"gate\":4,\"pass\":%s,\"board\":\"%s\",\"frames_sent\":%u,\"frames_answered\":%u,"
                      "\"board_max_loop_us\":%u,\"overruns\":%u,\"flights\":%zu,\"landed\":%d,\"within_30m\":%d,"
                      "\"median_miss_m\":%.2f,\"max_line_speed_mm_s\":%.1f}\n",
                   fails.empty() ? "true" : "false", loopback ? "stand-in" : port.c_str(), sent, recv, maxLoop,
                   overruns, reports.size(), landed, within30, median, lineSpeed);
      std::fclose(f);
    }
  }
  stop = true;
  if (boardThread.joinable()) {
    ::shutdown(fd, SHUT_RDWR);
    boardThread.join();
  }
  delete standIn;
  ::close(fd);
  return fails.empty() ? 0 : 1;
}
