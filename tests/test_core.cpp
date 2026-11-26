// Unit tests for the flight core.
#include <cmath>
#include <cstring>
#include <random>
#include <vector>

#include "gr/actuation.hpp"
#include "gr/estimators.hpp"
#include "gr/flight_core.hpp"
#include "gr/guidance.hpp"
#include "gr/phase_detector.hpp"
#include "gr/records.hpp"
#include "gr/ubx.hpp"
#include "mini_test.hpp"

using namespace gr;

TEST(wrap_angles) {
  CHECK_NEAR(wrapPi(3.5f * kPi), -0.5f * kPi, 1e-4);
  CHECK_NEAR(wrapPi(-3.5f * kPi), 0.5f * kPi, 1e-4);
  CHECK_NEAR(wrap2Pi(-0.5f), kTwoPi - 0.5f, 1e-4);
}

TEST(pressure_altitude_matches_isa) {
  const float p0 = 101325.0f;
  const float p100 = 101325.0f * std::pow(1.0f - 2.25577e-5f * 100.0f, 5.25588f);
  CHECK_NEAR(pressureToAltitude(p100, p0), 100.0, 0.5);
  CHECK_NEAR(pressureToAltitude(p0, p0), 0.0, 1e-3);
}

TEST(altitude_filter_tracks_climb_and_rejects_spike) {
  AltitudeFilter f;
  std::mt19937 g(1);
  std::normal_distribution<float> n(0.0f, 0.3f);
  float t = 0.0f;
  for (int i = 0; i < 1000; ++i) {  // 20 s at 50 Hz, 10 m/s climb
    for (int k = 0; k < 4; ++k) f.predict(0.005f);
    t += 0.02f;
    f.update(10.0f * t + n(g));
  }
  CHECK_NEAR(f.climbRate(), 10.0, 0.5);
  CHECK_NEAR(f.altitude(), 200.0, 1.5);
  const float before = f.altitude();
  for (int k = 0; k < 4; ++k) f.predict(0.005f);
  CHECK(!f.update(before + 60.0f));  // ejection-charge spike rejected
}

TEST(wind_estimator_recovers_wind_and_airspeed_from_circles) {
  WindEstimator w;
  w.reset(8.0f);
  const Vec2 wind{2.0f, -3.0f};
  const float va = 7.5f;
  std::mt19937 g(2);
  std::normal_distribution<float> n(0.0f, 0.1f);
  float t = 0.0f;
  for (int i = 0; i < 150; ++i) {
    const float psi = 0.35f * t;  // steady turn
    w.addSample(wind + unitFromBearing(psi) * va + Vec2{n(g), n(g)}, t);
    t += 0.1f;
  }
  w.solve(t);
  CHECK(w.valid());
  CHECK_NEAR(w.wind().x, 2.0, 0.3);
  CHECK_NEAR(w.wind().y, -3.0, 0.3);
  CHECK_NEAR(w.airspeed(), 7.5, 0.3);
}

TEST(wind_estimator_refuses_straight_flight) {
  WindEstimator w;
  w.reset(8.0f);
  for (int i = 0; i < 150; ++i) w.addSample(Vec2{6.0f, 1.0f}, 0.1f * i);
  w.solve(15.0f);
  CHECK(!w.valid());
}

TEST(ground_speed_along_handles_head_and_cross_wind) {
  CHECK_NEAR(groundSpeedAlong(Vec2{1, 0}, Vec2{-3, 0}, 8.0f), 5.0, 1e-4);
  CHECK_NEAR(groundSpeedAlong(Vec2{1, 0}, Vec2{3, 0}, 8.0f), 11.0, 1e-4);
  CHECK_NEAR(groundSpeedAlong(Vec2{1, 0}, Vec2{0, 9}, 8.0f), 0.0, 1e-4);  // crosswind beats airspeed
  CHECK_NEAR(groundSpeedAlong(Vec2{1, 0}, Vec2{0, 6}, 10.0f), 8.0, 1e-4);
}

TEST(height_needed_zero_wind) {
  GuidanceParams gp;
  CanopyParams cp;
  Guidance g(gp, cp);
  NavState n;
  n.pos = Vec2{70.0f, 0.0f};
  n.sinkRate = 3.5f;
  n.airspeed = 8.0f;
  CHECK_NEAR(g.heightNeeded(n, 0.0f), 3.5 * 70.0 / 8.0, 0.01);
  CHECK(g.heightNeeded(n, 0.5f) > g.heightNeeded(n, 0.0f));  // brakes steepen the glide
}

TEST(guidance_turns_toward_home) {
  GuidanceParams gp;
  CanopyParams cp;
  Guidance g(gp, cp);
  g.reset();
  NavState n;
  n.posValid = true;
  n.pos = Vec2{0.0f, 300.0f};  // 300 m east of home
  n.velGround = Vec2{8.0f, 0.0f};  // flying north
  n.groundSpeed = 8.0f;
  n.course = 0.0f;
  n.courseValid = true;
  n.agl = 300.0f;
  GuidanceCmd c = g.update(n, 0.005f);
  CHECK(c.mode == GuideMode::Homing);
  CHECK(c.turnRate < -0.3f);  // home is to the west: turn left (counter-clockwise)
}

TEST(brake_mixing_preserves_turn) {
  BrakeCmd b = mixBrakes(0.8f, 0.5f);
  CHECK_NEAR(b.right, 1.0, 1e-5);
  CHECK_NEAR(b.left, 0.5, 1e-5);
  b = mixBrakes(0.0f, -0.4f);
  CHECK_NEAR(b.left, 0.4, 1e-5);
  CHECK_NEAR(b.right, 0.0, 1e-5);
}

TEST(winch_detects_stall) {
  WinchParams p;
  WinchController w;
  w.configure(p, 0);
  bool stalled = false;
  for (int i = 0; i < 200 && !stalled; ++i) {
    w.update(0.8f, 0, 8.0f, 0.005f);  // encoder never moves
    stalled = w.stalled();
  }
  CHECK(stalled);
  CHECK_NEAR(w.update(0.8f, 0, 8.0f, 0.005f), 0.0, 1e-6);  // motor cut after stall
}

TEST(winch_duty_capped_for_6v_motor) {
  WinchParams p;
  WinchController w;
  w.configure(p, 0);
  const float d = w.update(1.0f, 0, 8.4f, 0.005f);
  CHECK_NEAR(d, 6.0 / 8.4, 1e-4);
}

namespace {
DetectorInput di(float t, float g, float agl, float climb) {
  DetectorInput d;
  d.t = t;
  d.armed = true;
  d.accelG = g;
  d.baroValid = true;
  d.agl = agl;
  d.climbRate = climb;
  return d;
}
}  // namespace

TEST(detector_ignores_bench_shaking) {
  DetectorParams p;
  PhaseDetector d(p);
  d.reset();
  float t = 0.0f;
  for (int i = 0; i < 4000; ++i) {  // 20 s of violent shaking, no altitude change
    const bool burst = (i % 200) < 40;
    d.update(di(t, burst ? 4.5f : 1.0f, 0.2f, 0.0f), 0.005f);
    t += 0.005f;
  }
  CHECK(d.stage() == FlightStage::Pad);
}

TEST(detector_drogue_never_confirms_then_main_does) {
  DetectorParams p;
  PhaseDetector d(p);
  d.reset();
  float t = 0.0f, h = 0.0f, v = 0.0f;
  const float dt = 0.005f;
  auto step = [&](float g, float climb) {
    v = climb;
    h += v * dt;
    d.update(di(t, g, h, v), dt);
    t += dt;
  };
  for (int i = 0; i < 400; ++i) step(1.0f, 0.0f);                  // pad
  for (int i = 0; i < 300; ++i) step(8.0f, 60.0f * i / 300.0f);   // boost
  for (int i = 0; i < 1200; ++i) step(0.3f, 60.0f - 60.0f * i / 1200.0f);  // coast to apogee
  CHECK(d.stage() == FlightStage::Ascent || d.stage() == FlightStage::Descent);
  step(30.0f, 0.0f);                                               // apogee charge jolt
  for (int i = 0; i < 3000; ++i) step(1.0f + 0.5f * std::sin(i * 0.1f), -20.0f);  // 15 s on drogue
  CHECK(d.stage() == FlightStage::Descent);
  step(8.0f, -10.0f);                                              // main opening jolt
  for (int i = 0; i < 400; ++i) step(1.0f, -5.0f);                 // 2 s at parafoil speed
  CHECK(d.stage() == FlightStage::Descent);
  for (int i = 0; i < 800; ++i) step(1.0f, -5.0f);                 // 6 s total
  CHECK(d.stage() == FlightStage::Canopy);
  CHECK(d.canopyTime() - d.lastJoltTime() < 5.0f);
}

namespace {
// Pad, boost and coast to an apogee near 225 m; returns the time.
struct DetFlight {
  PhaseDetector d;
  float t = 0.0f, h = 0.0f;
  explicit DetFlight(const DetectorParams& p) : d(p) { d.reset(); }
  void step(float g, float climb, float dt = 0.005f) {
    h += climb * dt;
    d.update(di(t, g, h, climb), dt);
    t += dt;
  }
  void toApogee() {
    for (int i = 0; i < 400; ++i) step(1.0f, 0.0f);
    for (int i = 0; i < 300; ++i) step(8.0f, 60.0f * i / 300.0f);
    for (int i = 0; i < 1200; ++i) step(0.3f, 60.0f - 60.0f * i / 1200.0f);
    step(30.0f, 0.0f);  // apogee charge
  }
};
}  // namespace

// Real logs: after a canopy opens the descent rate swings between ~1 and
// ~10 m/s for 10-15 s. The detector judges the mean, so it still confirms.
TEST(detector_confirms_through_canopy_swing) {
  DetectorParams p;
  DetFlight f(p);
  f.toApogee();
  for (int i = 0; i < 2000; ++i) f.step(1.0f, -20.0f);  // 10 s on a drogue
  CHECK(f.d.stage() == FlightStage::Descent);
  f.step(8.0f, -12.0f);  // main opening
  const float t0 = f.t;
  while (f.t - t0 < 12.0f && f.d.stage() == FlightStage::Descent) {
    const float swing = 4.5f * std::sin(2.0f * 3.14159265f * (f.t - t0) / 5.0f);
    f.step(1.0f + 0.3f * swing / 4.5f, -(5.5f + swing));
  }
  CHECK(f.d.stage() == FlightStage::Canopy);
  CHECK(f.d.canopyTime() - t0 >= 2.5f);  // never faster than the window allows
  CHECK(f.d.canopyTime() - t0 < 8.0f);
}

// A drogue falling at canopy speed can't be told from a canopy by descent
// rate. With canopy.mainAlt set, nothing above the main altitude counts.
TEST(detector_slow_drogue_needs_main_alt) {
  for (int withMainAlt = 0; withMainAlt < 2; ++withMainAlt) {
    DetectorParams p;
    p.mainAlt = withMainAlt ? 120.0f : 0.0f;
    DetFlight f(p);
    f.toApogee();
    bool confirmedHigh = false;
    while (f.h > 120.0f) {  // slow drogue: 8 m/s, swinging a little
      f.step(1.0f + 0.4f * std::sin(f.t * 3.0f), -8.0f);
      confirmedHigh = confirmedHigh || f.d.stage() == FlightStage::Canopy;
    }
    if (!withMainAlt) {
      CHECK(confirmedHigh);  // the documented limitation without the setting
      continue;
    }
    CHECK(!confirmedHigh);
    f.step(6.0f, -8.0f);  // main comes out at 120 m: the pod is yanked out
    const float t0 = f.t;
    while (f.t - t0 < 10.0f && f.d.stage() == FlightStage::Descent) f.step(1.0f, -5.0f);
    CHECK(f.d.stage() == FlightStage::Canopy);
    CHECK(f.d.canopyTime() - t0 < 6.0f);
  }
}

// Drogue at 15 m/s with the mean dipping as the rocket swings: never a canopy.
TEST(detector_ignores_swinging_fast_drogue) {
  DetectorParams p;
  DetFlight f(p);
  f.toApogee();
  for (int i = 0; i < 8000; ++i) {
    const float swing = 6.0f * std::sin(i * 0.005f * 2.0f * 3.14159265f / 4.0f);
    f.step(1.0f + 0.5f * std::fabs(swing) / 6.0f, -(15.0f + swing));
    if (f.h < 50.0f) break;
  }
  CHECK(f.d.stage() == FlightStage::Descent);
}

TEST(records_fixed_size_and_checksum) {
  LogRecord r;
  r.tMs = 1234;
  sealRecord(r);
  CHECK(recordValid(r));
  r.agl = 5.0f;
  CHECK(!recordValid(r));
}

TEST(attitude_filter_turn_rate_about_vertical) {
  AttitudeFilter a;
  a.reset();
  // Level, rotating about the vertical at 0.5 rad/s.
  const Vec3 acc{0.0f, 0.0f, -kGravity};
  for (int i = 0; i < 400; ++i) a.update(Vec3{0.0f, 0.0f, 0.5f}, acc, 0.005f);
  CHECK_NEAR(a.yawRate(Vec3{0.0f, 0.0f, 0.5f}), 0.5, 0.02);
  // Tilted 30 degrees in roll: gravity has a body-y component.
  AttitudeFilter b;
  b.reset();
  const float r = 30.0f * kDegToRad;
  const Vec3 acc2{0.0f, -kGravity * std::sin(r), -kGravity * std::cos(r)};
  const Vec3 w{0.0f, 0.5f * std::sin(r), 0.5f * std::cos(r)};  // vertical rotation seen in tilted body
  for (int i = 0; i < 2000; ++i) b.update(Vec3{}, acc2, 0.005f);
  CHECK_NEAR(b.yawRate(w), 0.5, 0.03);
}

TEST(flight_core_keeps_motors_off_until_canopy) {
  FlightCore core;
  Inputs in;
  in.imu.valid = true;
  in.imu.accel = Vec3{0.0f, 0.0f, -kGravity};
  in.armed = false;
  for (int i = 0; i < 100; ++i) {
    in.tUs = static_cast<uint64_t>(i) * 5000ULL;
    in.baroFresh = (i % 4) == 0;
    core.update(in);
  }
  CHECK(core.status().phase == Phase::Disarmed);
  CHECK(core.benchSetBrake(0, 0.5f));  // bench moves allowed while disarmed
  in.armed = true;
  for (int i = 100; i < 400; ++i) {
    in.tUs = static_cast<uint64_t>(i) * 5000ULL;
    in.baroFresh = (i % 4) == 0;
    const Outputs& o = core.update(in);
    CHECK(o.motorDuty[0] == 0.0f && o.motorDuty[1] == 0.0f);
  }
  CHECK(core.status().phase == Phase::Pad);
  CHECK(!core.benchSetBrake(0, 0.5f));  // and refused once armed
}

TEST(ubx_navpvt_roundtrip) {
  uint8_t payload[92] = {};
  auto put32 = [&](int off, int32_t v) {
    for (int k = 0; k < 4; ++k) payload[off + k] = static_cast<uint8_t>(static_cast<uint32_t>(v) >> (8 * k));
  };
  put32(0, 123456);           // iTOW
  payload[20] = 3;            // 3D fix
  payload[21] = 0x01;         // gnssFixOK
  payload[23] = 14;           // numSV
  put32(24, -1050000000);     // lon -105.0
  put32(28, 400000000);       // lat 40.0
  put32(36, 1523456);         // hMSL mm
  put32(40, 1800);            // hAcc mm
  put32(48, 2500);            // velN mm/s
  put32(52, -1200);
  put32(56, 3400);
  uint8_t frame[120];
  const size_t n = ubx::buildFrame(ubx::kClassNav, ubx::kIdNavPvt, payload, 92, frame, sizeof frame);
  CHECK(n == 100);
  ubx::Parser p;
  bool got = false;
  p.feed(0x00);  // junk before sync
  for (size_t i = 0; i < n; ++i) got = p.feed(frame[i]) || got;
  CHECK(got);
  CHECK(p.fix().fixType == 3 && p.fix().numSv == 14);
  CHECK(p.fix().latE7 == 400000000 && p.fix().lonE7 == -1050000000);
  CHECK_NEAR(p.fix().altMsl, 1523.456, 1e-3);
  CHECK_NEAR(p.fix().hAcc, 1.8, 1e-4);
  CHECK_NEAR(p.fix().velE, -1.2, 1e-4);
  frame[50] ^= 0x01;  // corrupt: must be rejected
  got = false;
  for (size_t i = 0; i < n; ++i) got = p.feed(frame[i]) || got;
  CHECK(!got);
  CHECK(p.framesBad() == 1);
}

TEST(ubx_valset_layout) {
  uint8_t out[256];
  const size_t n = ubx::buildPodConfig(115200, out, sizeof out);
  CHECK(n > 8);
  CHECK(out[2] == ubx::kClassCfg && out[3] == ubx::kIdCfgValset);
  const size_t len = out[4] | (out[5] << 8);
  CHECK(len + 8 == n);
  CHECK(out[6] == 0x00 && out[7] == ubx::kLayerRam);
  // First key: UART1 baud (4-byte value) = 115200
  const uint32_t key = out[10] | (out[11] << 8) | (out[12] << 16) | (static_cast<uint32_t>(out[13]) << 24);
  CHECK(key == ubx::kCfgUart1Baudrate);
  const uint32_t baud = out[14] | (out[15] << 8) | (out[16] << 16) | (static_cast<uint32_t>(out[17]) << 24);
  CHECK(baud == 115200);
  CHECK(ubx::keyValueSize(ubx::kCfgRateMeas) == 2);
  CHECK(ubx::keyValueSize(ubx::kCfgUart1OutNmea) == 1);
}

TEST(winch_detects_wrong_way_encoder) {
  WinchParams p;
  WinchController w;
  w.configure(p, 0);
  int32_t counts = 0;
  bool fault = false;
  for (int i = 0; i < 200 && !fault; ++i) {
    const float duty = w.update(0.5f, counts, 8.0f, 0.005f);
    counts -= static_cast<int32_t>(duty * 120.0f * p.countsPerMm * 0.005f);  // moves opposite to the drive
    fault = w.wrongWay();
  }
  CHECK(fault);
  CHECK(w.stalled());
}

// Found by the TLA+ model (DisarmedMeansNoLaunchPending): after a flight,
// re-arming without a power cycle must not inherit the old launch candidate.
// Before the fix, re-arming 12 m above the old reference jumped straight to
// Ascent and the pod could never learn a new home point.
TEST(detector_rearm_after_flight_starts_fresh) {
  DetectorParams p;
  PhaseDetector d(p);
  d.reset();
  float t = 0.0f, h = 0.0f;
  const float dt = 0.005f;
  auto step = [&](bool armed, float g, float agl, float climb) {
    DetectorInput in = di(t, g, agl, climb);
    in.armed = armed;
    d.update(in, dt);
    t += dt;
  };
  for (int i = 0; i < 200; ++i) step(true, 1.0f, 0.0f, 0.0f);                    // pad
  for (int i = 0; i < 300; ++i) { h += 0.2f; step(true, 8.0f, h, 40.0f); }     // boost
  CHECK(d.stage() == FlightStage::Ascent);
  for (int i = 0; i < 400; ++i) { h += 0.05f; step(true, 0.3f, h, 10.0f); }    // coast
  for (int i = 0; i < 400; ++i) { h -= 0.1f; step(true, 1.0f, h, -20.0f); }    // apogee, fast fall
  for (int i = 0; i < 1000; ++i) step(true, 1.0f, 0.0f, 0.0f);                  // on the ground
  CHECK(d.stage() == FlightStage::Landed);
  for (int i = 0; i < 40; ++i) step(false, 1.0f, 0.0f, 0.0f);                   // magnet on
  CHECK(d.stage() == FlightStage::Disarmed);
  CHECK(!d.launchCandidate());
  for (int i = 0; i < 400; ++i) step(true, 1.0f, 12.0f, 0.0f);                  // re-armed 12 m higher
  CHECK(d.stage() == FlightStage::Pad);
}

// ---- HIL protocol (gate 4)
#include "gr/hil.hpp"

TEST(hil_frames_roundtrip_and_resync) {
  Inputs in;
  in.tUs = 123456789ULL;
  in.imu.accel = Vec3{0.1f, -0.2f, -9.80665f};
  in.imu.gyro = Vec3{0.012f, -0.5f, 1.25f};
  in.baroFresh = true;
  in.pressurePa = 84123.5f;
  in.armed = true;
  hil::SensorFrame f = hil::toSensorFrame(in);
  uint8_t buf[64];
  const size_t n = hil::encode(hil::kTypeSensor, &f, sizeof f, buf, sizeof buf);
  CHECK(n == sizeof f + hil::kOverhead);
  hil::Decoder d;
  const uint8_t noise[] = {0x00, 0xA5, 0x53, 0xFF, 0x13};  // junk, including a fake sync
  for (uint8_t b : noise) d.feed(b);
  bool got = false;
  for (size_t i = 0; i < n; ++i) got = d.feed(buf[i]) || got;
  CHECK(got && d.type() == hil::kTypeSensor && d.length() == sizeof f);
  hil::SensorFrame back;
  std::memcpy(&back, d.payload(), sizeof back);
  Inputs out;
  hil::applySensorFrame(back, out);
  CHECK_NEAR(out.imu.accel.z, -9.80665, 0.01);
  CHECK_NEAR(out.imu.gyro.z, 1.25, 0.001);
  CHECK_NEAR(out.pressurePa, 84123.5, 1e-3);
  CHECK(out.baroFresh && out.armed);
  buf[7] ^= 0x40;  // corrupt a payload byte
  got = false;
  for (size_t i = 0; i < n; ++i) got = d.feed(buf[i]) || got;
  CHECK(!got);
}

namespace {
struct FakeIo : hil::BoardIo {
  std::vector<uint8_t> sent;
  bool interlock = true;
  bool lastEnabled = true;
  void readEncoders(int32_t e[2]) override { e[0] = e[1] = 0; }
  float batteryVolts() override { return 8.0f; }
  bool interlockArmed() override { return interlock; }
  void writeMotors(float, float, bool en) override { lastEnabled = en; }
  uint32_t micros() override { return t += 7; }
  void send(const uint8_t* p, size_t n) override { sent.insert(sent.end(), p, p + n); }
  uint32_t t = 0;
};
}  // namespace

TEST(hil_board_answers_every_frame_and_honours_interlock) {
  FlightCore core;
  hil::HilBoard board(core);
  FakeIo io;
  io.interlock = false;  // magnet on the board: motors must stay off
  Inputs in;
  in.imu.valid = true;
  in.imu.accel = Vec3{0.0f, 0.0f, -kGravity};
  in.pressurePa = 90000.0f;
  for (int i = 0; i < 400; ++i) {
    in.tUs = 1000000ULL + static_cast<uint64_t>(i) * 5000ULL;
    in.baroFresh = (i % 4) == 0;
    const hil::SensorFrame f = hil::toSensorFrame(in);
    uint8_t buf[64];
    const size_t n = hil::encode(hil::kTypeSensor, &f, sizeof f, buf, sizeof buf);
    board.onBytes(buf, n, io);
  }
  CHECK(board.steps() == 400);
  CHECK(io.sent.size() == 400 * (sizeof(hil::ActuatorFrame) + hil::kOverhead));
  CHECK(!io.lastEnabled);
  hil::Decoder d;
  int replies = 0;
  bool interlockFlag = false;
  for (uint8_t b : io.sent)
    if (d.feed(b) && d.type() == hil::kTypeActuator) {
      ++replies;
      hil::ActuatorFrame a;
      std::memcpy(&a, d.payload(), sizeof a);
      interlockFlag = (a.flags & hil::kActInterlock) != 0;
    }
  CHECK(replies == 400);
  CHECK(interlockFlag);
}
