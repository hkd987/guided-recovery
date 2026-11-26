// Tests for the simulator itself, plus end-to-end smoke runs.
#include <algorithm>
#include <cstdio>
#include <memory>

#include "gr/estimators.hpp"
#include "gr/flight_core.hpp"
#include "mini_test.hpp"
#include "sim/logs.hpp"
#include "sim/runners.hpp"
#include "sim/scenario.hpp"
#include "sim/sensors.hpp"
#include "sim/world.hpp"

using namespace sim;

TEST(geo_roundtrip_matches_firmware_projection) {
  gr::GeoOrigin o;
  o.set(40.0, -105.0);
  double lat, lon;
  localToLatLon(40.0, -105.0, 350.0, -820.0, lat, lon);
  gr::Vec2 p = o.toLocal(lat, lon);
  CHECK_NEAR(p.x, 350.0, 0.05);
  CHECK_NEAR(p.y, -820.0, 0.05);
}

TEST(winch_model_speed_and_load) {
  Scenario s;
  WinchTruth w;
  for (int i = 0; i < 200; ++i) w.step(6.0 / 8.0, 8.0 / 0.95, 0.0, 0.005, false, s);
  CHECK_NEAR(w.speed, s.winchNoLoadMmS, 3.0);
  WinchTruth h;
  for (int i = 0; i < 200; ++i) h.step(0.0, 8.0, 1.0, 0.005, false, s);
  CHECK_NEAR(h.pos, 0.0, 1e-9);  // unpowered and lightly loaded: holds
}

TEST(rocket_flies_deploys_and_lands) {
  ScenarioOptions o;
  Scenario s = makeScenario(7, o);
  World w(s);
  const double duty[2] = {0, 0};
  while (w.t() < 900 && w.stage() != TruthStage::Landed) w.step(0.005, duty, 8.0);
  CHECK(w.stage() == TruthStage::Landed);
  CHECK(w.apogeeH() > 80.0);
  CHECK(w.deployT() > w.launchT());
  CHECK(w.openT() > w.deployT());
}

TEST(sitl_nominal_run_is_clean) {
  ScenarioOptions o;
  RunOptions ro;
  for (uint64_t seed : {3ULL, 11ULL, 42ULL}) {
    RunResult r = runSitl(makeScenario(seed, o), ro);
    CHECK(r.landed);
    CHECK(r.canopyConfirmed);
    CHECK(!r.navBeforeCanopy);
    CHECK(!r.crashed);
  }
}

TEST(sitl_gps_loss_ends_in_spiral) {
  ScenarioOptions o;
  o.fault = FaultType::GpsLoss;
  RunOptions ro;
  int exercised = 0;
  for (uint64_t seed = 100; seed < 106; ++seed) {
    RunResult r = runSitl(makeScenario(seed, o), ro);
    if (!r.faultExercised) continue;
    ++exercised;
    CHECK(r.expectationMet);
  }
  CHECK(exercised > 0);
}

// AltosUI writes its CSV header as a comment line ("#version,serial,...").
TEST(reads_altosui_csv_header_and_units) {
  const char* path = "/tmp/gr_test_altos.csv";
  {
    std::FILE* f = std::fopen(path, "w");
    std::fprintf(f, "#version,serial,flight,call,time,state,state_name,acceleration,pressure,altitude,height,"
                    "accel_x,accel_y,accel_z\n");
    for (int i = 0; i < 40; ++i) {
      const double t = -0.3 + 0.1 * i;
      const char* st = t < 1.0 ? "boost" : t < 2.5 ? "drogue" : "main";
      std::fprintf(f, "6, 1234, 2,N0CALL, %6.2f, 3, %8s, %6.2f, %9.2f, 200.0, %6.2f, -9.70, 0.20, 0.30\n", t, st,
                   0.0, 98000.0 - 10.0 * i, 0.8 * i);
    }
    std::fclose(f);
  }
  FlightLog log;
  std::string err;
  CHECK(readFlightCsv(path, log, &err));
  CHECK(log.samples.size() == 40u);
  CHECK_NEAR(log.mainT, 2.5, 0.051);
  CHECK(log.accelSource == "3-axis");
  CHECK_NEAR(log.samples[5].lat1, 0.2, 1e-9);  // x carries gravity: y, z are cross-axis
  CHECK_NEAR(log.samples[5].pressure, 97950.0, 1e-6);

  // Units in the header, no callsign column and no IMU (EasyMini style).
  {
    std::FILE* f = std::fopen(path, "w");
    std::fprintf(f, "# Altus Metrum flight log\n#version,serial,flight,time (s),state,state_name,"
                    "acceleration (m/s^2),pressure (hPa),height (ft)\n");
    for (int i = 0; i < 20; ++i) std::fprintf(f, "6,1,1,%.2f,3,boost,1.0,980.0,%d\n", 0.1 * i, i);
    std::fclose(f);
  }
  CHECK(readFlightCsv(path, log, &err));
  CHECK(log.accelSource == "baro-derived");
  CHECK_NEAR(log.samples[0].pressure, 98000.0, 1e-6);
}

TEST(synthetic_log_roundtrip_and_replay) {
  FlightLog log = synthesizeLog(5);
  CHECK(log.samples.size() > 100);
  const char* path = "/tmp/gr_test_log.csv";
  CHECK(writeFlightCsv(path, log));
  FlightLog back;
  CHECK(readFlightCsv(path, back));
  CHECK_NEAR(back.mainT, log.mainT, 1e-3);
  ReplayResult r = replayLog(back, gr::Params{});
  CHECK(!r.falseTrigger);
  if (r.hasMain) CHECK(r.detected && !r.missed);
}

namespace {
// Fly a scenario; at `resetAt` (s after the canopy opened, or before it if
// negative) restart the flight core from its resume snapshot.
struct ResetResult {
  bool landed = false;
  double miss = 1e9;
  gr::Phase phaseAfter = gr::Phase::Disarmed;
  bool steeredAfterReset = false;
  double maxBrakeAfterRelease = 0.0;
  bool guidedLater = false;
};
ResetResult flyWithReset(uint64_t seed, double resetAt) {
  ScenarioOptions o;
  o.dualDeployOnly = true;  // gives a long descent before the canopy
  Scenario s = makeScenario(seed, o);
  World w(s);
  SensorSuite sens(s);
  auto core = std::make_unique<gr::FlightCore>();
  double duty[2] = {0, 0};
  gr::Inputs in;
  ResetResult res;
  bool didReset = false;
  double resetT = 0.0;
  while (w.t() < 1500 && w.stage() != TruthStage::Landed) {
    w.step(0.005, duty, sens.batteryV());
    sens.sample(w, duty, in);
    const bool trigger = !didReset && ((resetAt >= 0 && w.openT() >= 0 && w.t() > w.openT() + resetAt) ||
                                       (resetAt < 0 && core->status().stage == gr::FlightStage::Descent && w.openT() < 0));
    if (trigger) {
      const gr::ResumeState rs = core->resumeState(in.encoder);
      core = std::make_unique<gr::FlightCore>();
      CHECK(core->resume(rs));
      didReset = true;
      resetT = w.t();
      res.phaseAfter = core->status().phase;
    }
    const gr::Outputs& out = core->update(in);
    duty[0] = out.motorDuty[0];
    duty[1] = out.motorDuty[1];
    if (didReset) {
      if (core->status().phase == gr::Phase::Guided) res.guidedLater = true;
      if (resetAt >= 0 && core->status().steeringActive) res.steeredAfterReset = true;
      if (w.t() > resetT + 3.0) res.maxBrakeAfterRelease = std::max({res.maxBrakeAfterRelease, w.brakeFrac(0), w.brakeFrac(1)});
    }
  }
  res.landed = w.stage() == TruthStage::Landed;
  res.miss = std::hypot(w.landPos().x, w.landPos().y);
  return res;
}
}  // namespace

TEST(warm_reset_under_canopy_releases_brakes) {
  for (uint64_t seed : {4ULL, 9ULL, 23ULL}) {
    ResetResult r = flyWithReset(seed, 15.0);
    CHECK(r.phaseAfter == gr::Phase::Neutral);
    CHECK(!r.steeredAfterReset);
    CHECK(r.maxBrakeAfterRelease < 0.05);  // both lines let out within 3 s
    CHECK(r.landed);
  }
}

TEST(warm_reset_in_descent_still_guides_home) {
  int guidedHome = 0;
  for (uint64_t seed : {4ULL, 9ULL, 23ULL, 31ULL}) {
    ResetResult r = flyWithReset(seed, -1.0);
    CHECK(r.phaseAfter == gr::Phase::Descent);
    CHECK(r.landed);
    if (r.guidedLater && r.miss < 60.0) ++guidedHome;
  }
  CHECK(guidedHome >= 3);
}

TEST(resume_rejects_corrupt_or_ground_snapshots) {
  gr::FlightCore core;
  gr::ResumeState r;
  CHECK(!core.resume(r));  // no magic
  r.magic = gr::kResumeMagic;
  r.stage = static_cast<uint8_t>(gr::FlightStage::Pad);
  r.groundPressure = 84000.0f;
  r.checksum = gr::resumeChecksum(r);
  CHECK(!core.resume(r));  // pad: ordinary boot
  r.stage = static_cast<uint8_t>(gr::FlightStage::Canopy);
  r.checksum = gr::resumeChecksum(r) ^ 1u;
  CHECK(!core.resume(r));  // bad checksum
}
