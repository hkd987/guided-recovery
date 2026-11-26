// Tests for the simulator itself, plus end-to-end smoke runs.
#include <cstdio>

#include "gr/estimators.hpp"
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
