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
// Featherweight Blue Raven: a low-rate file (barometer, event flags) plus its
// high-rate sibling (3-axis accelerometer in g, 500 Hz), paired by file name.
TEST(reads_blue_raven_pair) {
  CHECK(blueRavenHighRatePath("/x/Rocket_BlRv_LR.csv") == "/x/Rocket_BlRv_HR.csv");
  CHECK(blueRavenHighRatePath("/x/WERT FBR LR_08-21-2025_14_11_45.csv") == "/x/WERT FBR HR_08-21-2025_14_11_45.csv");
  CHECK(blueRavenHighRatePath("/x/blueraven-low-rate.csv") == "/x/blueraven-high-rate.csv");
  CHECK(blueRavenHighRatePath("/x/Primary_lowrate.csv") == "/x/Primary_highrate.csv");
  CHECK(blueRavenHighRatePath("/x/featherweight_lr.csv") == "/x/featherweight_hr.csv");
  CHECK(blueRavenHighRatePath("/x/BR_LR_JAN.csv") == "/x/BR_HR_JAN.csv");
  CHECK(blueRavenHighRatePath("/LR_dir/flight.csv").empty());
  const char* lr = "/tmp/gr_test_br_LR.csv";
  const char* hr = "/tmp/gr_test_br_HR.csv";
  {
    std::FILE* f = std::fopen(lr, "w");
    std::fprintf(f, "Year,Month,Day,Time,Flight_Time_(s),Sync,Temperature_(F),Baro_Press_(atm),"
                    "Baro_Altitude_AGL_(feet),Apo_Volts,Main_Volts,Liftoff,Apogee,Apo_fired,Main_fired\n");
    // 50 Hz: pad, flight, then the export repeats the flight (flags reset).
    for (int i = 0; i < 300; ++i) {
      const double t = -1.0 + 0.02 * i;
      const int lift = t >= 0.0, apo = t >= 2.0, af = t >= 2.5, mf = t >= 4.0;
      std::fprintf(f, "2026,5,24,14:20:18.411,%.2f,1,70,%.4f,0,4,4,%d,%d,%d,%d\n", t, 0.98 - 0.0001 * i, lift, apo, af,
                   mf);
    }
    std::fprintf(f, "2026,5,24,14:20:30.000,5.10,1,70,0.9800,0,4,4,0,0,0,0\n");
    std::fprintf(f, "2026,5,24,14:20:30.020,5.12,1,70,0.9800,0,4,4,1,0,0,0\n");
    std::fclose(f);
    f = std::fopen(hr, "w");
    std::fprintf(f, "Year,Month,Day,Time,Flight_Time_(s),Sync,Gyro_X,Gyro_Y,Gyro_Z,Accel_X,Accel_Y,Accel_Z\n");
    for (int i = 0; i < 3000; ++i) {
      const double t = -1.0 + 0.002 * i;  // mounted with Z along the airframe
      std::fprintf(f, "2026,5,24,x,%.3f,1,0,0,0,0.10,0.00,%.2f\n", t, t >= 0.0 && t < 1.0 ? 8.0 : 1.0);
    }
    std::fclose(f);
  }
  FlightLog log;
  std::string err;
  bool companion = false;
  CHECK(!readFlightCsv(hr, log, &err, &companion));
  CHECK(companion);
  CHECK(readFlightCsv(lr, log, &err, &companion));
  CHECK(!companion);
  CHECK(log.hasTruth);
  CHECK_NEAR(log.mainT, 4.0, 0.011);
  CHECK(log.accelSource == "3-axis");
  CHECK(log.samples.back().t < 5.0);  // the repeated copy was dropped
  int fresh = 0, boost = 0;
  bool drogue = false;
  for (const auto& s : log.samples) {
    fresh += s.baroFresh ? 1 : 0;
    if (s.t > 0.1 && s.t < 0.9) { CHECK_NEAR(s.accel, 7.0 * 9.80665, 1e-6); ++boost; }
    drogue = drogue || s.state == "drogue";
  }
  CHECK(boost > 300);
  CHECK(drogue);
  CHECK(fresh >= 295 && fresh <= 300);  // one fresh barometer reading per low-rate row
  CHECK_NEAR(log.samples[100].lat1, 0.10 * 9.80665, 1e-6);
}

// Barometer-only altimeters: Missile Works RRC3 ("Events": Drogue, Main) and
// Eggtimer (Alt in feet, Drogue/Main nonzero once fired).
TEST(reads_rrc3_and_eggtimer_exports) {
  const char* path = "/tmp/gr_test_rrc3.csv";
  {
    std::FILE* f = std::fopen(path, "w");
    std::fprintf(f, "\xEF\xBB\xBFTime,Altitude,Pressure,Velocity,Temperature,Events,Voltages\n");
    for (int i = 0; i < 400; ++i) {
      const double t = 0.05 * i;
      const char* ev = i == 200 ? "Drogue" : i == 300 ? "Main" : "-";
      std::fprintf(f, "%.2f,0,%.2f,0,70, %s,9.2\n", t, 980.0 - 0.01 * i, ev);
    }
    std::fclose(f);
  }
  FlightLog log;
  std::string err;
  CHECK(readFlightCsv(path, log, &err));
  CHECK(log.hasTruth);
  CHECK_NEAR(log.mainT, 15.0, 1e-6);
  CHECK(log.accelSource == "baro-derived");
  CHECK(log.samples[250].state == "drogue");
  CHECK_NEAR(log.samples[0].pressure, 98000.0, 1e-6);
  {
    std::FILE* f = std::fopen(path, "w");
    std::fprintf(f, "T,Alt,Veloc,FAlt,FVeloc,LDA,LowV,Apogee,N-O,Drogue,Main\n");
    for (int i = 0; i < 100; ++i)
      std::fprintf(f, "%.2f,%d,0,0,0,0,0,0,0,%d,%d\n", 0.1 * i, 10 * i, i >= 50 ? 500 : 0, i >= 80 ? 800 : 0);
    std::fclose(f);
  }
  CHECK(readFlightCsv(path, log, &err));
  CHECK_NEAR(log.mainT, 8.0, 1e-6);
  CHECK(log.samples[60].state == "drogue");
  CHECK(log.samples[10].pressure > log.samples[90].pressure);  // feet above the pad -> lower pressure
}

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
