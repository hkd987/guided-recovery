#include "cli.h"

#include <Arduino.h>

#include "app.h"
#include "config.h"
#include "flashlog.h"
#include "hilmode.h"
#include "hw.h"
#include "radio.h"
#include "sensors.h"

namespace {

String line;

bool disarmed(const Snapshot& s) { return s.status.phase == gr::Phase::Disarmed; }

void printStatus(const Snapshot& s) {
  const gr::Status& st = s.status;
  Serial.printf("phase %s  mode %s  faults 0x%03X  armed %s\n", gr::phaseName(st.phase), gr::guideModeName(st.mode),
                st.faults, s.armed ? "yes" : "no (magnet on)");
  Serial.printf("gps %s  sv %u  home %s (%u fixes)\n", st.gpsGood ? "fix" : "none", st.numSv,
                st.homeValid ? "saved" : "not yet", st.homeFixes);
  if (st.homeValid) Serial.printf("home %.7f, %.7f\n", st.homeLat, st.homeLon);
  Serial.printf("agl %.1f m  climb %.2f m/s  pressure %.1f Pa  temp %.1f C\n", st.agl, st.climbRate, s.pressurePa,
                s.tempC);
  Serial.printf("accel %.2f %.2f %.2f m/s2  |a| %.2f g  gyro %.3f %.3f %.3f rad/s\n", s.accelBody.x, s.accelBody.y,
                s.accelBody.z, st.accelG, s.gyroBody.x, s.gyroBody.y, s.gyroBody.z);
  Serial.printf("winch L %.1f mm  R %.1f mm  (counts %ld / %ld)\n", st.winchMm[0], st.winchMm[1],
                static_cast<long>(s.encoder[0]), static_cast<long>(s.encoder[1]));
  const FlashLogInfo li = flashLogInfo();
  Serial.printf("battery %.2f V  log %lu/%lu records  loop max %lu us, overruns %lu\n", s.batteryV,
                static_cast<unsigned long>(li.used), static_cast<unsigned long>(li.capacity),
                static_cast<unsigned long>(s.maxLoopUs), static_cast<unsigned long>(s.overruns));
}

// Move one winch to a position and report how it got there.
bool jogAndMeasure(int ch, float mm, float& reachedMm, float& speedMmS) {
  Snapshot s;
  appSnapshot(s);
  const float start = s.status.winchMm[ch];
  appBenchSet(ch, mm / g_settings.winchTravelMm);
  const uint32_t t0 = millis();
  float last = start;
  uint32_t tReach = 0;
  while (millis() - t0 < 2500) {
    delay(20);
    appSnapshot(s);
    last = s.status.winchMm[ch];
    if (!tReach && fabsf(last - mm) < 1.5f) tReach = millis() - t0;
  }
  reachedMm = last;
  speedMmS = tReach ? fabsf(mm - start) / (tReach / 1000.0f) : 0.0f;
  return tReach != 0;
}

void selftest() {
  Snapshot s;
  appSnapshot(s);
  if (!disarmed(s)) { Serial.println("Put the magnet on (disarm) first."); return; }
  int fails = 0, warns = 0;
  auto pass = [](const char* what) { Serial.printf("  PASS  %s\n", what); };
  auto fail = [&](const char* what) { Serial.printf("  FAIL  %s\n", what); ++fails; };
  auto warn = [&](const char* what) { Serial.printf("  WARN  %s\n", what); ++warns; };

  Serial.println("Self-test (pod upright and still, magnet on)");
  i2cScan(Serial);

  // IMU
  if (g_sensors.imu == ImuType::None) fail("IMU not found (BMI088 at 0x18/0x68 or LSM6DSO32 at 0x6A)");
  else {
    pass(g_sensors.imu == ImuType::Bmi088 ? "IMU BMI088 found" : "IMU LSM6DSO32 found");
    const float g = s.accelBody.norm() / 9.80665f;
    if (g < 0.9f || g > 1.1f) fail("accelerometer magnitude not ~1 g at rest");
    else pass("accelerometer reads 1 g");
    if (s.accelBody.z > -8.0f) {
      warn("body z is not pointing down: fix 'imu.map' so accel z reads about -9.8 when upright");
      Serial.printf("        accel body = %.2f %.2f %.2f\n", s.accelBody.x, s.accelBody.y, s.accelBody.z);
    } else {
      pass("IMU orientation (z down)");
    }
    if (s.gyroBody.norm() > 0.05f) warn("gyro not near zero: keep the pod still");
    else pass("gyro at rest");
  }

  // Barometer
  if (!g_sensors.baro) fail("barometer BMP390 not found (0x77/0x76)");
  else if (s.pressurePa < 50000.0f || s.pressurePa > 110000.0f) fail("barometer pressure implausible");
  else pass("barometer BMP390");
  if (g_sensors.highG) pass("high-g ADXL375 (optional)");

  // GPS
  if (!g_gps.configured) fail("GPS did not answer (check RX/TX wiring and 3.3 V)");
  else {
    Serial.printf("  INFO  GPS found at %lu baud, %lu fixes, %u satellites\n", static_cast<unsigned long>(g_gps.baudFound),
                  static_cast<unsigned long>(g_gps.fixes), s.status.numSv);
    if (s.status.gpsGood) pass("GPS 3D fix");
    else warn("no GPS fix yet (needs open sky; can take a minute)");
  }

  // Battery
  if (s.batteryV < 6.6f) fail("battery low or divider miswired (2S needs 6.6-8.4 V)");
  else if (s.batteryV > 8.6f) fail("battery voltage too high (2S only: DRV8833 max 10.8 V)");
  else pass("battery");

  // Log and radio
  const FlashLogInfo li = flashLogInfo();
  if (!li.ok) fail("log partition missing (flash with the provided partitions.csv)");
  else if (li.used * 10 > li.capacity * 8) warn("log over 80% full: 'log dump' then 'log erase'");
  else pass("flight log space");
  if (radioOk()) pass("LoRa radio");
  else warn("LoRa radio did not start (telemetry only; flight is unaffected)");

  // Winches: pull 25 mm in, then release; check direction, speed, no jam.
  Serial.println("  ....  winches: each pulls 25 mm then releases. Keep fingers clear.");
  appZeroWinches();
  delay(100);
  for (int ch = 0; ch < 2; ++ch) {
    const char* name = ch == 0 ? "left" : "right";
    float reached = 0, speed = 0;
    const bool ok = jogAndMeasure(ch, 25.0f, reached, speed);
    appSnapshot(s);
    char msg[160];
    const char side = ch == 0 ? 'L' : 'R';
    if (reached < -3.0f) {
      // Encoder says it moved the opposite way to the drive: polarity mismatch.
      snprintf(msg, sizeof msg, "%s encoder counts backwards: set winch.enc%c -1 (or swap encoder A/B)", name, side);
      fail(msg);
    } else if (s.status.faults & (ch == 0 ? gr::kFaultWinchLeft : gr::kFaultWinchRight)) {
      snprintf(msg, sizeof msg, "%s winch stalled (jammed spool, motor wiring, or no encoder signal)", name);
      fail(msg);
    } else if (!ok) {
      snprintf(msg, sizeof msg, "%s winch did not reach 25 mm (got %.1f mm): check encoder wiring and winch.cpmm",
               name, reached);
      fail(msg);
    } else {
      snprintf(msg, sizeof msg, "%s winch: %.0f mm/s line speed. Did the line pull IN? If it paid out, set winch.dir%c -1",
               name, speed, side);
      if (speed < 50.0f) warn(msg);
      else pass(msg);
    }
    float back, sp;
    jogAndMeasure(ch, 0.0f, back, sp);
  }
  appBenchRelease();

  Serial.printf("Self-test: %d failed, %d warnings. %s\n", fails, warns,
                fails ? "Fix the failures before flying." : "Ready for bench and drop testing.");
}

void help() {
  Serial.println(
      "status              one-line summary of everything\n"
      "watch               status twice a second (any key stops)\n"
      "selftest            bench self-test (magnet on, pod upright)\n"
      "winch l|r <mm>      move one winch (magnet on)\n"
      "winch release       release both lines\n"
      "zero                current winch positions become zero (lines slack)\n"
      "params              list settings\n"
      "set <name> <value>  change a setting, then 'save'\n"
      "save | defaults     store settings / restore defaults\n"
      "log info|dump|erase flight log (dump prints CSV)\n"
      "gps                 GPS diagnostics\n"
      "hil                 bench hardware-in-the-loop with sim/apps/grhil (reboot to leave)\n"
      "reboot");
}

void handle(const String& cmdline) {
  String cmd = cmdline;
  cmd.trim();
  if (cmd.isEmpty()) return;
  const int sp = cmd.indexOf(' ');
  const String verb = sp < 0 ? cmd : cmd.substring(0, sp);
  String rest = sp < 0 ? String() : cmd.substring(sp + 1);
  rest.trim();
  Snapshot s;
  appSnapshot(s);

  if (verb == "help" || verb == "?") help();
  else if (verb == "status") printStatus(s);
  else if (verb == "watch") {
    while (!Serial.available()) { appSnapshot(s); printStatus(s); Serial.println(); delay(500); }
    while (Serial.available()) Serial.read();
  }
  else if (verb == "selftest") selftest();
  else if (verb == "winch") {
    if (!disarmed(s)) { Serial.println("refused: armed"); return; }
    if (rest == "release") { appBenchRelease(); return; }
    const int sp2 = rest.indexOf(' ');
    const String side = rest.substring(0, sp2);
    const float mm = rest.substring(sp2 + 1).toFloat();
    const int ch = side == "l" ? 0 : side == "r" ? 1 : -1;
    if (ch < 0 || sp2 < 0) { Serial.println("usage: winch l|r <mm>"); return; }
    appBenchSet(ch, mm / g_settings.winchTravelMm);
  }
  else if (verb == "zero") {
    if (!disarmed(s)) { Serial.println("refused: armed"); return; }
    appZeroWinches();
  }
  else if (verb == "params") settingsPrint(Serial);
  else if (verb == "set") {
    if (!disarmed(s)) { Serial.println("refused: armed"); return; }
    const int sp2 = rest.indexOf(' ');
    String err;
    if (sp2 < 0) Serial.println("usage: set <name> <value>");
    else if (!settingsSet(rest.substring(0, sp2), rest.substring(sp2 + 1), err)) Serial.printf("error: %s\n", err.c_str());
    else { appReconfigure(); Serial.println("ok (not saved yet: 'save')"); }
  }
  else if (verb == "save") Serial.println(settingsSave() ? "saved" : "save failed");
  else if (verb == "defaults") {
    if (!disarmed(s)) { Serial.println("refused: armed"); return; }
    settingsReset();
    appReconfigure();
    Serial.println("defaults restored (not saved yet: 'save')");
  }
  else if (verb == "log") {
    if (rest == "info") {
      const FlashLogInfo li = flashLogInfo();
      Serial.printf("log %s: %lu of %lu records used (%.0f min at 50 Hz free), dropped %lu\n", li.ok ? "ok" : "missing",
                    static_cast<unsigned long>(li.used), static_cast<unsigned long>(li.capacity),
                    (li.capacity - li.used) / 50.0f / 60.0f, static_cast<unsigned long>(flashLogDropped()));
    } else if (rest == "dump") {
      flashLogDumpCsv(Serial);
    } else if (rest == "erase") {
      if (!disarmed(s)) { Serial.println("refused: armed"); return; }
      flashLogErase(Serial);
    } else {
      Serial.println("usage: log info|dump|erase");
    }
  }
  else if (verb == "gps") {
    const gr::ubx::Parser& p = gpsParser();
    Serial.printf("configured %s at %lu baud; frames ok %lu bad %lu; fixes %lu; last: type %u sv %u hAcc %.1f m\n",
                  g_gps.configured ? "yes" : "no", static_cast<unsigned long>(g_gps.baudFound),
                  static_cast<unsigned long>(p.framesOk()), static_cast<unsigned long>(p.framesBad()),
                  static_cast<unsigned long>(g_gps.fixes), p.fix().fixType, p.fix().numSv, p.fix().hAcc);
  }
  else if (verb == "hil") {
    if (!disarmed(s)) { Serial.println("HIL refused: armed. Put the magnet on first."); return; }
    appEnterHil();
  }
  else if (verb == "reboot") ESP.restart();
  else Serial.println("unknown command; type 'help'");
}

}  // namespace

void cliPoll() {
  if (hilActive()) return;  // the serial port carries HIL frames now
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      handle(line);
      line = "";
      Serial.print("> ");
    } else if (line.length() < 120) {
      line += c;
    }
  }
}
