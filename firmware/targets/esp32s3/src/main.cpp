// Guided Recovery OS - S pod firmware entry point.
//
// Core 1: the 200 Hz flight task (sensors -> FlightCore -> winches).
// Core 0: flash logger, LoRa telemetry and OLED status tasks.
// Arduino loop(): the USB serial console (see cli.cpp).

#include <Arduino.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <esp_timer.h>

#include "app.h"
#include "board.h"
#include "cli.h"
#include "config.h"
#include "display.h"
#include "flashlog.h"
#include "hw.h"
#include "radio.h"
#include "sensors.h"

// Flight snapshot in RTC memory: survives brownout, watchdog and panic
// resets (not power-off). Written every cycle; read once at boot.
RTC_NOINIT_ATTR gr::ResumeState rtcSnapshot;

namespace {

gr::FlightCore* core = nullptr;
bool resumedFlight = false;
SemaphoreHandle_t snapMutex = nullptr;
Snapshot snap;

portMUX_TYPE reqMux = portMUX_INITIALIZER_UNLOCKED;
float benchReq[2] = {0.0f, 0.0f};
bool benchPending = false, benchReleaseReq = false, zeroReq = false, reconfigReq = false;

void flightTask(void*) {
  TickType_t last = xTaskGetTickCount();
  uint8_t baroDiv = 0, battDiv = 0, snapDiv = 0;
  uint32_t padLogDiv = 0;
  float battV = batteryVolts();
  float pressure = 0.0f, tempC = 0.0f;
  gr::Inputs in;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
    const uint32_t t0 = micros();

    in = gr::Inputs{};
    in.tUs = static_cast<uint64_t>(esp_timer_get_time());
    gr::Vec3 a, g;
    if (imuRead(a, g)) {
      in.imu.accel = toBody(a);
      in.imu.gyro = toBody(g);
      in.imu.valid = true;
    }
    if (++baroDiv >= 4) {
      baroDiv = 0;
      float p, t;
      if (baroRead(p, t)) {
        in.baroFresh = true;
        in.pressurePa = pressure = p;
        in.baroTempC = tempC = t;
      }
      gr::Vec3 hg;
      if (highGRead(hg)) { in.highG = toBody(hg); in.highGValid = true; }
    }
    gpsPoll(in.gps);
    encodersRead(in.encoder[0], in.encoder[1]);
    const int32_t rawEnc[2] = {in.encoder[0], in.encoder[1]};
    in.encoder[0] *= g_settings.encSignL;
    in.encoder[1] *= g_settings.encSignR;
    in.armed = armSwitchArmed();
    in.mainFiredInput = mainFiredInput();
    if (++battDiv >= 20) { battDiv = 0; battV = 0.8f * battV + 0.2f * batteryVolts(); }
    in.batteryV = battV;

    // Console requests (bench mode only).
    portENTER_CRITICAL(&reqMux);
    const bool doZero = zeroReq, doReconfig = reconfigReq, doRelease = benchReleaseReq, doBench = benchPending;
    const float req0 = benchReq[0], req1 = benchReq[1];
    zeroReq = reconfigReq = benchReleaseReq = benchPending = false;
    portEXIT_CRITICAL(&reqMux);
    const bool disarmed = core->status().phase == gr::Phase::Disarmed;
    if (disarmed) {
      if (doReconfig) { delete core; core = new gr::FlightCore(settingsToParams()); }
      if (doZero) { encodersZero(); in.encoder[0] = in.encoder[1] = 0; core->reset(); }
      if (doBench) { core->benchSetBrake(0, req0); core->benchSetBrake(1, req1); }
      if (doRelease) core->benchRelease();
    }

    const gr::Outputs& out = core->update(in);
    motorsWrite(out.motorDuty[0], out.motorDuty[1], out.motorsEnabled);
    rtcSnapshot = core->resumeState(rawEnc);
    buzzer(out.buzzer);
    led(out.led);

    gr::LogRecord rec;
    if (core->takeLogRecord(rec)) {
      const gr::Phase ph = core->status().phase;
      // Bench time is not logged; the pad is logged at 5 Hz, flight at 50 Hz.
      if (ph == gr::Phase::Pad) { if (padLogDiv++ % 10 == 0) flashLogPush(rec); }
      else if (ph != gr::Phase::Disarmed) flashLogPush(rec);
    }
    gr::TelemetryPacket tp;
    if (core->takeTelemetry(tp)) radioPush(tp);

    const uint32_t dt = micros() - t0;
    if (++snapDiv >= 10 && xSemaphoreTake(snapMutex, 0) == pdTRUE) {
      snapDiv = 0;
      snap.status = core->status();
      snap.accelBody = in.imu.accel;
      snap.gyroBody = in.imu.gyro;
      snap.pressurePa = pressure;
      snap.tempC = tempC;
      snap.batteryV = battV;
      snap.encoder[0] = in.encoder[0];
      snap.encoder[1] = in.encoder[1];
      snap.armed = in.armed;
      snap.mainFired = in.mainFiredInput;
      xSemaphoreGive(snapMutex);
    }
    ++snap.loops;
    if (dt > 5000) ++snap.overruns;
    if (dt > snap.maxLoopUs) snap.maxLoopUs = dt;
  }
}

void loggerTask(void*) {
  for (;;) {
    flashLogService();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void radioTask(void*) {
  for (;;) {
    radioService();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void displayTask(void*) {
  Snapshot s;
  for (;;) {
    appSnapshot(s);
    const FlashLogInfo li = flashLogInfo();
    const uint32_t freePct = li.capacity ? 100u - (100u * li.used) / li.capacity : 0;
    displayShow(s.status, s.batteryV, li.ok, freePct);
    // Blank the OLED in flight to save power; it is back after landing.
    const gr::Phase ph = s.status.phase;
    vTaskDelay(pdMS_TO_TICKS((ph == gr::Phase::Disarmed || ph == gr::Phase::Pad || ph == gr::Phase::Landed) ? 500 : 2000));
  }
}

}  // namespace

void appSnapshot(Snapshot& out) {
  if (xSemaphoreTake(snapMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    out = snap;
    xSemaphoreGive(snapMutex);
  }
}

void appBenchSet(int ch, float frac) {
  portENTER_CRITICAL(&reqMux);
  if (ch >= 0 && ch < 2) benchReq[ch] = frac;
  benchPending = true;
  portEXIT_CRITICAL(&reqMux);
}

void appBenchRelease() {
  portENTER_CRITICAL(&reqMux);
  benchReq[0] = benchReq[1] = 0.0f;
  benchPending = true;
  benchReleaseReq = true;
  portEXIT_CRITICAL(&reqMux);
}

void appZeroWinches() {
  portENTER_CRITICAL(&reqMux);
  zeroReq = true;
  portEXIT_CRITICAL(&reqMux);
}

void appReconfigure() {
  portENTER_CRITICAL(&reqMux);
  reconfigReq = true;
  portEXIT_CRITICAL(&reqMux);
}

bool warmRestart() {
  switch (esp_reset_reason()) {
    case ESP_RST_BROWNOUT:
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
    case ESP_RST_SW:
      return true;
    default:
      return false;  // power-on: RTC memory holds garbage
  }
}

void appStart() {
  snapMutex = xSemaphoreCreateMutex();
  core = new gr::FlightCore(settingsToParams());
  // A restart in flight resumes from the snapshot: Ascent/Descent carry on
  // with the saved home point; under a canopy both brakes are released.
  if (warmRestart() && core->resume(rtcSnapshot)) {
    encodersRestore(rtcSnapshot.encoder[0], rtcSnapshot.encoder[1]);
    resumedFlight = true;
  }
  rtcSnapshot.magic = 0;
  xTaskCreatePinnedToCore(flightTask, "flight", 8192, nullptr, configMAX_PRIORITIES - 2, nullptr, 1);
  xTaskCreatePinnedToCore(loggerTask, "logger", 4096, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(radioTask, "radio", 4096, nullptr, 4, nullptr, 0);
  xTaskCreatePinnedToCore(displayTask, "display", 4096, nullptr, 2, nullptr, 0);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nGuided Recovery OS - S pod (Heltec V3)");
  settingsLoad();
  ioBegin();
  motorsBegin();
  motorsWrite(0, 0, false);
  encodersBegin();
  displayBegin();
  sensorsBegin();
  Serial.printf("IMU %s, baro %s, high-g %s\n",
                g_sensors.imu == ImuType::Bmi088 ? "BMI088" : g_sensors.imu == ImuType::Lsm6dso32 ? "LSM6DSO32" : "MISSING",
                g_sensors.baro ? "BMP390" : "MISSING", g_sensors.highG ? "ADXL375" : "none");
  // After an in-flight restart, get going quickly: the receiver is still
  // configured, so the first baud rate tried answers at once.
  const bool maybeInFlight = warmRestart() && rtcSnapshot.magic == gr::kResumeMagic;
  Serial.print("GPS: configuring... ");
  Serial.println(gpsBegin(maybeInFlight ? 1500 : 4000) ? "ok" : "no response (check wiring)");
  Serial.printf("Log: %s\n", flashLogBegin() ? "ok" : "partition missing");
  Serial.printf("Radio: %s\n", radioBegin(g_settings.radioMHz, g_settings.radioDbm) ? "ok" : "failed");
  appStart();
  if (resumedFlight) Serial.printf("RESUMED FLIGHT after restart (reason %d): %s\n", static_cast<int>(esp_reset_reason()),
                                   gr::phaseName(core->status().phase));
  Serial.println("Type 'help'. Magnet on = disarmed (bench mode).");
}

void loop() {
  cliPoll();
  delay(10);
}
