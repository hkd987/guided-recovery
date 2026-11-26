#include "hw.h"

#include <driver/pcnt.h>

#include "board.h"
#include "config.h"

// ===================================================================== GPS

GpsStatus g_gps;

namespace {
HardwareSerial& gpsSerial = Serial1;
gr::ubx::Parser parser;

bool waitForPvt(uint32_t ms) {
  const uint32_t t0 = millis();
  const uint32_t before = parser.framesOk();
  while (millis() - t0 < ms) {
    while (gpsSerial.available()) {
      if (parser.feed(static_cast<uint8_t>(gpsSerial.read()))) return true;
    }
    delay(2);
  }
  // A configured receiver without a fix still sends NAV-PVT frames.
  return parser.framesOk() > before;
}
}  // namespace

const gr::ubx::Parser& gpsParser() { return parser; }

bool gpsBegin(uint32_t timeoutMs) {
  static const uint32_t bauds[] = {115200, 38400, 9600, 57600, 230400};
  uint8_t cfg[256];
  const size_t n = gr::ubx::buildPodConfig(115200, cfg, sizeof cfg);
  const uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    for (uint32_t b : bauds) {
      gpsSerial.begin(b, SERIAL_8N1, board::kGpsRx, board::kGpsTx);
      delay(50);
      gpsSerial.write(cfg, n);  // switches the receiver to 115200 + UBX
      gpsSerial.flush();
      delay(100);
      gpsSerial.updateBaudRate(115200);
      delay(20);
      gpsSerial.write(cfg, n);  // again at the new rate (signal config, rate)
      gpsSerial.flush();
      if (waitForPvt(1200)) {
        g_gps.configured = true;
        g_gps.baudFound = b;
        return true;
      }
    }
  }
  gpsSerial.updateBaudRate(115200);
  return false;
}

bool gpsPoll(gr::GpsFix& fix) {
  bool got = false;
  int budget = 512;  // bound the time spent per control cycle
  while (gpsSerial.available() && budget-- > 0) {
    if (parser.feed(static_cast<uint8_t>(gpsSerial.read()))) {
      fix = parser.fix();
      ++g_gps.fixes;
      got = true;
    }
  }
  return got;
}

// ================================================================== motors

namespace {
const int kPins[4] = {board::kMotorL_In1, board::kMotorL_In2, board::kMotorR_In1, board::kMotorR_In2};
const uint32_t kMaxDuty = (1u << board::kMotorPwmBits) - 1;

void pwmWrite(int idx, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(kPins[idx], duty);
#else
  ledcWrite(idx, duty);
#endif
}

void driveOne(int in1, int in2, float duty, bool enabled) {
  if (!enabled) {  // coast
    pwmWrite(in1, 0);
    pwmWrite(in2, 0);
    return;
  }
  duty = duty > 1.0f ? 1.0f : (duty < -1.0f ? -1.0f : duty);
  // Slow decay (drive/brake): one input held high, PWM on the other inverted.
  // Holds better against line tension than coasting between pulses.
  const uint32_t d = static_cast<uint32_t>(fabsf(duty) * kMaxDuty + 0.5f);
  if (duty > 0.0f) {
    pwmWrite(in1, kMaxDuty);
    pwmWrite(in2, kMaxDuty - d);
  } else if (duty < 0.0f) {
    pwmWrite(in1, kMaxDuty - d);
    pwmWrite(in2, kMaxDuty);
  } else {  // both high: brake
    pwmWrite(in1, kMaxDuty);
    pwmWrite(in2, kMaxDuty);
  }
}
}  // namespace

void motorsBegin() {
  for (int i = 0; i < 4; ++i) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(kPins[i], board::kMotorPwmHz, board::kMotorPwmBits);
#else
    ledcSetup(i, board::kMotorPwmHz, board::kMotorPwmBits);
    ledcAttachPin(kPins[i], i);
#endif
    pwmWrite(i, 0);
  }
}

void motorsWrite(float dutyL, float dutyR, bool enabled) {
  driveOne(0, 1, dutyL, enabled);
  driveOne(2, 3, dutyR, enabled);
}

// ================================================================ encoders

namespace {
const pcnt_unit_t kUnits[2] = {PCNT_UNIT_0, PCNT_UNIT_1};
int16_t lastRaw[2] = {0, 0};
int32_t total[2] = {0, 0};

void setupUnit(pcnt_unit_t unit, int a, int b) {
  pcnt_config_t c = {};
  c.unit = unit;
  c.counter_h_lim = 32767;
  c.counter_l_lim = -32768;
  // Channel 0: count A edges, direction from B.
  c.pulse_gpio_num = a;
  c.ctrl_gpio_num = b;
  c.channel = PCNT_CHANNEL_0;
  c.pos_mode = PCNT_COUNT_DEC;
  c.neg_mode = PCNT_COUNT_INC;
  c.lctrl_mode = PCNT_MODE_REVERSE;
  c.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_unit_config(&c);
  // Channel 1: count B edges, direction from A (full x4 quadrature).
  c.pulse_gpio_num = b;
  c.ctrl_gpio_num = a;
  c.channel = PCNT_CHANNEL_1;
  c.pos_mode = PCNT_COUNT_INC;
  c.neg_mode = PCNT_COUNT_DEC;
  pcnt_unit_config(&c);
  pcnt_set_filter_value(unit, 100);  // ~1.25 us glitch filter at 80 MHz
  pcnt_filter_enable(unit);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}
}  // namespace

void encodersBegin() {
  setupUnit(kUnits[0], board::kEncL_A, board::kEncL_B);
  setupUnit(kUnits[1], board::kEncR_A, board::kEncR_B);
  encodersZero();
}

void encodersRead(int32_t& left, int32_t& right) {
  for (int i = 0; i < 2; ++i) {
    int16_t raw = 0;
    pcnt_get_counter_value(kUnits[i], &raw);
    // 16-bit wrap-safe accumulation (called every 5 ms, far below wrap rate).
    total[i] += static_cast<int16_t>(raw - lastRaw[i]);
    lastRaw[i] = raw;
  }
  left = total[0];
  right = total[1];
}

void encodersRestore(int32_t left, int32_t right) {
  encodersZero();
  total[0] = left;
  total[1] = right;
}

void encodersZero() {
  for (int i = 0; i < 2; ++i) {
    pcnt_counter_clear(kUnits[i]);
    lastRaw[i] = 0;
    total[i] = 0;
  }
}

// ======================================================================= io

namespace {
uint32_t armChangeMs = 0, fireChangeMs = 0;
bool armStable = false, armLast = false, fireStable = false, fireLast = false;
}  // namespace

void ioBegin() {
  pinMode(board::kArmSwitch, INPUT_PULLUP);
  pinMode(board::kMainFired, INPUT_PULLUP);
  pinMode(board::kBuzzer, OUTPUT);
  pinMode(board::kLed, OUTPUT);
  digitalWrite(board::kBuzzer, LOW);
  analogReadResolution(12);
  armLast = armStable = digitalRead(board::kArmSwitch) == HIGH;
}

bool armSwitchArmed() {
  // Reed switch closes to GND while the remove-before-flight magnet is on.
  const bool raw = digitalRead(board::kArmSwitch) == HIGH;
  const uint32_t now = millis();
  if (raw != armLast) { armLast = raw; armChangeMs = now; }
  if (now - armChangeMs > 50) armStable = raw;
  return armStable;
}

bool mainFiredInput() {
  const bool raw = digitalRead(board::kMainFired) == LOW;
  const uint32_t now = millis();
  if (raw != fireLast) { fireLast = raw; fireChangeMs = now; }
  if (now - fireChangeMs > 5) fireStable = raw;
  return fireStable;
}

float batteryVolts() {
  const uint32_t mv = analogReadMilliVolts(board::kBatterySense);
  return mv * 0.001f * board::kBatteryDivider * g_settings.batteryScale;
}

void buzzer(bool on) { digitalWrite(board::kBuzzer, on ? HIGH : LOW); }
void led(bool on) { digitalWrite(board::kLed, on ? HIGH : LOW); }
