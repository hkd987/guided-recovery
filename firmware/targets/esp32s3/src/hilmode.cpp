#include "hilmode.h"

#include <Arduino.h>

#include <atomic>

#include "config.h"
#include "gr/hil.hpp"
#include "hw.h"

namespace {

std::atomic<bool> active{false};
gr::hil::HilBoard* board = nullptr;
uint32_t lastBytesMs = 0;

class EspIo : public gr::hil::BoardIo {
 public:
  void readEncoders(int32_t enc[2]) override {
    encodersRead(enc[0], enc[1]);
    enc[0] *= g_settings.encSignL;
    enc[1] *= g_settings.encSignR;
  }
  float batteryVolts() override {
    // The ADC read is slow-ish; refresh at ~10 Hz.
    if (++div_ >= 20) { div_ = 0; volts_ = 0.8f * volts_ + 0.2f * ::batteryVolts(); }
    return volts_;
  }
  bool interlockArmed() override { return armSwitchArmed(); }
  void writeMotors(float l, float r, bool enabled) override { motorsWrite(l, r, enabled); }
  uint32_t micros() override { return ::micros(); }
  void send(const uint8_t* data, size_t len) override { Serial.write(data, len); }

 private:
  uint8_t div_ = 0;
  float volts_ = 8.0f;
} io;

}  // namespace

bool hilActive() { return active.load(std::memory_order_acquire); }

bool hilEnter(gr::FlightCore& core) {
  if (core.status().phase != gr::Phase::Disarmed) {
    Serial.println("HIL refused: armed. Put the magnet on first.");
    return false;
  }
  board = new gr::hil::HilBoard(core);
  Serial.println("HIL READY");
  Serial.flush();
  lastBytesMs = millis();
  active.store(true, std::memory_order_release);
  return true;
}

void hilService() {
  if (!board) return;
  uint8_t buf[256];
  size_t n = 0;
  while (Serial.available() && n < sizeof buf) buf[n++] = static_cast<uint8_t>(Serial.read());
  if (n) {
    board->onBytes(buf, n, io);
    lastBytesMs = millis();
  } else if (millis() - lastBytesMs > 500) {
    motorsWrite(0.0f, 0.0f, false);  // simulator gone quiet: let the winches coast
  }
}
