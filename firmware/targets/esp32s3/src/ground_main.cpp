// Ground receiver: a second Heltec V3 that listens for the pod's LoRa
// telemetry and shows it on its OLED and USB serial. No other parts needed.
//
//   pio run -e ground -t upload

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <U8g2lib.h>

#include "board.h"
#include "gr/records.hpp"
#include "gr/types.hpp"

namespace {
SPIClass loraSpi(HSPI);
SX1262 lora = new Module(board::kLoraNss, board::kLoraDio1, board::kLoraRst, board::kLoraBusy, loraSpi);
U8G2_SSD1306_128X64_NONAME_F_SW_I2C oled(U8G2_R0, board::kOledScl, board::kOledSda, board::kOledRst);
volatile bool received = false;
float freqMHz = 915.0f;

void IRAM_ATTR onDio1() { received = true; }
}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(board::kVext, OUTPUT);
  digitalWrite(board::kVext, LOW);
  delay(20);
  oled.begin();
  oled.setFont(u8g2_font_6x10_tr);
  loraSpi.begin(board::kLoraSck, board::kLoraMiso, board::kLoraMosi, board::kLoraNss);
  int16_t st = lora.begin(freqMHz, 250.0, 7, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 10, 8, 1.8, false);
  if (st == RADIOLIB_ERR_NONE) st = lora.setDio2AsRfSwitch(true);
  if (st == RADIOLIB_ERR_NONE) st = lora.setCRC(true);
  lora.setDio1Action(onDio1);
  lora.startReceive();
  Serial.printf("Guided Recovery ground receiver, %.1f MHz, radio %s\n", freqMHz, st == RADIOLIB_ERR_NONE ? "ok" : "FAILED");
  oled.clearBuffer();
  oled.drawStr(0, 10, "GR ground receiver");
  oled.drawStr(0, 24, st == RADIOLIB_ERR_NONE ? "listening..." : "radio failed");
  oled.sendBuffer();
}

void loop() {
  if (!received) { delay(5); return; }
  received = false;
  gr::TelemetryPacket p;
  const size_t len = lora.getPacketLength();
  const int16_t st = lora.readData(reinterpret_cast<uint8_t*>(&p), sizeof(p));
  const float rssi = lora.getRSSI();
  lora.startReceive();
  if (st != RADIOLIB_ERR_NONE || len != sizeof(p) || p.magic != 0x47 || !gr::recordValid(p)) return;
  const char* phase = gr::phaseName(static_cast<gr::Phase>(p.phase));
  Serial.printf("seq %u %s %s faults 0x%03X lat %.7f lon %.7f agl %d m climb %.1f m/s home %u m batt %.2f V sv %u rssi %.0f\n",
                p.seq, phase, gr::guideModeName(static_cast<gr::GuideMode>(p.mode)), p.faults, p.latE7 * 1e-7,
                p.lonE7 * 1e-7, p.agl, p.climb * 0.1f, p.distHome, p.battery * 0.05f, p.numSv, rssi);
  char line[32];
  oled.clearBuffer();
  snprintf(line, sizeof line, "%s  %s", phase, gr::guideModeName(static_cast<gr::GuideMode>(p.mode)));
  oled.drawStr(0, 10, line);
  snprintf(line, sizeof line, "AGL %d m  %.1f m/s", p.agl, -p.climb * 0.1f);
  oled.drawStr(0, 22, line);
  snprintf(line, sizeof line, "home %u m  sv %u", p.distHome, p.numSv);
  oled.drawStr(0, 34, line);
  snprintf(line, sizeof line, "%.6f", p.latE7 * 1e-7);
  oled.drawStr(0, 46, line);
  snprintf(line, sizeof line, "%.6f  %.0fdB", p.lonE7 * 1e-7, rssi);
  oled.drawStr(0, 58, line);
  oled.sendBuffer();
}
