#include "radio.h"

#include <RadioLib.h>
#include <SPI.h>

#include "board.h"

namespace {
SPIClass loraSpi(HSPI);
SX1262 lora = new Module(board::kLoraNss, board::kLoraDio1, board::kLoraRst, board::kLoraBusy, loraSpi);
bool ok = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
gr::TelemetryPacket pending;
bool havePending = false;
}  // namespace

bool radioBegin(float mhz, int8_t dbm) {
  loraSpi.begin(board::kLoraSck, board::kLoraMiso, board::kLoraMosi, board::kLoraNss);
  // SF7 / 250 kHz keeps a 24-byte packet to ~30 ms on air (2 Hz telemetry).
  // The Heltec V3 TCXO runs from DIO3 at 1.8 V; DIO2 switches the antenna.
  int16_t st = lora.begin(mhz, 250.0, 7, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, dbm, 8, 1.8, false);
  if (st == RADIOLIB_ERR_NONE) st = lora.setDio2AsRfSwitch(true);
  if (st == RADIOLIB_ERR_NONE) st = lora.setCRC(true);
  ok = st == RADIOLIB_ERR_NONE;
  return ok;
}

bool radioOk() { return ok; }

void radioPush(const gr::TelemetryPacket& p) {
  portENTER_CRITICAL(&mux);
  pending = p;
  havePending = true;
  portEXIT_CRITICAL(&mux);
}

void radioService() {
  if (!ok) return;
  gr::TelemetryPacket p;
  bool have;
  portENTER_CRITICAL(&mux);
  have = havePending;
  p = pending;
  havePending = false;
  portEXIT_CRITICAL(&mux);
  if (have) lora.transmit(reinterpret_cast<uint8_t*>(&p), sizeof(p));
}
