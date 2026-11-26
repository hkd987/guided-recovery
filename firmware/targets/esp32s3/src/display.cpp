#include "display.h"

#include <U8g2lib.h>

#include "board.h"

namespace {
// Software I2C on the OLED's own pins, so it never touches the sensor bus.
U8G2_SSD1306_128X64_NONAME_F_SW_I2C oled(U8G2_R0, board::kOledScl, board::kOledSda, board::kOledRst);
bool on = false;
}  // namespace

void displayBegin() {
  pinMode(board::kVext, OUTPUT);
  digitalWrite(board::kVext, LOW);  // power the OLED
  delay(20);
  on = oled.begin();
  if (on) {
    oled.setFont(u8g2_font_6x10_tr);
    oled.clearBuffer();
    oled.drawStr(0, 10, "Guided Recovery OS");
    oled.drawStr(0, 24, "starting...");
    oled.sendBuffer();
  }
}

void displayShow(const gr::Status& st, float batteryV, bool logOk, uint32_t logFreePct) {
  if (!on) return;
  char line[32];
  oled.clearBuffer();
  snprintf(line, sizeof line, "%-8s %s", gr::phaseName(st.phase), gr::guideModeName(st.mode));
  oled.drawStr(0, 10, line);
  snprintf(line, sizeof line, "GPS %2u sv  %s", st.numSv, st.gpsGood ? "fix" : "no fix");
  oled.drawStr(0, 22, line);
  if (st.homeValid) snprintf(line, sizeof line, "HOME saved (%u fixes)", st.homeFixes);
  else snprintf(line, sizeof line, "home: waiting %u/50", st.homeFixes);
  oled.drawStr(0, 34, line);
  snprintf(line, sizeof line, "AGL %5.0f m  %4.1f m/s", st.agl, -st.climbRate);
  oled.drawStr(0, 46, line);
  snprintf(line, sizeof line, "BAT %.2fV LOG %s %u%%", batteryV, logOk ? "ok" : "--", logFreePct);
  oled.drawStr(0, 58, line);
  if (st.faults) {
    snprintf(line, sizeof line, "F%03X", st.faults);
    oled.drawStr(104, 10, line);
  }
  oled.sendBuffer();
}

void displayOff() {
  // Blank the panel only. Vext (GPIO 36) must stay low: on the V3 it also
  // powers the LoRa front end, so cutting it would deafen the radio.
  if (on) oled.setPowerSave(1);
  on = false;
}
