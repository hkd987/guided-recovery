// Status on the Heltec's built-in 0.96" OLED: phase, GPS, home, battery.
#pragma once

#include "gr/flight_core.hpp"

void displayBegin();
void displayShow(const gr::Status& st, float batteryV, bool logOk, uint32_t logFreePct);
void displayOff();
