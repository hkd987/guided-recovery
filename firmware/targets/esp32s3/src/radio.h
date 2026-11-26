// LoRa telemetry (SX1262 on the Heltec V3). Transmit-only on the pod.
#pragma once

#include <Arduino.h>

#include "gr/records.hpp"

bool radioBegin(float mhz, int8_t dbm);
bool radioOk();
// Queue a packet from the flight loop (keeps only the newest).
void radioPush(const gr::TelemetryPacket& p);
// Called from the radio task: sends the newest queued packet.
void radioService();
