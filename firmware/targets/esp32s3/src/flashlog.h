// Flight log in the ESP32's own flash ("grlog" partition, ~5.4 MB).
//
// Erasing flash stalls both CPU cores for tens of milliseconds, which the
// 200 Hz control loop cannot afford. So the log is erased only on the bench
// ("log erase"), and flights append 64-byte records into already-erased
// space; page writes stall for well under a millisecond.
#pragma once

#include <Arduino.h>

#include "gr/records.hpp"

struct FlashLogInfo {
  bool ok = false;
  uint32_t capacity = 0;   // records
  uint32_t used = 0;       // records written
};

bool flashLogBegin();
FlashLogInfo flashLogInfo();
// Queue a record from the flight loop (never blocks; drops if the queue is full).
void flashLogPush(const gr::LogRecord& r);
// Called from the logger task: writes queued records to flash.
void flashLogService();
uint32_t flashLogDropped();
// Bench operations (disarmed only).
bool flashLogErase(Print& out);
void flashLogDumpCsv(Print& out);
