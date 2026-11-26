// Fixed-size binary records: the flight log (written to SPI flash at 50 Hz)
// and the LoRa telemetry packet (2 Hz). Little-endian, packed.
#pragma once

#include <cstddef>
#include <cstdint>

namespace gr {

#pragma pack(push, 1)
struct LogRecord {
  uint8_t magic = 0xA5;
  uint8_t version = 1;
  uint32_t tMs = 0;
  uint8_t phase = 0;
  uint8_t mode = 0;
  uint16_t faults = 0;
  uint8_t flags = 0;        // see kLog* below
  int16_t accel[3] = {};    // 0.01 g
  int16_t gyro[3] = {};     // mrad/s
  float pressurePa = 0.0f;
  float agl = 0.0f;         // m
  int16_t climb = 0;        // cm/s
  float posN = 0.0f;        // m from home
  float posE = 0.0f;
  int16_t velN = 0;         // cm/s
  int16_t velE = 0;
  int8_t windN = 0;         // 0.2 m/s
  int8_t windE = 0;
  uint8_t airspeed = 0;     // 0.1 m/s
  int16_t turnCmd = 0;      // mrad/s
  int16_t yawRate = 0;      // mrad/s
  uint8_t brakeL = 0;       // target, 0.5 %
  uint8_t brakeR = 0;
  int16_t winchL = 0;       // measured, 0.1 mm
  int16_t winchR = 0;
  int8_t dutyL = 0;         // %
  int8_t dutyR = 0;
  uint8_t battery = 0;      // 0.05 V
  uint8_t numSv = 0;
  uint8_t reserved = 0;
  uint8_t checksum = 0;     // XOR of all previous bytes
};

struct TelemetryPacket {
  uint8_t magic = 0x47;     // 'G'
  uint8_t seq = 0;
  uint8_t phase = 0;
  uint8_t mode = 0;
  uint16_t faults = 0;
  int32_t latE7 = 0;
  int32_t lonE7 = 0;
  int16_t agl = 0;          // m
  int16_t climb = 0;        // 0.1 m/s
  uint16_t distHome = 0;    // m
  uint8_t battery = 0;      // 0.05 V
  uint8_t numSv = 0;
  uint8_t flags = 0;
  uint8_t checksum = 0;
};
#pragma pack(pop)

static_assert(sizeof(LogRecord) == 64, "log record must stay 64 bytes");
static_assert(sizeof(TelemetryPacket) == 24, "telemetry packet must stay 24 bytes");

enum LogFlags : uint8_t {
  kLogHomeValid = 1 << 0,
  kLogWindValid = 1 << 1,
  kLogGpsFresh = 1 << 2,
  kLogArmed = 1 << 3,
  kLogMainFired = 1 << 4,
  kLogLaunchCandidate = 1 << 5,
  kLogSteering = 1 << 6,
};

inline uint8_t xorChecksum(const void* data, size_t len) {
  const uint8_t* p = static_cast<const uint8_t*>(data);
  uint8_t c = 0;
  for (size_t i = 0; i < len; ++i) c ^= p[i];
  return c;
}

template <typename T>
void sealRecord(T& r) {
  r.checksum = xorChecksum(&r, sizeof(T) - 1);
}

template <typename T>
bool recordValid(const T& r) {
  return r.checksum == xorChecksum(&r, sizeof(T) - 1);
}

}  // namespace gr
