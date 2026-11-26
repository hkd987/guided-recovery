// Hardware-in-the-loop (gate 4): the real pod board flies simulated flights.
//
// The PC simulator streams sensor frames over USB serial at 200 Hz; the board
// runs FlightCore on them, drives its REAL winches, and answers each frame
// with an actuator frame carrying the real encoder positions, which steer the
// simulated canopy. The board side lives here (HilBoard) so the ESP32 target
// and the desktop stand-in board run identical code.
//
// Wire format, both directions:
//   0xA5 | type | len | payload[len] | crc16 (CCITT, little-endian, over type..payload)
#pragma once

#include <cstddef>
#include <cstdint>

#include "gr/flight_core.hpp"
#include "gr/types.hpp"

namespace gr {
namespace hil {

constexpr uint8_t kSync = 0xA5;
constexpr uint8_t kTypeSensor = 'S';
constexpr uint8_t kTypeGps = 'G';
constexpr uint8_t kTypeActuator = 'A';
constexpr size_t kOverhead = 5;  // sync, type, len, crc16
constexpr size_t kMaxPayload = 64;

#pragma pack(push, 1)
struct SensorFrame {
  uint32_t tUs = 0;           // simulation time, wraps every ~71 min
  int16_t accel[3] = {};      // mg, body frame specific force
  int16_t gyro[3] = {};       // mrad/s, body frame
  float pressurePa = 0.0f;
  uint8_t flags = 0;          // kSensor* bits
  uint8_t reserved = 0;
};
struct GpsFrame {
  uint8_t fixType = 0;
  uint8_t numSv = 0;
  int32_t latE7 = 0;
  int32_t lonE7 = 0;
  int32_t altMm = 0;
  int16_t velCm[3] = {};      // N, E, D in cm/s
  uint16_t hAccCm = 0;
};
struct ActuatorFrame {
  uint32_t tUs = 0;           // echo of the sensor frame answered
  int16_t duty[2] = {};       // 1e-4 units
  uint8_t flags = 0;          // kAct* bits
  uint8_t phase = 0;
  uint8_t stage = 0;
  uint8_t mode = 0;
  uint16_t faults = 0;
  int32_t encoder[2] = {};    // real winch encoder counts
  uint16_t loopUs = 0;        // board time spent on this frame
  uint16_t overruns = 0;      // frames that took longer than 5 ms
};
#pragma pack(pop)
static_assert(sizeof(SensorFrame) == 22, "wire layout");
static_assert(sizeof(GpsFrame) == 22, "wire layout");
static_assert(sizeof(ActuatorFrame) == 26, "wire layout");

enum : uint8_t {
  kSensorBaroFresh = 1 << 0,
  kSensorArmed = 1 << 1,
  kSensorMainFired = 1 << 2,
};
enum : uint8_t {
  kActMotorsEnabled = 1 << 0,
  kActSteering = 1 << 1,
  kActInterlock = 1 << 2,  // the board's physical magnet is ON: motors held off
};

uint16_t crc16(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);
size_t encode(uint8_t type, const void* payload, uint8_t len, uint8_t* out, size_t cap);

// Streaming decoder with resynchronisation on any bad byte.
class Decoder {
 public:
  // Returns true when a complete, CRC-valid frame has just been decoded.
  bool feed(uint8_t b);
  uint8_t type() const { return type_; }
  uint8_t length() const { return len_; }
  const uint8_t* payload() const { return buf_; }
  uint32_t framesOk() const { return ok_; }
  uint32_t framesBad() const { return bad_; }

 private:
  enum State { kWaitSync, kType, kLen, kPayload, kCrc1, kCrc2 };
  State st_ = kWaitSync;
  uint8_t type_ = 0, len_ = 0, idx_ = 0;
  uint8_t crcLo_ = 0;
  uint8_t buf_[kMaxPayload] = {};
  uint32_t ok_ = 0, bad_ = 0;
};

// Conversions between the flight core's inputs and the wire frames.
SensorFrame toSensorFrame(const Inputs& in);
GpsFrame toGpsFrame(const GpsFix& g);
void applySensorFrame(const SensorFrame& f, Inputs& in);
void applyGpsFrame(const GpsFrame& f, GpsFix& g);

// What a board must provide.
class BoardIo {
 public:
  virtual ~BoardIo() = default;
  virtual void readEncoders(int32_t enc[2]) = 0;
  virtual float batteryVolts() = 0;
  virtual bool interlockArmed() = 0;  // physical magnet removed
  virtual void writeMotors(float dutyL, float dutyR, bool enabled) = 0;
  virtual uint32_t micros() = 0;
  virtual void send(const uint8_t* data, size_t len) = 0;
};

// The board side of the loop: feed it received bytes; each sensor frame runs
// one FlightCore::update and sends one actuator frame back.
class HilBoard {
 public:
  explicit HilBoard(FlightCore& core) : core_(core) {}
  void onBytes(const uint8_t* data, size_t len, BoardIo& io);
  uint32_t steps() const { return steps_; }
  uint32_t overruns() const { return overruns_; }
  uint32_t maxLoopUs() const { return maxLoopUs_; }
  const Decoder& decoder() const { return dec_; }

 private:
  void step(const SensorFrame& f, BoardIo& io);

  FlightCore& core_;
  Decoder dec_;
  GpsFrame pendingGps_;
  bool havePendingGps_ = false;
  uint64_t timeHigh_ = 0;
  uint32_t lastTUs_ = 0;
  bool haveTime_ = false;
  uint32_t steps_ = 0, overruns_ = 0, maxLoopUs_ = 0;
};

}  // namespace hil
}  // namespace gr
