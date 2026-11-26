#include "gr/hil.hpp"

#include <cmath>
#include <cstring>

namespace gr {
namespace hil {

namespace {
int16_t sat16(float v) {
  if (!std::isfinite(v)) return 0;
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return static_cast<int16_t>(std::lround(v));
}
}  // namespace

uint16_t crc16(const uint8_t* data, size_t len, uint16_t crc) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int k = 0; k < 8; ++k) crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

size_t encode(uint8_t type, const void* payload, uint8_t len, uint8_t* out, size_t cap) {
  if (len > kMaxPayload || cap < static_cast<size_t>(len) + kOverhead) return 0;
  out[0] = kSync;
  out[1] = type;
  out[2] = len;
  if (len) std::memcpy(out + 3, payload, len);
  const uint16_t c = crc16(out + 1, static_cast<size_t>(len) + 2);
  out[3 + len] = static_cast<uint8_t>(c & 0xFF);
  out[4 + len] = static_cast<uint8_t>(c >> 8);
  return static_cast<size_t>(len) + kOverhead;
}

bool Decoder::feed(uint8_t b) {
  switch (st_) {
    case kWaitSync:
      if (b == kSync) st_ = kType;
      return false;
    case kType:
      type_ = b;
      st_ = kLen;
      return false;
    case kLen:
      len_ = b;
      idx_ = 0;
      if (len_ > kMaxPayload) { ++bad_; st_ = (b == kSync) ? kType : kWaitSync; return false; }
      st_ = len_ ? kPayload : kCrc1;
      return false;
    case kPayload:
      buf_[idx_++] = b;
      if (idx_ >= len_) st_ = kCrc1;
      return false;
    case kCrc1:
      crcLo_ = b;
      st_ = kCrc2;
      return false;
    case kCrc2: {
      st_ = kWaitSync;
      uint8_t hdr[2] = {type_, len_};
      uint16_t c = crc16(hdr, 2);
      c = crc16(buf_, len_, c);
      if ((c & 0xFF) == crcLo_ && (c >> 8) == b) { ++ok_; return true; }
      ++bad_;
      return false;
    }
  }
  return false;
}

SensorFrame toSensorFrame(const Inputs& in) {
  SensorFrame f;
  f.tUs = static_cast<uint32_t>(in.tUs & 0xFFFFFFFFu);
  const float mg = 1000.0f / kGravity;
  f.accel[0] = sat16(in.imu.accel.x * mg);
  f.accel[1] = sat16(in.imu.accel.y * mg);
  f.accel[2] = sat16(in.imu.accel.z * mg);
  f.gyro[0] = sat16(in.imu.gyro.x * 1000.0f);
  f.gyro[1] = sat16(in.imu.gyro.y * 1000.0f);
  f.gyro[2] = sat16(in.imu.gyro.z * 1000.0f);
  f.pressurePa = in.pressurePa;
  f.flags = static_cast<uint8_t>((in.baroFresh ? kSensorBaroFresh : 0) | (in.armed ? kSensorArmed : 0) |
                                 (in.mainFiredInput ? kSensorMainFired : 0));
  return f;
}

GpsFrame toGpsFrame(const GpsFix& g) {
  GpsFrame f;
  f.fixType = g.fixType;
  f.numSv = g.numSv;
  f.latE7 = g.latE7;
  f.lonE7 = g.lonE7;
  f.altMm = static_cast<int32_t>(std::lround(g.altMsl * 1000.0f));
  f.velCm[0] = sat16(g.velN * 100.0f);
  f.velCm[1] = sat16(g.velE * 100.0f);
  f.velCm[2] = sat16(g.velD * 100.0f);
  const float h = g.hAcc * 100.0f;
  f.hAccCm = static_cast<uint16_t>(h < 0.0f ? 0.0f : (h > 65535.0f ? 65535.0f : h));
  return f;
}

void applySensorFrame(const SensorFrame& f, Inputs& in) {
  const float s = kGravity / 1000.0f;
  in.imu.accel = Vec3{f.accel[0] * s, f.accel[1] * s, f.accel[2] * s};
  in.imu.gyro = Vec3{f.gyro[0] * 0.001f, f.gyro[1] * 0.001f, f.gyro[2] * 0.001f};
  in.imu.valid = true;
  in.baroFresh = (f.flags & kSensorBaroFresh) != 0;
  in.pressurePa = f.pressurePa;
  in.armed = (f.flags & kSensorArmed) != 0;
  in.mainFiredInput = (f.flags & kSensorMainFired) != 0;
}

void applyGpsFrame(const GpsFrame& f, GpsFix& g) {
  g = GpsFix{};
  g.fresh = true;
  g.fixType = f.fixType;
  g.numSv = f.numSv;
  g.latE7 = f.latE7;
  g.lonE7 = f.lonE7;
  g.altMsl = f.altMm * 0.001f;
  g.velN = f.velCm[0] * 0.01f;
  g.velE = f.velCm[1] * 0.01f;
  g.velD = f.velCm[2] * 0.01f;
  g.hAcc = f.hAccCm * 0.01f;
  g.vAcc = g.hAcc * 1.8f;
  g.sAcc = 0.15f;
}

void HilBoard::onBytes(const uint8_t* data, size_t len, BoardIo& io) {
  for (size_t i = 0; i < len; ++i) {
    if (!dec_.feed(data[i])) continue;
    if (dec_.type() == kTypeGps && dec_.length() == sizeof(GpsFrame)) {
      std::memcpy(&pendingGps_, dec_.payload(), sizeof(GpsFrame));
      havePendingGps_ = true;
    } else if (dec_.type() == kTypeSensor && dec_.length() == sizeof(SensorFrame)) {
      SensorFrame f;
      std::memcpy(&f, dec_.payload(), sizeof(SensorFrame));
      step(f, io);
    }
  }
}

void HilBoard::step(const SensorFrame& f, BoardIo& io) {
  const uint32_t t0 = io.micros();
  // Unwrap the 32-bit simulation clock into the core's 64-bit microseconds.
  if (haveTime_ && f.tUs < lastTUs_ && lastTUs_ - f.tUs > 0x80000000u) timeHigh_ += 0x100000000ULL;
  lastTUs_ = f.tUs;
  haveTime_ = true;

  Inputs in;
  in.tUs = timeHigh_ + f.tUs;
  applySensorFrame(f, in);
  if (havePendingGps_) {
    applyGpsFrame(pendingGps_, in.gps);
    havePendingGps_ = false;
  }
  io.readEncoders(in.encoder);
  in.batteryV = io.batteryVolts();
  const Outputs& out = core_.update(in);
  // Bench interlock: the real winches only move with the board's own magnet removed.
  const bool interlockArmed = io.interlockArmed();
  io.writeMotors(out.motorDuty[0], out.motorDuty[1], out.motorsEnabled && interlockArmed);

  const Status& st = core_.status();
  ActuatorFrame a;
  a.tUs = f.tUs;
  a.duty[0] = sat16(out.motorDuty[0] * 10000.0f);
  a.duty[1] = sat16(out.motorDuty[1] * 10000.0f);
  a.flags = static_cast<uint8_t>((out.motorsEnabled ? kActMotorsEnabled : 0) | (st.steeringActive ? kActSteering : 0) |
                                 (interlockArmed ? 0 : kActInterlock));
  a.phase = static_cast<uint8_t>(st.phase);
  a.stage = static_cast<uint8_t>(st.stage);
  a.mode = static_cast<uint8_t>(st.mode);
  a.faults = st.faults;
  a.encoder[0] = in.encoder[0];
  a.encoder[1] = in.encoder[1];
  const uint32_t dt = io.micros() - t0;
  ++steps_;
  if (dt > 5000) ++overruns_;
  if (dt > maxLoopUs_) maxLoopUs_ = dt;
  a.loopUs = static_cast<uint16_t>(dt > 65535 ? 65535 : dt);
  a.overruns = static_cast<uint16_t>(overruns_ > 65535 ? 65535 : overruns_);
  uint8_t buf[kOverhead + sizeof(ActuatorFrame)];
  const size_t n = encode(kTypeActuator, &a, sizeof(a), buf, sizeof(buf));
  io.send(buf, n);
}

}  // namespace hil
}  // namespace gr
