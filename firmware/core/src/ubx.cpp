#include "gr/ubx.hpp"

#include <cstring>

namespace gr {
namespace ubx {

namespace {
void put32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
}
int32_t get32(const uint8_t* p) {
  return static_cast<int32_t>(static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                              (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24));
}
uint32_t getU32(const uint8_t* p) { return static_cast<uint32_t>(get32(p)); }
}  // namespace

size_t keyValueSize(uint32_t key) {
  switch ((key >> 28) & 0x7) {
    case 1: return 1;  // L (one bit, stored as a byte)
    case 2: return 1;
    case 3: return 2;
    case 4: return 4;
    case 5: return 8;
    default: return 0;
  }
}

size_t buildFrame(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len, uint8_t* out, size_t cap) {
  const size_t total = static_cast<size_t>(len) + 8;
  if (total > cap) return 0;
  out[0] = kSync1;
  out[1] = kSync2;
  out[2] = cls;
  out[3] = id;
  out[4] = static_cast<uint8_t>(len & 0xFF);
  out[5] = static_cast<uint8_t>(len >> 8);
  if (len) std::memcpy(out + 6, payload, len);
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < 6 + static_cast<size_t>(len); ++i) {
    a = static_cast<uint8_t>(a + out[i]);
    b = static_cast<uint8_t>(b + a);
  }
  out[6 + len] = a;
  out[7 + len] = b;
  return total;
}

size_t buildValset(const KeyValue* kv, size_t n, uint8_t layers, uint8_t* out, size_t cap) {
  uint8_t payload[256];
  size_t p = 0;
  payload[p++] = 0x00;  // version
  payload[p++] = layers;
  payload[p++] = 0x00;
  payload[p++] = 0x00;
  for (size_t i = 0; i < n; ++i) {
    const size_t sz = keyValueSize(kv[i].key);
    if (sz == 0 || sz > 4 || p + 4 + sz > sizeof(payload)) return 0;
    put32(payload + p, kv[i].key);
    p += 4;
    for (size_t k = 0; k < sz; ++k) payload[p++] = static_cast<uint8_t>(kv[i].value >> (8 * k));
  }
  return buildFrame(kClassCfg, kIdCfgValset, payload, static_cast<uint16_t>(p), out, cap);
}

size_t buildPodConfig(uint32_t baud, uint8_t* out, size_t cap) {
  const KeyValue kv[] = {
      {kCfgUart1Baudrate, baud},
      {kCfgUart1InUbx, 1},
      {kCfgUart1OutUbx, 1},
      {kCfgUart1OutNmea, 0},
      {kCfgMsgoutNavPvtUart1, 1},
      {kCfgRateMeas, 100},
      {kCfgRateNav, 1},
      {kCfgNavspgDynmodel, 8},  // airborne < 4 g
      {kCfgSignalGpsEna, 1},
      {kCfgSignalGalEna, 1},
      {kCfgSignalBdsEna, 0},
      {kCfgSignalGloEna, 0},
  };
  return buildValset(kv, sizeof(kv) / sizeof(kv[0]), kLayerRam, out, cap);
}

bool Parser::feed(uint8_t b) {
  switch (st_) {
    case kWait1:
      if (b == kSync1) st_ = kWait2;
      return false;
    case kWait2:
      st_ = (b == kSync2) ? kClass : (b == kSync1 ? kWait2 : kWait1);
      return false;
    case kClass:
      cls_ = b;
      ckA_ = b;
      ckB_ = b;
      st_ = kId;
      return false;
    case kId:
      id_ = b;
      ckA_ = static_cast<uint8_t>(ckA_ + b);
      ckB_ = static_cast<uint8_t>(ckB_ + ckA_);
      st_ = kLen1;
      return false;
    case kLen1:
      len_ = b;
      ckA_ = static_cast<uint8_t>(ckA_ + b);
      ckB_ = static_cast<uint8_t>(ckB_ + ckA_);
      st_ = kLen2;
      return false;
    case kLen2:
      len_ = static_cast<uint16_t>(len_ | (static_cast<uint16_t>(b) << 8));
      ckA_ = static_cast<uint8_t>(ckA_ + b);
      ckB_ = static_cast<uint8_t>(ckB_ + ckA_);
      idx_ = 0;
      overflow_ = len_ > sizeof(buf_);
      st_ = len_ ? kPayload : kCkA;
      return false;
    case kPayload:
      if (idx_ < sizeof(buf_)) buf_[idx_] = b;
      ++idx_;
      ckA_ = static_cast<uint8_t>(ckA_ + b);
      ckB_ = static_cast<uint8_t>(ckB_ + ckA_);
      if (idx_ >= len_) st_ = kCkA;
      return false;
    case kCkA:
      st_ = (b == ckA_) ? kCkB : kWait1;
      if (b != ckA_) ++bad_;
      return false;
    case kCkB:
      st_ = kWait1;
      if (b != ckB_) { ++bad_; return false; }
      ++ok_;
      if (overflow_) return false;
      handleFrame();
      return cls_ == kClassNav && id_ == kIdNavPvt && len_ == 92;
  }
  return false;
}

void Parser::handleFrame() {
  if (cls_ == kClassAck && len_ == 2) {
    lastAck_ = (id_ == 0x01);
    lastAckId_ = buf_[1];
    ++acks_;
    return;
  }
  if (cls_ != kClassNav || id_ != kIdNavPvt || len_ != 92) return;
  const uint8_t* p = buf_;
  iTow_ = getU32(p + 0);
  const uint8_t fixType = p[20];
  const uint8_t flags = p[21];
  const bool fixOk = (flags & 0x01) != 0;
  GpsFix f;
  f.fresh = true;
  f.fixType = fixOk ? fixType : 0;
  f.numSv = p[23];
  f.lonE7 = get32(p + 24);
  f.latE7 = get32(p + 28);
  f.altMsl = static_cast<float>(get32(p + 36)) * 1e-3f;
  f.hAcc = static_cast<float>(getU32(p + 40)) * 1e-3f;
  f.vAcc = static_cast<float>(getU32(p + 44)) * 1e-3f;
  f.velN = static_cast<float>(get32(p + 48)) * 1e-3f;
  f.velE = static_cast<float>(get32(p + 52)) * 1e-3f;
  f.velD = static_cast<float>(get32(p + 56)) * 1e-3f;
  f.sAcc = static_cast<float>(getU32(p + 68)) * 1e-3f;
  fix_ = f;
}

}  // namespace ubx
}  // namespace gr
